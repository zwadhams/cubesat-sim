"""linkdump: decode the space link traffic inside a flight recording.

A protocol analyzer for the recorded bitstream — every TM frame the
ground received, every VC1 burst, every TC frame sent up and what the
spacecraft actually got after the channel chewed on it. Wireshark for a
downlink that lives in SQLite.

Usage:
    python -m cubesat_sim.linkdump runs/phase6_link.db
    python -m cubesat_sim.linkdump runs/foo.db --limit 40 --hex
"""

from __future__ import annotations

import argparse
import json
import sqlite3

from cubesat_sim import ccsds
from cubesat_sim.environment.orbit import CircularOrbit

TOPICS = ("radio/rx_ground", "ground/tx", "radio/rx_space")

CMD_NAMES = {v: k for k, v in ccsds.COMMANDS.items()}


def _fmt_tm(data: dict, show_hex: bool) -> str:
    frame = bytes.fromhex(data["hex"])
    p = ccsds.parse_tm_frame(frame)
    if not p["crc_ok"]:
        out = f"DOWN VC?  ** {'CRC FAIL' if p['asm_ok'] else 'SYNC LOST'} **"
    else:
        out = (f"DOWN VC{p['vcid']} mc={p['mc_count']:03d} "
               f"vc={p['vc_count']:03d} crc=ok ")
        for pkt in p["packets"]:
            if pkt["apid"] == ccsds.APID_BEACON:
                hk = ccsds.unpack_beacon(pkt["data"])
                out += (f" beacon#{pkt['seq']}: storage={hk['storage_frac']:.1%}"
                        f" queue={hk['queue_mb']:.1f}MB"
                        f" sent={hk['sent_mb']:.1f}MB"
                        f" dropped={hk['dropped_mb']:.1f}MB"
                        f" ack={hk['tc_ack']} flags={hk['flags']:#04b}")
            elif pkt["apid"] == ccsds.APID_EPS_HK:
                eps = ccsds.unpack_eps_hk(pkt["data"])
                out += (f" | eps#{pkt['seq']}: soc={eps['soc_est']:.1%}"
                        f" v={eps['battery_v']:.2f}"
                        f" gen={eps['solar_w']:.2f}W"
                        f" load={eps['load_w']:.2f}W"
                        f"{' SHED' if eps['shedding'] else ''}")
            elif pkt["apid"] == ccsds.APID_OBC_HK:
                obc = ccsds.unpack_obc_hk(pkt["data"])
                out += (f" | obc#{pkt['seq']}:"
                        f" {'SAFE' if obc['safe'] else 'NOMINAL'}"
                        f" adcs={int(obc['adcs_on'])}"
                        f" payload={int(obc['payload_on'])}")
            else:
                out += f" apid={pkt['apid']:#05x} len={len(pkt['data'])}"
    if show_hex:
        out += "\n           " + data["hex"]
    return out


def _fmt_vc1(data: dict) -> str:
    n, bad = int(data.get("frames", 0)), int(data.get("bad", 0))
    return (f"DOWN VC1 burst frames={n} bad={bad} "
            f"mb={float(data.get('mb', 0.0)):.3f} "
            f"vcfc0={int(data.get('vcfc0', 0))}")


def _fmt_tc(data: dict, direction: str, show_hex: bool) -> str:
    frame = bytes.fromhex(data["hex"])
    p = ccsds.parse_tc_frame(frame)
    if not p["crc_ok"]:
        out = f"{direction}   TC ** CRC FAIL **"
    else:
        name = ccsds.COMMANDS.get(p["cmd_id"], f"cmd_{p['cmd_id']:#04x}")
        out = (f"{direction}   TC seq={p['seq']} {name} arg={p['arg']} crc=ok")
    if show_hex:
        out += "\n           " + data["hex"]
    return out


def dump(db_path: str, limit: int | None = None, show_hex: bool = False) -> str:
    period = CircularOrbit().period_s
    db = sqlite3.connect(db_path)
    rows = db.execute(
        f"SELECT time, topic, data FROM messages WHERE topic IN "
        f"({','.join('?' * len(TOPICS))}) ORDER BY seq"
        + (f" LIMIT {int(limit)}" if limit else ""),
        TOPICS).fetchall()
    db.close()

    lines = []
    stats = {"tm_ok": 0, "tm_bad": 0, "vc1_frames": 0, "vc1_bad": 0,
             "tc_sent": 0, "tc_received": 0}
    for t, topic, blob in rows:
        data = json.loads(blob)
        kind = data.get("kind")
        if topic == "radio/rx_ground" and kind == "tm_frame":
            text = _fmt_tm(data, show_hex)
            stats["tm_bad" if "FAIL" in text or "LOST" in text
                  else "tm_ok"] += 1
        elif topic == "radio/rx_ground" and kind == "vc1_burst":
            text = _fmt_vc1(data)
            stats["vc1_frames"] += int(data.get("frames", 0))
            stats["vc1_bad"] += int(data.get("bad", 0))
        elif topic == "ground/tx" and "hex" in data:
            text = _fmt_tc(data, "UP  ", show_hex)
            stats["tc_sent"] += 1
        elif topic == "radio/rx_space" and "hex" in data:
            text = _fmt_tc(data, "UP* ", show_hex)  # * = after the channel
            stats["tc_received"] += 1
        else:
            text = f"{topic} {blob[:60]}"
        lines.append(f"orbit {t / period:6.2f}  t={t:8.0f}  {text}")

    lines.append("")
    lines.append(
        f"summary: TM frames ok={stats['tm_ok']} bad={stats['tm_bad']} | "
        f"VC1 science frames={stats['vc1_frames']} "
        f"corrupted={stats['vc1_bad']} | "
        f"TC sent={stats['tc_sent']} arrived={stats['tc_received']}")
    return "\n".join(lines)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Decode the space-link traffic in a flight recording.")
    ap.add_argument("recording", help="flight recorder .db")
    ap.add_argument("--limit", type=int, help="max link messages to decode")
    ap.add_argument("--hex", action="store_true", help="show raw frame hex")
    args = ap.parse_args()
    print(dump(args.recording, args.limit, args.hex))


if __name__ == "__main__":
    main()
