"""CCSDS-flavored space data link: frames, packets, CRC.

This is the ground-segment (and test-bench) implementation of the link
protocol; the flight side lives independently in backend/comms-cpp/main.cpp, the
way a real program has two implementations that must interoperate. The
formats are simplified but structurally honest CCSDS:

TM transfer frame (downlink, fixed 96 bytes):
    0   4   attached sync marker 1ACFFC1D
    4   2   version(2)=0 | spacecraft id(10) | virtual channel(3) | ocf(1)=0
    6   1   master channel frame count (mod 256)
    7   1   virtual channel frame count (mod 256)
    8   2   data field status (0)
    10  84  data field: space packets, then 0x55 idle fill
    94  2   FECF: CRC-16/CCITT-FALSE over bytes 4..93

Space packet:
    0   2   version(3)=0 | type(1) | sec hdr flag(1)=0 | APID(11)
    2   2   sequence flags(2)=3 | packet sequence count(14)
    4   2   data length - 1
    6   n   payload

TC transfer frame (uplink, variable <= 48 bytes):
    0   2   version(2)=0 | spare(4) | spacecraft id(10)
    2   1   frame sequence number N(S), mod 256
    3   1   total frame length in bytes
    4   n   command payload: u8 cmd id, u8 arg
    -2  2   FECF: CRC-16/CCITT-FALSE over everything before it

Bulk science data (virtual channel 1) crosses the sim's RF channel as
frame-*accounted* bursts rather than byte streams — counters and loss
statistics are real, payload bytes are not materialized. Housekeeping
(VC0) and commands are byte-true end to end.
"""

from __future__ import annotations

import struct

ASM = bytes.fromhex("1ACFFC1D")
SCID = 0x0C5
TM_FRAME_LEN = 96
TM_DATA_LEN = TM_FRAME_LEN - len(ASM) - 6 - 2  # 84
IDLE_FILL = 0x55

APID_BEACON = 0x020   # comms-local health (storage, counters, TC ack)
APID_EPS_HK = 0x040   # power subsystem health words
APID_OBC_HK = 0x060   # mode and load-request state
VC_HOUSEKEEPING = 0
VC_SCIENCE = 1
VC1_FRAME_BITS = (1024 + 32) * 8  # science frame: 1 KiB payload + overhead

CMD_PAYLOAD_ENABLE = 0x01
COMMANDS = {CMD_PAYLOAD_ENABLE: "payload/enable"}


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — the CCSDS FECF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_space_packet(apid: int, seq: int, payload: bytes,
                       type_tc: bool = False) -> bytes:
    head = struct.pack(
        ">HHH",
        ((1 if type_tc else 0) << 12) | (apid & 0x7FF),
        0xC000 | (seq & 0x3FFF),
        len(payload) - 1,
    )
    return head + payload


def build_tm_frame(vcid: int, mc_count: int, vc_count: int,
                   packets: list[bytes]) -> bytes:
    data = b"".join(packets)
    if len(data) > TM_DATA_LEN:
        raise ValueError(f"data field overflow: {len(data)} > {TM_DATA_LEN}")
    data = data + bytes([IDLE_FILL]) * (TM_DATA_LEN - len(data))
    head = struct.pack(">HBBH", (SCID & 0x3FF) << 4 | (vcid & 0x7) << 1,
                       mc_count & 0xFF, vc_count & 0xFF, 0)
    body = head + data
    return ASM + body + struct.pack(">H", crc16(body))


def parse_tm_frame(frame: bytes) -> dict:
    out: dict = {"asm_ok": frame[:4] == ASM, "crc_ok": False, "packets": []}
    if len(frame) != TM_FRAME_LEN:
        out["error"] = f"bad length {len(frame)}"
        return out
    body = frame[4:-2]
    (fecf,) = struct.unpack(">H", frame[-2:])
    out["crc_ok"] = out["asm_ok"] and crc16(body) == fecf
    ids, mc, vc, _status = struct.unpack(">HBBH", body[:6])
    out["scid"] = ids >> 4
    out["vcid"] = (ids >> 1) & 0x7
    out["mc_count"] = mc
    out["vc_count"] = vc
    data = body[6:]
    pos = 0
    while pos + 6 <= len(data):
        apid_word, seq_word, length = struct.unpack(">HHH", data[pos:pos + 6])
        if apid_word >> 13 != 0:  # nonzero version bits: idle fill from here
            break
        n = length + 1
        if pos + 6 + n > len(data):
            break
        out["packets"].append({
            "apid": apid_word & 0x7FF,
            "type_tc": bool(apid_word & 0x1000),
            "seq": seq_word & 0x3FFF,
            "data": data[pos + 6:pos + 6 + n],
        })
        pos += 6 + n
    return out


# -- housekeeping beacon payload (16 bytes) ----------------------------------

def pack_beacon(storage_frac: float, dropped_mb: float, sent_mb: float,
                queue_mb: float, tc_ack: int, flags: int) -> bytes:
    return struct.pack(
        ">HIIHBBH",
        min(10000, max(0, round(storage_frac * 10000.0))),
        min(0xFFFFFFFF, round(dropped_mb * 1024.0)),
        min(0xFFFFFFFF, round(sent_mb * 1024.0)),
        min(0xFFFF, round(queue_mb * 10.0)),
        tc_ack & 0xFF, flags & 0xFF, 0)


def unpack_beacon(data: bytes) -> dict:
    frac_e4, dropped_kb, sent_kb, queue_e1, tc_ack, flags, _ = struct.unpack(
        ">HIIHBBH", data)
    return {
        "storage_frac": frac_e4 / 10000.0,
        "dropped_mb": dropped_kb / 1024.0,
        "sent_mb": sent_kb / 1024.0,
        "queue_mb": queue_e1 / 10.0,
        "tc_ack": tc_ack,
        "flags": flags,
    }


def _sat16(v: float) -> int:
    """Saturating unsigned 16-bit conversion — telemetry words clip, they
    never wrap (a bit-flipped 1e300 volt reading packs as 65535). Clamp
    before rounding: round(inf) raises."""
    if not v > 0.0:  # also catches NaN
        return 0
    if v >= 65535.0:
        return 65535
    return round(v)


# -- EPS housekeeping payload (12 bytes) --------------------------------------

def pack_eps_hk(soc_est: float, battery_v: float, solar_w: float,
                load_w: float, shedding: bool) -> bytes:
    return struct.pack(">HHHHBBH",
                       _sat16(soc_est * 10000.0), _sat16(battery_v * 1000.0),
                       _sat16(solar_w * 1000.0), _sat16(load_w * 1000.0),
                       1 if shedding else 0, 0, 0)


def unpack_eps_hk(data: bytes) -> dict:
    soc_e4, batt_mv, solar_mw, load_mw, flags, _, _ = struct.unpack(
        ">HHHHBBH", data)
    return {
        "soc_est": soc_e4 / 10000.0,
        "battery_v": batt_mv / 1000.0,
        "solar_w": solar_mw / 1000.0,
        "load_w": load_mw / 1000.0,
        "shedding": bool(flags & 1),
    }


# -- OBC housekeeping payload (6 bytes) ---------------------------------------

def pack_obc_hk(safe: bool, adcs_on: bool, payload_on: bool) -> bytes:
    return struct.pack(">BBI", 1 if safe else 0,
                       (1 if adcs_on else 0) | (2 if payload_on else 0), 0)


def unpack_obc_hk(data: bytes) -> dict:
    mode, flags, _ = struct.unpack(">BBI", data)
    return {
        "safe": mode == 1,
        "adcs_on": bool(flags & 1),
        "payload_on": bool(flags & 2),
    }


# -- telecommand frames -------------------------------------------------------

def build_tc_frame(seq: int, cmd_id: int, arg: int) -> bytes:
    body = struct.pack(">HBB", SCID & 0x3FF, seq & 0xFF, 8) + \
        bytes([cmd_id & 0xFF, arg & 0xFF])
    return body + struct.pack(">H", crc16(body))


def parse_tc_frame(frame: bytes) -> dict:
    out: dict = {"crc_ok": False}
    if len(frame) < 8:
        out["error"] = "short frame"
        return out
    body, (fecf,) = frame[:-2], struct.unpack(">H", frame[-2:])
    out["crc_ok"] = crc16(body) == fecf
    scid, seq, _length = struct.unpack(">HBB", body[:4])
    out["scid"] = scid & 0x3FF
    out["seq"] = seq
    out["cmd_id"] = body[4]
    out["arg"] = body[5]
    return out


def seq_acked(ack: int, seq: int) -> bool:
    """True when mod-256 counter `ack` has advanced past `seq` — i.e. the
    receiver's next-expected count is beyond our frame number."""
    return (ack - seq - 1) % 256 < 128
