"""Phase 6b demo: the space link as a real protocol.

Flies the flood scenario with a mid-flight scintillation event (channel
BER x40 from orbit ~3.5), then plays protocol analyst over the recording:
decoded TM transfer frames, CRC rejections, sequence gaps, the science
virtual channel dying in the noise — and the short little TC frames
still punching through to disable the payload.

Full decode afterwards:  python -m cubesat_sim.linkdump runs/phase6_link.db
"""

from pathlib import Path

from cubesat_sim.faults import channel_ber
from cubesat_sim.linkdump import dump
from cubesat_sim.mission import build_sim


def main():
    Path("runs").mkdir(exist_ok=True)
    sim = build_sim(dt=1.0, seed=42, recorder_path="runs/phase6_link.db",
                    payload_rate_mb_s=2.0,
                    faults=[channel_ber(20000.0, mult=40.0)])
    period = sim.components[0].orbit.period_s
    print("flying 8 orbits: data flood + scintillation storm from orbit "
          f"{20000.0 / period:.1f} ...")
    sim.run(duration=8 * period)
    rec = sim.recorder

    print(f"\nground archive: {rec.telemetry('ground', 'archive_mb')[-1][-1]:.1f} MB | "
          f"frames ok {rec.telemetry('ground', 'frames_ok')[-1][-1]:.0f} | "
          f"rejected {rec.telemetry('ground', 'frames_rejected')[-1][-1]:.0f} | "
          f"seq gaps {rec.telemetry('ground', 'seq_gaps')[-1][-1]:.0f} | "
          f"TC retransmits {rec.telemetry('ground', 'tc_retransmits')[-1][-1]:.0f}")
    for source in ("ground", "comms"):
        for e in rec.events(source):
            if e[3] in ("operator_disable_payload", "uplink_dispatch",
                        "uplink_acked", "vc0_gap"):
                print(f"  t={e[1] / period:5.2f} orbits  [{source}] {e[3]}")
    sim.close()

    print("\n--- linkdump (last 18 link messages) ---")
    lines = dump("runs/phase6_link.db").splitlines()
    print("\n".join(lines[-20:]))


if __name__ == "__main__":
    main()
