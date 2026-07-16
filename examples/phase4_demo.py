"""Phase 4 demo: the data economy, end to end.

  nominal — image Tokyo/Sao Paulo/Reykjavik when overhead, queue onboard,
      downlink over Bozeman, watch storage breathe with the pass schedule.
  data_flood — instrument misconfigured to 6x data rate: storage slams
      full, megabytes die on the cutting-room floor, and on the next pass
      the ground operator rule sees it and uplinks payload/enable off.
      A control loop that runs through space, closing only minutes per
      orbit, hours apart.
"""

import json
from pathlib import Path

from cubesat_sim.mission import build_sim

BLOCKS = " ▁▂▃▄▅▆▇█"


def sparkline(values, width=78, lo=None, hi=None):
    if not values:
        return ""
    lo = min(values) if lo is None else lo
    hi = max(values) if hi is None else hi
    span = (hi - lo) or 1.0
    per = max(1, len(values) // width)
    buckets = [values[i:i + per] for i in range(0, len(values), per)][:width]
    return "".join(
        BLOCKS[max(0, min(8, round((sum(b) / len(b) - lo) / span * 8)))]
        for b in buckets
    )


def fly(label, orbits, seed, **cfg):
    sim = build_sim(dt=1.0, seed=seed, recorder_path=f"runs/phase4_{label}.db", **cfg)
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    rec = sim.recorder

    queue = [v for *_, v in rec.telemetry("comms", "queue_mb")]
    contact = [v for *_, v in rec.telemetry("physics", "gs_contact")]
    imaging = [v for *_, v in rec.telemetry("payload", "imaging")]
    generated = [v for *_, v in rec.telemetry("payload", "generated_mb")]
    archive = [v for *_, v in rec.telemetry("ground", "archive_mb")]
    dropped = [v for *_, v in rec.telemetry("comms", "dropped_mb")]
    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]

    print(f"\n=== {label}: {orbits} orbits ({orbits * period / 3600:.1f} h) ===")
    print(f"imaged {generated[-1]:.0f} MB | downlinked to archive "
          f"{archive[-1]:.0f} MB | lost to full storage {dropped[-1]:.0f} MB")
    print("imaging passes")
    print("  " + sparkline(imaging, lo=0.0, hi=1.0))
    print("onboard storage (256 MB)")
    print("  " + sparkline(queue, lo=0.0, hi=256.0))
    print("ground contact")
    print("  " + sparkline(contact, lo=0.0, hi=1.0))
    print(f"battery SoC  min {min(soc):.2f}")
    print("  " + sparkline(soc, lo=0.0, hi=1.0))

    shown = 0
    for source in ("comms", "ground", "payload"):
        for e in rec.events(source):
            detail = json.loads(e[4])
            extras = ", ".join(
                f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}"
                for k, v in detail.items())
            print(f"  t={e[1] / period:5.2f} orbits  [{source}] {e[3]}"
                  + (f" ({extras})" if extras else ""))
            shown += 1
            if shown >= 14:
                break
        if shown >= 14:
            break
    sim.close()


def main():
    Path("runs").mkdir(exist_ok=True)
    fly("nominal", orbits=10, seed=42)
    fly("data_flood", orbits=10, seed=42, payload_rate_mb_s=2.0)
    print("\nflight recordings in runs/phase4_*.db")


if __name__ == "__main__":
    main()
