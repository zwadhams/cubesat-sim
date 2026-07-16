"""Phase 3 demo: the ADCS (Rust flight software) takes over attitude.

  healthy — launch tumble at 4 deg/s: B-dot detumble on magnetorquers,
      sun acquisition on reaction wheels, and the power system feels all
      of it: generation is now attitude-coupled.
  cold+degraded — the Phase 2 death spiral, now with attitude in the loop:
      when the EPS sheds the ADCS load, the satellite slowly loses pointing
      and generation sags toward the tumbling average, deepening the hole.
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
    sim = build_sim(dt=1.0, seed=seed, recorder_path=f"runs/phase3_{label}.db", **cfg)
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    rec = sim.recorder

    rate = [v for *_, v in rec.telemetry("physics", "rate_dps")]
    facing = [v for *_, v in rec.telemetry("physics", "sun_facing")]
    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]
    wheels = [v for *_, v in rec.telemetry("physics", "wheel_h_frac")]

    print(f"\n=== {label}: {orbits} orbits ({orbits * period / 3600:.1f} h) ===")
    print(f"body rate       start {rate[0]:.1f}  end {rate[-1]:.2f} deg/s")
    print("  " + sparkline(rate, lo=0.0))
    print(f"sun facing (cos) min {min(facing):+.2f}  max {max(facing):+.2f}")
    print("  " + sparkline(facing, lo=-1.0, hi=1.0))
    print(f"wheel momentum   peak {max(wheels):.0%} of capacity")
    print("  " + sparkline(wheels, lo=0.0, hi=1.0))
    print(f"battery SoC      min {min(soc):.2f}  max {max(soc):.2f}")
    print("  " + sparkline(soc, lo=0.0, hi=1.0))

    timeline = []
    for source in ("adcs", "eps", "obc", "physics"):
        for e in rec.events(source):
            if e[3] in ("eclipse_enter", "eclipse_exit"):
                continue
            timeline.append((e[1], source, e[3], json.loads(e[4])))
    timeline.sort()
    for time_s, source, kind, detail in timeline[:12]:
        extras = ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}"
                           for k, v in detail.items())
        print(f"  t={time_s / period:5.2f} orbits  [{source}] {kind}"
              + (f" ({extras})" if extras else ""))
    if len(timeline) > 12:
        print(f"  ... {len(timeline) - 12} more events")
    sim.close()


def main():
    Path("runs").mkdir(exist_ok=True)
    fly("healthy", orbits=4, seed=42)
    fly("cold_degraded", orbits=12, seed=42, illumination=0.45, thermal_sun_w=26.0)
    print("\nflight recordings in runs/phase3_*.db")


if __name__ == "__main__":
    main()
