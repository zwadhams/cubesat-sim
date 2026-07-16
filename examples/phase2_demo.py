"""Phase 2 demo: thermal joins the fight for watts.

  healthy — the battery heater duty-cycles through eclipse; everything holds
  cold+degraded — a weak array in a cold season. Watch the causal chain the
      simulator produces on its own: energy deficit -> EPS sheds loads,
      including the battery heater -> battery drops below 0 C -> charging is
      physically inhibited -> the sun comes back but the watts can't get in
      -> brownout. A death spiral: each protection makes the next problem
      worse. Nobody scripted it.
"""

import json
from pathlib import Path

from cubesat_sim.mission import build_sim
from cubesat_sim.physics.thermal import CELSIUS_ZERO_K

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
    sim = build_sim(dt=1.0, seed=seed, recorder_path=f"runs/phase2_{label}.db", **cfg)
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    rec = sim.recorder

    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]
    t_batt = [v - CELSIUS_ZERO_K for *_, v in rec.telemetry("physics", "t_batt_k")]

    print(f"\n=== {label}: {orbits} orbits ({orbits * period / 3600:.1f} h) ===")
    print(f"battery SoC (truth)   min {min(soc):.2f}  max {max(soc):.2f}")
    print("  " + sparkline(soc, lo=0.0, hi=1.0))
    print(f"battery temp (truth)  min {min(t_batt):+.1f} C  max {max(t_batt):+.1f} C")
    print("  " + sparkline(t_batt))

    timeline = []
    for source in ("physics", "eps", "obc"):
        for e in rec.events(source):
            timeline.append((e[1], source, e[3], e[4]))
    timeline.sort()
    interesting = [x for x in timeline
                   if x[2] not in ("eclipse_enter", "eclipse_exit")]
    for time_s, source, kind, detail_json in interesting[:14]:
        detail = json.loads(detail_json)
        extras = ", ".join(f"{k}={v:.2f}" if isinstance(v, float) else f"{k}={v}"
                           for k, v in detail.items())
        print(f"  t={time_s / period:5.2f} orbits  [{source}] {kind}"
              + (f" ({extras})" if extras else ""))
    if len(interesting) > 14:
        print(f"  ... {len(interesting) - 14} more events")
    if not interesting:
        print("  no anomalies — heater cycled, satellite held NOMINAL")
    sim.close()


def main():
    Path("runs").mkdir(exist_ok=True)
    fly("healthy", orbits=4, seed=42)
    fly("cold_degraded", orbits=20, seed=42, illumination=0.45, thermal_sun_w=26.0)
    print("\nflight recordings in runs/phase2_*.db")


if __name__ == "__main__":
    main()
