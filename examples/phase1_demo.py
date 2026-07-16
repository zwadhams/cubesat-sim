"""Phase 1 demo: the satellite breathing through eclipse cycles.

Two flights of the same spacecraft:

  healthy   — full solar array; SoC breathes with the eclipse cycle
  degraded  — array at 45% (think: bad deployment or wrong attitude);
              the OBC/EPS rules produce an emergent payload duty cycle
              via NOMINAL <-> SAFE limit cycling. Nobody programmed a
              duty cycle — it falls out of two hysteresis bands and an
              energy deficit.

Historical note: this was written before Phase 2 added thermal. With the
battery heater in the energy budget the degraded flight now parks in SAFE
and is held at ~0.25 SoC by EPS shed/restore flapping instead of limit
cycling — see phase2_demo.py for the current story.
"""

import json

from cubesat_sim.mission import build_sim

BLOCKS = " ▁▂▃▄▅▆▇█"


def sparkline(values, width=78, lo=0.0, hi=1.0):
    if not values:
        return ""
    per = max(1, len(values) // width)
    buckets = [values[i:i + per] for i in range(0, len(values), per)][:width]
    out = []
    for b in buckets:
        x = (sum(b) / len(b) - lo) / (hi - lo)
        out.append(BLOCKS[max(0, min(8, round(x * 8)))])
    return "".join(out)


def fly(label, illumination, orbits, seed):
    sim = build_sim(dt=1.0, seed=seed, illumination=illumination,
                    recorder_path=f"runs/phase1_{label}.db")
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    rec = sim.recorder

    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]
    eclipse = [v for *_, v in rec.telemetry("physics", "eclipse")]
    safe = [v for *_, v in rec.telemetry("obc", "safe_mode")]
    mode_events = rec.events("obc")
    shed_events = [e for e in rec.events("eps")
                   if e[3] in ("load_shed", "load_restore")]

    print(f"\n=== {label}: illumination {illumination:.0%}, "
          f"{orbits} orbits ({orbits * period / 3600:.1f} h simulated) ===")
    print(f"orbit period {period / 60:.1f} min, "
          f"eclipse fraction {sum(eclipse) / len(eclipse):.2f}")
    print(f"battery SoC (truth)  min {min(soc):.2f}  max {max(soc):.2f}")
    print("  " + sparkline(soc))
    print("safe mode (1=SAFE)")
    print("  " + sparkline(safe))
    for e in mode_events:
        detail = json.loads(e[4])
        print(f"  t={e[1] / period:5.2f} orbits  OBC -> {detail['to']} "
              f"(soc_est {detail['soc_est']:.2f})")
    if not mode_events:
        print("  no mode changes — stayed NOMINAL")
    if shed_events:
        print(f"  EPS hard sheds: {len(shed_events)}")
    sim.close()


def main():
    from pathlib import Path
    Path("runs").mkdir(exist_ok=True)
    fly("healthy", illumination=0.80, orbits=4, seed=42)
    fly("degraded", illumination=0.45, orbits=12, seed=42)
    print("\nflight recordings in runs/phase1_*.db")


if __name__ == "__main__":
    main()
