"""Phase 5 demo: FDIR, fault injection, and the Monte Carlo campaign.

  fault_drill — a gyro latches up mid-flight. The OBC's watchdog spots the
      frozen output word, power-cycles the ADCS, and the latch clears:
      detection, isolation, recovery, all onboard, no ground in the loop.
  hard_failure — the same fault, but permanent. FDIR burns its three-cycle
      budget, gives up, and the satellite flies on with a frozen gyro —
      watch what the controller does with a rate measurement that never
      changes.
  monte_carlo — sixteen satellites, sixteen seeds, each with its own
      random misfortunes (stuck sensors, worn bearings, debris strikes,
      SEU weather peaking over the South Atlantic Anomaly). The table is
      the mission postmortem; every flight recording is kept for replay.
"""

import json
from pathlib import Path

from cubesat_sim.faults import sensor_stuck
from cubesat_sim.mission import build_sim
from cubesat_sim.montecarlo import format_table, sweep

BLOCKS = " ▁▂▃▄▅▆▇█"

EVENT_SOURCES = ("obc", "physics", "faults", "eps")
EVENT_KINDS = ("inject", "gyro_anomaly", "fdir_adcs_power_cycle",
               "fdir_adcs_repower", "fdir_giveup", "latchup_cleared",
               "load_shed", "load_restore", "mode_change", "brownout")


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


def fly(label, faults, orbits=4, seed=42):
    sim = build_sim(dt=1.0, seed=seed, recorder_path=f"runs/phase5_{label}.db",
                    faults=faults)
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    rec = sim.recorder

    rate = [v for *_, v in rec.telemetry("physics", "rate_dps")]
    facing = [v for *_, v in rec.telemetry("physics", "sun_facing")]
    wheel = [v for *_, v in rec.telemetry("physics", "wheel_h_frac")]
    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]

    print(f"\n=== {label}: {orbits} orbits ({orbits * period / 3600:.1f} h) ===")
    print("body rate (deg/s)")
    print("  " + sparkline(rate, lo=0.0))
    print("sun facing (-1..1)")
    print("  " + sparkline(facing, lo=-1.0, hi=1.0))
    print("wheel momentum (frac of max)")
    print("  " + sparkline(wheel, lo=0.0, hi=1.0))
    print(f"battery SoC  min {min(soc):.2f}")
    print("  " + sparkline(soc, lo=0.0, hi=1.0))

    timeline = []
    for source in EVENT_SOURCES:
        for e in rec.events(source):
            if e[3] in EVENT_KINDS:
                detail = json.loads(e[4])
                extras = ", ".join(
                    f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                    for k, v in detail.items())
                timeline.append((e[1], f"  t={e[1] / period:5.2f} orbits  "
                                 f"[{source}] {e[3]}"
                                 + (f" ({extras})" if extras else "")))
    for _, line in sorted(timeline)[:16]:
        print(line)
    sim.close()


def main():
    Path("runs").mkdir(exist_ok=True)

    fly("fault_drill", [sensor_stuck(2500.0, "gyro")])
    fly("hard_failure", [sensor_stuck(2500.0, "gyro", hard=True)])

    print("\n=== monte_carlo: 16 seeds x 8 orbits, faults + SEU weather ===")
    summaries = sweep(range(16), orbits=8, dt=5.0, out_dir="runs/mc",
                      seu_rate_per_day=6.0, max_workers=4)
    print(format_table(summaries))
    worst = [s for s in summaries if s.outcome != "NOMINAL"]
    print(f"\n{len(worst)}/{len(summaries)} flights ended off-nominal; "
          "replay any of them from runs/mc/flight_<seed>.db")


if __name__ == "__main__":
    main()
