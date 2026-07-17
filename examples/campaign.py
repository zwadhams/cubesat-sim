"""Science campaign runner: fly a seed sweep, keep everything, mine later.

Campaign 1 defaults: 24 seeds x 12 orbits (~18 h of satellite life each),
single arm, random fault campaigns + SEU weather, full recordings — every
flight opens directly in the dashboard:

    python examples/campaign.py
    python -m cubesat_sim.dashboard runs/campaign1/flight_0007.db
"""

import argparse
from pathlib import Path

from cubesat_sim.montecarlo import format_table, sweep


def main():
    ap = argparse.ArgumentParser(description="Fly a Monte Carlo campaign.")
    ap.add_argument("--seeds", type=int, default=24)
    ap.add_argument("--orbits", type=float, default=12.0)
    ap.add_argument("--out", default="runs/campaign1")
    ap.add_argument("--seu-rate", type=float, default=6.0,
                    help="ambient SEU rate per day (x25 in the SAA)")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    summaries = sweep(range(args.seeds), orbits=args.orbits, dt=5.0,
                      out_dir=args.out, seu_rate_per_day=args.seu_rate,
                      max_workers=args.workers)
    table = format_table(summaries)
    print(table)
    Path(args.out, "summary.txt").write_text(table + "\n")
    flagged = [s.seed for s in summaries if s.outcome != "NOMINAL"]
    print(f"\noff-nominal flights: {flagged if flagged else 'none'}")
    print(f"recordings + summary.txt in {args.out}/")


if __name__ == "__main__":
    main()
