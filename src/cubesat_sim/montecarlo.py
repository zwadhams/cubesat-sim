"""Monte Carlo campaign harness: fly many seeds, mine the wreckage.

Each flight gets a seed-derived random fault campaign (plus ambient SEUs)
and a full flight recording on disk, so any interesting outcome can be
replayed exactly with `run_flight(seed)` or by opening its .db. `sweep()`
runs flights in parallel processes — each simulation already spawns its
Rust/C++ flight software as subprocesses, so keep max_workers modest.

Outcome classification is deliberately coarse; the recordings hold the
detail. Priority order: CRASHED (sim integrity failure — always a bug to
fix, per project policy) > DEAD > BROWNED_OUT > FDIR_GIVEUP > SHED >
SAFE > NOMINAL.
"""

from __future__ import annotations

import json
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.faults import ScheduledFault
from cubesat_sim.mission import build_sim

OUTCOMES = ("CRASHED", "DEAD", "BROWNED_OUT", "FDIR_GIVEUP",
            "SHED", "SAFE", "NOMINAL")


@dataclass
class FlightSummary:
    seed: int
    outcome: str
    min_soc: float
    end_soc: float
    brownouts: int
    safe_entries: int
    sheds: int
    fdir_cycles: int
    fdir_gave_up: bool
    faults: int
    seus: int
    archived_mb: float
    dropped_mb: float
    end_rate_dps: float
    capacity_wh: float
    illumination: float
    wall_s: float
    notes: str


def random_fault_campaign(
    seed: int, duration_s: float, max_faults: int = 3,
) -> list[ScheduledFault]:
    """Seed-deterministic fault script: 0..max_faults draws from a menu of
    plausible hardware misfortunes, at random times inside the flight."""
    rng = np.random.default_rng(np.random.SeedSequence([seed, 0xFA07]))
    out: list[ScheduledFault] = []
    for _ in range(int(rng.integers(0, max_faults + 1))):
        at = float(rng.uniform(0.05, 0.85) * duration_s)
        roll = float(rng.random())
        if roll < 0.35:
            out.append(ScheduledFault(at, "fault/sensor_stuck", {
                "sensor": "gyro", "stuck": True,
                "hard": bool(rng.random() < 0.25)}))
        elif roll < 0.55:
            out.append(ScheduledFault(at, "fault/sensor_stuck", {
                "sensor": "mag", "stuck": True, "hard": False}))
        elif roll < 0.75:
            out.append(ScheduledFault(at, "fault/wheel_friction", {
                "nm_per_nms": float(rng.uniform(1e-5, 1e-4))}))
        else:
            out.append(ScheduledFault(at, "fault/array_hit", {
                "mult": float(rng.uniform(0.55, 0.9))}))
    return sorted(out, key=lambda f: f.at_s)


def _last(rows, default=0.0):
    return rows[-1][-1] if rows else default


def run_flight(
    seed: int,
    *,
    orbits: float = 8.0,
    dt: float = 5.0,
    out_dir: str | Path = "runs/mc",
    seu_rate_per_day: float = 4.0,
    campaign: bool = True,
    **build_kw,
) -> FlightSummary:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"flight_{seed:04d}.db"
    path.unlink(missing_ok=True)

    period = CircularOrbit().period_s
    duration = orbits * period
    faults = random_fault_campaign(seed, duration) if campaign else []

    t0 = time.perf_counter()
    sim = build_sim(dt=dt, seed=seed, recorder_path=path, faults=faults,
                    seu_rate_per_day=seu_rate_per_day, **build_kw)
    crashed = ""
    try:
        sim.run(duration=duration)
    except Exception as exc:  # sim-integrity failure: one bad flight must
        crashed = f"{type(exc).__name__}: {exc}"  # never kill the campaign
    sim.recorder.flush()
    rec = sim.recorder

    soc = [v for *_, v in rec.telemetry("physics", "soc_true")]
    phys_events = rec.events("physics")
    obc_events = rec.events("obc")
    brownouts = sum(1 for e in phys_events if e[3] == "brownout")
    safe_entries = sum(
        1 for e in obc_events
        if e[3] == "mode_change" and json.loads(e[4]).get("to") == "SAFE")
    sheds = sum(1 for e in rec.events("eps") if e[3] == "load_shed")
    gave_up = any(e[3] == "fdir_giveup" for e in obc_events)
    fault_events = rec.events("faults")
    n_seus = sum(1 for e in fault_events if e[3] == "inject_seu")
    n_faults = sum(1 for e in fault_events if e[3] == "inject")

    end_soc = soc[-1] if soc else 0.0
    shedding_end = _last(rec.telemetry("eps", "shedding")) == 1.0
    safe_end = _last(rec.telemetry("obc", "safe_mode")) == 1.0

    if crashed:
        outcome = "CRASHED"
    elif brownouts and end_soc <= 0.05:
        outcome = "DEAD"
    elif brownouts:
        outcome = "BROWNED_OUT"
    elif gave_up:
        outcome = "FDIR_GIVEUP"
    elif shedding_end:
        outcome = "SHED"
    elif safe_end:
        outcome = "SAFE"
    else:
        outcome = "NOMINAL"

    def describe(detail: dict) -> str:
        kind = detail.get("topic", "?").split("/")[-1]
        if detail.get("sensor"):
            kind += f":{detail['sensor']}"
        if detail.get("hard"):
            kind += "(hard)"
        return kind

    notes = "; ".join(
        f"{describe(json.loads(e[4]))}@{e[1] / period:.1f}orb"
        for e in fault_events if e[3] == "inject")
    if crashed:
        notes = (notes + "; " if notes else "") + f"CRASH: {crashed[:80]}"

    summary = FlightSummary(
        seed=seed,
        outcome=outcome,
        min_soc=round(min(soc), 3) if soc else 0.0,
        end_soc=round(end_soc, 3),
        brownouts=brownouts,
        safe_entries=safe_entries,
        sheds=sheds,
        fdir_cycles=int(_last(rec.telemetry("obc", "fdir_cycles"))),
        fdir_gave_up=gave_up,
        faults=n_faults,
        seus=n_seus,
        archived_mb=round(_last(rec.telemetry("ground", "archive_mb")), 1),
        dropped_mb=round(_last(rec.telemetry("comms", "dropped_mb")), 1),
        end_rate_dps=round(_last(rec.telemetry("physics", "rate_dps")), 2),
        capacity_wh=round(sim.components[0].battery.capacity_wh, 3),
        illumination=round(sim.components[0].array.illumination, 4),
        wall_s=round(time.perf_counter() - t0, 1),
        notes=notes,
    )
    sim.close()
    return summary


def sweep(
    seeds,
    *,
    max_workers: int | None = 4,
    **flight_kw,
) -> list[FlightSummary]:
    seeds = list(seeds)
    if max_workers == 1 or len(seeds) == 1:
        return [run_flight(s, **flight_kw) for s in seeds]
    results = []
    with ProcessPoolExecutor(max_workers=max_workers) as pool:
        futures = [pool.submit(run_flight, s, **flight_kw) for s in seeds]
        for fut in as_completed(futures):
            results.append(fut.result())
    return sorted(results, key=lambda r: r.seed)


def format_table(summaries: list[FlightSummary]) -> str:
    head = (f"{'seed':>4}  {'outcome':<12} {'minSoC':>6} {'endSoC':>6} "
            f"{'safe':>4} {'shed':>4} {'fdir':>4} {'seus':>4} "
            f"{'arch MB':>7} {'drop MB':>7} {'rate':>6}  notes")
    rows = [head, "-" * len(head)]
    for s in summaries:
        rows.append(
            f"{s.seed:>4}  {s.outcome:<12} {s.min_soc:>6.2f} {s.end_soc:>6.2f} "
            f"{s.safe_entries:>4} {s.sheds:>4} {s.fdir_cycles:>4} {s.seus:>4} "
            f"{s.archived_mb:>7.1f} {s.dropped_mb:>7.1f} {s.end_rate_dps:>6.2f}"
            f"  {s.notes}")
    return "\n".join(rows)
