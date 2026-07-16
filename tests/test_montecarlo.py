"""Monte Carlo harness: flights run, classify, replay deterministically."""

from dataclasses import asdict

import pytest

from cubesat_sim.montecarlo import (
    OUTCOMES,
    random_fault_campaign,
    run_flight,
    sweep,
)

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def test_campaign_is_seed_deterministic():
    assert random_fault_campaign(5, 40000.0) == random_fault_campaign(5, 40000.0)
    campaigns = [random_fault_campaign(s, 40000.0) for s in range(20)]
    assert any(campaigns)  # the menu does get used
    for faults in campaigns:
        assert all(0.0 < f.at_s < 40000.0 for f in faults)
        assert faults == sorted(faults, key=lambda f: f.at_s)


def test_flight_summary_is_reproducible(tmp_path):
    a = run_flight(9, orbits=2, dt=5.0, out_dir=tmp_path / "a",
                   seu_rate_per_day=50.0)
    b = run_flight(9, orbits=2, dt=5.0, out_dir=tmp_path / "b",
                   seu_rate_per_day=50.0)
    da, db = asdict(a), asdict(b)
    da.pop("wall_s"), db.pop("wall_s")
    assert da == db


def test_sweep_runs_and_classifies(tmp_path):
    summaries = sweep([1, 2, 3], orbits=2, dt=5.0, out_dir=tmp_path,
                      max_workers=2)
    assert [s.seed for s in summaries] == [1, 2, 3]
    for s in summaries:
        assert s.outcome in OUTCOMES
        assert s.outcome != "CRASHED", s.notes  # integrity failures are bugs
        assert 0.0 <= s.min_soc <= 1.0
        assert s.min_soc <= s.end_soc + 1e-9
        assert (tmp_path / f"flight_{s.seed:04d}.db").exists()
