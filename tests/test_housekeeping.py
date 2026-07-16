"""Subsystem health through the real link: EPS and OBC housekeeping
packets ride the beacon, and the ground's picture of the spacecraft is
exactly as stale as the pass schedule allows."""

import json

import pytest

from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def test_ground_power_rule_closes_through_housekeeping():
    """Degraded flight: the EPS SoC estimate rides the beacon down, the
    ground's power veto trips during a pass, and a protective disable
    goes up — a control loop that exists only because power health now
    crosses the link."""
    sim = build_sim(dt=5.0, seed=11, illumination=0.45)
    sim.run(duration=6 * sim.components[0].orbit.period_s)
    rec = sim.recorder

    disables = [e for e in rec.events("ground")
                if e[3] == "operator_disable_payload"]
    assert disables
    assert json.loads(disables[0][4])["reason"] == "power"
    assert any(e[3] == "uplink_dispatch" for e in rec.events("comms"))

    # the ground's belief only changes when a beacon lands: heavily
    # quantized compared to the onboard estimate it mirrors
    sat = [v for *_, v in rec.telemetry("ground", "sat_soc_est")]
    onboard = [v for *_, v in rec.telemetry("eps", "soc_est")]
    assert sat and len(set(sat)) < len(set(onboard)) / 10
    # and it tracks reality: every heard value is one the EPS produced
    # (to beacon quantization, 1e-4)
    onboard_set = set(round(v, 4) for v in onboard)
    assert all(any(abs(round(v, 4) - o) <= 1e-4 for o in onboard_set)
               for v in set(sat))

    # OBC mode crossed the link too (this flight enters SAFE)
    modes = [v for *_, v in rec.telemetry("ground", "sat_safe_mode")]
    assert 1.0 in modes
    sim.close()
