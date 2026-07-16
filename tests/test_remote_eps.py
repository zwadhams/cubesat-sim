"""The C EPS must fly bit-identically to the Python reference.

Same bar as the OBC port: identical message log, telemetry, and events
across the process boundary. The degraded scenario exercises the
shed/restore hysteresis and the full-charge SoC clamp (the "1.0 must not
print as 1" case); the FDIR scenario routes ADCS power-cycle requests
through the EPS's veto logic.
"""

import pytest

from cubesat_sim.faults import sensor_stuck
from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def flight(eps_impl, orbits=6, **cfg):
    sim = build_sim(dt=5.0, seed=11, eps_impl=eps_impl, **cfg)
    period = sim.components[0].orbit.period_s
    sim.run(duration=orbits * period)
    data = (
        sim.recorder.messages(),
        sim.recorder.telemetry("eps"),
        sim.recorder.events("eps"),
    )
    sim.close()
    return data


def test_c_eps_bit_identical_to_python_reference(c_eps_binary):
    assert flight("python", illumination=0.45) == flight("c", illumination=0.45)


def test_c_eps_bit_identical_through_full_charge(c_eps_binary):
    """A healthy flight tops the battery out: measured volts exceed 8.4
    under charge rise and the SoC estimate clamps to exactly 1.0 — the
    integral-float formatting trap both C ports must sidestep."""
    py = flight("python", orbits=2, initial_soc=0.98)
    assert any(key == "soc_est" and v == 1.0
               for *_, key, v in py[1])  # the clamp was actually reached
    assert py == flight("c", orbits=2, initial_soc=0.98)


def test_c_eps_bit_identical_under_fdir_power_cycling(c_eps_binary):
    faults = [sensor_stuck(600.0, "gyro")]
    assert flight("python", orbits=3, faults=faults) == \
           flight("c", orbits=3, faults=faults)