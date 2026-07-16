import pytest

from cubesat_sim import RemoteComponent, Simulation
from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary")


def flight(obc_impl, **cfg):
    sim = build_sim(dt=5.0, seed=11, obc_impl=obc_impl, **cfg)
    period = sim.components[0].orbit.period_s
    sim.run(duration=6 * period)
    data = (
        sim.recorder.messages(),
        sim.recorder.telemetry("obc"),
        sim.recorder.events("obc"),
    )
    sim.close()
    return data


def test_c_obc_bit_identical_to_python_reference(c_obc_binary):
    """The whole point of the bridge: swapping flight software language
    must not change the flight. Same seed, same scenario -> identical
    message log, telemetry, and events, across a process boundary and a
    C/Python divide. Uses the degraded scenario so mode changes and load
    requests actually exercise the logic."""
    assert flight("python", illumination=0.45) == flight("c", illumination=0.45)


def test_c_obc_flies_healthy_scenario(c_obc_binary):
    sim = build_sim(dt=5.0, seed=12, obc_impl="c")
    sim.run(duration=2 * sim.components[0].orbit.period_s)
    assert sim.recorder.events("obc") == []  # stayed NOMINAL
    modes = [v for *_, v in sim.recorder.telemetry("obc", "safe_mode")]
    assert modes and all(v == 0.0 for v in modes)
    sim.close()


def test_dead_flight_software_is_a_loud_failure():
    sim = Simulation(dt=1.0)
    with pytest.raises(RuntimeError, match="exited"):
        sim.add(RemoteComponent("dead", period=1.0, argv=["/bin/false"]))
