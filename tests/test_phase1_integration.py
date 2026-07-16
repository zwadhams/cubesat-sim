import json

import pytest

from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary")


def orbit_period(sim):
    return sim.components[0].orbit.period_s


def test_healthy_satellite_breathes_and_stays_nominal():
    sim = build_sim(dt=5.0, seed=1, illumination=0.8)
    sim.run(duration=3 * orbit_period(sim))

    soc = [v for *_, v in sim.recorder.telemetry("physics", "soc_true")]
    assert min(soc) > 0.5           # never gets into trouble
    assert max(soc) - min(soc) > 0.02  # but visibly breathes with eclipse

    eclipse = [v for *_, v in sim.recorder.telemetry("physics", "eclipse")]
    frac = sum(eclipse) / len(eclipse)
    assert 0.2 < frac < 0.42        # eclipse present, below the in-plane max

    kinds = [e[3] for e in sim.recorder.events("physics")]
    assert "eclipse_enter" in kinds and "eclipse_exit" in kinds
    assert "brownout" not in kinds
    assert sim.recorder.events("obc") == []  # no mode changes


def test_degraded_array_adcs_shed_is_a_one_way_door():
    """This scenario's fate has evolved with each phase (see
    EMERGENT_BEHAVIORS.md entries 1, 2, 6). With attitude in the loop, the
    EPS hard-shed kills the ADCS; the freewheeling satellite is captured by
    gravity-gradient torque with its panel averaging anti-sun, generation
    pins at the side-panel floor below even essential loads, SoC can never
    recover past the restore threshold — so the shed latches permanently
    and the satellite coasts slowly toward brownout."""
    sim = build_sim(dt=5.0, seed=2, illumination=0.45)
    sim.run(duration=12 * orbit_period(sim))

    modes = [json.loads(e[4])["to"] for e in sim.recorder.events("obc")]
    assert modes.count("SAFE") >= 1  # protection engaged

    payload_cmds = sim.recorder.messages(topic="cmd/loads/payload")
    states = [json.loads(row[5])["on"] for row in payload_cmds]
    assert True in states and False in states

    eps_events = sim.recorder.events("eps")
    eps_kinds = [e[3] for e in eps_events]
    assert "load_shed" in eps_kinds
    shed_time = min(e[1] for e in eps_events if e[3] == "load_shed")
    assert "load_restore" not in eps_kinds  # the trap never re-opens

    # after the shed, pointing is lost: panel averages away from the sun
    facing = [(r[1], r[4]) for r in sim.recorder.telemetry("physics", "sun_facing")]
    post = [v for t, v in facing if t > shed_time + orbit_period(sim)]
    assert sum(post) / len(post) < 0.2

    soc = [v for *_, v in sim.recorder.telemetry("physics", "soc_true")]
    assert min(soc) > 0.05  # dying slowly, not dead within this horizon
    assert soc[-1] < 0.25   # and clearly not recovering
    kinds = [e[3] for e in sim.recorder.events("physics")]
    assert "brownout" not in kinds


def test_same_seed_same_flight():
    def fingerprint(seed):
        sim = build_sim(dt=5.0, seed=seed, illumination=0.45)
        sim.run(duration=4 * orbit_period(sim))
        return sim.recorder.messages()

    assert fingerprint(7) == fingerprint(7)
    assert fingerprint(7) != fingerprint(8)
