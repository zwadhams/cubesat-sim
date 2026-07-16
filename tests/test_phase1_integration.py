import json

from cubesat_sim.mission import build_sim


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


def test_degraded_array_limit_cycles_between_modes():
    sim = build_sim(dt=5.0, seed=2, illumination=0.45)
    sim.run(duration=12 * orbit_period(sim))

    modes = [json.loads(e[4])["to"] for e in sim.recorder.events("obc")]
    assert modes.count("SAFE") >= 2      # entered SAFE more than once...
    assert modes.count("NOMINAL") >= 1   # ...and recovered in between: limit cycle

    payload_cmds = sim.recorder.messages(topic="cmd/loads/payload")
    states = [json.loads(row[5])["on"] for row in payload_cmds]
    assert True in states and False in states

    soc = [v for *_, v in sim.recorder.telemetry("physics", "soc_true")]
    assert min(soc) > 0.05  # struggles but survives
    kinds = [e[3] for e in sim.recorder.events("physics")]
    assert "brownout" not in kinds


def test_same_seed_same_flight():
    def fingerprint(seed):
        sim = build_sim(dt=5.0, seed=seed, illumination=0.45)
        sim.run(duration=4 * orbit_period(sim))
        return sim.recorder.messages()

    assert fingerprint(7) == fingerprint(7)
    assert fingerprint(7) != fingerprint(8)
