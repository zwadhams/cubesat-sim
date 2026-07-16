import json

from cubesat_sim.mission import build_sim


def orbit_period(sim):
    return sim.components[0].orbit.period_s


def test_healthy_flight_heater_cycles_and_stays_safe():
    sim = build_sim(dt=5.0, seed=3)
    sim.run(duration=3 * orbit_period(sim))

    t_batt = [v for *_, v in sim.recorder.telemetry("physics", "t_batt_k")]
    assert min(t_batt) > 268.0  # heater keeps the battery out of the danger zone
    assert max(t_batt) < 300.0

    heater_cmds = sim.recorder.messages(topic="cmd/loads/bat_heater")
    states = [json.loads(row[5])["on"] for row in heater_cmds]
    assert True in states  # thermostat actually asked for heat at some point

    kinds = [e[3] for e in sim.recorder.events("physics")]
    assert "brownout" not in kinds
    assert sim.recorder.events("obc") == []  # still NOMINAL throughout


def test_cold_degraded_flight_death_spirals():
    """The signature Phase 2 emergent failure: energy deficit -> shedding
    kills the heater -> battery freezes -> charging inhibited -> brownout.
    Nobody scripted this sequence; it falls out of the coupled physics."""
    sim = build_sim(dt=5.0, seed=4, illumination=0.45, thermal_sun_w=26.0)
    sim.run(duration=20 * orbit_period(sim))

    phys_events = sim.recorder.events("physics")
    kinds = [e[3] for e in phys_events]
    assert "charge_inhibit_on" in kinds
    assert "brownout" in kinds

    eps_kinds = [e[3] for e in sim.recorder.events("eps")]
    assert "load_shed" in eps_kinds

    # causality: the EPS shed (killing the heater) precedes the brownout
    shed_tick = min(e[0] for e in sim.recorder.events("eps") if e[3] == "load_shed")
    brownout_tick = min(e[0] for e in phys_events if e[3] == "brownout")
    assert shed_tick < brownout_tick

    soc = [v for *_, v in sim.recorder.telemetry("physics", "soc_true")]
    assert soc[-1] < 0.1  # the satellite does not recover
