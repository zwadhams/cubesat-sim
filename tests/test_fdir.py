"""FDIR: the OBC's gyro watchdog and its ADCS power-cycle response.

Most tests fly without the compiled subsystems (the watchdog only needs
physics + EPS + OBC); C/Python OBC equivalence with FDIR active is covered
in test_remote_obc.py.
"""

from cubesat_sim.faults import sensor_stuck
from cubesat_sim.mission import build_sim
from cubesat_sim.subsystems.obc import MAX_CYCLES


def light_sim(**cfg):
    return build_sim(adcs_impl="none", comms_impl="none", **cfg)


def obc_kinds(sim):
    return [e[3] for e in sim.recorder.events("obc")]


def test_fdir_recovers_stuck_gyro_with_one_cycle():
    sim = light_sim(dt=1.0, seed=5, faults=[sensor_stuck(300.0, "gyro")])
    sim.run(duration=900)
    kinds = obc_kinds(sim)
    assert kinds.count("fdir_adcs_power_cycle") == 1
    assert kinds.count("fdir_adcs_repower") == 1
    assert "fdir_giveup" not in kinds
    assert any(e[3] == "latchup_cleared"
               for e in sim.recorder.events("physics"))
    # detection was prompt: stuck at ~301, cycle within a dozen samples
    cycle_t = next(e[1] for e in sim.recorder.events("obc")
                   if e[3] == "fdir_adcs_power_cycle")
    assert 300.0 < cycle_t < 320.0
    # and it never retriggered: the budget shows exactly one cycle
    assert sim.recorder.telemetry("obc", "fdir_cycles")[-1][-1] == 1.0
    sim.close()


def test_fdir_burns_budget_then_gives_up_on_hard_fault():
    sim = light_sim(dt=1.0, seed=5,
                    faults=[sensor_stuck(300.0, "gyro", hard=True)])
    sim.run(duration=900)
    kinds = obc_kinds(sim)
    assert kinds.count("fdir_adcs_power_cycle") == MAX_CYCLES
    assert kinds.count("fdir_giveup") == 1
    assert not any(e[3] == "latchup_cleared"
                   for e in sim.recorder.events("physics"))
    # do no harm: after giving up the ADCS is left powered
    last_loads = [m for m in sim.recorder.messages(topic="cmd/loads/adcs")][-1]
    assert '"on": true' in last_loads[5]
    sim.close()


def test_single_seu_transient_logs_anomaly_without_cycling():
    """One insane sample is radiation weather; three in a row is a broken
    sensor. The watchdog must tell them apart."""
    sim = light_sim(dt=1.0, seed=6)
    sim.run(duration=120)
    sim.bus.publish("sensors/adcs/gyro", "test", {"x": 9.9, "y": 0.0, "z": 0.0})
    sim.run(duration=60)
    kinds = obc_kinds(sim)
    assert kinds.count("gyro_anomaly") == 1
    assert "fdir_adcs_power_cycle" not in kinds

    for _ in range(3):  # now a sustained storm of garbage
        sim.bus.publish("sensors/adcs/gyro", "test",
                        {"x": 9.9, "y": 0.0, "z": 0.0})
        sim.run(ticks=1)
    sim.run(duration=5)
    assert "fdir_adcs_power_cycle" in obc_kinds(sim)
    sim.close()


def test_healthy_flight_never_trips_fdir():
    sim = light_sim(dt=1.0, seed=8)
    sim.run(duration=2 * sim.components[0].orbit.period_s)
    assert sim.recorder.telemetry("obc", "fdir_cycles")[-1][-1] == 0.0
    assert "gyro_anomaly" not in obc_kinds(sim)
    sim.close()
