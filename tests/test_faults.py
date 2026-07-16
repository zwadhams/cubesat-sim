"""Fault injection engine: stuck sensors, SEUs, and what clears them.

These run physics + injector only (no flight software), so fault mechanics
are tested in isolation — cmd/loads traffic is played by hand.
"""

import math

import numpy as np

from cubesat_sim import Simulation
from cubesat_sim.faults import (
    FaultInjector,
    ScheduledFault,
    flip_bit,
    sensor_stuck,
    seu,
    seu_upset,
)
from cubesat_sim.physics.spacecraft import SpacecraftPhysics


def physics_sim(*faults, seed=7, **injector_kw):
    sim = Simulation(dt=1.0, seed=seed)
    sim.add(SpacecraftPhysics())
    sim.add(FaultInjector(schedule=list(faults), **injector_kw))
    return sim


def gyro_payloads(sim):
    return [row[5] for row in sim.recorder.messages(topic="sensors/adcs/gyro")]


def test_seu_upset_is_finite_and_flips_bits():
    rng = np.random.default_rng(0)
    for value in (0.0, 1.0, -3.7e-5, 8.1, 2.9e17):
        for _ in range(200):
            out = seu_upset(value, rng)
            assert math.isfinite(out)
            assert flip_bit(out, 0) != out  # sanity: helper really flips


def test_scheduled_stuck_gyro_freezes_output_word():
    sim = physics_sim(sensor_stuck(60.0, "gyro"))
    sim.run(duration=120)
    frames = gyro_payloads(sim)
    # healthy sensor never repeats exactly; latched sensor never varies
    assert len(set(frames[:60])) == len(frames[:60])
    assert len(set(frames[-50:])) == 1
    sim.close()


def test_adcs_power_cycle_clears_soft_latchup():
    sim = physics_sim(sensor_stuck(30.0, "gyro"))
    sim.run(duration=60)
    sim.bus.publish("cmd/loads/adcs", "test", {"on": False})
    sim.run(duration=5)
    sim.bus.publish("cmd/loads/adcs", "test", {"on": True})
    sim.run(duration=60)
    cleared = [e for e in sim.recorder.events("physics")
               if e[3] == "latchup_cleared"]
    assert len(cleared) == 1
    frames = gyro_payloads(sim)
    assert len(set(frames[-40:])) == len(frames[-40:])  # noise is back
    sim.close()


def test_hard_fault_survives_power_cycle():
    sim = physics_sim(sensor_stuck(30.0, "gyro", hard=True))
    sim.run(duration=60)
    sim.bus.publish("cmd/loads/adcs", "test", {"on": False})
    sim.run(duration=5)
    sim.bus.publish("cmd/loads/adcs", "test", {"on": True})
    sim.run(duration=60)
    assert not any(e[3] == "latchup_cleared" for e in sim.recorder.events("physics"))
    frames = gyro_payloads(sim)
    assert len(set(frames[-40:])) == 1  # still frozen
    sim.close()


def test_seu_corrupts_exactly_one_reading():
    sim = physics_sim(seu(30.0, "gyro"))
    sim.run(duration=80)
    hits = [e for e in sim.recorder.events("physics") if e[3] == "seu_corruption"]
    assert len(hits) == 1
    frames = gyro_payloads(sim)
    assert len(set(frames)) == len(frames)  # never latches, life goes on
    sim.close()


def test_wheel_friction_and_array_hit_change_truth():
    sim = physics_sim(
        ScheduledFault(20.0, "fault/wheel_friction", {"nm_per_nms": 5e-5}),
        ScheduledFault(20.0, "fault/array_hit", {"mult": 0.5}),
    )
    physics = sim.components[0]
    illum_before = physics.array.illumination
    sim.run(duration=40)
    assert physics.attitude.p.wheel_friction_nm_per_nms == 5e-5
    assert abs(physics.array.illumination - 0.5 * illum_before) < 1e-6
    sim.close()


def test_saa_geometry_is_crossed_and_left():
    """The orbit (51.6 deg, ~95 min) must pass in and out of the SAA box;
    the injector's geometry test is what modulates SEU rates."""
    sim = Simulation(dt=60.0, seed=7)
    sim.add(SpacecraftPhysics())
    sim.add(FaultInjector(seu_rate_per_day=1e-9))  # enable geometry tracking
    sim.run(duration=3 * sim.components[0].orbit.period_s)
    samples = [v for *_, v in sim.recorder.telemetry("faults", "in_saa")]
    assert 1.0 in samples and 0.0 in samples
    sim.close()


def test_injection_storm_is_seed_deterministic():
    def storm(seed):
        sim = physics_sim(seed=seed, seu_rate_per_day=2000.0)
        sim.run(duration=1500)
        events = [(e[0], e[3], e[4]) for e in sim.recorder.events("faults")]
        sim.close()
        return events

    a, b, other = storm(3), storm(3), storm(4)
    assert a and a == b
    assert a != other
