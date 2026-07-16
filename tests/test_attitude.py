import numpy as np

from cubesat_sim.environment.magfield import dipole_field_eci
from cubesat_sim.environment.orbit import R_EARTH
from cubesat_sim.physics.attitude import (
    AttitudeDynamics,
    AttitudeParams,
    dcm_from_quat,
)


def test_dcm_convention_rotation_about_z():
    """Body rotates +90 deg about shared z: inertial x seen from the body
    frame should be [0, -1, 0] under our body-from-inertial convention."""
    att = AttitudeDynamics()
    att.omega = np.array([0.0, 0.0, np.pi / 2 / 100.0])
    for _ in range(100):
        att.step(1.0, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    v_body = att.body_from_eci(np.array([1.0, 0.0, 0.0]))
    assert np.allclose(v_body, [0.0, -1.0, 0.0], atol=0.02)


def test_quaternion_stays_normalized():
    att = AttitudeDynamics()
    att.omega = np.deg2rad([3.0, -2.0, 4.0])
    for _ in range(5000):
        att.step(1.0, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    assert abs(np.linalg.norm(att.q) - 1.0) < 1e-9


def test_angular_momentum_conserved_without_external_torque():
    att = AttitudeDynamics()
    att.omega = np.deg2rad([4.0, -1.0, 2.0])
    h0 = att.angular_momentum_eci()
    # wheels exchange momentum internally; total must stay put
    for i in range(2000):
        tau_w = np.array([2e-4 * np.sin(i / 50.0), -1e-4, 5e-5])
        att.step(0.1, np.zeros(3), tau_w, np.zeros(3), np.zeros(3))
    h1 = att.angular_momentum_eci()
    assert np.linalg.norm(h1 - h0) < 0.02 * np.linalg.norm(h0)


def test_wheels_saturate():
    att = AttitudeDynamics()
    # +x body torque -> wheel momentum runs negative until the stop
    for _ in range(200):
        att.step(1.0, np.zeros(3), np.array([1e-3, 0, 0]), np.zeros(3), np.zeros(3))
    assert abs(att.h_wheel[0] + att.p.wheel_h_max_nms) < 1e-12
    # a saturated wheel delivers no further torque
    w_before = att.omega.copy()
    att.step(1.0, np.zeros(3), np.array([1e-3, 0, 0]), np.zeros(3), np.zeros(3))
    assert np.allclose(att.omega, w_before, atol=1e-9)


def test_wheel_command_is_body_torque():
    """Flight-natural sign contract: positive commanded torque spins the
    body positive. The ADCS damping law depends on this; getting it
    backwards turned the damper into a pump (EMERGENT_BEHAVIORS.md #5)."""
    att = AttitudeDynamics()
    att.step(1.0, np.zeros(3), np.array([1e-4, 0, 0]), np.zeros(3), np.zeros(3))
    assert att.omega[0] > 0
    assert att.h_wheel[0] < 0  # reaction stored in the wheel


def test_wheel_friction_conserves_momentum():
    """Bearing drag moves momentum from the wheels into the body — it must
    never create or destroy any. A frictious wheel spins down; the body
    picks up exactly what the wheel loses."""
    att = AttitudeDynamics(AttitudeParams(wheel_friction_nm_per_nms=1e-4))
    att.h_wheel = np.array([5e-3, -3e-3, 0.0])
    h0 = att.angular_momentum_eci()
    wheel0 = np.abs(att.h_wheel.copy())
    for _ in range(5000):
        att.step(1.0, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    # spinning wheels bled down (~40% at k=1e-4 over 5000 s); the idle
    # z wheel has nothing to bleed and stays put
    assert np.all(np.abs(att.h_wheel[:2]) < 0.8 * wheel0[:2])
    assert att.h_wheel[2] == 0.0
    assert att.rate_rad_s > 0.0  # the body picked the momentum up
    h1 = att.angular_momentum_eci()
    assert np.linalg.norm(h1 - h0) < 0.02 * np.linalg.norm(h0)


def test_wheel_friction_grows_over_time():
    att = AttitudeDynamics(AttitudeParams(wheel_friction_growth_per_s=1e-9))
    for _ in range(1000):
        att.step(1.0, np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3))
    assert abs(att.p.wheel_friction_nm_per_nms - 1e-6) < 1e-12


def test_dipole_field_shape():
    equator = dipole_field_eci(np.array([R_EARTH, 0.0, 0.0]))
    pole = dipole_field_eci(np.array([0.0, 0.0, R_EARTH]))
    assert equator[2] > 0  # points north at the equator
    assert abs(np.linalg.norm(equator) - 3.12e-5) < 1e-7
    assert np.linalg.norm(pole) / np.linalg.norm(equator) > 1.9  # ~2x at poles
    far = dipole_field_eci(np.array([2 * R_EARTH, 0.0, 0.0]))
    assert np.linalg.norm(far) < np.linalg.norm(equator) / 7.9  # 1/r^3
