import math
from datetime import datetime, timezone

import numpy as np

from cubesat_sim.environment import R_EARTH, CircularOrbit, sun_direction_eci


def test_radius_constant_and_orbit_closes():
    orbit = CircularOrbit(altitude_m=500e3)
    a = orbit.semi_major_axis_m
    for t in (0.0, 1234.5, 4000.0):
        assert abs(np.linalg.norm(orbit.position_eci(t)) - a) < 1.0
    r0 = orbit.position_eci(0.0)
    r_t = orbit.position_eci(orbit.period_s)
    assert np.linalg.norm(r0 - r_t) < 1e-6 * a


def test_period_is_leo_scale():
    orbit = CircularOrbit(altitude_m=500e3)
    assert 5500 < orbit.period_s < 5800  # ~94-96 min


def test_eclipse_fraction_with_sun_in_orbit_plane():
    orbit = CircularOrbit(altitude_m=500e3, inclination_rad=0.0)
    sun = np.array([1.0, 0.0, 0.0])
    a = orbit.semi_major_axis_m
    samples = np.linspace(0.0, orbit.period_s, 20000, endpoint=False)
    frac = np.mean([orbit.in_eclipse(orbit.position_eci(t), sun) for t in samples])
    expected = math.asin(R_EARTH / a) / math.pi
    assert abs(frac - expected) < 0.005


def test_no_eclipse_when_sun_normal_to_orbit_plane():
    orbit = CircularOrbit(altitude_m=500e3, inclination_rad=0.0)
    sun = np.array([0.0, 0.0, 1.0])
    for t in np.linspace(0.0, orbit.period_s, 500):
        assert not orbit.in_eclipse(orbit.position_eci(t), sun)


def test_sun_direction_sanity():
    equinox = sun_direction_eci(datetime(2026, 3, 20, 12, tzinfo=timezone.utc))
    assert abs(np.linalg.norm(equinox) - 1.0) < 1e-9
    assert abs(equinox[2]) < 0.02  # declination near zero at equinox

    solstice = sun_direction_eci(datetime(2026, 12, 21, 12, tzinfo=timezone.utc))
    assert solstice[2] < -0.35  # sun well south in December
