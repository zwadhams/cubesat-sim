from datetime import datetime, timedelta, timezone

import numpy as np

from cubesat_sim.environment.groundstation import GroundSite, gmst_rad
from cubesat_sim.environment.orbit import R_EARTH


def test_elevation_overhead_and_antipode():
    site = GroundSite("test", 0.0, 0.0)
    when = datetime(2026, 1, 1, tzinfo=timezone.utc)
    overhead = site.position_eci(when) * (R_EARTH + 500e3) / R_EARTH
    assert site.elevation_deg(overhead, when) > 89.0
    assert site.elevation_deg(-overhead, when) < -80.0


def test_earth_rotation_moves_the_site():
    site = GroundSite("test", 45.0, 0.0)
    t0 = datetime(2026, 1, 1, tzinfo=timezone.utc)
    p0 = site.position_eci(t0)
    p6h = site.position_eci(t0 + timedelta(hours=6))
    # six hours ~ quarter turn: positions nearly orthogonal in x-y
    cos_angle = np.dot(p0[:2], p6h[:2]) / (
        np.linalg.norm(p0[:2]) * np.linalg.norm(p6h[:2]))
    assert abs(cos_angle) < 0.1
    assert abs(p0[2] - p6h[2]) < 1.0  # z untouched by spin


def test_gmst_rate():
    t0 = datetime(2026, 1, 1, tzinfo=timezone.utc)
    dtheta = gmst_rad(t0 + timedelta(hours=1)) - gmst_rad(t0)
    dtheta %= 2 * np.pi
    assert abs(np.degrees(dtheta) - 15.04) < 0.05  # sidereal rate
