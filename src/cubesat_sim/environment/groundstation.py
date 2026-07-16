"""Ground site geometry: where is a lat/lon site in ECI, and can it see us?

Earth rotation matters here — a ground station sweeps east at ~15 deg/h,
which is what gives LEO its characteristic pass pattern (clusters of a few
passes, then hours of silence). Sidereal angle via the standard GMST
approximation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

from cubesat_sim.environment.orbit import R_EARTH

_J2000 = datetime(2000, 1, 1, 12, 0, tzinfo=timezone.utc)


def gmst_rad(when: datetime) -> float:
    days = (when - _J2000).total_seconds() / 86400.0
    theta_deg = (280.46061837 + 360.98564736629 * days) % 360.0
    return math.radians(theta_deg)


@dataclass(frozen=True)
class GroundSite:
    name: str
    lat_deg: float
    lon_deg: float

    def position_eci(self, when: datetime) -> np.ndarray:
        lat = math.radians(self.lat_deg)
        lon_eci = math.radians(self.lon_deg) + gmst_rad(when)
        return R_EARTH * np.array([
            math.cos(lat) * math.cos(lon_eci),
            math.cos(lat) * math.sin(lon_eci),
            math.sin(lat),
        ])

    def elevation_deg(self, sat_r_eci: np.ndarray, when: datetime) -> float:
        """Satellite elevation above this site's horizon."""
        site = self.position_eci(when)
        up = site / np.linalg.norm(site)
        rel = sat_r_eci - site
        sin_el = float(np.dot(rel, up) / np.linalg.norm(rel))
        return math.degrees(math.asin(max(-1.0, min(1.0, sin_el))))
