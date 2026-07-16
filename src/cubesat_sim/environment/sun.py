"""Low-precision solar position (Astronomical Almanac approximation).

Accurate to ~0.01 degrees over decades around J2000 — far more than enough
for power and eclipse modeling.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone

import numpy as np

J2000 = datetime(2000, 1, 1, 12, 0, tzinfo=timezone.utc)


def sun_direction_eci(when: datetime) -> np.ndarray:
    """Unit vector from Earth's center toward the Sun, in ECI coordinates."""
    n = (when - J2000).total_seconds() / 86400.0
    mean_lon = math.radians((280.460 + 0.9856474 * n) % 360.0)
    mean_anom = math.radians((357.528 + 0.9856003 * n) % 360.0)
    ecl_lon = (mean_lon
               + math.radians(1.915) * math.sin(mean_anom)
               + math.radians(0.020) * math.sin(2.0 * mean_anom))
    obliquity = math.radians(23.439 - 4.0e-7 * n)
    s = np.array([
        math.cos(ecl_lon),
        math.cos(obliquity) * math.sin(ecl_lon),
        math.sin(obliquity) * math.sin(ecl_lon),
    ])
    return s / np.linalg.norm(s)
