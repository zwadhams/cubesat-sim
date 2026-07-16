"""Two-body circular orbit propagation and eclipse geometry (ECI frame)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

MU_EARTH = 3.986004418e14  # m^3/s^2
R_EARTH = 6371e3  # mean radius, m — used for both orbit altitude and shadow


@dataclass
class CircularOrbit:
    """Circular two-body orbit, parameterized by altitude, inclination, RAAN,
    and argument of latitude at epoch. No J2 or drag yet — orbits repeat."""

    altitude_m: float = 500e3
    inclination_rad: float = math.radians(51.6)
    raan_rad: float = 0.0
    arg_lat_epoch_rad: float = 0.0

    _p: np.ndarray = field(init=False, repr=False)
    _q: np.ndarray = field(init=False, repr=False)

    def __post_init__(self) -> None:
        ci, si = math.cos(self.inclination_rad), math.sin(self.inclination_rad)
        co, so = math.cos(self.raan_rad), math.sin(self.raan_rad)
        # in-plane basis: _p at ascending node, _q 90 deg ahead in flight direction
        self._p = np.array([co, so, 0.0])
        self._q = np.array([-so * ci, co * ci, si])

    @property
    def semi_major_axis_m(self) -> float:
        return R_EARTH + self.altitude_m

    @property
    def mean_motion_rad_s(self) -> float:
        return math.sqrt(MU_EARTH / self.semi_major_axis_m**3)

    @property
    def period_s(self) -> float:
        return 2.0 * math.pi / self.mean_motion_rad_s

    def position_eci(self, t: float) -> np.ndarray:
        u = self.arg_lat_epoch_rad + self.mean_motion_rad_s * t
        return self.semi_major_axis_m * (math.cos(u) * self._p + math.sin(u) * self._q)

    @staticmethod
    def in_eclipse(r_eci: np.ndarray, sun_hat: np.ndarray) -> bool:
        """Cylindrical Earth-shadow model: in eclipse when on the anti-sun
        side and within one Earth radius of the shadow axis."""
        along = float(np.dot(r_eci, sun_hat))
        if along >= 0.0:
            return False
        perp = r_eci - along * sun_hat
        return float(np.linalg.norm(perp)) < R_EARTH
