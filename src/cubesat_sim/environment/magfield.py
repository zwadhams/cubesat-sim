"""Earth magnetic field: centered dipole model.

Good to ~20-30% versus the real field (no tilt, no anomalies) — plenty for
magnetorquer control studies. The dipole moment is aligned with the spin
axis, pointing geographic south, which makes the field point north at the
equator as it should.
"""

from __future__ import annotations

import numpy as np

from cubesat_sim.environment.orbit import R_EARTH

B0_SURFACE_T = 3.12e-5  # mean equatorial surface field
_M_HAT = np.array([0.0, 0.0, -1.0])  # dipole moment direction (south)


def dipole_field_eci(r_eci: np.ndarray) -> np.ndarray:
    """Magnetic field vector (tesla) at ECI position `r_eci` (meters)."""
    r = float(np.linalg.norm(r_eci))
    r_hat = r_eci / r
    scale = B0_SURFACE_T * (R_EARTH / r) ** 3
    return scale * (3.0 * float(np.dot(_M_HAT, r_hat)) * r_hat - _M_HAT)
