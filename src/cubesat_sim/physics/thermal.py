"""Lumped-parameter thermal truth model.

Two nodes: the spacecraft structure (absorbs sun and Earth IR, radiates to
space as T^4, receives most electronics dissipation) and the battery
(conductively coupled to structure, warmed by its heater). Parameters are
tuned to a plausible 3U CubeSat: roughly -5..+20 C structure swing per
orbit at the default solar input.
"""

from __future__ import annotations

from dataclasses import dataclass

CELSIUS_ZERO_K = 273.15


@dataclass
class ThermalNode:
    heat_capacity_j_k: float
    temp_k: float


class ThermalModel:
    def __init__(
        self,
        sun_absorbed_w: float = 36.0,
        earth_ir_w: float = 8.0,
        rad_coeff_w_k4: float = 6.35e-9,   # eps * sigma * A_effective
        g_batt_struct_w_k: float = 0.15,
        c_struct_j_k: float = 3500.0,
        c_batt_j_k: float = 300.0,
        initial_temp_k: float = 283.0,
    ) -> None:
        self.sun_absorbed_w = sun_absorbed_w
        self.earth_ir_w = earth_ir_w
        self.rad_coeff_w_k4 = rad_coeff_w_k4
        self.g_batt_struct_w_k = g_batt_struct_w_k
        self.structure = ThermalNode(c_struct_j_k, initial_temp_k)
        self.battery = ThermalNode(c_batt_j_k, initial_temp_k)

    def step(self, dt_s: float, in_sun: bool,
             struct_dissipation_w: float, batt_dissipation_w: float) -> None:
        t_s = self.structure.temp_k
        t_b = self.battery.temp_k

        q_env = self.earth_ir_w + (self.sun_absorbed_w if in_sun else 0.0)
        q_rad = self.rad_coeff_w_k4 * t_s**4
        q_cond = self.g_batt_struct_w_k * (t_b - t_s)  # battery -> structure

        self.structure.temp_k += (
            (q_env + struct_dissipation_w + q_cond - q_rad)
            / self.structure.heat_capacity_j_k * dt_s
        )
        self.battery.temp_k += (
            (batt_dissipation_w - q_cond)
            / self.battery.heat_capacity_j_k * dt_s
        )
