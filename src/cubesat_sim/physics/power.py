"""Physical truth models for the power chain: solar array and battery."""

from __future__ import annotations

from dataclasses import dataclass

YEAR_S = 365.25 * 86400.0


@dataclass
class SolarArray:
    """Body-mounted array, dominant panel normal on body +Z. `illumination`
    is an array-health factor in [0, 1] (deployment damage, degradation).
    `facing` is the cosine of the sun angle on the main panel; side panels
    provide the `min_facing` floor when the main panel looks away.
    `decay_per_year` is radiation darkening: illumination decays while the
    array is in sunlight, down to `illum_floor`."""

    p_max_w: float = 10.0
    illumination: float = 0.8
    min_facing: float = 0.15
    decay_per_year: float = 0.0
    illum_floor: float = 0.1

    def generation_w(self, in_eclipse: bool, facing: float = 1.0) -> float:
        if in_eclipse:
            return 0.0
        return self.p_max_w * self.illumination * max(self.min_facing, facing)

    def age(self, dt_s: float, in_sun: bool) -> None:
        if in_sun and self.decay_per_year > 0.0:
            self.illumination = max(
                self.illum_floor,
                self.illumination * (1.0 - self.decay_per_year * dt_s / YEAR_S),
            )


@dataclass
class Battery:
    """2S li-ion pack: linear open-circuit voltage curve plus an internal
    resistance term, so terminal voltage sags under discharge and rises
    under charge. That sag is what makes voltage-based SoC estimation
    honestly wrong in interesting ways.

    `fade_per_wh` is cycle aging: capacity lost per Wh of throughput in
    either direction (~2e-4 gives the classic 20% loss over 500 full
    cycles), floored at `capacity_floor_wh`."""

    capacity_wh: float = 20.0
    soc: float = 0.85  # state of charge, 0..1 (ground truth)
    v_empty: float = 6.0
    v_full: float = 8.4
    esr_v_per_a: float = 0.25
    fade_per_wh: float = 0.0
    capacity_floor_wh: float = 4.0

    def integrate(self, p_net_w: float, dt_s: float) -> None:
        """Advance state of charge by net power (generation - load)."""
        wh = p_net_w * dt_s / 3600.0
        if self.fade_per_wh > 0.0:
            self.capacity_wh = max(self.capacity_floor_wh,
                                   self.capacity_wh - abs(wh) * self.fade_per_wh)
        self.soc = min(1.0, max(0.0, self.soc + wh / self.capacity_wh))

    @property
    def open_circuit_v(self) -> float:
        return self.v_empty + (self.v_full - self.v_empty) * self.soc

    def voltage(self, p_net_w: float) -> float:
        """Terminal voltage under the given net power flow (charge positive)."""
        v_oc = self.open_circuit_v
        current_a = p_net_w / max(v_oc, 1e-6)
        return v_oc + self.esr_v_per_a * current_a
