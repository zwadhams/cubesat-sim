"""Physical truth models for the power chain: solar array and battery."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SolarArray:
    """Body-mounted array, dominant panel normal on body +Z. `illumination`
    is an array-health factor in [0, 1] (deployment damage, degradation).
    `facing` is the cosine of the sun angle on the main panel; side panels
    provide the `min_facing` floor when the main panel looks away."""

    p_max_w: float = 10.0
    illumination: float = 0.8
    min_facing: float = 0.15

    def generation_w(self, in_eclipse: bool, facing: float = 1.0) -> float:
        if in_eclipse:
            return 0.0
        return self.p_max_w * self.illumination * max(self.min_facing, facing)


@dataclass
class Battery:
    """2S li-ion pack: linear open-circuit voltage curve plus an internal
    resistance term, so terminal voltage sags under discharge and rises
    under charge. That sag is what makes voltage-based SoC estimation
    honestly wrong in interesting ways."""

    capacity_wh: float = 20.0
    soc: float = 0.85  # state of charge, 0..1 (ground truth)
    v_empty: float = 6.0
    v_full: float = 8.4
    esr_v_per_a: float = 0.25

    def integrate(self, p_net_w: float, dt_s: float) -> None:
        """Advance state of charge by net power (generation - load)."""
        delta = p_net_w * dt_s / 3600.0 / self.capacity_wh
        self.soc = min(1.0, max(0.0, self.soc + delta))

    @property
    def open_circuit_v(self) -> float:
        return self.v_empty + (self.v_full - self.v_empty) * self.soc

    def voltage(self, p_net_w: float) -> float:
        """Terminal voltage under the given net power flow (charge positive)."""
        v_oc = self.open_circuit_v
        current_a = p_net_w / max(v_oc, 1e-6)
        return v_oc + self.esr_v_per_a * current_a
