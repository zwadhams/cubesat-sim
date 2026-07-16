"""On-board computer: mode management.

Two modes for now. NOMINAL runs the payload; SAFE turns it off to protect
the battery. Transitions are hysteresis-banded on the EPS's *estimated*
state of charge — which is noisy and sag-biased, so the OBC is steering by
a gauge that reads low exactly when things are most stressed.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component

NOMINAL = "NOMINAL"
SAFE = "SAFE"


class OBC(Component):
    def __init__(
        self,
        safe_enter_soc: float = 0.25,
        safe_exit_soc: float = 0.45,
        reassert_every_s: float = 30.0,
    ) -> None:
        super().__init__("obc", period=1.0)
        self.safe_enter_soc = safe_enter_soc
        self.safe_exit_soc = safe_exit_soc
        self.reassert_every_s = reassert_every_s
        self.mode = NOMINAL
        self.soc_est: float | None = None
        self._last_assert: float | None = None

    def on_start(self) -> None:
        self.subscribe("eps/status")

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            self.soc_est = float(msg.data["soc_est"])

        changed = False
        if self.soc_est is not None:
            if self.mode == NOMINAL and self.soc_est < self.safe_enter_soc:
                self.mode = SAFE
                changed = True
            elif self.mode == SAFE and self.soc_est > self.safe_exit_soc:
                self.mode = NOMINAL
                changed = True
            if changed:
                self.event("mode_change", to=self.mode, soc_est=self.soc_est)
                self.publish("obc/mode", mode=self.mode)

        desired = {"adcs": True, "payload": self.mode == NOMINAL}
        due_reassert = (
            self._last_assert is None or t - self._last_assert >= self.reassert_every_s
        )
        if changed or due_reassert:
            self.publish("obc/request/loads", loads=desired)
            self._last_assert = t

        self.record("safe_mode", float(self.mode == SAFE))
