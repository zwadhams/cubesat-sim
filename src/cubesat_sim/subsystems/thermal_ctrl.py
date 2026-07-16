"""Thermal control: a thermostat for the battery heater.

It does not own the heater switch — it *requests* the heater from the EPS,
which can veto it during load shedding. A power system protecting the
battery's charge by freezing the battery is exactly the kind of
inter-subsystem conflict this simulator exists to study.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component


class ThermalControl(Component):
    def __init__(
        self,
        heater_on_k: float = 278.15,   # request heat below +5 C
        heater_off_k: float = 283.15,  # release above +10 C
        reassert_every_s: float = 60.0,
    ) -> None:
        super().__init__("thermal", period=1.0)
        self.heater_on_k = heater_on_k
        self.heater_off_k = heater_off_k
        self.reassert_every_s = reassert_every_s
        self.t_batt: float | None = None
        self.want_heater = False
        self._last_assert: float | None = None

    def on_start(self) -> None:
        self.subscribe("sensors/thermal/*")

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            if msg.topic == "sensors/thermal/battery_temp":
                self.t_batt = float(msg.data["kelvin"])

        if self.t_batt is None:
            return

        changed = False
        if not self.want_heater and self.t_batt < self.heater_on_k:
            self.want_heater = True
            changed = True
        elif self.want_heater and self.t_batt > self.heater_off_k:
            self.want_heater = False
            changed = True

        due = (self._last_assert is None
               or t - self._last_assert >= self.reassert_every_s)
        if changed or due:
            self.publish("thermal/request/heater", on=self.want_heater)
            self._last_assert = t

        self.record("battery_temp_k", self.t_batt)
        self.record("heater_request", float(self.want_heater))
