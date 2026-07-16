"""Electrical power subsystem: the power distribution unit.

EPS is the single authority over load switches. The OBC *requests* load
states; EPS honors requests unless its own (deliberately lower) shed
threshold trips, in which case it vetoes non-essential loads until the
battery recovers. Two controllers, two thresholds, one resource — that
overlap is an emergence ingredient, not an accident.

EPS estimates state of charge from measured bus voltage, so its estimate
inherits sensor noise and sag error: under eclipse discharge it reads
pessimistic, under charge optimistic.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component

NON_ESSENTIAL = ("adcs", "payload")

# voltage->SoC calibration (matches the battery design values)
V_EMPTY = 6.0
V_FULL = 8.4


class EPS(Component):
    def __init__(self, shed_soc: float = 0.15, restore_soc: float = 0.30) -> None:
        super().__init__("eps", period=1.0)
        self.shed_soc = shed_soc
        self.restore_soc = restore_soc
        self.v_meas: float | None = None
        self.solar_w = 0.0
        self.load_w = 0.0
        self.soc_est: float | None = None
        self.shedding = False
        self.desired = {"adcs": True, "payload": False}  # until OBC says otherwise
        self._commanded: dict[str, bool] = {}

    def on_start(self) -> None:
        self.subscribe("sensors/eps/*")
        self.subscribe("obc/request/loads")

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            if msg.topic == "sensors/eps/battery_voltage":
                self.v_meas = float(msg.data["volts"])
            elif msg.topic == "sensors/eps/solar_power":
                self.solar_w = float(msg.data["watts"])
            elif msg.topic == "sensors/eps/load_power":
                self.load_w = float(msg.data["watts"])
            elif msg.topic == "obc/request/loads":
                self.desired.update(msg.data["loads"])

        if self.v_meas is None:
            return  # no telemetry yet

        self.soc_est = min(1.0, max(0.0, (self.v_meas - V_EMPTY) / (V_FULL - V_EMPTY)))

        if not self.shedding and self.soc_est < self.shed_soc:
            self.shedding = True
            self.event("load_shed", soc_est=self.soc_est)
        elif self.shedding and self.soc_est > self.restore_soc:
            self.shedding = False
            self.event("load_restore", soc_est=self.soc_est)

        target = dict(self.desired)
        if self.shedding:
            for name in NON_ESSENTIAL:
                target[name] = False

        for name, on in target.items():
            if self._commanded.get(name) != on:
                self.publish(f"cmd/loads/{name}", on=on)
                self._commanded[name] = on

        self.publish(
            "eps/status",
            soc_est=self.soc_est,
            battery_v=self.v_meas,
            solar_w=self.solar_w,
            load_w=self.load_w,
            shedding=self.shedding,
        )
        self.record("soc_est", self.soc_est)
        self.record("shedding", float(self.shedding))
