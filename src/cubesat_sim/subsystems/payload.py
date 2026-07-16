"""Payload controller — software on the payload computer.

Realistically Python: payload processors are Linux boards and often run
exactly this. Images when (a) the payload load switch is powered, (b) the
instrument is software-enabled (ground-commandable via `payload/enable`),
and (c) a target site is in view. Produces data chunks onto the bus; the
comms subsystem owns the downlink queue and its limits.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component


class PayloadController(Component):
    def __init__(self, data_rate_mb_s: float = 0.25) -> None:
        super().__init__("payload", period=1.0)
        self.data_rate_mb_s = data_rate_mb_s
        self.powered = False
        self.visible = False
        self.enabled = True
        self.total_mb = 0.0
        self._was_imaging = False

    def on_start(self) -> None:
        self.subscribe("sensors/payload/*")
        self.subscribe("payload/enable")

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            if msg.topic == "sensors/payload/powered":
                self.powered = bool(msg.data["on"])
            elif msg.topic == "sensors/payload/target_visible":
                self.visible = bool(msg.data["visible"])
            elif msg.topic == "payload/enable":
                self.enabled = bool(msg.data["on"])
                self.event("instrument_enable" if self.enabled else "instrument_disable")

        imaging = self.powered and self.enabled and self.visible
        if imaging:
            mb = self.data_rate_mb_s * dt
            self.total_mb += mb
            self.publish("payload/data", mb=mb)
        if imaging != self._was_imaging:
            self.event("imaging_start" if imaging else "imaging_stop")
            self._was_imaging = imaging

        self.record("generated_mb", self.total_mb)
        self.record("imaging", float(imaging))
