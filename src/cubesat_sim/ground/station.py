"""Ground segment: the station and its (automated) operator.

Receives whatever the RF channel delivers — data frames and telemetry
beacons — and archives it. The operator rule closes a control loop through
space: if downlinked telemetry shows the spacecraft's storage nearly full,
uplink `payload/enable off`; re-enable when it drains. The catch that makes
this interesting: the ground only *sees* storage during a pass, and
commands only *arrive* during a pass, so every decision acts on stale data
with hours of actuation delay.

Commands are sent blind and retried; the physics channel silently drops
them outside contact windows (and sometimes inside — packet loss), exactly
like a real uplink.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component


class GroundStation(Component):
    def __init__(
        self,
        disable_above_frac: float = 0.8,
        enable_below_frac: float = 0.4,
        resend_every_s: float = 90.0,
    ) -> None:
        super().__init__("ground", period=1.0)
        self.disable_above_frac = disable_above_frac
        self.enable_below_frac = enable_below_frac
        self.resend_every_s = resend_every_s
        self.archive_mb = 0.0
        self.telemetry_frames = 0
        self.last_storage_frac: float | None = None
        self.desired_enable: bool | None = None  # None: no opinion yet
        self._last_sent: float | None = None

    def on_start(self) -> None:
        self.subscribe("radio/rx_ground")

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            kind = msg.data.get("kind")
            if kind == "data":
                self.archive_mb += float(msg.data["mb"])
            elif kind == "telemetry":
                self.telemetry_frames += 1
                self.last_storage_frac = float(msg.data["storage_frac"])

        if self.last_storage_frac is not None:
            if (self.desired_enable is not False
                    and self.last_storage_frac > self.disable_above_frac):
                self.desired_enable = False
                self._last_sent = None  # send immediately
                self.event("operator_disable_payload",
                           storage_frac=self.last_storage_frac)
            elif (self.desired_enable is False
                    and self.last_storage_frac < self.enable_below_frac):
                self.desired_enable = True
                self._last_sent = None
                self.event("operator_enable_payload",
                           storage_frac=self.last_storage_frac)

        if self.desired_enable is not None and (
                self._last_sent is None or t - self._last_sent >= self.resend_every_s):
            # blind transmit; the channel decides whether it arrives
            self.publish("ground/tx", cmd_topic="payload/enable",
                         on=self.desired_enable)
            self._last_sent = t

        self.record("archive_mb", self.archive_mb)
        self.record("telemetry_frames", float(self.telemetry_frames))
