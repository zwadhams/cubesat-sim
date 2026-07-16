"""On-board computer: mode management and FDIR.

Two modes. NOMINAL runs the payload; SAFE turns it off to protect the
battery. Transitions are hysteresis-banded on the EPS's *estimated* state
of charge — which is noisy and sag-biased, so the OBC is steering by a
gauge that reads low exactly when things are most stressed.

FDIR (fault detection, isolation, recovery): a gyro health watchdog.
Detection is twofold — a *latched* sensor repeats its output word exactly
(healthy noise never does), and an *insane* sensor reads rates no real
flight can reach. Either one, sustained, triggers the classic first move
of spacecraft FDIR: power-cycle the misbehaving unit. The budget is
MAX_CYCLES per flight; after that the OBC gives up, leaves the ADCS
powered, and lets the ground worry about it.

NOTE: this file is the reference implementation. c/obc/main.c must remain
bit-identical to it — the equivalence test in tests/test_remote_obc.py
compares whole flights. Change both or neither.
"""

from __future__ import annotations

from cubesat_sim.kernel.component import Component

NOMINAL = "NOMINAL"
SAFE = "SAFE"

RATE2_LIMIT = 0.1225   # (0.35 rad/s)^2 ~ 20 deg/s: no credible flight rate
STUCK_TRIGGER = 5      # consecutive exact-repeat samples -> latched sensor
INSANE_TRIGGER = 3     # consecutive out-of-range samples -> latched sensor
CYCLE_OFF_S = 20.0     # power-cycle off dwell
MAX_CYCLES = 3         # FDIR retry budget per flight


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
        # FDIR state
        self.cycling = False
        self.cycle_started = 0.0
        self.cycles_used = 0
        self.gave_up = False
        self.stuck_run = 0
        self.insane_run = 0
        self._prev_gyro: tuple[float, float, float] | None = None

    def on_start(self) -> None:
        self.subscribe("eps/status")
        self.subscribe("sensors/adcs/gyro")

    def step(self, t: float, dt: float) -> None:
        gyro: tuple[float, float, float] | None = None
        for msg in self.drain():
            if msg.topic == "eps/status":
                self.soc_est = float(msg.data["soc_est"])
            elif msg.topic == "sensors/adcs/gyro":
                gyro = (float(msg.data["x"]), float(msg.data["y"]),
                        float(msg.data["z"]))

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

        # -- FDIR: gyro health watchdog -> ADCS power cycle ------------------
        fdir_changed = False
        masked = self.cycling  # data from an unpowered rail proves nothing
        if self.cycling and t - self.cycle_started >= CYCLE_OFF_S:
            self.cycling = False
            fdir_changed = True
            self.event("fdir_adcs_repower", cycles_used=self.cycles_used)
        if gyro is not None and not masked:
            if self._prev_gyro is not None and gyro == self._prev_gyro:
                self.stuck_run += 1
            else:
                self.stuck_run = 0
            rate2 = gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]
            if rate2 > RATE2_LIMIT:
                self.insane_run += 1
                if self.insane_run == 1:
                    self.event("gyro_anomaly")
            else:
                self.insane_run = 0
            self._prev_gyro = gyro
            if self.stuck_run >= STUCK_TRIGGER or self.insane_run >= INSANE_TRIGGER:
                if self.cycles_used >= MAX_CYCLES:
                    if not self.gave_up:
                        self.gave_up = True
                        self.event("fdir_giveup", cycles_used=self.cycles_used)
                else:
                    self.cycling = True
                    self.cycle_started = t
                    self.cycles_used += 1
                    self.stuck_run = 0
                    self.insane_run = 0
                    self._prev_gyro = None
                    fdir_changed = True
                    self.event("fdir_adcs_power_cycle", n=self.cycles_used)

        desired = {"adcs": not self.cycling, "payload": self.mode == NOMINAL}
        due_reassert = (
            self._last_assert is None or t - self._last_assert >= self.reassert_every_s
        )
        if changed or fdir_changed or due_reassert:
            self.publish("obc/request/loads", loads=desired)
            self._last_assert = t

        self.record("safe_mode", float(self.mode == SAFE))
        self.record("fdir_cycles", float(self.cycles_used))
