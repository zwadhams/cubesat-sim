"""The spacecraft physics component — the only place ground truth lives.

Every tick it propagates the orbit, works out sun/eclipse, applies switched
loads, integrates the battery, and publishes *noisy sensor readings* to the
bus. Subsystems never see truth: they see `sensors/*` topics and nothing
else. Actuation comes back in as `cmd/loads/*` switch commands.
"""

from __future__ import annotations

import numpy as np

from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.kernel.component import Component
from cubesat_sim.physics.power import Battery, SolarArray

DEFAULT_LOADS = {
    "obc": 0.4,      # W, always on
    "radio": 0.3,    # W, always on (receiver)
    "adcs": 1.2,     # W, switchable
    "payload": 2.5,  # W, switchable
}
ESSENTIAL_LOADS = frozenset({"obc", "radio"})


class SpacecraftPhysics(Component):
    def __init__(
        self,
        orbit: CircularOrbit | None = None,
        array: SolarArray | None = None,
        battery: Battery | None = None,
        loads: dict[str, float] | None = None,
        record_every_s: float = 5.0,
    ) -> None:
        super().__init__("physics", period=0.0)  # every tick
        self.orbit = orbit or CircularOrbit()
        self.array = array or SolarArray()
        self.battery = battery or Battery()
        self.loads = dict(loads or DEFAULT_LOADS)
        # essentials and adcs start powered; payload waits for a command
        self.switches = {n: (n in ESSENTIAL_LOADS or n == "adcs") for n in self.loads}
        self._record_every_s = record_every_s
        self._prev_eclipse: bool | None = None
        self._brownout = False

    def on_start(self) -> None:
        self.subscribe("cmd/loads/*")

    def step(self, t: float, dt: float) -> None:
        # apply switch commands (essential loads cannot be switched off)
        for msg in self.drain():
            name = msg.topic.rsplit("/", 1)[-1]
            if name in self.loads and name not in ESSENTIAL_LOADS:
                self.switches[name] = bool(msg.data.get("on"))

        # environment truth
        sun_hat = sun_direction_eci(self.clock.utc)
        r_eci = self.orbit.position_eci(t)
        eclipse = self.orbit.in_eclipse(r_eci, sun_hat)
        if self._prev_eclipse is None:
            self._prev_eclipse = eclipse
        elif eclipse != self._prev_eclipse:
            self.event("eclipse_enter" if eclipse else "eclipse_exit")
            self._prev_eclipse = eclipse

        # power truth
        p_gen = self.array.generation_w(eclipse)
        if p_gen > 0.0:
            p_gen *= max(0.0, 1.0 + float(self.rng.normal(0.0, 0.02)))
        p_load = sum(self.loads[n] for n, on in self.switches.items() if on)
        p_net = p_gen - p_load
        self.battery.integrate(p_net, dt)

        if self.battery.soc <= 0.0 and not self._brownout:
            self._brownout = True
            for name in self.switches:
                if name not in ESSENTIAL_LOADS:
                    self.switches[name] = False
            self.event("brownout")
            self.publish("physics/brownout")
        elif self._brownout and self.battery.soc > 0.05:
            self._brownout = False

        # noisy sensor readings — all any subsystem ever gets to see
        v_true = self.battery.voltage(p_net)
        self.publish("sensors/eps/battery_voltage",
                     volts=v_true + float(self.rng.normal(0.0, 0.02)))
        self.publish("sensors/eps/solar_power",
                     watts=max(0.0, p_gen + float(self.rng.normal(0.0, 0.05))))
        self.publish("sensors/eps/load_power",
                     watts=max(0.0, p_load + float(self.rng.normal(0.0, 0.05))))

        # ground-truth flight recording (never on the bus)
        every = max(1, round(self._record_every_s / dt))
        if self.clock.tick % every == 0:
            self.record("soc_true", self.battery.soc)
            self.record("battery_v_true", v_true)
            self.record("p_gen_w", p_gen)
            self.record("p_load_w", p_load)
            self.record("eclipse", float(eclipse))
