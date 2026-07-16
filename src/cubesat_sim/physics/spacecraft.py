"""The spacecraft physics component — the only place ground truth lives.

Every tick it propagates the orbit, works out sun/eclipse, applies switched
loads, advances the thermal state, and integrates the battery. It publishes
*noisy sensor readings* to the bus; subsystems never see truth. Actuation
comes back in as `cmd/loads/*` switch commands.

One hard physical constraint couples power to thermal: lithium-ion cells
must not charge below 0 C. When the battery is that cold, charge power is
inhibited and dumped as heat into the structure instead. Combined with a
sheddable heater, this is the raw material of a death spiral.
"""

from __future__ import annotations

import numpy as np

from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.kernel.component import Component
from cubesat_sim.physics.power import Battery, SolarArray
from cubesat_sim.physics.thermal import CELSIUS_ZERO_K, ThermalModel

DEFAULT_LOADS = {
    "obc": 0.4,         # W, always on
    "radio": 0.3,       # W, always on (receiver)
    "adcs": 1.2,        # W, switchable
    "payload": 2.5,     # W, switchable
    "bat_heater": 1.5,  # W, switchable, dissipates into the battery node
}
ESSENTIAL_LOADS = frozenset({"obc", "radio"})

MIN_CHARGE_TEMP_K = CELSIUS_ZERO_K  # li-ion cold-charge cutoff


class SpacecraftPhysics(Component):
    def __init__(
        self,
        orbit: CircularOrbit | None = None,
        array: SolarArray | None = None,
        battery: Battery | None = None,
        thermal: ThermalModel | None = None,
        loads: dict[str, float] | None = None,
        record_every_s: float = 5.0,
    ) -> None:
        super().__init__("physics", period=0.0)  # every tick
        self.orbit = orbit or CircularOrbit()
        self.array = array or SolarArray()
        self.battery = battery or Battery()
        self.thermal = thermal or ThermalModel()
        self.loads = dict(loads or DEFAULT_LOADS)
        # essentials and adcs start powered; payload and heater wait for commands
        self.switches = {n: (n in ESSENTIAL_LOADS or n == "adcs") for n in self.loads}
        self._record_every_s = record_every_s
        self._prev_eclipse: bool | None = None
        self._brownout = False
        self._charge_blocked = False

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

        # electrical truth
        p_gen = self.array.generation_w(eclipse)
        if p_gen > 0.0:
            p_gen *= max(0.0, 1.0 + float(self.rng.normal(0.0, 0.02)))
        heater_w = self.loads["bat_heater"] if self.switches["bat_heater"] else 0.0
        p_load = sum(self.loads[n] for n, on in self.switches.items() if on)
        p_net = p_gen - p_load

        # cold-charge inhibit: charging a frozen li-ion pack is forbidden;
        # blocked charge power is dumped as heat into the structure
        charge_blocked = (
            p_net > 0.0 and self.thermal.battery.temp_k < MIN_CHARGE_TEMP_K
        )
        if charge_blocked != self._charge_blocked:
            self.event("charge_inhibit_on" if charge_blocked else "charge_inhibit_off",
                       batt_temp_k=self.thermal.battery.temp_k)
            self._charge_blocked = charge_blocked
        dump_w = p_net if charge_blocked else 0.0

        # thermal truth (electronics dissipate into structure, heater into battery)
        self.thermal.step(
            dt,
            in_sun=not eclipse,
            struct_dissipation_w=(p_load - heater_w) + dump_w,
            batt_dissipation_w=heater_w,
        )

        if not charge_blocked:
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
        v_true = self.battery.voltage(0.0 if charge_blocked else p_net)
        self.publish("sensors/eps/battery_voltage",
                     volts=v_true + float(self.rng.normal(0.0, 0.02)))
        self.publish("sensors/eps/solar_power",
                     watts=max(0.0, p_gen + float(self.rng.normal(0.0, 0.05))))
        self.publish("sensors/eps/load_power",
                     watts=max(0.0, p_load + float(self.rng.normal(0.0, 0.05))))
        self.publish("sensors/thermal/battery_temp",
                     kelvin=self.thermal.battery.temp_k + float(self.rng.normal(0.0, 0.3)))
        self.publish("sensors/thermal/structure_temp",
                     kelvin=self.thermal.structure.temp_k + float(self.rng.normal(0.0, 0.3)))

        # ground-truth flight recording (never on the bus)
        every = max(1, round(self._record_every_s / dt))
        if self.clock.tick % every == 0:
            self.record("soc_true", self.battery.soc)
            self.record("battery_v_true", v_true)
            self.record("p_gen_w", p_gen)
            self.record("p_load_w", p_load)
            self.record("eclipse", float(eclipse))
            self.record("t_batt_k", self.thermal.battery.temp_k)
            self.record("t_struct_k", self.thermal.structure.temp_k)
            self.record("charge_blocked", float(charge_blocked))
