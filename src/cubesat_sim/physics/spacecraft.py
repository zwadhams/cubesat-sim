"""The spacecraft physics component — the only place ground truth lives.

Every tick it propagates the orbit, works out sun/eclipse and the local
magnetic field, advances rigid-body attitude, applies switched loads,
advances the thermal state, and integrates the battery. It publishes
*noisy sensor readings* to the bus; subsystems never see truth. Actuation
comes back in as `cmd/loads/*` switches, `cmd/adcs/wheel_torque`, and
`cmd/adcs/mtq`.

Cross-couplings that make life interesting:
- solar generation scales with how well the +Z panel faces the sun, so
  attitude trouble is power trouble;
- ADCS actuator commands are honored only while the `adcs` load switch is
  powered — shed the ADCS and the satellite starts to drift;
- lithium-ion cells must not charge below 0 C (blocked charge power dumps
  as heat into the structure).
"""

from __future__ import annotations

import numpy as np

from cubesat_sim.environment.groundstation import GroundSite
from cubesat_sim.environment.magfield import dipole_field_eci
from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.kernel.component import Component
from cubesat_sim.physics.attitude import AttitudeDynamics
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
PANEL_NORMAL_BODY = np.array([0.0, 0.0, 1.0])

DEFAULT_STATION = GroundSite("bozeman", 45.68, -111.04)
DEFAULT_TARGETS = (
    GroundSite("tokyo", 35.68, 139.69),
    GroundSite("sao_paulo", -23.55, -46.63),
    GroundSite("reykjavik", 64.13, -21.90),
)
CONTACT_MIN_ELEV_DEG = 10.0   # station pass threshold
TARGET_MIN_ELEV_DEG = 25.0    # imaging look-angle limit
RADIO_TX_POWER_W = 2.0        # transmitting costs real watts
FRAME_DROP_PROB = 0.02        # RF packet loss inside a contact
DOWNLINK_RATE_MB_S = 0.25     # channel rate: sets data-frame airtime
TELEM_FRAME_AIRTIME_S = 0.5   # beacon airtime


class SpacecraftPhysics(Component):
    def __init__(
        self,
        orbit: CircularOrbit | None = None,
        array: SolarArray | None = None,
        battery: Battery | None = None,
        thermal: ThermalModel | None = None,
        attitude: AttitudeDynamics | None = None,
        loads: dict[str, float] | None = None,
        station: GroundSite | None = None,
        targets: tuple[GroundSite, ...] | None = None,
        initial_tumble_dps: float = 4.0,
        record_every_s: float = 5.0,
    ) -> None:
        super().__init__("physics", period=0.0)  # every tick
        self.orbit = orbit or CircularOrbit()
        self.array = array or SolarArray()
        self.battery = battery or Battery()
        self.thermal = thermal or ThermalModel()
        self.attitude = attitude or AttitudeDynamics()
        self.loads = dict(loads or DEFAULT_LOADS)
        # essentials and adcs start powered; payload and heater wait for commands
        self.switches = {n: (n in ESSENTIAL_LOADS or n == "adcs") for n in self.loads}
        self.station = station or DEFAULT_STATION
        self.targets = targets if targets is not None else DEFAULT_TARGETS
        self.initial_tumble_dps = initial_tumble_dps
        self._record_every_s = record_every_s
        self._prev_contact = False
        self._wheel_tau_cmd = np.zeros(3)
        self._mtq_m_cmd = np.zeros(3)
        self._gyro_bias: np.ndarray | None = None
        self._prev_eclipse: bool | None = None
        self._brownout = False
        self._charge_blocked = False

    def on_start(self) -> None:
        self.subscribe("cmd/loads/*")
        self.subscribe("cmd/adcs/*")
        self.subscribe("radio/tx")   # spacecraft transmitter into the channel
        self.subscribe("ground/tx")  # ground transmitter into the channel

    def _init_run_state(self) -> None:
        """Seed-dependent initial conditions, drawn once on the first step."""
        spin_axis = self.rng.normal(size=3)
        spin_axis /= np.linalg.norm(spin_axis)
        self.attitude.omega = np.deg2rad(self.initial_tumble_dps) * spin_axis
        self._gyro_bias = np.deg2rad(self.rng.normal(0.0, 0.05, size=3))

    def step(self, t: float, dt: float) -> None:
        if self._gyro_bias is None:
            self._init_run_state()

        # apply commands (essential loads cannot be switched off)
        downlink_frames: list[dict] = []
        uplink_frames: list[dict] = []
        for msg in self.drain():
            if msg.topic.startswith("cmd/loads/"):
                name = msg.topic.rsplit("/", 1)[-1]
                if name in self.loads and name not in ESSENTIAL_LOADS:
                    self.switches[name] = bool(msg.data.get("on"))
            elif msg.topic == "cmd/adcs/wheel_torque":
                self._wheel_tau_cmd = np.array(
                    [msg.data["x"], msg.data["y"], msg.data["z"]], dtype=float)
            elif msg.topic == "cmd/adcs/mtq":
                self._mtq_m_cmd = np.array(
                    [msg.data["x"], msg.data["y"], msg.data["z"]], dtype=float)
            elif msg.topic == "radio/tx":
                downlink_frames.append(msg.data)
            elif msg.topic == "ground/tx":
                uplink_frames.append(msg.data)

        # environment truth
        sun_hat = sun_direction_eci(self.clock.utc)
        r_eci = self.orbit.position_eci(t)
        eclipse = self.orbit.in_eclipse(r_eci, sun_hat)
        b_eci = dipole_field_eci(r_eci)
        if self._prev_eclipse is None:
            self._prev_eclipse = eclipse
        elif eclipse != self._prev_eclipse:
            self.event("eclipse_enter" if eclipse else "eclipse_exit")
            self._prev_eclipse = eclipse

        # attitude truth
        att = self.attitude
        b_body = att.body_from_eci(b_eci)
        nadir_body = att.body_from_eci(-r_eci / np.linalg.norm(r_eci))
        inertia = att.p.inertia_kg_m2
        n_orb = self.orbit.mean_motion_rad_s
        tau_gg = 3.0 * n_orb**2 * np.cross(nadir_body, inertia * nadir_body)
        tau_res = np.cross(att.p.residual_dipole_am2, b_body)
        tau_noise = self.rng.normal(0.0, att.p.dist_torque_std_nm, size=3)

        adcs_powered = self.switches["adcs"]
        wheel_cmd = self._wheel_tau_cmd if adcs_powered else np.zeros(3)
        mtq_cmd = self._mtq_m_cmd if adcs_powered else np.zeros(3)
        att.step(dt, tau_gg + tau_res + tau_noise, wheel_cmd, mtq_cmd, b_body)
        if not np.isfinite(att.omega).all() or not np.isfinite(att.q).all():
            raise RuntimeError(
                f"attitude state non-finite at t={t:.0f}s — control loop "
                "unstable for this timestep or corrupt actuator command")

        # ground geometry and the RF channel: the physics layer IS the link.
        # Frames only cross while the station sees the satellite, minus loss;
        # transmitting costs watts whether or not anyone hears you.
        when = self.clock.utc
        station_elev = self.station.elevation_deg(r_eci, when)
        contact = station_elev > CONTACT_MIN_ELEV_DEG
        if contact != self._prev_contact:
            self.event("contact_aos" if contact else "contact_los",
                       elevation_deg=station_elev)
            self._prev_contact = contact
        target_visible = any(
            site.elevation_deg(r_eci, when) > TARGET_MIN_ELEV_DEG
            for site in self.targets)
        # TX energy scales with airtime, not tick length — a beacon costs
        # the same joules at dt=1 and dt=5
        airtime_s = sum(
            (float(d.get("mb", 0.0)) / DOWNLINK_RATE_MB_S
             if d.get("kind") == "data" else TELEM_FRAME_AIRTIME_S)
            for d in downlink_frames)
        tx_w = RADIO_TX_POWER_W * min(dt, airtime_s) / dt
        if contact:
            for data in downlink_frames:
                if self.rng.random() >= FRAME_DROP_PROB:
                    self.publish("radio/rx_ground", **data)
            for data in uplink_frames:
                if self.rng.random() >= FRAME_DROP_PROB:
                    self.publish("radio/rx_space", **data)

        # electrical truth (attitude-coupled generation)
        panel_eci = att.eci_from_body(PANEL_NORMAL_BODY)
        facing = float(np.dot(panel_eci, sun_hat))
        p_gen = self.array.generation_w(eclipse, facing)
        if p_gen > 0.0:
            p_gen *= max(0.0, 1.0 + float(self.rng.normal(0.0, 0.02)))
        heater_w = self.loads["bat_heater"] if self.switches["bat_heater"] else 0.0
        p_load = sum(self.loads[n] for n, on in self.switches.items() if on) + tx_w
        p_net = p_gen - p_load

        # cold-charge inhibit: charging a frozen li-ion pack is forbidden;
        # blocked charge power is dumped as heat into the structure.
        # 50 mW threshold so generation noise around p_net = 0 doesn't
        # toggle the inhibit (and spam events) when gen ~= load
        charge_blocked = (
            p_net > 0.05 and self.thermal.battery.temp_k < MIN_CHARGE_TEMP_K
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

        gyro = att.omega + self._gyro_bias + self.rng.normal(0.0, np.deg2rad(0.01), 3)
        self.publish("sensors/adcs/gyro",
                     x=float(gyro[0]), y=float(gyro[1]), z=float(gyro[2]))
        mag = b_body + self.rng.normal(0.0, 1.0e-7, 3)
        self.publish("sensors/adcs/mag",
                     x=float(mag[0]), y=float(mag[1]), z=float(mag[2]))
        sun_body = att.body_from_eci(sun_hat) + self.rng.normal(0.0, 0.017, 3)
        if eclipse:
            self.publish("sensors/adcs/sun", x=0.0, y=0.0, z=0.0, valid=False)
        else:
            self.publish("sensors/adcs/sun",
                         x=float(sun_body[0]), y=float(sun_body[1]),
                         z=float(sun_body[2]), valid=True)
        h = att.h_wheel
        self.publish("sensors/adcs/wheel_momentum",
                     x=float(h[0]), y=float(h[1]), z=float(h[2]))
        self.publish("sensors/comms/carrier", detected=contact)
        self.publish("sensors/payload/target_visible", visible=target_visible)
        self.publish("sensors/payload/powered", on=self.switches["payload"])

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
            self.record("rate_dps", float(np.rad2deg(att.rate_rad_s)))
            self.record("sun_facing", facing)
            self.record("wheel_h_frac",
                        float(np.max(np.abs(h)) / att.p.wheel_h_max_nms))
            self.record("gs_contact", float(contact))
            self.record("target_visible", float(target_visible))
            self.record("tx_w", tx_w)
