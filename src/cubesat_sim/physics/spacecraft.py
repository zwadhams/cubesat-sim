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

Faults are ground truth too: `fault/*` messages (from the FaultInjector)
latch sensors, flip bits in sensor words, step wheel-bearing friction, and
scar the solar array. A *soft* sensor latch-up clears when the ADCS rail
power-cycles (off -> on); *hard* faults are forever. Degradation (battery
fade, array darkening, friction growth) runs continuously inside the
component models themselves.
"""

from __future__ import annotations

import math

import numpy as np

from cubesat_sim.ccsds import VC1_FRAME_BITS

from cubesat_sim.environment.groundstation import GroundSite
from cubesat_sim.environment.magfield import dipole_field_eci
from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.faults import seu_upset
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
DEFAULT_STATIONS = (DEFAULT_STATION,)   # single-station default (unchanged).
# A ground station near the Tokyo imaging AOI: captured science can be dumped in
# the same pass it was collected, instead of waiting for a Bozeman revisit the
# ground-track drift may not provide for many orbits. Add stations to build_sim
# (or DEFAULT_STATIONS) to close the coverage gaps a lone station leaves.
TOKYO_STATION = GroundSite("tokyo_gs", 35.68, 139.69)
DEFAULT_TARGETS = (
    GroundSite("tokyo", 35.68, 139.69),
    GroundSite("sao_paulo", -23.55, -46.63),
    GroundSite("reykjavik", 64.13, -21.90),
)
CONTACT_MIN_ELEV_DEG = 10.0   # station pass threshold
TARGET_MIN_ELEV_DEG = 25.0    # imaging look-angle limit
RADIO_TX_POWER_W = 2.0        # transmitting costs real watts
DOWNLINK_RATE_MB_S = 0.25     # channel rate for VC1 burst airtime
LINK_RATE_BPS = 2_000_000     # the same rate, in bits, for byte-true frames
BURST_OVERHEAD_S = 0.02       # preamble/ramp-up per transmitted burst
# bit error rate vs elevation: slant range (so SNR) is worst at the horizon.
# log-linear in normalized sin(elevation) between these anchors:
BER_AT_MIN_ELEV = 1e-4        # at the 10 deg contact threshold
BER_AT_ZENITH = 1e-6


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
        stations: tuple[GroundSite, ...] | list[GroundSite] | None = None,
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
        if stations is not None:
            self.stations = list(stations)
        elif station is not None:
            self.stations = [station]
        else:
            self.stations = list(DEFAULT_STATIONS)
        self.station = self.stations[0]   # primary, for any legacy reference
        self.targets = targets if targets is not None else DEFAULT_TARGETS
        self.initial_tumble_dps = initial_tumble_dps
        self._record_every_s = record_every_s
        self._prev_contact = False
        self._active_station: str | None = None
        self._wheel_tau_cmd = np.zeros(3)
        self._mtq_m_cmd = np.zeros(3)
        self._gyro_bias: np.ndarray | None = None
        self._prev_eclipse: bool | None = None
        self._brownout = False
        self._charge_blocked = False
        self._stuck: dict[str, str] = {}          # sensor -> "soft" | "hard"
        self._stuck_vals: dict[str, tuple] = {}   # latched output words
        self._seu_pending: list[str] = []         # sensors owed a bit flip
        self._ber_mult = 1.0                      # fault/channel scintillation
        self._last_ber = 0.0

    def on_start(self) -> None:
        self.subscribe("cmd/loads/*")
        self.subscribe("cmd/adcs/*")
        self.subscribe("radio/tx")   # spacecraft transmitter into the channel
        self.subscribe("ground/tx")  # ground transmitter into the channel
        self.subscribe("fault/*")    # injected hardware faults are truth

    def _clear_soft_latchups(self) -> None:
        """An ADCS rail power cycle clears latched (soft-stuck) sensors."""
        for sensor, kind in list(self._stuck.items()):
            if kind == "soft":
                del self._stuck[sensor]
                self._stuck_vals.pop(sensor, None)
                self.event("latchup_cleared", sensor=sensor)

    def _sensor_out(self, sensor: str, values: tuple) -> tuple:
        """Apply latched-stuck and pending-SEU faults to a sensor reading."""
        if sensor in self._stuck:
            values = self._stuck_vals.setdefault(sensor, values)
        if sensor in self._seu_pending:
            self._seu_pending.remove(sensor)
            vals = list(values)
            # the sun sensor's valid flag is not a corruptible float word
            n_words = 3 if sensor == "sun" else len(vals)
            idx = int(self.rng.integers(n_words))
            vals[idx] = seu_upset(float(vals[idx]), self.rng)
            self.event("seu_corruption", sensor=sensor)
            values = tuple(vals)
        return values

    def _link_ber(self, elev_deg: float) -> float:
        """Bit error rate for the current pass geometry: log-linear in
        normalized sin(elevation) between the horizon and zenith anchors."""
        s_min = math.sin(math.radians(CONTACT_MIN_ELEV_DEG))
        s_el = math.sin(math.radians(max(elev_deg, CONTACT_MIN_ELEV_DEG)))
        x = (s_el - s_min) / (1.0 - s_min)
        log_ber = (math.log10(BER_AT_MIN_ELEV)
                   + x * (math.log10(BER_AT_ZENITH) - math.log10(BER_AT_MIN_ELEV)))
        return self._ber_mult * 10.0 ** log_ber

    def _corrupt_hex(self, hex_str: str, ber: float) -> str:
        """Pass a byte-true frame through the noisy channel: draw the
        number of bit errors, flip those bits. The CRC finds them later."""
        data = bytearray.fromhex(hex_str)
        n_bits = len(data) * 8
        n_err = int(self.rng.binomial(n_bits, ber))
        if n_err:
            for pos in self.rng.choice(n_bits, size=min(n_err, n_bits),
                                       replace=False):
                data[pos // 8] ^= 1 << (7 - (pos % 8))
        return data.hex()

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
                    on = bool(msg.data.get("on"))
                    if name == "adcs" and on and not self.switches["adcs"]:
                        self._clear_soft_latchups()
                    self.switches[name] = on
            elif msg.topic == "fault/sensor_stuck":
                sensor = msg.data["sensor"]
                if msg.data.get("stuck", True):
                    self._stuck[sensor] = "hard" if msg.data.get("hard") else "soft"
                else:
                    self._stuck.pop(sensor, None)
                    self._stuck_vals.pop(sensor, None)
            elif msg.topic == "fault/seu":
                self._seu_pending.append(msg.data["sensor"])
            elif msg.topic == "fault/wheel_friction":
                self.attitude.p.wheel_friction_nm_per_nms = float(msg.data["nm_per_nms"])
            elif msg.topic == "fault/array_hit":
                self.array.illumination *= float(msg.data["mult"])
            elif msg.topic == "fault/channel":
                self._ber_mult = float(msg.data.get("ber_mult", 1.0))
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
        # the satellite works whichever station gives the best geometry right
        # now (several antennas, one ops center); that station's elevation
        # drives the channel. A change of active station while staying in
        # contact is a handover.
        active = None
        station_elev = 0.0
        for st in self.stations:
            el = st.elevation_deg(r_eci, when)
            if el > CONTACT_MIN_ELEV_DEG and el > station_elev:
                station_elev, active = el, st
        contact = active is not None
        active_name = active.name if active is not None else None
        if contact != self._prev_contact:
            self.event("contact_aos" if contact else "contact_los",
                       station=(active_name or self._active_station),
                       elevation_deg=station_elev)
            self._prev_contact = contact
        elif contact and active_name != self._active_station:
            self.event("contact_handover", station=active_name,
                       elevation_deg=station_elev)
        self._active_station = active_name
        target_visible = any(
            site.elevation_deg(r_eci, when) > TARGET_MIN_ELEV_DEG
            for site in self.targets)
        # TX energy scales with airtime, not tick length — a frame costs
        # the same joules at dt=1 and dt=5
        airtime_s = 0.0
        for d in downlink_frames:
            if d.get("kind") == "vc1_burst":
                airtime_s += float(d.get("mb", 0.0)) / DOWNLINK_RATE_MB_S
            else:
                airtime_s += (len(d.get("hex", "")) * 4.0 / LINK_RATE_BPS
                              + BURST_OVERHEAD_S)
        tx_w = RADIO_TX_POWER_W * min(dt, airtime_s) / dt

        # the channel itself: frames cross only during contact, picking up
        # bit errors along the way. Byte-true frames get real bit flips
        # (the ground's CRC does the rejecting); VC1 bursts get a binomial
        # draw of corrupted frames out of the burst.
        self._last_ber = self._link_ber(station_elev) if contact else 0.0
        if contact:
            ber = self._last_ber
            for d in downlink_frames:
                if d.get("kind") == "tm_frame":
                    self.publish("radio/rx_ground", kind="tm_frame",
                                 hex=self._corrupt_hex(d["hex"], ber))
                elif d.get("kind") == "vc1_burst":
                    n = int(d.get("frames", 0))
                    p_frame = 1.0 - (1.0 - ber) ** VC1_FRAME_BITS
                    bad = int(self.rng.binomial(n, p_frame)) if n > 0 else 0
                    self.publish("radio/rx_ground", kind="vc1_burst",
                                 frames=n, bad=bad,
                                 mb=float(d.get("mb", 0.0)),
                                 vcfc0=int(d.get("vcfc0", 0)))
                else:
                    self.publish("radio/rx_ground", **d)
            for d in uplink_frames:
                if "hex" in d:
                    self.publish("radio/rx_space", kind="tc_frame",
                                 hex=self._corrupt_hex(d["hex"], ber))
                else:
                    self.publish("radio/rx_space", **d)

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
        self.array.age(dt, in_sun=not eclipse)

        if self.battery.soc <= 0.0 and not self._brownout:
            self._brownout = True
            for name in self.switches:
                if name not in ESSENTIAL_LOADS:
                    self.switches[name] = False
            self.event("brownout")
            self.publish("physics/brownout")
        elif self._brownout and self.battery.soc > 0.05:
            self._brownout = False

        # noisy sensor readings — all any subsystem ever gets to see.
        # each reading passes through _sensor_out, where latched-stuck and
        # SEU faults corrupt the output word
        v_true = self.battery.voltage(0.0 if charge_blocked else p_net)
        (v_out,) = self._sensor_out(
            "battery_voltage", (v_true + float(self.rng.normal(0.0, 0.02)),))
        self.publish("sensors/eps/battery_voltage", volts=v_out)
        self.publish("sensors/eps/solar_power",
                     watts=max(0.0, p_gen + float(self.rng.normal(0.0, 0.05))))
        self.publish("sensors/eps/load_power",
                     watts=max(0.0, p_load + float(self.rng.normal(0.0, 0.05))))
        self.publish("sensors/thermal/battery_temp",
                     kelvin=self.thermal.battery.temp_k + float(self.rng.normal(0.0, 0.3)))
        self.publish("sensors/thermal/structure_temp",
                     kelvin=self.thermal.structure.temp_k + float(self.rng.normal(0.0, 0.3)))

        gyro = att.omega + self._gyro_bias + self.rng.normal(0.0, np.deg2rad(0.01), 3)
        gx, gy, gz = self._sensor_out(
            "gyro", (float(gyro[0]), float(gyro[1]), float(gyro[2])))
        self.publish("sensors/adcs/gyro", x=gx, y=gy, z=gz)
        mag = b_body + self.rng.normal(0.0, 1.0e-7, 3)
        mx, my, mz = self._sensor_out(
            "mag", (float(mag[0]), float(mag[1]), float(mag[2])))
        self.publish("sensors/adcs/mag", x=mx, y=my, z=mz)
        sun_body = att.body_from_eci(sun_hat) + self.rng.normal(0.0, 0.017, 3)
        if eclipse:
            sun_raw = (0.0, 0.0, 0.0, False)
        else:
            sun_raw = (float(sun_body[0]), float(sun_body[1]),
                       float(sun_body[2]), True)
        sx, sy, sz, sv = self._sensor_out("sun", sun_raw)
        self.publish("sensors/adcs/sun", x=sx, y=sy, z=sz, valid=bool(sv))
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
            # body-from-ECI attitude quaternion truth (scalar-first) — the
            # close-up view needs the real orientation, which isn't
            # recomputable from (seed, dt). Observing truth only; no RNG or
            # control effect, so determinism holds.
            self.record("q0", float(att.q[0]))
            self.record("q1", float(att.q[1]))
            self.record("q2", float(att.q[2]))
            self.record("q3", float(att.q[3]))
            self.record("wheel_h_frac",
                        float(np.max(np.abs(h)) / att.p.wheel_h_max_nms))
            self.record("gs_contact", float(contact))
            self.record("target_visible", float(target_visible))
            self.record("tx_w", tx_w)
            self.record("batt_capacity_wh", self.battery.capacity_wh)
            self.record("array_illum", self.array.illumination)
            if contact:
                self.record("link_ber", self._last_ber)
