"""Rigid-body attitude truth: quaternion kinematics, Euler dynamics,
reaction wheels, magnetorquers.

Conventions (scalar-first, aerospace standard — Markley & Crassidis):
the attitude quaternion q maps inertial (ECI) vectors into the body frame
via the DCM A(q):  v_body = A(q) @ v_eci.  Angular velocity is body-frame.

Wheels: the command convention is flight-natural — `wheel_tau_cmd` is the
torque the wheel assembly exerts ON THE BODY. Wheel momentum changes by the
reaction (-cmd). Torque and momentum are clamped per axis; a saturated
wheel delivers nothing further in the saturating direction.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def dcm_from_quat(q: np.ndarray) -> np.ndarray:
    """Body-from-inertial direction cosine matrix."""
    w, x, y, z = q
    return np.array([
        [w*w + x*x - y*y - z*z, 2*(x*y + w*z),         2*(x*z - w*y)],
        [2*(x*y - w*z),         w*w - x*x + y*y - z*z, 2*(y*z + w*x)],
        [2*(x*z + w*y),         2*(y*z - w*x),         w*w - x*x - y*y + z*z],
    ])


def quat_derivative(q: np.ndarray, omega_body: np.ndarray) -> np.ndarray:
    wx, wy, wz = omega_body
    big_omega = np.array([
        [0.0, -wx, -wy, -wz],
        [wx,  0.0,  wz, -wy],
        [wy, -wz,  0.0,  wx],
        [wz,  wy, -wx,  0.0],
    ])
    return 0.5 * big_omega @ q


@dataclass
class AttitudeParams:
    inertia_kg_m2: np.ndarray = field(
        default_factory=lambda: np.array([0.025, 0.05, 0.05]))  # 3U diagonal
    wheel_h_max_nms: float = 0.010     # per-axis momentum storage
    wheel_tau_max_nm: float = 1.0e-3   # per-axis torque
    mtq_m_max_am2: float = 0.2         # per-axis magnetic moment
    residual_dipole_am2: np.ndarray = field(
        default_factory=lambda: np.array([0.002, 0.0, 0.001]))  # build residual
    dist_torque_std_nm: float = 2.0e-7  # unmodeled disturbance noise


class AttitudeDynamics:
    def __init__(self, params: AttitudeParams | None = None) -> None:
        self.p = params or AttitudeParams()
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.omega = np.zeros(3)          # body rates, rad/s
        self.h_wheel = np.zeros(3)        # wheel momenta, N m s

    @property
    def rate_rad_s(self) -> float:
        return float(np.linalg.norm(self.omega))

    def body_from_eci(self, v_eci: np.ndarray) -> np.ndarray:
        return dcm_from_quat(self.q) @ v_eci

    def eci_from_body(self, v_body: np.ndarray) -> np.ndarray:
        return dcm_from_quat(self.q).T @ v_body

    def step(
        self,
        dt: float,
        tau_ext_body: np.ndarray,
        wheel_tau_cmd: np.ndarray,
        mtq_m_cmd: np.ndarray,
        b_body: np.ndarray,
    ) -> None:
        # RK4 substeps capped at 2.5 s: explicit Euler slowly pumps energy
        # into free tumbling (it blew up an uncontrolled satellite after
        # ~18 simulated hours); RK4 is slightly dissipative and holds for days
        n_sub = max(1, -(-round(dt * 10) // 25))  # ceil(dt / 2.5) on ticks
        sub_dt = dt / n_sub
        for _ in range(n_sub):
            self._substep(sub_dt, tau_ext_body, wheel_tau_cmd, mtq_m_cmd, b_body)

    def _substep(
        self,
        dt: float,
        tau_ext_body: np.ndarray,
        wheel_tau_cmd: np.ndarray,
        mtq_m_cmd: np.ndarray,
        b_body: np.ndarray,
    ) -> None:
        p = self.p
        inertia = p.inertia_kg_m2

        # actuator limits; tau_w is torque delivered to the body,
        # wheel momentum takes the reaction: dh/dt = -tau_w
        tau_w = np.clip(wheel_tau_cmd, -p.wheel_tau_max_nm, p.wheel_tau_max_nm)
        # a wheel at its momentum limit can't push h further outward
        saturating = (np.abs(self.h_wheel) >= p.wheel_h_max_nms) & \
                     (np.sign(-tau_w) == np.sign(self.h_wheel))
        tau_w = np.where(saturating, 0.0, tau_w)

        m_mtq = np.clip(mtq_m_cmd, -p.mtq_m_max_am2, p.mtq_m_max_am2)
        tau_body = tau_ext_body + np.cross(m_mtq, b_body) + tau_w

        # RK4 on (q, omega); torques held constant over the substep, wheel
        # momentum ramps linearly (dh/dt = -tau_w), which each stage sees at
        # its own stage time — keeps momentum exchange 4th-order consistent
        h_wheel = self.h_wheel

        def deriv(q, omega, stage_t):
            h_total = inertia * omega + h_wheel - tau_w * stage_t
            omega_dot = (tau_body - np.cross(omega, h_total)) / inertia
            return quat_derivative(q, omega), omega_dot

        q0, w0 = self.q, self.omega
        k1q, k1w = deriv(q0, w0, 0.0)
        k2q, k2w = deriv(q0 + 0.5 * dt * k1q, w0 + 0.5 * dt * k1w, 0.5 * dt)
        k3q, k3w = deriv(q0 + 0.5 * dt * k2q, w0 + 0.5 * dt * k2w, 0.5 * dt)
        k4q, k4w = deriv(q0 + dt * k3q, w0 + dt * k3w, dt)
        self.q = q0 + dt / 6.0 * (k1q + 2 * k2q + 2 * k3q + k4q)
        self.q = self.q / np.linalg.norm(self.q)
        self.omega = w0 + dt / 6.0 * (k1w + 2 * k2w + 2 * k3w + k4w)

        self.h_wheel = np.clip(h_wheel - tau_w * dt,
                               -p.wheel_h_max_nms, p.wheel_h_max_nms)

    def angular_momentum_eci(self) -> np.ndarray:
        """Total angular momentum in ECI — conserved when no external torque."""
        h_body = self.p.inertia_kg_m2 * self.omega + self.h_wheel
        return self.eci_from_body(h_body)
