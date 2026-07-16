"""Fault injection: the environment actor for entropy.

The `FaultInjector` runs alongside the physics component and publishes
`fault/*` messages that the physics layer honors — faults are ground truth,
so they live on the truth side of the sensor boundary. Two sources:

- a deterministic *schedule* of `ScheduledFault`s (the campaign script), and
- random single-event upsets (SEUs), a Poisson process whose rate is
  multiplied while the orbit ground track crosses the South Atlantic
  Anomaly — the same seed always produces the same storm.

Fault kinds physics understands:

- ``fault/sensor_stuck {sensor, stuck, hard}`` — sensor output latches at
  its next reading. Soft latch-ups clear when the ADCS rail power-cycles;
  hard faults are forever.
- ``fault/seu {sensor}`` — one-shot bit flip in the next reading's word.
- ``fault/wheel_friction {nm_per_nms}`` — bearing drag steps to a new value.
- ``fault/array_hit {mult}`` — debris/micrometeorite strike scales array
  output permanently.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

import numpy as np

from cubesat_sim.environment.groundstation import gmst_rad
from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.kernel.component import Component

SEU_TARGETS = ("gyro", "mag", "sun", "battery_voltage")

# rough box around the South Atlantic Anomaly (geodetic degrees)
SAA_LAT_DEG = (-50.0, 0.0)
SAA_LON_DEG = (-90.0, 40.0)


def flip_bit(value: float, bit: int) -> float:
    """Flip one bit of the IEEE 754 double representation."""
    bits = struct.unpack("<Q", struct.pack("<d", value))[0] ^ (1 << bit)
    return struct.unpack("<d", struct.pack("<Q", bits))[0]


def seu_upset(value: float, rng: np.random.Generator) -> float:
    """A radiation hit on one memory word: flip a random bit. Falls back to
    a sign flip when the chosen bit would produce inf/NaN — a non-finite
    value on the bus is a sim-integrity violation, not a fault model."""
    out = flip_bit(value, int(rng.integers(0, 64)))
    if not math.isfinite(out):
        out = flip_bit(value, 63)
    return out


@dataclass(frozen=True)
class ScheduledFault:
    at_s: float
    topic: str
    data: dict = field(default_factory=dict)


# -- readable schedule constructors ------------------------------------------

def sensor_stuck(at_s: float, sensor: str = "gyro", hard: bool = False) -> ScheduledFault:
    return ScheduledFault(at_s, "fault/sensor_stuck",
                          {"sensor": sensor, "stuck": True, "hard": hard})


def sensor_unstuck(at_s: float, sensor: str) -> ScheduledFault:
    return ScheduledFault(at_s, "fault/sensor_stuck", {"sensor": sensor, "stuck": False})


def seu(at_s: float, sensor: str) -> ScheduledFault:
    return ScheduledFault(at_s, "fault/seu", {"sensor": sensor})


def wheel_friction(at_s: float, nm_per_nms: float) -> ScheduledFault:
    return ScheduledFault(at_s, "fault/wheel_friction", {"nm_per_nms": nm_per_nms})


def array_hit(at_s: float, mult: float) -> ScheduledFault:
    return ScheduledFault(at_s, "fault/array_hit", {"mult": mult})


def channel_ber(at_s: float, mult: float) -> ScheduledFault:
    """Scintillation / interference: scale the RF channel's bit error rate."""
    return ScheduledFault(at_s, "fault/channel", {"ber_mult": mult})


class FaultInjector(Component):
    def __init__(
        self,
        schedule: tuple[ScheduledFault, ...] | list[ScheduledFault] = (),
        seu_rate_per_day: float = 0.0,
        saa_multiplier: float = 25.0,
        orbit: CircularOrbit | None = None,
        seu_targets: tuple[str, ...] = SEU_TARGETS,
    ) -> None:
        super().__init__("faults", period=1.0)
        self.schedule = sorted(schedule, key=lambda f: f.at_s)
        self.seu_rate_per_day = seu_rate_per_day
        self.saa_multiplier = saa_multiplier
        self.orbit = orbit or CircularOrbit()
        self.seu_targets = seu_targets
        self._next = 0

    def _in_saa(self, t: float) -> bool:
        r = self.orbit.position_eci(t)
        lat = math.degrees(math.asin(r[2] / float(np.linalg.norm(r))))
        lon = math.degrees(math.atan2(r[1], r[0]) - gmst_rad(self.clock.utc))
        lon = (lon + 180.0) % 360.0 - 180.0
        return (SAA_LAT_DEG[0] <= lat <= SAA_LAT_DEG[1]
                and SAA_LON_DEG[0] <= lon <= SAA_LON_DEG[1])

    def step(self, t: float, dt: float) -> None:
        while self._next < len(self.schedule) and self.schedule[self._next].at_s <= t:
            fault = self.schedule[self._next]
            self._next += 1
            self.publish(fault.topic, **fault.data)
            self.event("inject", topic=fault.topic, **fault.data)

        if self.seu_rate_per_day > 0.0:
            in_saa = self._in_saa(t)
            self.record("in_saa", float(in_saa))
            rate_s = self.seu_rate_per_day / 86400.0
            if in_saa:
                rate_s *= self.saa_multiplier
            if self.rng.random() < rate_s * dt:
                target = self.seu_targets[int(self.rng.integers(len(self.seu_targets)))]
                self.publish("fault/seu", sensor=target)
                self.event("inject_seu", sensor=target, in_saa=in_saa)
