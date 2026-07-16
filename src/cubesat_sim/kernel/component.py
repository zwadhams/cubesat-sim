"""Base class for everything that lives on the bus.

A component declares how often it wants to run (`period`, in simulated
seconds); the kernel steps it at that rate. Components see the world through
their inbox and their sensors, and act by publishing — never by calling each
other.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from cubesat_sim.kernel.bus import Message
from cubesat_sim.kernel.rng import stream

if TYPE_CHECKING:
    import numpy as np

    from cubesat_sim.kernel.simulation import Simulation


class Component:
    def __init__(self, name: str, period: float) -> None:
        self.name = name
        self.period = period
        self.inbox: list[Message] = []
        self.rng: np.random.Generator | None = None
        self._sim: Simulation | None = None
        self._every_ticks = 1

    # -- wiring (called by Simulation.add) ----------------------------------

    def _attach(self, sim: Simulation) -> None:
        self._sim = sim
        self._every_ticks = max(1, round(self.period / sim.clock.dt))
        self.rng = stream(sim.seed, self.name)
        self.on_start()

    def due(self, tick: int) -> bool:
        return tick % self._every_ticks == 0

    @property
    def clock(self):
        return self._sim.clock

    @property
    def step_dt(self) -> float:
        """Simulated seconds elapsed between two of this component's steps."""
        return self._every_ticks * self._sim.clock.dt

    # -- I/O -----------------------------------------------------------------

    def subscribe(self, pattern: str) -> None:
        """Route matching bus traffic into this component's inbox."""
        self._sim.bus.subscribe(pattern, self._receive)

    def _receive(self, msg: Message) -> None:
        self.inbox.append(msg)

    def publish(self, topic: str, **data: Any) -> Message:
        return self._sim.bus.publish(topic, self.name, data)

    def drain(self) -> list[Message]:
        """Take and clear the inbox."""
        msgs = list(self.inbox)
        self.inbox.clear()
        return msgs

    def record(self, key: str, value: float) -> None:
        clock = self._sim.clock
        self._sim.recorder.log_telemetry(clock.tick, clock.time, self.name, key, value)

    def event(self, kind: str, **detail: Any) -> None:
        clock = self._sim.clock
        self._sim.recorder.log_event(clock.tick, clock.time, self.name, kind, detail)

    # -- lifecycle hooks ------------------------------------------------------

    def on_start(self) -> None:
        """Called once when added to the simulation. Subscribe here."""

    def step(self, t: float, dt: float) -> None:
        """Called every `period` simulated seconds. `t` is current sim time,
        `dt` the time elapsed since this component's previous step."""
        raise NotImplementedError
