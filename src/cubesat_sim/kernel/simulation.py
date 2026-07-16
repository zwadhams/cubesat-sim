"""The simulation kernel: wires clock, bus, recorder, and components together.

Tick order is fixed and deterministic:

1. dispatch the bus (deliver everything published during the previous tick)
2. step every component that is due this tick, in registration order
3. advance the clock

Given the same (seed, dt, components), two runs produce identical logs.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from cubesat_sim.kernel.bus import MessageBus
from cubesat_sim.kernel.clock import SimClock
from cubesat_sim.kernel.component import Component
from cubesat_sim.kernel.recorder import FlightRecorder

_FLUSH_EVERY_TICKS = 256


class Simulation:
    def __init__(
        self,
        dt: float = 1.0,
        seed: int = 0,
        recorder_path: str | Path | None = None,
        epoch: datetime | None = None,
    ) -> None:
        self.dt = dt
        self.seed = seed
        self.clock = SimClock(dt=dt) if epoch is None else SimClock(dt=dt, epoch=epoch)
        self.recorder = FlightRecorder(recorder_path)
        self.recorder.set_meta(seed=seed, dt=dt, epoch=self.clock.epoch.isoformat())
        self.bus = MessageBus(self.clock, self.recorder)
        self.components: list[Component] = []

    def add(self, component: Component) -> Component:
        if any(c.name == component.name for c in self.components):
            raise ValueError(f"duplicate component name: {component.name!r}")
        self.components.append(component)
        component._attach(self)
        return component

    def run(self, duration: float | None = None, ticks: int | None = None) -> None:
        """Run for `duration` simulated seconds or an exact number of ticks."""
        if (duration is None) == (ticks is None):
            raise ValueError("pass exactly one of duration= or ticks=")
        n = ticks if ticks is not None else max(1, round(duration / self.dt))
        for _ in range(n):
            self.bus.dispatch()
            tick = self.clock.tick
            for comp in self.components:
                if comp.due(tick):
                    comp.step(self.clock.time, comp.step_dt)
            self.clock.advance()
            if self.clock.tick % _FLUSH_EVERY_TICKS == 0:
                self.recorder.flush()
        self.recorder.flush()

    def close(self) -> None:
        self.recorder.close()
