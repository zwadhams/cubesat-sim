"""Fixed-timestep simulation clock."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone

DEFAULT_EPOCH = datetime(2026, 1, 1, tzinfo=timezone.utc)


@dataclass
class SimClock:
    """Discrete simulation clock. One tick = `dt` seconds of simulated time."""

    dt: float
    epoch: datetime = DEFAULT_EPOCH
    tick: int = 0

    @property
    def time(self) -> float:
        """Simulated seconds since epoch."""
        return self.tick * self.dt

    @property
    def utc(self) -> datetime:
        """Simulated wall-clock time (needed later for orbit geometry)."""
        return self.epoch + timedelta(seconds=self.time)

    def advance(self) -> None:
        self.tick += 1
