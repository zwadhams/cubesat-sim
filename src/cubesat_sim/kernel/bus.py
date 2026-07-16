"""Pub/sub message bus standing in for the spacecraft data bus.

Delivery semantics: messages published during tick N are delivered at the
start of tick N+1 (the next `dispatch()`). The one-tick latency is deliberate
— acting on slightly stale data is one of the ingredients of emergent
behavior, and it matches how a real polled bus feels to subsystems.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from cubesat_sim.kernel.clock import SimClock

Handler = Callable[["Message"], None]


@dataclass(frozen=True)
class Message:
    topic: str
    sender: str
    data: dict[str, Any]
    tick: int
    time: float
    seq: int


class MessageBus:
    def __init__(self, clock: SimClock, recorder=None) -> None:
        self._clock = clock
        self._recorder = recorder
        self._subs: list[tuple[str, Handler]] = []
        self._pending: list[Message] = []
        self._seq = 0

    def subscribe(self, pattern: str, handler: Handler) -> None:
        """Register a handler for a topic. Patterns are an exact topic,
        a prefix wildcard like ``"eps/*"``, or ``"*"`` for everything."""
        self._subs.append((pattern, handler))

    def publish(self, topic: str, sender: str, data: dict[str, Any]) -> Message:
        msg = Message(
            topic=topic,
            sender=sender,
            data=dict(data),
            tick=self._clock.tick,
            time=self._clock.time,
            seq=self._seq,
        )
        self._seq += 1
        self._pending.append(msg)
        if self._recorder is not None:
            self._recorder.log_message(msg)
        return msg

    @staticmethod
    def matches(pattern: str, topic: str) -> bool:
        if pattern == "*":
            return True
        if pattern.endswith("/*"):
            return topic.startswith(pattern[:-1])
        return topic == pattern

    def dispatch(self) -> int:
        """Deliver everything published since the last dispatch, in publish
        order. Returns the number of messages delivered."""
        batch, self._pending = self._pending, []
        for msg in batch:
            for pattern, handler in self._subs:
                if self.matches(pattern, msg.topic):
                    handler(msg)
        return len(batch)
