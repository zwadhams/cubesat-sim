"""Bridge to flight software running as an external OS process.

Real flight software doesn't live in the simulator's address space — it runs
on its own computer, in its own language, talking over a bus. RemoteComponent
makes that literal: it launches a subsystem binary (C, C++, Rust, anything)
and runs a lockstep newline-delimited-JSON protocol over stdin/stdout:

  sim -> process   {"type":"init","name":...}
  process -> sim   {"type":"ready","subscribe":[<topic patterns>]}
  sim -> process   {"type":"step","t":...,"dt":...,"msgs":[{topic,sender,data}]}
  process -> sim   {"type":"out","pub":[{topic,data}],
                    "telemetry":{key:value}, "events":[{kind,detail}]}
  sim -> process   {"type":"shutdown"}

Lockstep (one request, one blocking reply per step) preserves the kernel's
determinism guarantee across the language boundary: the external process
just has to be deterministic itself — no wall clock, no unseeded RNG —
which is idiomatic flight software anyway.
"""

from __future__ import annotations

import json
import math
import subprocess
from typing import Any

from cubesat_sim.kernel.component import Component


def _poison(value: Any) -> bool:
    """True for values that must not touch the bus or recorder: JSON null
    (serde_json's laundering of NaN/inf) and non-finite numbers (Python's
    json.loads accepts the nonstandard Infinity/NaN tokens)."""
    if value is None:
        return True
    return isinstance(value, float) and not math.isfinite(value)


class RemoteComponent(Component):
    def __init__(self, name: str, period: float, argv: list[str]) -> None:
        super().__init__(name, period)
        self.argv = list(argv)
        self._proc: subprocess.Popen | None = None

    def on_start(self) -> None:
        self._proc = subprocess.Popen(
            self.argv,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
        )
        self._send({"type": "init", "name": self.name})
        ready = self._recv()
        if ready.get("type") != "ready":
            raise RuntimeError(
                f"{self.name}: flight software sent {ready!r} instead of ready")
        for pattern in ready.get("subscribe", []):
            self.subscribe(pattern)

    def step(self, t: float, dt: float) -> None:
        frame = {
            "type": "step",
            "t": t,
            "dt": dt,
            "msgs": [
                {"topic": m.topic, "sender": m.sender, "data": m.data}
                for m in self.drain()
            ],
        }
        self._send(frame)
        out = self._recv()
        # inbound quarantine: flight software can emit garbage (serde_json
        # launders non-finite floats into JSON null), but garbage must not
        # reach the bus or the recorder — reject it loudly instead
        for pub in out.get("pub", []):
            data = pub.get("data", {})
            if any(_poison(v) for v in data.values()):
                self.event("pub_reject", topic=pub.get("topic", "?"))
                continue
            self.publish(pub["topic"], **data)
        for key, value in out.get("telemetry", {}).items():
            if _poison(value):
                self.event("telemetry_reject", key=key)
                continue
            self.record(key, float(value))
        for ev in out.get("events", []):
            self.event(ev["kind"], **ev.get("detail", {}))

    def on_stop(self) -> None:
        if self._proc is None or self._proc.poll() is not None:
            return
        try:
            self._send({"type": "shutdown"})
            self._proc.wait(timeout=2.0)
        except Exception:
            self._proc.kill()

    # -- wire helpers --------------------------------------------------------

    def _send(self, obj: dict[str, Any]) -> None:
        try:
            # allow_nan=False: NaN/Infinity are not JSON; sending them once
            # deadlocked the lockstep when the peer rejected the frame.
            # A non-finite value here means the physics is sick — fail loud.
            payload = json.dumps(obj, separators=(",", ":"), allow_nan=False)
        except ValueError as exc:
            raise RuntimeError(
                f"{self.name}: refusing to send non-finite value in frame; "
                f"upstream state is corrupt: {exc}") from exc
        try:
            self._proc.stdin.write(payload + "\n")
            self._proc.stdin.flush()
        except (BrokenPipeError, OSError) as exc:
            raise RuntimeError(
                f"{self.name}: flight software process died "
                f"(exit code {self._proc.poll()})") from exc

    def _recv(self) -> dict[str, Any]:
        line = self._proc.stdout.readline()
        if not line:
            raise RuntimeError(
                f"{self.name}: flight software process exited "
                f"(exit code {self._proc.poll()})")
        return json.loads(line)
