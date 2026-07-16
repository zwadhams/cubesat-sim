"""Flight recorder: everything that happens in a run, queryable after.

Three streams go to SQLite — bus messages, per-component telemetry samples,
and discrete events (mode changes, faults, alarms). Combined with the seeded
RNG this is what turns "huh, weird" into a reproducible finding.
"""

from __future__ import annotations

import json
import sqlite3
from pathlib import Path
from typing import Any

from cubesat_sim.kernel.bus import Message

_SCHEMA = """
CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT);
CREATE TABLE IF NOT EXISTS messages (
    tick INTEGER, time REAL, seq INTEGER, topic TEXT, sender TEXT, data TEXT
);
CREATE TABLE IF NOT EXISTS telemetry (
    tick INTEGER, time REAL, source TEXT, key TEXT, value REAL
);
CREATE TABLE IF NOT EXISTS events (
    tick INTEGER, time REAL, source TEXT, kind TEXT, detail TEXT
);
CREATE INDEX IF NOT EXISTS idx_messages_topic ON messages(topic);
CREATE INDEX IF NOT EXISTS idx_telemetry_key ON telemetry(source, key);
"""


class FlightRecorder:
    def __init__(self, path: str | Path | None = None) -> None:
        self._conn = sqlite3.connect(str(path) if path is not None else ":memory:")
        self._conn.executescript(_SCHEMA)
        self._messages: list[tuple] = []
        self._telemetry: list[tuple] = []
        self._events: list[tuple] = []

    def set_meta(self, **kv: Any) -> None:
        self._conn.executemany(
            "INSERT OR REPLACE INTO meta (key, value) VALUES (?, ?)",
            [(k, json.dumps(v, default=str)) for k, v in kv.items()],
        )
        self._conn.commit()

    def log_message(self, msg: Message) -> None:
        self._messages.append(
            (msg.tick, msg.time, msg.seq, msg.topic, msg.sender,
             json.dumps(msg.data, default=str))
        )

    def log_telemetry(self, tick: int, time: float, source: str, key: str, value: float) -> None:
        self._telemetry.append((tick, time, source, key, float(value)))

    def log_event(self, tick: int, time: float, source: str, kind: str,
                  detail: dict[str, Any] | None = None) -> None:
        self._events.append(
            (tick, time, source, kind, json.dumps(detail or {}, default=str))
        )

    def flush(self) -> None:
        if self._messages:
            self._conn.executemany(
                "INSERT INTO messages VALUES (?, ?, ?, ?, ?, ?)", self._messages)
            self._messages = []
        if self._telemetry:
            self._conn.executemany(
                "INSERT INTO telemetry VALUES (?, ?, ?, ?, ?)", self._telemetry)
            self._telemetry = []
        if self._events:
            self._conn.executemany(
                "INSERT INTO events VALUES (?, ?, ?, ?, ?)", self._events)
            self._events = []
        self._conn.commit()

    # -- query helpers ------------------------------------------------------

    def messages(self, topic: str | None = None) -> list[tuple]:
        self.flush()
        if topic is None:
            q = "SELECT tick, time, seq, topic, sender, data FROM messages ORDER BY seq"
            return self._conn.execute(q).fetchall()
        q = ("SELECT tick, time, seq, topic, sender, data FROM messages "
             "WHERE topic = ? ORDER BY seq")
        return self._conn.execute(q, (topic,)).fetchall()

    def telemetry(self, source: str | None = None, key: str | None = None) -> list[tuple]:
        self.flush()
        q = "SELECT tick, time, source, key, value FROM telemetry"
        clauses, args = [], []
        if source is not None:
            clauses.append("source = ?")
            args.append(source)
        if key is not None:
            clauses.append("key = ?")
            args.append(key)
        if clauses:
            q += " WHERE " + " AND ".join(clauses)
        return self._conn.execute(q + " ORDER BY tick", args).fetchall()

    def events(self, source: str | None = None) -> list[tuple]:
        self.flush()
        q = "SELECT tick, time, source, kind, detail FROM events"
        args: list = []
        if source is not None:
            q += " WHERE source = ?"
            args.append(source)
        return self._conn.execute(q + " ORDER BY tick", args).fetchall()

    def close(self) -> None:
        self.flush()
        self._conn.close()
