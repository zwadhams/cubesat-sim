"""Live mission console: fly a mission in real time and watch it happen.

The simulator already streams everything it does — every bus message,
telemetry sample, and event — into the SQLite flight recording. Live mode
runs the mission one tick at a time, paced to wall clock, and a small
stdlib HTTP server tails the recording, pushing new rows to the browser
over Server-Sent Events. The page is an ops console: mission clock, orbit
globe, live telemetry lanes, the raw spacecraft bus, the decoded space
link, an event ticker — and, on live flights, a command panel: queue a
real TC through the ground station's ARQ, inject a hardware fault, or
publish a raw bus message mid-flight. Injections are applied between
ticks on the runner thread and recorded like all other traffic, so the
recording stays the complete, replayable record of the flight (though
(seed, dt) alone no longer reproduces a manually-commanded one). Replays
stay view-only.

Because the server only ever tails a recording, `--replay` can "fly" any
finished flight (a Monte Carlo campaign recording, say) at any speed.

Usage:
    python -m cubesat_sim.frontend.live --seed 19 --orbits 4 --seu-rate 6 --speed 60
    python -m cubesat_sim.frontend.live --replay runs/campaign1/flight_0019.db
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
import threading
import time as _time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

from cubesat_sim.frontend.dashboard import DIGITAL_TRACKS, ANALOG_LANES, EVENT_GLOSS, \
    GLOSSARY, _fmt_detail, _orbit_geometry, _severity, compute_annotations, \
    parse_catalog
from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.linkdump import describe_link_message
from cubesat_sim.mission import build_sim
from cubesat_sim.montecarlo import random_fault_campaign

_PERIOD = CircularOrbit().period_s
LANE_WINDOW_S = 1.5 * _PERIOD

# the rolling lanes worth watching live (subset of the report's lanes)
LIVE_LANE_TITLES = ("State of charge", "Electrical power", "Body rate",
                    "Temperatures", "Data")

# per-poll row caps so a late-joining client backfills over a few frames
# instead of one giant one
_LIMITS = {"messages": 3000, "telemetry": 12000, "events": 1000}
_POLL_S = 0.25


@dataclass
class LiveControl:
    """Shared state between the mission runner, /control, and SSE tailers."""
    speed: float = 30.0        # sim seconds per wall second; inf = unpaced
    paused: bool = False
    stop: bool = False
    sim_time: float = 0.0
    tick: int = 0
    done: bool = False
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _inject: list = field(default_factory=list, repr=False)

    def set(self, **kw) -> None:
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def queue_inject(self, topic: str, data: dict) -> bool:
        """Queue a bus publish for the runner thread; the bus and recorder
        are never touched from HTTP threads. False if the queue is full
        (a paused mission accumulating spam)."""
        with self._lock:
            if len(self._inject) >= 256:
                return False
            self._inject.append((topic, data))
            return True

    def take_injects(self) -> list:
        with self._lock:
            out, self._inject = self._inject, []
            return out

    def status(self) -> dict:
        with self._lock:
            return {"t": round(self.sim_time, 3), "tick": self.tick,
                    "paused": self.paused, "done": self.done,
                    "speed": None if math.isinf(self.speed) else self.speed}


def fly(sim, ctl: LiveControl, duration_s: float) -> None:
    """Run the mission one tick at a time, paced to wall clock. Each
    run(ticks=1) call flushes, so the tick's rows are committed and visible
    to the read-only tailer connections the moment it lands."""
    ticks = max(1, round(duration_s / sim.dt))
    try:
        for _ in range(ticks):
            while ctl.paused and not ctl.stop:
                _time.sleep(0.05)
            if ctl.stop:
                break
            for topic, data in ctl.take_injects():
                sim.bus.publish(topic, "console", data)
                kind = ("inject" if topic.startswith("fault/")
                        else "uplink" if topic == "ops/tc" else "publish")
                sim.recorder.log_event(sim.clock.tick, sim.clock.time,
                                       "console", kind,
                                       {"topic": topic, **data})
            sim.run(ticks=1)
            ctl.set(sim_time=sim.clock.time, tick=sim.clock.tick)
            # pace: chunked wait so pause and speed changes bite within ~100 ms
            deadline = _time.monotonic()
            while not ctl.stop and not ctl.paused:
                speed = ctl.speed
                if not math.isfinite(speed):
                    break
                wait = deadline + sim.dt / speed - _time.monotonic()
                if wait <= 0:
                    break
                _time.sleep(min(wait, 0.1))
    finally:
        # even a crashed runner must not leave the console reading "LIVE"
        ctl.set(done=True)


def pace_replay(ctl: LiveControl, t_end: float, dt: float) -> None:
    """Advance a virtual mission clock through a finished recording; the
    SSE tailers only serve rows up to the virtual now."""
    while not ctl.stop and ctl.sim_time < t_end:
        _time.sleep(0.05)
        if ctl.paused:
            continue
        speed = ctl.speed
        t = t_end if math.isinf(speed) else min(t_end, ctl.sim_time + 0.05 * speed)
        ctl.set(sim_time=t, tick=int(t / dt))
    ctl.set(done=True)


def _lane_specs() -> list[dict]:
    out = []
    for title, _section, hint, unit, domain, series in ANALOG_LANES:
        if title not in LIVE_LANE_TITLES:
            continue
        out.append({
            "title": title, "hint": hint, "unit": unit,
            "domain": list(domain) if domain else None,
            "series": [{"source": src, "key": key, "label": label,
                        "tf": tf_name}
                       for src, key, label, (tf_name, _fn) in series],
        })
    return out


def _boot_common(title: str, mode: str, meta: dict) -> dict:
    return {
        "title": title, "mode": mode, "meta": meta,
        "orbit3d": _orbit_geometry(meta["epoch"]),
        "lanes": _lane_specs(),
        "pills": [{"source": s, "key": k, "label": lb}
                  for s, k, lb in DIGITAL_TRACKS],
        "gloss": GLOSSARY, "evgloss": EVENT_GLOSS, "catalog": parse_catalog(),
    }


def _live_boot(sim, duration_s: float, db_path: Path) -> dict:
    return _boot_common(db_path.stem, "live", {
        "seed": sim.seed, "dt": sim.dt,
        "epoch": sim.clock.epoch.isoformat(),
        "duration_s": round(duration_s, 2), "period_s": round(_PERIOD, 2)})


def _replay_boot(db_path: Path) -> tuple[dict, float, float]:
    db = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    try:
        meta = dict(db.execute("SELECT key, value FROM meta").fetchall())
        t_end = db.execute("SELECT MAX(time) FROM telemetry").fetchone()[0] or 0.0
    finally:
        db.close()
    dt = float(json.loads(meta.get("dt", "1.0")))
    boot = _boot_common(db_path.stem, "replay", {
        "seed": json.loads(meta.get("seed", "0")), "dt": dt,
        "epoch": json.loads(meta["epoch"]) if "epoch" in meta else None,
        "duration_s": round(t_end, 2), "period_s": round(_PERIOD, 2)})
    return boot, t_end, dt


def _payload_ok(v) -> bool:
    """True when an injected value is safe for the bus: no None (the bridge
    quarantines JSON null) and no non-finite floats (the bridge refuses to
    serialize them). Currently DISARMED by the owner's choice — see the
    commented call site in _inject_request. Kept here so the door can be
    re-armed by uncommenting one check."""
    if isinstance(v, (bool, str)):
        return True
    if isinstance(v, (int, float)):
        return math.isfinite(v)
    if isinstance(v, dict):
        return all(isinstance(k, str) and _payload_ok(x) for k, x in v.items())
    if isinstance(v, list):
        return all(_payload_ok(x) for x in v)
    return False


class _Handler(BaseHTTPRequestHandler):
    # injected by Console via a per-instance subclass:
    ctl: LiveControl
    db_path: str
    page: bytes
    live: bool  # False in replay mode: the console is view-only there

    def log_message(self, fmt, *args):  # keep the terminal quiet
        pass

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/?"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(self.page)))
            self.end_headers()
            self.wfile.write(self.page)
        elif self.path.startswith("/events"):
            self._stream()
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path != "/control":
            self.send_error(404)
            return
        n = int(self.headers.get("Content-Length") or 0)
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            self.send_error(400)
            return
        action = body.get("action")
        if action == "pause":
            self.ctl.set(paused=True)
        elif action == "resume":
            self.ctl.set(paused=False)
        elif action == "speed":
            try:
                v = float(body.get("value", 30.0))
            except (TypeError, ValueError):
                self.send_error(400)
                return
            if math.isfinite(v) and v > 0:
                self.ctl.set(speed=min(max(v, 0.1), 10000.0))
        elif action in ("tc", "inject"):
            if not self._inject_request(action, body):
                return
        else:
            self.send_error(400)
            return
        self.send_response(204)
        self.end_headers()

    def _inject_request(self, action: str, body: dict) -> bool:
        """Validate and queue a command-panel request; on failure send the
        error response and return False."""
        if not self.live or self.ctl.status()["done"]:
            self.send_error(409, "mission is not flying")
            return False
        if action == "tc":
            try:
                cmd, arg = int(body["cmd"]), int(body["arg"])
            except (KeyError, TypeError, ValueError):
                self.send_error(400)
                return False
            if not (0 <= cmd <= 255 and 0 <= arg <= 255):
                self.send_error(400)
                return False
            topic, data = "ops/tc", {"cmd": cmd, "arg": arg}
        else:
            topic, data = body.get("topic"), body.get("data", {})
            if (not isinstance(topic, str) or not topic or len(topic) > 128
                    or any(c.isspace() for c in topic)
                    or not isinstance(data, dict)):
                self.send_error(400)
                return False
            # Owner's call (2026-07-17): poison payloads (JSON null,
            # non-finite floats) go through on purpose — what they do to
            # the bus, the bridge, and the recorder is emergent behavior
            # worth having on tap, and the deeper tripwires (allow_nan
            # refusal, inbound quarantine, frame_reject) are the show.
            # Re-arm the door by restoring:
            #     if not _payload_ok(data):
            #         self.send_error(400)
            #         return False
        if not self.ctl.queue_inject(topic, data):
            self.send_error(409, "inject queue full")
            return False
        return True

    # -- the tail ------------------------------------------------------------

    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        db = sqlite3.connect(f"file:{self.db_path}?mode=ro", uri=True)
        db.execute("PRAGMA busy_timeout=2000")
        try:
            self.wfile.write(b"retry: 1500\n\n")
            now = self.ctl.status()["t"]
            # a late joiner backfills one lane window of telemetry, a minute
            # of bus traffic (the topic table repopulates itself anyway),
            # and the whole event history (small)
            cursors = {
                "messages": self._floor(db, "messages", now - 60.0),
                "telemetry": self._floor(db, "telemetry", now - LANE_WINDOW_S),
                "events": 0,
            }
            findings_state = {"last": None, "at": 0.0}
            while not self.ctl.stop:
                frame, backlog = self._collect(db, cursors)
                self._maybe_findings(db, frame, findings_state)
                blob = json.dumps(frame, separators=(",", ":")).encode()
                self.wfile.write(b"data: " + blob + b"\n\n")
                self.wfile.flush()
                if not backlog:
                    _time.sleep(_POLL_S)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass  # client went away
        finally:
            db.close()

    def _maybe_findings(self, db, frame, state) -> None:
        """Recompute the catalog-signature findings against the recording
        up to the current mission time, throttled to a few seconds so a
        long flight doesn't rescan every poll. Only emitted when changed;
        the client replaces its card wholesale. The detectors treat 'now'
        as the end of flight, so a finding may appear, refine, or (rarely)
        withdraw as more of the story lands — honest for a live feed."""
        wall = _time.monotonic()
        if state["last"] is not None and wall - state["at"] < 3.0:
            return
        state["at"] = wall
        now = self.ctl.status()["t"]
        try:
            notes = compute_annotations(db, now + 1e-9, _PERIOD)
        except sqlite3.Error:
            return  # a mid-write read lost the race; try again next tick
        key = json.dumps(notes, separators=(",", ":"))
        if key != state["last"]:
            state["last"] = key
            frame["findings"] = notes

    @staticmethod
    def _floor(db, table: str, t: float) -> int:
        row = db.execute(
            f"SELECT COALESCE(MAX(rowid), 0) FROM {table} WHERE time < ?",
            (max(t, 0.0),)).fetchone()
        return row[0]

    def _collect(self, db, cursors) -> tuple[dict, bool]:
        status = self.ctl.status()
        now = status["t"] + 1e-9
        frame: dict = {"status": status}
        backlog = False

        rows = db.execute(
            "SELECT rowid, time, topic, sender, data FROM messages "
            "WHERE rowid > ? AND time <= ? ORDER BY rowid LIMIT ?",
            (cursors["messages"], now, _LIMITS["messages"])).fetchall()
        if rows:
            cursors["messages"] = rows[-1][0]
            backlog |= len(rows) == _LIMITS["messages"]
            msgs = []
            for _rid, t, topic, sender, blob in rows:
                data = json.loads(blob)
                m = {"t": round(t, 2), "topic": topic, "sender": sender,
                     "data": data}
                link = describe_link_message(topic, data)
                if link is not None:
                    m["link"] = link
                msgs.append(m)
            frame["messages"] = msgs

        rows = db.execute(
            "SELECT rowid, time, source, key, value FROM telemetry "
            "WHERE rowid > ? AND time <= ? ORDER BY rowid LIMIT ?",
            (cursors["telemetry"], now, _LIMITS["telemetry"])).fetchall()
        if rows:
            cursors["telemetry"] = rows[-1][0]
            backlog |= len(rows) == _LIMITS["telemetry"]
            frame["telemetry"] = [
                [round(t, 2), source, key, round(value, 6)]
                for _rid, t, source, key, value in rows]

        rows = db.execute(
            "SELECT rowid, time, source, kind, detail FROM events "
            "WHERE rowid > ? AND time <= ? ORDER BY rowid LIMIT ?",
            (cursors["events"], now, _LIMITS["events"])).fetchall()
        if rows:
            cursors["events"] = rows[-1][0]
            backlog |= len(rows) == _LIMITS["events"]
            evs = []
            for _rid, t, source, kind, detail_json in rows:
                detail = json.loads(detail_json) if detail_json else {}
                evs.append({"t": round(t, 2), "source": source, "kind": kind,
                            "sev": _severity(kind, detail),
                            "detail": _fmt_detail(detail)})
            frame["events"] = evs

        return frame, backlog


class Console:
    """One live console: mission (or replay pacer) + HTTP server. `main()`
    is a thin CLI over this; tests drive it directly."""

    def __init__(self, *, replay: str | Path | None = None,
                 db: str | Path = "runs/live.db",
                 host: str = "127.0.0.1", port: int = 8765,
                 speed: float = 30.0, seed: int = 0, orbits: float = 4.0,
                 duration: float | None = None, dt: float = 5.0,
                 seu_rate_per_day: float = 0.0, campaign: bool = False,
                 **build_kw) -> None:
        self.ctl = LiveControl(speed=float(speed))
        self.sim = None
        if replay is not None:
            self.db_path = Path(replay)
            boot, t_end, rec_dt = _replay_boot(self.db_path)
            self._worker = threading.Thread(
                target=pace_replay, args=(self.ctl, t_end, rec_dt), daemon=True)
        else:
            self.db_path = Path(db)
            self.db_path.parent.mkdir(parents=True, exist_ok=True)
            duration = duration if duration is not None else orbits * _PERIOD
            faults = random_fault_campaign(seed, duration) if campaign else []
            self.sim = build_sim(dt=dt, seed=seed, recorder_path=self.db_path,
                                 faults=faults,
                                 seu_rate_per_day=seu_rate_per_day, **build_kw)
            self.sim.recorder.enable_wal()
            self.sim.recorder.flush()
            boot = _live_boot(self.sim, duration, self.db_path)
            self._worker = threading.Thread(
                target=fly, args=(self.sim, self.ctl, duration), daemon=True)
        blob = json.dumps(boot, separators=(",", ":")).replace("</", "<\\/")
        page = (_PAGE.replace("__TITLE__", boot["title"])
                .replace("__BOOT__", blob)).encode()
        handler = type("BoundHandler", (_Handler,), {
            "ctl": self.ctl, "db_path": str(self.db_path), "page": page,
            "live": self.sim is not None})
        self.server = ThreadingHTTPServer((host, port), handler)
        self._server_thread = threading.Thread(
            target=self.server.serve_forever, kwargs={"poll_interval": 0.2},
            daemon=True)

    @property
    def url(self) -> str:
        host, port = self.server.server_address[:2]
        return f"http://{host}:{port}/"

    def start(self) -> "Console":
        self._worker.start()
        self._server_thread.start()
        return self

    def join(self, timeout: float | None = None) -> None:
        self._worker.join(timeout)

    def stop(self) -> None:
        self.ctl.set(stop=True)
        self._worker.join(timeout=10)
        self.server.shutdown()
        self.server.server_close()
        if self.sim is not None:
            self.sim.close()
            self.sim = None


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Fly a mission paced to wall clock and watch it live "
                    "in the browser (or --replay a finished recording).")
    ap.add_argument("--replay", metavar="DB",
                    help="watch a finished recording instead of flying")
    ap.add_argument("--seed", type=int, default=0,
                    help="mission seed; every run is reproducible from it")
    ap.add_argument("--orbits", type=float, default=4.0,
                    help="mission length in orbits (default 4, ~1.6 h each)")
    ap.add_argument("--duration", type=float,
                    help="mission length in sim seconds (overrides --orbits)")
    ap.add_argument("--dt", type=float, default=5.0,
                    help="sim timestep in seconds (default 5)")
    ap.add_argument("--seu-rate", type=float, default=0.0,
                    help="SEU bit flips per day (SAA-modulated)")
    ap.add_argument("--campaign", action="store_true",
                    help="seed-deterministic random fault campaign "
                         "(same generator as the Monte Carlo harness)")
    ap.add_argument("--speed", default="30",
                    help="sim seconds per wall second, or 'max' (default 30)")
    ap.add_argument("--port", type=int, default=8765,
                    help="console port (default 8765)")
    ap.add_argument("--host", default="127.0.0.1",
                    help="bind address; 0.0.0.0 exposes it to your LAN")
    ap.add_argument("--db", default="runs/live.db",
                    help="recording path for live flights")
    args = ap.parse_args()

    speed = math.inf if args.speed == "max" else float(args.speed)
    console = Console(replay=args.replay, db=args.db, host=args.host,
                      port=args.port, speed=speed, seed=args.seed,
                      orbits=args.orbits, duration=args.duration, dt=args.dt,
                      seu_rate_per_day=args.seu_rate, campaign=args.campaign)
    console.start()
    mode = "replaying" if args.replay else "flying"
    print(f"{mode} {console.db_path} — console at {console.url}  "
          f"(Ctrl+C to stop)")
    try:
        announced = False
        while True:
            _time.sleep(0.5)
            if console.ctl.done and not announced:
                announced = True
                what = ("replay finished"
                        if args.replay else
                        f"mission complete — recording at {console.db_path}")
                print(f"{what}; console still up at {console.url}")
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        console.stop()


_PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__ — live console</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb;
  --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7;
  --border: rgba(11,11,11,0.10);
  --s1: #2a78d6; --s2: #008300; --s3: #e87ba4; --s4: #eda100;
  --good: #0ca30c; --warning: #fab219; --serious: #ec835a; --critical: #d03b3b;
  --band: rgba(11,11,11,0.045);
  --bandc: rgba(42,120,214,0.07);
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19;
    --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835;
    --border: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #008300; --s3: #d55181; --s4: #c98500;
    --band: rgba(255,255,255,0.055);
    --bandc: rgba(57,135,229,0.13);
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19;
  --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835;
  --border: rgba(255,255,255,0.10);
  --s1: #3987e5; --s2: #008300; --s3: #d55181; --s4: #c98500;
  --band: rgba(255,255,255,0.055);
  --bandc: rgba(57,135,229,0.13);
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--page); color: var(--ink);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif;
}
.console { max-width: 1120px; margin: 0 auto; padding: 18px 16px 48px; }
header { display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap;
         margin: 4px 2px 12px; }
header h1 { font-size: 19px; font-weight: 650; margin: 0; }
header .chips { color: var(--ink-2); font-size: 12.5px; }
header button {
  margin-left: auto; font: inherit; font-size: 12.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 3px 10px; cursor: pointer;
}
.conn { width: 9px; height: 9px; border-radius: 50%; align-self: center;
        background: var(--muted); flex: none; }
.conn.ok { background: var(--good); }
.conn.err { background: var(--critical); }
.card { background: var(--surface); border: 1px solid var(--border);
        border-radius: 10px; padding: 12px 14px 12px; margin-bottom: 12px; }
.card h2 { font-size: 13px; font-weight: 650; margin: 0 0 8px; color: var(--ink); }
.card h2 .zinfo { font-weight: 400; font-size: 11.5px; color: var(--muted);
                  margin-left: 8px; }
.cmdbar { display: flex; align-items: center; gap: 14px; flex-wrap: wrap; }
.clock .t { font-size: 26px; font-weight: 650;
            font-variant-numeric: tabular-nums; line-height: 1.15; }
.clock .sub { font-size: 12px; color: var(--ink-2);
              font-variant-numeric: tabular-nums; }
.runstate { font-size: 11.5px; font-weight: 650; letter-spacing: 0.07em;
            border: 1px solid var(--border); border-radius: 999px;
            padding: 3px 12px; color: var(--ink-2); }
.runstate.live { color: var(--good); border-color: var(--good); }
.runstate.paused { color: var(--warning); border-color: var(--warning); }
.cmdbar .grow { flex: 1; }
.cmdbar button, .speeds button {
  font: inherit; font-size: 12.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 4px 12px; cursor: pointer;
}
.speeds { display: flex; gap: 6px; align-items: center; }
.speeds .lb { font-size: 11.5px; color: var(--muted); margin-right: 2px; }
.speeds button.on { color: var(--ink); border-color: var(--s1); }
.tiles { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
         gap: 10px; margin-bottom: 12px; }
.tile { background: var(--surface); border: 1px solid var(--border);
        border-radius: 10px; padding: 10px 12px 9px; }
.tile .lb { font-size: 12px; color: var(--ink-2); }
.tile .vl { font-size: 22px; font-weight: 600; margin-top: 1px;
            font-variant-numeric: tabular-nums; }
.tile .vl.warn { color: var(--serious); }
.tile .nt { font-size: 11.5px; color: var(--muted); margin-top: 1px;
            min-height: 15px; font-variant-numeric: tabular-nums; }
.duo { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
@media (max-width: 840px) { .duo { grid-template-columns: 1fr; } }
#orbit canvas, #closeup canvas { display: block; width: 100%;
                touch-action: none; border-radius: 6px; cursor: grab; }
.orbit-chips { display: flex; gap: 8px; margin-top: 8px; flex-wrap: wrap; }
.chip { font-size: 11.5px; color: var(--muted); border: 1px solid var(--border);
        border-radius: 999px; padding: 2px 9px; white-space: nowrap; }
.chip.on { color: var(--ink); border-color: var(--s1); }
.pills { display: flex; flex-wrap: wrap; gap: 6px; }
.pill { font-size: 11.5px; color: var(--muted); border: 1px solid var(--border);
        border-radius: 999px; padding: 2px 10px; white-space: nowrap; }
.pill.on { color: var(--ink); border-color: var(--s1);
           background: var(--bandc); }
.feed { max-height: 260px; overflow-y: auto; font-size: 12px;
        overscroll-behavior: contain; }
.feed.mono { font: 11.5px/1.6 ui-monospace, SFMono-Regular, Menlo, Consolas,
             monospace; white-space: pre-wrap; word-break: break-all; }
.ev { padding: 1.5px 0; color: var(--ink-2);
      font-variant-numeric: tabular-nums; }
.ev b { color: var(--ink); font-weight: 600; }
.sev { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
       margin-right: 6px; }
.lane { margin-bottom: 4px; }
.lane-head { display: flex; align-items: baseline; gap: 8px; margin: 8px 2px 2px;
             flex-wrap: wrap; }
.lane-head .t { font-size: 12.5px; font-weight: 650; }
.lane-head .u { font-size: 11.5px; color: var(--muted); }
.legend { margin-left: auto; display: flex; gap: 14px; font-size: 11.5px;
          color: var(--ink-2); font-variant-numeric: tabular-nums; }
.legend .key { display: inline-block; width: 14px; height: 0;
               border-top: 2.5px solid; border-radius: 2px;
               vertical-align: middle; margin-right: 5px; }
.lane canvas { display: block; width: 100%; }
.tblwrap { overflow-x: auto; }
table { border-collapse: collapse; font-size: 12px; width: 100%; }
th { text-align: left; color: var(--ink-2); font-weight: 600;
     position: sticky; top: 0; background: var(--surface); }
th, td { padding: 3px 10px 3px 0; border-bottom: 1px solid var(--grid);
         white-space: nowrap; }
td.num { font-variant-numeric: tabular-nums; }
td.mono { font: 11px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
          max-width: 480px; overflow: hidden; text-overflow: ellipsis; }
tr.pinned td { background: var(--bandc); }
#bustbl tbody tr { cursor: pointer; }
.dot { display: inline-block; width: 7px; height: 7px; border-radius: 50%;
       background: var(--s1); opacity: 0.15; }
.dot.ping { animation: ping 0.9s ease-out; }
@keyframes ping { from { opacity: 1; } to { opacity: 0.15; } }
@media (prefers-reduced-motion: reduce) {
  .dot.ping { animation: none; opacity: 0.6; }
}
.card input[type="text"] {
  font: inherit; font-size: 12.5px; color: var(--ink);
  background: var(--page); border: 1px solid var(--border);
  border-radius: 7px; padding: 3px 10px; margin-left: auto; width: 220px;
}
.cmdrow { display: flex; align-items: center; gap: 8px; flex-wrap: wrap;
          margin: 7px 0; }
.cmdrow .cl { font-size: 11.5px; color: var(--muted); width: 56px;
              flex: none; }
.cmdrow .hint { font-size: 11.5px; color: var(--muted); }
.cmdrow button {
  font: inherit; font-size: 12.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 4px 12px; cursor: pointer;
}
.cmdrow select, .cmdrow input[type="number"], .cmdrow label {
  font: inherit; font-size: 12.5px; color: var(--ink);
}
.cmdrow select, .cmdrow input[type="number"] {
  background: var(--page); border: 1px solid var(--border);
  border-radius: 7px; padding: 3px 8px;
}
.cmdrow input[type="number"] { width: 90px; }
.cmdrow input[type="text"] { margin-left: 0; width: 200px; }
.cmdrow input[type="text"].grow { flex: 1; min-width: 180px; width: auto; }
.cmdrow label { color: var(--ink-2); display: inline-flex;
                align-items: center; gap: 4px; }
.cmdnote { font-size: 11.5px; color: var(--muted); margin-top: 6px;
           min-height: 15px; }
.cmdnote.bad { color: var(--critical); }
.card-head { display: flex; align-items: baseline; gap: 10px;
             margin-bottom: 8px; flex-wrap: wrap; }
.card-head h2 { margin: 0; }
.tail-head { display: flex; align-items: center; gap: 10px; margin: 10px 0 4px;
             font-size: 12px; color: var(--ink-2); }
.tail-head button { font: inherit; font-size: 11.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 1px 9px; cursor: pointer; }
.linkline { color: var(--ink-2); }
.linkline.up { color: var(--s1); }
.linkline.bad { color: var(--critical); }
details { margin: 10px 2px 0; color: var(--ink-2); }
summary { cursor: pointer; font-size: 12.5px; }
.overlay { text-align: center; color: var(--muted); font-size: 12.5px;
           padding: 8px 0 2px; }
.card-head .zinfo { font-weight: 400; font-size: 11.5px; color: var(--muted); }
/* primer + glossary (mirrors the flight-report dashboard) */
.intro { background: var(--surface); border: 1px solid var(--border);
         border-radius: 10px; padding: 4px 14px; margin: 0 0 12px;
         color: var(--ink-2); font-size: 13px; }
.intro summary { font-weight: 650; color: var(--ink); font-size: 13px;
                 padding: 8px 0; cursor: pointer; }
.intro p { margin: 6px 0 10px; }
.intro .gloss { display: grid; grid-template-columns: max-content 1fr;
                gap: 3px 12px; margin: 6px 0 12px; }
.intro .gloss b { color: var(--ink); font-weight: 600; white-space: nowrap; }
/* glossary terms: dotted underline, definition on hover/tap */
.term { text-decoration: underline dotted var(--muted);
        text-underline-offset: 2.5px; cursor: help; }
#tooltip {
  position: fixed; display: none; pointer-events: none; z-index: 20;
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 8px; padding: 7px 10px; font-size: 12px;
  box-shadow: 0 4px 14px rgba(0,0,0,0.13); max-width: 320px;
}
#tooltip .tt-t { color: var(--muted); font-size: 11px; margin-bottom: 3px; }
#tooltip .tt-d { color: var(--ink-2); }
#tooltip .tt-d b { color: var(--ink); }
/* auto-detected findings */
.findings { display: flex; flex-direction: column; gap: 0; }
.finding { display: flex; gap: 10px; align-items: flex-start;
           padding: 8px 4px; border-top: 1px solid var(--grid); }
.finding:first-child { border-top: none; }
.finding.isnew { background: var(--bandc);
                 border-left: 3px solid var(--critical); padding-left: 6px; }
.finding .fsev { flex: none; width: 9px; height: 9px; border-radius: 50%;
                 margin-top: 5px; }
.finding .fmain { flex: 1; min-width: 0; }
.finding .ftext { color: var(--ink-2); }
.finding .fwhen { flex: none; color: var(--muted); font-size: 11.5px;
                  font-variant-numeric: tabular-nums; white-space: nowrap;
                  margin-top: 1px; }
.newbadge { display: inline-block; font-size: 10.5px; font-weight: 700;
            letter-spacing: 0.05em; text-transform: uppercase;
            color: var(--surface); background: var(--critical);
            border-radius: 5px; padding: 1px 6px; margin-right: 6px;
            vertical-align: 1px; }
.catchip { display: inline-block; margin-top: 5px; font-size: 11.5px;
           color: var(--s1); cursor: pointer; user-select: none; }
.catchip:hover { text-decoration: underline; }
.catpanel { margin: 6px 0 2px; padding: 8px 10px; border-left: 2px solid
            var(--border); background: var(--band); border-radius: 0 6px 6px 0;
            font-size: 12px; color: var(--ink-2); }
.catpanel .catmech { margin-bottom: 6px; }
.catpanel .catstatus { color: var(--muted); font-size: 11.5px; }
</style>
</head>
<body>
<div class="console">
  <header>
    <h1>__TITLE__</h1>
    <span class="chips" id="metachips"></span>
    <span class="conn" id="conn" title="stream"></span>
    <button id="themebtn" type="button">theme: auto</button>
  </header>

  <details class="intro">
    <summary>What am I looking at?</summary>
    <p><strong>The spacecraft in one minute.</strong> The physics layer
      holds the truth — orbit, attitude, battery, temperatures. No
      subsystem sees it: they read noisy sensors and one-tick-stale bus
      traffic, and each pursues its own local objective. The EPS
      protects the battery (load shed), the OBC protects the mission
      (safe mode, FDIR), the ADCS points the panel at the sun (that's
      where the watts come from), thermal keeps the battery warm enough
      to charge, the payload fills the data queue, and the ground —
      seeing only what a pass lets through — commands the payload from
      stale telemetry. Every protection is locally sensible; the
      interesting behavior is what happens when they interact.</p>
    <p>Dotted-underlined terms anywhere on this page show their
      definition on hover (tap on a phone); event names do the same.
      <strong>What happened</strong> lists signatures the console
      auto-detects as the flight unfolds. The full glossary:</p>
    <div class="gloss" id="gloss"></div>
  </details>

  <div class="card cmdbar">
    <div class="clock">
      <div class="t" id="clk">T+00:00:00</div>
      <div class="sub" id="clksub">—</div>
    </div>
    <span class="runstate" id="runstate">connecting</span>
    <span class="grow"></span>
    <button id="pausebtn" type="button">Pause</button>
    <span class="speeds" id="speeds"><span class="lb">speed</span></span>
  </div>

  <div class="tiles" id="tiles"></div>

  <div class="card" id="findingscard" style="display:none">
    <div class="card-head"><h2>What happened</h2>
      <span class="zinfo">auto-detected as the flight unfolds</span></div>
    <div class="findings" id="findings"></div></div>

  <div class="duo">
    <div class="card">
      <h2>Orbit <span class="zinfo">drag to rotate</span></h2>
      <div id="orbit"></div>
      <div class="orbit-chips">
        <span class="chip" id="orbchip">orbit 0.00</span>
        <span class="chip" id="eclchip">—</span>
        <span class="chip" id="conchip">—</span>
        <span class="chip" id="saachip">—</span>
      </div>
    </div>
    <div class="card">
      <h2>Attitude <span class="zinfo">drag to rotate &middot; 3rd-person</span></h2>
      <div id="closeup"></div>
      <div class="orbit-chips">
        <span class="chip" id="attlit">&mdash;</span>
        <span class="chip" id="attsun">sun &mdash;</span>
        <span class="chip" id="attrate">rate &mdash;</span>
        <span class="chip" id="atttech" style="cursor:pointer"
              title="show orientation cues">tech</span>
      </div>
    </div>
  </div>

  <div class="card">
    <h2>Spacecraft state</h2>
    <div class="pills" id="pills"></div>
    <h2 style="margin:14px 0 6px">Events</h2>
    <div class="feed" id="events"></div>
  </div>

  <div class="card">
    <h2>Telemetry <span class="zinfo" id="lanewin"></span></h2>
    <div id="lanes"></div>
    <details id="alltlm">
      <summary>All telemetry — latest values</summary>
      <div class="tblwrap"><table id="alltbl">
        <thead><tr><th>source</th><th>key</th><th>value</th><th>age</th></tr></thead>
        <tbody></tbody>
      </table></div>
    </details>
  </div>

  <div class="card">
    <div class="card-head">
      <h2>Spacecraft bus <span class="zinfo">every topic, live — click a row
        to tail it</span></h2>
      <input type="text" id="busfilter" placeholder="filter topics, e.g. sensors/">
    </div>
    <div class="tblwrap"><table id="bustbl">
      <thead><tr><th></th><th>topic</th><th>sender</th><th>rate</th>
        <th>last payload</th></tr></thead>
      <tbody></tbody>
    </table></div>
    <div id="tailwrap" style="display:none">
      <div class="tail-head"><span id="tailtitle"></span>
        <button id="tailclose" type="button">unpin</button></div>
      <div class="feed mono" id="tail"></div>
    </div>
  </div>

  <div class="card">
    <h2>Space link <span class="zinfo">decoded frames, ground &#8596; sat
      (&#8595; telemetry down, &#8593; commands up)</span></h2>
    <div class="feed mono" id="link"></div>
  </div>

  <div class="card" id="cmdpanel" style="display:none">
    <h2>Commanding <span class="zinfo">everything here is recorded and
      lands on the next tick</span></h2>
    <div class="cmdrow"><span class="cl">uplink</span>
      <button id="tcon" type="button">payload ON</button>
      <button id="tcoff" type="button">payload OFF</button>
      <span class="hint">a real TC through the ground station's ARQ —
        retransmitted until a pass gets it through</span>
    </div>
    <div class="cmdrow"><span class="cl">fault</span>
      <select id="fkind"></select>
      <span id="fparams" class="cmdrow" style="margin:0"></span>
      <button id="finject" type="button">inject</button>
      <span class="hint">ground truth — physics honors it immediately</span>
    </div>
    <div class="cmdrow"><span class="cl">raw bus</span>
      <input type="text" id="rtopic" placeholder="topic, e.g. obc/mode">
      <input type="text" id="rdata" class="grow"
        placeholder='JSON payload, e.g. {"mode":"SAFE"}'>
      <button id="rpub" type="button">publish</button>
    </div>
    <div class="cmdnote" id="cmdnote"></div>
  </div>
</div>
<div id="tooltip"></div>
<script>
"use strict";
var BOOT = __BOOT__;
var PERIOD = BOOT.meta.period_s;
var WIN = PERIOD * 1.5;
var EPOCH_MS = BOOT.meta.epoch ? Date.parse(BOOT.meta.epoch) : null;
var SCOL = ["--s1", "--s2", "--s3", "--s4"];
var reduced = window.matchMedia &&
    matchMedia("(prefers-reduced-motion: reduce)").matches;

function $(id) { return document.getElementById(id); }
function div(cls, parent, text) {
  var d = document.createElement("div");
  d.className = cls;
  if (text !== undefined) d.textContent = text;
  if (parent) parent.appendChild(d);
  return d;
}
function css(name) {
  return getComputedStyle(document.documentElement)
    .getPropertyValue(name).trim();
}
function fmt(v) {
  if (v === null || v === undefined) return "—";
  var a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 100) return v.toFixed(1);
  if (a >= 1) return v.toFixed(2);
  return v.toFixed(3);
}
function pad(n) { return (n < 10 ? "0" : "") + n; }
function sevColor(sev) {
  return css({ critical: "--critical", warning: "--warning",
               good: "--good" }[sev] || "--muted");
}

/* ---- glossary: every term of art teaches itself on hover ----
   Mirrors the flight-report dashboard. One dictionary (BOOT.gloss) feeds
   the primer grid and dotted-underline .term spans wrapped around matches
   in visible text; event kinds get BOOT.evgloss in the event feed. */
var GLOSS = BOOT.gloss || {}, EVGLOSS = BOOT.evgloss || {};
var ALIAS = { "ground contact": "pass", "load shed": "load shed" };
var GLOSS_LC = {}, tooltip = $("tooltip"), termActive = false;
function normTerm(s) { return String(s).toLowerCase().replace(/-/g, " "); }
Object.keys(GLOSS).forEach(function (k) { GLOSS_LC[normTerm(k)] = k; });
function defFor(name) {
  var lc = normTerm(name);
  if (ALIAS[lc]) lc = normTerm(ALIAS[lc]);
  var k = GLOSS_LC[lc];
  return k ? { name: k, def: GLOSS[k] } : null;
}
function termRegex(keys, flags) {
  keys = keys.slice().sort(function (a, b) { return b.length - a.length; });
  var alts = keys.map(function (k) {
    return k.replace(/[.*+?^${}()|[\]\\]/g, "\\$&").replace(/[ -]/g, "[ -]");
  });
  return new RegExp("(^|[^A-Za-z0-9_-])(" + alts.join("|") +
                    ")((?:es|s)?)(?![A-Za-z0-9_-])", flags);
}
var _acr = [], _phr = [];
Object.keys(GLOSS).forEach(function (k) {
  (/[A-Z]/.test(k) ? _acr : _phr).push(k);
});
var reACR = _acr.length ? termRegex(_acr, "") : null;
var rePHR = _phr.length ? termRegex(_phr, "i") : null;
function glossifyNode(textNode) {
  var s = textNode.nodeValue, out = [], pos = 0, hits = 0;
  while (pos < s.length) {
    var rest = s.slice(pos), bm = null, bat = Infinity;
    [reACR, rePHR].forEach(function (re) {
      if (!re) return;
      var m = rest.match(re);
      if (m && m.index + m[1].length < bat) { bat = m.index + m[1].length; bm = m; }
    });
    if (!bm) break;
    var start = pos + bat, len = bm[2].length + bm[3].length;
    var d = defFor(bm[2]);
    if (d) {
      out.push(s.slice(pos, start));
      out.push({ text: s.slice(start, start + len), d: d });
      hits++;
    } else {
      out.push(s.slice(pos, start + len));
    }
    pos = start + len;
  }
  if (!hits) return;
  out.push(s.slice(pos));
  var frag = document.createDocumentFragment();
  out.forEach(function (o) {
    if (typeof o === "string") { if (o) frag.appendChild(document.createTextNode(o)); return; }
    var sp = document.createElement("span");
    sp.className = "term"; sp.textContent = o.text;
    sp.dataset.name = o.d.name; sp.dataset.def = o.d.def;
    frag.appendChild(sp);
  });
  textNode.parentNode.replaceChild(frag, textNode);
}
function glossify(root) {
  if (!root) return;
  var walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, null);
  var nodes = [];
  while (walker.nextNode()) {
    var p = walker.currentNode.parentNode;
    if (p.closest && p.closest(".term,.gloss,.evkind,.catchip,script,style")) continue;
    nodes.push(walker.currentNode);
  }
  nodes.forEach(glossifyNode);
}
function placeTip(ev) {
  var tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
  var x = ev.clientX + 14, y = ev.clientY + 12;
  if (x + tw > window.innerWidth - 8) x = ev.clientX - tw - 14;
  if (y + th > window.innerHeight - 8) y = ev.clientY - th - 12;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}
function showTermTip(ev, name, def) {
  tooltip.textContent = "";
  div("tt-t", tooltip, name);
  div("tt-d", tooltip, def);
  tooltip.style.display = "block";
  placeTip(ev);
}
document.addEventListener("mouseover", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
});
document.addEventListener("mouseout", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = false; tooltip.style.display = "none"; }
});
document.addEventListener("click", function (ev) {  // touch: tap toggles
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
  else if (termActive) { termActive = false; tooltip.style.display = "none"; }
});

/* ---- findings: catalog signatures the server streams as they fire ---- */
var CAT = BOOT.catalog || {};
function drawFindings(notes) {
  var card = $("findingscard"), host = $("findings");
  if (!notes) return;
  card.style.display = notes.length ? "" : "none";
  host.textContent = "";
  notes.forEach(function (n) {
    var row = div("finding" + (n.new ? " isnew" : ""), host);
    div("fsev", row).style.background = n.new ? sevColor("critical")
                                              : sevColor(n.sev);
    var main = div("fmain", row);
    var body = div("ftext", main);
    if (n.new) {
      var badge = document.createElement("span");
      badge.className = "newbadge"; badge.textContent = "possibly new";
      body.appendChild(badge);
    }
    body.appendChild(document.createTextNode(n.text));
    var cat = n.entry && CAT[n.entry];
    if (cat) {
      var label = "catalog entry " + n.entry + ": " + cat.title;
      var chip = div("catchip nogloss", main);
      chip.textContent = "▸ " + label;
      var panel = div("catpanel", main);
      panel.style.display = "none";
      div("catmech", panel, cat.mechanism);
      if (cat.status) div("catstatus", panel, "status: " + cat.status);
      chip.addEventListener("click", function () {
        var open = panel.style.display === "none";
        panel.style.display = open ? "" : "none";
        chip.textContent = (open ? "▾ " : "▸ ") + label;
      });
    }
    var span = n.t1 - n.t0;
    div("fwhen", row, span > PERIOD * 0.05
      ? "orbit " + (n.t0 / PERIOD).toFixed(2) + "–" + (n.t1 / PERIOD).toFixed(2)
      : "orbit " + (n.t0 / PERIOD).toFixed(2));
  });
  glossify(host);
}

/* one-time: fill the glossary grid and glossify the primer */
(function () {
  var grid = $("gloss");
  Object.keys(GLOSS).forEach(function (k) {
    var b = document.createElement("b"); b.textContent = k;
    var sp = document.createElement("span"); sp.textContent = GLOSS[k];
    grid.appendChild(b); grid.appendChild(sp);
  });
  glossify(document.querySelector(".intro"));
})();

/* ---- theme ---- */
(function () {
  var KEY = "cubesat-live-theme", modes = ["auto", "light", "dark"];
  var cur = localStorage.getItem(KEY) || "auto";
  function apply() {
    if (cur === "auto") document.documentElement.removeAttribute("data-theme");
    else document.documentElement.setAttribute("data-theme", cur);
    $("themebtn").textContent = "theme: " + cur;
  }
  $("themebtn").addEventListener("click", function () {
    cur = modes[(modes.indexOf(cur) + 1) % 3];
    localStorage.setItem(KEY, cur);
    apply();
  });
  apply();
})();

/* ---- state ---- */
var state = {
  simTime: 0, tick: 0, paused: false, speed: 30, done: false,
  statusWall: null, tickSimTime: 0, tickWall: null, mode: BOOT.mode,
};
var latest = {};       // "source/key" -> [t, value]
var laneIndex = {};    // "source/key" -> [series buffer, ...]
var lanes = [];
var bus = {};          // "topic|sender" -> row record
var pinned = null;

function lv(key) {
  var e = latest[key];
  return e ? e[1] : null;
}
function displayTime() {
  // advance smoothly from the last tick's sim time at the paced speed, so the
  // clock ticks in real time at 1x instead of jumping one dt per tick; capped
  // at one tick so a stalled stream can never run the clock away
  if (state.tickWall === null || state.paused || state.done ||
      state.speed === null) return state.simTime;
  var e = (performance.now() - state.tickWall) / 1000;
  return state.tickSimTime + Math.min(e * state.speed, BOOT.meta.dt);
}

/* ---- header chips ---- */
$("metachips").textContent =
  (BOOT.mode === "replay" ? "replay · " : "live · ") +
  "seed " + BOOT.meta.seed + " · dt " + BOOT.meta.dt + " s · plan " +
  (BOOT.meta.duration_s / PERIOD).toFixed(1) + " orbits";
$("lanewin").textContent = "rolling window · last 1.5 orbits";

/* ---- control strip ---- */
function post(body) {
  fetch("/control", { method: "POST", body: JSON.stringify(body) });
}
$("pausebtn").addEventListener("click", function () {
  post({ action: state.paused ? "resume" : "pause" });
});
var SPEEDS = [1, 10, 30, 60, 120];
SPEEDS.forEach(function (v) {
  var b = document.createElement("button");
  b.textContent = v + "×";
  b.dataset.v = v;
  b.addEventListener("click", function () {
    post({ action: "speed", value: v });
  });
  $("speeds").appendChild(b);
});
function drawControls() {
  $("pausebtn").textContent = state.paused ? "Resume" : "Pause";
  $("pausebtn").disabled = state.done;
  var rs = $("runstate");
  if (state.done) {
    rs.textContent = state.mode === "replay" ? "REPLAY DONE" : "COMPLETE";
    rs.className = "runstate";
  } else if (state.paused) {
    rs.textContent = "PAUSED"; rs.className = "runstate paused";
  } else {
    rs.textContent = state.mode === "replay" ? "REPLAY" : "LIVE";
    rs.className = "runstate live";
  }
  var btns = $("speeds").querySelectorAll("button");
  for (var i = 0; i < btns.length; i++) {
    btns[i].className =
      state.speed !== null && Math.abs(parseFloat(btns[i].dataset.v) -
        state.speed) < 0.01 ? "on" : "";
  }
}

/* ---- command panel (live flights only) ---- */
(function () {
  if (BOOT.mode !== "live") return;
  $("cmdpanel").style.display = "";
  function note(txt, bad) {
    $("cmdnote").textContent = txt;
    $("cmdnote").className = "cmdnote" + (bad ? " bad" : "");
  }
  function send(body, desc) {
    fetch("/control", { method: "POST", body: JSON.stringify(body) })
      .then(function (r) {
        if (r.ok) {
          note(desc + " queued — lands on the next tick" +
               (state.paused ? " (mission is paused)" : ""), false);
        } else {
          note(desc + " rejected (HTTP " + r.status + ")", true);
        }
      })
      .catch(function () { note(desc + " failed — server unreachable", true); });
  }
  $("tcon").addEventListener("click", function () {
    send({ action: "tc", cmd: 1, arg: 1 }, "TC payload-enable(1)");
  });
  $("tcoff").addEventListener("click", function () {
    send({ action: "tc", cmd: 1, arg: 0 }, "TC payload-enable(0)");
  });

  var SENSORS = ["gyro", "mag", "sun", "battery_voltage"];
  var FAULTS = [
    { label: "stuck sensor", topic: "fault/sensor_stuck",
      params: [["sensor", "select", SENSORS], ["hard", "check", false]],
      data: function (p) {
        return { sensor: p.sensor, stuck: true, hard: !!p.hard }; } },
    { label: "unstick sensor", topic: "fault/sensor_stuck",
      params: [["sensor", "select", SENSORS]],
      data: function (p) { return { sensor: p.sensor, stuck: false }; } },
    { label: "SEU bit flip", topic: "fault/seu",
      params: [["sensor", "select", SENSORS]],
      data: function (p) { return { sensor: p.sensor }; } },
    { label: "wheel friction", topic: "fault/wheel_friction",
      params: [["nm_per_nms", "number", "5e-5"]],
      data: function (p) { return { nm_per_nms: p.nm_per_nms }; } },
    { label: "array strike", topic: "fault/array_hit",
      params: [["mult", "number", "0.7"]],
      data: function (p) { return { mult: p.mult }; } },
    { label: "channel BER", topic: "fault/channel",
      params: [["ber_mult", "number", "50"]],
      data: function (p) { return { ber_mult: p.ber_mult }; } },
  ];
  FAULTS.forEach(function (f, i) {
    var o = document.createElement("option");
    o.value = i; o.textContent = f.label;
    $("fkind").appendChild(o);
  });
  var fwidgets = {};
  function renderParams() {
    var f = FAULTS[+$("fkind").value];
    $("fparams").textContent = "";
    fwidgets = {};
    f.params.forEach(function (spec) {
      var name = spec[0], kind = spec[1], init = spec[2], el;
      if (kind === "select") {
        el = document.createElement("select");
        init.forEach(function (s) {
          var o = document.createElement("option");
          o.value = s; o.textContent = s;
          el.appendChild(o);
        });
        $("fparams").appendChild(el);
      } else if (kind === "check") {
        var lab = document.createElement("label");
        el = document.createElement("input");
        el.type = "checkbox";
        lab.appendChild(el);
        lab.appendChild(document.createTextNode(name));
        $("fparams").appendChild(lab);
      } else {
        el = document.createElement("input");
        el.type = "number"; el.step = "any"; el.value = init;
        el.title = name;
        $("fparams").appendChild(el);
      }
      fwidgets[name] = { el: el, kind: kind };
    });
  }
  $("fkind").addEventListener("change", renderParams);
  renderParams();
  $("finject").addEventListener("click", function () {
    var f = FAULTS[+$("fkind").value], p = {}, name;
    for (name in fwidgets) {
      var w = fwidgets[name];
      if (w.kind === "check") p[name] = w.el.checked;
      else if (w.kind === "number") {
        p[name] = parseFloat(w.el.value);
        if (!isFinite(p[name])) { note(name + " is not a number", true); return; }
      } else p[name] = w.el.value;
    }
    send({ action: "inject", topic: f.topic, data: f.data(p) }, f.label);
  });

  $("rpub").addEventListener("click", function () {
    var topic = $("rtopic").value.trim(), data;
    if (!topic) { note("topic is empty", true); return; }
    try {
      data = JSON.parse($("rdata").value.trim() || "{}");
    } catch (e) {
      note("payload is not valid JSON", true); return;
    }
    send({ action: "inject", topic: topic, data: data }, topic);
  });
})();

/* ---- clock ---- */
function drawClock() {
  var t = displayTime();
  var s = Math.floor(t), d = Math.floor(s / 86400);
  var txt = "T+" + (d ? d + "d " : "") +
    pad(Math.floor(s / 3600) % 24) + ":" + pad(Math.floor(s / 60) % 60) +
    ":" + pad(s % 60);
  $("clk").textContent = txt;
  var sub = "orbit " + (t / PERIOD).toFixed(2) + " · tick " + state.tick;
  if (EPOCH_MS !== null) {
    sub += " · " + new Date(EPOCH_MS + t * 1000).toISOString()
      .slice(0, 19).replace("T", " ") + " UTC";
  }
  if (state.speed === null) sub += " · max speed";
  $("clksub").textContent = sub;
}

/* ---- stat tiles ---- */
var TILES = [
  { label: "Mode", make: function () {
      var safe = lv("obc/safe_mode"), shed = lv("eps/shedding");
      if (safe === null) return { v: "—", note: "", warn: false };
      var v = safe >= 0.5 ? "SAFE" : "NOMINAL";
      return { v: v, note: shed >= 0.5 ? "load shedding" : "",
               warn: safe >= 0.5 || shed >= 0.5 };
    } },
  { label: "State of charge", make: function () {
      var est = lv("eps/soc_est"), tru = lv("physics/soc_true");
      return { v: est === null ? "—" : (est * 100).toFixed(1) + "%",
               note: tru === null ? "" : "true " + (tru * 100).toFixed(1) + "%",
               warn: tru !== null && tru < 0.3 };
    } },
  { label: "Body rate", make: function () {
      var tru = lv("physics/rate_dps"), est = lv("adcs/rate_dps");
      return { v: tru === null ? "—" : fmt(tru) + "°/s",
               note: est === null ? "" : "ADCS believes " + fmt(est),
               warn: tru !== null && tru > 2.0 };
    } },
  { label: "Power", make: function () {
      var g = lv("physics/p_gen_w"), l = lv("physics/p_load_w");
      return { v: g === null ? "—" : fmt(g) + " W in",
               note: l === null ? "" : fmt(l) + " W out", warn: false };
    } },
  { label: "Data queue", make: function () {
      var q = lv("comms/queue_mb"), dr = lv("comms/dropped_mb");
      return { v: q === null ? "—" : fmt(q) + " MB",
               note: dr > 0 ? fmt(dr) + " MB dropped" : "",
               warn: dr > 0 };
    } },
  { label: "Ground archive", make: function () {
      var a = lv("ground/archive_mb"), rej = lv("ground/frames_rejected");
      return { v: a === null ? "—" : fmt(a) + " MB",
               note: rej > 0 ? rej.toFixed(0) + " frames rejected" : "",
               warn: false };
    } },
];
TILES.forEach(function (tl) {
  var t = div("tile", $("tiles"));
  div("lb", t, tl.label);
  tl.vEl = div("vl", t, "—");
  tl.nEl = div("nt", t, "");
});
glossify($("tiles"));  // static labels only; values/notes reset by text
function drawTiles() {
  TILES.forEach(function (tl) {
    var r = tl.make();
    tl.vEl.textContent = r.v;
    tl.vEl.className = "vl" + (r.warn ? " warn" : "");
    tl.nEl.textContent = r.note;
  });
}

/* ---- state pills ---- */
BOOT.pills.forEach(function (p) {
  p.el = div("pill", $("pills"), p.label);
  p.k = p.source + "/" + p.key;
});
glossify($("pills"));  // pill labels are static; only .on class toggles
function drawPills() {
  BOOT.pills.forEach(function (p) {
    var v = lv(p.k);
    p.el.className = "pill" + (v !== null && v >= 0.5 ? " on" : "");
  });
}

/* ---- telemetry lanes ---- */
BOOT.lanes.forEach(function (spec) {
  var host = div("lane", $("lanes"));
  var head = div("lane-head", host);
  div("t", head, spec.title);
  div("u", head, spec.unit);
  var legend = div("legend", head);
  var canvas = document.createElement("canvas");
  host.appendChild(canvas);
  var lane = { spec: spec, canvas: canvas, series: [] };
  spec.series.forEach(function (s, i) {
    var item = document.createElement("span");
    var key = document.createElement("span");
    key.className = "key";
    key.style.borderTopColor = "var(" + SCOL[i % 4] + ")";
    item.appendChild(key);
    var txt = document.createTextNode(s.label + " —");
    item.appendChild(txt);
    legend.appendChild(item);
    var buf = { pts: [], tf: s.tf, label: s.label, txt: txt };
    lane.series.push(buf);
    var k = s.source + "/" + s.key;
    (laneIndex[k] = laneIndex[k] || []).push(buf);
  });
  lanes.push(lane);
});
function tfApply(tf, v) { return tf === "kelvin" ? v - 273.15 : v; }
function drawLane(lane) {
  var c = lane.canvas, w = c.parentNode.clientWidth - 4;
  if (w < 40) return;
  var H = 74, dpr = window.devicePixelRatio || 1;
  if (c.width !== Math.round(w * dpr)) {
    c.width = Math.round(w * dpr); c.height = Math.round(H * dpr);
    c.style.width = w + "px"; c.style.height = H + "px";
  }
  var ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, H);
  var t1 = displayTime(), t0 = t1 - WIN;
  var lo, hi;
  if (lane.spec.domain) { lo = lane.spec.domain[0]; hi = lane.spec.domain[1]; }
  else {
    lo = Infinity; hi = -Infinity;
    lane.series.forEach(function (s) {
      for (var i = 0; i < s.pts.length; i += 2) {
        if (s.pts[i] < t0) continue;
        var v = s.pts[i + 1];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    });
    if (lo > hi) { lo = 0; hi = 1; }
    if (hi - lo < 1e-9) { hi += 0.5; lo -= 0.5; }
    var padv = (hi - lo) * 0.08;
    lo -= padv; hi += padv;
  }
  var gridc = css("--grid");
  ctx.strokeStyle = gridc; ctx.lineWidth = 1;
  [0, 0.5, 1].forEach(function (fr) {
    var y = 4 + (H - 8) * fr;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  });
  ctx.font = "10px system-ui, sans-serif";
  ctx.fillStyle = css("--muted");
  ctx.fillText(fmt(hi), 3, 12);
  ctx.fillText(fmt(lo), 3, H - 5);
  function X(t) { return (t - t0) / WIN * w; }
  function Y(v) {
    var f = (v - lo) / (hi - lo);
    return 4 + (H - 8) * (1 - Math.max(0, Math.min(1, f)));
  }
  lane.series.forEach(function (s, i) {
    ctx.strokeStyle = css(SCOL[i % 4]);
    ctx.lineWidth = 2; ctx.lineJoin = "round";
    ctx.beginPath();
    var pen = false, lastV = null;
    for (var j = 0; j < s.pts.length; j += 2) {
      var t = s.pts[j], v = s.pts[j + 1];
      if (t < t0 - 60) continue;
      if (pen) ctx.lineTo(X(t), Y(v));
      else { ctx.moveTo(X(t), Y(v)); pen = true; }
      lastV = v;
    }
    ctx.stroke();
    s.txt.textContent = s.label + " " + fmt(lastV);
  });
}
function drawLanes() { lanes.forEach(drawLane); }
window.addEventListener("resize", drawLanes);

/* ---- all-telemetry table ---- */
function drawAllTable() {
  if (!$("alltlm").open) return;
  var keys = Object.keys(latest).sort();
  var tb = $("alltbl").tBodies[0];
  tb.textContent = "";
  var now = displayTime();
  keys.forEach(function (k) {
    var tr = tb.insertRow();
    var s = k.split("/");
    tr.insertCell().textContent = s[0];
    tr.insertCell().textContent = s.slice(1).join("/");
    var c = tr.insertCell(); c.className = "num";
    c.textContent = fmt(latest[k][1]);
    var a = tr.insertCell(); a.className = "num";
    a.textContent = fmt(Math.max(0, now - latest[k][0])) + " s";
  });
}

/* ---- bus monitor ---- */
function busUpdate(m) {
  var k = m.topic + "|" + m.sender;
  var e = bus[k];
  if (!e) {
    e = bus[k] = { topic: m.topic, sender: m.sender, times: [] };
    var tb = $("bustbl").tBodies[0];
    // keep rows sorted by topic so the table never jumps around
    var rows = tb.rows, at = rows.length;
    for (var i = 0; i < rows.length; i++) {
      if (rows[i].dataset.k > k) { at = i; break; }
    }
    var tr = tb.insertRow(at);
    tr.dataset.k = k;
    e.tr = tr;
    var dcell = tr.insertCell();
    e.dot = document.createElement("span");
    e.dot.className = "dot";
    dcell.appendChild(e.dot);
    tr.insertCell().textContent = m.topic;
    tr.insertCell().textContent = m.sender;
    e.rate = tr.insertCell(); e.rate.className = "num";
    e.pay = tr.insertCell(); e.pay.className = "mono";
    tr.addEventListener("click", function () { pin(e.topic); });
    applyFilter(tr);
  }
  e.times.push(m.t);
  if (e.times.length > 6) e.times.shift();
  var n = e.times.length;
  e.rate.textContent = n > 1 && e.times[n - 1] > e.times[0]
    ? (60 * (n - 1) / (e.times[n - 1] - e.times[0])).toFixed(1) + "/min"
    : "—";
  var pj = JSON.stringify(m.data);
  e.pay.textContent = pj.length > 160 ? pj.slice(0, 157) + "…" : pj;
  e.pay.title = "t=" + m.t;
  e.dot.classList.remove("ping");
  void e.dot.offsetWidth;
  e.dot.classList.add("ping");
  if (pinned === e.topic) tailPush(m);
}
function applyFilter(tr) {
  var q = $("busfilter").value.trim();
  tr.style.display = !q || tr.dataset.k.indexOf(q) !== -1 ? "" : "none";
}
$("busfilter").addEventListener("input", function () {
  var rows = $("bustbl").tBodies[0].rows;
  for (var i = 0; i < rows.length; i++) applyFilter(rows[i]);
});
function pin(topic) {
  pinned = topic;
  $("tailwrap").style.display = "";
  $("tailtitle").textContent = "tailing " + topic;
  $("tail").textContent = "";
  var rows = $("bustbl").tBodies[0].rows;
  for (var i = 0; i < rows.length; i++) {
    rows[i].className =
      rows[i].dataset.k.split("|")[0] === topic ? "pinned" : "";
  }
}
$("tailclose").addEventListener("click", function () {
  pinned = null;
  $("tailwrap").style.display = "none";
  var rows = $("bustbl").tBodies[0].rows;
  for (var i = 0; i < rows.length; i++) rows[i].className = "";
});
function pushFeed(host, node, cap) {
  var stick =
    host.scrollTop + host.clientHeight >= host.scrollHeight - 12;
  host.appendChild(node);
  while (host.childNodes.length > cap) host.removeChild(host.firstChild);
  if (stick) host.scrollTop = host.scrollHeight;
}
function tailPush(m) {
  var line = document.createElement("div");
  line.textContent = "orbit " + (m.t / PERIOD).toFixed(3) + "  [" +
    m.sender + "] " + JSON.stringify(m.data);
  pushFeed($("tail"), line, 300);
}

/* ---- link monitor ---- */
function linkPush(m) {
  var line = document.createElement("div");
  var txt = m.link;
  line.className = "linkline" +
    (txt.indexOf("**") !== -1 ? " bad" : txt.indexOf("UP") === 0 ? " up" : "");
  txt = txt.replace(/^DOWN/, "↓").replace(/^UP\* /, "↑* ")
           .replace(/^UP {2}/, "↑  ");
  line.textContent = "orbit " + (m.t / PERIOD).toFixed(3) + "  " + txt;
  pushFeed($("link"), line, 250);
}

/* ---- event ticker ---- */
function evPush(ev) {
  var line = div("ev", null);
  var dot = document.createElement("span");
  dot.className = "sev";
  var col = { good: "--good", warning: "--warning",
              critical: "--critical" }[ev.sev] || "--muted";
  dot.style.background = "var(" + col + ")";
  line.appendChild(dot);
  line.appendChild(document.createTextNode(
    "orbit " + (ev.t / PERIOD).toFixed(2) + " · "));
  var b = document.createElement("b");
  var sd = defFor(ev.source);
  if (sd) {
    var ss = document.createElement("span");
    ss.className = "term"; ss.textContent = ev.source;
    ss.dataset.name = sd.name; ss.dataset.def = sd.def;
    b.appendChild(ss);
  } else b.appendChild(document.createTextNode(ev.source));
  b.appendChild(document.createTextNode(" "));
  if (EVGLOSS[ev.kind]) {
    var ks = document.createElement("span");
    ks.className = "term"; ks.textContent = ev.kind;
    ks.dataset.name = ev.kind; ks.dataset.def = EVGLOSS[ev.kind];
    b.appendChild(ks);
  } else b.appendChild(document.createTextNode(ev.kind));
  line.appendChild(b);
  if (ev.detail) line.appendChild(document.createTextNode(" " + ev.detail));
  var host = $("events");
  host.insertBefore(line, host.firstChild);
  while (host.childNodes.length > 300) host.removeChild(host.lastChild);
}

/* ---- orbit globe (adapted from the flight-report dashboard) ---- */
var globe = (function () {
  var host = $("orbit"), O = BOOT.orbit3d;
  var canvas = document.createElement("canvas");
  host.appendChild(canvas);
  var vs = { yaw: -0.9, pitch: 0.38 };
  var W = 0, H = 0, cx = 0, cy = 0, sc = 1, ctx = null;
  function size() {
    var w = host.clientWidth;
    if (w < 40) return false;
    var h = Math.max(220, Math.min(400, Math.round(w * 0.62)));
    var dpr = window.devicePixelRatio || 1;
    if (W !== w || H !== h) {
      W = w; H = h;
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      canvas.style.width = w + "px"; canvas.style.height = h + "px";
      cx = W / 2; cy = H / 2;
      sc = (Math.min(W, H) / 2 - 8) / 1.32;
      ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    return true;
  }
  function rot(v) {
    var c = Math.cos(vs.yaw), sn = Math.sin(vs.yaw);
    var x = v[0] * c - v[1] * sn, y = v[0] * sn + v[1] * c, z = v[2];
    var cp = Math.cos(vs.pitch), sp = Math.sin(vs.pitch);
    return [x, y * cp - z * sp, y * sp + z * cp];
  }
  function P(v) {
    var r = rot(v);
    return { x: cx + r[0] * sc, y: cy - r[2] * sc, d: -r[1] };
  }
  function satPos(t) {
    var u = O.n_rad_s * t, cu = Math.cos(u), su = Math.sin(u),
        R = O.r_orbit_re;
    return [R * (cu * O.e1[0] + su * O.e2[0]),
            R * (cu * O.e1[1] + su * O.e2[1]),
            R * (cu * O.e1[2] + su * O.e2[2])];
  }
  function siteEci(lat, lon, t) {
    var la = lat * Math.PI / 180;
    var lo = lon * Math.PI / 180 + O.gmst0_rad + O.w_earth_rad_s * t;
    return [Math.cos(la) * Math.cos(lo), Math.cos(la) * Math.sin(lo),
            Math.sin(la)];
  }
  function latLon(lat, lon) {
    var la = lat * Math.PI / 180, lo = lon * Math.PI / 180;
    return [Math.cos(la) * Math.cos(lo), Math.cos(la) * Math.sin(lo),
            Math.sin(la)];
  }
  function occluded(p) {
    var dx = p.x - cx, dy = p.y - cy;
    return p.d < 0 && (dx * dx + dy * dy) < (sc * 0.998) * (sc * 0.998);
  }
  function inShadow(r) {  // cylindrical Earth shadow, r in Earth radii
    var d = r[0] * O.sun[0] + r[1] * O.sun[1] + r[2] * O.sun[2];
    if (d >= 0) return false;
    var px = r[0] - d * O.sun[0], py = r[1] - d * O.sun[1],
        pz = r[2] - d * O.sun[2];
    return px * px + py * py + pz * pz < 1.0;
  }
  function polyline(pts, filter, color, width, alpha) {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.globalAlpha = alpha;
    ctx.beginPath();
    var pen = false;
    for (var i = 0; i < pts.length; i++) {
      if (filter(pts[i])) {
        if (pen) ctx.lineTo(pts[i].x, pts[i].y);
        else { ctx.moveTo(pts[i].x, pts[i].y); pen = true; }
      } else pen = false;
    }
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  var PERIOD_O = 2 * Math.PI / O.n_rad_s;
  function render(t) {
    if (!size()) return;
    ctx.clearRect(0, 0, W, H);
    var grid = css("--grid"), axis = css("--axis"), s1 = css("--s1"),
        s2 = css("--s2"), s3 = css("--s3"), s4 = css("--s4"),
        serious = css("--serious"), ink2 = css("--ink-2"),
        surface = css("--surface");
    var gmst = O.gmst0_rad + O.w_earth_rad_s * t;
    var lonOff = gmst * 180 / Math.PI;
    function earthPt(lat, lon) { return P(latLon(lat, lon + lonOff)); }

    var lines = [];
    for (var lon = 0; lon < 180; lon += 30) {
      var pts = [];
      for (var a = 0; a <= 360; a += 5) {
        var la = (a <= 180 ? a - 90 : 270 - a);
        pts.push(earthPt(la, a <= 180 ? lon : lon + 180));
      }
      lines.push(pts);
    }
    [-60, -30, 0, 30, 60].forEach(function (lat) {
      var pts = [];
      for (var b = 0; b <= 360; b += 5) pts.push(earthPt(lat, b));
      lines.push(pts);
    });
    lines.forEach(function (pts) {
      polyline(pts, function (p) { return p.d < 0; }, grid, 1, 0.45);
    });
    ctx.fillStyle = css("--band");
    ctx.beginPath(); ctx.arc(cx, cy, sc, 0, 2 * Math.PI); ctx.fill();
    var sv = rot(O.sun), sl = Math.hypot(sv[0], sv[2]) || 1;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, sc, 0, 2 * Math.PI); ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(Math.atan2(-sv[2], sv[0]) + Math.PI);
    ctx.fillStyle = "rgba(0,0,0,0.10)";
    ctx.fillRect(0, -sc, sc, 2 * sc);
    ctx.restore();
    lines.forEach(function (pts) {
      polyline(pts, function (p) { return p.d >= 0; }, grid, 1, 0.9);
    });
    ctx.strokeStyle = axis; ctx.lineWidth = 1; ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(cx, cy, sc, 0, 2 * Math.PI); ctx.stroke();

    var saa = [];
    var la0 = O.saa.lat[0], la1 = O.saa.lat[1],
        lo0 = O.saa.lon[0], lo1 = O.saa.lon[1], k;
    for (k = lo0; k <= lo1; k += 5) saa.push(earthPt(la1, k));
    for (k = la1; k >= la0; k -= 5) saa.push(earthPt(k, lo1));
    for (k = lo1; k >= lo0; k -= 5) saa.push(earthPt(la0, k));
    for (k = la0; k <= la1; k += 5) saa.push(earthPt(k, lo0));
    polyline(saa, function (p) { return p.d >= 0; }, serious, 1.4, 0.6);
    var saaC = earthPt((la0 + la1) / 2, (lo0 + lo1) / 2);
    if (saaC.d > 0.15) {
      ctx.fillStyle = serious; ctx.globalAlpha = 0.75;
      ctx.font = "10px system-ui, sans-serif";
      ctx.fillText("SAA", saaC.x - 10, saaC.y + 3);
      ctx.globalAlpha = 1;
    }

    // orbit ring, faded where the geometry says shadow
    var t0ring = Math.floor(t / PERIOD_O) * PERIOD_O;
    ctx.lineWidth = 2;
    for (var i = 0; i < 180; i++) {
      var ta = t0ring + (i / 180) * PERIOD_O,
          tb = t0ring + ((i + 1) / 180) * PERIOD_O;
      var ra = satPos(ta), rb = satPos(tb);
      var pa = P(ra), pb = P(rb);
      var hid = occluded(pa) || occluded(pb);
      ctx.strokeStyle = s1;
      ctx.globalAlpha = hid ? 0.10 : (inShadow(ra) ? 0.25 : 0.75);
      ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

    O.sites.forEach(function (site) {
      var p = P(siteEci(site.lat, site.lon, t));
      if (p.d <= 0) return;
      var stn = site.kind === "station";
      ctx.beginPath();
      ctx.arc(p.x, p.y, stn ? 4.5 : 3.5, 0, 2 * Math.PI);
      ctx.fillStyle = stn ? s2 : s3; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
      ctx.fillStyle = ink2; ctx.font = "10px system-ui, sans-serif";
      ctx.fillText(site.name, p.x + 7, p.y + 3);
    });

    var rNow = satPos(t), sp = P(rNow);
    ctx.lineWidth = 2; ctx.strokeStyle = s1;
    var TRAIL = Math.min(t, PERIOD_O * 0.22);
    for (var j = 0; j < 24; j++) {
      var u0 = t - TRAIL * (1 - j / 24),
          u1 = t - TRAIL * (1 - (j + 1) / 24);
      var qa = P(satPos(u0)), qb = P(satPos(u1));
      if (occluded(qa) || occluded(qb)) continue;
      ctx.globalAlpha = 0.06 + 0.5 * (j / 24);
      ctx.beginPath(); ctx.moveTo(qa.x, qa.y); ctx.lineTo(qb.x, qb.y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    if (lv("physics/gs_contact") >= 0.5) {
      var stn0 = O.sites[0], gp = P(siteEci(stn0.lat, stn0.lon, t));
      ctx.strokeStyle = s2; ctx.lineWidth = 1.5; ctx.globalAlpha = 0.8;
      ctx.beginPath(); ctx.moveTo(gp.x, gp.y); ctx.lineTo(sp.x, sp.y);
      ctx.stroke(); ctx.globalAlpha = 1;
    }
    ctx.globalAlpha = occluded(sp) ? 0.35 : 1;
    ctx.beginPath(); ctx.arc(sp.x, sp.y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = s1; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
    ctx.globalAlpha = 1;

    var gx = cx + (sv[0] / sl) * sc * 1.22,
        gy = cy - (sv[2] / sl) * sc * 1.22;
    ctx.globalAlpha = -sv[1] >= 0 ? 0.95 : 0.55;
    ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI);
    ctx.fillStyle = s4; ctx.fill();
    for (var r8 = 0; r8 < 8; r8++) {
      var an = r8 * Math.PI / 4;
      ctx.beginPath();
      ctx.moveTo(gx + Math.cos(an) * 8, gy + Math.sin(an) * 8);
      ctx.lineTo(gx + Math.cos(an) * 11, gy + Math.sin(an) * 11);
      ctx.strokeStyle = s4; ctx.lineWidth = 1.4; ctx.stroke();
    }
    ctx.globalAlpha = 1;

    $("orbchip").textContent = "orbit " + (t / PERIOD_O).toFixed(2);
    var ecl = inShadow(rNow);
    $("eclchip").textContent = ecl ? "eclipse" : "sunlit";
    $("eclchip").className = "chip" + (ecl ? "" : " on");
    var con = lv("physics/gs_contact") >= 0.5;
    $("conchip").textContent = con ? "in contact" : "no contact";
    $("conchip").className = "chip" + (con ? " on" : "");
    // sub-satellite point vs the SAA box (same box the fault injector uses)
    var rl = Math.hypot(rNow[0], rNow[1], rNow[2]) || 1;
    var rlat = Math.asin(rNow[2] / rl) * 180 / Math.PI;
    var rlon = (Math.atan2(rNow[1], rNow[0]) - gmst) * 180 / Math.PI;
    rlon = ((rlon % 360) + 540) % 360 - 180;
    var saaNow = rlat >= la0 && rlat <= la1 && rlon >= lo0 && rlon <= lo1;
    $("saachip").textContent = saaNow ? "in SAA" : "outside SAA";
    $("saachip").className = "chip" + (saaNow ? " on" : "");
  }
  var dragging = false, lx = 0, ly = 0;
  canvas.addEventListener("pointerdown", function (ev) {
    dragging = true; lx = ev.clientX; ly = ev.clientY;
    canvas.setPointerCapture(ev.pointerId);
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!dragging) return;
    vs.yaw += (ev.clientX - lx) * 0.008;
    vs.pitch = Math.max(-1.35,
      Math.min(1.35, vs.pitch + (ev.clientY - ly) * 0.008));
    lx = ev.clientX; ly = ev.clientY;
    if (reduced) render(displayTime());
  });
  canvas.addEventListener("pointerup", function () { dragging = false; });
  canvas.addEventListener("pointercancel", function () { dragging = false; });
  return { render: render };
})();

/* ---- close-up attitude view: true attitude over an orbital scene --------
   The globe shows WHERE the satellite is; this shows HOW it is oriented in
   sunlight. The recorded body-from-ECI quaternion (physics/q0..q3) rotates
   the body into ECI where the sun sits, so which faces are lit and whether
   it is in eclipse are physically real. The Earth limb + starfield are a
   fixed "postcard" backdrop; technical cues (sun-axis arrow, tumble arcs)
   hide behind the `tech` chip. Hand-rolled 2D approximation, no 3D engine. */
var closeup = (function () {
  var host = $("closeup"), O = BOOT.orbit3d;
  var canvas = document.createElement("canvas");
  host.appendChild(canvas);
  // pick a "home" camera that lifts the sun into the sky (never into Earth),
  // whatever the mission epoch's sun direction happens to be
  function homeFromSun() {
    var s = O.sun, best = { yaw: 0.6, pitch: 0.32 }, bestScore = -1e9, yi, pi;
    for (yi = 0; yi < 48; yi++) {
      var yy = -Math.PI + yi * Math.PI / 24;
      var cc = Math.cos(yy), sn = Math.sin(yy);
      var x = s[0]*cc - s[1]*sn, y = s[0]*sn + s[1]*cc, z = s[2];
      for (pi = 0; pi <= 20; pi++) {
        var pp = -1.0 + pi * 0.1, cp = Math.cos(pp), sp = Math.sin(pp);
        var up = y*sp + z*cp, depth = y*cp - z*sp;   // up = screen +z, depth +y
        var score = up - 0.20*Math.max(0, depth) - 0.15*Math.abs(pp) + 0.30*x;
        if (score > bestScore) { bestScore = score; best = { yaw: yy, pitch: pp }; }
      }
    }
    return best;
  }
  var home = homeFromSun();
  var vs = { yaw: home.yaw, pitch: home.pitch };
  var showTech = false, lastInteract = 0;
  var W = 0, H = 0, cx = 0, cy = 0, sc = 1, ctx = null;
  function size() {
    var w = host.clientWidth;
    if (w < 40) return false;
    var h = Math.max(220, Math.min(400, Math.round(w * 0.62)));
    var dpr = window.devicePixelRatio || 1;
    if (W !== w || H !== h) {
      W = w; H = h;
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
      canvas.style.width = w + "px"; canvas.style.height = h + "px";
      cx = W / 2; cy = H / 2;
      sc = (Math.min(W, H) / 2 - 10) / 2.8;
      ctx = canvas.getContext("2d");
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }
    return true;
  }
  function rot(v) {                 // camera yaw/pitch, shared with the globe
    var c = Math.cos(vs.yaw), sn = Math.sin(vs.yaw);
    var x = v[0] * c - v[1] * sn, y = v[0] * sn + v[1] * c, z = v[2];
    var cp = Math.cos(vs.pitch), sp = Math.sin(vs.pitch);
    return [x, y * cp - z * sp, y * sp + z * cp];
  }
  function P(v) {
    var r = rot(v);
    return { x: cx + r[0] * sc, y: cy - r[2] * sc, d: -r[1] };
  }
  // body-from-ECI DCM, scalar-first — mirrors dcm_from_quat in attitude.py
  function dcm(q) {
    var w = q[0], x = q[1], y = q[2], z = q[3];
    return [[w*w+x*x-y*y-z*z, 2*(x*y+w*z),     2*(x*z-w*y)],
            [2*(x*y-w*z),     w*w-x*x+y*y-z*z, 2*(y*z+w*x)],
            [2*(x*z+w*y),     2*(y*z-w*x),     w*w-x*x-y*y+z*z]];
  }
  function toEci(A, v) {            // ECI image of a body vector = A^T v
    return [A[0][0]*v[0]+A[1][0]*v[1]+A[2][0]*v[2],
            A[0][1]*v[0]+A[1][1]*v[1]+A[2][1]*v[2],
            A[0][2]*v[0]+A[1][2]*v[1]+A[2][2]*v[2]];
  }
  function dot(a, b) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
  function col(a) { return "rgb(" + (a[0]|0) + "," + (a[1]|0) + "," + (a[2]|0) + ")"; }
  function mix(a, b, t) {
    return [a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t, a[2]+(b[2]-a[2])*t];
  }
  function arrow(a, bx, by, col, wid) {
    ctx.strokeStyle = col; ctx.lineWidth = wid;
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(bx, by); ctx.stroke();
    var ang = Math.atan2(by - a.y, bx - a.x), h = 7;
    ctx.beginPath(); ctx.moveTo(bx, by);
    ctx.lineTo(bx - h*Math.cos(ang - 0.4), by - h*Math.sin(ang - 0.4));
    ctx.lineTo(bx - h*Math.cos(ang + 0.4), by - h*Math.sin(ang + 0.4));
    ctx.closePath(); ctx.fillStyle = col; ctx.fill();
  }

  // 3U bus (long along +Z), unit corners scaled by BUS; six outward faces
  var BUS = [0.42, 0.42, 1.25], MET = [150, 160, 176];
  var faces = [
    { n:[0,0,1],  v:[[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]] },
    { n:[0,0,-1], v:[[-1,-1,-1],[-1,1,-1],[1,1,-1],[1,-1,-1]] },
    { n:[1,0,0],  v:[[1,-1,-1],[1,1,-1],[1,1,1],[1,-1,1]] },
    { n:[-1,0,0], v:[[-1,-1,-1],[-1,-1,1],[-1,1,1],[-1,1,-1]] },
    { n:[0,1,0],  v:[[-1,1,-1],[-1,1,1],[1,1,1],[1,1,-1]] },
    { n:[0,-1,0], v:[[-1,-1,-1],[1,-1,-1],[1,-1,1],[-1,-1,1]] }
  ];
  // two deployed panels in the body XY-plane, normal +Z (== PANEL_NORMAL_BODY)
  var PANEL = [30, 52, 130];
  var panels = [
    [[0.42,-0.55,0.03],[1.75,-0.55,0.03],[1.75,0.55,0.03],[0.42,0.55,0.03]],
    [[-0.42,-0.55,0.03],[-0.42,0.55,0.03],[-1.75,0.55,0.03],[-1.75,-0.55,0.03]]
  ];

  // deterministic starfield + clouds (seeded, so they don't jitter each frame)
  function rng(s) { return function () { s |= 0; s = s + 0x6D2B79F5 | 0;
    var x = Math.imul(s ^ s >>> 15, 1 | s); x = x + Math.imul(x ^ x >>> 7, 61 | x) ^ x;
    return ((x ^ x >>> 14) >>> 0) / 4294967296; }; }
  var _r = rng(1927), stars = [], clouds = [], _i;
  for (_i = 0; _i < 80; _i++) stars.push({ x: _r(), y: _r() * 0.6,
    r: 0.4 + _r() * 1.1, a: 0.25 + _r() * 0.6 });
  for (_i = 0; _i < 12; _i++) clouds.push({ x: _r(), y: 0.10 + _r() * 0.78,
    r: 24 + _r() * 46, a: 0.14 + _r() * 0.2 });
  var land = [];   // dim ocean-tone patches beneath the clouds (parallax layer)
  for (_i = 0; _i < 9; _i++) land.push({ x: _r(), y: 0.16 + _r() * 0.78,
    r: 44 + _r() * 74, a: 0.10 + _r() * 0.12, dark: _r() < 0.5 });

  // scene materials — always a dark space view, independent of page theme
  var SPACE_TOP=[6,8,16], SPACE_LOW=[12,18,40];
  var OCEAN_HI=[36,96,176], OCEAN_LO=[9,32,84];
  var NIGHT_HI=[10,22,44], NIGHT_LO=[4,9,22];
  var BUS_DK=[26,28,34], BUS_LT=[208,204,190];   // charcoal bus -> sunlit metal
  var PAN_DK=[18,26,58], PAN_LT=[120,152,224];   // dark cells -> lit blue
  var GOLD=[255,206,120], SUNCORE=[255,248,232];
  var HORIZON = 0.60;   // Earth's top edge, as a fraction of card height

  // one scrolling layer of soft blobs (ocean tone or cloud), wrapped in x
  function drawBlobs(arr, drift, yBand, ecl, cloud) {
    for (var i = 0; i < arr.length; i++) {
      var b = arr[i], base = b.x - drift; base = base - Math.floor(base);
      var yy = H * HORIZON + b.y * yBand;
      var a = b.a * (ecl ? (cloud ? 0 : 0.5) : 1);
      if (a <= 0) continue;
      var cc = cloud ? "240,244,250" : (b.dark ? "6,26,66" : "70,130,205");
      for (var w = -1; w <= 1; w++) {
        var bx = base * W + w * W;
        if (bx < -b.r || bx > W + b.r) continue;
        var rg = ctx.createRadialGradient(bx, yy, 0, bx, yy, b.r);
        rg.addColorStop(0, "rgba(" + cc + "," + a + ")");
        rg.addColorStop(1, "rgba(" + cc + ",0)");
        ctx.fillStyle = rg;
        ctx.beginPath(); ctx.ellipse(bx, yy, b.r, b.r * 0.5, 0, 0, 2*Math.PI); ctx.fill();
      }
    }
  }
  function drawScene(ecl, t) {
    var g = ctx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, col(SPACE_TOP)); g.addColorStop(1, col(SPACE_LOW));
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
    var s, st;
    for (s = 0; s < stars.length; s++) {
      st = stars[s];
      ctx.globalAlpha = st.a * (ecl ? 1 : 0.85); ctx.fillStyle = "#dfe7ff";
      ctx.beginPath(); ctx.arc(st.x * W, st.y * H, st.r, 0, 2*Math.PI); ctx.fill();
    }
    ctx.globalAlpha = 1;
    // Earth disc — huge radius centred far below, only its top cap shows
    var eR = Math.max(W, H) * 1.7, eCy = H * HORIZON + eR, yBand = H - H * HORIZON;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, eCy, eR, 0, 2*Math.PI);
    var eg = ctx.createLinearGradient(0, H * HORIZON, 0, H);
    eg.addColorStop(0, col(ecl ? NIGHT_HI : OCEAN_HI));
    eg.addColorStop(1, col(ecl ? NIGHT_LO : OCEAN_LO));
    ctx.fillStyle = eg; ctx.fill(); ctx.clip();
    // surface features scroll to sell orbital motion; clouds drift a touch
    // faster than the ocean tone beneath (parallax). Speed tracks orbital
    // phase, so it scales with sim speed automatically (slow at 1x).
    var ph = t * O.n_rad_s;
    drawBlobs(land, ph * 0.36, yBand, ecl, false);
    drawBlobs(clouds, ph * 0.52, yBand, ecl, true);
    ctx.restore();
    ctx.save();                                    // atmosphere rim glow
    ctx.strokeStyle = "rgba(130,190,255,0.55)";
    ctx.shadowColor = "rgba(130,190,255,0.9)"; ctx.shadowBlur = 18;
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.arc(cx, eCy, eR + 1.5, 0, 2*Math.PI); ctx.stroke();
    ctx.restore();
  }

  function drawSun(sunx, suny, behind) {
    var bloom = ctx.createRadialGradient(sunx, suny, 0, sunx, suny, 130);
    bloom.addColorStop(0, "rgba(255,224,150," + (behind ? 0.32 : 0.6) + ")");
    bloom.addColorStop(0.4, "rgba(255,205,120,0.2)");
    bloom.addColorStop(1, "rgba(255,205,120,0)");
    ctx.fillStyle = bloom; ctx.fillRect(0, 0, W, H);
    ctx.globalAlpha = behind ? 0.25 : 0.5; ctx.strokeStyle = col(GOLD);
    ctx.lineWidth = 1.4;
    for (var k = 0; k < 12; k++) {
      var an = k * Math.PI / 6, r0 = 22, r1 = 32 + (k % 2) * 9;
      ctx.beginPath();
      ctx.moveTo(sunx + Math.cos(an) * r0, suny + Math.sin(an) * r0);
      ctx.lineTo(sunx + Math.cos(an) * r1, suny + Math.sin(an) * r1);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    var core = ctx.createRadialGradient(sunx, suny, 0, sunx, suny, 15);
    core.addColorStop(0, col(SUNCORE)); core.addColorStop(0.6, col(GOLD));
    core.addColorStop(1, "rgba(255,190,90,0.15)");
    ctx.fillStyle = core;
    ctx.beginPath(); ctx.arc(sunx, suny, 15, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = "rgba(255,226,158,0.92)"; ctx.font = "11px system-ui, sans-serif";
    ctx.fillText("Sun", sunx + 20, suny + 4);
  }

  function render(t) {
    if (!size()) return;
    // after a few idle seconds, drift the camera back to its home pose
    if (!dragging && performance.now() - lastInteract > 2500) {
      var dyaw = Math.atan2(Math.sin(home.yaw - vs.yaw), Math.cos(home.yaw - vs.yaw));
      vs.yaw += dyaw * 0.08;
      vs.pitch += (home.pitch - vs.pitch) * 0.08;
    }
    var ecl = lv("physics/eclipse") >= 0.5;
    var sun = O.sun;
    var sr = rot(sun), sdx = sr[0], sdy = -sr[2], sl = Math.hypot(sdx, sdy) || 1;
    var Rs = Math.min(W, H) * 0.40;
    var sunx = cx + sdx/sl*Rs, suny = cy + sdy/sl*Rs, behind = sr[1] > 0;
    var skyMax = H * HORIZON - 18;             // keep the sun disk out of Earth
    if (suny > skyMax) suny = skyMax;

    drawScene(ecl, t);
    if (!ecl) drawSun(sunx, suny, behind);

    var q0 = lv("physics/q0"), q1 = lv("physics/q1"),
        q2 = lv("physics/q2"), q3 = lv("physics/q3");
    if (q0 == null || q1 == null || q2 == null || q3 == null) {
      ctx.fillStyle = "rgba(220,225,235,0.8)";
      ctx.font = "12px system-ui, sans-serif"; ctx.textAlign = "center";
      ctx.fillText("awaiting attitude telemetry…", cx, H * 0.42);
      ctx.textAlign = "left";
      return;
    }
    var qm = Math.hypot(q0, q1, q2, q3) || 1;
    var q = [q0/qm, q1/qm, q2/qm, q3/qm], A = dcm(q);

    // collect bus faces + panels, paint back-to-front (painter's algorithm)
    var prims = [];
    faces.forEach(function (f) {
      var neci = toEci(A, f.n), b = ecl ? 0 : Math.max(0, dot(neci, sun));
      var pts = f.v.map(function (c) {
        return P(toEci(A, [c[0]*BUS[0], c[1]*BUS[1], c[2]*BUS[2]])); });
      var d = (pts[0].d + pts[1].d + pts[2].d + pts[3].d) / 4;
      prims.push({ pts: pts, d: d, edge: true,
        fill: col(mix(BUS_DK, BUS_LT, ecl ? 0.06 : 0.10 + 0.90*b)) });
    });
    var panelLit = ecl ? 0 : Math.max(0, dot(toEci(A, [0,0,1]), sun));
    panels.forEach(function (p) {
      var pts = p.map(function (c) { return P(toEci(A, c)); });
      var d = (pts[0].d + pts[1].d + pts[2].d + pts[3].d) / 4;
      prims.push({ pts: pts, d: d, panel: true, lit: panelLit,
        fill: col(mix(PAN_DK, PAN_LT, ecl ? 0.06 : 0.10 + 0.90*panelLit)) });
    });
    prims.sort(function (a, b) { return a.d - b.d; });
    prims.forEach(function (pr) {
      ctx.beginPath();
      ctx.moveTo(pr.pts[0].x, pr.pts[0].y);
      for (var i = 1; i < pr.pts.length; i++) ctx.lineTo(pr.pts[i].x, pr.pts[i].y);
      ctx.closePath();
      ctx.fillStyle = pr.fill; ctx.fill();
      if (pr.edge) {
        ctx.strokeStyle = "rgba(0,0,0,0.35)"; ctx.lineWidth = 1; ctx.stroke();
      }
      if (pr.panel) {                       // cell grid + gold sheen when lit
        ctx.strokeStyle = "rgba(120,150,220,0.5)"; ctx.lineWidth = 0.6;
        for (var g = 1; g < 4; g++) {
          var ax = pr.pts[0].x + (pr.pts[1].x - pr.pts[0].x) * g/4;
          var ay = pr.pts[0].y + (pr.pts[1].y - pr.pts[0].y) * g/4;
          var bx = pr.pts[3].x + (pr.pts[2].x - pr.pts[3].x) * g/4;
          var by = pr.pts[3].y + (pr.pts[2].y - pr.pts[3].y) * g/4;
          ctx.beginPath(); ctx.moveTo(ax, ay); ctx.lineTo(bx, by); ctx.stroke();
        }
        if (pr.lit > 0.5) {
          ctx.globalAlpha = (pr.lit - 0.5) * 1.6;
          ctx.strokeStyle = col(GOLD); ctx.lineWidth = 1.6; ctx.stroke();
          ctx.globalAlpha = 1;
        }
      }
    });

    var rate = lv("physics/rate_dps");
    if (showTech) {
      var origin = P([0, 0, 0]);
      // +Z panel-normal arrow: where the array is aimed (at the sun => 100%)
      var zt = P(toEci(A, [0, 0, 1.9]));
      ctx.setLineDash([4, 3]);
      arrow(origin, zt.x, zt.y, col(GOLD), 1.6);
      ctx.setLineDash([]);
      ctx.fillStyle = col(GOLD); ctx.font = "10px system-ui, sans-serif";
      ctx.fillText("+Z array", zt.x + 5, zt.y);
      // tumble indicator: spin arcs, warming past the 0.5°/s detumble gate
      if (rate != null && rate > 0.03) {
        var rc = rate < 0.5 ? "#5fd08a" : rate < 2.0 ? "#f5c451" : "#ef8a5a";
        ctx.strokeStyle = rc; ctx.lineWidth = 2; ctx.globalAlpha = 0.9;
        var rr = sc * 1.85;
        for (var s = 0; s < 2; s++) {
          var a0 = s*Math.PI + 0.3, a1 = s*Math.PI + 1.5;
          ctx.beginPath(); ctx.arc(cx, cy, rr, a0, a1); ctx.stroke();
          var hx = cx + Math.cos(a1)*rr, hy = cy + Math.sin(a1)*rr;
          var ta = a1 + Math.PI/2;
          ctx.beginPath(); ctx.moveTo(hx, hy);
          ctx.lineTo(hx - 6*Math.cos(ta-0.4), hy - 6*Math.sin(ta-0.4));
          ctx.lineTo(hx - 6*Math.cos(ta+0.4), hy - 6*Math.sin(ta+0.4));
          ctx.closePath(); ctx.fillStyle = rc; ctx.fill();
        }
        ctx.globalAlpha = 1;
      }
    }

    if (ecl) {                              // cool wash + label when in shadow
      ctx.fillStyle = "rgba(18,36,84,0.30)"; ctx.fillRect(0, 0, W, H);
      ctx.fillStyle = "rgba(200,210,232,0.8)"; ctx.font = "11px system-ui, sans-serif";
      ctx.fillText("in eclipse — no sunlight", 10, H - 10);
    }

    // chips
    var sf = lv("physics/sun_facing");
    $("attlit").textContent = ecl ? "ECLIPSE" : "SUNLIT";
    $("attlit").className = "chip" + (ecl ? "" : " on");
    $("attsun").textContent = "sun " + (sf == null ? "—"
      : Math.round(Math.max(0, sf) * 100) + "%");
    $("attsun").className = "chip" + (sf != null && sf > 0.02 ? " on" : "");
    $("attrate").textContent = "rate " + (rate == null ? "—"
      : rate.toFixed(2) + " °/s");
    $("attrate").className = "chip" + (rate != null && rate <= 0.5 ? " on" : "");
  }

  var techBtn = $("atttech");
  if (techBtn) techBtn.addEventListener("click", function () {
    showTech = !showTech;
    techBtn.className = "chip" + (showTech ? " on" : "");
    techBtn.textContent = showTech ? "tech ✓" : "tech";
    if (reduced) render(displayTime());
  });

  var dragging = false, lx = 0, ly = 0;
  canvas.addEventListener("pointerdown", function (ev) {
    dragging = true; lastInteract = performance.now();
    lx = ev.clientX; ly = ev.clientY;
    canvas.setPointerCapture(ev.pointerId);
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!dragging) return;
    vs.yaw += (ev.clientX - lx) * 0.008;
    vs.pitch = Math.max(-1.2,
      Math.min(1.2, vs.pitch + (ev.clientY - ly) * 0.008));
    lx = ev.clientX; ly = ev.clientY;
    lastInteract = performance.now();
    if (reduced) render(displayTime());
  });
  canvas.addEventListener("pointerup", function () {
    dragging = false; lastInteract = performance.now();
  });
  canvas.addEventListener("pointercancel", function () {
    dragging = false; lastInteract = performance.now();
  });
  return { render: render };
})();

/* ---- stream ---- */
function apply(f) {
  var st = f.status;
  // re-anchor the smooth clock only when the sim actually advanced a tick
  if (st.t !== state.simTime || state.tickWall === null) {
    state.tickSimTime = st.t; state.tickWall = performance.now();
  }
  state.simTime = st.t; state.tick = st.tick; state.paused = st.paused;
  state.speed = st.speed; state.done = st.done;
  state.statusWall = performance.now();
  (f.telemetry || []).forEach(function (row) {
    var k = row[1] + "/" + row[2];
    latest[k] = [row[0], row[3]];
    var subs = laneIndex[k];
    if (subs) {
      for (var i = 0; i < subs.length; i++) {
        subs[i].pts.push(row[0], tfApply(subs[i].tf, row[3]));
      }
    }
  });
  // evict lane points that scrolled out of the window
  var cut = state.simTime - WIN - 120;
  lanes.forEach(function (lane) {
    lane.series.forEach(function (s) {
      var i = 0;
      while (i < s.pts.length && s.pts[i] < cut) i += 2;
      if (i > 0) s.pts.splice(0, i);
    });
  });
  (f.messages || []).forEach(function (m) {
    busUpdate(m);
    if (m.link) linkPush(m);
  });
  (f.events || []).forEach(evPush);
  if (f.findings) drawFindings(f.findings);
  drawControls(); drawTiles(); drawPills(); drawLanes(); drawAllTable();
  if (reduced) { drawClock(); globe.render(displayTime());
                 closeup.render(displayTime()); }
}
var es = new EventSource("/events");
es.onmessage = function (e) { apply(JSON.parse(e.data)); };
es.onopen = function () { $("conn").className = "conn ok"; };
es.onerror = function () { $("conn").className = "conn err"; };

if (!reduced) {
  (function frame() {
    drawClock();
    globe.render(displayTime());
    closeup.render(displayTime());
    requestAnimationFrame(frame);
  })();
} else {
  drawClock();
  globe.render(0);
  closeup.render(0);
}
</script>
</body>
</html>
"""


if __name__ == "__main__":
    main()
