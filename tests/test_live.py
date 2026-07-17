"""The live console: mission paced to wall clock, recording tailed over
HTTP/SSE, pause/speed controls, and replay of finished recordings.

All tests fly a lightweight all-Python mission (no flight binaries) — the
machinery under test is the runner/tailer/server, not the flight stack.
"""

import json
import math
import sqlite3
import time
import urllib.error
import urllib.request

import pytest

from cubesat_sim import Simulation, ccsds
from cubesat_sim.ground.station import GroundStation
from cubesat_sim.live import Console
from cubesat_sim.mission import build_sim

LIGHT = dict(obc_impl="python", eps_impl="python",
             adcs_impl="none", comms_impl="none")


def _get(url: str, timeout: float = 5.0):
    return urllib.request.urlopen(url, timeout=timeout)


def _post(url: str, body: dict) -> int:
    req = urllib.request.Request(url + "control",
                                 data=json.dumps(body).encode(),
                                 method="POST")
    with urllib.request.urlopen(req, timeout=5.0) as resp:
        return resp.status


def _read_until(resp, pred, timeout: float = 20.0):
    """Read SSE data frames until pred(frames) is truthy. The stream
    heartbeats every poll, so readline never blocks for long."""
    frames = []
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = resp.readline()
        if not line:
            break
        if line.startswith(b"data: "):
            frames.append(json.loads(line[6:]))
            if pred(frames):
                return frames
    raise AssertionError(f"stream never satisfied predicate; "
                         f"got {len(frames)} frames")


def _wait(cond, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return
        time.sleep(0.05)
    raise AssertionError("condition never became true")


def test_console_page_and_controls(tmp_path):
    """The page serves, and pause / speed / resume land in the runner."""
    con = Console(db=tmp_path / "live.db", port=0, speed=1.0,
                  duration=600.0, dt=5.0, seed=0, **LIGHT).start()
    try:
        with _get(con.url) as resp:
            assert resp.status == 200
            page = resp.read().decode()
        assert "Spacecraft bus" in page and "orbit3d" in page

        assert _post(con.url, {"action": "pause"}) == 204
        _wait(lambda: con.ctl.paused)
        assert _post(con.url, {"action": "speed", "value": 60}) == 204
        _wait(lambda: con.ctl.speed == 60.0)
        assert _post(con.url, {"action": "resume"}) == 204
        _wait(lambda: not con.ctl.paused)

        # the status frame reflects the control state
        with _get(con.url + "events", timeout=10.0) as resp:
            frames = _read_until(resp, lambda fs: True)
        st = frames[0]["status"]
        assert st["paused"] is False and st["speed"] == 60.0

        with pytest.raises(urllib.error.HTTPError):
            _post(con.url, {"action": "explode"})
    finally:
        con.stop()


def test_ticks_visible_while_flying(tmp_path):
    """Every tick is committed as it lands: a second read-only connection
    sees rows growing while the mission is still in the air."""
    db = tmp_path / "live.db"
    con = Console(db=db, port=0, speed=60.0, duration=3000.0, dt=5.0,
                  seed=1, **LIGHT).start()
    try:
        _wait(lambda: con.ctl.tick >= 3)
        assert not con.ctl.done
        ro = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        n1 = ro.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        assert n1 > 0
        tick1 = con.ctl.tick
        _wait(lambda: con.ctl.tick >= tick1 + 3)
        n2 = ro.execute("SELECT COUNT(*) FROM telemetry").fetchone()[0]
        ro.close()
        assert n2 > n1
    finally:
        con.stop()


def test_stream_delivers_mission(tmp_path):
    """An unpaced mission completes; the SSE stream hands a late-joining
    client the telemetry backfill and a done status."""
    con = Console(db=tmp_path / "live.db", port=0, speed=math.inf,
                  duration=300.0, dt=5.0, seed=2, **LIGHT).start()
    try:
        con.join(timeout=60)
        assert con.ctl.done

        def settled(frames):
            got_tlm = any("telemetry" in f for f in frames)
            got_msg = any("messages" in f for f in frames)
            return got_tlm and got_msg and frames[-1]["status"]["done"]

        with _get(con.url + "events", timeout=10.0) as resp:
            frames = _read_until(resp, settled)
        tlm = [row for f in frames for row in f.get("telemetry", [])]
        assert any(r[1] == "physics" and r[2] == "soc_true" for r in tlm)
        msgs = [m for f in frames for m in f.get("messages", [])]
        assert msgs and all("topic" in m and "data" in m for m in msgs)
    finally:
        con.stop()


def test_replay_streams_whole_recording(tmp_path):
    """Replay mode re-flies a finished recording: every row comes through
    the stream, in time order."""
    db = tmp_path / "old_flight.db"
    sim = build_sim(dt=5.0, seed=3, recorder_path=db, **LIGHT)
    sim.run(duration=300.0)
    n_tlm = len(sim.recorder.telemetry())
    sim.close()
    assert n_tlm > 0

    con = Console(replay=db, port=0, speed=math.inf).start()
    try:
        def drained(frames):
            got = sum(len(f.get("telemetry", [])) for f in frames)
            return got >= n_tlm and frames[-1]["status"]["done"]

        with _get(con.url + "events", timeout=10.0) as resp:
            frames = _read_until(resp, drained)
        tlm = [row for f in frames for row in f.get("telemetry", [])]
        assert len(tlm) == n_tlm
        times = [r[0] for r in tlm]
        assert times == sorted(times)
    finally:
        con.stop()


def test_ops_tc_queues_a_real_frame():
    """The command panel's ground-station hook: an ops/tc bus message
    becomes a CRC'd TC frame in the ARQ pipeline, and the operator's
    payload model tracks the manual command."""
    sim = Simulation(dt=1.0, seed=0)
    gs = sim.add(GroundStation())
    sim.bus.publish("ops/tc", "console",
                    {"cmd": ccsds.CMD_PAYLOAD_ENABLE, "arg": 0})
    sim.run(ticks=3)

    txs = sim.recorder.messages(topic="ground/tx")
    assert txs, "manual TC never transmitted"
    parsed = ccsds.parse_tc_frame(bytes.fromhex(json.loads(txs[0][5])["hex"]))
    assert parsed["crc_ok"]
    assert parsed["cmd_id"] == ccsds.CMD_PAYLOAD_ENABLE and parsed["arg"] == 0
    assert gs.desired_enable is False
    assert any(e[3] == "operator_manual_tc"
               for e in sim.recorder.events("ground"))
    sim.close()


def test_command_panel_reaches_the_sim(tmp_path):
    """POST tc / inject land on the bus between ticks: the fault message
    (sender console) and the uplinked TC frame show up in the recording,
    with console events beside them."""
    db = tmp_path / "live.db"
    con = Console(db=db, port=0, speed=60.0, duration=3000.0, dt=5.0,
                  seed=4, **LIGHT).start()
    try:
        _wait(lambda: con.ctl.tick >= 1)
        assert _post(con.url, {"action": "inject", "topic": "fault/array_hit",
                               "data": {"mult": 0.7}}) == 204
        assert _post(con.url, {"action": "tc", "cmd": 1, "arg": 0}) == 204

        ro = sqlite3.connect(f"file:{db}?mode=ro", uri=True)

        def landed():
            n = ro.execute(
                "SELECT COUNT(*) FROM messages WHERE topic='fault/array_hit' "
                "AND sender='console'").fetchone()[0]
            m = ro.execute(
                "SELECT COUNT(*) FROM messages WHERE topic='ground/tx'"
            ).fetchone()[0]
            return n > 0 and m > 0

        _wait(landed)
        hexstr = json.loads(ro.execute(
            "SELECT data FROM messages WHERE topic='ground/tx' "
            "ORDER BY rowid LIMIT 1").fetchone()[0])["hex"]
        parsed = ccsds.parse_tc_frame(bytes.fromhex(hexstr))
        assert parsed["crc_ok"] and parsed["arg"] == 0
        kinds = {row[0] for row in ro.execute(
            "SELECT kind FROM events WHERE source='console'")}
        assert kinds == {"inject", "uplink"}
        assert ro.execute(
            "SELECT COUNT(*) FROM events WHERE source='ground' "
            "AND kind='operator_manual_tc'").fetchone()[0] > 0
        ro.close()
    finally:
        con.stop()


def test_inject_door_policy_and_replay(tmp_path):
    """Malformed requests are refused, but poison payloads (JSON null,
    non-finite floats) ride through — the owner keeps that failure
    surface open on purpose (see _inject_request; the armed check sits
    in a comment there). Replay consoles stay view-only."""
    db = tmp_path / "live.db"
    con = Console(db=db, port=0, speed=30.0, duration=600.0, dt=5.0,
                  seed=5, **LIGHT).start()
    try:
        for bad in (
            {"action": "inject", "topic": "bad topic", "data": {}},
            {"action": "inject", "topic": "", "data": {}},
            {"action": "tc", "cmd": 999, "arg": 0},
        ):
            with pytest.raises(urllib.error.HTTPError) as err:
                _post(con.url, bad)
            assert err.value.code == 400
        # poison goes through the open door (aimed at an unsubscribed
        # topic here — aiming it at a live one is the experiment)
        assert _post(con.url, {"action": "inject", "topic": "chaos/poison",
                               "data": {"v": float("nan")}}) == 204
        assert _post(con.url, {"action": "inject", "topic": "chaos/poison",
                               "data": {"v": None}}) == 204
        ro = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        _wait(lambda: ro.execute(
            "SELECT COUNT(*) FROM messages WHERE topic='chaos/poison'"
        ).fetchone()[0] >= 2)
        ro.close()
    finally:
        con.stop()

    replay = Console(replay=db, port=0, speed=math.inf).start()
    try:
        with pytest.raises(urllib.error.HTTPError) as err:
            _post(replay.url, {"action": "tc", "cmd": 1, "arg": 1})
        assert err.value.code == 409
    finally:
        replay.stop()
