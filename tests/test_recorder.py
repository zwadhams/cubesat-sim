import json

from cubesat_sim import FlightRecorder, Message


def test_roundtrip_all_streams(tmp_path):
    rec = FlightRecorder(tmp_path / "run.db")
    rec.set_meta(seed=7, dt=1.0)
    rec.log_message(Message("eps/battery", "eps", {"soc": 0.8}, tick=3, time=3.0, seq=0))
    rec.log_telemetry(3, 3.0, "eps", "soc", 0.8)
    rec.log_event(4, 4.0, "obc", "mode_change", {"to": "SAFE"})

    msgs = rec.messages(topic="eps/battery")
    assert len(msgs) == 1
    assert json.loads(msgs[0][5]) == {"soc": 0.8}

    tlm = rec.telemetry(source="eps", key="soc")
    assert tlm == [(3, 3.0, "eps", "soc", 0.8)]

    events = rec.events(source="obc")
    assert len(events) == 1
    assert events[0][3] == "mode_change"
    rec.close()


def test_in_memory_default():
    rec = FlightRecorder()
    rec.log_telemetry(0, 0.0, "x", "k", 1.0)
    assert len(rec.telemetry()) == 1
