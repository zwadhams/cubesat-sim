import json

import pytest

from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def orbit_period(sim):
    return sim.components[0].orbit.period_s


def test_data_flows_end_to_end():
    """Payload images targets -> comms queues -> passes downlink -> ground
    archives. Every stage of the pipeline must show life within 8 orbits."""
    sim = build_sim(dt=5.0, seed=31)
    sim.run(duration=8 * orbit_period(sim))
    rec = sim.recorder

    kinds = [e[3] for e in rec.events("physics")]
    assert "contact_aos" in kinds and "contact_los" in kinds

    generated = [v for *_, v in rec.telemetry("payload", "generated_mb")]
    assert generated[-1] > 10.0  # imaged something

    sent = [v for *_, v in rec.telemetry("comms", "sent_mb")]
    assert sent[-1] > 1.0  # downlinked something

    archive = [v for *_, v in rec.telemetry("ground", "archive_mb")]
    frames = [v for *_, v in rec.telemetry("ground", "telemetry_frames")]
    assert archive[-1] > 1.0  # ground actually received data
    assert frames[-1] > 0     # and telemetry beacons
    # channel loss: ground can only have less than was sent
    assert archive[-1] <= sent[-1] + 1e-6
    sim.close()


def test_no_downlink_outside_contact():
    sim = build_sim(dt=5.0, seed=32)
    sim.run(duration=6 * orbit_period(sim))
    rec = sim.recorder

    contact = {r[0]: r[4] for r in rec.telemetry("physics", "gs_contact")}
    rx = rec.messages(topic="radio/rx_ground")
    assert rx, "expected at least one pass with received frames"
    for row in rx:
        tick = row[0]
        assert contact.get(tick) == 1.0  # every delivery inside a contact
    sim.close()


def test_ground_closes_the_loop_on_storage_pressure():
    """Flood the recorder (payload rate x6): storage fills, data drops,
    the ground sees it in telemetry during a pass and uplinks
    payload/enable off — which must actually reach and stop the payload."""
    sim = build_sim(dt=5.0, seed=33, payload_rate_mb_s=2.0)
    sim.run(duration=10 * orbit_period(sim))
    rec = sim.recorder

    comms_kinds = [e[3] for e in rec.events("comms")]
    assert "storage_full" in comms_kinds
    dropped = [v for *_, v in rec.telemetry("comms", "dropped_mb")]
    assert dropped[-1] > 0.0

    assert any(e[3] == "operator_disable_payload" for e in rec.events("ground"))
    assert any(e[3] == "uplink_dispatch" for e in rec.events("comms"))
    assert any(e[3] == "instrument_disable" for e in rec.events("payload"))

    # after the disable lands, imaging stops even over targets
    disable_tick = min(e[0] for e in rec.events("payload")
                       if e[3] == "instrument_disable")
    imaging_after = [r[4] for r in rec.telemetry("payload", "imaging")
                     if r[0] > disable_tick]
    visible_after = [r[4] for r in rec.telemetry("physics", "target_visible")
                     if r[0] > disable_tick]
    assert any(v == 1.0 for v in visible_after)  # targets did come by
    assert all(v == 0.0 for v in imaging_after)  # and were not imaged
    sim.close()


def test_uplink_commands_arrive_only_during_contact():
    sim = build_sim(dt=5.0, seed=33, payload_rate_mb_s=2.0)
    sim.run(duration=10 * orbit_period(sim))
    rec = sim.recorder

    contact = {r[0]: r[4] for r in rec.telemetry("physics", "gs_contact")}
    dispatched = [e for e in rec.events("comms") if e[3] == "uplink_dispatch"]
    assert dispatched
    for e in dispatched:
        # command was dispatched the step after an in-contact delivery
        assert contact.get(e[0] - 1, contact.get(e[0])) == 1.0
    for e in dispatched:
        assert json.loads(e[4])["cmd_topic"] == "payload/enable"
    sim.close()


def test_second_station_downlinks_more_and_stays_in_contact():
    """A ground station near the imaging AOI closes the coverage gap a lone
    mid-latitude station leaves: materially more captured science reaches the
    ground, contacts happen via both stations, and every delivery still lands
    inside a contact window."""
    from cubesat_sim.physics.spacecraft import DEFAULT_STATION, TOKYO_STATION

    def archived(sim):
        r = sim.recorder.telemetry("ground", "archive_mb")
        return r[-1][4] if r else 0.0

    base = build_sim(dt=5.0, seed=1)
    base.run(duration=8 * orbit_period(base))
    dual = build_sim(dt=5.0, seed=1, stations=[DEFAULT_STATION, TOKYO_STATION])
    dual.run(duration=8 * orbit_period(dual))

    # the second station brings materially more data home (~2.4x here)
    assert archived(dual) > 1.5 * archived(base)

    # and it actually carried traffic: contacts happen via both stations
    stations = {json.loads(e[4])["station"]
                for e in dual.recorder.events("physics")
                if e[3] == "contact_aos"}
    assert {"bozeman", "tokyo_gs"} <= stations

    # invariant preserved with two stations: no delivery outside a contact
    contact = {r[0]: r[4] for r in dual.recorder.telemetry("physics", "gs_contact")}
    rx = dual.recorder.messages(topic="radio/rx_ground")
    assert rx
    for row in rx:
        assert contact.get(row[0]) == 1.0

    base.close()
    dual.close()
