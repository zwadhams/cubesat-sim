"""The space link, end to end: C++ framer <-> physics channel <-> Python
ground decoder. Interop between the two independent protocol
implementations is pinned here — every frame the ground decodes in these
flights was built by the C++ flight software."""

import pytest

from cubesat_sim import Simulation, ccsds
from cubesat_sim.faults import channel_ber
from cubesat_sim.mission import CPP_COMMS_BIN, build_sim
from cubesat_sim.kernel.remote import RemoteComponent

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def test_cpp_frames_decode_in_python():
    """Interop: TM frames built by the C++ framer parse cleanly with the
    ground-segment codec, headers and housekeeping payload intact."""
    sim = build_sim(dt=5.0, seed=21)
    sim.run(duration=3 * sim.components[0].orbit.period_s)
    rec = sim.recorder

    frames = [row for row in rec.messages(topic="radio/tx")
              if '"tm_frame"' in row[5]]
    assert frames, "comms never built a beacon frame"
    import json
    decoded = 0
    full_hk = 0
    for row in frames:
        parsed = ccsds.parse_tm_frame(
            bytes.fromhex(json.loads(row[5])["hex"]))
        assert parsed["asm_ok"] and parsed["crc_ok"]  # pre-channel: pristine
        assert parsed["scid"] == ccsds.SCID
        assert parsed["vcid"] == ccsds.VC_HOUSEKEEPING
        apids = [p["apid"] for p in parsed["packets"]]
        assert apids[0] == ccsds.APID_BEACON
        hk = ccsds.unpack_beacon(parsed["packets"][0]["data"])
        assert 0.0 <= hk["storage_frac"] <= 1.0
        if ccsds.APID_EPS_HK in apids and ccsds.APID_OBC_HK in apids:
            full_hk += 1
            eps = next(ccsds.unpack_eps_hk(p["data"])
                       for p in parsed["packets"]
                       if p["apid"] == ccsds.APID_EPS_HK)
            assert 0.0 <= eps["soc_est"] <= 1.0
            assert 5.0 < eps["battery_v"] < 10.0
        decoded += 1
    assert decoded >= 20
    # after the first samples arrive, every beacon carries all three
    # subsystems' packets
    assert full_hk >= decoded - 2

    # the ground decoded beacons and archived science across the channel
    assert rec.telemetry("ground", "telemetry_frames")[-1][-1] > 0
    assert rec.telemetry("ground", "frames_ok")[-1][-1] > 0
    sim.close()


def test_noisy_channel_rejects_frames_by_crc():
    """Crank the BER (scintillation fault): corrupted frames must be
    rejected by the ground CRC and show up as VC0 counter gaps."""
    sim = build_sim(dt=5.0, seed=22, faults=[channel_ber(0.0, mult=50.0)])
    sim.run(duration=4 * sim.components[0].orbit.period_s)
    rec = sim.recorder

    rejected = rec.telemetry("ground", "frames_rejected")[-1][-1]
    gaps = rec.telemetry("ground", "seq_gaps")[-1][-1]
    kinds = [e[3] for e in rec.events("ground")]
    assert rejected > 0
    assert "frame_reject" in kinds
    assert gaps > 0 and "vc0_gap" in kinds  # the miss is *observable*
    # science archive lost some frames to corruption too
    assert rec.telemetry("ground", "frames_ok")[-1][-1] > 0
    sim.close()


def test_uplink_arq_closes_the_loop():
    """Flood scenario: the disable command goes up as a TC frame, the
    FARM accepts it exactly once, the beacon acks it, the ground stops
    retransmitting."""
    sim = build_sim(dt=5.0, seed=33, payload_rate_mb_s=2.0)
    sim.run(duration=10 * sim.components[0].orbit.period_s)
    rec = sim.recorder

    dispatches = [e for e in rec.events("comms") if e[3] == "uplink_dispatch"]
    acks = [e for e in rec.events("ground") if e[3] == "uplink_acked"]
    assert dispatches and acks
    assert any(e[3] == "instrument_disable" for e in rec.events("payload"))
    # FARM accepted each unique sequence number at most once
    import json
    seqs = [json.loads(e[4])["seq"] for e in dispatches]
    assert len(seqs) == len(set(seqs))
    # after the ack, comms' next-expected count moved past the command
    assert rec.telemetry("comms", "tc_ack")[-1][-1] >= 1.0
    sim.close()


def test_farm_sequence_logic_against_real_flight_software(cpp_comms_binary):
    """Drive the C++ FARM directly: accept in order, flag duplicates,
    refuse out-of-sequence, reject bad CRC — dispatch exactly once."""
    sim = Simulation(dt=1.0, seed=0)
    sim.add(RemoteComponent("comms", period=1.0, argv=[str(CPP_COMMS_BIN)]))

    def send(frame_hex):
        sim.bus.publish("radio/rx_space", "test", {"hex": frame_hex})
        sim.run(ticks=2)

    good = ccsds.build_tc_frame(0, ccsds.CMD_PAYLOAD_ENABLE, 1)
    send(good.hex())                      # seq 0: accept
    send(good.hex())                      # seq 0 again: duplicate
    send(ccsds.build_tc_frame(5, ccsds.CMD_PAYLOAD_ENABLE, 1).hex())  # gap
    corrupt = bytearray(ccsds.build_tc_frame(1, ccsds.CMD_PAYLOAD_ENABLE, 0))
    corrupt[5] ^= 0x01
    send(corrupt.hex())                   # right seq, bad CRC
    send(ccsds.build_tc_frame(1, ccsds.CMD_PAYLOAD_ENABLE, 0).hex())  # accept

    kinds = [e[3] for e in sim.recorder.events("comms")]
    assert kinds.count("uplink_dispatch") == 2
    assert kinds.count("tc_duplicate") == 1
    assert kinds.count("tc_out_of_sequence") == 1
    assert kinds.count("tc_reject") == 1
    enables = sim.recorder.messages(topic="payload/enable")
    assert len(enables) == 2
    sim.close()
