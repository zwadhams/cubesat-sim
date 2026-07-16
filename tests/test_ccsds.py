"""The link protocol contract: frames, packets, CRC, counters."""

from cubesat_sim import ccsds


def test_crc16_known_vector():
    # CRC-16/CCITT-FALSE of "123456789" is the classic check value 0x29B1
    assert ccsds.crc16(b"123456789") == 0x29B1


def test_tm_frame_round_trip():
    beacon = ccsds.pack_beacon(0.42, 12.5, 100.0, 107.5, tc_ack=7, flags=0b10)
    pkt = ccsds.build_space_packet(ccsds.APID_BEACON, 33, beacon)
    frame = ccsds.build_tm_frame(ccsds.VC_HOUSEKEEPING, 200, 199, [pkt])
    assert len(frame) == ccsds.TM_FRAME_LEN

    parsed = ccsds.parse_tm_frame(frame)
    assert parsed["asm_ok"] and parsed["crc_ok"]
    assert parsed["scid"] == ccsds.SCID
    assert parsed["vcid"] == ccsds.VC_HOUSEKEEPING
    assert parsed["mc_count"] == 200 and parsed["vc_count"] == 199
    assert len(parsed["packets"]) == 1
    pk = parsed["packets"][0]
    assert pk["apid"] == ccsds.APID_BEACON and pk["seq"] == 33

    hk = ccsds.unpack_beacon(pk["data"])
    assert abs(hk["storage_frac"] - 0.42) < 1e-4
    assert abs(hk["dropped_mb"] - 12.5) < 1e-3
    assert abs(hk["queue_mb"] - 107.5) < 0.05
    assert hk["tc_ack"] == 7 and hk["flags"] == 0b10


def test_tm_frame_counters_wrap():
    frame = ccsds.build_tm_frame(0, 256 + 5, 512 + 9, [])
    parsed = ccsds.parse_tm_frame(frame)
    assert parsed["mc_count"] == 5 and parsed["vc_count"] == 9
    assert parsed["packets"] == []  # pure idle fill parses clean


def test_single_bit_flip_fails_crc():
    pkt = ccsds.build_space_packet(ccsds.APID_BEACON, 0,
                                   ccsds.pack_beacon(0.5, 0, 0, 0, 0, 0))
    frame = bytearray(ccsds.build_tm_frame(0, 1, 1, [pkt]))
    for bit in (32, 200, 500, len(frame) * 8 - 1):
        flipped = bytearray(frame)
        flipped[bit // 8] ^= 1 << (bit % 8)
        assert not ccsds.parse_tm_frame(bytes(flipped))["crc_ok"]


def test_tc_frame_round_trip_and_rejection():
    frame = ccsds.build_tc_frame(seq=41, cmd_id=ccsds.CMD_PAYLOAD_ENABLE, arg=0)
    parsed = ccsds.parse_tc_frame(frame)
    assert parsed["crc_ok"] and parsed["scid"] == ccsds.SCID
    assert parsed["seq"] == 41 and parsed["cmd_id"] == 0x01 and parsed["arg"] == 0

    corrupt = bytearray(frame)
    corrupt[2] ^= 0x04
    assert not ccsds.parse_tc_frame(bytes(corrupt))["crc_ok"]


def test_seq_acked_mod256():
    assert ccsds.seq_acked(ack=42, seq=41)
    assert not ccsds.seq_acked(ack=41, seq=41)  # ack == next expected
    assert ccsds.seq_acked(ack=1, seq=255)      # wraparound
    assert not ccsds.seq_acked(ack=200, seq=60)  # far-future seq not acked
