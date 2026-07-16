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


def test_eps_and_obc_hk_round_trip():
    eps = ccsds.unpack_eps_hk(ccsds.pack_eps_hk(0.4321, 7.912, 5.5, 4.25, True))
    assert abs(eps["soc_est"] - 0.4321) < 1e-4
    assert abs(eps["battery_v"] - 7.912) < 1e-3
    assert abs(eps["solar_w"] - 5.5) < 1e-3
    assert abs(eps["load_w"] - 4.25) < 1e-3
    assert eps["shedding"] is True

    obc = ccsds.unpack_obc_hk(
        ccsds.pack_obc_hk(True, adcs_on=False, payload_on=True))
    assert obc == {"safe": True, "adcs_on": False, "payload_on": True}


def test_hk_words_saturate_never_wrap():
    """A SEU-corrupted 1e300-volt reading must pack as full-scale, not
    wrap into a small plausible number."""
    eps = ccsds.unpack_eps_hk(
        ccsds.pack_eps_hk(7.0, 1e300, -3.0, float("inf"), False))
    assert eps["soc_est"] == 6.5535    # saturated word, decoded honestly
    assert eps["battery_v"] == 65.535
    assert eps["solar_w"] == 0.0       # negative clips to zero
    assert eps["load_w"] == 65.535


def test_multi_packet_housekeeping_frame():
    """One beacon frame carries one packet per subsystem, demuxed by APID."""
    packets = [
        ccsds.build_space_packet(ccsds.APID_BEACON, 3,
                                 ccsds.pack_beacon(0.1, 0, 0, 25.6, 0, 0)),
        ccsds.build_space_packet(ccsds.APID_EPS_HK, 3,
                                 ccsds.pack_eps_hk(0.8, 8.1, 6.0, 4.0, False)),
        ccsds.build_space_packet(ccsds.APID_OBC_HK, 3,
                                 ccsds.pack_obc_hk(False, True, True)),
    ]
    parsed = ccsds.parse_tm_frame(ccsds.build_tm_frame(0, 9, 9, packets))
    assert parsed["crc_ok"]
    assert [p["apid"] for p in parsed["packets"]] == [
        ccsds.APID_BEACON, ccsds.APID_EPS_HK, ccsds.APID_OBC_HK]


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
