"""Ground segment: the station, its frame decoder, and its operator.

Phase 6b: the downlink is a bitstream now. The station decodes CCSDS-style
TM transfer frames (cubesat_sim.ccsds — the independent Python mirror of
the C++ flight framer): checks the sync marker and CRC, rejects corrupted
frames, tracks virtual-channel frame counters so *missing* frames are
observable as sequence gaps, unpacks housekeeping beacons, and accounts
VC1 science bursts into the archive minus their corrupted frames.

The operator rule is unchanged in spirit — storage pressure seen in
telemetry drives payload enable/disable — but the uplink is a real ARQ
loop: commands go up as CRC'd, sequence-numbered TC frames, retransmitted
until the beacon's acceptance counter (tc_ack) shows the spacecraft's
FARM took them. Every decision still acts on stale data with hours of
actuation delay; now the retries are visible protocol, not blind faith.
"""

from __future__ import annotations

from cubesat_sim import ccsds
from cubesat_sim.kernel.component import Component


class GroundStation(Component):
    def __init__(
        self,
        disable_above_frac: float = 0.8,
        enable_below_frac: float = 0.4,
        resend_every_s: float = 90.0,
    ) -> None:
        super().__init__("ground", period=1.0)
        self.disable_above_frac = disable_above_frac
        self.enable_below_frac = enable_below_frac
        self.resend_every_s = resend_every_s
        self.archive_mb = 0.0
        self.telemetry_frames = 0     # beacons decoded
        self.frames_ok = 0            # frames accepted (both VCs)
        self.frames_rejected = 0      # CRC/sync failures, either VC
        self.seq_gaps = 0             # frame-counter discontinuities seen
        self.tc_retransmits = 0
        self.last_storage_frac: float | None = None
        self.desired_enable: bool | None = None  # None: no opinion yet
        self._last_ack: int | None = None
        self._vc0_expect: int | None = None
        self._vc1_expect: int | None = None
        self._tc_seq = 0              # next sequence number to assign
        self._pending: tuple[int, int, int] | None = None  # (seq, cmd, arg)
        self._sent_once = False
        self._last_sent: float | None = None

    def on_start(self) -> None:
        self.subscribe("radio/rx_ground")

    # -- frame handling -------------------------------------------------------

    def _handle_tm_frame(self, hex_str: str) -> None:
        frame = bytes.fromhex(hex_str)
        parsed = ccsds.parse_tm_frame(frame)
        if not parsed["crc_ok"]:
            self.frames_rejected += 1
            self.event("frame_reject",
                       reason="crc" if parsed["asm_ok"] else "sync")
            return
        self.frames_ok += 1
        if parsed["vcid"] != ccsds.VC_HOUSEKEEPING:
            return
        if self._vc0_expect is not None and parsed["vc_count"] != self._vc0_expect:
            missed = (parsed["vc_count"] - self._vc0_expect) % 256
            self.seq_gaps += 1
            self.event("vc0_gap", missed=missed)
        self._vc0_expect = (parsed["vc_count"] + 1) % 256
        for pkt in parsed["packets"]:
            if pkt["apid"] == ccsds.APID_BEACON:
                hk = ccsds.unpack_beacon(pkt["data"])
                self.telemetry_frames += 1
                self.last_storage_frac = hk["storage_frac"]
                self._last_ack = hk["tc_ack"]

    def _handle_vc1_burst(self, data: dict) -> None:
        n = int(data.get("frames", 0))
        bad = int(data.get("bad", 0))
        vcfc0 = int(data.get("vcfc0", 0))
        if self._vc1_expect is not None and vcfc0 != self._vc1_expect:
            self.seq_gaps += 1
            self.event("vc1_gap", missed=vcfc0 - self._vc1_expect)
        self._vc1_expect = vcfc0 + n
        self.frames_ok += n - bad
        self.frames_rejected += bad
        if n > 0:
            self.archive_mb += float(data.get("mb", 0.0)) * (n - bad) / n

    # -- step -----------------------------------------------------------------

    def step(self, t: float, dt: float) -> None:
        for msg in self.drain():
            kind = msg.data.get("kind")
            if kind == "tm_frame":
                self._handle_tm_frame(msg.data["hex"])
            elif kind == "vc1_burst":
                self._handle_vc1_burst(msg.data)

        # ARQ: has the spacecraft's FARM accepted our pending command?
        if (self._pending is not None and self._last_ack is not None
                and ccsds.seq_acked(self._last_ack, self._pending[0])):
            self.event("uplink_acked", seq=self._pending[0])
            self._pending = None

        if self.last_storage_frac is not None:
            if (self.desired_enable is not False
                    and self.last_storage_frac > self.disable_above_frac):
                self.desired_enable = False
                self._queue_command(ccsds.CMD_PAYLOAD_ENABLE, 0)
                self.event("operator_disable_payload",
                           storage_frac=self.last_storage_frac)
            elif (self.desired_enable is False
                    and self.last_storage_frac < self.enable_below_frac):
                self.desired_enable = True
                self._queue_command(ccsds.CMD_PAYLOAD_ENABLE, 1)
                self.event("operator_enable_payload",
                           storage_frac=self.last_storage_frac)

        if self._pending is not None and (
                self._last_sent is None
                or t - self._last_sent >= self.resend_every_s):
            seq, cmd, arg = self._pending
            # blind transmit; the channel decides whether it arrives
            self.publish("ground/tx",
                         hex=ccsds.build_tc_frame(seq, cmd, arg).hex())
            if self._sent_once:
                self.tc_retransmits += 1
            self._sent_once = True
            self._last_sent = t

        self.record("archive_mb", self.archive_mb)
        self.record("telemetry_frames", float(self.telemetry_frames))
        self.record("frames_ok", float(self.frames_ok))
        self.record("frames_rejected", float(self.frames_rejected))
        self.record("seq_gaps", float(self.seq_gaps))
        self.record("tc_retransmits", float(self.tc_retransmits))

    def _queue_command(self, cmd_id: int, arg: int) -> None:
        self._pending = (self._tc_seq, cmd_id, arg)
        self._tc_seq = (self._tc_seq + 1) % 256
        self._sent_once = False
        self._last_sent = None  # transmit at the next step