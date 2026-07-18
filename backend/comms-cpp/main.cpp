// Comms flight software — C++ port of a radio/CTO stack.
//
// Phase 6b: the link is a real protocol now. Housekeeping telemetry goes
// out as byte-true CCSDS-style TM transfer frames on virtual channel 0
// (sync marker, frame counters, space packets, CRC-16 FECF), built here
// by hand the way a flight framer would. Bulk science data drains on
// virtual channel 1 as frame-accounted bursts — counters real, payload
// bytes not materialized. Uplink telecommands arrive as TC frames; a
// FARM-style acceptance loop checks CRC and sequence, dispatches exactly
// once, and reports its next-expected sequence in every beacon so the
// ground's ARQ can see what landed.
//
// The Python mirror of these formats lives in cubesat_sim/ccsds.py (the
// ground segment) — two independent implementations of one contract,
// interop pinned by the integration tests.
//
// Style notes, SDR-stack flavored: explicit state, no third-party JSON,
// fixed message shapes parsed with targeted scanners. Deterministic: no
// wall clock, no randomness.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr double kQueueCapacityMb = 256.0;
constexpr double kDownlinkRateMbS = 0.25;
constexpr double kBeaconPeriodS = 30.0;

constexpr uint16_t kScid = 0x0C5;
constexpr int kTmFrameLen = 96;
constexpr int kTmDataLen = 84;
constexpr uint16_t kApidBeacon = 0x020;   // comms-local health
constexpr uint16_t kApidEpsHk = 0x040;    // power subsystem health words
constexpr uint16_t kApidObcHk = 0x060;    // mode + load-request state
constexpr uint8_t kCmdPayloadEnable = 0x01;

bool contains(const std::string& s, const char* needle) {
    return s.find(needle) != std::string::npos;
}

std::vector<std::string> topic_segments(const std::string& line) {
    std::vector<std::string> segs;
    const std::string anchor = "\"topic\":\"";
    size_t pos = line.find(anchor);
    while (pos != std::string::npos) {
        pos += anchor.size();
        size_t next = line.find(anchor, pos);
        segs.push_back(line.substr(
            pos, next == std::string::npos ? line.size() - pos : next - pos));
        pos = next;
    }
    return segs;
}

bool find_num(const std::string& seg, const char* key, double* out) {
    size_t pos = seg.find(key);
    if (pos == std::string::npos) {
        return false;
    }
    *out = std::strtod(seg.c_str() + pos + std::strlen(key), nullptr);
    return true;
}

bool find_bool(const std::string& seg, const char* key, bool* out) {
    size_t pos = seg.find(key);
    if (pos == std::string::npos) {
        return false;
    }
    *out = seg.compare(pos + std::strlen(key), 4, "true") == 0;
    return true;
}

bool find_str(const std::string& seg, const char* key, std::string* out) {
    size_t pos = seg.find(key);
    if (pos == std::string::npos) {
        return false;
    }
    pos += std::strlen(key);
    size_t end = seg.find('"', pos);
    if (end == std::string::npos) {
        return false;
    }
    *out = seg.substr(pos, end - pos);
    return true;
}

// CRC-16/CCITT-FALSE — the CCSDS frame error control field.
uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ (data[i] << 8));
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000)
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}

// telemetry words clip, they never wrap: a bit-flipped 1e300 volt
// reading packs as 65535, not garbage
uint16_t sat_u16(double v) {
    if (!(v > 0.0)) {
        return 0;
    }
    if (v >= 65535.0) {
        return 65535;
    }
    return static_cast<uint16_t>(v + 0.5);
}

uint8_t* write_pkt_header(uint8_t* p, uint16_t apid, uint16_t seq,
                          uint16_t payload_len) {
    put_u16(p, apid);
    put_u16(p + 2, static_cast<uint16_t>(0xC000 | (seq & 0x3FFF)));
    put_u16(p + 4, static_cast<uint16_t>(payload_len - 1));
    return p + 6;
}

std::string to_hex(const uint8_t* data, size_t len) {
    static const char* digits = "0123456789abcdef";
    std::string out;
    out.reserve(len * 2);
    for (size_t i = 0; i < len; ++i) {
        out.push_back(digits[data[i] >> 4]);
        out.push_back(digits[data[i] & 0x0F]);
    }
    return out;
}

bool from_hex(const std::string& hex, std::vector<uint8_t>* out) {
    if (hex.size() % 2 != 0) {
        return false;
    }
    out->clear();
    out->reserve(hex.size() / 2);
    for (size_t i = 0; i < hex.size(); i += 2) {
        int hi = -1, lo = -1;
        for (int half = 0; half < 2; ++half) {
            char c = hex[i + half];
            int v = (c >= '0' && c <= '9') ? c - '0'
                  : (c >= 'a' && c <= 'f') ? c - 'a' + 10
                  : (c >= 'A' && c <= 'F') ? c - 'A' + 10 : -1;
            (half == 0 ? hi : lo) = v;
        }
        if (hi < 0 || lo < 0) {
            return false;
        }
        out->push_back(static_cast<uint8_t>((hi << 4) | lo));
    }
    return true;
}

class Comms {
  public:
    void ingest_and_step(const std::string& line) {
        double t = 0.0;
        double dt = 1.0;
        double v = 0.0;
        if (find_num(line, "\"t\":", &v)) {
            t = v;
        }
        if (find_num(line, "\"dt\":", &v)) {
            dt = v;
        }

        double incoming_mb = 0.0;
        std::vector<std::string> tc_hex;
        for (const std::string& seg : topic_segments(line)) {
            if (seg.rfind("payload/data\"", 0) == 0) {
                double mb = 0.0;
                if (find_num(seg, "\"mb\":", &mb)) {
                    incoming_mb += mb;
                }
            } else if (seg.rfind("sensors/comms/carrier\"", 0) == 0) {
                bool det = false;
                if (find_bool(seg, "\"detected\":", &det)) {
                    carrier_ = det;
                }
            } else if (seg.rfind("radio/rx_space\"", 0) == 0) {
                std::string hex;
                if (find_str(seg, "\"hex\":\"", &hex)) {
                    tc_hex.push_back(hex);
                }
            } else if (seg.rfind("eps/status\"", 0) == 0) {
                // sample power health off the bus for the housekeeping frame
                double v = 0.0;
                bool b = false;
                if (find_num(seg, "\"soc_est\":", &v)) {
                    eps_soc_ = v;
                    has_eps_ = true;
                }
                if (find_num(seg, "\"battery_v\":", &v)) {
                    eps_vbatt_ = v;
                }
                if (find_num(seg, "\"solar_w\":", &v)) {
                    eps_solar_ = v;
                }
                if (find_num(seg, "\"load_w\":", &v)) {
                    eps_load_ = v;
                }
                if (find_bool(seg, "\"shedding\":", &b)) {
                    eps_shedding_ = b;
                }
            } else if (seg.rfind("obc/mode\"", 0) == 0) {
                std::string mode;
                if (find_str(seg, "\"mode\":\"", &mode)) {
                    obc_safe_ = (mode == "SAFE");
                    has_obc_ = true;
                }
            } else if (seg.rfind("obc/request/loads\"", 0) == 0) {
                bool b = false;
                if (find_bool(seg, "\"adcs\":", &b)) {
                    obc_adcs_ = b;
                    has_obc_ = true;
                }
                if (find_bool(seg, "\"payload\":", &b)) {
                    obc_payload_ = b;
                }
            }
        }

        double space = kQueueCapacityMb - queue_mb_;
        double accepted = incoming_mb < space ? incoming_mb : space;
        queue_mb_ += accepted;
        dropped_mb_ += incoming_mb - accepted;
        bool full = queue_mb_ >= kQueueCapacityMb - 1e-9;

        // FARM: validate + sequence-check every received telecommand
        struct Dispatch { uint8_t cmd; uint8_t arg; uint8_t seq; };
        std::vector<Dispatch> dispatches;
        std::vector<std::string> tc_events;
        char ev[192];
        for (const std::string& hex : tc_hex) {
            std::vector<uint8_t> fr;
            if (!from_hex(hex, &fr) || fr.size() < 8) {
                std::snprintf(ev, sizeof ev,
                    "{\"kind\":\"tc_reject\",\"detail\":{\"reason\":\"malformed\"}}");
                tc_events.push_back(ev);
                continue;
            }
            uint16_t fecf = static_cast<uint16_t>(
                (fr[fr.size() - 2] << 8) | fr[fr.size() - 1]);
            if (crc16(fr.data(), fr.size() - 2) != fecf) {
                std::snprintf(ev, sizeof ev,
                    "{\"kind\":\"tc_reject\",\"detail\":{\"reason\":\"crc\"}}");
                tc_events.push_back(ev);
                continue;
            }
            uint8_t seq = fr[2];
            uint8_t cmd = fr[4];
            uint8_t arg = fr[5];
            if (seq == tc_expected_) {
                tc_expected_ = static_cast<uint8_t>(tc_expected_ + 1);
                dispatches.push_back({cmd, arg, seq});
            } else if (static_cast<uint8_t>(tc_expected_ - 1) == seq) {
                std::snprintf(ev, sizeof ev,
                    "{\"kind\":\"tc_duplicate\",\"detail\":{\"seq\":%d}}", seq);
                tc_events.push_back(ev);  // already accepted; ack re-covers it
            } else {
                std::snprintf(ev, sizeof ev,
                    "{\"kind\":\"tc_out_of_sequence\",\"detail\":"
                    "{\"seq\":%d,\"expected\":%d}}", seq, tc_expected_);
                tc_events.push_back(ev);
            }
        }

        std::printf("{\"type\":\"out\",\"pub\":[");
        bool first = true;

        // VC1: drain the science queue as a frame-accounted burst
        double tx_mb = 0.0;
        if (carrier_ && queue_mb_ > 0.0) {
            tx_mb = kDownlinkRateMbS * dt;
            if (tx_mb > queue_mb_) {
                tx_mb = queue_mb_;
            }
            queue_mb_ -= tx_mb;
            sent_mb_ += tx_mb;
            long frames = static_cast<long>(tx_mb * 1024.0 + 0.5);
            if (frames < 1) {
                frames = 1;
            }
            emit_sep(&first);
            std::printf(
                "{\"topic\":\"radio/tx\",\"data\":{\"kind\":\"vc1_burst\","
                "\"mb\":%.17g,\"frames\":%ld,\"vcfc0\":%ld}}",
                tx_mb, frames, vc1_count_);
            vc1_count_ += frames;
            mc_count_ = static_cast<uint8_t>(mc_count_ + frames);
        }

        // VC0: housekeeping beacon as a byte-true TM transfer frame
        if (last_beacon_t_ < 0.0 || t - last_beacon_t_ >= kBeaconPeriodS) {
            last_beacon_t_ = t;
            std::string hex = build_beacon_frame(full);
            emit_sep(&first);
            std::printf("{\"topic\":\"radio/tx\",\"data\":"
                        "{\"kind\":\"tm_frame\",\"hex\":\"%s\"}}", hex.c_str());
        }
        for (const Dispatch& d : dispatches) {
            if (d.cmd == kCmdPayloadEnable) {
                emit_sep(&first);
                std::printf("{\"topic\":\"payload/enable\",\"data\":{\"on\":%s}}",
                            d.arg ? "true" : "false");
            }
        }

        std::printf("],\"telemetry\":{\"queue_mb\":%.17g,\"dropped_mb\":%.17g,"
                    "\"sent_mb\":%.17g,\"carrier\":%s,\"tc_ack\":%.1f}",
                    queue_mb_, dropped_mb_, sent_mb_, carrier_ ? "1.0" : "0.0",
                    static_cast<double>(tc_expected_));

        bool first_ev = true;
        std::printf(",\"events\":[");
        if (full && !was_full_) {
            emit_sep(&first_ev);
            std::printf("{\"kind\":\"storage_full\",\"detail\":"
                        "{\"dropped_mb\":%.17g}}", dropped_mb_);
        } else if (!full && was_full_) {
            emit_sep(&first_ev);
            std::printf("{\"kind\":\"storage_recovered\",\"detail\":{}}");
        }
        was_full_ = full;
        for (const std::string& e : tc_events) {
            emit_sep(&first_ev);
            std::printf("%s", e.c_str());
        }
        for (const Dispatch& d : dispatches) {
            emit_sep(&first_ev);
            std::printf("{\"kind\":\"uplink_dispatch\",\"detail\":"
                        "{\"cmd_topic\":\"payload/enable\",\"on\":%s,\"seq\":%d}}",
                        d.arg ? "true" : "false", d.seq);
        }
        std::printf("]}\n");
        std::fflush(stdout);
    }

  private:
    static void emit_sep(bool* first) {
        if (!*first) {
            std::printf(",");
        }
        *first = false;
    }

    std::string build_beacon_frame(bool full) {
        uint8_t f[kTmFrameLen];
        std::memset(f, 0, sizeof f);
        f[0] = 0x1A; f[1] = 0xCF; f[2] = 0xFC; f[3] = 0x1D;
        put_u16(f + 4, static_cast<uint16_t>((kScid << 4) | (0 /*VC0*/ << 1)));
        f[6] = mc_count_;
        f[7] = vc0_count_;
        put_u16(f + 8, 0);
        std::memset(f + 10, 0x55, kTmDataLen);  // idle fill

        // housekeeping frame: one space packet per subsystem heard on the
        // bus, demultiplexed at the ground by APID
        uint8_t* p = f + 10;
        p = write_pkt_header(p, kApidBeacon, beacon_seq_, 16);
        double frac = queue_mb_ / kQueueCapacityMb;
        put_u16(p, static_cast<uint16_t>(frac * 10000.0 + 0.5));
        put_u32(p + 2, static_cast<uint32_t>(dropped_mb_ * 1024.0 + 0.5));
        put_u32(p + 6, static_cast<uint32_t>(sent_mb_ * 1024.0 + 0.5));
        put_u16(p + 10, static_cast<uint16_t>(queue_mb_ * 10.0 + 0.5));
        p[12] = tc_expected_;
        p[13] = static_cast<uint8_t>((full ? 1 : 0) | (carrier_ ? 2 : 0));
        put_u16(p + 14, 0);
        p += 16;
        beacon_seq_ = static_cast<uint16_t>((beacon_seq_ + 1) & 0x3FFF);
        if (has_eps_) {
            p = write_pkt_header(p, kApidEpsHk, eps_seq_, 12);
            put_u16(p, sat_u16(eps_soc_ * 10000.0));
            put_u16(p + 2, sat_u16(eps_vbatt_ * 1000.0));
            put_u16(p + 4, sat_u16(eps_solar_ * 1000.0));
            put_u16(p + 6, sat_u16(eps_load_ * 1000.0));
            p[8] = eps_shedding_ ? 1 : 0;
            p[9] = 0;
            put_u16(p + 10, 0);
            p += 12;
            eps_seq_ = static_cast<uint16_t>((eps_seq_ + 1) & 0x3FFF);
        }
        if (has_obc_) {
            p = write_pkt_header(p, kApidObcHk, obc_seq_, 6);
            p[0] = obc_safe_ ? 1 : 0;
            p[1] = static_cast<uint8_t>((obc_adcs_ ? 1 : 0) |
                                        (obc_payload_ ? 2 : 0));
            put_u32(p + 2, 0);
            p += 6;
            obc_seq_ = static_cast<uint16_t>((obc_seq_ + 1) & 0x3FFF);
        }

        put_u16(f + kTmFrameLen - 2,
                crc16(f + 4, kTmFrameLen - 4 - 2));
        vc0_count_ = static_cast<uint8_t>(vc0_count_ + 1);
        mc_count_ = static_cast<uint8_t>(mc_count_ + 1);
        return to_hex(f, sizeof f);
    }

    bool carrier_ = false;
    bool was_full_ = false;
    double queue_mb_ = 0.0;
    double dropped_mb_ = 0.0;
    double sent_mb_ = 0.0;
    double last_beacon_t_ = -1.0;
    uint8_t mc_count_ = 0;
    uint8_t vc0_count_ = 0;
    long vc1_count_ = 0;
    uint16_t beacon_seq_ = 0;
    uint16_t eps_seq_ = 0;
    uint16_t obc_seq_ = 0;
    uint8_t tc_expected_ = 0;
    // last-heard subsystem health, sampled off the bus for housekeeping
    bool has_eps_ = false;
    double eps_soc_ = 0.0;
    double eps_vbatt_ = 0.0;
    double eps_solar_ = 0.0;
    double eps_load_ = 0.0;
    bool eps_shedding_ = false;
    bool has_obc_ = false;
    bool obc_safe_ = false;
    bool obc_adcs_ = true;
    bool obc_payload_ = true;
};

}  // namespace

int main() {
    Comms comms;
    std::string line;
    line.reserve(1 << 16);

    int ch;
    while (true) {
        line.clear();
        while ((ch = std::getchar()) != EOF && ch != '\n') {
            line.push_back(static_cast<char>(ch));
        }
        if (ch == EOF && line.empty()) {
            break;
        }
        if (contains(line, "\"type\":\"init\"")) {
            std::printf("{\"type\":\"ready\",\"subscribe\":[\"payload/data\","
                        "\"sensors/comms/*\",\"radio/rx_space\",\"eps/status\","
                        "\"obc/mode\",\"obc/request/loads\"]}\n");
            std::fflush(stdout);
        } else if (contains(line, "\"type\":\"shutdown\"")) {
            break;
        } else if (contains(line, "\"type\":\"step\"")) {
            comms.ingest_and_step(line);
        }
    }
    return 0;
}
