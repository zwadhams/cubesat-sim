// Comms flight software — C++ port of a radio/CTO stack.
//
// Owns the bounded downlink queue: payload data comes in over the bus,
// frames go out over the RF channel (the physics layer decides what
// actually reaches the ground). Beacons telemetry every BEACON_PERIOD_S
// whether or not anyone is listening — the channel does the gating, as in
// real life. Dispatches uplinked commands onto the spacecraft bus.
//
// Style notes, SDR-stack flavored: a small class with explicit state, no
// third-party JSON dependency — the bus message shapes are fixed, so
// targeted scanners parse them, and replies are emitted with printf
// formatting. Deterministic: no wall clock, no randomness.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr double kQueueCapacityMb = 256.0;
constexpr double kDownlinkRateMbS = 0.25;
constexpr double kBeaconPeriodS = 30.0;

bool contains(const std::string& s, const char* needle) {
    return s.find(needle) != std::string::npos;
}

// Split a step frame into per-message segments on the "topic" key. Each
// segment begins with the topic name and carries that message's fields.
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
        std::vector<std::pair<std::string, bool>> uplinks;
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
                std::string cmd_topic;
                bool on = false;
                if (find_str(seg, "\"cmd_topic\":\"", &cmd_topic) &&
                    find_bool(seg, "\"on\":", &on)) {
                    uplinks.emplace_back(cmd_topic, on);
                }
            }
        }

        // enqueue payload data; a full recorder rejects new data (and we
        // count every megabyte lost)
        double space = kQueueCapacityMb - queue_mb_;
        double accepted = incoming_mb < space ? incoming_mb : space;
        queue_mb_ += accepted;
        dropped_mb_ += incoming_mb - accepted;
        bool full = queue_mb_ >= kQueueCapacityMb - 1e-9;

        // emit reply frame
        std::printf("{\"type\":\"out\",\"pub\":[");
        bool first = true;

        double tx_mb = 0.0;
        if (carrier_ && queue_mb_ > 0.0) {
            tx_mb = kDownlinkRateMbS * dt;
            if (tx_mb > queue_mb_) {
                tx_mb = queue_mb_;
            }
            queue_mb_ -= tx_mb;
            sent_mb_ += tx_mb;
            emit_sep(&first);
            std::printf(
                "{\"topic\":\"radio/tx\",\"data\":{\"kind\":\"data\",\"mb\":%.17g}}",
                tx_mb);
        }
        if (last_beacon_t_ < 0.0 || t - last_beacon_t_ >= kBeaconPeriodS) {
            last_beacon_t_ = t;
            emit_sep(&first);
            std::printf(
                "{\"topic\":\"radio/tx\",\"data\":{\"kind\":\"telemetry\","
                "\"storage_frac\":%.17g,\"dropped_mb\":%.17g,\"sent_mb\":%.17g}}",
                queue_mb_ / kQueueCapacityMb, dropped_mb_, sent_mb_);
        }
        for (const auto& cmd : uplinks) {
            emit_sep(&first);
            std::printf("{\"topic\":\"%s\",\"data\":{\"on\":%s}}",
                        cmd.first.c_str(), cmd.second ? "true" : "false");
        }

        std::printf("],\"telemetry\":{\"queue_mb\":%.17g,\"dropped_mb\":%.17g,"
                    "\"sent_mb\":%.17g,\"carrier\":%s}",
                    queue_mb_, dropped_mb_, sent_mb_, carrier_ ? "1.0" : "0.0");

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
        for (const auto& cmd : uplinks) {
            emit_sep(&first_ev);
            std::printf("{\"kind\":\"uplink_dispatch\",\"detail\":"
                        "{\"cmd_topic\":\"%s\",\"on\":%s}}",
                        cmd.first.c_str(), cmd.second ? "true" : "false");
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

    bool carrier_ = false;
    bool was_full_ = false;
    double queue_mb_ = 0.0;
    double dropped_mb_ = 0.0;
    double sent_mb_ = 0.0;
    double last_beacon_t_ = -1.0;
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
                        "\"sensors/comms/*\",\"radio/rx_space\"]}\n");
            std::fflush(stdout);
        } else if (contains(line, "\"type\":\"shutdown\"")) {
            break;
        } else if (contains(line, "\"type\":\"step\"")) {
            comms.ingest_and_step(line);
        } else if (!line.empty()) {
            // unknown frame type: if it claims to be a step, never hang the bus
            if (contains(line, "\"type\":\"step\"")) {
                std::printf("{\"type\":\"out\",\"pub\":[],\"telemetry\":{},"
                            "\"events\":[{\"kind\":\"frame_reject\",\"detail\":{}}]}\n");
                std::fflush(stdout);
            }
        }
    }
    return 0;
}
