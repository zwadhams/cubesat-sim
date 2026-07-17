"""The bridge's inbound quarantine: garbage from flight software is
rejected loudly, never delivered. Found by campaign 1: an SEU-corrupted
gyro word (~1e300, finite, passes every guard) squared to inf inside the
Rust ADCS, serde_json laundered the inf into JSON null, and float(None)
killed the flight."""

import sys

import pytest

from cubesat_sim import RemoteComponent, Simulation
from cubesat_sim.mission import RUST_ADCS_BIN

# a fake flight computer that replies with poisoned output: one pub with
# a null field, one clean pub, one null telemetry key, one clean key
FAKE_PEER = r"""
import json, sys
for line in sys.stdin:
    m = json.loads(line)
    if m["type"] == "init":
        print(json.dumps({"type": "ready", "subscribe": []}), flush=True)
    elif m["type"] == "step":
        print(json.dumps({
            "type": "out",
            "pub": [{"topic": "junk/cmd", "data": {"x": None}},
                    {"topic": "ok/cmd", "data": {"y": 2.0}}],
            "telemetry": {"bad": None, "good": 1.5},
            "events": [],
        }), flush=True)
    else:
        break
"""


def test_bridge_quarantines_null_output():
    sim = Simulation(dt=1.0)
    sim.add(RemoteComponent("fake", period=1.0,
                            argv=[sys.executable, "-c", FAKE_PEER]))
    sim.run(ticks=3)  # must not raise
    rec = sim.recorder

    kinds = [e[3] for e in rec.events("fake")]
    assert "pub_reject" in kinds and "telemetry_reject" in kinds
    assert rec.messages(topic="ok/cmd")          # clean traffic flows
    assert not rec.messages(topic="junk/cmd")    # poison does not
    assert rec.telemetry("fake", "good")
    assert not rec.telemetry("fake", "bad")
    sim.close()


@pytest.mark.usefixtures("rust_adcs_binary")
def test_adcs_saturates_seu_sized_gyro_words():
    """Feed the real Rust ADCS a gyro reading of 1e300 (what a top-exponent
    SEU bit flip produces): its rate word must saturate, its commands must
    stay finite, and nothing may crash."""
    sim = Simulation(dt=1.0, seed=0)
    sim.add(RemoteComponent("adcs", period=1.0, argv=[str(RUST_ADCS_BIN)]))
    for _ in range(3):
        sim.bus.publish("sensors/adcs/gyro", "test",
                        {"x": 1e300, "y": -1e290, "z": 0.0})
        sim.run(ticks=1)
    sim.run(ticks=2)

    rates = [v for *_, v in sim.recorder.telemetry("adcs", "rate_dps")]
    assert rates and all(r == r and r <= 9999.0 for r in rates)  # finite, capped
    assert max(rates) == 9999.0                                  # saturation hit
    assert not any(e[3] in ("pub_reject", "telemetry_reject")
                   for e in sim.recorder.events("adcs"))
    sim.close()
