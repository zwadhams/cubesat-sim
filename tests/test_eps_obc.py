import json

from cubesat_sim import Component, Simulation
from cubesat_sim.subsystems import EPS, OBC


class ScriptedVoltage(Component):
    """Stands in for physics: plays back a battery-voltage script, noise-free."""

    def __init__(self, script):
        super().__init__("physics", period=1.0)
        self.script = list(script)

    def step(self, t, dt):
        volts = self.script.pop(0) if self.script else None
        if volts is not None:
            self.publish("sensors/eps/battery_voltage", volts=volts)


class ScriptedStatus(Component):
    """Stands in for EPS: plays back soc_est values on eps/status."""

    def __init__(self, script):
        super().__init__("eps", period=1.0)
        self.script = list(script)

    def step(self, t, dt):
        if self.script:
            self.publish("eps/status", soc_est=self.script.pop(0))


class Collector(Component):
    def __init__(self, *patterns):
        super().__init__("collector", period=1.0)
        self.patterns = patterns
        self.seen = []

    def on_start(self):
        for p in self.patterns:
            self.subscribe(p)

    def step(self, t, dt):
        self.seen.extend(self.drain())


def test_eps_sheds_and_restores_with_hysteresis():
    # soc_est = (v - 6.0) / 2.4:  7.5 -> 0.625, 6.2 -> 0.083 (< shed 0.15),
    # 6.5 -> 0.21 (inside band: still shed), 7.0 -> 0.417 (> restore 0.30)
    script = [7.5] * 5 + [6.2] * 5 + [6.5] * 5 + [7.0] * 5
    sim = Simulation(dt=1.0)
    sim.add(ScriptedVoltage(script))
    sim.add(EPS())
    collector = sim.add(Collector("cmd/loads/*"))
    sim.run(ticks=len(script) + 3)

    adcs = [(m.tick, m.data["on"]) for m in collector.seen
            if m.topic == "cmd/loads/adcs"]
    assert [on for _, on in adcs] == [True, False, True]  # on -> shed -> restore

    kinds = [e[3] for e in sim.recorder.events("eps")]
    assert kinds == ["load_shed", "load_restore"]

    # while inside the hysteresis band (6.5 V) shedding must persist
    shed_tick = adcs[1][0]
    restore_tick = adcs[2][0]
    assert restore_tick - shed_tick >= 10  # not restored during the 6.5 V stretch


def test_obc_mode_hysteresis_and_payload_requests():
    # 0.5 nominal, 0.2 -> SAFE, 0.35 inside band (stay SAFE), 0.5 -> NOMINAL
    script = [0.5] * 5 + [0.2] * 5 + [0.35] * 5 + [0.5] * 5
    sim = Simulation(dt=1.0)
    sim.add(ScriptedStatus(script))
    sim.add(OBC())
    collector = sim.add(Collector("obc/mode", "obc/request/loads"))
    sim.run(ticks=len(script) + 3)

    modes = [m.data["mode"] for m in collector.seen if m.topic == "obc/mode"]
    assert modes == ["SAFE", "NOMINAL"]

    events = sim.recorder.events("obc")
    assert [json.loads(e[4])["to"] for e in events] == ["SAFE", "NOMINAL"]

    requests = [m for m in collector.seen if m.topic == "obc/request/loads"]
    assert any(r.data["loads"]["payload"] is False for r in requests)  # while SAFE
    assert requests[-1].data["loads"]["payload"] is True  # after recovery
