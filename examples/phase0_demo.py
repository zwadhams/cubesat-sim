"""Phase 0 smoke demo: two toy components breathing over the bus.

A beacon pulses every 5 simulated seconds; a monitor listens and records the
bus latency it observes. Proves the kernel loop end to end and leaves a
queryable flight recording in runs/phase0_demo.db.
"""

from pathlib import Path

from cubesat_sim import Component, Simulation


class Beacon(Component):
    def __init__(self):
        super().__init__("beacon", period=5.0)
        self.count = 0

    def step(self, t, dt):
        self.count += 1
        self.publish("beacon/pulse", count=self.count,
                     jitter=float(self.rng.normal(0.0, 0.1)))
        self.record("pulses_sent", self.count)


class Monitor(Component):
    def __init__(self):
        super().__init__("monitor", period=1.0)

    def on_start(self):
        self.subscribe("beacon/*")

    def step(self, t, dt):
        for msg in self.drain():
            self.record("observed_latency_s", t - msg.time)
            if msg.data["count"] % 5 == 0:
                self.event("milestone", pulses=msg.data["count"])


def main():
    Path("runs").mkdir(exist_ok=True)
    sim = Simulation(dt=1.0, seed=42, recorder_path="runs/phase0_demo.db")
    sim.add(Beacon())
    sim.add(Monitor())
    sim.run(duration=120)

    messages = sim.recorder.messages()
    latencies = [v for *_, v in sim.recorder.telemetry("monitor", "observed_latency_s")]
    events = sim.recorder.events()
    print(f"ran {sim.clock.tick} ticks ({sim.clock.time:.0f} simulated seconds)")
    print(f"bus messages: {len(messages)}")
    print(f"observed bus latency: {min(latencies):.1f}-{max(latencies):.1f} s "
          f"across {len(latencies)} pulses")
    print(f"events logged: {len(events)}")
    print("flight recording: runs/phase0_demo.db")
    sim.close()


if __name__ == "__main__":
    main()
