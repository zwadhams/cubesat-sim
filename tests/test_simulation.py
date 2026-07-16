import pytest

from cubesat_sim import Component, Simulation


class Ticker(Component):
    """Publishes a noisy sample every step and records when it stepped."""

    def __init__(self, name="ticker", period=1.0):
        super().__init__(name, period)
        self.step_times = []

    def step(self, t, dt):
        self.step_times.append((t, dt))
        self.publish("tick/sample", noise=float(self.rng.normal()))


class Collector(Component):
    def __init__(self):
        super().__init__("collector", 1.0)
        self.received = []

    def on_start(self):
        self.subscribe("tick/*")

    def step(self, t, dt):
        for msg in self.drain():
            self.received.append((t, msg))


def test_scheduling_respects_period():
    sim = Simulation(dt=1.0)
    slow = sim.add(Ticker("slow", period=3.0))
    sim.run(ticks=7)
    assert [t for t, _ in slow.step_times] == [0.0, 3.0, 6.0]
    assert all(dt == 3.0 for _, dt in slow.step_times)


def test_one_tick_delivery_latency():
    sim = Simulation(dt=1.0)
    sim.add(Ticker())
    collector = sim.add(Collector())
    sim.run(ticks=3)
    # ticker publishes at ticks 0,1,2; the first two arrive during the
    # collector's tick 1 and 2 steps (one-tick latency), the third is in flight
    assert len(collector.received) == 2
    for received_at, msg in collector.received:
        assert received_at == msg.time + sim.dt


def test_duplicate_names_rejected():
    sim = Simulation()
    sim.add(Ticker())
    with pytest.raises(ValueError):
        sim.add(Ticker())


def test_run_arg_validation():
    sim = Simulation()
    with pytest.raises(ValueError):
        sim.run()
    with pytest.raises(ValueError):
        sim.run(duration=10, ticks=10)


def run_and_log(seed):
    sim = Simulation(dt=1.0, seed=seed)
    sim.add(Ticker())
    sim.run(ticks=50)
    return sim.recorder.messages()


def test_runs_reproducible_from_seed():
    assert run_and_log(42) == run_and_log(42)
    assert run_and_log(42) != run_and_log(43)
