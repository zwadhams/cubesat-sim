from cubesat_sim import MessageBus, SimClock


def make_bus():
    clock = SimClock(dt=1.0)
    return clock, MessageBus(clock)


def test_delivery_waits_for_dispatch():
    _, bus = make_bus()
    got = []
    bus.subscribe("a/b", got.append)
    bus.publish("a/b", "sender", {"x": 1})
    assert got == []
    assert bus.dispatch() == 1
    assert len(got) == 1
    assert got[0].data == {"x": 1}
    # already-delivered messages don't come back
    assert bus.dispatch() == 0


def test_wildcard_patterns():
    _, bus = make_bus()
    prefix, everything, exact = [], [], []
    bus.subscribe("eps/*", prefix.append)
    bus.subscribe("*", everything.append)
    bus.subscribe("adcs/mode", exact.append)

    bus.publish("eps/battery", "eps", {})
    bus.publish("eps/loads", "eps", {})
    bus.publish("adcs/mode", "adcs", {})
    bus.dispatch()

    assert [m.topic for m in prefix] == ["eps/battery", "eps/loads"]
    assert len(everything) == 3
    assert [m.topic for m in exact] == ["adcs/mode"]


def test_messages_stamped_and_ordered():
    clock, bus = make_bus()
    got = []
    bus.subscribe("*", got.append)
    bus.publish("t", "s", {})
    clock.advance()
    bus.publish("t", "s", {})
    bus.dispatch()
    assert [m.seq for m in got] == [0, 1]
    assert [m.tick for m in got] == [0, 1]
