import json

import pytest

from cubesat_sim.mission import build_sim

pytestmark = pytest.mark.usefixtures("rust_adcs_binary", "cpp_comms_binary")


def orbit_period(sim):
    return sim.components[0].orbit.period_s


@pytest.fixture(scope="module")
def healthy_flight(request):
    request.getfixturevalue("rust_adcs_binary")
    sim = build_sim(dt=1.0, seed=21)
    sim.run(duration=4 * orbit_period(sim))
    yield sim
    sim.close()


def test_detumbles_from_initial_spin(healthy_flight):
    rate = [(r[1], r[4]) for r in healthy_flight.recorder.telemetry("physics", "rate_dps")]
    assert rate[0][1] > 3.0  # started tumbling
    period = orbit_period(healthy_flight)
    late = [v for t, v in rate if t > 2 * period]
    # steady state is calm; brief dawn re-acquisition slews after eclipse
    # drift legitimately ride at ~kp/kd (~2.3 dps), so bound the average
    # and the endpoint, not the max
    assert sum(late) / len(late) < 1.0
    assert late[-1] < 0.5

    events = healthy_flight.recorder.events("adcs")
    to_sun = [e for e in events
              if e[3] == "mode_change" and json.loads(e[4])["to"] == "SUN_POINT"]
    assert to_sun and to_sun[0][1] < period  # acquired within the first orbit


def test_acquires_and_holds_sun_pointing(healthy_flight):
    period = orbit_period(healthy_flight)
    rec = healthy_flight.recorder
    eclipse = {r[0]: r[4] for r in rec.telemetry("physics", "eclipse")}
    facing = [(r[0], r[1], r[4]) for r in rec.telemetry("physics", "sun_facing")]
    late_sunlit = [v for tick, t, v in facing
                   if t > 2 * period and eclipse.get(tick) == 0.0]
    assert late_sunlit
    avg = sum(late_sunlit) / len(late_sunlit)
    assert avg > 0.9  # panel on the sun when the sun is there


def test_wheels_stay_within_envelope(healthy_flight):
    frac = [r[4] for r in healthy_flight.recorder.telemetry("physics", "wheel_h_frac")]
    assert max(frac) < 0.95  # momentum managed, no hard saturation


def test_attitude_couples_into_power(healthy_flight):
    """With sun pointing established, generation should beat the tumbling
    average by a wide margin during sunlit stretches."""
    period = orbit_period(healthy_flight)
    rec = healthy_flight.recorder
    gen = [(r[1], r[4]) for r in rec.telemetry("physics", "p_gen_w")]
    early = [v for t, v in gen if t < 0.2 * period and v > 0]  # tumbling, sunlit
    late = [v for t, v in gen if t > 2 * period and v > 0]     # pointing, sunlit
    assert sum(late) / len(late) > 1.5 * sum(early) / len(early)
