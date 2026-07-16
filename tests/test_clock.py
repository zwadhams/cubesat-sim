from datetime import datetime, timezone

from cubesat_sim import SimClock


def test_time_advances_by_dt():
    clock = SimClock(dt=0.5)
    assert clock.time == 0.0
    for _ in range(3):
        clock.advance()
    assert clock.tick == 3
    assert clock.time == 1.5


def test_utc_tracks_epoch():
    epoch = datetime(2026, 7, 15, 12, 0, tzinfo=timezone.utc)
    clock = SimClock(dt=60.0, epoch=epoch)
    clock.advance()
    assert clock.utc == datetime(2026, 7, 15, 12, 1, tzinfo=timezone.utc)
