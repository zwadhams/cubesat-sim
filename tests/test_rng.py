from cubesat_sim import stream


def test_same_seed_same_name_reproduces():
    a = stream(42, "eps").normal(size=5)
    b = stream(42, "eps").normal(size=5)
    assert (a == b).all()


def test_streams_are_independent():
    base = stream(42, "eps").normal(size=5)
    other_name = stream(42, "adcs").normal(size=5)
    other_seed = stream(43, "eps").normal(size=5)
    assert not (base == other_name).all()
    assert not (base == other_seed).all()
