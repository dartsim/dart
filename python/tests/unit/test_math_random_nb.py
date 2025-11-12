import dartpy_nb


def test_seed_round_trip():
    dartpy_nb.math.Random.setSeed(7)
    assert dartpy_nb.math.Random.getSeed() == 7


def test_uniform_is_deterministic_with_seed():
    dartpy_nb.math.Random.setSeed(123)
    seq_one = [dartpy_nb.math.Random.uniform(-1.0, 1.0) for _ in range(5)]
    dartpy_nb.math.Random.setSeed(123)
    seq_two = [dartpy_nb.math.Random.uniform(-1.0, 1.0) for _ in range(5)]
    assert seq_one == seq_two
