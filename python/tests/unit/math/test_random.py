import platform

import numpy as np
import pytest
from dartpy.math import Random


def test_create():
    rand = Random()


def test_seed():
    rand = Random()

    N = 10

    min = -10
    max = 10

    first = []
    second = []
    third = []

    tol = 1e-6

    for i in range(N):
        Random.setSeed(i)
        first.append(Random.uniform(min, max))
        second.append(Random.uniform(min, max))
        third.append(Random.uniform(min, max))

    for i in range(N):
        Random.setSeed(i)
        assert Random.getSeed() == i
        assert Random.uniform(min, max) == pytest.approx(first[i], tol)
        assert Random.uniform(min, max) == pytest.approx(second[i], tol)
        assert Random.uniform(min, max) == pytest.approx(third[i], tol)


def test_float_uniform_keeps_single_precision_and_seeded_sequence():
    lower, upper = np.float32(-0.2), np.float32(0.5)
    Random.setSeed(1234)
    values = [Random.uniform_float(lower, upper) for _ in range(12)]
    next_double = Random.uniform(0, 1)
    assert all(lower <= value <= upper for value in values)
    assert all(value == float(np.float32(value)) for value in values)
    assert len(set(values)) > 1
    Random.setSeed(1234)
    assert [Random.uniform_float(lower, upper) for _ in range(12)] == values
    assert Random.uniform(0, 1) == next_double


def test_integer_uniform_is_inclusive_seeded_and_handles_singleton_ranges():
    Random.setSeed(1234)
    values = [Random.uniform_int(lower=-2, upper=2) for _ in range(100)]
    next_double = Random.uniform(0, 1)
    assert all(type(value) is int for value in values)
    assert set(values) == {-2, -1, 0, 1, 2}
    Random.setSeed(1234)
    assert [Random.uniform_int(-2, 2) for _ in range(100)] == values
    assert Random.uniform(0, 1) == next_double

    Random.setSeed(1234)
    assert Random.uniform_int(7, 7) == 7
    after_singleton = Random.uniform(0, 1)
    Random.setSeed(1234)
    assert Random.uniform_int(7, 7) == 7
    assert Random.uniform(0, 1) == after_singleton


def test_integer_uniform_rejects_reversed_bounds_without_consuming_rng():
    Random.setSeed(1234)
    expected = Random.uniform_int(-2, 2)
    Random.setSeed(1234)
    with pytest.raises(ValueError, match="lower must not exceed upper"):
        Random.uniform_int(3, 2)
    assert Random.uniform_int(-2, 2) == expected


if __name__ == "__main__":
    pytest.main()
