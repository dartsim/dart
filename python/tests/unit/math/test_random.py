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


if __name__ == "__main__":
    pytest.main()
