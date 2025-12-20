import platform

import pytest
from dartpy import Random


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
        Random.set_seed(i)
        first.append(Random.uniform(min, max))
        second.append(Random.uniform(min, max))
        third.append(Random.uniform(min, max))

    for i in range(N):
        Random.set_seed(i)
        assert Random.get_seed() == i
        assert Random.uniform(min, max) == pytest.approx(first[i], tol)
        assert Random.uniform(min, max) == pytest.approx(second[i], tol)
        assert Random.uniform(min, max) == pytest.approx(third[i], tol)


if __name__ == "__main__":
    pytest.main()
