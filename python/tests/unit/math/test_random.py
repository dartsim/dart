import platform

import pytest
from dartpy.math import Random


def test_create():
    """Test that we can create a Random object"""
    rng = Random()


def test_seed():
    """Test that the seed is set correctly"""

    numSeeds = 10
    N = 10
    min = -1000
    max = 1000

    # Test that the same seed produces the same sequence of random numbers
    rng = Random()
    for i in range(numSeeds):
        seed = rng.uniformInt(0, 100000)

        # Create two random number generators with the same seed
        rng1 = Random(seed)
        rng2 = Random(seed)

        # Test that the seed is set correctly
        assert rng1.getSeed() == seed
        assert rng1.getSeed() == rng2.getSeed()

        # Test that the random number generators produce the same sequence of
        # random numbers
        for j in range(N):
            assert rng1.uniformInt(min, max) == rng2.uniformInt(min, max)


if __name__ == "__main__":
    pytest.main()
