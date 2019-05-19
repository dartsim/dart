import platform
import pytest
from dartpy.simulation import World


def test_empty_world():
    world = World('my world')
    assert world.getNumSkeletons() is 0
    assert world.getNumSimpleFrames() is 0


if __name__ == "__main__":
    pytest.main()
