import os
import platform

import dartpy_nb as dart
import pytest
from util import get_asset_path


@pytest.mark.skipif(platform.system() == "Windows", reason="Windows path issues")
def test_read_world():
    world = dart.utils.MjcfParser.readWorld(get_asset_path("mjcf/mjcf_test.xml"))
    assert world is not None
    assert world.getNumSkeletons() == 2


if __name__ == "__main__":
    pytest.main()
