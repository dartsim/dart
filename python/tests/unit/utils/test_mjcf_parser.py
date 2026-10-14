import os
import platform

import dartpy as dart
import pytest
from tests.util import get_asset_path


def test_parse_fetch():
    world = dart.utils.MjcfParser.readWorld(
        "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml"
    )
    assert world is not None
    # M1f switches only the Python World and io.load_world wrappers. The
    # direct binding calls the C++ parser, whose FCL default must stay intact.
    assert isinstance(
        world.getConstraintSolver().getCollisionDetector(),
        dart.collision.FCLCollisionDetector,
    )


if __name__ == "__main__":
    pytest.main()
