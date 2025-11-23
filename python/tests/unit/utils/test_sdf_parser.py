# Copyright (c) 2011-2025, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


def test_read_world():
    assert (
        dart.utils.SdfParser.readWorld("dart://sample/sdf/double_pendulum.world")
        is not None
    )


def test_read_high_version_world():
    world = dart.utils.SdfParser.readWorld(
        "dart://sample/sdf/test/high_version.world"
    )
    assert world is not None
    assert world.getNumSkeletons() == 1


if __name__ == "__main__":
    pytest.main()
