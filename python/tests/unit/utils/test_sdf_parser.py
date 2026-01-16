# Copyright (c) 2011-2026, The DART development contributors
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
        dart.io.SdfParser.read_world("dart://sample/sdf/double_pendulum.world")
        is not None
    )


def test_read_high_version_world():
    world = dart.io.SdfParser.read_world(
        "dart://sample/sdf/test/high_version.world"
    )
    assert world is not None
    assert world.get_num_skeletons() == 1


if __name__ == "__main__":
    pytest.main()
