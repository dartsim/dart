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
    assert dart.io.SkelParser.read_world("dart://sample/skel/cubes.skel") is not None
    assert (
        dart.io.SkelParser.read_world("dart://sample/skel/cubes.skel", None)
        is not None
    )


if __name__ == "__main__":
    pytest.main()
