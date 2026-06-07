# Copyright (c) 2011, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


def test_read_world_removed():
    assert hasattr(dart.io.SdfParser, "read_skeleton")
    assert hasattr(dart.io.SdfParser, "Options")
    assert not hasattr(dart.io.SdfParser, "read_world")
    assert not hasattr(dart.io.SdfParser, "readWorld")


if __name__ == "__main__":
    pytest.main()
