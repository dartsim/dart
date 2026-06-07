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
    assert hasattr(dart.io.SkelParser, "read_skeleton")
    assert not hasattr(dart.io.SkelParser, "read_world")
    assert not hasattr(dart.io.SkelParser, "readWorld")
    assert not hasattr(dart.io.SkelParser, "read_world_xml")
    assert not hasattr(dart.io.SkelParser, "readWorldXML")


if __name__ == "__main__":
    pytest.main()
