# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


def test_read_world():
    assert (
        dart.io.SdfParser.readWorld("dart://sample/sdf/double_pendulum.world")
        is not None
    )


if __name__ == "__main__":
    pytest.main()
