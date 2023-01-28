# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import pytest
from dartpy.math import SE3, SO3


def test_default_constructor():
    y = SE3()


if __name__ == "__main__":
    pytest.main()
