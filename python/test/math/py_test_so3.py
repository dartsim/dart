# Copyright (c) 2011-2021, The DART development contributors

import pytest
import dartpy8 as dart


def test_constructors():
    x1 = dart.math.SO3()
    x2 = dart.math.SO3.Random()


if __name__ == "__main__":
    pytest.main()
