# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import pytest
from dartpy.math import SO3


def test_identity():
    x = SO3()
    assert x.isIdentity()


def test_random():
    x = SO3.Random()


def test_inverse():
    x = SO3()
    assert x.inverse().inverse().isApprox(x)
    assert x.inverse().inverse() == x


def test_inverse_in_place():
    x = SO3.Random()
    x_inv = x.inverse()
    assert x_inv == x.inverseInPlace()


def test_equality():
    a = SO3.Identity()
    b = SO3.Identity()
    assert a == b

    c = SO3.Random()
    d = c.inverse()
    assert c * d == SO3.Identity()


if __name__ == "__main__":
    pytest.main()
