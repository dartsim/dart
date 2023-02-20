# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import pytest
from dartpy.math import SE3, SO3, Exp, Log


def test_identity():
    x = SE3()
    assert x.isIdentity()


def test_random():
    x = SE3.Random()


def test_inverse():
    x = SE3()
    assert x.inverse().inverse().isApprox(x)
    assert x.inverse().inverse() == x


def test_inverse_in_place():
    x = SE3.Random()
    x_inv = x.inverse()
    assert x_inv == x.inverseInPlace()


def test_equality():
    a = SE3.Identity()
    b = SE3.Identity()
    assert a == b

    c = SE3.Random()
    d = c.inverse()
    assert c * d == SE3.Identity()


def test_exp():
    x = SE3.Random()
    assert x == x.log().exp()
    assert x == Exp(Log(x))


if __name__ == "__main__":
    pytest.main()
