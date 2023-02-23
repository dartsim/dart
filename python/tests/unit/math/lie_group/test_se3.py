# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import pytest
from dartpy.math import SE3, exp, log, rand_se3


def test_identity():
    x = SE3()
    assert x.is_identity()


def test_random():
    x = rand_se3()


def test_inverse():
    x = SE3()
    assert x.inverse().inverse().is_approx(x)
    assert x.inverse().inverse() == x


def test_inverse_in_place():
    x = rand_se3()
    x_inv = x.inverse()
    assert x_inv == x.inverse_in_place()


def test_equality():
    a = SE3()
    b = SE3()
    assert a == b

    c = rand_se3()
    d = c.inverse()
    assert c * d == SE3()


def test_exp():
    x = rand_se3()
    assert x == x.log().exp()
    assert x == exp(log(x))


if __name__ == "__main__":
    pytest.main()
