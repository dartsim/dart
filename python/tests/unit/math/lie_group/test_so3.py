# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.

import pytest
from dartpy.math import SO3, exp, log, rand_so3


def test_identity():
    x = SO3()
    assert x.is_identity()


def test_random():
    x = rand_so3()


def test_inverse():
    x = SO3()
    assert x.inverse().inverse().is_approx(x)
    assert x.inverse().inverse() == x


def test_inverse_in_place():
    x = rand_so3()
    x_inv = x.inverse()
    assert x_inv == x.inverse_in_place()


def test_equality():
    a = SO3()
    b = SO3()
    assert a == b

    c = rand_so3()
    d = c.inverse()
    assert c * d == SO3()


def test_exp():
    x = rand_so3()
    assert x == x.log().exp()
    assert x == exp(log(x))


if __name__ == "__main__":
    pytest.main()
