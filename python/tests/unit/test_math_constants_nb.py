# Copyright (c) The DART development contributors
# All rights reserved.

import math

import pytest

import dartpy_nb


def test_constants_match_cpp_values():
    math_mod = dartpy_nb.math
    assert math_mod.pi == pytest.approx(math.pi)
    assert math_mod.two_pi == pytest.approx(2.0 * math.pi)
    assert math_mod.half_pi == pytest.approx(0.5 * math.pi)
    assert math_mod.phi == pytest.approx(1.6180339887498948)


def test_angle_conversion_round_trip():
    degrees = 123.456
    radians = dartpy_nb.math.deg2rad(degrees)
    assert radians == pytest.approx(math.radians(degrees))
    assert dartpy_nb.math.rad2deg(radians) == pytest.approx(degrees)
