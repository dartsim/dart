# Copyright (c) 2011-2022, The DART development contributors

import pytest
import dartpy8 as dart


def test_constructors():
    x1 = dart.math.SO3()
    x2 = dart.math.SO3.Random()
    algebra = dart.math.SO3Algebra.Random()
    tangent = dart.math.SO3Tangent.Random()
    cotangent = dart.math.SO3Cotangent.Random()


def test_nested_classes():
    dart.math.SO3.Algebra.Random()
    dart.math.SO3.Tangent.Random()
    dart.math.SO3.Cotangent.Random()


def test_inverse():
    a = dart.math.SO3()
    b = a.inverse()


if __name__ == "__main__":
    pytest.main()
