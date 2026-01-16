# Copyright (c) 2011-2026, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/main/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


def test_case_conversions():
    assert dart.to_upper("to UppEr") == "TO UPPER"
    assert dart.to_lower("to LowEr") == "to lower"


def test_trim():
    assert dart.trim_left(" trim ThIs ") == "trim ThIs "
    assert dart.trim_right(" trim ThIs ") == " trim ThIs"
    assert dart.trim(" trim ThIs ") == "trim ThIs"

    assert dart.trim_left("\n trim ThIs ", " ") == "\n trim ThIs "
    assert dart.trim_left("\n trim ThIs ", "\n") == " trim ThIs "
    assert dart.trim_right(" trim ThIs \n", " ") == " trim ThIs \n"
    assert dart.trim_right(" trim ThIs \n", "\n") == " trim ThIs "
    assert dart.trim("\n trim ThIs \n", " ") == "\n trim ThIs \n"
    assert dart.trim("\n trim ThIs \n", "\n") == " trim ThIs "

    assert dart.trim_left("\n trim ThIs \n", " \n") == "trim ThIs \n"
    assert dart.trim_right("\n trim ThIs \n", " \n") == "\n trim ThIs"
    assert dart.trim("\n trim ThIs \n", " \n") == "trim ThIs"


def test_split():
    assert len(dart.split(" trim ThIs ")) == 2
    assert dart.split(" trim ThIs ")[0] == "trim"
    assert dart.split(" trim ThIs ")[1] == "ThIs"


if __name__ == "__main__":
    pytest.main()
