# Copyright (c) 2011-2023, The DART development contributors
# All rights reserved.
#
# The list of contributors can be found at:
#   https://github.com/dartsim/dart/blob/master/LICENSE
#
# This file is provided under the "BSD-style" License

import dartpy as dart
import pytest


def test_case_conversions():
    assert dart.common.toUpper("to UppEr") == "TO UPPER"
    assert dart.common.toLower("to LowEr") == "to lower"


def test_trim():
    assert dart.common.trimLeft(" trim ThIs ") == "trim ThIs "
    assert dart.common.trimRight(" trim ThIs ") == " trim ThIs"
    assert dart.common.trim(" trim ThIs ") == "trim ThIs"

    assert dart.common.trimLeft("\n trim ThIs ", " ") == "\n trim ThIs "
    assert dart.common.trimLeft("\n trim ThIs ", "\n") == " trim ThIs "
    assert dart.common.trimRight(" trim ThIs \n", " ") == " trim ThIs \n"
    assert dart.common.trimRight(" trim ThIs \n", "\n") == " trim ThIs "
    assert dart.common.trim("\n trim ThIs \n", " ") == "\n trim ThIs \n"
    assert dart.common.trim("\n trim ThIs \n", "\n") == " trim ThIs "

    assert dart.common.trimLeft("\n trim ThIs \n", " \n") == "trim ThIs \n"
    assert dart.common.trimRight("\n trim ThIs \n", " \n") == "\n trim ThIs"
    assert dart.common.trim("\n trim ThIs \n", " \n") == "trim ThIs"


def test_split():
    assert len(dart.common.split(" trim ThIs ")) == 2
    assert dart.common.split(" trim ThIs ")[0] == "trim"
    assert dart.common.split(" trim ThIs ")[1] == "ThIs"


if __name__ == "__main__":
    pytest.main()
