import dartpy_nb as dart
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
    result = dart.common.split(" trim ThIs ")
    assert len(result) == 2
    assert result[0] == "trim"
    assert result[1] == "ThIs"


if __name__ == "__main__":
    pytest.main()
