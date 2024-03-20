import dartpy as dart
import pytest


def test_import():
    import dartpy.v7 as v7
    dir(v7)


if __name__ == "__main__":
    pytest.main()
