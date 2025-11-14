import dartpy_nb as dart
import pytest


def test_read_world():
    assert (
        dart.utils.SdfParser.readWorld("dart://sample/sdf/double_pendulum.world")
        is not None
    )


if __name__ == "__main__":
    pytest.main()
