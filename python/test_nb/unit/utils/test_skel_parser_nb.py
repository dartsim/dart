import dartpy_nb as dart
import pytest


def test_read_world():
    assert dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel") is not None
    assert (
        dart.utils.SkelParser.readWorld("dart://sample/skel/cubes.skel", None)
        is not None
    )


if __name__ == "__main__":
    pytest.main()
