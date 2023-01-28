import os
import platform

import dartpy as dart
import pytest
from tests.util import get_asset_path


def test_parse_fetch():
    assert (
        dart.io.MjcfParser.readWorld(
            "dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml"
        )
        is not None
    )


if __name__ == "__main__":
    pytest.main()
