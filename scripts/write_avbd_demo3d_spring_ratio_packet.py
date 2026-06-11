#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo3d Spring Ratio source-demo evidence packet."""

from __future__ import annotations

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from write_avbd_demo3d_spring_packet import (  # noqa: E402
    SPRING_RATIO3D_SPEC,
    main_for_spec,
)


def main(argv: list[str]) -> int:
    return main_for_spec(argv, SPRING_RATIO3D_SPEC)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
