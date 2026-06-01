"""Source-level API style checks for the Python demo catalog."""

from __future__ import annotations

from pathlib import Path


_ROOT = Path(__file__).resolve().parents[3]
_DEMO_ROOT = _ROOT / "python" / "examples" / "demos"


def test_python_demos_use_snake_case_frame_world_factory() -> None:
    offenders = [
        path.relative_to(_ROOT)
        for path in _DEMO_ROOT.rglob("*.py")
        if "Frame.World(" in path.read_text()
    ]

    assert offenders == []
