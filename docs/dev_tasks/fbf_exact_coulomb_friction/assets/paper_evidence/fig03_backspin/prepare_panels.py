#!/usr/bin/env python3
"""Prepare fixed-camera world-view panels for the Figure 3 still sheet."""

from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent
STEPS = (0, 10, 50, 120, 130)
WORLD_VIEW = (260, 58, 1560, 565)
PANEL_SIZE = (800, 312)


def main() -> None:
    panel_dir = ROOT / "panels"
    panel_dir.mkdir(exist_ok=True)
    for step in STEPS:
        source_path = ROOT / "frames" / f"frame_{step:03d}.png"
        with Image.open(source_path) as source:
            panel = (
                source.convert("RGB")
                .crop(WORLD_VIEW)
                .resize(PANEL_SIZE, Image.Resampling.LANCZOS)
            )
            panel.save(panel_dir / f"frame_{step:03d}.png")


if __name__ == "__main__":
    main()
