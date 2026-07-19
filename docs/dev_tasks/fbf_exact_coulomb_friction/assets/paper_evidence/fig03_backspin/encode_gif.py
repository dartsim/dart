#!/usr/bin/env python3
"""Encode the deterministic Figure 3 frame sequence as a near-60-fps GIF."""

from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parent
FRAME_PATHS = sorted((ROOT / "frames").glob("frame_*.png"))
OUTPUT = ROOT / "fig03_backspin_dart_fbf.gif"


def main() -> None:
    if len(FRAME_PATHS) != 131:
        raise RuntimeError(f"expected 131 frames, found {len(FRAME_PATHS)}")

    frames = []
    for path in FRAME_PATHS:
        with Image.open(path) as source:
            resized = source.convert("RGB").resize((960, 360), Image.Resampling.LANCZOS)
            frames.append(resized.quantize(colors=192, method=Image.Quantize.MEDIANCUT))

    # GIF stores centiseconds. This pattern averages 16.67 ms per frame.
    durations = [20 if index % 3 in (0, 1) else 10 for index in range(len(frames))]
    frames[0].save(
        OUTPUT,
        save_all=True,
        append_images=frames[1:],
        duration=durations,
        loop=0,
        disposal=2,
        optimize=False,
    )


if __name__ == "__main__":
    main()
