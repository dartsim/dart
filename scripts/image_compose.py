"""Compose evidence images: side-by-side, overlay blend, and diff heatmap.

Builds the comparison views the agent verification workflow uses in PR
evidence: before/after or expected/actual panels, normal/debug pairs, and a
colorized diff. Stdlib-only, PPM/PNG in and out, deterministic output, and a
machine-readable manifest on stdout.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from _image_tools import (
    ImageData,
    diff_heatmap,
    overlay_blend,
    read_image,
    side_by_side,
    write_image,
)

SCHEMA_VERSION = "dart.image_compose/v1"


def _load(paths: list[Path]) -> list[ImageData]:
    return [read_image(path) for path in paths]


def compose(args: argparse.Namespace) -> dict:
    images = _load(args.inputs)
    stats: dict = {}
    if args.mode == "side-by-side":
        labels = args.labels
        if labels is not None and len(labels) != len(images):
            raise ValueError(f"--labels needs {len(images)} entries, got {len(labels)}")
        result = side_by_side(images, labels=labels, gap=args.gap)
    elif args.mode == "blend":
        if len(images) != 2:
            raise ValueError("blend requires exactly two inputs")
        result = overlay_blend(images[0], images[1], alpha=args.alpha)
    elif args.mode == "diff":
        if len(images) != 2:
            raise ValueError("diff requires exactly two inputs")
        result, stats = diff_heatmap(images[0], images[1], amplify=args.amplify)
    else:  # pragma: no cover - argparse restricts choices
        raise ValueError(f"unknown mode {args.mode}")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_image(args.out, result)
    manifest = {
        "schema_version": SCHEMA_VERSION,
        "mode": args.mode,
        "inputs": [str(path) for path in args.inputs],
        "labels": list(args.labels) if args.labels else None,
        "out": str(args.out),
        "width": result.width,
        "height": result.height,
        "stats": stats,
        "pass": True,
    }
    return manifest


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode", choices=("side-by-side", "blend", "diff"), help="composition mode"
    )
    parser.add_argument("inputs", nargs="+", type=Path, help="input images (PNG/PPM)")
    parser.add_argument("--out", type=Path, required=True, help="output image path")
    parser.add_argument(
        "--labels",
        nargs="+",
        default=None,
        help="per-panel labels for side-by-side (e.g. BEFORE AFTER)",
    )
    parser.add_argument("--gap", type=int, default=8, help="panel gap in pixels")
    parser.add_argument(
        "--alpha", type=float, default=0.5, help="blend weight of the second image"
    )
    parser.add_argument(
        "--amplify", type=float, default=4.0, help="diff heatmap gain factor"
    )
    parser.add_argument("--manifest", type=Path, help="also write manifest JSON here")
    args = parser.parse_args(argv)

    try:
        manifest = compose(args)
    except (OSError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    text = json.dumps(manifest, indent=2, sort_keys=True)
    if args.manifest is not None:
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        args.manifest.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
