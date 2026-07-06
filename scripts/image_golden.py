#!/usr/bin/env python3
"""Golden-image update and compare workflow for DART visual checks."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from _image_tools import read_image, write_image
from image_verdict import (
    DEFAULT_FAIL,
    DEFAULT_FAILPERCENT,
    build_verdict,
    parse_meta,
)

SCHEMA_VERSION = "dart.image_golden/v1"


def golden_sidecar_path(golden_path: Path) -> Path:
    return golden_path.with_suffix(golden_path.suffix + ".json")


def update_golden(
    capture_path: Path,
    golden_path: Path,
    *,
    fail: float = DEFAULT_FAIL,
    failpercent: float = DEFAULT_FAILPERCENT,
    ignore_aa: bool = True,
    relnorm: float | None = None,
    metadata: dict[str, str] | None = None,
) -> dict[str, object]:
    capture = read_image(capture_path)
    write_image(golden_path, capture)
    sidecar = {
        "schema_version": SCHEMA_VERSION,
        "golden": {
            "path": str(golden_path),
            "width": capture.width,
            "height": capture.height,
        },
        "source": {"path": str(capture_path)},
        "metadata": metadata or {},
        "tolerance": {
            "fail": fail,
            "failpercent": failpercent,
            "ignore_aa": ignore_aa,
            "relnorm": relnorm,
        },
    }
    sidecar_path = golden_sidecar_path(golden_path)
    sidecar_path.parent.mkdir(parents=True, exist_ok=True)
    sidecar_path.write_text(
        json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return sidecar


def compare_golden(
    capture_path: Path,
    golden_path: Path,
    *,
    fail: float = DEFAULT_FAIL,
    failpercent: float = DEFAULT_FAILPERCENT,
    ignore_aa: bool = True,
    relnorm: float | None = None,
    metadata: dict[str, str] | None = None,
    retries: int = 1,
) -> dict[str, object]:
    if retries < 1:
        raise ValueError("--retries must be at least 1")
    if not golden_path.is_file():
        raise FileNotFoundError(
            f"{golden_path}: golden does not exist; rerun with --update"
        )

    last_verdict: dict[str, object] | None = None
    for attempt in range(1, retries + 1):
        verdict = build_verdict(
            capture_path,
            golden_path,
            fail=fail,
            failpercent=failpercent,
            ignore_aa=ignore_aa,
            relnorm=relnorm,
            metadata=metadata,
        )
        verdict["golden_workflow"] = {
            "golden": str(golden_path),
            "sidecar": str(golden_sidecar_path(golden_path)),
            "attempt": attempt,
            "retries": retries,
        }
        last_verdict = verdict
        if verdict["pass"]:
            break
    assert last_verdict is not None
    return last_verdict


def _metadata_from_args(args: argparse.Namespace) -> dict[str, str]:
    metadata = {"backend": args.backend, "fidelity": args.fidelity}
    metadata.update(parse_meta(args.meta))
    return metadata


def _validate_thresholds(
    fail: float, failpercent: float, relnorm: float | None
) -> None:
    if not 0.0 <= fail <= 1.0:
        raise ValueError("--fail must be in normalized range [0, 1]")
    if not 0.0 <= failpercent <= 100.0:
        raise ValueError("--failpercent must be in range [0, 100]")
    if relnorm is not None and relnorm < 0.0:
        raise ValueError("--relnorm must be non-negative")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare a capture against a committed golden, or update the golden."
    )
    parser.add_argument("capture", type=Path)
    parser.add_argument("golden", type=Path)
    parser.add_argument(
        "--update", action="store_true", help="replace the golden with capture"
    )
    parser.add_argument(
        "--out", type=Path, help="write JSON output to this path instead of stdout"
    )
    parser.add_argument(
        "--fail",
        type=float,
        default=DEFAULT_FAIL,
        help="normalized per-pixel channel threshold (default: 0.016)",
    )
    parser.add_argument(
        "--failpercent",
        type=float,
        default=DEFAULT_FAILPERCENT,
        help="percent of pixels allowed over --fail (default: 1.0)",
    )
    aa_group = parser.add_mutually_exclusive_group()
    aa_group.add_argument(
        "--ignore-aa",
        dest="ignore_aa",
        action="store_true",
        help="ignore anti-aliased edge pixels in the diff budget (default: on)",
    )
    aa_group.add_argument(
        "--no-ignore-aa",
        dest="ignore_aa",
        action="store_false",
        help="count anti-aliased edge pixels in the diff budget",
    )
    parser.set_defaults(ignore_aa=True)
    parser.add_argument("--relnorm", type=float, default=None)
    parser.add_argument(
        "--retries", type=int, default=1, help="compare attempts before failure"
    )
    parser.add_argument("--backend", default="unspecified")
    parser.add_argument("--fidelity", default="unspecified")
    parser.add_argument(
        "--meta",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="additional metadata to record",
    )
    args = parser.parse_args(argv)

    try:
        _validate_thresholds(args.fail, args.failpercent, args.relnorm)
        metadata = _metadata_from_args(args)
        if args.update:
            output = update_golden(
                args.capture,
                args.golden,
                fail=args.fail,
                failpercent=args.failpercent,
                ignore_aa=args.ignore_aa,
                relnorm=args.relnorm,
                metadata=metadata,
            )
            exit_code = 0
        else:
            output = compare_golden(
                args.capture,
                args.golden,
                fail=args.fail,
                failpercent=args.failpercent,
                ignore_aa=args.ignore_aa,
                relnorm=args.relnorm,
                metadata=metadata,
                retries=args.retries,
            )
            exit_code = 0 if output["pass"] else 1
    except (OSError, ValueError) as exc:
        print(f"image_golden.py: {exc}", file=sys.stderr)
        return 2

    payload = json.dumps(output, indent=2, sort_keys=True) + "\n"
    if args.out is None:
        sys.stdout.write(payload)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload, encoding="utf-8")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
