#!/usr/bin/env python3
"""Emit machine-readable image verification verdicts for DART captures."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from _image_tools import ImageData, read_image

SCHEMA_VERSION = "dart.image_verdict/v1"
DEFAULT_FAIL = 0.016
DEFAULT_FAILPERCENT = 1.0


def build_verdict(
    image_path: Path,
    reference_path: Path | None = None,
    *,
    fail: float = DEFAULT_FAIL,
    failpercent: float = DEFAULT_FAILPERCENT,
    ignore_aa: bool = True,
    relnorm: float | None = None,
    metadata: dict[str, str] | None = None,
    include_ssim: bool = True,
    require_contrast: bool = False,
) -> dict[str, Any]:
    image = read_image(image_path)
    reasons: list[str] = []
    checks: dict[str, Any] = {
        "non_blank": analyze_non_blank(image),
        "contrast": analyze_contrast(image),
    }
    if not checks["non_blank"]["pass"]:
        reasons.extend(checks["non_blank"]["reasons"])
    # Contrast is scene-dependent: a legitimately flat but non-blank render
    # (soft lighting, a single primitive) is not a defect, so contrast is
    # always reported but only gates `pass` when the caller opts in via
    # require_contrast (e.g. a full-demo-scene golden gate). This mirrors the
    # A/B finding that the strict shadow/lighting heuristic false-fails minimal
    # scenes.
    if require_contrast and not checks["contrast"]["pass"]:
        reasons.extend(checks["contrast"]["reasons"])

    verdict: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "image": {
            "path": str(image_path),
            "width": image.width,
            "height": image.height,
        },
        "metadata": metadata or {},
        "checks": checks,
        "thresholds_used": {
            "non_blank": {"min_nonzero_pixels": 1},
            "contrast": checks["contrast"]["thresholds"],
            "require_contrast": require_contrast,
            "diff": {
                "fail": fail,
                "failpercent": failpercent,
                "ignore_aa": ignore_aa,
                "relnorm": relnorm,
            },
        },
        "pass": False,
        "reasons": reasons,
    }

    if reference_path is not None:
        reference = read_image(reference_path)
        verdict["reference"] = {
            "path": str(reference_path),
            "width": reference.width,
            "height": reference.height,
        }
        diff = analyze_diff(
            image,
            reference,
            fail=fail,
            failpercent=failpercent,
            ignore_aa=ignore_aa,
            relnorm=relnorm,
        )
        checks["diff"] = diff
        if not diff["pass"]:
            reasons.extend(diff["reasons"])
        if include_ssim:
            checks["ssim"] = analyze_ssim(image, reference)

    verdict["pass"] = not reasons
    verdict["reasons"] = reasons
    return verdict


def analyze_non_blank(image: ImageData) -> dict[str, Any]:
    nonzero_pixels = 0
    for offset in range(0, len(image.pixels), 3):
        if any(image.pixels[offset : offset + 3]):
            nonzero_pixels += 1
    passed = nonzero_pixels > 0
    return {
        "pass": passed,
        "nonzero_pixels": nonzero_pixels,
        "total_pixels": image.pixel_count,
        "reasons": [] if passed else ["image contains only zero-valued pixels"],
    }


def analyze_contrast(image: ImageData) -> dict[str, Any]:
    x0, y0, x1, y1 = _scene_region(image.width, image.height)
    values = []
    for y in range(y0, y1):
        row_offset = y * image.width * 3
        for x in range(x0, x1):
            offset = row_offset + x * 3
            values.append(_luminance_bytes(image.pixels[offset : offset + 3]))

    thresholds: dict[str, Any] = {
        "dark_luminance_lt": 55.0,
        "mid_luminance_min": 70.0,
        "mid_luminance_max": 170.0,
        "bright_luminance_gt": 190.0,
        "min_spread": 90.0,
    }
    if not values:
        thresholds.update(
            {
                "min_dark_pixels": 32,
                "min_mid_pixels": 32,
                "min_bright_pixels": 32,
            }
        )
        return {
            "pass": False,
            "region": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
            "sample_count": 0,
            "thresholds": thresholds,
            "reasons": ["scene luminance probe region is empty"],
        }

    values.sort()
    dark_count = sum(value < 55.0 for value in values)
    mid_count = sum(70.0 <= value <= 170.0 for value in values)
    bright_count = sum(value > 190.0 for value in values)
    p05 = values[int(len(values) * 0.05)]
    p95 = values[int(len(values) * 0.95)]
    spread = p95 - p05

    min_dark = max(32, len(values) // 200)
    min_mid = max(32, len(values) // 400)
    min_bright = max(32, len(values) // 200)
    min_spread = 90.0
    thresholds.update(
        {
            "min_dark_pixels": min_dark,
            "min_mid_pixels": min_mid,
            "min_bright_pixels": min_bright,
        }
    )

    failures = []
    if dark_count < min_dark:
        failures.append(f"dark pixels {dark_count} < {min_dark}")
    if mid_count < min_mid:
        failures.append(f"mid-tone pixels {mid_count} < {min_mid}")
    if bright_count < min_bright:
        failures.append(f"bright pixels {bright_count} < {min_bright}")
    if spread < min_spread:
        failures.append(f"luminance spread {spread:.1f} < {min_spread:.1f}")

    return {
        "pass": not failures,
        "region": {"x0": x0, "y0": y0, "x1": x1, "y1": y1},
        "sample_count": len(values),
        "dark_pixels": dark_count,
        "mid_pixels": mid_count,
        "bright_pixels": bright_count,
        "p05": round(p05, 3),
        "p95": round(p95, 3),
        "spread": round(spread, 3),
        "thresholds": thresholds,
        "reasons": failures,
    }


def analyze_diff(
    image: ImageData,
    reference: ImageData,
    *,
    fail: float = DEFAULT_FAIL,
    failpercent: float = DEFAULT_FAILPERCENT,
    ignore_aa: bool = True,
    relnorm: float | None = None,
) -> dict[str, Any]:
    if (image.width, image.height) != (reference.width, reference.height):
        reason = (
            "image dimensions "
            f"{image.width}x{image.height} do not match reference "
            f"{reference.width}x{reference.height}"
        )
        return {
            "pass": False,
            "reason": reason,
            "reasons": [reason],
            "mean_abs": None,
            "max_abs": None,
            "pct_pixels_over_threshold": None,
            "per_channel": None,
        }

    channel_sum = [0, 0, 0]
    channel_max = [0, 0, 0]
    over_threshold = 0
    raw_over_threshold = 0
    ignored_aa = 0
    squared_sum = 0.0
    reference_squared_sum = 0.0

    for pixel in range(image.pixel_count):
        offset = pixel * 3
        diffs = [
            abs(image.pixels[offset + channel] - reference.pixels[offset + channel])
            for channel in range(3)
        ]
        for channel, value in enumerate(diffs):
            channel_sum[channel] += value
            channel_max[channel] = max(channel_max[channel], value)
            squared_sum += value * value
            reference_value = reference.pixels[offset + channel]
            reference_squared_sum += reference_value * reference_value
        pixel_error = max(diffs) / 255.0
        if pixel_error > fail:
            raw_over_threshold += 1
            x = pixel % image.width
            y = pixel // image.width
            if ignore_aa and _is_antialiased_pixel(image, reference, x, y):
                ignored_aa += 1
            else:
                over_threshold += 1

    total_channels = image.pixel_count * 3
    total_abs = sum(channel_sum)
    pct_pixels_over_threshold = over_threshold / image.pixel_count * 100.0
    raw_pct_pixels_over_threshold = raw_over_threshold / image.pixel_count * 100.0
    mean_abs = total_abs / total_channels / 255.0
    max_abs = max(channel_max) / 255.0
    per_channel = {
        name: {
            "mean_abs": round(channel_sum[index] / image.pixel_count / 255.0, 6),
            "max_abs": round(channel_max[index] / 255.0, 6),
        }
        for index, name in enumerate(("red", "green", "blue"))
    }

    reasons = []
    if pct_pixels_over_threshold > failpercent:
        reasons.append(
            "diff pixels over threshold "
            f"{pct_pixels_over_threshold:.3f}% > {failpercent:.3f}%"
        )

    relative_norm = None
    if relnorm is not None:
        if reference_squared_sum == 0.0:
            relative_norm = 0.0 if squared_sum == 0.0 else float("inf")
        else:
            relative_norm = (squared_sum / reference_squared_sum) ** 0.5
        if relative_norm > relnorm:
            reasons.append(f"relative norm {relative_norm:.6f} > {relnorm:.6f}")

    result: dict[str, Any] = {
        "pass": not reasons,
        "mean_abs": round(mean_abs, 6),
        "max_abs": round(max_abs, 6),
        "pct_pixels_over_threshold": round(pct_pixels_over_threshold, 6),
        "raw_pct_pixels_over_threshold": round(raw_pct_pixels_over_threshold, 6),
        "pixels_over_threshold": over_threshold,
        "raw_pixels_over_threshold": raw_over_threshold,
        "ignored_aa_pixels": ignored_aa,
        "total_pixels": image.pixel_count,
        "per_channel": per_channel,
        "thresholds": {
            "fail": fail,
            "failpercent": failpercent,
            "ignore_aa": ignore_aa,
            "relnorm": relnorm,
        },
        "reasons": reasons,
    }
    if relnorm is not None:
        result["relative_norm"] = (
            "inf" if relative_norm == float("inf") else round(relative_norm or 0.0, 6)
        )
    return result


def analyze_ssim(image: ImageData, reference: ImageData) -> dict[str, Any]:
    if (image.width, image.height) != (reference.width, reference.height):
        return {
            "method": "global_luminance",
            "value": None,
            "reason": "dimension mismatch",
        }
    numpy, reason = _load_numpy()
    if numpy is None:
        return {
            "method": "global_luminance",
            "value": None,
            "reason": reason,
        }

    image_rgb = (
        numpy.frombuffer(image.pixels, dtype=numpy.uint8)
        .reshape((image.height, image.width, 3))
        .astype(numpy.float64)
        / 255.0
    )
    reference_rgb = (
        numpy.frombuffer(reference.pixels, dtype=numpy.uint8)
        .reshape((reference.height, reference.width, 3))
        .astype(numpy.float64)
        / 255.0
    )
    weights = numpy.array([0.2126, 0.7152, 0.0722], dtype=numpy.float64)
    x = numpy.tensordot(image_rgb, weights, axes=([2], [0])).ravel()
    y = numpy.tensordot(reference_rgb, weights, axes=([2], [0])).ravel()

    c1 = 0.01**2
    c2 = 0.03**2
    mean_x = float(numpy.mean(x))
    mean_y = float(numpy.mean(y))
    var_x = float(numpy.mean((x - mean_x) ** 2))
    var_y = float(numpy.mean((y - mean_y) ** 2))
    covariance = float(numpy.mean((x - mean_x) * (y - mean_y)))
    denominator = (mean_x**2 + mean_y**2 + c1) * (var_x + var_y + c2)
    if denominator == 0.0:
        value = 1.0 if mean_x == mean_y and var_x == var_y else 0.0
    else:
        value = ((2 * mean_x * mean_y + c1) * (2 * covariance + c2)) / denominator
    return {
        "method": "global_luminance",
        "value": round(max(-1.0, min(1.0, value)), 6),
    }


def parse_meta(values: list[str] | None) -> dict[str, str]:
    metadata: dict[str, str] = {}
    for value in values or []:
        if "=" not in value:
            raise ValueError(f"--meta must be key=value, got {value!r}")
        key, item = value.split("=", 1)
        key = key.strip()
        if not key:
            raise ValueError(f"--meta key must not be empty in {value!r}")
        metadata[key] = item
    return metadata


def _load_numpy():
    try:
        import numpy
    except ImportError as exc:
        return None, f"numpy unavailable: {exc}"
    return numpy, None


def _scene_region(width: int, height: int) -> tuple[int, int, int, int]:
    return (
        max(0, int(width * 0.40)),
        max(0, int(height * 0.45)),
        min(width, int(width * 0.97)),
        min(height, int(height * 0.90)),
    )


def _luminance_bytes(pixel: bytes) -> float:
    red, green, blue = pixel
    return 0.2126 * red + 0.7152 * green + 0.0722 * blue


def _luminance_at(image: ImageData, x: int, y: int) -> float:
    offset = (y * image.width + x) * 3
    return _luminance_bytes(image.pixels[offset : offset + 3])


def _is_antialiased_pixel(
    image: ImageData, reference: ImageData, x: int, y: int
) -> bool:
    image_center = _luminance_at(image, x, y)
    reference_center = _luminance_at(reference, x, y)
    return _neighborhood_spans(reference, x, y, image_center) or _neighborhood_spans(
        image, x, y, reference_center
    )


def _neighborhood_spans(image: ImageData, x: int, y: int, other_center: float) -> bool:
    center = _luminance_at(image, x, y)
    values = []
    for yy in range(max(0, y - 1), min(image.height, y + 2)):
        for xx in range(max(0, x - 1), min(image.width, x + 2)):
            if xx == x and yy == y:
                continue
            values.append(_luminance_at(image, xx, yy))
    if not values:
        return False
    darker = any(value < center - 1.0 for value in values)
    brighter = any(value > center + 1.0 for value in values)
    if not (darker and brighter):
        return False
    return min(values) - 1.0 <= other_center <= max(values) + 1.0


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
        description="Emit a JSON image verdict for one capture, optionally against a golden."
    )
    parser.add_argument("image", type=Path)
    parser.add_argument("reference", type=Path, nargs="?")
    parser.add_argument(
        "--out", type=Path, help="write JSON to this path instead of stdout"
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
    parser.add_argument(
        "--ignore-aa",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="ignore anti-aliased edge pixels in the diff budget (default: on)",
    )
    parser.add_argument(
        "--relnorm",
        type=float,
        default=None,
        help="optional relative L2 norm threshold, Habitat-style",
    )
    parser.add_argument(
        "--meta",
        action="append",
        default=[],
        metavar="KEY=VALUE",
        help="renderer/capture metadata to record in the verdict",
    )
    parser.add_argument(
        "--require-contrast",
        action="store_true",
        help="gate pass/fail on the shadow/lighting contrast check; off by "
        "default because flat non-blank renders are legitimate",
    )
    args = parser.parse_args(argv)

    try:
        _validate_thresholds(args.fail, args.failpercent, args.relnorm)
        verdict = build_verdict(
            args.image,
            args.reference,
            fail=args.fail,
            failpercent=args.failpercent,
            ignore_aa=args.ignore_aa,
            relnorm=args.relnorm,
            metadata=parse_meta(args.meta),
            require_contrast=args.require_contrast,
        )
    except (OSError, ValueError) as exc:
        print(f"image_verdict.py: {exc}", file=sys.stderr)
        return 2

    payload = json.dumps(verdict, indent=2, sort_keys=True) + "\n"
    if args.out is None:
        sys.stdout.write(payload)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(payload, encoding="utf-8")
    return 0 if verdict["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
