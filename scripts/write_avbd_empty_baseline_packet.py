#!/usr/bin/env python3
"""Write a validated AVBD empty source-demo baseline evidence packet."""

from __future__ import annotations

import argparse
import json
import math
import re
import struct
import sys
from hashlib import sha256
from pathlib import Path
from typing import Any

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/avbd-empty-baseline-packet.json"
)
SCENE_ID = "avbd_empty_baseline"
BENCHMARK_NAME = "BM_AvbdEmptyWorldStep"
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")

SOURCE_REFERENCE_ROWS: tuple[dict[str, Any], ...] = (
    {
        "demo": "avbd-demo2d",
        "repository": "https://github.com/savant117/avbd-demo2d",
        "revision": "74699a11f858",
        "dimension": 2,
        "scene_index": 0,
        "scene_name": "Empty",
        "scene_count": 19,
        "scene_builder": "sceneEmpty",
        "scene_effect": "solver->clear()",
        "solver_defaults": {
            "time_step": 1.0 / 60.0,
            "gravity_axis": "y",
            "gravity": -10.0,
            "iterations": 10,
        },
        "expected_counts": {
            "rigid_bodies": 0,
            "joints": 0,
            "springs": 0,
            "motors": 0,
        },
    },
    {
        "demo": "avbd-demo3d",
        "repository": "https://github.com/savant117/avbd-demo3d",
        "revision": "7701bd427d55",
        "dimension": 3,
        "scene_index": 0,
        "scene_name": "Empty",
        "scene_count": 14,
        "scene_builder": "sceneEmpty",
        "scene_effect": "solver->clear()",
        "solver_defaults": {
            "time_step": 1.0 / 60.0,
            "gravity_axis": "z",
            "gravity": -10.0,
            "iterations": 10,
        },
        "expected_counts": {
            "rigid_bodies": 0,
            "joints": 0,
            "springs": 0,
            "motors": 0,
        },
    },
)


class AvbdEmptyBaselinePacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capture-manifest",
        type=Path,
        required=True,
        help="manifest.json from scripts/capture_py_demo.py.",
    )
    parser.add_argument(
        "--benchmark-json",
        type=Path,
        required=True,
        help="Google Benchmark JSON containing BM_AvbdEmptyWorldStep.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output packet JSON path.",
    )
    return parser.parse_args(argv)


def _load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = json.load(stream)
    if not isinstance(data, dict):
        raise AvbdEmptyBaselinePacketError(f"{path}: JSON root must be an object")
    return data


def _sha256(path: Path) -> str:
    digest = sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _png_dimensions(path: Path) -> tuple[int, int]:
    with path.open("rb") as stream:
        header = stream.read(24)
    if (
        len(header) != 24
        or header[:8] != b"\x89PNG\r\n\x1a\n"
        or header[12:16] != b"IHDR"
    ):
        raise AvbdEmptyBaselinePacketError(f"{path}: expected a PNG image")
    width, height = struct.unpack(">II", header[16:24])
    if width < 1 or height < 1:
        raise AvbdEmptyBaselinePacketError(f"{path}: invalid PNG dimensions")
    return width, height


def _artifact_path(manifest_dir: Path, value: object, key: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdEmptyBaselinePacketError(f"capture manifest missing {key}")
    path = Path(value)
    if not path.is_absolute():
        path = manifest_dir / path
    return path


def _artifact_label(manifest_dir: Path, path: Path) -> str:
    try:
        return path.resolve().relative_to(manifest_dir.resolve()).as_posix()
    except ValueError:
        return path.name


def _validate_capture(manifest_path: Path) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdEmptyBaselinePacketError("capture manifest schema_version must be 1")
    if manifest.get("scene") != SCENE_ID:
        raise AvbdEmptyBaselinePacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdEmptyBaselinePacketError(
            "empty baseline capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdEmptyBaselinePacketError("empty baseline capture must not force-drag")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdEmptyBaselinePacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdEmptyBaselinePacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdEmptyBaselinePacketError(f"{frames_dir}: frame directory not found")

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdEmptyBaselinePacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    first_width, first_height = _png_dimensions(frame_paths[0])
    last_width, last_height = _png_dimensions(frame_paths[-1])
    if (first_width, first_height) != (width, height):
        raise AvbdEmptyBaselinePacketError(
            "first frame dimensions do not match screenshot"
        )
    if (last_width, last_height) != (width, height):
        raise AvbdEmptyBaselinePacketError(
            "last frame dimensions do not match screenshot"
        )

    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdEmptyBaselinePacketError("capture manifest missing ui_ready")

    return {
        "manifest_sha256": _sha256(manifest_path),
        "scene": SCENE_ID,
        "show_ui": bool(manifest.get("show_ui")),
        "ui_ready": ui_ready,
        "screenshot": {
            "file": _artifact_label(manifest_dir, screenshot),
            "sha256": _sha256(screenshot),
            "width": width,
            "height": height,
        },
        "frames": {
            "directory": _artifact_label(manifest_dir, frames_dir),
            "count": len(frame_paths),
            "first_frame": {
                "file": _artifact_label(manifest_dir, frame_paths[0]),
                "sha256": _sha256(frame_paths[0]),
            },
            "last_frame": {
                "file": _artifact_label(manifest_dir, frame_paths[-1]),
                "sha256": _sha256(frame_paths[-1]),
            },
        },
    }


def _canonical_name(name: str) -> str:
    return _AGGREGATE_SUFFIX_RE.sub("", _REPEATS_SUFFIX_RE.sub("", name))


def _row_name(row: dict[str, Any]) -> str:
    name = row.get("run_name", row.get("name", ""))
    return name if isinstance(name, str) else ""


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdEmptyBaselinePacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdEmptyBaselinePacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdEmptyBaselinePacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_name(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdEmptyBaselinePacketError(f"benchmark JSON missing {BENCHMARK_NAME}")

    packet_rows = []
    for row in matching_rows:
        rigid_bodies = _finite_counter(row, "rigid_bodies")
        multibodies = _finite_counter(row, "multibodies")
        if rigid_bodies != 0.0 or multibodies != 0.0:
            raise AvbdEmptyBaselinePacketError(
                f"{BENCHMARK_NAME}: expected zero bodies, got "
                f"rigid_bodies={rigid_bodies:g}, multibodies={multibodies:g}"
            )
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    context_subset = {
        key: context[key]
        for key in (
            "executable",
            "num_cpus",
            "mhz_per_cpu",
            "library_version",
            "library_build_type",
            "json_schema_version",
        )
        if key in context
    }
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": BENCHMARK_NAME,
        "context": context_subset,
        "rows": packet_rows,
        "invariants": {
            "rigid_bodies": 0,
            "multibodies": 0,
            "time_advances_without_simulated_bodies": True,
        },
    }


def make_packet(capture_manifest: Path, benchmark_json: Path) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "packet": "avbd_empty_source_demo_baseline",
        "scene": SCENE_ID,
        "source_demo_reference_rows": list(SOURCE_REFERENCE_ROWS),
        "visual_capture": _validate_capture(capture_manifest),
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py --scene avbd_empty_baseline "
                "--frames 8 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdEmptyWorldStep$ "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
        },
        "remaining_gates": [
            "non-empty avbd-demo2d/avbd-demo3d source-scene ports",
            "CPU reference-demo timing comparisons",
            "GPU AVBD row parity and same-hardware benchmark packets",
            "paper/site/video scene visual and performance packets",
        ],
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(args.capture_manifest, args.benchmark_json)
    except AvbdEmptyBaselinePacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD empty baseline packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
