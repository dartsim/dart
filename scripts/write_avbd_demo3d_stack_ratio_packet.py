#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo3d Stack Ratio evidence packet."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from write_avbd_demo3d_static_friction_packet import (  # noqa: E402
    _artifact_label,
    _artifact_path,
    _canonical_name,
    _load_json,
    _png_dimensions,
    _row_name,
    _sha256,
)

DEFAULT_OUTPUT = Path(
    "docs/plans/104-vertex-block-descent-solver/" "avbd-demo3d-stack-ratio-packet.json"
)
SCENE_ID = "avbd_demo3d_stack_ratio"
BENCHMARK_NAME = "BM_AvbdDemo3dStackRatioStep"

SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 10,
    "scene_name": "Stack Ratio",
    "scene_count": 14,
    "scene_builder": "sceneStackRatio",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "z",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "size": (100.0, 100.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 0.0),
        },
        "boxes": {
            "count": 4,
            "initial_size": 1.0,
            "size_multiplier": 2.0,
            "sizes": (
                (1.0, 1.0, 1.0),
                (2.0, 2.0, 2.0),
                (4.0, 4.0, 4.0),
                (8.0, 8.0, 8.0),
            ),
            "centers_z": (1.0, 2.5, 5.5, 11.5),
            "density": 1.0,
            "friction": 0.5,
        },
    },
    "expected_counts": {
        "rigid_bodies": 5,
        "joints": 0,
        "collision_shapes": 5,
        "dynamic_bodies": 4,
        "static_bodies": 1,
    },
}


class AvbdDemo3dStackRatioPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--reference-timing-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _validate_capture(manifest_path: Path) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdDemo3dStackRatioPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdDemo3dStackRatioPacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdDemo3dStackRatioPacketError(
            "Stack Ratio capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdDemo3dStackRatioPacketError("Stack Ratio capture must not force-drag")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdDemo3dStackRatioPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdDemo3dStackRatioPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdDemo3dStackRatioPacketError(
            f"{frames_dir}: frame directory not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdDemo3dStackRatioPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdDemo3dStackRatioPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdDemo3dStackRatioPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdDemo3dStackRatioPacketError("capture manifest missing ui_ready")

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


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo3dStackRatioPacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dStackRatioPacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdDemo3dStackRatioPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdDemo3dStackRatioPacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_name(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdDemo3dStackRatioPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}"
        )
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdDemo3dStackRatioPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "rigid_bodies", 5.0)
            _require_counter(row, "rigid_body_joints", 0.0)
            _require_counter(row, "collision_shapes", 5.0)
            _require_counter(row, "source_scene_index", 10.0)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdDemo3dStackRatioPacketError(
            f"benchmark JSON missing representative rows for {BENCHMARK_NAME}"
        )
    median_rows = [
        row for row in representative_rows if row.get("aggregate_name") == "median"
    ]
    timing_row = median_rows[0] if median_rows else iteration_rows[0]

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": BENCHMARK_NAME,
        "context": {
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
        },
        "rows": packet_rows,
        "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time"),
        "invariants": {
            "rigid_bodies": 5,
            "rigid_body_joints": 0,
            "collision_shapes": 5,
            "source_scene_index": 10,
            "time_step": 1.0 / 60.0,
        },
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo3dStackRatioPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dStackRatioPacketError(f"reference timing has non-finite {key}")
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdDemo3dStackRatioPacketError(f"reference timing missing integer {key}")
    return value


def _validate_reference_timing(reference_json: Path) -> dict[str, Any]:
    data = _load_json(reference_json)
    expected = {
        "schema_version": 1,
        "source_demo": "avbd-demo3d",
        "source_revision": "7701bd427d55",
        "scene_index": 10,
        "scene_name": "Stack Ratio",
        "scene_builder": "sceneStackRatio",
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdDemo3dStackRatioPacketError(
                f"reference timing expected {key}={value!r}, got {data.get(key)!r}"
            )
    if _int_field(data, "rigid_bodies") != 5:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected 5 rigid bodies"
        )
    if _int_field(data, "dynamic_bodies") != 4:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected 4 dynamic bodies"
        )
    if _int_field(data, "static_bodies") != 1:
        raise AvbdDemo3dStackRatioPacketError("reference timing expected 1 static body")
    if _int_field(data, "joints") != 0:
        raise AvbdDemo3dStackRatioPacketError("reference timing expected 0 joints")
    if _int_field(data, "collision_shapes") != 5:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected 5 collision shapes"
        )
    if _int_field(data, "stack_ratio_box_count") != 4:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected stack_ratio_box_count=4"
        )
    if _int_field(data, "steps") < 1:
        raise AvbdDemo3dStackRatioPacketError("reference timing steps must be positive")
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing warmup_steps must be non-negative"
        )
    if _finite_field(data, "initial_size") != 1.0:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected initial_size=1"
        )
    if _finite_field(data, "size_multiplier") != 2.0:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected size_multiplier=2"
        )
    if _finite_field(data, "min_friction") != 0.5:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected min_friction=0.5"
        )
    if _finite_field(data, "max_friction") != 0.5:
        raise AvbdDemo3dStackRatioPacketError(
            "reference timing expected max_friction=0.5"
        )
    for key in ("elapsed_ns", "cpu_time_per_step_ns", "final_time"):
        _finite_field(data, key)

    return {
        "json_sha256": _sha256(reference_json),
        "source_demo": data["source_demo"],
        "repository": data.get("repository", SOURCE_REFERENCE_ROW["repository"]),
        "source_revision": data["source_revision"],
        "scene_index": data["scene_index"],
        "scene_name": data["scene_name"],
        "scene_builder": data["scene_builder"],
        "compiler": data.get("compiler"),
        "compile_flags": data.get("compile_flags"),
        "compile_command": data.get("compile_command"),
        "warmup_steps": data["warmup_steps"],
        "steps": data["steps"],
        "elapsed_ns": data["elapsed_ns"],
        "cpu_time_per_step_ns": data["cpu_time_per_step_ns"],
        "invariants": {
            "rigid_bodies": data["rigid_bodies"],
            "dynamic_bodies": data["dynamic_bodies"],
            "static_bodies": data["static_bodies"],
            "joints": data["joints"],
            "collision_shapes": data["collision_shapes"],
            "stack_ratio_box_count": data["stack_ratio_box_count"],
            "initial_size": data["initial_size"],
            "size_multiplier": data["size_multiplier"],
            "min_friction": data["min_friction"],
            "max_friction": data["max_friction"],
            "final_time": data["final_time"],
        },
    }


def _comparison(
    dart_benchmark: dict[str, Any],
    reference_timing: dict[str, Any],
) -> dict[str, Any]:
    dart_ns = float(dart_benchmark["cpu_time_per_step_ns"])
    reference_ns = float(reference_timing["cpu_time_per_step_ns"])
    ratio = dart_ns / reference_ns
    return {
        "dart_cpu_time_per_step_ns": dart_ns,
        "reference_cpu_time_per_step_ns": reference_ns,
        "dart_to_reference_cpu_time_ratio": ratio,
        "dart_faster_than_reference": ratio < 1.0,
        "performance_claim": (
            "DART beats the native avbd-demo3d Stack Ratio row"
            if ratio < 1.0
            else "DART does not yet beat the native avbd-demo3d Stack Ratio row"
        ),
    }


def make_packet(
    capture_manifest: Path,
    benchmark_json: Path,
    reference_timing_json: Path,
) -> dict[str, Any]:
    dart_benchmark = _validate_benchmark(benchmark_json)
    reference_timing = _validate_reference_timing(reference_timing_json)
    comparison = _comparison(dart_benchmark, reference_timing)
    remaining_gates = [
        "remaining avbd-demo3d soft/spring/rope/bridge source-scene ports",
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
        "broad size-ratio stacking and contact-coupling coverage beyond this source row",
    ]
    if not comparison["dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            "DART CPU performance for this Stack Ratio row must beat the native reference before a CPU win is claimed",
        )

    return {
        "schema_version": 1,
        "packet": "avbd_demo3d_stack_ratio_source_demo",
        "scene": SCENE_ID,
        "source_demo_reference_row": SOURCE_REFERENCE_ROW,
        "visual_capture": _validate_capture(capture_manifest),
        "dart_benchmark": dart_benchmark,
        "reference_timing": reference_timing,
        "comparison": comparison,
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py --scene avbd_demo3d_stack_ratio "
                "--frames 24 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "dart_benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo3dStackRatioStep$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                "python scripts/run_avbd_demo3d_reference_timing.py "
                "--source-dir <avbd-demo3d-source-dir> --scene stack_ratio "
                "--output <reference-timing-json>"
            ),
        },
        "remaining_gates": remaining_gates,
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    try:
        packet = make_packet(
            args.capture_manifest,
            args.benchmark_json,
            args.reference_timing_json,
        )
    except AvbdDemo3dStackRatioPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD demo3d Stack Ratio packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
