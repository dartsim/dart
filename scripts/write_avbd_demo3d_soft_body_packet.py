#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo3d Soft Body source-demo evidence packet."""

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
    "docs/plans/104-vertex-block-descent-solver/" "avbd-demo3d-soft-body-packet.json"
)
SCENE_ID = "avbd_demo3d_soft_body"
BENCHMARK_NAME = "BM_AvbdDemo3dSoftBodyStep"

SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 11,
    "scene_name": "Soft Body",
    "scene_count": 14,
    "scene_builder": "sceneSoftBody",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "z",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "count": 1,
            "size": (100.0, 100.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 0.0),
        },
        "soft_cell": {
            "count": 192,
            "size": (0.8, 0.8, 0.8),
            "density": 1.0,
            "friction": 0.5,
            "width": 4,
            "depth": 4,
            "height": 4,
            "stacks": 3,
            "base_z": 8.0,
            "stack_gap": 2.0,
            "first_position": (-1.2, -1.2, 8.0),
        },
    },
    "source_constraints": {
        "finite_all_axis_fixed_joints": {
            "count": 432,
            "x_count": 144,
            "y_count": 144,
            "z_count": 144,
            "x_parent_anchor": (0.4, 0.0, 0.0),
            "x_child_anchor": (-0.4, 0.0, 0.0),
            "y_parent_anchor": (0.0, 0.4, 0.0),
            "y_child_anchor": (0.0, -0.4, 0.0),
            "z_parent_anchor": (0.0, 0.0, 0.4),
            "z_child_anchor": (0.0, 0.0, -0.4),
            "linear_stiffness": 1000.0,
            "angular_stiffness": 250.0,
            "fracture": "infinity",
        },
    },
    "source_collision_filters": {
        "diagonal_ignore_collision_pairs": 648,
    },
    "expected_counts": {
        "rigid_bodies": 193,
        "dynamic_bodies": 192,
        "static_bodies": 1,
        "joints": 432,
        "fixed_joints": 432,
        "finite_stiffness_fixed_joints": 432,
        "collision_shapes": 193,
    },
}


class AvbdDemo3dSoftBodyPacketError(RuntimeError):
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
        raise AvbdDemo3dSoftBodyPacketError("capture manifest schema_version must be 1")
    if manifest.get("scene") != SCENE_ID:
        raise AvbdDemo3dSoftBodyPacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdDemo3dSoftBodyPacketError("Soft Body capture must not switch scenes")
    if manifest.get("force_drag") is not None:
        raise AvbdDemo3dSoftBodyPacketError("Soft Body capture must not force-drag")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdDemo3dSoftBodyPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdDemo3dSoftBodyPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdDemo3dSoftBodyPacketError(f"{frames_dir}: frame directory not found")

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdDemo3dSoftBodyPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdDemo3dSoftBodyPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdDemo3dSoftBodyPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdDemo3dSoftBodyPacketError("capture manifest missing ui_ready")

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
        raise AvbdDemo3dSoftBodyPacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dSoftBodyPacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdDemo3dSoftBodyPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdDemo3dSoftBodyPacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_name(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdDemo3dSoftBodyPacketError(f"benchmark JSON missing {BENCHMARK_NAME}")
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdDemo3dSoftBodyPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "rigid_bodies", 193.0)
            _require_counter(row, "rigid_body_joints", 432.0)
            _require_counter(row, "fixed_joints", 432.0)
            _require_counter(row, "finite_stiffness_fixed_joints", 432.0)
            _require_counter(row, "collision_shapes", 193.0)
            _require_counter(row, "ignored_collision_pairs", 648.0)
            _require_counter(row, "source_scene_index", 11.0)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdDemo3dSoftBodyPacketError(
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
            "rigid_bodies": 193,
            "rigid_body_joints": 432,
            "fixed_joints": 432,
            "finite_stiffness_fixed_joints": 432,
            "collision_shapes": 193,
            "ignored_collision_pairs": 648,
            "source_scene_index": 11,
            "time_step": 1.0 / 60.0,
        },
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo3dSoftBodyPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dSoftBodyPacketError(f"reference timing has non-finite {key}")
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdDemo3dSoftBodyPacketError(f"reference timing missing integer {key}")
    return value


def _validate_reference_timing(reference_json: Path) -> dict[str, Any]:
    data = _load_json(reference_json)
    expected = {
        "schema_version": 1,
        "source_demo": "avbd-demo3d",
        "source_revision": "7701bd427d55",
        "scene_index": 11,
        "scene_name": "Soft Body",
        "scene_builder": "sceneSoftBody",
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdDemo3dSoftBodyPacketError(
                f"reference timing expected {key}={value!r}, got {data.get(key)!r}"
            )
    required_counts = {
        "rigid_bodies": 193,
        "dynamic_bodies": 192,
        "static_bodies": 1,
        "joints": 432,
        "fixed_joints": 432,
        "finite_stiffness_fixed_joints": 432,
        "collision_shapes": 193,
        "soft_body_width": 4,
        "soft_body_depth": 4,
        "soft_body_height": 4,
        "soft_body_stacks": 3,
        "soft_body_cells": 192,
        "diagonal_ignore_collision_pairs": 648,
    }
    for key, expected_value in required_counts.items():
        if _int_field(data, key) != expected_value:
            raise AvbdDemo3dSoftBodyPacketError(
                f"reference timing expected {key}={expected_value}"
            )
    if _int_field(data, "steps") < 1:
        raise AvbdDemo3dSoftBodyPacketError("reference timing steps must be positive")
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdDemo3dSoftBodyPacketError(
            "reference timing warmup_steps must be non-negative"
        )
    expected_floats = {
        "min_friction": 0.5,
        "max_friction": 0.5,
        "min_linear_stiffness": 1000.0,
        "max_linear_stiffness": 1000.0,
        "min_angular_stiffness": 250.0,
        "max_angular_stiffness": 250.0,
    }
    for key, expected_value in expected_floats.items():
        if _finite_field(data, key) != expected_value:
            raise AvbdDemo3dSoftBodyPacketError(
                f"reference timing expected {key}={expected_value:g}"
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
            key: data[key]
            for key in (
                "rigid_bodies",
                "dynamic_bodies",
                "static_bodies",
                "joints",
                "fixed_joints",
                "finite_stiffness_fixed_joints",
                "collision_shapes",
                "soft_body_width",
                "soft_body_depth",
                "soft_body_height",
                "soft_body_stacks",
                "soft_body_cells",
                "diagonal_ignore_collision_pairs",
                "min_linear_stiffness",
                "max_linear_stiffness",
                "min_angular_stiffness",
                "max_angular_stiffness",
                "min_friction",
                "max_friction",
                "final_time",
            )
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
            "DART beats the native avbd-demo3d Soft Body row"
            if ratio < 1.0
            else "DART does not yet beat the native avbd-demo3d Soft Body row"
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
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
        "broader soft-body deformation and contact stability coverage beyond this source row",
    ]
    if not comparison["dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            "DART CPU performance for this Soft Body row must beat the native reference before a CPU win is claimed",
        )

    return {
        "schema_version": 1,
        "packet": "avbd_demo3d_soft_body_source_demo",
        "scene": SCENE_ID,
        "source_demo_reference_row": SOURCE_REFERENCE_ROW,
        "visual_capture": _validate_capture(capture_manifest),
        "dart_benchmark": dart_benchmark,
        "reference_timing": reference_timing,
        "comparison": comparison,
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py --scene avbd_demo3d_soft_body "
                "--frames 8 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "dart_benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo3dSoftBodyStep$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                "python scripts/run_avbd_demo3d_reference_timing.py "
                "--source-dir <avbd-demo3d-source-dir> --scene soft_body "
                "--warmup-steps 16 --steps 512 --output <reference-timing-json>"
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
    except AvbdDemo3dSoftBodyPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD demo3d Soft Body packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
