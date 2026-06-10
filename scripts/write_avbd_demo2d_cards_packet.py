#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo2d Cards evidence packet."""

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
    "docs/plans/104-vertex-block-descent-solver/avbd-demo2d-cards-packet.json"
)
SCENE_ID = "avbd_demo2d_cards"
BENCHMARK_NAME = "BM_AvbdDemo2dCardsStep"

SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 5,
    "scene_name": "Cards",
    "scene_count": 19,
    "scene_builder": "sceneCards",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "y",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "size": (80.0, 4.0),
            "density": 0.0,
            "friction": 0.7,
            "position": (0.0, -2.0, 0.0),
        },
        "cards": {
            "count": 40,
            "levels": 5,
            "horizontal_count": 10,
            "leaning_count": 30,
            "size": (0.002, 0.4),
            "density": 1.0,
            "friction": 0.7,
            "angle_positive": 25.0 * 3.14159 / 180.0,
            "angle_negative": -25.0 * 3.14159 / 180.0,
            "angle_horizontal": 0.5 * 3.14159,
            "first_position": (0.25, 0.36, 0.5 * 3.14159),
            "last_position": (0.875, 1.62, 25.0 * 3.14159 / 180.0),
        },
    },
    "expected_counts": {
        "rigid_bodies": 41,
        "joints": 0,
        "collision_shapes": 41,
        "dynamic_bodies": 40,
        "static_bodies": 1,
    },
}


class AvbdDemo2dCardsPacketError(RuntimeError):
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
        raise AvbdDemo2dCardsPacketError("capture manifest schema_version must be 1")
    if manifest.get("scene") != SCENE_ID:
        raise AvbdDemo2dCardsPacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdDemo2dCardsPacketError("Cards capture must not switch scenes")
    if manifest.get("force_drag") is not None:
        raise AvbdDemo2dCardsPacketError("Cards capture must not force-drag")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdDemo2dCardsPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdDemo2dCardsPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdDemo2dCardsPacketError(f"{frames_dir}: frame directory not found")

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdDemo2dCardsPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdDemo2dCardsPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdDemo2dCardsPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdDemo2dCardsPacketError("capture manifest missing ui_ready")

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
        raise AvbdDemo2dCardsPacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo2dCardsPacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdDemo2dCardsPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdDemo2dCardsPacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_name(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdDemo2dCardsPacketError(f"benchmark JSON missing {BENCHMARK_NAME}")
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdDemo2dCardsPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "rigid_bodies", 41.0)
            _require_counter(row, "rigid_body_joints", 0.0)
            _require_counter(row, "collision_shapes", 41.0)
            _require_counter(row, "cards", 40.0)
            _require_counter(row, "source_scene_index", 5.0)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdDemo2dCardsPacketError(
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
            "rigid_bodies": 41,
            "rigid_body_joints": 0,
            "collision_shapes": 41,
            "cards": 40,
            "source_scene_index": 5,
            "time_step": 1.0 / 60.0,
        },
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo2dCardsPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo2dCardsPacketError(f"reference timing has non-finite {key}")
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdDemo2dCardsPacketError(f"reference timing missing integer {key}")
    return value


def _require_int_field(data: dict[str, Any], key: str, expected: int) -> None:
    value = _int_field(data, key)
    if value != expected:
        raise AvbdDemo2dCardsPacketError(
            f"reference timing expected {key}={expected}, got {value}"
        )


def _require_finite_field(
    data: dict[str, Any],
    key: str,
    expected: float,
    *,
    abs_tol: float = 1.0e-9,
) -> None:
    value = _finite_field(data, key)
    if not math.isclose(value, expected, rel_tol=0.0, abs_tol=abs_tol):
        raise AvbdDemo2dCardsPacketError(
            f"reference timing expected {key}={expected:g}, got {value:g}"
        )


def _validate_reference_timing(reference_json: Path) -> dict[str, Any]:
    data = _load_json(reference_json)
    expected = {
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "scene_index": 5,
        "scene_name": "Cards",
        "scene_builder": "sceneCards",
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdDemo2dCardsPacketError(
                f"reference timing expected {key}={value!r}, got {data.get(key)!r}"
            )

    for key, expected_value in (
        ("rigid_bodies", 41),
        ("dynamic_bodies", 40),
        ("static_bodies", 1),
        ("joints", 0),
        ("collision_shapes", 41),
        ("card_count", 40),
        ("card_levels", 5),
        ("horizontal_card_count", 10),
        ("leaning_card_count", 30),
        ("negative_card_count", 15),
        ("positive_card_count", 15),
    ):
        _require_int_field(data, key, expected_value)

    if _int_field(data, "steps") < 1:
        raise AvbdDemo2dCardsPacketError("reference timing steps must be positive")
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdDemo2dCardsPacketError(
            "reference timing warmup_steps must be non-negative"
        )
    for key, expected_value in (
        ("card_height", 0.4),
        ("card_thickness", 0.002),
        ("angle_positive", 25.0 * 3.14159 / 180.0),
        ("angle_negative", -25.0 * 3.14159 / 180.0),
        ("angle_horizontal", 0.5 * 3.14159),
        ("ground_width", 80.0),
        ("ground_height", 4.0),
        ("min_friction", 0.7),
        ("max_friction", 0.7),
    ):
        _require_finite_field(data, key, expected_value, abs_tol=1.0e-6)
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
            "card_count": data["card_count"],
            "card_levels": data["card_levels"],
            "horizontal_card_count": data["horizontal_card_count"],
            "leaning_card_count": data["leaning_card_count"],
            "negative_card_count": data["negative_card_count"],
            "positive_card_count": data["positive_card_count"],
            "card_height": data["card_height"],
            "card_thickness": data["card_thickness"],
            "angle_positive": data["angle_positive"],
            "angle_negative": data["angle_negative"],
            "angle_horizontal": data["angle_horizontal"],
            "ground_width": data["ground_width"],
            "ground_height": data["ground_height"],
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
            "DART beats the native avbd-demo2d Cards row"
            if ratio < 1.0
            else "DART does not yet beat the native avbd-demo2d Cards row"
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
        "remaining avbd-demo2d source-scene ports",
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
        "broad card-tower contact stability beyond this narrow source row",
    ]
    if not comparison["dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            "DART CPU performance for this Cards row must beat the native reference before a CPU win is claimed",
        )

    return {
        "schema_version": 1,
        "packet": "avbd_demo2d_cards_source_demo",
        "scene": SCENE_ID,
        "source_demo_reference_row": SOURCE_REFERENCE_ROW,
        "visual_capture": _validate_capture(capture_manifest),
        "dart_benchmark": dart_benchmark,
        "reference_timing": reference_timing,
        "comparison": comparison,
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py --scene avbd_demo2d_cards "
                "--frames 24 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "dart_benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo2dCardsStep$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                "python scripts/run_avbd_demo2d_reference_timing.py "
                "--source-dir <avbd-demo2d-source-dir> --scene cards "
                "--warmup-steps 128 --steps 20000 "
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
    except AvbdDemo2dCardsPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD demo2d Cards packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
