#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo2d Spring source-demo evidence packet."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
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


class AvbdDemo2dSpringPacketError(RuntimeError):
    pass


@dataclass(frozen=True)
class SpringPacketSpec:
    output: Path
    packet: str
    scene_id: str
    benchmark_name: str
    reference_scene_arg: str
    source_reference_row: dict[str, Any]
    benchmark_counts: dict[str, float]
    reference_counts: dict[str, int]
    reference_floats: dict[str, float]
    remaining_corpus_gate: str


SPRING_SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 9,
    "scene_name": "Spring",
    "scene_count": 19,
    "scene_builder": "sceneSpring",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "y",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "anchor": {
            "count": 1,
            "size": (1.0, 1.0),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, 0.0),
        },
        "block": {
            "count": 1,
            "size": (4.0, 4.0),
            "density": 1.0,
            "friction": 0.5,
            "position": (0.0, -8.0, 0.0),
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": 1,
            "parent_anchor": (0.0, 0.0),
            "child_anchor": (0.0, 0.0),
            "rest_length": 4.0,
            "stiffness": 100.0,
        },
    },
    "expected_counts": {
        "rigid_bodies": 2,
        "dynamic_bodies": 1,
        "static_bodies": 1,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 2,
    },
}

SPRING_RATIO_SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 10,
    "scene_name": "Spring Ratio",
    "scene_count": 19,
    "scene_builder": "sceneSpringsRatio",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "y",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "links": {
            "count": 8,
            "size": (1.0, 0.5),
            "dynamic_density": 1.0,
            "static_endpoint_density": 0.0,
            "friction": 0.5,
            "first_position": (0.0, 10.0, 0.0),
            "x_spacing": 4.0,
            "static_endpoint_indices": (0, 7),
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": 7,
            "parent_anchor": (0.5, 0.0),
            "child_anchor": (-0.5, 0.0),
            "rest_length": 0.1,
            "stiffness_pattern": (
                1.0e6,
                1.0e3,
                1.0e6,
                1.0e3,
                1.0e6,
                1.0e3,
                1.0e6,
            ),
            "high_stiffness_springs": 4,
            "low_stiffness_springs": 3,
        },
    },
    "expected_counts": {
        "rigid_bodies": 8,
        "dynamic_bodies": 6,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 8,
    },
}

SPRING_SPEC = SpringPacketSpec(
    output=Path(
        "docs/plans/104-vertex-block-descent-solver/avbd-demo2d-spring-packet.json"
    ),
    packet="avbd_demo2d_spring_source_demo",
    scene_id="avbd_demo2d_spring",
    benchmark_name="BM_AvbdDemo2dSpringStep",
    reference_scene_arg="spring",
    source_reference_row=SPRING_SOURCE_REFERENCE_ROW,
    benchmark_counts={
        "rigid_bodies": 2.0,
        "rigid_body_joints": 0.0,
        "distance_springs": 1.0,
        "collision_shapes": 2.0,
        "ignored_collision_pairs": 1.0,
        "source_scene_index": 9.0,
    },
    reference_counts={
        "rigid_bodies": 2,
        "dynamic_bodies": 1,
        "static_bodies": 1,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 2,
    },
    reference_floats={
        "min_friction": 0.5,
        "max_friction": 0.5,
        "min_spring_stiffness": 100.0,
        "max_spring_stiffness": 100.0,
        "spring_rest_length": 4.0,
    },
    remaining_corpus_gate="broad spring-corpus coverage beyond this source row",
)

SPRING_RATIO_SPEC = SpringPacketSpec(
    output=Path(
        "docs/plans/104-vertex-block-descent-solver/"
        "avbd-demo2d-spring-ratio-packet.json"
    ),
    packet="avbd_demo2d_spring_ratio_source_demo",
    scene_id="avbd_demo2d_spring_ratio",
    benchmark_name="BM_AvbdDemo2dSpringRatioStep",
    reference_scene_arg="spring_ratio",
    source_reference_row=SPRING_RATIO_SOURCE_REFERENCE_ROW,
    benchmark_counts={
        "rigid_bodies": 8.0,
        "rigid_body_joints": 0.0,
        "distance_springs": 7.0,
        "collision_shapes": 8.0,
        "ignored_collision_pairs": 7.0,
        "source_scene_index": 10.0,
    },
    reference_counts={
        "rigid_bodies": 8,
        "dynamic_bodies": 6,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 7,
        "collision_shapes": 8,
        "spring_link_count": 8,
        "high_stiffness_springs": 4,
        "low_stiffness_springs": 3,
    },
    reference_floats={
        "min_friction": 0.5,
        "max_friction": 0.5,
        "min_spring_stiffness": 1.0e3,
        "max_spring_stiffness": 1.0e6,
        "spring_rest_length": 0.1,
    },
    remaining_corpus_gate=(
        "broader stiffness-ratio sweep coverage beyond this source row"
    ),
)


def parse_args(argv: list[str], spec: SpringPacketSpec) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--reference-timing-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=spec.output)
    return parser.parse_args(argv)


def _validate_capture(manifest_path: Path, spec: SpringPacketSpec) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdDemo2dSpringPacketError("capture manifest schema_version must be 1")
    if manifest.get("scene") != spec.scene_id:
        raise AvbdDemo2dSpringPacketError(f"capture scene must be {spec.scene_id}")
    if manifest.get("switch_scene") is not None:
        raise AvbdDemo2dSpringPacketError(
            f"{spec.source_reference_row['scene_name']} capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdDemo2dSpringPacketError(
            f"{spec.source_reference_row['scene_name']} capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdDemo2dSpringPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdDemo2dSpringPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdDemo2dSpringPacketError(f"{frames_dir}: frame directory not found")

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdDemo2dSpringPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdDemo2dSpringPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdDemo2dSpringPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdDemo2dSpringPacketError("capture manifest missing ui_ready")

    return {
        "manifest_sha256": _sha256(manifest_path),
        "scene": spec.scene_id,
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


def _finite_counter(row: dict[str, Any], key: str, spec: SpringPacketSpec) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo2dSpringPacketError(f"{spec.benchmark_name}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo2dSpringPacketError(f"{spec.benchmark_name}: non-finite {key}")
    return value


def _require_counter(
    row: dict[str, Any],
    key: str,
    expected: float,
    spec: SpringPacketSpec,
) -> None:
    value = _finite_counter(row, key, spec)
    if value != expected:
        raise AvbdDemo2dSpringPacketError(
            f"{spec.benchmark_name}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(
    benchmark_json: Path,
    spec: SpringPacketSpec,
) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdDemo2dSpringPacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict)
        and _canonical_name(_row_name(row)) == spec.benchmark_name
    ]
    if not matching_rows:
        raise AvbdDemo2dSpringPacketError(
            f"benchmark JSON missing {spec.benchmark_name}"
        )
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdDemo2dSpringPacketError(
            f"benchmark JSON missing iteration rows for {spec.benchmark_name}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            for key, expected in spec.benchmark_counts.items():
                _require_counter(row, key, expected, spec)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key, spec)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdDemo2dSpringPacketError(
            f"benchmark JSON missing representative rows for {spec.benchmark_name}"
        )
    median_rows = [
        row for row in representative_rows if row.get("aggregate_name") == "median"
    ]
    timing_row = median_rows[0] if median_rows else iteration_rows[0]

    context = data.get("context", {})
    if not isinstance(context, dict):
        context = {}
    invariants: dict[str, int | float] = {
        key: int(value) if value.is_integer() else value
        for key, value in spec.benchmark_counts.items()
    }
    invariants["time_step"] = 1.0 / 60.0
    return {
        "json_sha256": _sha256(benchmark_json),
        "benchmark": spec.benchmark_name,
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
        "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time", spec),
        "invariants": invariants,
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo2dSpringPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo2dSpringPacketError(f"reference timing has non-finite {key}")
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdDemo2dSpringPacketError(f"reference timing missing integer {key}")
    return value


def _validate_reference_timing(
    reference_json: Path,
    spec: SpringPacketSpec,
) -> dict[str, Any]:
    data = _load_json(reference_json)
    source_demo = spec.source_reference_row["demo"]
    source_revision = spec.source_reference_row["revision"]
    expected = {
        "schema_version": 1,
        "source_demo": source_demo,
        "source_revision": source_revision,
        "scene_index": spec.source_reference_row["scene_index"],
        "scene_name": spec.source_reference_row["scene_name"],
        "scene_builder": spec.source_reference_row["scene_builder"],
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdDemo2dSpringPacketError(
                f"reference timing expected {key}={value!r}, got {data.get(key)!r}"
            )
    for key, expected_value in spec.reference_counts.items():
        if _int_field(data, key) != expected_value:
            raise AvbdDemo2dSpringPacketError(
                f"reference timing expected {key}={expected_value}"
            )
    for key, expected_value in spec.reference_floats.items():
        if _finite_field(data, key) != expected_value:
            raise AvbdDemo2dSpringPacketError(
                f"reference timing expected {key}={expected_value:g}"
            )
    if _int_field(data, "steps") < 1:
        raise AvbdDemo2dSpringPacketError("reference timing steps must be positive")
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdDemo2dSpringPacketError(
            "reference timing warmup_steps must be non-negative"
        )
    for key in ("elapsed_ns", "cpu_time_per_step_ns", "final_time"):
        _finite_field(data, key)

    invariant_keys = (
        tuple(spec.reference_counts) + tuple(spec.reference_floats) + ("final_time",)
    )
    return {
        "json_sha256": _sha256(reference_json),
        "source_demo": data["source_demo"],
        "repository": data.get("repository", spec.source_reference_row["repository"]),
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
        "invariants": {key: data[key] for key in invariant_keys},
    }


def _comparison(
    dart_benchmark: dict[str, Any],
    reference_timing: dict[str, Any],
    spec: SpringPacketSpec,
) -> dict[str, Any]:
    dart_ns = float(dart_benchmark["cpu_time_per_step_ns"])
    reference_ns = float(reference_timing["cpu_time_per_step_ns"])
    ratio = dart_ns / reference_ns
    scene_name = spec.source_reference_row["scene_name"]
    source_demo = spec.source_reference_row["demo"]
    return {
        "dart_cpu_time_per_step_ns": dart_ns,
        "reference_cpu_time_per_step_ns": reference_ns,
        "dart_to_reference_cpu_time_ratio": ratio,
        "dart_faster_than_reference": ratio < 1.0,
        "performance_claim": (
            f"DART beats the native {source_demo} {scene_name} row"
            if ratio < 1.0
            else f"DART does not yet beat the native {source_demo} {scene_name} row"
        ),
    }


def make_packet(
    capture_manifest: Path,
    benchmark_json: Path,
    reference_timing_json: Path,
    spec: SpringPacketSpec = SPRING_SPEC,
) -> dict[str, Any]:
    dart_benchmark = _validate_benchmark(benchmark_json, spec)
    reference_timing = _validate_reference_timing(reference_timing_json, spec)
    comparison = _comparison(dart_benchmark, reference_timing, spec)
    scene_name = spec.source_reference_row["scene_name"]
    remaining_gates = [
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
        spec.remaining_corpus_gate,
    ]
    if not comparison["dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            f"DART CPU performance for this {scene_name} row must beat the "
            "native reference before a CPU win is claimed",
        )
    source_demo = spec.source_reference_row["demo"]
    source_demo_module = source_demo.replace("-", "_")

    return {
        "schema_version": 1,
        "packet": spec.packet,
        "scene": spec.scene_id,
        "source_demo_reference_row": spec.source_reference_row,
        "visual_capture": _validate_capture(capture_manifest, spec),
        "dart_benchmark": dart_benchmark,
        "reference_timing": reference_timing,
        "comparison": comparison,
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                f"python scripts/capture_py_demo.py --scene {spec.scene_id} "
                "--frames 24 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "dart_benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                f"--benchmark_filter={spec.benchmark_name}$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                f"python scripts/run_{source_demo_module}_reference_timing.py "
                f"--source-dir <{source_demo}-source-dir> "
                f"--scene {spec.reference_scene_arg} "
                "--output <reference-timing-json>"
            ),
        },
        "remaining_gates": remaining_gates,
    }


def write_packet(path: Path, packet: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(packet, indent=2, sort_keys=True) + "\n")


def main_for_spec(argv: list[str], spec: SpringPacketSpec) -> int:
    args = parse_args(argv, spec)
    try:
        packet = make_packet(
            args.capture_manifest,
            args.benchmark_json,
            args.reference_timing_json,
            spec,
        )
    except AvbdDemo2dSpringPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    source_demo = spec.source_reference_row["demo"]
    print(
        f"Wrote AVBD {source_demo} {spec.source_reference_row['scene_name']} packet: "
        f"{args.output}"
    )
    return 0


def main(argv: list[str]) -> int:
    return main_for_spec(argv, SPRING_SPEC)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
