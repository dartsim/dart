#!/usr/bin/env python3
"""Write a validated AVBD avbd-demo3d Ground source-demo evidence packet."""

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
    "docs/plans/104-vertex-block-descent-solver/avbd-demo3d-ground-packet.json"
)
SCENE_ID = "avbd_demo3d_ground"
BENCHMARK_NAME = "BM_AvbdDemo3dGroundStep"
_AGGREGATE_SUFFIX_RE = re.compile(r"_(?:mean|median|stddev|cv)$")
_REPEATS_SUFFIX_RE = re.compile(r"/repeats:\d+")

SOURCE_REFERENCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 1,
    "scene_name": "Ground",
    "scene_count": 14,
    "scene_builder": "sceneGround",
    "solver_defaults": {
        "time_step": 1.0 / 60.0,
        "gravity_axis": "z",
        "gravity": -10.0,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {"size": (100.0, 100.0, 1.0), "density": 0.0, "friction": 0.5},
        "box": {"size": (1.0, 1.0, 1.0), "density": 1.0, "friction": 0.5},
    },
    "expected_counts": {
        "rigid_bodies": 2,
        "joints": 0,
        "collision_shapes": 2,
    },
}


class AvbdDemo3dGroundPacketError(RuntimeError):
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
        help="Google Benchmark JSON containing BM_AvbdDemo3dGroundStep.",
    )
    parser.add_argument(
        "--reference-timing-json",
        type=Path,
        required=True,
        help="JSON from scripts/run_avbd_demo3d_reference_timing.py.",
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
        raise AvbdDemo3dGroundPacketError(f"{path}: JSON root must be an object")
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
        raise AvbdDemo3dGroundPacketError(f"{path}: expected a PNG image")
    width, height = struct.unpack(">II", header[16:24])
    if width < 1 or height < 1:
        raise AvbdDemo3dGroundPacketError(f"{path}: invalid PNG dimensions")
    return width, height


def _artifact_path(manifest_dir: Path, value: object, key: str) -> Path:
    if not isinstance(value, str) or not value:
        raise AvbdDemo3dGroundPacketError(f"capture manifest missing {key}")
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
        raise AvbdDemo3dGroundPacketError("capture manifest schema_version must be 1")
    if manifest.get("scene") != SCENE_ID:
        raise AvbdDemo3dGroundPacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdDemo3dGroundPacketError("Ground capture must not switch scenes")
    if manifest.get("force_drag") is not None:
        raise AvbdDemo3dGroundPacketError("Ground capture must not force-drag")

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdDemo3dGroundPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdDemo3dGroundPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdDemo3dGroundPacketError(f"{frames_dir}: frame directory not found")

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdDemo3dGroundPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    first_width, first_height = _png_dimensions(frame_paths[0])
    last_width, last_height = _png_dimensions(frame_paths[-1])
    if (first_width, first_height) != (width, height):
        raise AvbdDemo3dGroundPacketError(
            "first frame dimensions do not match screenshot"
        )
    if (last_width, last_height) != (width, height):
        raise AvbdDemo3dGroundPacketError(
            "last frame dimensions do not match screenshot"
        )

    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdDemo3dGroundPacketError("capture manifest missing ui_ready")

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
        raise AvbdDemo3dGroundPacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dGroundPacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdDemo3dGroundPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdDemo3dGroundPacketError("benchmark JSON missing benchmarks list")

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _canonical_name(_row_name(row)) == BENCHMARK_NAME
    ]
    if not matching_rows:
        raise AvbdDemo3dGroundPacketError(f"benchmark JSON missing {BENCHMARK_NAME}")

    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdDemo3dGroundPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "rigid_bodies", 2.0)
            _require_counter(row, "rigid_body_joints", 0.0)
            _require_counter(row, "collision_shapes", 2.0)
            _require_counter(row, "source_scene_index", 1.0)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdDemo3dGroundPacketError(
            f"benchmark JSON missing representative rows for {BENCHMARK_NAME}"
        )

    median_rows = [
        row for row in representative_rows if row.get("aggregate_name") == "median"
    ]
    timing_row = median_rows[0] if median_rows else iteration_rows[0]

    dart_cpu_time_ns = _finite_counter(timing_row, "cpu_time")
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
        "cpu_time_per_step_ns": dart_cpu_time_ns,
        "invariants": {
            "rigid_bodies": 2,
            "rigid_body_joints": 0,
            "collision_shapes": 2,
            "source_scene_index": 1,
            "time_step": 1.0 / 60.0,
        },
    }


def _finite_field(data: dict[str, Any], key: str) -> float:
    value = data.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdDemo3dGroundPacketError(f"reference timing missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdDemo3dGroundPacketError(f"reference timing has non-finite {key}")
    return value


def _int_field(data: dict[str, Any], key: str) -> int:
    value = data.get(key)
    if not isinstance(value, int) or isinstance(value, bool):
        raise AvbdDemo3dGroundPacketError(f"reference timing missing integer {key}")
    return value


def _validate_reference_timing(reference_json: Path) -> dict[str, Any]:
    data = _load_json(reference_json)
    expected = {
        "schema_version": 1,
        "source_demo": "avbd-demo3d",
        "source_revision": "7701bd427d55",
        "scene_index": 1,
        "scene_name": "Ground",
        "scene_builder": "sceneGround",
    }
    for key, value in expected.items():
        if data.get(key) != value:
            raise AvbdDemo3dGroundPacketError(
                f"reference timing expected {key}={value!r}, got {data.get(key)!r}"
            )
    if _int_field(data, "rigid_bodies") != 2:
        raise AvbdDemo3dGroundPacketError("reference timing expected 2 rigid bodies")
    if _int_field(data, "joints") != 0:
        raise AvbdDemo3dGroundPacketError("reference timing expected 0 joints")
    if _int_field(data, "collision_shapes") != 2:
        raise AvbdDemo3dGroundPacketError(
            "reference timing expected 2 collision shapes"
        )
    if _int_field(data, "steps") < 1:
        raise AvbdDemo3dGroundPacketError("reference timing steps must be positive")
    if _int_field(data, "warmup_steps") < 0:
        raise AvbdDemo3dGroundPacketError(
            "reference timing warmup_steps must be non-negative"
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
            "joints": data["joints"],
            "collision_shapes": data["collision_shapes"],
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
            "DART beats the native avbd-demo3d Ground row"
            if ratio < 1.0
            else "DART does not yet beat the native avbd-demo3d Ground row"
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
        "remaining non-empty avbd-demo2d and avbd-demo3d source-scene ports",
        "GPU AVBD row parity and same-hardware benchmark packets",
        "paper/site/video scene visual and performance packets",
    ]
    if not comparison["dart_faster_than_reference"]:
        remaining_gates.insert(
            0,
            "DART CPU performance for this Ground row must beat the native reference before a CPU win is claimed",
        )

    return {
        "schema_version": 1,
        "packet": "avbd_demo3d_ground_source_demo",
        "scene": SCENE_ID,
        "source_demo_reference_row": SOURCE_REFERENCE_ROW,
        "visual_capture": _validate_capture(capture_manifest),
        "dart_benchmark": dart_benchmark,
        "reference_timing": reference_timing,
        "comparison": comparison,
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py --scene avbd_demo3d_ground "
                "--frames 24 --width 640 --height 360 "
                "--output-dir <capture-dir>"
            ),
            "dart_benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdDemo3dGroundStep$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
            "reference_timing_command": (
                "python scripts/run_avbd_demo3d_reference_timing.py "
                "--source-dir <avbd-demo3d-source-dir> --scene ground "
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
    except AvbdDemo3dGroundPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD demo3d Ground packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
