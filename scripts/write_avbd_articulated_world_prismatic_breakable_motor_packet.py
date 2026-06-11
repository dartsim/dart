#!/usr/bin/env python3
"""Write a validated AVBD world-prismatic breakable motor packet."""

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
    "docs/plans/104-vertex-block-descent-solver/"
    "avbd-articulated-world-prismatic-breakable-motor-packet.json"
)
SCENE_ID = "avbd_articulated_prismatic_motor_breakable_joint"
BENCHMARK_NAME = "BM_AvbdArticulatedWorldPrismaticBreakableMotorStep"
BENCHMARK_ARG = 1
TIME_STEP = 0.005
DEMO_TARGET_SPEED_M_PER_S = 0.35
DEMO_MAX_FORCE_N = 900.0
DEMO_BREAK_FORCE_N = 1.0e-18
DEMO_RESET_BREAK_FORCE_N = 1.0e12
DEMO_PULL_FORCE_N = (0.0, 5.0, 0.0)
BENCHMARK_TARGET_SPEED_M_PER_S = 0.35
BENCHMARK_MAX_FORCE_N = 800.0
BENCHMARK_BREAK_FORCE_N = 1.0e12


class AvbdArticulatedWorldPrismaticBreakableMotorPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _validate_capture(manifest_path: Path) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"capture scene must be {SCENE_ID}"
        )
    if manifest.get("switch_scene") is not None:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "articulated world-prismatic breakable motor capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "articulated world-prismatic breakable motor capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "capture manifest missing artifacts"
        )
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{screenshot}: screenshot not found"
        )
    if not frames_dir.is_dir():
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{frames_dir}: frame directory not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{frames_dir}: no PNG frames found"
        )

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "capture manifest missing ui_ready"
        )

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


def _benchmark_identity(row: dict[str, Any]) -> str:
    return _canonical_name(_row_name(row))


def _matches_benchmark_arg(row: dict[str, Any]) -> bool:
    return _benchmark_identity(row) == f"{BENCHMARK_NAME}/{BENCHMARK_ARG}"


def _finite_counter(row: dict[str, Any], key: str) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{BENCHMARK_NAME}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{BENCHMARK_NAME}: non-finite {key}"
        )
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row for row in rows if isinstance(row, dict) and _matches_benchmark_arg(row)
    ]
    if not matching_rows:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}/{BENCHMARK_ARG}"
        )
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "motors", float(BENCHMARK_ARG))
            _require_counter(row, "breakable_motors", float(BENCHMARK_ARG))
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdArticulatedWorldPrismaticBreakableMotorPacketError(
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
        "benchmark": f"{BENCHMARK_NAME}/{BENCHMARK_ARG}",
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
            "motors": BENCHMARK_ARG,
            "breakable_motors": BENCHMARK_ARG,
            "time_step": TIME_STEP,
            "target_speed_m_per_s": BENCHMARK_TARGET_SPEED_M_PER_S,
            "max_force_n": BENCHMARK_MAX_FORCE_N,
            "break_force_n": BENCHMARK_BREAK_FORCE_N,
        },
    }


def make_packet(capture_manifest: Path, benchmark_json: Path) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "packet": "avbd_articulated_world_prismatic_breakable_motor",
        "scene": SCENE_ID,
        "target": {
            "row_family": (
                "public world-anchored articulated prismatic velocity motor "
                "break/reset"
            ),
            "scope": (
                "single world-anchored public articulated prismatic motor "
                "break/reset visual plus break-force-armed benchmark"
            ),
            "broad_motor_lifecycle_complete": False,
        },
        "visual_scene_invariants": {
            "multibodies": 1,
            "links": 2,
            "motors": 1,
            "breakable_motors": 1,
            "joint_anchor_scope": "world_link",
            "target_speed_m_per_s": DEMO_TARGET_SPEED_M_PER_S,
            "max_force_n": DEMO_MAX_FORCE_N,
            "break_force_n": DEMO_BREAK_FORCE_N,
            "reset_break_force_n": DEMO_RESET_BREAK_FORCE_N,
            "pull_force_n": list(DEMO_PULL_FORCE_N),
            "time_step": TIME_STEP,
        },
        "visual_capture": _validate_capture(capture_manifest),
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py "
                "--scene avbd_articulated_prismatic_motor_breakable_joint "
                "--frames 8 --width 640 --height 360 --output-dir <capture-dir>"
            ),
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter="
                "BM_AvbdArticulatedWorldPrismaticBreakableMotorStep/1$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
        },
        "remaining_gates": [
            "rigid/articulated non-motor breakable joint packets",
            "broad motor lifecycle and command-update corpus",
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
    except AvbdArticulatedWorldPrismaticBreakableMotorPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(
        f"Wrote AVBD articulated world-prismatic breakable motor packet: {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
