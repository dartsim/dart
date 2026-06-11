#!/usr/bin/env python3
"""Write a validated AVBD rigid breakable fixed-joint packet."""

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
    "avbd-rigid-breakable-joint-packet.json"
)
SCENE_ID = "avbd_rigid_breakable_joint"
BENCHMARK_NAME = "BM_AvbdRigidBreakableJointStep"
BENCHMARK_ARG = 1
DEMO_TIME_STEP = 0.004
BENCHMARK_TIME_STEP = 0.005
DEMO_GRAVITY_M_PER_S2 = (0.0, 0.0, -9.81)
BENCHMARK_GRAVITY_M_PER_S2 = (0.0, -9.81, 0.0)
DEMO_BREAK_FORCE_N = 1.0e-12
DEMO_RESET_BREAK_FORCE_N = 1.0e12
BENCHMARK_BREAK_FORCE_N = 1.0e12
DEMO_CAPTURED_OFFSET_M = (0.62, 0.0, 0.0)
DEMO_PAYLOAD_PRESTRAIN_M = (0.18, 0.0, -0.08)
DEMO_INITIAL_LINEAR_VELOCITY_M_PER_S = (0.65, 0.0, -0.35)


class AvbdRigidBreakableJointPacketError(RuntimeError):
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
        raise AvbdRigidBreakableJointPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != SCENE_ID:
        raise AvbdRigidBreakableJointPacketError(f"capture scene must be {SCENE_ID}")
    if manifest.get("switch_scene") is not None:
        raise AvbdRigidBreakableJointPacketError(
            "rigid breakable joint capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdRigidBreakableJointPacketError(
            "rigid breakable joint capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdRigidBreakableJointPacketError("capture manifest missing artifacts")
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdRigidBreakableJointPacketError(f"{screenshot}: screenshot not found")
    if not frames_dir.is_dir():
        raise AvbdRigidBreakableJointPacketError(
            f"{frames_dir}: frame directory not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdRigidBreakableJointPacketError(f"{frames_dir}: no PNG frames found")

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdRigidBreakableJointPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdRigidBreakableJointPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdRigidBreakableJointPacketError("capture manifest missing ui_ready")

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
        raise AvbdRigidBreakableJointPacketError(f"{BENCHMARK_NAME}: missing {key}")
    value = float(value)
    if not math.isfinite(value):
        raise AvbdRigidBreakableJointPacketError(f"{BENCHMARK_NAME}: non-finite {key}")
    return value


def _require_counter(row: dict[str, Any], key: str, expected: float) -> None:
    value = _finite_counter(row, key)
    if value != expected:
        raise AvbdRigidBreakableJointPacketError(
            f"{BENCHMARK_NAME}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdRigidBreakableJointPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row for row in rows if isinstance(row, dict) and _matches_benchmark_arg(row)
    ]
    if not matching_rows:
        raise AvbdRigidBreakableJointPacketError(
            f"benchmark JSON missing {BENCHMARK_NAME}/{BENCHMARK_ARG}"
        )
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdRigidBreakableJointPacketError(
            f"benchmark JSON missing iteration rows for {BENCHMARK_NAME}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "breakable_joints", float(BENCHMARK_ARG))
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdRigidBreakableJointPacketError(
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
            "breakable_joints": BENCHMARK_ARG,
            "joint_anchor_scope": "rigid_body_chain",
            "time_step": BENCHMARK_TIME_STEP,
            "gravity_m_per_s2": list(BENCHMARK_GRAVITY_M_PER_S2),
            "break_force_n": BENCHMARK_BREAK_FORCE_N,
        },
    }


def make_packet(capture_manifest: Path, benchmark_json: Path) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "packet": "avbd_rigid_breakable_joint",
        "scene": SCENE_ID,
        "target": {
            "broad_breakable_constraint_complete": False,
            "row_family": "public free-rigid-body fixed point-joint break/reset",
            "scope": (
                "single public rigid fixed point-joint break/reset visual plus "
                "break-force-armed benchmark"
            ),
        },
        "visual_scene_invariants": {
            "rigid_bodies": 3,
            "breakable_joints": 1,
            "joint_anchor_scope": "rigid_body_pair",
            "captured_offset_m": list(DEMO_CAPTURED_OFFSET_M),
            "payload_prestain_m": list(DEMO_PAYLOAD_PRESTRAIN_M),
            "initial_linear_velocity_m_per_s": list(
                DEMO_INITIAL_LINEAR_VELOCITY_M_PER_S
            ),
            "break_force_n": DEMO_BREAK_FORCE_N,
            "reset_break_force_n": DEMO_RESET_BREAK_FORCE_N,
            "gravity_m_per_s2": list(DEMO_GRAVITY_M_PER_S2),
            "time_step": DEMO_TIME_STEP,
        },
        "visual_capture": _validate_capture(capture_manifest),
        "benchmark": _validate_benchmark(benchmark_json),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py "
                "--scene avbd_rigid_breakable_joint "
                "--frames 8 --width 640 --height 360 --output-dir <capture-dir>"
            ),
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                "--benchmark_filter=BM_AvbdRigidBreakableJointStep/1$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
        },
        "remaining_gates": [
            "articulated fixed break/reset packet",
            "spherical and same-multibody break/reset packets",
            "broad breakable-constraint corpus",
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
    except AvbdRigidBreakableJointPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(args.output, packet)
    print(f"Wrote AVBD rigid breakable joint packet: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
