#!/usr/bin/env python3
"""Write validated AVBD spherical breakable-joint packets."""

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

PACKET_DIR = Path("docs/plans/104-vertex-block-descent-solver")
BENCHMARK_ARG = 1
BENCHMARK_TIME_STEP = 0.005
BENCHMARK_GRAVITY_M_PER_S2 = (0.0, -9.81, 0.0)
BENCHMARK_BREAK_FORCE_N = 1.0e12


@dataclass(frozen=True)
class Variant:
    packet: str
    scene: str
    output: Path
    benchmark: str
    benchmark_anchor_scope: str
    row_family: str
    scope: str
    visual_invariants: dict[str, Any]


VARIANTS: dict[str, Variant] = {
    "rigid": Variant(
        packet="avbd_rigid_spherical_breakable_joint",
        scene="avbd_rigid_spherical_breakable_joint",
        output=PACKET_DIR / "avbd-rigid-spherical-breakable-joint-packet.json",
        benchmark="BM_AvbdRigidSphericalBreakableJointStep",
        benchmark_anchor_scope="rigid_body_chain",
        row_family="public free-rigid-body spherical point-joint break/reset",
        scope=(
            "single public rigid spherical point-joint break/reset visual plus "
            "break-force-armed benchmark"
        ),
        visual_invariants={
            "rigid_bodies": 3,
            "breakable_joints": 1,
            "joint_anchor_scope": "rigid_body_pair",
            "captured_offset_m": [0.62, 0.0, 0.0],
            "payload_prestain_m": [0.22, 0.0, -0.08],
            "initial_yaw_rad": 0.42,
            "initial_linear_velocity_m_per_s": [0.45, 0.0, -0.25],
            "initial_angular_velocity_rad_per_s": [0.0, 0.0, 1.1],
            "break_force_n": 1.0e-12,
            "reset_break_force_n": 1.0e12,
            "gravity_m_per_s2": [0.0, 0.0, -9.81],
            "time_step": 0.004,
        },
    ),
    "articulated-world": Variant(
        packet="avbd_articulated_spherical_breakable_joint",
        scene="avbd_articulated_spherical_breakable_joint",
        output=PACKET_DIR / "avbd-articulated-spherical-breakable-joint-packet.json",
        benchmark="BM_AvbdArticulatedWorldSphericalBreakableJointStep",
        benchmark_anchor_scope="world_link",
        row_family="public world-link articulated spherical point-joint break/reset",
        scope=(
            "single public world-link articulated spherical point-joint "
            "break/reset visual plus break-force-armed benchmark"
        ),
        visual_invariants={
            "multibodies": 1,
            "links": 2,
            "breakable_joints": 1,
            "joint_anchor_scope": "world_link",
            "captured_position_m": [0.68, 0.0, 0.0],
            "captured_yaw_rad": 0.22,
            "child_anchor_m": [-0.34, 0.0, 0.0],
            "pull_force_n": [0.0, 5.0, 0.0],
            "force_point_m": [0.21, 0.0, 0.0],
            "break_force_n": 1.0e-18,
            "reset_break_force_n": 1.0e6,
            "time_step": 0.005,
        },
    ),
    "articulated-pair": Variant(
        packet="avbd_articulated_spherical_pair_breakable_joint",
        scene="avbd_articulated_spherical_pair_breakable_joint",
        output=PACKET_DIR
        / "avbd-articulated-spherical-pair-breakable-joint-packet.json",
        benchmark="BM_AvbdArticulatedSphericalPairBreakableJointStep",
        benchmark_anchor_scope="same_multibody_pair",
        row_family=(
            "public same-multibody articulated spherical point-joint break/reset"
        ),
        scope=(
            "single public same-multibody articulated spherical point-joint "
            "break/reset visual plus break-force-armed benchmark"
        ),
        visual_invariants={
            "multibodies": 1,
            "links": 2,
            "breakable_joints": 1,
            "joint_anchor_scope": "same_multibody_pair",
            "captured_position_m": [0.68, 0.0, 0.0],
            "captured_yaw_rad": 0.22,
            "child_anchor_m": [-0.34, 0.0, 0.0],
            "pull_force_n": [0.0, 5.0, 0.0],
            "force_point_m": [0.21, 0.0, 0.0],
            "break_force_n": 1.0e-18,
            "reset_break_force_n": 1.0e12,
            "time_step": 0.005,
        },
    ),
}


class AvbdSphericalBreakableJointPacketError(RuntimeError):
    pass


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=sorted(VARIANTS), required=True)
    parser.add_argument("--capture-manifest", type=Path, required=True)
    parser.add_argument("--benchmark-json", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args(argv)


def _validate_capture(manifest_path: Path, variant: Variant) -> dict[str, Any]:
    manifest = _load_json(manifest_path)
    if manifest.get("schema_version") != 1:
        raise AvbdSphericalBreakableJointPacketError(
            "capture manifest schema_version must be 1"
        )
    if manifest.get("scene") != variant.scene:
        raise AvbdSphericalBreakableJointPacketError(
            f"capture scene must be {variant.scene}"
        )
    if manifest.get("switch_scene") is not None:
        raise AvbdSphericalBreakableJointPacketError(
            "spherical breakable joint capture must not switch scenes"
        )
    if manifest.get("force_drag") is not None:
        raise AvbdSphericalBreakableJointPacketError(
            "spherical breakable joint capture must not force-drag"
        )

    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise AvbdSphericalBreakableJointPacketError(
            "capture manifest missing artifacts"
        )
    manifest_dir = manifest_path.parent
    screenshot = _artifact_path(manifest_dir, artifacts.get("screenshot"), "screenshot")
    frames_dir = _artifact_path(manifest_dir, artifacts.get("frames"), "frames")
    if not screenshot.is_file():
        raise AvbdSphericalBreakableJointPacketError(
            f"{screenshot}: screenshot not found"
        )
    if not frames_dir.is_dir():
        raise AvbdSphericalBreakableJointPacketError(
            f"{frames_dir}: frame directory not found"
        )

    frame_paths = sorted(frames_dir.glob("frame_*.png"))
    if not frame_paths:
        raise AvbdSphericalBreakableJointPacketError(
            f"{frames_dir}: no PNG frames found"
        )

    width, height = _png_dimensions(screenshot)
    if _png_dimensions(frame_paths[0]) != (width, height):
        raise AvbdSphericalBreakableJointPacketError(
            "first frame dimensions do not match screenshot"
        )
    if _png_dimensions(frame_paths[-1]) != (width, height):
        raise AvbdSphericalBreakableJointPacketError(
            "last frame dimensions do not match screenshot"
        )
    ui_ready = manifest.get("ui_ready")
    if not isinstance(ui_ready, dict):
        raise AvbdSphericalBreakableJointPacketError(
            "capture manifest missing ui_ready"
        )

    return {
        "manifest_sha256": _sha256(manifest_path),
        "scene": variant.scene,
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


def _matches_benchmark_arg(row: dict[str, Any], variant: Variant) -> bool:
    return _benchmark_identity(row) == f"{variant.benchmark}/{BENCHMARK_ARG}"


def _finite_counter(row: dict[str, Any], key: str, variant: Variant) -> float:
    value = row.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise AvbdSphericalBreakableJointPacketError(
            f"{variant.benchmark}: missing {key}"
        )
    value = float(value)
    if not math.isfinite(value):
        raise AvbdSphericalBreakableJointPacketError(
            f"{variant.benchmark}: non-finite {key}"
        )
    return value


def _require_counter(
    row: dict[str, Any], key: str, expected: float, variant: Variant
) -> None:
    value = _finite_counter(row, key, variant)
    if value != expected:
        raise AvbdSphericalBreakableJointPacketError(
            f"{variant.benchmark}: expected {key}={expected:g}, got {value:g}"
        )


def _validate_benchmark(benchmark_json: Path, variant: Variant) -> dict[str, Any]:
    data = _load_json(benchmark_json)
    rows = data.get("benchmarks")
    if not isinstance(rows, list):
        raise AvbdSphericalBreakableJointPacketError(
            "benchmark JSON missing benchmarks list"
        )

    matching_rows = [
        row
        for row in rows
        if isinstance(row, dict) and _matches_benchmark_arg(row, variant)
    ]
    if not matching_rows:
        raise AvbdSphericalBreakableJointPacketError(
            f"benchmark JSON missing {variant.benchmark}/{BENCHMARK_ARG}"
        )
    iteration_rows = [
        row for row in matching_rows if row.get("run_type", "iteration") == "iteration"
    ]
    if not iteration_rows:
        raise AvbdSphericalBreakableJointPacketError(
            f"benchmark JSON missing iteration rows for {variant.benchmark}"
        )

    representative_rows = []
    packet_rows = []
    for row in matching_rows:
        run_type = row.get("run_type", "iteration")
        aggregate_name = row.get("aggregate_name")
        if run_type == "iteration" or aggregate_name in ("mean", "median"):
            representative_rows.append(row)
            _require_counter(row, "breakable_joints", float(BENCHMARK_ARG), variant)
        for key in ("real_time", "cpu_time"):
            _finite_counter(row, key, variant)
        packet_rows.append(row)

    if not representative_rows:
        raise AvbdSphericalBreakableJointPacketError(
            f"benchmark JSON missing representative rows for {variant.benchmark}"
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
        "benchmark": f"{variant.benchmark}/{BENCHMARK_ARG}",
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
        "cpu_time_per_step_ns": _finite_counter(timing_row, "cpu_time", variant),
        "invariants": {
            "breakable_joints": BENCHMARK_ARG,
            "joint_anchor_scope": variant.benchmark_anchor_scope,
            "time_step": BENCHMARK_TIME_STEP,
            "gravity_m_per_s2": list(BENCHMARK_GRAVITY_M_PER_S2),
            "break_force_n": BENCHMARK_BREAK_FORCE_N,
        },
    }


def make_packet(
    variant: Variant, capture_manifest: Path, benchmark_json: Path
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "packet": variant.packet,
        "scene": variant.scene,
        "target": {
            "broad_breakable_constraint_complete": False,
            "row_family": variant.row_family,
            "scope": variant.scope,
        },
        "visual_scene_invariants": variant.visual_invariants,
        "visual_capture": _validate_capture(capture_manifest, variant),
        "benchmark": _validate_benchmark(benchmark_json, variant),
        "reproduction": {
            "capture_command": (
                "LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe "
                "python scripts/capture_py_demo.py "
                f"--scene {variant.scene} "
                "--frames 8 --width 640 --height 360 --output-dir <capture-dir>"
            ),
            "benchmark_command": (
                "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint "
                f"--benchmark_filter={variant.benchmark}/1$ "
                "--benchmark_min_time=0.5s --benchmark_repetitions=3 "
                "--benchmark_out=<benchmark-json> --benchmark_out_format=json"
            ),
        },
        "remaining_gates": [
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
    variant = VARIANTS[args.variant]
    output = args.output if args.output is not None else variant.output
    try:
        packet = make_packet(variant, args.capture_manifest, args.benchmark_json)
    except AvbdSphericalBreakableJointPacketError as exc:
        raise SystemExit(str(exc)) from exc
    write_packet(output, packet)
    print(f"Wrote AVBD spherical breakable joint packet: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
