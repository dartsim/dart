from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo2d_joint_grid_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo2d_joint_grid_packet",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _png_chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)
    )


def _write_png(path: Path, width: int = 4, height: int = 3) -> None:
    rows = [b"\x00" + (b"\x10\x20\x30" * width) for _ in range(height)]
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + _png_chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + _png_chunk(b"IDAT", zlib.compress(b"".join(rows), 6))
        + _png_chunk(b"IEND", b"")
    )


def _write_capture_manifest(
    tmp_path: Path,
    *,
    scene: str = "avbd_demo2d_joint_grid",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo2d_joint_grid.png"
    _write_png(screenshot)
    _write_png(frames / "frame_000001.png")
    _write_png(frames / "frame_000002.png")
    manifest = {
        "artifacts": {
            "events": None,
            "frames": str(frames),
            "screenshot": str(screenshot),
        },
        "force_drag": None,
        "scene": scene,
        "schema_version": 1,
        "show_ui": False,
        "switch_frame": None,
        "switch_scene": None,
        "ui_ready": {
            "dropped_warmup_frames": 0,
            "required": False,
        },
    }
    path = capture / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


def _write_benchmark_json(
    tmp_path: Path,
    *,
    rigid_bodies: float = 625.0,
    fixed_joints: float = 1200.0,
    ignored_collision_pairs: float = 1152.0,
    source_scene_index: float = 15.0,
    cpu_time: float = 240.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "collision_shapes": 625.0,
                "cpu_time": cpu_time,
                "fixed_joints": fixed_joints,
                "ignored_collision_pairs": ignored_collision_pairs,
                "iterations": 10,
                "name": "BM_AvbdDemo2dJointGridStep",
                "real_time": 250.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 1200.0,
                "run_name": "BM_AvbdDemo2dJointGridStep",
                "run_type": "iteration",
                "source_scene_index": source_scene_index,
                "time_unit": "ns",
            }
        ],
        "context": {
            "executable": "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint",
            "json_schema_version": 1,
            "library_build_type": "release",
            "library_version": "v1.9.5",
            "mhz_per_cpu": 3200,
            "num_cpus": 8,
        },
    }
    path = tmp_path / "benchmark.json"
    path.write_text(json.dumps(benchmark), encoding="utf-8")
    return path


def _write_reference_timing_json(
    tmp_path: Path,
    *,
    scene_index: int = 15,
    cpu_time: float = 80.0,
    fixed_joints: int = 1200,
    diagonal_ignore_collision_pairs: int = 1152,
) -> Path:
    timing = {
        "collision_shapes": 625,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "diagonal_ignore_collision_pairs": diagonal_ignore_collision_pairs,
        "dynamic_bodies": 623,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "fixed_joints": fixed_joints,
        "joint_grid_cells": 625,
        "joint_grid_height": 25,
        "joint_grid_width": 25,
        "joints": 1200,
        "max_friction": 0.5,
        "min_friction": 0.5,
        "repository": "https://github.com/savant117/avbd-demo2d",
        "rigid_bodies": 625,
        "scene_builder": "sceneJointGrid",
        "scene_index": scene_index,
        "scene_name": "Joint Grid",
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "static_bodies": 2,
        "steps": 1000,
        "warmup_steps": 128,
    }
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo2d_joint_grid_packet_records_evidence(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["scene"] == "avbd_demo2d_joint_grid"
    assert packet["source_demo_reference_row"]["scene_name"] == "Joint Grid"
    assert packet["source_demo_reference_row"]["source_shapes"]["grid_cell"] == {
        "count": 625,
        "dynamic_density": 1.0,
        "first_position": [0.0, 0.0, 0.0],
        "friction": 0.5,
        "height": 25,
        "size": [1.0, 1.0],
        "static_corner_density": 0.0,
        "static_corner_positions": [[0.0, 24.0, 0.0], [24.0, 24.0, 0.0]],
        "width": 25,
    }
    assert packet["source_demo_reference_row"]["source_constraints"][
        "all_axis_fixed_joints"
    ] == {
        "angular_stiffness": "infinity",
        "count": 1200,
        "fracture": "infinity",
        "horizontal_child_anchor": [-0.5, 0.0],
        "horizontal_count": 600,
        "horizontal_parent_anchor": [0.5, 0.0],
        "linear_stiffness": "infinity",
        "vertical_child_anchor": [0.0, -0.5],
        "vertical_count": 600,
        "vertical_parent_anchor": [0.0, 0.5],
    }
    assert packet["source_demo_reference_row"]["source_collision_filters"] == {
        "diagonal_ignore_collision_pairs": 1152,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo2dJointGridStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 625,
        "fixed_joints": 1200,
        "ignored_collision_pairs": 1152,
        "rigid_bodies": 625,
        "rigid_body_joints": 1200,
        "source_scene_index": 15,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneJointGrid"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 623
    assert packet["reference_timing"]["invariants"]["static_bodies"] == 2
    assert packet["reference_timing"]["invariants"]["fixed_joints"] == 1200
    assert packet["reference_timing"]["invariants"]["joint_grid_cells"] == 625
    assert (
        packet["reference_timing"]["invariants"]["diagonal_ignore_collision_pairs"]
        == 1152
    )
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert packet["comparison"]["dart_faster_than_reference"] is False
    assert not any(
        "per-pair collision-ignore" in gate for gate in packet["remaining_gates"]
    )


def test_avbd_demo2d_joint_grid_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, fixed_joints=1199.0)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="fixed_joints=1200"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
                "--output",
                str(tmp_path / "packet.json"),
            ]
        )

    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(
        tmp_path, diagonal_ignore_collision_pairs=1151
    )
    with pytest.raises(SystemExit, match="diagonal_ignore_collision_pairs=1152"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
                "--output",
                str(tmp_path / "packet.json"),
            ]
        )
