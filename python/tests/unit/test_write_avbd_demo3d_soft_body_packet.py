from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo3d_soft_body_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo3d_soft_body_packet",
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
    scene: str = "avbd_demo3d_soft_body",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo3d_soft_body.png"
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
    fixed_joints: float = 432.0,
    finite_stiffness_fixed_joints: float = 432.0,
    ignored_collision_pairs: float = 648.0,
    source_scene_index: float = 11.0,
    cpu_time: float = 240.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "collision_shapes": 193.0,
                "cpu_time": cpu_time,
                "finite_stiffness_fixed_joints": finite_stiffness_fixed_joints,
                "fixed_joints": fixed_joints,
                "ignored_collision_pairs": ignored_collision_pairs,
                "iterations": 10,
                "name": "BM_AvbdDemo3dSoftBodyStep",
                "real_time": 250.0,
                "rigid_bodies": 193.0,
                "rigid_body_joints": 432.0,
                "run_name": "BM_AvbdDemo3dSoftBodyStep",
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
    scene_index: int = 11,
    cpu_time: float = 80.0,
    finite_stiffness_fixed_joints: int = 432,
    diagonal_ignore_collision_pairs: int = 648,
) -> Path:
    timing = {
        "collision_shapes": 193,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "diagonal_ignore_collision_pairs": diagonal_ignore_collision_pairs,
        "dynamic_bodies": 192,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "finite_stiffness_fixed_joints": finite_stiffness_fixed_joints,
        "fixed_joints": 432,
        "joints": 432,
        "max_angular_stiffness": 250.0,
        "max_friction": 0.5,
        "max_linear_stiffness": 1000.0,
        "min_angular_stiffness": 250.0,
        "min_friction": 0.5,
        "min_linear_stiffness": 1000.0,
        "repository": "https://github.com/savant117/avbd-demo3d",
        "rigid_bodies": 193,
        "scene_builder": "sceneSoftBody",
        "scene_index": scene_index,
        "scene_name": "Soft Body",
        "schema_version": 1,
        "soft_body_cells": 192,
        "soft_body_depth": 4,
        "soft_body_height": 4,
        "soft_body_stacks": 3,
        "soft_body_width": 4,
        "source_demo": "avbd-demo3d",
        "source_revision": "7701bd427d55",
        "static_bodies": 1,
        "steps": 1000,
        "warmup_steps": 128,
    }
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo3d_soft_body_packet_records_evidence(tmp_path: Path) -> None:
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
    assert packet["scene"] == "avbd_demo3d_soft_body"
    assert packet["source_demo_reference_row"]["scene_name"] == "Soft Body"
    assert packet["source_demo_reference_row"]["source_shapes"]["soft_cell"] == {
        "base_z": 8.0,
        "count": 192,
        "density": 1.0,
        "depth": 4,
        "first_position": [-1.2, -1.2, 8.0],
        "friction": 0.5,
        "height": 4,
        "size": [0.8, 0.8, 0.8],
        "stack_gap": 2.0,
        "stacks": 3,
        "width": 4,
    }
    assert packet["source_demo_reference_row"]["source_constraints"][
        "finite_all_axis_fixed_joints"
    ] == {
        "angular_stiffness": 250.0,
        "count": 432,
        "fracture": "infinity",
        "linear_stiffness": 1000.0,
        "x_child_anchor": [-0.4, 0.0, 0.0],
        "x_count": 144,
        "x_parent_anchor": [0.4, 0.0, 0.0],
        "y_child_anchor": [0.0, -0.4, 0.0],
        "y_count": 144,
        "y_parent_anchor": [0.0, 0.4, 0.0],
        "z_child_anchor": [0.0, 0.0, -0.4],
        "z_count": 144,
        "z_parent_anchor": [0.0, 0.0, 0.4],
    }
    assert packet["source_demo_reference_row"]["source_collision_filters"] == {
        "diagonal_ignore_collision_pairs": 648,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo3dSoftBodyStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 193,
        "finite_stiffness_fixed_joints": 432,
        "fixed_joints": 432,
        "ignored_collision_pairs": 648,
        "rigid_bodies": 193,
        "rigid_body_joints": 432,
        "source_scene_index": 11,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneSoftBody"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 192
    assert packet["reference_timing"]["invariants"]["static_bodies"] == 1
    assert packet["reference_timing"]["invariants"]["fixed_joints"] == 432
    assert (
        packet["reference_timing"]["invariants"]["finite_stiffness_fixed_joints"]
        == 432
    )
    assert packet["reference_timing"]["invariants"]["soft_body_cells"] == 192
    assert packet["reference_timing"]["invariants"]["soft_body_depth"] == 4
    assert (
        packet["reference_timing"]["invariants"]["diagonal_ignore_collision_pairs"]
        == 648
    )
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert packet["comparison"]["dart_faster_than_reference"] is False


def test_avbd_demo3d_soft_body_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, fixed_joints=431.0)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="fixed_joints=432"):
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
        tmp_path, diagonal_ignore_collision_pairs=647
    )
    with pytest.raises(SystemExit, match="diagonal_ignore_collision_pairs=648"):
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
