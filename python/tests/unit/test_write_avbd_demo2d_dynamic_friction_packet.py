from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo2d_dynamic_friction_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo2d_dynamic_friction_packet",
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
    scene: str = "avbd_demo2d_dynamic_friction",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo2d_dynamic_friction.png"
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
    rigid_bodies: float = 12.0,
    cpu_time: float = 90.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "collision_shapes": 12.0,
                "cpu_time": cpu_time,
                "iterations": 10,
                "name": "BM_AvbdDemo2dDynamicFrictionStep",
                "real_time": 100.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 0.0,
                "run_name": "BM_AvbdDemo2dDynamicFrictionStep",
                "run_type": "iteration",
                "source_scene_index": 2.0,
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
    scene_index: int = 2,
    cpu_time: float = 30.0,
    box_count: int = 11,
    max_friction: float = 5.0,
) -> Path:
    timing = {
        "box_count": box_count,
        "collision_shapes": 12,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "dynamic_bodies": 11,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "initial_speed": 10.0,
        "joints": 0,
        "max_friction": max_friction,
        "min_friction": 0.0,
        "repository": "https://github.com/savant117/avbd-demo2d",
        "rigid_bodies": 12,
        "scene_builder": "sceneDynamicFriction",
        "scene_index": scene_index,
        "scene_name": "Dynamic Friction",
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "static_bodies": 1,
        "steps": 1000,
        "warmup_steps": 128,
    }
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo2d_dynamic_friction_packet_records_evidence(
    tmp_path: Path,
) -> None:
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
    assert packet["scene"] == "avbd_demo2d_dynamic_friction"
    assert packet["source_demo_reference_row"]["scene_name"] == "Dynamic Friction"
    assert packet["source_demo_reference_row"]["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 0.5,
        "size": [100.0, 1.0],
    }
    assert packet["source_demo_reference_row"]["source_shapes"]["boxes"] == {
        "count": 11,
        "density": 1.0,
        "first_position": [-30.0, 0.75, 0.0],
        "friction_range": [5.0, 0.0],
        "initial_velocity": [10.0, 0.0, 0.0],
        "size": [1.0, 0.5],
        "x_spacing": 2.0,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert (
        packet["dart_benchmark"]["benchmark"]
        == "BM_AvbdDemo2dDynamicFrictionStep"
    )
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 12,
        "rigid_bodies": 12,
        "rigid_body_joints": 0,
        "source_scene_index": 2,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneDynamicFriction"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 11
    assert packet["reference_timing"]["invariants"]["static_bodies"] == 1
    assert packet["reference_timing"]["invariants"]["box_count"] == 11
    assert packet["reference_timing"]["invariants"]["initial_speed"] == 10.0
    assert packet["reference_timing"]["invariants"]["max_friction"] == 5.0
    assert packet["comparison"]["dart_cpu_time_per_step_ns"] == 90.0
    assert packet["comparison"]["reference_cpu_time_per_step_ns"] == 30.0
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert packet["comparison"]["dart_faster_than_reference"] is False


def test_avbd_demo2d_dynamic_friction_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo2d_motor")
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(
        SystemExit,
        match="capture scene must be avbd_demo2d_dynamic_friction",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )


def test_avbd_demo2d_dynamic_friction_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, rigid_bodies=13.0)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="expected rigid_bodies=12"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )


def test_avbd_demo2d_dynamic_friction_packet_rejects_wrong_reference(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path, box_count=10)

    with pytest.raises(SystemExit, match="expected box_count=11"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--reference-timing-json",
                str(reference_json),
            ]
        )


def test_avbd_demo2d_dynamic_friction_packet_uses_benchmark_median(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, cpu_time=90.0)
    benchmark = json.loads(benchmark_json.read_text())
    median_row = dict(benchmark["benchmarks"][0])
    median_row.update(
        {
            "aggregate_name": "median",
            "cpu_time": 80.0,
            "name": "BM_AvbdDemo2dDynamicFrictionStep_median",
            "run_type": "aggregate",
        }
    )
    stddev_row = dict(median_row)
    stddev_row.update(
        {
            "aggregate_name": "stddev",
            "collision_shapes": 0.0,
            "cpu_time": 5.0,
            "name": "BM_AvbdDemo2dDynamicFrictionStep_stddev",
            "rigid_bodies": 0.0,
            "rigid_body_joints": 0.0,
            "run_type": "aggregate",
            "source_scene_index": 0.0,
        }
    )
    benchmark["benchmarks"].extend([median_row, stddev_row])
    benchmark_json.write_text(json.dumps(benchmark), encoding="utf-8")
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
    assert packet["comparison"]["dart_cpu_time_per_step_ns"] == 80.0
