from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo2d_static_friction_packet.py"
_RAMP_ANGLE = 3.14159 / 6.0


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo2d_static_friction_packet",
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
    scene: str = "avbd_demo2d_static_friction",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo2d_static_friction.png"
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
                "name": "BM_AvbdDemo2dStaticFrictionStep",
                "real_time": 100.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 0.0,
                "run_name": "BM_AvbdDemo2dStaticFrictionStep",
                "run_type": "iteration",
                "source_scene_index": 3.0,
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
    scene_index: int = 3,
    cpu_time: float = 30.0,
    max_friction: float = 1.0,
) -> Path:
    timing = {
        "box_count": 11,
        "collision_shapes": 12,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "dynamic_bodies": 11,
        "dynamic_max_friction": 1.0,
        "dynamic_min_friction": 1.0,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "joints": 0,
        "max_friction": max_friction,
        "min_friction": 1.0,
        "ramp_angle": _RAMP_ANGLE,
        "repository": "https://github.com/savant117/avbd-demo2d",
        "rigid_bodies": 12,
        "scene_builder": "sceneStaticFriction",
        "scene_index": scene_index,
        "scene_name": "Static Friction",
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


def test_avbd_demo2d_static_friction_packet_records_evidence(
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
    assert packet["scene"] == "avbd_demo2d_static_friction"
    assert packet["source_demo_reference_row"]["scene_name"] == "Static Friction"
    assert packet["source_demo_reference_row"]["source_shapes"]["ground"] == {
        "density": 0.0,
        "friction": 1.0,
        "position": [0.0, 0.0, _RAMP_ANGLE],
        "size": [100.0, 1.0],
    }
    assert packet["source_demo_reference_row"]["source_shapes"]["boxes"] == {
        "count": 11,
        "density": 1.0,
        "first_position": [0.0, 1.0, _RAMP_ANGLE],
        "friction": 1.0,
        "size": [5.0, 0.5],
        "y_spacing": 1.0,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo2dStaticFrictionStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 12,
        "rigid_bodies": 12,
        "rigid_body_joints": 0,
        "source_scene_index": 3,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneStaticFriction"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 11
    assert packet["reference_timing"]["invariants"]["static_bodies"] == 1
    assert packet["reference_timing"]["invariants"]["box_count"] == 11
    assert packet["reference_timing"]["invariants"]["ramp_angle"] == _RAMP_ANGLE
    assert packet["reference_timing"]["invariants"]["min_friction"] == 1.0
    assert packet["reference_timing"]["invariants"]["max_friction"] == 1.0
    assert packet["reference_timing"]["invariants"]["dynamic_min_friction"] == 1.0
    assert packet["reference_timing"]["invariants"]["dynamic_max_friction"] == 1.0
    assert packet["comparison"]["dart_cpu_time_per_step_ns"] == 90.0
    assert packet["comparison"]["reference_cpu_time_per_step_ns"] == 30.0
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert packet["comparison"]["dart_faster_than_reference"] is False


def test_avbd_demo2d_static_friction_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(
        tmp_path, scene="avbd_demo2d_dynamic_friction"
    )
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(
        SystemExit,
        match="capture scene must be avbd_demo2d_static_friction",
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


def test_avbd_demo2d_static_friction_packet_rejects_wrong_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, rigid_bodies=11.0)
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


def test_avbd_demo2d_static_friction_packet_rejects_wrong_reference(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path, max_friction=0.5)

    with pytest.raises(SystemExit, match="expected max_friction=1"):
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
