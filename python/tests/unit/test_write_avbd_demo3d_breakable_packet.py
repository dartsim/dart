from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo3d_breakable_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo3d_breakable_packet",
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


def _write_capture_manifest(tmp_path: Path, *, scene: str = "avbd_demo3d_breakable"):
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo3d_breakable.png"
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
    rigid_bodies: float = 19.0,
    cpu_time: float = 120.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "breakable_joints": 10.0,
                "collision_shapes": 19.0,
                "cpu_time": cpu_time,
                "iterations": 10,
                "name": "BM_AvbdDemo3dBreakableStep",
                "real_time": 130.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 10.0,
                "run_name": "BM_AvbdDemo3dBreakableStep",
                "run_type": "iteration",
                "source_scene_index": 13.0,
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
    scene_index: int = 13,
    cpu_time: float = 30.0,
) -> Path:
    timing = {
        "breakable_joints": 10,
        "broken_joints_after_timing": 0,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "elapsed_ns": cpu_time * 1000.0,
        "final_time": 18.8,
        "joints": 10,
        "repository": "https://github.com/savant117/avbd-demo3d",
        "rigid_bodies": 19,
        "scene_builder": "sceneBreakable",
        "scene_index": scene_index,
        "scene_name": "Breakable",
        "schema_version": 1,
        "source_demo": "avbd-demo3d",
        "source_revision": "7701bd427d55",
        "steps": 1000,
        "warmup_steps": 128,
    }
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo3d_breakable_packet_records_visual_benchmark_and_reference(
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
    assert packet["scene"] == "avbd_demo3d_breakable"
    assert packet["source_demo_reference_row"]["scene_name"] == "Breakable"
    assert packet["visual_capture"]["screenshot"]["width"] == 4
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo3dBreakableStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "break_force": 90.0,
        "breakable_joints": 10,
        "collision_shapes": 19,
        "falling_blocks": 5,
        "rigid_bodies": 19,
        "rigid_body_joints": 10,
        "source_scene_index": 13,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneBreakable"
    assert packet["reference_timing"]["invariants"]["breakable_joints"] == 10
    assert packet["comparison"]["dart_cpu_time_per_step_ns"] == 120.0
    assert packet["comparison"]["reference_cpu_time_per_step_ns"] == 30.0
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 4.0
    assert packet["comparison"]["dart_faster_than_reference"] is False


def test_avbd_demo3d_breakable_packet_rejects_wrong_scene(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo2d_motor")
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="capture scene must be avbd_demo3d_breakable"):
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


def test_avbd_demo3d_breakable_packet_rejects_wrong_benchmark_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, rigid_bodies=18.0)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="expected rigid_bodies=19"):
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


def test_avbd_demo3d_breakable_packet_uses_benchmark_median(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, cpu_time=200.0)
    benchmark = json.loads(benchmark_json.read_text())
    median_row = dict(benchmark["benchmarks"][0])
    median_row.update(
        {
            "aggregate_name": "median",
            "cpu_time": 110.0,
            "name": "BM_AvbdDemo3dBreakableStep_median",
            "run_type": "aggregate",
        }
    )
    stddev_row = dict(median_row)
    stddev_row.update(
        {
            "aggregate_name": "stddev",
            "breakable_joints": 0.0,
            "collision_shapes": 0.0,
            "cpu_time": 1.0,
            "name": "BM_AvbdDemo3dBreakableStep_stddev",
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
    assert packet["comparison"]["dart_cpu_time_per_step_ns"] == 110.0


def test_avbd_demo3d_breakable_packet_rejects_wrong_reference_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path, scene_index=12)

    with pytest.raises(SystemExit, match="reference timing expected scene_index=13"):
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
