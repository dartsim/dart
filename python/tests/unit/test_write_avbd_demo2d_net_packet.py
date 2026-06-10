from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_demo2d_net_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_demo2d_net_packet",
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
    scene: str = "avbd_demo2d_net",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_demo2d_net.png"
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
    rigid_bodies: float = 91.0,
    cpu_time: float = 300.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "collision_shapes": 91.0,
                "cpu_time": cpu_time,
                "iterations": 10,
                "linear_point_joints": 39.0,
                "name": "BM_AvbdDemo2dNetStep",
                "real_time": 320.0,
                "rigid_bodies": rigid_bodies,
                "rigid_body_joints": 39.0,
                "run_name": "BM_AvbdDemo2dNetStep",
                "run_type": "iteration",
                "source_scene_index": 16.0,
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
    scene_index: int = 16,
    cpu_time: float = 100.0,
    falling_blocks: int = 50,
) -> Path:
    timing = {
        "collision_shapes": 91,
        "compile_flags": "-std=c++17 -O3 -DNDEBUG",
        "compiler": "g++",
        "cpu_time_per_step_ns": cpu_time,
        "dynamic_bodies": 88,
        "elapsed_ns": cpu_time * 1000.0,
        "falling_blocks": falling_blocks,
        "final_time": 18.8,
        "ground_bodies": 1,
        "joints": 39,
        "linear_point_joints": 39,
        "max_friction": 0.5,
        "min_friction": 0.5,
        "net_links": 40,
        "repository": "https://github.com/savant117/avbd-demo2d",
        "rigid_bodies": 91,
        "scene_builder": "sceneNet",
        "scene_index": scene_index,
        "scene_name": "Net",
        "schema_version": 1,
        "source_demo": "avbd-demo2d",
        "source_revision": "74699a11f858",
        "static_bodies": 3,
        "steps": 1000,
        "warmup_steps": 128,
    }
    path = tmp_path / "reference.json"
    path.write_text(json.dumps(timing), encoding="utf-8")
    return path


def test_avbd_demo2d_net_packet_records_evidence(tmp_path: Path) -> None:
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
    assert packet["scene"] == "avbd_demo2d_net"
    assert packet["source_demo_reference_row"]["scene_name"] == "Net"
    assert packet["source_demo_reference_row"]["source_shapes"]["net_link"] == {
        "count": 40,
        "dynamic_density": 1.0,
        "first_position": [-20.0, 10.0, 0.0],
        "friction": 0.5,
        "size": [1.0, 0.5],
        "static_endpoint_density": 0.0,
        "x_spacing": 1.0,
    }
    assert packet["source_demo_reference_row"]["source_shapes"][
        "falling_block"
    ] == {
        "columns": 10,
        "count": 50,
        "density": 1.0,
        "first_position": [-5.0, 15.0, 0.0],
        "friction": 0.5,
        "rows": 5,
        "size": [1.0, 1.0],
        "x_spacing": 1.0,
        "y_spacing": 1.0,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["dart_benchmark"]["benchmark"] == "BM_AvbdDemo2dNetStep"
    assert packet["dart_benchmark"]["invariants"] == {
        "collision_shapes": 91,
        "linear_point_joints": 39,
        "rigid_bodies": 91,
        "rigid_body_joints": 39,
        "source_scene_index": 16,
        "time_step": 1.0 / 60.0,
    }
    assert packet["reference_timing"]["scene_builder"] == "sceneNet"
    assert packet["reference_timing"]["invariants"]["dynamic_bodies"] == 88
    assert packet["reference_timing"]["invariants"]["static_bodies"] == 3
    assert packet["reference_timing"]["invariants"]["linear_point_joints"] == 39
    assert packet["reference_timing"]["invariants"]["net_links"] == 40
    assert packet["reference_timing"]["invariants"]["falling_blocks"] == 50
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 3.0
    assert packet["comparison"]["dart_faster_than_reference"] is False


def test_avbd_demo2d_net_packet_rejects_wrong_scene(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_demo2d_rope")
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="capture scene must be avbd_demo2d_net"):
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


def test_avbd_demo2d_net_packet_rejects_wrong_counts(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, rigid_bodies=90.0)
    reference_json = _write_reference_timing_json(tmp_path)

    with pytest.raises(SystemExit, match="expected rigid_bodies=91"):
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


def test_avbd_demo2d_net_packet_rejects_wrong_reference(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path, scene_index=15)

    with pytest.raises(SystemExit, match="reference timing expected scene_index=16"):
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


def test_avbd_demo2d_net_packet_rejects_wrong_reference_shape_counts(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    reference_json = _write_reference_timing_json(tmp_path, falling_blocks=49)

    with pytest.raises(SystemExit, match="reference timing expected falling_blocks=50"):
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


def test_avbd_demo2d_net_packet_uses_benchmark_median(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, cpu_time=300.0)
    benchmark = json.loads(benchmark_json.read_text())
    benchmark["benchmarks"].append(
        {
            **benchmark["benchmarks"][0],
            "aggregate_name": "median",
            "cpu_time": 250.0,
            "iterations": 3,
            "name": "BM_AvbdDemo2dNetStep_median",
            "run_type": "aggregate",
        }
    )
    benchmark_json.write_text(json.dumps(benchmark), encoding="utf-8")
    reference_json = _write_reference_timing_json(tmp_path, cpu_time=100.0)
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
    assert packet["dart_benchmark"]["cpu_time_per_step_ns"] == 250.0
    assert packet["comparison"]["dart_to_reference_cpu_time_ratio"] == 2.5
