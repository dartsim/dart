from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    ROOT / "scripts" / "write_avbd_articulated_world_revolute_breakable_motor_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_articulated_world_revolute_breakable_motor_packet",
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
    scene: str = "avbd_articulated_world_revolute_motor_breakable_joint",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = (
        capture / "avbd_articulated_world_revolute_motor_breakable_joint.png"
    )
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
    motors: float = 1.0,
    breakable_motors: float = 1.0,
    benchmark_name: str = "BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1",
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "breakable_motors": breakable_motors,
                "cpu_time": 6100.0,
                "iterations": 10,
                "motors": motors,
                "name": benchmark_name,
                "real_time": 6200.0,
                "run_name": benchmark_name,
                "run_type": "iteration",
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


def test_avbd_world_revolute_breakable_motor_packet_records_evidence(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["scene"] == "avbd_articulated_world_revolute_motor_breakable_joint"
    assert packet["target"] == {
        "broad_motor_lifecycle_complete": False,
        "row_family": (
            "public world-anchored articulated revolute velocity motor break/reset"
        ),
        "scope": (
            "single world-anchored public articulated revolute motor "
            "break/reset visual plus break-force-armed benchmark"
        ),
    }
    assert packet["visual_scene_invariants"] == {
        "break_force_n": 1.0e-18,
        "breakable_motors": 1,
        "joint_anchor_scope": "world_link",
        "links": 2,
        "max_torque_nm": 900.0,
        "motors": 1,
        "multibodies": 1,
        "pull_force_n": [0.0, 5.0, 0.0],
        "reset_break_force_n": 1.0e12,
        "target_speed_rad_per_s": 0.75,
        "time_step": 0.005,
    }
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert (
        packet["benchmark"]["benchmark"]
        == "BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1"
    )
    assert packet["benchmark"]["cpu_time_per_step_ns"] == 6100.0
    assert packet["benchmark"]["invariants"] == {
        "break_force_n": 1.0e12,
        "breakable_motors": 1,
        "max_torque_nm": 800.0,
        "motors": 1,
        "target_speed_rad_per_s": 0.5,
        "time_step": 0.005,
    }
    assert "broad motor lifecycle and command-update corpus" in packet[
        "remaining_gates"
    ]


def test_avbd_world_revolute_breakable_motor_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_empty_baseline")
    benchmark_json = _write_benchmark_json(tmp_path)

    with pytest.raises(
        SystemExit,
        match=(
            "capture scene must be "
            "avbd_articulated_world_revolute_motor_breakable_joint"
        ),
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_world_revolute_breakable_motor_packet_rejects_wrong_arg(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(
        tmp_path,
        benchmark_name="BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/8",
    )

    with pytest.raises(
        SystemExit,
        match="missing BM_AvbdArticulatedWorldRevoluteBreakableMotorStep/1",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_world_revolute_breakable_motor_packet_rejects_wrong_motor_count(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, motors=2.0)

    with pytest.raises(SystemExit, match="expected motors=1"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_world_revolute_breakable_motor_packet_rejects_wrong_breakable_count(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, breakable_motors=2.0)

    with pytest.raises(SystemExit, match="expected breakable_motors=1"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )
