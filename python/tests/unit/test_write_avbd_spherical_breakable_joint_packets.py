from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_spherical_breakable_joint_packets.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_spherical_breakable_joint_packets",
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


def _write_capture_manifest(tmp_path: Path, *, scene: str) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / f"{scene}.png"
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
    benchmark_name: str,
    breakable_joints: float = 1.0,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "breakable_joints": breakable_joints,
                "cpu_time": 4300.0,
                "iterations": 10,
                "name": benchmark_name,
                "real_time": 4400.0,
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


@pytest.mark.parametrize(
    ("variant_name", "scene", "benchmark"),
    [
        (
            "rigid",
            "avbd_rigid_spherical_breakable_joint",
            "BM_AvbdRigidSphericalBreakableJointStep/1",
        ),
        (
            "articulated-world",
            "avbd_articulated_spherical_breakable_joint",
            "BM_AvbdArticulatedWorldSphericalBreakableJointStep/1",
        ),
        (
            "articulated-pair",
            "avbd_articulated_spherical_pair_breakable_joint",
            "BM_AvbdArticulatedSphericalPairBreakableJointStep/1",
        ),
    ],
)
def test_avbd_spherical_breakable_joint_packets_record_evidence(
    tmp_path: Path,
    variant_name: str,
    scene: str,
    benchmark: str,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene=scene)
    benchmark_json = _write_benchmark_json(tmp_path, benchmark_name=benchmark)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--variant",
                variant_name,
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
    assert packet["scene"] == scene
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["benchmark"]["benchmark"] == benchmark
    assert packet["benchmark"]["cpu_time_per_step_ns"] == 4300.0
    assert packet["benchmark"]["invariants"]["breakable_joints"] == 1
    assert packet["target"]["broad_breakable_constraint_complete"] is False


def test_avbd_spherical_breakable_joint_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_empty_baseline")
    benchmark_json = _write_benchmark_json(
        tmp_path,
        benchmark_name="BM_AvbdRigidSphericalBreakableJointStep/1",
    )

    with pytest.raises(
        SystemExit,
        match="capture scene must be avbd_rigid_spherical_breakable_joint",
    ):
        module.main(
            [
                "--variant",
                "rigid",
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_spherical_breakable_joint_packet_rejects_wrong_arg(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(
        tmp_path,
        scene="avbd_articulated_spherical_breakable_joint",
    )
    benchmark_json = _write_benchmark_json(
        tmp_path,
        benchmark_name="BM_AvbdArticulatedWorldSphericalBreakableJointStep/8",
    )

    with pytest.raises(
        SystemExit,
        match="missing BM_AvbdArticulatedWorldSphericalBreakableJointStep/1",
    ):
        module.main(
            [
                "--variant",
                "articulated-world",
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_spherical_breakable_joint_packet_rejects_wrong_joint_count(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(
        tmp_path,
        scene="avbd_articulated_spherical_pair_breakable_joint",
    )
    benchmark_json = _write_benchmark_json(
        tmp_path,
        benchmark_name="BM_AvbdArticulatedSphericalPairBreakableJointStep/1",
        breakable_joints=2.0,
    )

    with pytest.raises(SystemExit, match="expected breakable_joints=1"):
        module.main(
            [
                "--variant",
                "articulated-pair",
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )
