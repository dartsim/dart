from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_empty_baseline_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_empty_baseline_packet",
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


def _write_capture_manifest(tmp_path: Path, *, scene: str = "avbd_empty_baseline"):
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_empty_baseline.png"
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


def _write_benchmark_json(tmp_path: Path, *, rigid_bodies: float = 0.0) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "cpu_time": 2.0,
                "iterations": 10,
                "multibodies": 0.0,
                "name": "BM_AvbdEmptyWorldStep",
                "real_time": 3.0,
                "rigid_bodies": rigid_bodies,
                "run_name": "BM_AvbdEmptyWorldStep",
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


def test_avbd_empty_baseline_packet_records_visual_and_benchmark_evidence(
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
    assert packet["scene"] == "avbd_empty_baseline"
    assert [row["demo"] for row in packet["source_demo_reference_rows"]] == [
        "avbd-demo2d",
        "avbd-demo3d",
    ]
    assert packet["visual_capture"]["screenshot"]["width"] == 4
    assert packet["visual_capture"]["screenshot"]["height"] == 3
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert packet["benchmark"]["benchmark"] == "BM_AvbdEmptyWorldStep"
    assert packet["benchmark"]["invariants"] == {
        "multibodies": 0,
        "rigid_bodies": 0,
        "time_advances_without_simulated_bodies": True,
    }
    assert packet["benchmark"]["rows"][0]["rigid_bodies"] == 0.0


def test_avbd_empty_baseline_packet_rejects_wrong_scene(tmp_path: Path) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="vbd_cloth")
    benchmark_json = _write_benchmark_json(tmp_path)

    with pytest.raises(SystemExit, match="capture scene must be avbd_empty_baseline"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_empty_baseline_packet_rejects_nonempty_benchmark_row(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, rigid_bodies=1.0)

    with pytest.raises(SystemExit, match="expected zero bodies"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )
