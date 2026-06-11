from __future__ import annotations

import importlib.util
import json
import struct
import sys
import zlib
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_avbd_paper_scale_high_ratio_chain_packet.py"


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_scale_high_ratio_chain_packet",
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
    scene: str = "avbd_paper_scale_high_ratio_chain",
) -> Path:
    capture = tmp_path / "capture"
    frames = capture / "png_frames"
    frames.mkdir(parents=True)
    screenshot = capture / "avbd_paper_scale_high_ratio_chain.png"
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
    links: float = 50.0,
    mass_ratio: float = 50000.0,
    max_iterations: float = 200.0,
    tolerance: float = 1e-9,
    replay_steps: float = 32.0,
    replay_seconds: float = 0.16,
) -> Path:
    benchmark = {
        "benchmarks": [
            {
                "cpu_time": 420000.0,
                "iterations": 10,
                "links": links,
                "mass_ratio": mass_ratio,
                "max_iterations": max_iterations,
                "name": "BM_AvbdPaperScaleHighRatioChainStep",
                "real_time": 421000.0,
                "replay_seconds": replay_seconds,
                "replay_steps": replay_steps,
                "run_name": "BM_AvbdPaperScaleHighRatioChainStep",
                "run_type": "iteration",
                "time_unit": "ns",
                "tolerance": tolerance,
            },
            {
                "aggregate_name": "median",
                "aggregate_unit": "time",
                "cpu_time": 430000.0,
                "iterations": 1,
                "links": links,
                "mass_ratio": mass_ratio,
                "max_iterations": max_iterations,
                "name": "BM_AvbdPaperScaleHighRatioChainStep_median",
                "real_time": 431000.0,
                "replay_seconds": replay_seconds,
                "replay_steps": replay_steps,
                "run_name": "BM_AvbdPaperScaleHighRatioChainStep",
                "run_type": "aggregate",
                "time_unit": "ns",
                "tolerance": tolerance,
            },
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


def test_avbd_paper_scale_high_ratio_chain_packet_records_benchmark_evidence(
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
    assert packet["packet"] == "avbd_paper_scale_high_ratio_chain"
    assert packet["scene"] == "avbd_paper_scale_high_ratio_chain"
    assert packet["target"] == {
        "complete_paper_reproduction": False,
        "paper_gap": "50-body pendulum with 50,000:1 mass ratio",
        "scope": (
            "paper-scale 50-link articulated-chain visual/CPU benchmark smoke "
            "with a 50,000:1 heavy tip"
        ),
    }
    assert packet["scene_invariants"] == {
        "light_link_mass": 1.0,
        "links": 50,
        "mass_ratio": 50000.0,
        "max_iterations": 200,
        "replay_seconds": 0.16,
        "replay_steps": 32,
        "time_step": 0.005,
        "tip_link_mass": 50000.0,
        "tolerance": 1e-9,
    }
    assert packet["benchmark"]["benchmark"] == "BM_AvbdPaperScaleHighRatioChainStep"
    assert packet["benchmark"]["cpu_time_per_step_ns"] == 430000.0
    assert packet["benchmark"]["invariants"] == {
        "links": 50,
        "mass_ratio": 50000.0,
        "max_iterations": 200,
        "replay_seconds": 0.16,
        "replay_steps": 32,
        "time_step": 0.005,
        "tolerance": 1e-9,
    }
    assert (
        packet["visual_capture"]["scene"] == "avbd_paper_scale_high_ratio_chain"
    )
    assert packet["visual_capture"]["frames"]["count"] == 2
    assert (
        "same-hardware paper-number comparison for the 50-link 50,000:1 chain"
        in packet["remaining_gates"]
    )


def test_avbd_paper_scale_high_ratio_chain_packet_rejects_wrong_benchmark(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark = {
        "benchmarks": [
            {
                "cpu_time": 1.0,
                "name": "BM_AvbdArticulatedHighRatioChainStep",
                "real_time": 1.0,
                "run_name": "BM_AvbdArticulatedHighRatioChainStep",
                "run_type": "iteration",
            }
        ]
    }
    benchmark_json = tmp_path / "benchmark.json"
    benchmark_json.write_text(json.dumps(benchmark), encoding="utf-8")

    with pytest.raises(
        SystemExit,
        match="benchmark JSON missing BM_AvbdPaperScaleHighRatioChainStep",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_paper_scale_high_ratio_chain_packet_rejects_wrong_counters(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path)
    benchmark_json = _write_benchmark_json(tmp_path, links=5.0)

    with pytest.raises(SystemExit, match="expected links=50"):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )


def test_avbd_paper_scale_high_ratio_chain_packet_rejects_wrong_scene(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    capture_manifest = _write_capture_manifest(tmp_path, scene="avbd_empty_baseline")
    benchmark_json = _write_benchmark_json(tmp_path)

    with pytest.raises(
        SystemExit,
        match="capture scene must be avbd_paper_scale_high_ratio_chain",
    ):
        module.main(
            [
                "--capture-manifest",
                str(capture_manifest),
                "--benchmark-json",
                str(benchmark_json),
            ]
        )
