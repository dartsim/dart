import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "write_phase5_cuda_packet.py"

CPU_ROW = "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100"
GPU_ROW = "BM_Phase5RigidBodyBatchGpu/4096/128/100"


def _load_module():
    spec = importlib.util.spec_from_file_location("write_phase5_cuda_packet", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _median_row(name, real_time, **extra):
    return {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "run_type": "aggregate",
        "aggregate_name": "median",
        "real_time": real_time,
        "time_unit": "ms",
        **extra,
    }


def _benchmark_json(*, max_error=1e-12, cpu_ms=100.0, gpu_ms=70.0):
    return {
        "benchmarks": [
            _median_row(CPU_ROW, cpu_ms),
            _median_row(
                GPU_ROW,
                gpu_ms,
                max_final_state_abs_error=max_error,
            ),
        ]
    }


def _write_benchmark_json(tmp_path, data):
    path = tmp_path / "phase5_cuda.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _evidence_args():
    return [
        "--includes-transfer-setup-compute-readback",
        "--gpu-build-import-gate-passed",
        "--compute-backend-boundaries-passed",
        "--no-gpu-runtime-dependencies-passed",
        "--phase5-benchmark-contract-passed",
    ]


def test_main_writes_validated_cuda_packet(tmp_path, capsys):
    module = _load_module()
    benchmark_path = _write_benchmark_json(tmp_path, _benchmark_json())
    packet_path = tmp_path / "phase5_packet.json"

    assert (
        module.main(
            [
                "--benchmark-json",
                str(benchmark_path),
                "--output",
                str(packet_path),
                *_evidence_args(),
            ]
        )
        == 0
    )

    packet = json.loads(packet_path.read_text(encoding="utf-8"))
    metadata = packet["phase5_gpu_packet"]
    assert metadata["world_count"] == 4096
    assert metadata["body_count"] == 128
    assert metadata["step_count"] == 100
    assert metadata["includes_transfer_setup_compute_readback"] is True
    assert metadata["max_final_state_abs_error"] == pytest.approx(1e-12)
    assert packet["benchmarks"][1]["name"] == f"{GPU_ROW}_median"

    captured = capsys.readouterr()
    assert "Wrote Phase 5 CUDA packet" in captured.out
    assert "Phase 5 GPU packet accepted" in captured.out


def test_extract_max_final_state_abs_error_prefers_median_row():
    module = _load_module()
    rows = [
        {"name": GPU_ROW, "max_final_state_abs_error": 5e-12},
        {
            "name": f"{GPU_ROW}_median",
            "run_name": f"{GPU_ROW}_median",
            "aggregate_name": "median",
            "max_final_state_abs_error": 1e-12,
        },
    ]

    max_error = module.extract_max_final_state_abs_error(
        rows,
        gpu_prefix="BM_Phase5RigidBodyBatchGpu",
        world_count=4096,
        body_count=128,
        step_count=100,
    )

    assert max_error == pytest.approx(1e-12)


def test_main_rejects_missing_final_state_error_counter(tmp_path):
    module = _load_module()
    data = _benchmark_json()
    del data["benchmarks"][1]["max_final_state_abs_error"]
    benchmark_path = _write_benchmark_json(tmp_path, data)

    with pytest.raises(SystemExit, match="max_final_state_abs_error"):
        module.main(
            [
                "--benchmark-json",
                str(benchmark_path),
                *_evidence_args(),
            ]
        )


def test_main_rejects_missing_evidence_flag(tmp_path):
    module = _load_module()
    benchmark_path = _write_benchmark_json(tmp_path, _benchmark_json())

    with pytest.raises(SystemExit, match="gpu_build_import_gate_passed"):
        module.main(
            [
                "--benchmark-json",
                str(benchmark_path),
                "--includes-transfer-setup-compute-readback",
                "--compute-backend-boundaries-passed",
                "--no-gpu-runtime-dependencies-passed",
                "--phase5-benchmark-contract-passed",
            ]
        )
