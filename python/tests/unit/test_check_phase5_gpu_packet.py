import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_phase5_gpu_packet.py"

CPU_ROW = "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100"
GPU_ROW = "BM_Phase5RigidBodyBatchGpu/4096/128/100"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_phase5_gpu_packet", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _median_row(
    name,
    real_time,
    *,
    backend="cpu",
    includes_transfer_time=False,
    lane_count=4096,
    step_count=100,
):
    return {
        "name": f"{name}_median",
        "run_name": f"{name}_median",
        "run_type": "aggregate",
        "aggregate_name": "median",
        "real_time": real_time,
        "time_unit": "ms",
        "backend": backend,
        "precision": "double-reference",
        "includes_transfer_time": includes_transfer_time,
        "lane_count": lane_count,
        "resolved_execution_shape": "homogeneous-batch",
        "step_count": step_count,
    }


def _packet(*, cpu_ms=100.0, gpu_ms=70.0, max_error=1e-12, world_count=4096):
    return {
        "phase5_gpu_packet": {
            "world_count": world_count,
            "body_count": 128,
            "step_count": 100,
            "includes_transfer_setup_compute_readback": True,
            "max_final_state_abs_error": max_error,
            "gpu_build_import_gate_passed": True,
            "compute_backend_boundaries_passed": True,
            "no_gpu_runtime_dependencies_passed": True,
            "phase5_benchmark_contract_passed": True,
        },
        "benchmarks": [
            _median_row(
                f"BM_Phase5RigidBodyBatchCpuBaseline/{world_count}/128/100",
                cpu_ms,
                lane_count=world_count,
            ),
            _median_row(
                f"BM_Phase5RigidBodyBatchGpu/{world_count}/128/100",
                gpu_ms,
                backend="cuda",
                includes_transfer_time=True,
                lane_count=world_count,
            ),
        ],
    }


def test_validate_packet_accepts_phase5_go_packet():
    module = _load_module()

    summary = module.validate_packet(_packet())

    assert summary["world_count"] == 4096
    assert summary["body_count"] == 128
    assert summary["step_count"] == 100
    assert summary["speedup"] == pytest.approx(100.0 / 70.0)
    assert summary["max_final_state_abs_error"] == pytest.approx(1e-12)


def test_make_packet_template_uses_registered_workload_shape():
    module = _load_module()

    template = module.make_packet_template()

    metadata = template["phase5_gpu_packet"]
    assert metadata["world_count"] == 4096
    assert metadata["body_count"] == 128
    assert metadata["step_count"] == 100
    assert metadata["includes_transfer_setup_compute_readback"] is False
    assert metadata["max_final_state_abs_error"] is None
    assert metadata["gpu_build_import_gate_passed"] is False
    assert metadata["compute_backend_boundaries_passed"] is False
    assert metadata["no_gpu_runtime_dependencies_passed"] is False
    assert metadata["phase5_benchmark_contract_passed"] is False
    names = [row["name"] for row in template["benchmarks"]]
    assert names == [
        "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100_median",
        "BM_Phase5RigidBodyBatchGpu/4096/128/100_median",
    ]


def test_validate_packet_allows_memory_bounded_power_of_two_world_count():
    module = _load_module()

    summary = module.validate_packet(_packet(world_count=1024))

    assert summary["world_count"] == 1024


def test_validate_packet_rejects_missing_metadata():
    module = _load_module()

    with pytest.raises(module.Phase5PacketError, match="phase5_gpu_packet"):
        module.validate_packet({"benchmarks": [_median_row(CPU_ROW, 100.0)]})


def test_validate_packet_rejects_missing_gpu_row():
    module = _load_module()
    packet = _packet()
    packet["benchmarks"] = [_median_row(CPU_ROW, 100.0)]

    with pytest.raises(module.Phase5PacketError, match=GPU_ROW):
        module.validate_packet(packet)


def test_validate_packet_rejects_non_median_rows():
    module = _load_module()
    packet = _packet()
    packet["benchmarks"] = [
        {"name": CPU_ROW, "run_name": CPU_ROW, "real_time": 100.0},
        {"name": GPU_ROW, "run_name": GPU_ROW, "real_time": 70.0},
    ]

    with pytest.raises(module.Phase5PacketError, match="missing median"):
        module.validate_packet(packet)


def test_validate_packet_rejects_small_world_count():
    module = _load_module()

    with pytest.raises(module.Phase5PacketError, match="below the go-decision"):
        module.validate_packet(_packet(world_count=512))


def test_validate_packet_rejects_non_power_of_two_world_count():
    module = _load_module()

    with pytest.raises(module.Phase5PacketError, match="power of two"):
        module.validate_packet(_packet(world_count=1536))


def test_validate_packet_rejects_partial_workload_metadata():
    module = _load_module()
    packet = _packet()
    packet["phase5_gpu_packet"]["includes_transfer_setup_compute_readback"] = False

    with pytest.raises(module.Phase5PacketError, match="transfer"):
        module.validate_packet(packet)


def test_validate_packet_rejects_final_state_mismatch():
    module = _load_module()

    with pytest.raises(module.Phase5PacketError, match="final-state error"):
        module.validate_packet(_packet(max_error=1e-6))


@pytest.mark.parametrize(
    "flag",
    [
        "gpu_build_import_gate_passed",
        "compute_backend_boundaries_passed",
        "no_gpu_runtime_dependencies_passed",
        "phase5_benchmark_contract_passed",
    ],
)
def test_validate_packet_rejects_missing_evidence_gate(flag):
    module = _load_module()
    packet = _packet()
    del packet["phase5_gpu_packet"][flag]

    with pytest.raises(module.Phase5PacketError, match=flag):
        module.validate_packet(packet)


@pytest.mark.parametrize(
    "flag",
    [
        "gpu_build_import_gate_passed",
        "compute_backend_boundaries_passed",
        "no_gpu_runtime_dependencies_passed",
        "phase5_benchmark_contract_passed",
    ],
)
def test_validate_packet_rejects_failed_evidence_gate(flag):
    module = _load_module()
    packet = _packet()
    packet["phase5_gpu_packet"][flag] = False

    with pytest.raises(module.Phase5PacketError, match=f"{flag} must be true"):
        module.validate_packet(packet)


def test_validate_packet_rejects_insufficient_speedup():
    module = _load_module()

    with pytest.raises(module.Phase5PacketError, match="below required"):
        module.validate_packet(_packet(cpu_ms=100.0, gpu_ms=90.0))


def test_main_reads_packet_and_prints_summary(tmp_path, capsys):
    module = _load_module()
    packet_path = tmp_path / "phase5_packet.json"
    packet_path.write_text(json.dumps(_packet()), encoding="utf-8")

    assert module.main(["--input", str(packet_path)]) == 0

    captured = capsys.readouterr()
    assert "Phase 5 GPU packet accepted" in captured.out
    assert "worldCount=4096" in captured.out


def test_main_requires_input_unless_writing_template():
    module = _load_module()

    with pytest.raises(SystemExit, match="--input is required"):
        module.main([])


def test_main_writes_packet_template(tmp_path, capsys):
    module = _load_module()
    packet_path = tmp_path / "phase5_template.json"

    assert module.main(["--write-template", str(packet_path)]) == 0

    packet = json.loads(packet_path.read_text(encoding="utf-8"))
    assert packet["phase5_gpu_packet"]["world_count"] == 4096
    assert packet["benchmarks"][1]["name"] == (
        "BM_Phase5RigidBodyBatchGpu/4096/128/100_median"
    )
    captured = capsys.readouterr()
    assert "Wrote Phase 5 GPU packet template" in captured.out
