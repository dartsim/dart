import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_contact_candidate_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_contact_candidate_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(name: str, **counters):
    row = {
        "name": name,
        "real_time": 10.0,
        "cpu_time": 10.0,
        "time_unit": "ns",
        "aggregate_name": "median",
        "iterations": 1,
    }
    row.update(counters)
    return row


def _benchmark_data(**overrides):
    point_cpu = _row(
        "BM_Plan083ContactCandidateCpu/1024",
        stencils=1024,
        accepted_count=768,
        max_result_abs_error=0.0,
    )
    point_gpu = _row(
        "BM_Plan083ContactCandidateCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        stencils=1024,
        accepted_count=768,
        gpu_accepted_count=768,
        max_result_abs_error=1e-14,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    edge_cpu = _row(
        "BM_Plan083EdgeEdgeContactCandidateCpu/1024",
        stencils=1024,
        accepted_count=768,
        max_result_abs_error=0.0,
    )
    edge_gpu = _row(
        "BM_Plan083EdgeEdgeContactCandidateCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        stencils=1024,
        accepted_count=768,
        gpu_accepted_count=768,
        max_result_abs_error=1e-14,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    point_gpu.update(overrides)
    return {"benchmarks": [point_cpu, point_gpu, edge_cpu, edge_gpu]}


def test_plan083_gpu_contact_candidate_packet_accepts_parity_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(),
        stencil_count=1024,
        tolerance=1e-10,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_contact_candidate_packet"]
    assert row["row_id"] == "contact-stencils-candidate-filtering"
    assert row["same_scene_cpu_gpu"] is True
    assert row["accepted_count"] == 1536
    assert row["max_result_abs_error"] == 1e-14
    assert row["meets_speedup_gate"] is True
    assert set(row["primitive_families"]) == {"point_triangle", "edge_edge"}


def test_plan083_gpu_contact_candidate_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(max_result_abs_error=1e-3),
            stencil_count=1024,
            tolerance=1e-10,
            speedup_gate=1.25,
        )
    except module.Plan083GpuContactCandidatePacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_plan083_gpu_contact_candidate_packet_rejects_count_mismatch() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(gpu_accepted_count=767),
            stencil_count=1024,
            tolerance=1e-10,
            speedup_gate=1.25,
        )
    except module.Plan083GpuContactCandidatePacketError as exc:
        assert "accepted count" in str(exc)
    else:
        raise AssertionError("expected count failure")


def test_plan083_gpu_contact_candidate_packet_records_speedup_gate_miss() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(real_time=20.0, cpu_time=20.0),
        stencil_count=1024,
        tolerance=1e-10,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_contact_candidate_packet"]
    assert row["speedup"] == 0.5
    assert row["meets_speedup_gate"] is False
