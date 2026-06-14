import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_ccd_line_search_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_ccd_line_search_packet",
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
    cpu = _row(
        "BM_Plan083CcdLineSearchCpu/1024",
        pairs=1024,
        hits=768,
        min_step_bound=0.5,
        max_result_abs_error=0.0,
    )
    gpu = _row(
        "BM_Plan083CcdLineSearchCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        pairs=1024,
        hits=768,
        gpu_hits=768,
        min_step_bound=0.5,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    edge_cpu = _row(
        "BM_Plan083EdgeEdgeCcdLineSearchCpu/1024",
        pairs=1024,
        hits=512,
        min_step_bound=0.25,
        max_result_abs_error=0.0,
    )
    edge_gpu = _row(
        "BM_Plan083EdgeEdgeCcdLineSearchCuda/1024",
        real_time=8.0,
        cpu_time=8.0,
        pairs=1024,
        hits=512,
        gpu_hits=512,
        min_step_bound=0.25,
        max_result_abs_error=2e-12,
        host_setup_ns=1.5,
        host_to_device_ns=2.5,
        kernel_ns=3.5,
        device_to_host_ns=4.5,
    )
    gpu.update(overrides)
    edge_gpu.update(overrides)
    return {"benchmarks": [cpu, gpu, edge_cpu, edge_gpu]}


def test_plan083_gpu_ccd_line_search_packet_accepts_parity_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(),
        pair_count=1024,
        tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_ccd_line_search_packet"]
    assert row["row_id"] == "ccd-line-search"
    assert row["same_scene_cpu_gpu"] is True
    assert row["pair_count"] == 2048
    assert row["hit_count"] == 1280
    assert row["min_step_bound"] == 0.25
    assert row["max_result_abs_error"] == 2e-12
    assert set(row["primitive_families"]) == {"point_triangle", "edge_edge"}
    assert row["meets_speedup_gate"] is True


def test_plan083_gpu_ccd_line_search_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(max_result_abs_error=1e-3),
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_plan083_gpu_ccd_line_search_packet_rejects_hit_mismatch() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(gpu_hits=767),
            pair_count=1024,
            tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.Plan083GpuCcdLineSearchPacketError as exc:
        assert "hit count" in str(exc)
    else:
        raise AssertionError("expected hit-count failure")


def test_plan083_gpu_ccd_line_search_packet_records_speedup_gate_miss() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(real_time=20.0, cpu_time=20.0),
        pair_count=1024,
        tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_ccd_line_search_packet"]
    assert row["speedup"] == 0.5
    assert row["meets_speedup_gate"] is False
