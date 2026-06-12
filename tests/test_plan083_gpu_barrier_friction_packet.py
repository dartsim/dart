import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_gpu_barrier_friction_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_gpu_barrier_friction_packet",
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
        "BM_Plan083BarrierFrictionLocalCpu/1024",
        samples=1024,
        active_barriers=819,
        active_friction=1024,
        dynamic_friction=342,
        max_barrier_value=1.0,
        max_friction_work=0.5,
        max_result_abs_error=0.0,
    )
    gpu = _row(
        "BM_Plan083BarrierFrictionLocalCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        samples=1024,
        active_barriers=819,
        active_friction=1024,
        dynamic_friction=342,
        gpu_active_barriers=819,
        gpu_active_friction=1024,
        gpu_dynamic_friction=342,
        max_barrier_value=1.0,
        max_friction_work=0.5,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    point_triangle_cpu = _row(
        "BM_Plan083PointTriangleBarrierGradientCpu/1024",
        samples=1024,
        active_barriers=930,
        max_barrier_value=2.0,
        max_result_abs_error=0.0,
    )
    point_triangle_gpu = _row(
        "BM_Plan083PointTriangleBarrierGradientCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        samples=1024,
        active_barriers=930,
        gpu_active_barriers=930,
        max_barrier_value=2.0,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    point_triangle_tangent_cpu = _row(
        "BM_Plan083PointTriangleTangentStencilCpu/1024",
        samples=1024,
        fallback_bases=0,
        max_result_abs_error=0.0,
    )
    point_triangle_tangent_gpu = _row(
        "BM_Plan083PointTriangleTangentStencilCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        samples=1024,
        fallback_bases=0,
        gpu_fallback_bases=0,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    edge_edge_tangent_cpu = _row(
        "BM_Plan083EdgeEdgeTangentStencilCpu/1024",
        samples=1024,
        fallback_bases=0,
        max_result_abs_error=0.0,
    )
    edge_edge_tangent_gpu = _row(
        "BM_Plan083EdgeEdgeTangentStencilCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        samples=1024,
        fallback_bases=0,
        gpu_fallback_bases=0,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        kernel_ns=3.0,
        device_to_host_ns=4.0,
    )
    gpu.update(overrides)
    return {
        "benchmarks": [
            cpu,
            gpu,
            point_triangle_cpu,
            point_triangle_gpu,
            point_triangle_tangent_cpu,
            point_triangle_tangent_gpu,
            edge_edge_tangent_cpu,
            edge_edge_tangent_gpu,
        ]
    }


def test_plan083_gpu_barrier_friction_packet_accepts_parity_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(),
        sample_count=1024,
        tolerance=1e-10,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_barrier_friction_packet"]
    assert row["row_id"] == "barrier-friction-local-kernels"
    assert row["same_scene_cpu_gpu"] is True
    assert row["active_barrier_count"] == 819
    assert row["active_friction_count"] == 1024
    assert row["dynamic_friction_count"] == 342
    assert row["max_result_abs_error"] == 1e-12
    assert row["meets_speedup_gate"] is True
    assert row["scalar_local"]["active_barrier_count"] == 819
    assert row["point_triangle_barrier_gradient"]["active_barrier_count"] == 930
    assert row["point_triangle_barrier_gradient"]["max_result_abs_error"] == 1e-12
    assert row["point_triangle_tangent_stencil"]["fallback_basis_count"] == 0
    assert row["point_triangle_tangent_stencil"]["max_result_abs_error"] == 1e-12
    assert row["edge_edge_tangent_stencil"]["fallback_basis_count"] == 0
    assert row["edge_edge_tangent_stencil"]["max_result_abs_error"] == 1e-12


def test_plan083_gpu_barrier_friction_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(max_result_abs_error=1e-3),
            sample_count=1024,
            tolerance=1e-10,
            speedup_gate=1.25,
        )
    except module.Plan083GpuBarrierFrictionPacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_plan083_gpu_barrier_friction_packet_rejects_count_mismatch() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(gpu_active_barriers=818),
            sample_count=1024,
            tolerance=1e-10,
            speedup_gate=1.25,
        )
    except module.Plan083GpuBarrierFrictionPacketError as exc:
        assert "count" in str(exc)
    else:
        raise AssertionError("expected count failure")


def test_plan083_gpu_barrier_friction_packet_records_speedup_gate_miss() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(real_time=20.0, cpu_time=20.0),
        sample_count=1024,
        tolerance=1e-10,
        speedup_gate=1.25,
    )

    row = packet["plan083_gpu_barrier_friction_packet"]
    assert row["speedup"] == 0.5
    assert row["meets_speedup_gate"] is False
