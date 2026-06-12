import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_newton_assembly_solve_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_newton_assembly_solve_packet",
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
        "BM_NewtonAssemblySolveCpu/1024",
        rows=1024,
        bodies=128,
        dofs=768,
        active_dofs=768,
        max_diagonal=10.0,
        max_gradient_abs=1.0,
        step_norm=0.25,
        residual_norm=0.0,
        max_result_abs_error=0.0,
    )
    gpu = _row(
        "BM_NewtonAssemblySolveCuda/1024",
        real_time=5.0,
        cpu_time=5.0,
        rows=1024,
        bodies=128,
        dofs=768,
        active_dofs=768,
        gpu_rows=1024,
        gpu_bodies=128,
        gpu_active_dofs=768,
        gpu_step_norm=0.25,
        gpu_residual_norm=1e-14,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        assembly_kernel_ns=3.0,
        solve_kernel_ns=4.0,
        device_to_host_ns=5.0,
    )
    off_diagonal_cpu = _row(
        "BM_NewtonOffDiagonalAssemblyCpu/1024",
        rows=1024,
        pairs=64,
        block_entries=2304,
        active_blocks=64,
        max_block_abs=0.5,
        max_result_abs_error=0.0,
    )
    off_diagonal_gpu = _row(
        "BM_NewtonOffDiagonalAssemblyCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        rows=1024,
        pairs=64,
        block_entries=2304,
        active_blocks=64,
        gpu_rows=1024,
        gpu_pairs=64,
        gpu_active_blocks=64,
        gpu_max_block_abs=0.5,
        max_result_abs_error=1e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        assembly_kernel_ns=3.0,
        solve_kernel_ns=0.0,
        device_to_host_ns=5.0,
    )
    gpu.update(overrides)
    return {"benchmarks": [cpu, gpu, off_diagonal_cpu, off_diagonal_gpu]}


def test_newton_assembly_solve_packet_accepts_parity_rows() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(),
        row_count=1024,
        tolerance=1e-10,
        residual_tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["newton_assembly_solve_packet"]
    assert row["row_id"] == "assembly-linear-solve"
    assert row["same_scene_cpu_gpu"] is True
    assert row["row_count"] == 1024
    assert row["body_count"] == 128
    assert row["dof_count"] == 768
    assert row["active_dof_count"] == 768
    assert row["max_result_abs_error"] == 1e-12
    assert row["residual_norm"] == 1e-14
    assert row["meets_speedup_gate"] is True
    assert row["diagonal_assembly_solve"]["body_count"] == 128
    assert row["off_diagonal_sparse_block_assembly"]["pair_count"] == 64
    assert row["off_diagonal_sparse_block_assembly"]["active_block_count"] == 64
    assert row["off_diagonal_sparse_block_assembly"]["max_result_abs_error"] == 1e-12


def test_newton_assembly_solve_packet_rejects_accuracy_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(max_result_abs_error=1e-3),
            row_count=1024,
            tolerance=1e-10,
            residual_tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.NewtonAssemblySolvePacketError as exc:
        assert "exceeds tolerance" in str(exc)
    else:
        raise AssertionError("expected accuracy failure")


def test_newton_assembly_solve_packet_rejects_residual_failure() -> None:
    module = _load_module()

    try:
        module.make_packet(
            _benchmark_data(gpu_residual_norm=1e-4),
            row_count=1024,
            tolerance=1e-10,
            residual_tolerance=1e-8,
            speedup_gate=1.25,
        )
    except module.NewtonAssemblySolvePacketError as exc:
        assert "residual norm" in str(exc)
    else:
        raise AssertionError("expected residual failure")


def test_newton_assembly_solve_packet_records_speedup_gate_miss() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_data(real_time=20.0, cpu_time=20.0),
        row_count=1024,
        tolerance=1e-10,
        residual_tolerance=1e-8,
        speedup_gate=1.25,
    )

    row = packet["newton_assembly_solve_packet"]
    assert row["speedup"] == 0.5
    assert row["meets_speedup_gate"] is False
