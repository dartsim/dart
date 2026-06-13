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
    sparse_residual_cpu = _row(
        "BM_NewtonSparseResidualCpu/1024",
        rows=1024,
        bodies=128,
        dofs=768,
        blocks=128,
        block_entries=4608,
        active_dofs=768,
        max_diagonal=10.0,
        max_gradient_abs=1.0,
        output_norm=4.0,
        max_output_abs=0.75,
        max_result_abs_error=0.0,
    )
    sparse_residual_gpu = _row(
        "BM_NewtonSparseResidualCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        rows=1024,
        bodies=128,
        dofs=768,
        blocks=128,
        block_entries=4608,
        active_dofs=768,
        gpu_rows=1024,
        gpu_bodies=128,
        gpu_dofs=768,
        gpu_blocks=128,
        gpu_active_dofs=768,
        gpu_output_norm=4.0,
        gpu_max_output_abs=0.75,
        max_result_abs_error=3e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        assembly_kernel_ns=3.0,
        gradient_seed_ns=4.0,
        diagonal_kernel_ns=5.0,
        off_diagonal_kernel_ns=6.0,
        device_to_host_ns=7.0,
    )
    equality_cpu = _row(
        "BM_NewtonEqualityReducedSolveCpu/1024",
        rows=1024,
        bodies=128,
        dofs=768,
        reduction_entries=768,
        reduced_dofs=384,
        active_reduced_dofs=384,
        max_diagonal=12.0,
        max_gradient_abs=1.25,
        step_norm=0.5,
        residual_norm=0.0,
        max_result_abs_error=0.0,
    )
    equality_gpu = _row(
        "BM_NewtonEqualityReducedSolveCuda/1024",
        real_time=4.0,
        cpu_time=4.0,
        rows=1024,
        bodies=128,
        dofs=768,
        reduction_entries=768,
        reduced_dofs=384,
        active_reduced_dofs=384,
        gpu_rows=1024,
        gpu_bodies=128,
        gpu_dofs=768,
        gpu_reduction_entries=768,
        gpu_reduced_dofs=384,
        gpu_active_reduced_dofs=384,
        gpu_step_norm=0.5,
        gpu_residual_norm=2e-14,
        max_result_abs_error=2e-12,
        host_setup_ns=1.0,
        host_to_device_ns=2.0,
        assembly_kernel_ns=3.0,
        reduction_kernel_ns=4.0,
        solve_kernel_ns=5.0,
        expansion_kernel_ns=6.0,
        device_to_host_ns=7.0,
    )
    gpu.update(overrides)
    return {
        "benchmarks": [
            cpu,
            gpu,
            off_diagonal_cpu,
            off_diagonal_gpu,
            sparse_residual_cpu,
            sparse_residual_gpu,
            equality_cpu,
            equality_gpu,
        ]
    }


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
    assert row["max_result_abs_error"] == 3e-12
    assert row["residual_norm"] == 2e-14
    assert row["meets_speedup_gate"] is True
    assert row["diagonal_assembly_solve"]["body_count"] == 128
    assert row["off_diagonal_sparse_block_assembly"]["pair_count"] == 64
    assert row["off_diagonal_sparse_block_assembly"]["active_block_count"] == 64
    assert row["off_diagonal_sparse_block_assembly"]["max_result_abs_error"] == 1e-12
    sparse = row["sparse_block_residual"]
    assert sparse["body_count"] == 128
    assert sparse["dof_count"] == 768
    assert sparse["block_count"] == 128
    assert sparse["block_entry_count"] == 4608
    assert sparse["active_dof_count"] == 768
    assert sparse["max_result_abs_error"] == 3e-12
    assert sparse["output_norm"] == 4.0
    assert sparse["timing_ns"]["gradient_seed"] == 4.0
    assert sparse["timing_ns"]["off_diagonal"] == 6.0
    equality = row["equality_reduced_diagonal_solve"]
    assert equality["body_count"] == 128
    assert equality["full_dof_count"] == 768
    assert equality["reduction_entry_count"] == 768
    assert equality["reduced_dof_count"] == 384
    assert equality["active_reduced_dof_count"] == 384
    assert equality["max_result_abs_error"] == 2e-12
    assert equality["residual_norm"] == 2e-14
    assert equality["timing_ns"]["reduction"] == 4.0
    assert equality["timing_ns"]["expansion"] == 6.0


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
