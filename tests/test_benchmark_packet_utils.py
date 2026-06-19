import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / "scripts"


def load_script_module(name):
    if str(SCRIPTS) not in sys.path:
        sys.path.insert(0, str(SCRIPTS))
    script_path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, script_path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def aggregate_row(name, aggregate, real_time, *, run_name=True, time_unit="ns"):
    row = {
        "aggregate_name": aggregate,
        "cpu_time": real_time,
        "real_time": real_time,
        "time_unit": time_unit,
    }
    row["run_name" if run_name else "name"] = name
    return row


def test_canonical_benchmark_name_strips_google_benchmark_suffixes():
    utils = load_script_module("benchmark_packet_utils")

    assert utils.canonical_benchmark_name("BM_Test/repeats:3_median") == "BM_Test"
    assert utils.canonical_benchmark_name("BM_Test_mean") == "BM_Test"
    assert utils.canonical_benchmark_name("BM_Test/8/4_cv") == "BM_Test/8/4"


def test_median_timing_by_name_prefers_positive_median_rows():
    utils = load_script_module("benchmark_packet_utils")
    rows = [
        aggregate_row("BM_Test/repeats:3_mean", "mean", 10.0),
        aggregate_row("BM_Test/repeats:3_median", "median", 3.0, time_unit="us"),
        aggregate_row("BM_Bad_median", "median", 0.0),
    ]

    assert utils.median_timing_by_name(rows) == {"BM_Test": 3000.0}


def test_benchmark_timing_field_errors_reject_bad_google_rows():
    utils = load_script_module("benchmark_packet_utils")

    errors = utils.benchmark_timing_field_errors(
        {"real_time": float("nan"), "cpu_time": -1.0}, "BM_Bad"
    )

    assert errors == [
        "BM_Bad has non-finite real_time: nan",
        "BM_Bad has non-positive cpu_time: -1.0",
        "BM_Bad is missing time_unit",
    ]


def test_benchmark_packet_timing_schema_accepts_per_step_subphases():
    utils = load_script_module("benchmark_packet_utils")

    errors = utils.benchmark_packet_timing_schema_errors(
        {
            "step_count": 100,
            "solver_subphase_timings_ns": {
                "assembly": 125.0,
                "linear_solve": 250,
            },
        },
        "phase_packet",
        required_subphases=("assembly", "linear_solve"),
    )

    assert errors == []


def test_benchmark_packet_timing_schema_rejects_bad_fields():
    utils = load_script_module("benchmark_packet_utils")

    errors = utils.benchmark_packet_timing_schema_errors(
        {
            "step_count": 0,
            "solver_subphase_timings_ns": {
                "assembly": -1.0,
                "": 1.0,
                7: 2.0,
                "linear_solve": True,
            },
        },
        "phase_packet",
        required_subphases=("assembly", "ccd"),
    )

    assert errors == [
        "phase_packet.step_count must be positive",
        "phase_packet.solver_subphase_timings_ns.ccd is missing",
        "phase_packet.solver_subphase_timings_ns has an invalid key",
        "phase_packet.solver_subphase_timings_ns has an invalid key",
        "phase_packet.solver_subphase_timings_ns.assembly must be finite and non-negative",
        "phase_packet.solver_subphase_timings_ns.linear_solve must be numeric",
    ]


def test_batched_benchmark_row_schema_accepts_required_reporting_fields():
    utils = load_script_module("benchmark_packet_utils")

    errors = utils.batched_benchmark_row_schema_errors(
        {
            "backend": "cuda-resident",
            "precision": "float64-reference",
            "includes_transfer_time": False,
            "lane_count": 4,
            "resolved_execution_shape": "homogeneous-batch",
            "step_count": 100,
        },
        "plan091_batched_packet.rows[0]",
    )

    assert errors == []


def test_batched_benchmark_row_schema_rejects_missing_reporting_fields():
    utils = load_script_module("benchmark_packet_utils")

    errors = utils.batched_benchmark_row_schema_errors(
        {
            "backend": "",
            "precision": None,
            "includes_transfer_time": "no",
            "lane_count": 0,
            "resolved_execution_shape": "",
            "step_count": True,
        },
        "plan091_batched_packet.rows[0]",
    )

    assert errors == [
        "plan091_batched_packet.rows[0].backend must be a non-empty string",
        "plan091_batched_packet.rows[0].precision must be a non-empty string",
        "plan091_batched_packet.rows[0].includes_transfer_time must be a boolean",
        "plan091_batched_packet.rows[0].lane_count must be positive",
        "plan091_batched_packet.rows[0].resolved_execution_shape must be a non-empty string",
        "plan091_batched_packet.rows[0].step_count must be an integer",
    ]


def test_phase5_packet_validator_uses_shared_canonical_rows():
    phase5 = load_script_module("check_phase5_gpu_packet")
    data = phase5.make_packet_template()
    metadata = data["phase5_gpu_packet"]
    metadata["includes_transfer_setup_compute_readback"] = True
    metadata["max_final_state_abs_error"] = 1e-12
    for key in phase5.REQUIRED_EVIDENCE_FLAGS:
        metadata[key] = True
    cpu_row = aggregate_row(
        "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100/repeats:3_median",
        "median",
        4.0,
        time_unit="ms",
    )
    cpu_row.update(
        {
            "backend": "cpu",
            "precision": "double-reference",
            "includes_transfer_time": False,
            "lane_count": 4096,
            "resolved_execution_shape": "homogeneous-batch",
            "step_count": 100,
        }
    )
    gpu_row = aggregate_row(
        "BM_Phase5RigidBodyBatchGpu/4096/128/100/repeats:3_median",
        "median",
        2.0,
        time_unit="ms",
    )
    gpu_row.update(
        {
            "backend": "cuda",
            "precision": "double-reference",
            "includes_transfer_time": True,
            "lane_count": 4096,
            "resolved_execution_shape": "homogeneous-batch",
            "step_count": 100,
        }
    )
    data["benchmarks"] = [cpu_row, gpu_row]

    summary = phase5.validate_packet(data)

    assert summary["cpu_median_ns"] == 4_000_000.0
    assert summary["gpu_median_ns"] == 2_000_000.0
    assert summary["speedup"] == 2.0


def test_phase5_packet_validator_rejects_swapped_backend_labels():
    phase5 = load_script_module("check_phase5_gpu_packet")
    data = phase5.make_packet_template()
    metadata = data["phase5_gpu_packet"]
    metadata["includes_transfer_setup_compute_readback"] = True
    metadata["max_final_state_abs_error"] = 1e-12
    for key in phase5.REQUIRED_EVIDENCE_FLAGS:
        metadata[key] = True
    cpu_row, gpu_row = data["benchmarks"]
    cpu_row["backend"] = "cuda"
    gpu_row["backend"] = "cpu"
    cpu_row["real_time"] = 4.0
    cpu_row["cpu_time"] = 4.0
    gpu_row["real_time"] = 2.0
    gpu_row["cpu_time"] = 2.0

    try:
        phase5.validate_packet(data)
    except phase5.Phase5PacketError as exc:
        assert (
            "phase5_gpu_packet.benchmarks[BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100].backend must be 'cpu'"
            in str(exc)
        )
    else:
        raise AssertionError("expected swapped backend labels to fail")


def test_phase5_packet_validator_rejects_missing_batched_row_metadata():
    phase5 = load_script_module("check_phase5_gpu_packet")
    data = phase5.make_packet_template()
    metadata = data["phase5_gpu_packet"]
    metadata["includes_transfer_setup_compute_readback"] = True
    metadata["max_final_state_abs_error"] = 1e-12
    for key in phase5.REQUIRED_EVIDENCE_FLAGS:
        metadata[key] = True
    data["benchmarks"] = [
        aggregate_row(
            "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100_median",
            "median",
            4.0,
            time_unit="ms",
        ),
        aggregate_row(
            "BM_Phase5RigidBodyBatchGpu/4096/128/100_median",
            "median",
            2.0,
            time_unit="ms",
        ),
    ]

    try:
        phase5.validate_packet(data)
    except phase5.Phase5PacketError as exc:
        assert (
            "phase5_gpu_packet.benchmarks[BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100].backend"
            in str(exc)
        )
    else:
        raise AssertionError("expected missing batched-row metadata to fail")


def test_phase5_cuda_packet_writer_annotates_representative_batched_rows():
    writer = load_script_module("write_phase5_cuda_packet")

    class Args:
        cpu_prefix = "BM_Phase5RigidBodyBatchCpuBaseline"
        gpu_prefix = "BM_Phase5RigidBodyBatchGpu"
        world_count = 4096
        body_count = 128
        step_count = 100
        includes_transfer_setup_compute_readback = True
        gpu_build_import_gate_passed = True
        compute_backend_boundaries_passed = True
        no_gpu_runtime_dependencies_passed = True
        phase5_benchmark_contract_passed = True

    rows = [
        {
            "run_name": "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100/repeats:3",
            "cpu_time": 4.1,
            "real_time": 4.1,
            "time_unit": "ms",
        },
        aggregate_row(
            "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100_median",
            "median",
            4.0,
            time_unit="ms",
        ),
        {
            "run_name": "BM_Phase5RigidBodyBatchGpu/4096/128/100/repeats:3",
            "cpu_time": 2.1,
            "real_time": 2.1,
            "time_unit": "ms",
        },
        aggregate_row(
            "BM_Phase5RigidBodyBatchGpu/4096/128/100_median",
            "median",
            2.0,
            time_unit="ms",
        )
        | {"max_final_state_abs_error": 1e-12},
        aggregate_row("BM_Unrelated/1_median", "median", 8.0, time_unit="ms"),
    ]

    packet = writer.make_packet({"benchmarks": rows}, Args())

    cpu_raw, cpu_row, gpu_raw, gpu_row, unrelated = packet["benchmarks"]
    assert cpu_raw["backend"] == "cpu"
    assert cpu_row["backend"] == "cpu"
    assert cpu_row["includes_transfer_time"] is False
    assert gpu_raw["backend"] == "cuda"
    assert gpu_row["backend"] == "cuda"
    assert gpu_row["precision"] == "double-reference"
    assert gpu_row["includes_transfer_time"] is True
    assert gpu_row["lane_count"] == 4096
    assert gpu_row["resolved_execution_shape"] == "homogeneous-batch"
    assert "backend" not in unrelated


def test_abd_packet_validator_uses_shared_row_validation(tmp_path, capsys):
    abd = load_script_module("check_abd_comparison_packet")
    rows = [
        aggregate_row(f"{name}_median", "median", 1.0)
        for name in sorted(abd.EXPECTED_BENCHMARKS)
    ]
    for row in rows:
        if row["run_name"] == f"{abd.MICRO_SOLVE_BENCHMARK}_median":
            row.update(
                {
                    "row_abd_alg_affine_body": 1.0,
                    "paper_scale": 0.0,
                    "affine_dynamic_body_count": 1.0,
                    "static_triangle_body_count": 1.0,
                    "point_triangle_pair_count": 1.0,
                    "valid_solve": 1.0,
                    "converged": 1.0,
                    "barrier_active": 1.0,
                    "solver_iterations": 8.0,
                    "initial_objective": 2.0,
                    "final_objective": 1.0,
                    "objective_decrease": 1.0,
                    "initial_gradient_norm": 3.0,
                    "final_gradient_norm": 0.5,
                    "initial_squared_distance": 0.01,
                    "final_squared_distance": 0.02,
                    "squared_activation_distance": 0.1,
                }
            )
    packet = {
        "benchmarks": rows,
    }
    path = tmp_path / "abd_packet.json"
    path.write_text(json.dumps(packet), encoding="utf-8")

    abd.validate_packet(path)

    assert "ABD comparison packet OK" in capsys.readouterr().out
