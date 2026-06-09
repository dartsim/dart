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


def test_canonical_benchmark_row_name_prefers_run_name():
    utils = load_script_module("benchmark_packet_utils")
    row = {
        "name": "BM_Ignored",
        "run_name": "BM_Test/8/4/repeats:3_median",
    }

    assert utils.canonical_benchmark_row_name(row) == "BM_Test/8/4"


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


def test_phase5_packet_validator_uses_shared_canonical_rows():
    phase5 = load_script_module("check_phase5_gpu_packet")
    data = phase5.make_packet_template()
    metadata = data["phase5_gpu_packet"]
    metadata["includes_transfer_setup_compute_readback"] = True
    metadata["max_final_state_abs_error"] = 1e-12
    for key in phase5.REQUIRED_EVIDENCE_FLAGS:
        metadata[key] = True
    data["benchmarks"] = [
        aggregate_row(
            "BM_Phase5RigidBodyBatchCpuBaseline/4096/128/100/repeats:3_median",
            "median",
            4.0,
            time_unit="ms",
        ),
        aggregate_row(
            "BM_Phase5RigidBodyBatchGpu/4096/128/100/repeats:3_median",
            "median",
            2.0,
            time_unit="ms",
        ),
    ]

    summary = phase5.validate_packet(data)

    assert summary["cpu_median_ns"] == 4_000_000.0
    assert summary["gpu_median_ns"] == 2_000_000.0
    assert summary["speedup"] == 2.0


def test_phase5_cuda_packet_writer_uses_shared_canonical_rows():
    writer = load_script_module("write_phase5_cuda_packet")
    rows = [
        aggregate_row(
            "BM_Phase5RigidBodyBatchGpu/4096/128/100/repeats:3_mean",
            "mean",
            1.0,
        ),
        aggregate_row(
            "BM_Phase5RigidBodyBatchGpu/4096/128/100/repeats:3_median",
            "median",
            1.0,
            run_name=False,
        ),
    ]
    rows[0]["max_final_state_abs_error"] = 1e-9
    rows[1]["max_final_state_abs_error"] = 1e-12

    max_error = writer.extract_max_final_state_abs_error(
        rows,
        gpu_prefix="BM_Phase5RigidBodyBatchGpu",
        world_count=4096,
        body_count=128,
        step_count=100,
    )

    assert max_error == 1e-12


def test_abd_packet_validator_uses_shared_row_validation(tmp_path, capsys):
    abd = load_script_module("check_abd_comparison_packet")
    packet = {
        "benchmarks": [
            aggregate_row(f"{name}_median", "median", 1.0)
            for name in sorted(abd.EXPECTED_BENCHMARKS)
        ]
    }
    path = tmp_path / "abd_packet.json"
    path.write_text(json.dumps(packet), encoding="utf-8")

    abd.validate_packet(path)

    assert "ABD comparison packet OK" in capsys.readouterr().out
