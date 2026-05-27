import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_compute_graph_benchmarks.py"

REQUIRED_NAMES = (
    "BM_ComputeGraphBuild/1024/1",
    "BM_ComputeGraphBuild/1024/32",
    "BM_ComputeGraphBuild/4096/64",
    "BM_ComputeGraphSequential/1024/1",
    "BM_ComputeGraphSequential/1024/32",
    "BM_ComputeGraphSequential/4096/64",
    "BM_ComputeGraphParallel/1024/1",
    "BM_ComputeGraphParallel/1024/32",
    "BM_ComputeGraphParallel/4096/64",
    "BM_WorldStepSequential/32/8",
    "BM_WorldStepSequential/128/8",
    "BM_WorldStepSequential/128/32",
    "BM_WorldStepParallel/32/8",
    "BM_WorldStepParallel/128/8",
    "BM_WorldStepParallel/128/32",
    "BM_RigidBodyStepSequential/128",
    "BM_RigidBodyStepSequential/1024",
    "BM_RigidBodyStepSequential/4096",
    "BM_RigidBodyStepParallel/128",
    "BM_RigidBodyStepParallel/1024",
    "BM_RigidBodyStepParallel/4096",
    "BM_ContactShapedSequential/1024/16",
    "BM_ContactShapedSequential/4096/16",
    "BM_ContactShapedSequential/1024/64",
    "BM_ContactShapedParallel/1024/16",
    "BM_ContactShapedParallel/4096/16",
    "BM_ContactShapedParallel/1024/64",
    "BM_ContactIslandShapedSequential/4/512/64",
    "BM_ContactIslandShapedSequential/8/512/64",
    "BM_ContactIslandShapedSequential/16/512/64",
    "BM_ContactIslandShapedParallel/4/512/64",
    "BM_ContactIslandShapedParallel/8/512/64",
    "BM_ContactIslandShapedParallel/16/512/64",
    "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10",
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_compute_graph_benchmarks",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _complete_rows():
    rows = [
        {"name": name, "real_time": index + 1.0}
        for index, name in enumerate(REQUIRED_NAMES)
    ]
    for row in rows:
        if row["name"].startswith("BM_ContactIslandShapedParallel/"):
            row["real_time"] = 1.0
    return rows


def test_validate_benchmark_rows_accepts_required_compute_surfaces():
    module = _load_module()

    summary = module.validate_benchmark_rows(_complete_rows())

    assert summary["row_count"] == len(REQUIRED_NAMES)
    assert summary["ratio_count"] == 15
    ratios = {
        row["benchmark"]: row["parallel_over_sequential"]
        for row in summary["parallel_ratios"]
    }
    assert ratios["BM_ComputeGraph/1024/1"] == 7.0 / 4.0
    assert ratios["BM_WorldStep/32/8"] == 13.0 / 10.0
    assert ratios["BM_RigidBodyStep/128"] == 19.0 / 16.0
    assert ratios["BM_ContactShaped/1024/16"] == 25.0 / 22.0
    assert ratios["BM_ContactIslandShaped/4/512/64"] == 1.0 / 28.0
    assert "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10" not in ratios


def test_validate_benchmark_rows_accepts_aggregate_row_names():
    module = _load_module()
    rows = [
        {
            "name": f"{row['name']}_median",
            "run_name": f"{row['name']}_median",
            "real_time": row["real_time"],
            "run_type": "aggregate",
            "aggregate_name": "median",
        }
        for row in _complete_rows()
    ]

    summary = module.validate_benchmark_rows(rows)

    assert summary["row_count"] == len(REQUIRED_NAMES)
    assert summary["ratio_count"] == 15


def test_validate_benchmark_rows_ignores_non_timing_aggregate_rows():
    module = _load_module()
    rows = _complete_rows()
    rows.extend(
        {
            "name": f"{name}_stddev",
            "run_name": f"{name}_stddev",
            "real_time": 0.0,
            "run_type": "aggregate",
            "aggregate_name": "stddev",
        }
        for name in REQUIRED_NAMES
    )

    summary = module.validate_benchmark_rows(rows)

    assert summary["row_count"] == len(REQUIRED_NAMES)


def test_validate_benchmark_rows_rejects_missing_required_surface():
    module = _load_module()
    rows = [
        row
        for row in _complete_rows()
        if row["name"] != "BM_ContactShapedParallel/4096/16"
    ]

    with pytest.raises(
        module.BenchmarkCheckError,
        match="BM_ContactShapedParallel/4096/16",
    ):
        module.validate_benchmark_rows(rows)


def test_validate_benchmark_rows_rejects_nonpositive_timing():
    module = _load_module()
    rows = _complete_rows()
    rows[3] = {"name": "BM_ComputeGraphSequential/1024/1", "real_time": 0.0}

    with pytest.raises(
        module.BenchmarkCheckError,
        match="invalid timings: BM_ComputeGraphSequential/1024/1",
    ):
        module.validate_benchmark_rows(rows)


def test_validate_benchmark_rows_rejects_nonfinite_timing():
    module = _load_module()
    rows = _complete_rows()
    rows[3] = {"name": "BM_ComputeGraphSequential/1024/1", "real_time": "nan"}

    with pytest.raises(
        module.BenchmarkCheckError,
        match="invalid timings: BM_ComputeGraphSequential/1024/1",
    ):
        module.validate_benchmark_rows(rows)


def test_validate_benchmark_rows_rejects_missing_contact_island_speedup():
    module = _load_module()
    rows = _complete_rows()
    for row in rows:
        if row["name"] == "BM_ContactIslandShapedParallel/16/512/64":
            row["real_time"] = 100.0

    with pytest.raises(
        module.BenchmarkCheckError,
        match="BM_ContactIslandShaped/16/512/64 parallel/sequential",
    ):
        module.validate_benchmark_rows(rows)


def test_validate_benchmark_rows_can_skip_contact_island_speedup_check():
    module = _load_module()
    rows = _complete_rows()
    for row in rows:
        if row["name"] == "BM_ContactIslandShapedParallel/16/512/64":
            row["real_time"] = 100.0

    summary = module.validate_benchmark_rows(
        rows,
        check_contact_island_speedup=False,
    )

    assert summary["row_count"] == len(REQUIRED_NAMES)


def test_main_input_path_skips_benchmark_execution(tmp_path, monkeypatch, capsys):
    module = _load_module()
    input_path = tmp_path / "compute_graph.json"
    input_path.write_text(
        json.dumps({"benchmarks": _complete_rows()}),
        encoding="utf-8",
    )

    def fail_run_benchmark(*args, **kwargs):
        raise AssertionError("input mode should not run benchmarks")

    monkeypatch.setattr(module, "run_benchmark", fail_run_benchmark)

    assert module.main(["--input", str(input_path)]) == 0
    captured = capsys.readouterr()
    assert f"Checked {len(REQUIRED_NAMES)} compute benchmark rows." in captured.out


def test_main_skip_speedup_check_accepts_slow_contact_island_ratio(
    tmp_path,
    monkeypatch,
):
    module = _load_module()
    rows = _complete_rows()
    for row in rows:
        if row["name"] == "BM_ContactIslandShapedParallel/16/512/64":
            row["real_time"] = 100.0
    input_path = tmp_path / "compute_graph.json"
    input_path.write_text(json.dumps({"benchmarks": rows}), encoding="utf-8")

    def fail_run_benchmark(*args, **kwargs):
        raise AssertionError("input mode should not run benchmarks")

    monkeypatch.setattr(module, "run_benchmark", fail_run_benchmark)

    assert (
        module.main(
            [
                "--input",
                str(input_path),
                "--skip-contact-island-speedup-check",
            ]
        )
        == 0
    )
