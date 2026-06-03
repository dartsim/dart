import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_allocator_comparative_benchmarks.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_allocator_comparative_benchmarks",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _row(name, cpu_time, *, aggregate=True):
    row = {
        "name": name,
        "run_name": name,
        "cpu_time": cpu_time,
        "real_time": cpu_time,
        "time_unit": "ns",
    }
    if aggregate:
        row.update(
            {
                "run_type": "aggregate",
                "aggregate_name": "median",
            }
        )
    return row


def _complete_rows():
    return [
        _row("BM_Pool_DART/32/1024/repeats:5_median", 90.0),
        _row("BM_Pool_Foonathan/32/1024/repeats:5_median", 100.0),
        _row("BM_Pool_StdPmr/32/1024/repeats:5_median", 300.0),
        _row("BM_Stack_DART/256/1024/repeats:5_median", 40.0),
        _row("BM_Stack_Foonathan/256/1024/repeats:5_median", 50.0),
        _row("BM_Stack_StdPmr/256/1024/repeats:5_median", 70.0),
    ]


def test_collect_timings_normalizes_aggregate_names():
    module = _load_module()

    timings = module.collect_timings(_complete_rows())

    assert timings["BM_Pool/32/1024"]["DART"] == 90.0
    assert timings["BM_Pool/32/1024"]["Foonathan"] == 100.0
    assert timings["BM_Stack/256/1024"]["StdPmr"] == 70.0


def test_evaluate_comparisons_accepts_dart_faster_than_foonathan():
    module = _load_module()

    failures, passes = module.evaluate_comparisons(
        _complete_rows(),
        baseline_allocators=["Foonathan"],
    )

    assert failures == []
    assert {row["benchmark"] for row in passes} == {
        "BM_Pool/32/1024",
        "BM_Stack/256/1024",
    }


def test_evaluate_comparisons_rejects_dart_slower_than_foonathan():
    module = _load_module()
    rows = _complete_rows()
    rows[0]["cpu_time"] = 120.0

    failures, passes = module.evaluate_comparisons(
        rows,
        baseline_allocators=["Foonathan"],
    )

    assert len(failures) == 1
    assert failures[0]["benchmark"] == "BM_Pool/32/1024"
    assert failures[0]["baseline"] == "Foonathan"
    assert failures[0]["ratio"] == 1.2
    assert {row["benchmark"] for row in passes} == {"BM_Stack/256/1024"}


def test_evaluate_comparisons_rejects_equal_time_by_default():
    module = _load_module()
    rows = _complete_rows()
    rows[0]["cpu_time"] = 100.0

    failures, _ = module.evaluate_comparisons(
        rows,
        baseline_allocators=["Foonathan"],
    )

    assert any(
        row["benchmark"] == "BM_Pool/32/1024" and row["ratio"] == 1.0
        for row in failures
    )


def test_evaluate_comparisons_rejects_missing_baseline():
    module = _load_module()
    rows = [
        row
        for row in _complete_rows()
        if not row["name"].startswith("BM_Stack_Foonathan")
    ]

    failures, _ = module.evaluate_comparisons(
        rows,
        baseline_allocators=["Foonathan"],
    )

    missing = [
        row for row in failures if row["status"] == "MISSING_BASELINE"
    ]
    assert missing == [
        {
            "benchmark": "BM_Stack/256/1024",
            "baseline": "Foonathan",
            "status": "MISSING_BASELINE",
        }
    ]


def test_evaluate_comparisons_can_check_stdpmr_too():
    module = _load_module()

    failures, passes = module.evaluate_comparisons(
        _complete_rows(),
        baseline_allocators=["Foonathan", "StdPmr"],
    )

    assert failures == []
    assert len(passes) == 4


def test_main_input_path_skips_benchmark_execution(tmp_path, monkeypatch, capsys):
    module = _load_module()
    input_path = tmp_path / "allocator_comparative.json"
    input_path.write_text(
        json.dumps({"benchmarks": _complete_rows()}),
        encoding="utf-8",
    )

    def fail_run_benchmark(*args, **kwargs):
        raise AssertionError("input mode should not run benchmarks")

    monkeypatch.setattr(module, "run_benchmark", fail_run_benchmark)

    assert module.main(["--input", str(input_path)]) == 0
    captured = capsys.readouterr()
    assert "2 comparative allocator checks performed" in captured.out


def test_main_reports_failures(tmp_path, monkeypatch, capsys):
    module = _load_module()
    rows = _complete_rows()
    rows[0]["cpu_time"] = 120.0
    input_path = tmp_path / "allocator_comparative.json"
    input_path.write_text(json.dumps({"benchmarks": rows}), encoding="utf-8")

    def fail_run_benchmark(*args, **kwargs):
        raise AssertionError("input mode should not run benchmarks")

    monkeypatch.setattr(module, "run_benchmark", fail_run_benchmark)

    assert module.main(["--input", str(input_path)]) == 1
    captured = capsys.readouterr()
    assert "COMPARATIVE FAILURES (1)" in captured.out
    assert "BM_Pool/32/1024 vs Foonathan" in captured.out
