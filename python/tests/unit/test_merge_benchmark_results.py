import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "merge_benchmark_results.py"
FIXTURE = (
    ROOT
    / "tests"
    / "fixtures"
    / "performance_dashboard"
    / "google_benchmark_sample.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location("merge_benchmark_results", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, names: list[str]) -> None:
    path.write_text(
        json.dumps(
            {
                "context": {"host_name": path.stem},
                "benchmarks": [
                    {"name": name, "real_time": 1.0, "time_unit": "ns"}
                    for name in names
                ],
            }
        ),
        encoding="utf-8",
    )


def test_merge_concatenates_benchmarks_and_keeps_first_context(tmp_path):
    module = _load_module()
    _write(tmp_path / "a.json", ["BM_A"])
    _write(tmp_path / "b.json", ["BM_B", "BM_C"])

    merged = module.merge([tmp_path])

    assert merged["context"]["host_name"] == "a"
    assert [row["name"] for row in merged["benchmarks"]] == [
        "BM_A",
        "BM_B",
        "BM_C",
    ]


def test_merge_directory_defaults_to_median(tmp_path):
    module = _load_module()
    out = tmp_path / "out" / "combined.json"
    rc = module.main([str(FIXTURE), "--output", str(out)])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    names = [row["name"] for row in data["benchmarks"]]
    assert names == [
        "BM_Kinematics/10",
        "BM_ContactInverseDynamics/4",
        "BM_ContactContainerActive/60/0/1",
    ]


def test_merge_all_keeps_every_row(tmp_path):
    module = _load_module()
    out = tmp_path / "out" / "combined.json"
    rc = module.main([str(FIXTURE), "--output", str(out), "--aggregate", "all"])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert len(data["benchmarks"]) == 4


def test_merge_empty_raises(tmp_path):
    module = _load_module()
    (tmp_path / "empty.json").write_text(
        json.dumps({"benchmarks": []}), encoding="utf-8"
    )

    with pytest.raises(SystemExit):
        module.merge([tmp_path / "empty.json"])


def test_merge_humanize_rewrites_names(tmp_path):
    module = _load_module()
    _write(
        tmp_path / "a.json",
        ["BM_Kinematics/10", "BM_ContactContainerActive/60/0/1"],
    )

    merged = module.merge([tmp_path], humanize=True)

    names = [row["name"] for row in merged["benchmarks"]]
    assert names == [
        "Skel kinematics update corpus - 10 iterations",
        "Contact container active step - 60 objects - 0 engine - 1 threads",
    ]
