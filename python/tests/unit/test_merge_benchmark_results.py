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
                    {"name": n, "real_time": 1.0, "time_unit": "ns"} for n in names
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
    assert [b["name"] for b in merged["benchmarks"]] == ["BM_A", "BM_B", "BM_C"]


def test_merge_directory_defaults_to_median(tmp_path):
    module = _load_module()
    out = tmp_path / "out" / "combined.json"
    rc = module.main([str(FIXTURE), "--output", str(out)])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    names = [b["name"] for b in data["benchmarks"]]
    # One median series per benchmark, with the _median suffix stripped.
    assert names == [
        "BM_Distance_BoxBox_Native",
        "BM_Distance_BoxBox_Bullet",
        "BM_DartAdapter_CollidePersistentScene/64",
    ]


def test_merge_all_keeps_every_row(tmp_path):
    module = _load_module()
    out = tmp_path / "out" / "combined.json"
    rc = module.main([str(FIXTURE), "--output", str(out), "--aggregate", "all"])

    assert rc == 0
    data = json.loads(out.read_text(encoding="utf-8"))
    assert len(data["benchmarks"]) == 5


def test_merge_empty_raises(tmp_path):
    module = _load_module()
    (tmp_path / "empty.json").write_text(
        json.dumps({"benchmarks": []}), encoding="utf-8"
    )
    with pytest.raises(SystemExit):
        module.merge([tmp_path / "empty.json"])
