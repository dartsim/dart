import importlib.util
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "preview_performance_dashboard.py"
FIXTURE = (
    ROOT
    / "tests"
    / "fixtures"
    / "performance_dashboard"
    / "google_benchmark_sample.json"
)


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "preview_performance_dashboard", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _read_data(data_js: Path) -> dict:
    match = re.search(
        r"window\.BENCHMARK_DATA\s*=\s*(\{.*\})\s*;",
        data_js.read_text(encoding="utf-8"),
        re.S,
    )
    assert match is not None
    return json.loads(match.group(1))


def test_preview_writes_index_and_data(tmp_path):
    module = _load_module()
    rc = module.main(
        [
            str(FIXTURE),
            "--output-dir",
            str(tmp_path),
            "--suite-name",
            "DART 6 Performance",
        ]
    )

    assert rc == 0
    assert (tmp_path / "index.html").is_file()
    data = _read_data(tmp_path / "data.js")
    runs = data["entries"]["DART 6 Performance"]
    assert len(runs) == 1
    assert runs[0]["tool"] == "googlecpp"
    bench = runs[0]["benches"][0]
    assert {"name", "value", "unit", "family"} <= bench.keys()


def test_preview_append_accumulates_history(tmp_path):
    module = _load_module()
    module.main([str(FIXTURE), "--output-dir", str(tmp_path), "--commit-id", "aaa"])
    module.main(
        [str(FIXTURE), "--output-dir", str(tmp_path), "--commit-id", "bbb", "--append"]
    )

    data = _read_data(tmp_path / "data.js")
    runs = data["entries"]["DART 6 Performance"]
    assert [run["commit"]["id"] for run in runs] == ["aaa", "bbb"]
