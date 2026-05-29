import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "benchmark_pr_compare.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("benchmark_pr_compare", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(tmp_path, name, text):
    p = tmp_path / name
    p.write_text(text, encoding="utf-8")
    return p


def test_parse_current_normalizes_units(tmp_path):
    m = _load_module()
    cur = _write(
        tmp_path,
        "combined.json",
        json.dumps(
            {
                "benchmarks": [
                    {"name": "BM_A", "real_time": 1000.0, "time_unit": "ns"},
                    {"name": "BM_B", "real_time": 2.0, "time_unit": "ms"},
                ]
            }
        ),
    )
    parsed = m.parse_current(cur)
    assert parsed["BM_A"] == 1000.0 * 1e-9
    assert parsed["BM_B"] == 2.0 * 1e-3


def test_parse_baseline_reads_latest_entry(tmp_path):
    m = _load_module()
    data_js = _write(
        tmp_path,
        "data.js",
        "window.BENCHMARK_DATA = "
        + json.dumps(
            {
                "entries": {
                    "DART": [
                        {"benches": [{"name": "BM_A", "value": 1.0, "unit": "ns"}]},
                        {"benches": [{"name": "BM_A", "value": 900.0, "unit": "ns"}]},
                    ]
                }
            }
        )
        + ";",
    )
    parsed = m.parse_baseline(data_js, "DART")
    # Latest entry wins (900 ns), not the first (1 ns).
    assert parsed["BM_A"] == 900.0 * 1e-9


def test_render_flags_regression_above_threshold():
    m = _load_module()
    out = m.render_comment(
        {"BM_A": 1500e-9},
        {"BM_A": 1000e-9},
        series="DART",
        alert_ratio=1.5,
        baseline_available=True,
    )
    assert m.COMMENT_MARKER in out
    assert "⚠️" in out and "🔴" in out
    assert "+50.0%" in out


def test_render_marks_improvement_and_new():
    m = _load_module()
    out = m.render_comment(
        {"BM_A": 800e-9, "BM_NEW": 100e-9},
        {"BM_A": 1000e-9},
        series="DART",
        alert_ratio=1.5,
        baseline_available=True,
    )
    assert "-20.0%" in out  # improvement
    assert "🆕" in out  # new benchmark not in baseline
    assert "✅" in out  # no regression past threshold


def test_render_handles_missing_baseline():
    m = _load_module()
    out = m.render_comment(
        {"BM_A": 1000e-9},
        {},
        series="DART",
        alert_ratio=1.5,
        baseline_available=False,
    )
    assert "No published baseline" in out
    assert m.COMMENT_MARKER in out


def test_render_flags_subalert_regression_and_noise_band():
    m = _load_module()
    out = m.render_comment(
        {"BM_SLOW": 1200e-9, "BM_FLAT": 1003e-9},
        {"BM_SLOW": 1000e-9, "BM_FLAT": 1000e-9},
        series="DART",
        alert_ratio=1.5,
        baseline_available=True,
    )
    # +20% is a regression but below the 50% alert threshold -> yellow, not red.
    assert "🟡" in out and "🔴" not in out
    assert "✅" in out  # nothing crossed the alert threshold
    # +0.3% sits in the noise band.
    assert "➖" in out


def test_render_unit_mismatch_baseline_vs_current():
    m = _load_module()
    # Baseline in us, current in ns: 1 us baseline vs 2000 ns current = 2x slower.
    out = m.render_comment(
        {"BM_A": 2000e-9},
        {"BM_A": 1e-6},
        series="DART",
        alert_ratio=1.5,
        baseline_available=True,
    )
    assert "+100.0%" in out
    assert "🔴" in out
