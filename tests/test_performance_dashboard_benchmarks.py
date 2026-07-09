"""Tests for the DART 6 performance dashboard benchmark runner."""

import importlib.util
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_performance_dashboard_benchmarks.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location(
        "run_performance_dashboard_benchmarks", RUNNER
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _contact_container_filter() -> re.Pattern[str]:
    runner = _load_runner()
    specs = [
        spec for spec in runner.BENCHMARK_SPECS if spec.surface == "contact-container"
    ]
    assert len(specs) == 1
    return re.compile(specs[0].benchmark_filter)


def test_contact_container_dashboard_filter_selects_native_rows():
    pattern = _contact_container_filter()

    assert pattern.fullmatch("BM_ContactContainerActive/60/4/1")
    assert pattern.fullmatch("BM_ContactContainerActive/60/4/16")
    assert pattern.fullmatch("BM_ContactContainerActive/120/4/4")
    assert pattern.fullmatch("BM_ContactContainerDeactivation/60/4/16/iterations:1")


def test_contact_container_dashboard_filter_keeps_incumbent_rows():
    pattern = _contact_container_filter()

    assert pattern.fullmatch("BM_ContactContainerActive/60/0/1")
    assert pattern.fullmatch("BM_ContactContainerActive/60/1/16")
    assert pattern.fullmatch("BM_ContactContainerActive/120/2/4")
    assert pattern.fullmatch("BM_ContactContainerActive/120/3/16")
    assert pattern.fullmatch("BM_ContactContainerDeactivation/60/0/16/iterations:1")
