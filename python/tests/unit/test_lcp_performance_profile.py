from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "lcp_performance_profile.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("lcp_performance_profile", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_lcp_profile_parser_preserves_concrete_support_counter() -> None:
    module = _load_module()

    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    row = results["Standard"][("Dantzig", 12)]
    assert row["solver_supports_problem"] == 1.0
    assert row["problem_type_standard"] == 1.0


def test_lcp_profile_coverage_rejects_current_schema_unsupported_rows() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_supports_problem": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="solver_supports_problem=0"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )


def test_lcp_profile_coverage_accepts_historical_rows_without_support_counter() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                }
            ]
        }
    )

    module.check_native_profile_coverage(
        results,
        {
            "Standard": {"Dantzig"},
            "Boxed": set(),
            "FrictionIndex": set(),
        },
    )


def test_lcp_profile_coverage_rejects_problem_type_name_mismatches() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 0.0,
                    "problem_type_boxed": 1.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="problem_type counters"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )
