from __future__ import annotations

import csv
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
                    "problem_size": 12.0,
                    "iterations": 4.0,
                    "solver_identity_schema_version": 1.0,
                    "solver_manifest_index": 1.0,
                    "solver_family_pivoting": 1.0,
                    "solver_family_projection": 0.0,
                    "solver_family_newton": 0.0,
                    "solver_family_other": 0.0,
                    "bound_violation": 0.25,
                    "solver_supports_problem": 1.0,
                    "solver_supports_standard": 1.0,
                    "solver_supports_boxed": 1.0,
                    "solver_supports_friction_index": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    row = results["Standard"][("Dantzig", 12)]
    assert row["lcp_dimension"] == 12.0
    assert row["contact_count"] is None
    assert row["solver_identity_schema_version"] == 1.0
    assert row["solver_manifest_index"] == 1.0
    assert row["solver_family_pivoting"] == 1.0
    assert row["solver_family_projection"] == 0.0
    assert row["solver_family_newton"] == 0.0
    assert row["solver_family_other"] == 0.0
    assert row["iterations"] == 4.0
    assert row["bound_violation"] == 0.25
    assert row["solver_supports_problem"] == 1.0
    assert row["solver_supports_standard"] == 1.0
    assert row["problem_type_standard"] == 1.0


def test_lcp_profile_parser_rejects_duplicate_result_keys() -> None:
    module = _load_module()
    benchmark = {
        "name": "BM_LcpCompare/Standard/Dantzig/12",
        "run_type": "iteration",
        "cpu_time": 10.0,
        "contract_ok": 1.0,
    }

    with pytest.raises(
        RuntimeError,
        match="duplicate LCP performance profile benchmark row for Standard/Dantzig/12",
    ):
        module.parse_benchmark_results({"benchmarks": [benchmark, benchmark]})


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


def test_lcp_profile_coverage_rejects_failed_contract_rows() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="contract_ok!=1"):
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


def test_lcp_profile_coverage_allows_partial_missing_native_solvers(
    capsys: pytest.CaptureFixture[str],
) -> None:
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
            "Standard": {"Dantzig", "Lemke"},
            "Boxed": set(),
            "FrictionIndex": set(),
        },
        allow_partial=True,
    )

    assert "missing native solvers ['Lemke']" in capsys.readouterr().err


def test_lcp_profile_coverage_rejects_invalid_rows_even_when_partial() -> None:
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

    with pytest.raises(RuntimeError, match="evidence is invalid"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig", "Lemke"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
            allow_partial=True,
        )


def test_lcp_profile_coverage_rejects_historical_non_native_rows() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Boxed/Lemke/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="non-native rows"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Lemke"},
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


def test_lcp_profile_coverage_rejects_problem_dimension_counter_mismatches() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "problem_size": 99.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="problem dimension counters"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )


def test_lcp_profile_coverage_rejects_form_support_mismatches() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_supports_standard": 0.0,
                    "solver_supports_boxed": 1.0,
                    "solver_supports_friction_index": 1.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="unsupported form counters"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )


def test_lcp_profile_coverage_rejects_solver_identity_mismatches() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_identity_schema_version": 1.0,
                    "solver_manifest_index": 5.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="solver identity counters"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )


def test_lcp_profile_coverage_rejects_solver_family_mismatches() -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Standard/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "solver_identity_schema_version": 1.0,
                    "solver_manifest_index": 1.0,
                    "solver_family_pivoting": 0.0,
                    "solver_family_projection": 1.0,
                    "solver_family_newton": 0.0,
                    "solver_family_other": 0.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 1.0,
                    "problem_type_boxed": 0.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    with pytest.raises(RuntimeError, match="solver family counters"):
        module.check_native_profile_coverage(
            results,
            {
                "Standard": {"Dantzig"},
                "Boxed": set(),
                "FrictionIndex": set(),
            },
        )


def test_lcp_profile_evidence_constants_match_roster_schema() -> None:
    module = _load_module()
    roster = sys.modules["check_lcp_solver_roster"]

    assert module.PROFILE_CATEGORIES == roster.PROFILE_CATEGORIES
    assert module.SOLVER_IDENTITY_SCHEMA_VERSION == (
        roster.SOLVER_IDENTITY_SCHEMA_VERSION
    )
    assert module.SOLVER_IDENTITY_COUNTERS == roster.SOLVER_IDENTITY_COUNTERS
    assert module.SOLVER_FAMILY_COUNTER_BY_FAMILY == (
        roster.SOLVER_FAMILY_COUNTER_BY_FAMILY
    )
    assert module.SOLVER_FAMILY_COUNTERS == roster.SOLVER_FAMILY_COUNTERS
    assert module.FORM_SUPPORT_COUNTER_BY_CATEGORY == (
        roster.FORM_SUPPORT_COUNTER_BY_CATEGORY
    )
    assert module.FORM_SUPPORT_COUNTERS == roster.FORM_SUPPORT_COUNTERS
    assert module.PROBLEM_TYPE_COUNTER_BY_CATEGORY == (
        roster.PROBLEM_TYPE_COUNTER_BY_CATEGORY
    )
    assert module.PROBLEM_TYPE_COUNTERS == roster.PROBLEM_TYPE_COUNTERS
    assert module.REQUIRED_EVIDENCE_COLUMNS == roster.REQUIRED_EVIDENCE_COLUMNS


def test_lcp_profile_evidence_csv_header_matches_roster_schema(
    tmp_path: Path,
) -> None:
    module = _load_module()
    path = tmp_path / "profile_evidence.csv"

    module.save_profile_evidence_csv({}, path)

    with path.open(newline="") as f:
        header = next(csv.reader(f))

    assert tuple(header) == module.REQUIRED_EVIDENCE_COLUMNS


def test_lcp_profile_evidence_csv_rejects_missing_current_schema_fields(
    tmp_path: Path,
) -> None:
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

    with pytest.raises(
        RuntimeError,
        match=(
            "missing current-schema fields for Standard/Dantzig/12: "
            "\\['lcp_dimension'"
        ),
    ):
        module.save_profile_evidence_csv(results, tmp_path / "profile_evidence.csv")


def test_lcp_profile_evidence_csv_records_support_and_problem_type(
    tmp_path: Path,
) -> None:
    module = _load_module()
    results = module.parse_benchmark_results(
        {
            "benchmarks": [
                {
                    "name": "BM_LcpCompare/Boxed/Dantzig/12",
                    "run_type": "iteration",
                    "cpu_time": 10.0,
                    "contract_ok": 1.0,
                    "problem_size": 12.0,
                    "contact_count": None,
                    "iterations": 4.0,
                    "residual": 2.0,
                    "complementarity": 3.0,
                    "bound_violation": 0.25,
                    "solver_identity_schema_version": 1.0,
                    "solver_manifest_index": 1.0,
                    "solver_family_pivoting": 1.0,
                    "solver_family_projection": 0.0,
                    "solver_family_newton": 0.0,
                    "solver_family_other": 0.0,
                    "solver_supports_standard": 1.0,
                    "solver_supports_boxed": 1.0,
                    "solver_supports_friction_index": 1.0,
                    "solver_supports_problem": 1.0,
                    "problem_type_standard": 0.0,
                    "problem_type_boxed": 1.0,
                    "problem_type_friction_index": 0.0,
                    "problem_type_invalid": 0.0,
                }
            ]
        }
    )

    path = tmp_path / "profile_evidence.csv"
    module.save_profile_evidence_csv(results, path)

    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    assert rows == [
        {
            "category": "Boxed",
            "solver": "Dantzig",
            "problem_size": "12",
            "lcp_dimension": "12",
            "contact_count": "",
            "solver_identity_schema_version": "1",
            "solver_manifest_index": "1",
            "solver_family_pivoting": "1",
            "solver_family_projection": "0",
            "solver_family_newton": "0",
            "solver_family_other": "0",
            "time_ns": "10",
            "contract_ok": "1",
            "iterations": "4",
            "residual": "2",
            "complementarity": "3",
            "bound_violation": "0.25",
            "solver_supports_standard": "1",
            "solver_supports_boxed": "1",
            "solver_supports_friction_index": "1",
            "solver_supports_problem": "1",
            "problem_type_standard": "0",
            "problem_type_boxed": "1",
            "problem_type_friction_index": "0",
            "problem_type_invalid": "0",
        }
    ]
