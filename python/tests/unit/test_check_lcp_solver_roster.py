from __future__ import annotations

import csv
import importlib.util
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_lcp_solver_roster.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_lcp_solver_roster", SCRIPT)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _valid_evidence_row() -> dict[str, str]:
    return {
        "category": "Standard",
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
        "iterations": "0",
        "residual": "0",
        "complementarity": "0",
        "bound_violation": "0",
        "solver_supports_standard": "1",
        "solver_supports_boxed": "1",
        "solver_supports_friction_index": "1",
        "solver_supports_problem": "1",
        "problem_type_standard": "1",
        "problem_type_boxed": "0",
        "problem_type_friction_index": "0",
        "problem_type_invalid": "0",
    }


def test_lcp_solver_roster_surfaces_match() -> None:
    module = _load_module()

    module.check_roster()


def test_lcp_solver_roster_rejects_demo_profile_column_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_schema = dict(module.parse_demo_profile_evidence_schema())
    stale_columns = tuple(
        column
        for column in module.REQUIRED_EVIDENCE_COLUMNS
        if column != "time_ns"
    )
    stale_schema["_PERFORMANCE_PROFILE_EVIDENCE_REQUIRED_COLUMNS"] = stale_columns
    monkeypatch.setattr(
        module,
        "parse_demo_profile_evidence_schema",
        lambda: stale_schema,
    )

    with pytest.raises(
        AssertionError,
        match="profile evidence required columns do not match",
    ):
        module.check_demo_profile_evidence_required_columns()


def test_lcp_solver_roster_rejects_demo_profile_schema_drift(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    stale_schema = dict(module.parse_demo_profile_evidence_schema())
    stale_schema["_SOLVER_IDENTITY_SCHEMA_VERSION"] = 2
    monkeypatch.setattr(
        module,
        "parse_demo_profile_evidence_schema",
        lambda: stale_schema,
    )

    with pytest.raises(
        AssertionError,
        match="profile evidence schema does not match",
    ):
        module.check_demo_profile_evidence_required_columns()


def test_lcp_solver_roster_reads_demo_benchmark_filter_tokens() -> None:
    module = _load_module()

    tokens = module.parse_demo_benchmark_filter_tokens()

    assert "BM_LCP_COMPARE_SMOKE" in tokens
    assert "BM_LcpCompare/Standard/Dantzig/12" in tokens
    assert "BM_LcpNewtonWarmStart" in tokens
    assert "BM_LcpNewtonWarmStartBatchSerial" in tokens
    assert "BM_LcpNewtonWarmStartBatchParallel" in tokens
    assert "BM_LcpJacobiSolverThreading" in tokens
    assert "BM_LcpRedBlackGaussSeidelSolverThreadingBanded" in tokens
    assert "BM_LcpBlockedJacobiSolverThreadingBanded" in tokens
    assert "BM_LcpMildIllConditioned" in tokens
    assert "BM_LcpMildIllConditionedBatchSerial" in tokens
    assert "BM_LcpMildIllConditionedBatchParallel" in tokens
    assert "BM_LcpNearSingular" in tokens
    assert "BM_LcpNearSingularBatchSerial" in tokens
    assert "BM_LcpNearSingularBatchParallel" in tokens
    assert "BM_LcpValidation_Serial_FrictionIndex" in tokens
    assert "BM_LcpValidation_Threaded_FrictionIndex" in tokens
    assert "BM_LcpWorldSeparatedStep_BoxedLcp" in tokens
    assert "BM_LcpWorldBoxStep_BoxedLcp" in tokens
    assert "BM_LcpWorldBilliardsStep_BoxedLcp" in tokens
    assert "BM_LcpArticulatedUnifiedContact" in tokens
    assert "BM_LcpCudaJacobiBatch" in tokens
    assert "BM_LcpCudaPgsGroupedBatch" in tokens
    assert "BM_LcpCudaJacobiWorldContactBatch" in tokens
    assert "BM_LcpCudaPgsWorldBoxContactGroupedBatch" in tokens
    assert "BM_LcpCudaJacobiArticulatedUnifiedContactGroupedBatch" in tokens
    assert "BM_LcpCudaPgsMixedContactGroupedBatch" in tokens


def test_lcp_solver_roster_rejects_unknown_demo_benchmark_filter(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_demo_benchmark_filter_tokens",
        lambda: ["BM_LcpCompare/Standard", "BM_LcpMissingDemoPacket"],
    )

    with pytest.raises(
        AssertionError,
        match="benchmark filters do not match BM_LCP_COMPARE benchmarks",
    ):
        module.check_demo_benchmark_filters()


def test_lcp_solver_roster_rejects_extra_bound_solver_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_bound_solver_classes",
        lambda: {
            "DantzigSolver": "DantzigSolver",
            "ExtraSolver": "ExtraSolver",
        },
    )

    with pytest.raises(
        AssertionError,
        match="dartpy bindings contain non-manifest LCP solver classes",
    ):
        module.check_bound_solver_classes(["DantzigSolver"])


def test_lcp_solver_roster_rejects_missing_math_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_extra_math_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver", "ExtraSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi contains non-manifest solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_math_all_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(module, "parse_math_stub_all_names", lambda: {"DantzigSolver"})
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/math\\.pyi __all__ is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_init_stub_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/__init__\\.pyi is missing \\.math imports",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_solver_roster_rejects_missing_init_all_class(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    module = _load_module()
    monkeypatch.setattr(
        module,
        "parse_math_stub_solver_classes",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_math_stub_all_names",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_math_imports",
        lambda: {"DantzigSolver", "LemkeSolver"},
    )
    monkeypatch.setattr(
        module,
        "parse_init_stub_all_names",
        lambda: {"DantzigSolver"},
    )

    with pytest.raises(
        AssertionError,
        match="python/stubs/dartpy/__init__\\.pyi __all__ is missing solver classes",
    ):
        module.check_python_stub_solver_classes(["DantzigSolver", "LemkeSolver"])


def test_lcp_profile_evidence_rejects_solver_identity_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"solver_manifest_index": "5"})

    with pytest.raises(AssertionError, match="solver_manifest_index"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_headers_reject_missing_native_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
        module.SolverEntry("Lemke", "Pivoting", True, False, False, "LemkeSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="missing native standard solvers"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_invalid_header(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("scale,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="invalid header"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_unknown_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig,Ghost\n1,1.0,2.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="contains unknown solvers"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_duplicate_solver_column(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text("tau,Dantzig,Dantzig\n1,1.0,1.1\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match="contains duplicate solver columns"):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_headers_reject_non_native_solver(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_boxed.csv"
    path.write_text("tau,Dantzig\n1,1.0\n", encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"boxed": path})

    with pytest.raises(AssertionError, match="contains non-native boxed solvers"):
        module.check_performance_profile_headers(manifest)


@pytest.mark.parametrize(
    ("csv_text", "expected_error"),
    [
        ("tau,Dantzig\n0,1.0\n", "invalid tau"),
        ("tau,Dantzig\n1,1.0\n1,1.0\n", "non-increasing tau"),
        ("tau,Dantzig\n1,1.1\n", "invalid profile value"),
        ("tau,Dantzig\n1\n", "columns; expected 2"),
    ],
)
def test_lcp_profile_headers_reject_invalid_profile_rows(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    csv_text: str,
    expected_error: str,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    path = tmp_path / "performance_profile_standard.csv"
    path.write_text(csv_text, encoding="utf-8")
    monkeypatch.setattr(module, "LCP_PROFILE_CSV_PATHS", {"standard": path})

    with pytest.raises(AssertionError, match=expected_error):
        module.check_performance_profile_headers(manifest)


def test_lcp_profile_evidence_rejects_missing_native_solver(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
        module.SolverEntry("Lemke", "Pivoting", True, False, False, "LemkeSolver"),
    ]
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {
                "solver_supports_boxed": "0",
                "solver_supports_friction_index": "0",
            }
        )

    with pytest.raises(AssertionError, match="missing native profile evidence"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_duplicate_column(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"
    fieldnames = (*module.REQUIRED_EVIDENCE_COLUMNS, "time_ns")

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerow(_valid_evidence_row())

    with pytest.raises(AssertionError, match="contains duplicate evidence columns"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_duplicate_row(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = [
        module.SolverEntry("Dantzig", "Pivoting", True, False, False, "DantzigSolver"),
    ]
    row = _valid_evidence_row() | {
        "solver_supports_boxed": "0",
        "solver_supports_friction_index": "0",
    }
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(row)
        writer.writerow(row)

    with pytest.raises(AssertionError, match="duplicate evidence row"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_invalid_metric(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"bound_violation": ""})

    with pytest.raises(AssertionError, match="bound_violation"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_solver_family_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {"solver_family_pivoting": "0", "solver_family_projection": "1"}
        )

    with pytest.raises(AssertionError, match="solver_family_projection"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_unsupported_solver_category(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(
            _valid_evidence_row()
            | {
                "category": "Boxed",
                "solver": "Lemke",
                "solver_manifest_index": "2",
                "solver_supports_standard": "1",
                "solver_supports_boxed": "0",
                "solver_supports_friction_index": "0",
                "solver_supports_problem": "0",
                "problem_type_standard": "0",
                "problem_type_boxed": "1",
            }
        )

    with pytest.raises(AssertionError, match="not native-supported"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_problem_dimension_mismatch(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"lcp_dimension": "99"})

    with pytest.raises(AssertionError, match="lcp_dimension"):
        module.check_performance_profile_evidence(manifest, path)


def test_lcp_profile_evidence_rejects_nonfinite_integer_counter(
    tmp_path: Path,
) -> None:
    module = _load_module()
    manifest = module.parse_cpp_manifest()
    path = tmp_path / "performance_profile_evidence.csv"

    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=module.REQUIRED_EVIDENCE_COLUMNS)
        writer.writeheader()
        writer.writerow(_valid_evidence_row() | {"problem_size": "inf"})

    with pytest.raises(AssertionError, match="invalid problem_size 'inf'"):
        module.check_performance_profile_evidence(manifest, path)
