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
