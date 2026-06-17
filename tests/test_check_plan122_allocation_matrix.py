"""Tests for scripts/check_plan122_allocation_matrix.py."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_plan122_allocation_matrix.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_plan122_allocation_matrix", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_HEADER = (
    "| Row | Status | Surface | Current evidence | Remaining task | Owner |\n"
    "| --- | ------ | ------- | ---------------- | -------------- | ----- |\n"
)


def _write_matrix(tmp_path, *rows):
    path = tmp_path / "coverage-matrix.md"
    path.write_text(_HEADER + "\n".join(rows) + "\n", encoding="utf-8")
    return path


def _write_test_root(tmp_path):
    test_root = tmp_path / "tests"
    test_root.mkdir()
    (test_root / "test_world.cpp").write_text(
        "TEST(World, ExistingAllocationGate) {}\n", encoding="utf-8"
    )
    return test_root


def test_committed_plan122_matrix_passes():
    module = _load_module()
    assert module.main([]) == 0


def test_closed_row_with_existing_test_passes(tmp_path):
    module = _load_module()
    matrix = _write_matrix(
        tmp_path,
        "| H-001 | Closed | Harness | `ExistingAllocationGate` covers it. | None | WP |",
    )
    assert module.validate_matrix(matrix, _write_test_root(tmp_path)) == []


def test_closed_row_with_missing_test_is_rejected(tmp_path):
    module = _load_module()
    matrix = _write_matrix(
        tmp_path,
        "| H-001 | Closed | Harness | `MissingAllocationGate` covers it. | None | WP |",
    )
    errors = module.validate_matrix(matrix, _write_test_root(tmp_path))
    assert any("missing test" in error for error in errors), errors


def test_closed_row_without_test_citation_is_rejected(tmp_path):
    module = _load_module()
    matrix = _write_matrix(
        tmp_path,
        "| H-001 | Closed | Harness | `scripts/check_plan122_allocation_matrix.py` | None | WP |",
    )
    errors = module.validate_matrix(matrix, _write_test_root(tmp_path))
    assert any("cites no test" in error for error in errors), errors


def test_invalid_status_is_rejected(tmp_path):
    module = _load_module()
    matrix = _write_matrix(
        tmp_path,
        "| H-001 | Maybe | Harness | Open. | Fix it. | WP |",
    )
    errors = module.validate_matrix(matrix, _write_test_root(tmp_path))
    assert any("invalid status" in error for error in errors), errors


def test_duplicate_row_id_is_rejected(tmp_path):
    module = _load_module()
    matrix = _write_matrix(
        tmp_path,
        "| H-001 | Open | Harness | Open. | Fix it. | WP |",
        "| H-001 | Open | Harness | Open. | Fix it. | WP |",
    )
    errors = module.validate_matrix(matrix, _write_test_root(tmp_path))
    assert any("duplicate row H-001" in error for error in errors), errors
