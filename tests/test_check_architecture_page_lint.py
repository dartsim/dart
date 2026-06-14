"""Tests for scripts/check_architecture_page_lint.py (PLAN-091 WP-091.3)."""

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_architecture_page_lint.py"
HEADER_ROOT = ROOT / "dart"
REAL_DOC = ROOT / "docs" / "readthedocs" / "architecture.md"
# A test file that genuinely exists, for use in synthetic "valid" rows.
REAL_TEST = "tests/unit/simulation/world/test_world.cpp"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_architecture_page_lint", SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_HEADER = (
    "| Domain | Status | Public entry point | Owner |\n"
    "| ------ | ------ | ------------------ | ----- |\n"
)


def _write_doc(tmp_path, row):
    path = tmp_path / "architecture.md"
    path.write_text(_HEADER + row + "\n", encoding="utf-8")
    return path


def test_real_doc_passes():
    module = _load_module()
    assert module.check(REAL_DOC, HEADER_ROOT) == []


def test_real_doc_has_eight_available_rows():
    module = _load_module()
    rows = module.available_rows(REAL_DOC.read_text(encoding="utf-8"))
    assert len(rows) == 8, [lineno for lineno, _ in rows]


def test_valid_synthetic_row_passes(tmp_path):
    module = _load_module()
    doc = _write_doc(
        tmp_path,
        f"| Rigid | ✅ available | `World::addRigidBody` | tested by [t]"
        f"(https://github.com/dartsim/dart/blob/main/{REAL_TEST}) |",
    )
    assert module.check(doc, HEADER_ROOT) == []


def test_missing_test_file_is_rejected(tmp_path):
    module = _load_module()
    doc = _write_doc(
        tmp_path,
        "| Rigid | ✅ available | `World::addRigidBody` | tested by "
        "tests/unit/simulation/world/test_does_not_exist.cpp |",
    )
    errors = module.check(doc, HEADER_ROOT)
    assert any("do not exist" in e for e in errors), errors


def test_unresolvable_symbol_is_rejected(tmp_path):
    module = _load_module()
    doc = _write_doc(
        tmp_path,
        f"| Rigid | ✅ available | `Bogus::definitelyNotARealApiSymbolXyzzy` | "
        f"tested by {REAL_TEST} |",
    )
    errors = module.check(doc, HEADER_ROOT)
    assert any("resolve" in e for e in errors), errors


def test_row_without_test_is_rejected(tmp_path):
    module = _load_module()
    doc = _write_doc(
        tmp_path,
        "| Rigid | ✅ available | `World::addRigidBody` | no test cited here |",
    )
    errors = module.check(doc, HEADER_ROOT)
    assert any("cites no test" in e for e in errors), errors


def test_row_without_symbol_is_rejected(tmp_path):
    module = _load_module()
    doc = _write_doc(
        tmp_path,
        f"| Rigid | ✅ available | plain prose, no symbol | tested by {REAL_TEST} |",
    )
    errors = module.check(doc, HEADER_ROOT)
    assert any("no public-header symbol" in e for e in errors), errors


def test_no_available_rows_is_rejected(tmp_path):
    module = _load_module()
    path = tmp_path / "architecture.md"
    path.write_text(_HEADER + "| Rigid | 🧪 experimental | x | y |\n", encoding="utf-8")
    errors = module.check(path, HEADER_ROOT)
    assert any("no ✅-available rows" in e for e in errors), errors
