import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_header_guards.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_header_guards", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_header_guard_check_passes_current_tree():
    module = _load_module()

    assert module.find_violations(ROOT) == []


def test_guard_macro_detects_plain_include_guard():
    module = _load_module()

    macro = module.guard_macro(
        "#ifndef DART_EXAMPLE_WIDGET_HPP_\n"
        "#define DART_EXAMPLE_WIDGET_HPP_\n"
        "class Widget;\n"
        "#endif // DART_EXAMPLE_WIDGET_HPP_\n"
    )

    assert macro == "DART_EXAMPLE_WIDGET_HPP_"


def test_guard_macro_accepts_pragma_once():
    module = _load_module()

    assert module.guard_macro("#pragma once\nclass Widget;\n") is None


def test_guard_macro_ignores_feature_conditional():
    module = _load_module()

    # An #ifndef whose #define names a different macro is configuration
    # logic, not an include guard.
    macro = module.guard_macro(
        "#pragma once\n"
        "#ifndef DART_DISABLE_WIDGET\n"
        "  #define DART_WIDGET_ENABLED 1\n"
        "#endif\n"
    )

    assert macro is None


def test_allowlisted_vendored_trees_are_skipped():
    module = _load_module()

    tracked = module.tracked_headers(ROOT)

    assert all(not rel.startswith("dart/math/lcp/pivoting/dantzig/") for rel in tracked)
    assert all(not rel.startswith("tests/baseline/odelcpsolver/") for rel in tracked)
    assert "dart/dynamics/ikfast.h" not in tracked
    assert all(not rel.endswith("-impl.hpp") for rel in tracked)
    assert all("poison" not in Path(rel).parts for rel in tracked)
