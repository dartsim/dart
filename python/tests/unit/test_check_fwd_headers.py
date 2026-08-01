import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_fwd_headers.py"

LICENSE = "/*\n * Copyright (c) 2011, The DART development contributors\n */\n\n"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_fwd_headers", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(LICENSE + text, encoding="utf-8")


def test_fwd_header_check_passes_current_tree():
    module = _load_module()

    assert module.find_violations(ROOT) == []


def test_strip_noise_drops_macro_bodies():
    module = _load_module()

    # `class X;` inside a macro body is not a real declaration.
    clean = module._strip_noise(
        "#define DART_DECLARE(X) \\\n"
        "  class X;              \\\n"
        "  using X##Ptr = int;\n"
        "namespace dart { namespace common { class Real; } }\n"
    )

    names = {name for _ns, _kind, name in module._namespace_scope_decls(clean)}
    assert names == {"Real"}


def test_namespace_scope_decls_skips_class_nested_declarations():
    module = _load_module()

    clean = module._strip_noise(
        "namespace dart {\n"
        "namespace simulation {\n"
        "class World {\n"
        "private:\n"
        "  struct Impl;\n"  # PIMPL, not a namespace-scope forward declaration
        "};\n"
        "struct WorldOptions;\n"
        "}\n"
        "}\n"
    )

    found = [(ns, name) for ns, _kind, name in module._namespace_scope_decls(clean)]
    assert found == [("dart::simulation", "WorldOptions")]


def test_defined_names_covers_definitions_and_specializations():
    module = _load_module()

    clean = module._strip_noise(
        "template <typename T>\n"
        "struct Contains;\n"
        "template <typename T>\n"
        "struct Contains<T> {\n"
        "};\n"
    )

    assert "Contains" in module._defined_names(clean)


def test_reports_declaration_a_reachable_fwd_header_already_owns(tmp_path):
    module = _load_module()

    _write(
        tmp_path / "dart" / "widget" / "fwd.hpp",
        "#pragma once\nnamespace dart {\nnamespace widget {\nclass Gadget;\n}\n}\n",
    )
    _write(
        tmp_path / "dart" / "widget" / "gadget_user.hpp",
        "#pragma once\nnamespace dart {\nnamespace widget {\nclass Gadget;\n}\n}\n",
    )

    violations = module.find_violations(tmp_path)

    assert [(v.namespace, v.name) for v in violations] == [("dart::widget", "Gadget")]
    assert "dart/widget/fwd.hpp" in violations[0].message


def test_allows_declaration_the_same_file_defines(tmp_path):
    module = _load_module()

    _write(
        tmp_path / "dart" / "widget" / "fwd.hpp",
        "#pragma once\nnamespace dart {\nnamespace widget {\nclass Gadget;\n}\n}\n",
    )
    # Declared before use, then defined in the same file: ordinary in-file
    # ordering, not a cross-header declaration.
    _write(
        tmp_path / "dart" / "widget" / "gadget.hpp",
        "#pragma once\n"
        "namespace dart {\n"
        "namespace widget {\n"
        "class Gadget;\n"
        "class Holder {\n"
        "  Gadget* m_gadget;\n"
        "};\n"
        "class Gadget {\n"
        "};\n"
        "}\n"
        "}\n",
    )

    assert module.find_violations(tmp_path) == []


def test_allows_allowlisted_never_defined_tag_type(tmp_path):
    module = _load_module()

    _write(
        tmp_path / "dart" / "math" / "detail" / "configuration_space.hpp",
        "#pragma once\n"
        "namespace dart {\n"
        "namespace math {\n"
        "namespace detail {\n"
        "template <bool>\n"
        "struct Range;\n"
        "}\n"
        "}\n"
        "}\n",
    )

    assert ("dart::math::detail", "Range") in module.ALLOWLIST
    assert module.find_violations(tmp_path) == []


def test_reports_gui_backend_declaration_outside_backend_fwd(tmp_path):
    module = _load_module()

    _write(
        tmp_path / "dart" / "gui" / "detail" / "viewer.hpp",
        "#pragma once\nnamespace filament {\nclass Engine;\n}\nstruct GLFWwindow;\n",
    )

    violations = module.find_violations(tmp_path)

    assert [(v.namespace, v.name) for v in violations] == [
        ("filament", "Engine"),
        ("(global)", "GLFWwindow"),
    ]
    assert all("backend_fwd.hpp" in v.message for v in violations)
