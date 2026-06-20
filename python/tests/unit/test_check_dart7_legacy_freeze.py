import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_dart7_legacy_freeze.py"
BASELINE = ROOT / "scripts" / "check_dart7_legacy_freeze_baseline.txt"


def _load_module():
    spec = importlib.util.spec_from_file_location("check_dart7_legacy_freeze", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


def _messages(violations) -> list[str]:
    return [violation.message for violation in violations]


def _write_required_decision_docs(root: Path, plan_text: str | None = None) -> None:
    tag = "dart7-legacy-freeze: bugfix-port"
    _write(
        root / "docs" / "plans" / "042-dart7-public-api-and-source-layout.md",
        plan_text
        or f"""
DART 6 legacy removal staging
completely remove DART 6 legacy from the DART 7 public contract
DART 6.20+ port lane
release-6.* branches remain
{tag}
orphaned cylindrical joint constraint
""",
    )
    _write(
        root
        / "docs"
        / "plans"
        / "042-dart7-public-api-and-source-layout"
        / "post-promotion-source-layout-decision.md",
        f"""
DART 6.20+ port lane
eventual removal from the DART 7 public contract
{tag}
""",
    )
    _write(
        root
        / "docs"
        / "plans"
        / "042-dart7-public-api-and-source-layout"
        / "api-source-layout-audit.md",
        f"""
quarantine now, eventual removal
DART 6.20+ port lane
{tag}
""",
    )


def _baseline_current_tmp_surface(module, tmp_path: Path) -> Path:
    baseline = tmp_path / "baseline.txt"
    module.write_baseline(baseline, module.collect_entries(tmp_path))
    return baseline


def test_current_repo_legacy_freeze_passes():
    module = _load_module()

    assert module.find_violations(ROOT, BASELINE) == []


def test_new_legacy_cpp_symbol_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8")
        + "\nclass DART_API NewLegacyJoint {};\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("New untagged DART 6 legacy public surface" in m for m in messages)
    assert any("NewLegacyJoint" in m for m in messages)


def test_bugfix_port_tag_allows_new_legacy_cpp_symbol(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8")
        + "\n// dart7-legacy-freeze: bugfix-port release-6.* parity\n"
        + "class DART_API BugfixPortJoint {};\n",
        encoding="utf-8",
    )

    assert module.find_violations(tmp_path, baseline) == []


def test_new_legacy_cpp_member_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        """
class DART_API LegacyJoint {
public:
  void oldMethod();
};
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8").replace(
            "  void oldMethod();",
            "  void oldMethod();\n  int newMethod(\n      int value) const;",
        ),
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("LegacyJoint.newMethod" in m for m in messages)


def test_new_legacy_cpp_member_with_next_line_class_brace_requires_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "constraint" / "legacy_constraint.hpp",
        """
class DART_API LegacyConstraint
{
public:
  void oldMethod();
};
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8").replace(
            "  void oldMethod();",
            "  void oldMethod();\n  double newTolerance;",
        ),
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("LegacyConstraint.newTolerance" in m for m in messages)


def test_legacy_cpp_inline_member_body_locals_are_not_public_surface(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        """
class DART_API LegacyJoint {
public:
  int oldMethod() const
  {
    return 0;
  }
};
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8").replace(
            "    return 0;",
            "    const int tmp = 1;\n    return tmp;",
        ),
        encoding="utf-8",
    )

    assert module.find_violations(tmp_path, baseline) == []


def test_private_legacy_cpp_nested_type_is_not_public_surface(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        """
class DART_API LegacyJoint {
public:
  void oldMethod();
};
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8").replace(
            "public:",
            "private:\n  struct Helper {};\n\npublic:",
        ),
        encoding="utf-8",
    )

    assert module.find_violations(tmp_path, baseline) == []


def test_bugfix_port_tag_does_not_allow_adjacent_legacy_symbol(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    header = _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        """
class DART_API LegacyJoint {
public:
};
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    header.write_text(
        header.read_text(encoding="utf-8").replace(
            "public:",
            "public:\n"
            "  // dart7-legacy-freeze: bugfix-port release-6.* parity\n"
            "  void allowedMethod();\n"
            "  void untaggedMethod();",
        ),
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert not any("LegacyJoint.allowedMethod" in m for m in messages)
    assert any("LegacyJoint.untaggedMethod" in m for m in messages)


def test_new_legacy_binding_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)
    _write(
        tmp_path / "python" / "dartpy" / "dynamics" / "new_binding.cpp",
        'void bind(nb::module_& m) { m.def("newLegacyUtility", [] {}); }\n',
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("module.newLegacyUtility" in m for m in messages)


def test_new_legacy_multiline_module_binding_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)
    _write(
        tmp_path / "python" / "dartpy" / "dynamics" / "new_binding.cpp",
        """
void bind(nb::module_& m) {
  nb::class_<ExistingJoint>(m, "ExistingJoint");
  m.def(
      "newLegacyUtility",
      [] {});
}
""",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("module.newLegacyUtility" in m for m in messages)
    assert not any("ExistingJoint.newLegacyUtility" in m for m in messages)


def test_new_legacy_multiline_binding_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)
    _write(
        tmp_path / "python" / "dartpy" / "dynamics" / "new_binding.cpp",
        """
void bind(nb::module_& m) {
  nb::class_<NewLegacyJoint>(
      m, "NewLegacyJoint")
      .def("new_method", [] {});
}
""",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("NewLegacyJoint" in m for m in messages)
    assert any("NewLegacyJoint.new_method" in m for m in messages)


def test_new_legacy_multiline_binding_member_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)
    _write(
        tmp_path / "python" / "dartpy" / "dynamics" / "new_binding.cpp",
        """
void bind(nb::module_& m) {
  nb::class_<NewLegacyJoint>(m, "NewLegacyJoint")
      .def(
          "new_method",
          [] {});
}
""",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("NewLegacyJoint.new_method" in m for m in messages)


def test_new_legacy_enum_binding_and_attr_require_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)
    _write(
        tmp_path / "python" / "dartpy" / "dynamics" / "new_binding.cpp",
        """
void bind(nb::module_& m) {
  nb::enum_<LegacyEnum>(m, "LegacyEnum").value("NEW_VALUE", LegacyEnum::NewValue);
  m.attr("NewLegacyConstant") = 1;
}
""",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("LegacyEnum.NEW_VALUE" in m for m in messages)
    assert any("module.NewLegacyConstant" in m for m in messages)


def test_new_legacy_stub_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "dynamics.pyi",
        "class ExistingJoint:\n    def name(self) -> str: ...\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        stub.read_text(encoding="utf-8")
        + "\nclass NewLegacyJoint:\n    def name(self) -> str: ...\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("NewLegacyJoint" in m for m in messages)


def test_new_root_legacy_reexport_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "__init__.pyi",
        "from .dynamics import (\n    ExistingJoint,\n)\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        "from .dynamics import (\n    ExistingJoint,\n    NewLegacyJoint,\n)\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("dynamics.NewLegacyJoint" in m for m in messages)


def test_new_same_line_root_legacy_reexport_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "__init__.pyi",
        "from .dynamics import ExistingJoint\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        "from .dynamics import ExistingJoint, NewLegacyJoint\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("dynamics.NewLegacyJoint" in m for m in messages)


def test_new_legacy_stub_alias_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "dynamics.pyi",
        """
class ExistingJoint:
    def oldName(self) -> str: ...

    old_name = oldName
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        stub.read_text(encoding="utf-8")
        + "\n    def newName(self) -> str: ...\n\n    new_name = newName\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("ExistingJoint.new_name" in m for m in messages)


def test_tagged_legacy_stub_multiline_def_parameters_are_not_attrs(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "dynamics.pyi",
        """
class ExistingJoint:
    def oldName(self) -> str: ...
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        stub.read_text(encoding="utf-8")
        + "\n"
        + "    # dart7-legacy-freeze: bugfix-port release-6.* parity\n"
        + "    def newName(\n"
        + "        self,\n"
        + "        value: int,\n"
        + "    ) -> str: ...\n",
        encoding="utf-8",
    )

    assert module.find_violations(tmp_path, baseline) == []


def test_new_legacy_stub_enum_value_requires_bugfix_port_tag(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path)
    stub = _write(
        tmp_path / "python" / "stubs" / "dartpy" / "dynamics.pyi",
        """
import enum

class LegacyEnum(enum.Enum):
    OLD_VALUE = 0
""",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    stub.write_text(
        stub.read_text(encoding="utf-8") + "\n    NEW_VALUE = 1\n",
        encoding="utf-8",
    )

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("LegacyEnum.NEW_VALUE" in m for m in messages)


def test_decision_record_must_explain_removal_staging(tmp_path):
    module = _load_module()
    _write_required_decision_docs(tmp_path, plan_text="DART 6.20+ port lane\n")
    _write(
        tmp_path / "dart" / "dynamics" / "legacy_joint.hpp",
        "class DART_API ExistingJoint {};\n",
    )
    baseline = _baseline_current_tmp_surface(module, tmp_path)

    messages = _messages(module.find_violations(tmp_path, baseline))

    assert any("DART 6 legacy removal staging" in m for m in messages)
