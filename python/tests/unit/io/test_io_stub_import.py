"""Ensure the sanitized dartpy.io stub can be imported by autodoc."""

from __future__ import annotations

import ast
import importlib.util
import sys
import types
from pathlib import Path

import pytest


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _load_generate_stubs_module():
    script_path = _repo_root() / "scripts" / "generate_stubs.py"
    spec = importlib.util.spec_from_file_location("generate_stubs", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _stub_all(relative_path: str) -> set[str]:
    stub_path = _repo_root() / "python" / "stubs" / relative_path
    tree = ast.parse(stub_path.read_text(encoding="utf-8"))

    for node in tree.body:
        value = None
        if (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "__all__"
        ):
            value = node.value
        elif isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "__all__"
            for target in node.targets
        ):
            value = node.value

        if value is not None:
            return set(ast.literal_eval(value))

    raise AssertionError(f"{stub_path} does not define literal __all__")


def test_io_stub_sanitized_executes(monkeypatch):
    """Exec the sanitized stub to catch missing symbols such as Options aliases."""

    monkeypatch.setenv("DART_DOCS_SKIP_DARTPY_AUTODOC", "1")
    repo_root = _repo_root()
    stub_path = repo_root / "python" / "stubs" / "dartpy" / "io" / "__init__.pyi"
    conf_path = repo_root / "docs" / "readthedocs" / "conf.py"
    spec = importlib.util.spec_from_file_location(
        "docs_readthedocs_conf", conf_path
    )
    assert spec is not None and spec.loader is not None
    conf_module = importlib.util.module_from_spec(spec)
    sys_path_before = list(sys.path)
    try:
        spec.loader.exec_module(conf_module)
    finally:
        sys.path[:] = sys_path_before
    sanitized = conf_module._sanitize_stub_source(stub_path.read_text())

    # Provide minimal placeholder modules that the stub expects to import.
    modules = {
        "dartpy": types.ModuleType("dartpy"),
        "dartpy.common": types.ModuleType("dartpy.common"),
        "dartpy.dynamics": types.ModuleType("dartpy.dynamics"),
        "dartpy.simulation": types.ModuleType("dartpy.simulation"),
        "dartpy.io": types.ModuleType("dartpy.io"),
        "dartpy.io.MjcfParser": types.ModuleType("dartpy.io.MjcfParser"),
        "dartpy.io.SdfParser": types.ModuleType("dartpy.io.SdfParser"),
    }
    modules["dartpy"].common = modules["dartpy.common"]
    modules["dartpy"].dynamics = modules["dartpy.dynamics"]
    modules["dartpy"].simulation = modules["dartpy.simulation"]
    modules["dartpy.io"].__package__ = "dartpy.io"
    modules["dartpy.io"].__path__ = []

    for name, mod in modules.items():
        monkeypatch.setitem(sys.modules, name, mod)

    exec(compile(sanitized, str(stub_path), "exec"), modules["dartpy.io"].__dict__)

    parser_cls = modules["dartpy.io"].UrdfParser
    assert parser_cls.Options is modules["dartpy.io"].UrdfParserOptions
    assert parser_cls.RootJointType is modules["dartpy.io"].UrdfParserRootJointType


def test_io_stub_all_preserves_parser_exports():
    public_names = _stub_all("dartpy/io/__init__.pyi")

    for name in (
        "MjcfParser",
        "SdfParser",
        "UrdfParser",
        "UrdfParserOptions",
        "UrdfParserRootJointType",
    ):
        assert name in public_names


def test_top_level_stub_promotes_runtime_symbols():
    public_names = _stub_all("dartpy/__init__.pyi")

    for name in (
        "CollisionDetector",
        "CollisionGroup",
        "ComputeExecutionProfile",
        "ComputeExecutor",
        "ComputeNodeExecutionProfile",
        "ContinuousCollisionResult",
        "get_profile_summary_text",
        "is_profile_enabled",
        "is_text_profile_enabled",
        "mark_profile_frame",
        "ParallelExecutor",
        "reset_profile",
        "SequentialExecutor",
        "Skeleton",
        "World",
    ):
        assert name in public_names

    for name in (
        "getProfileSummaryText",
        "isProfileEnabled",
        "isTextProfileEnabled",
        "markProfileFrame",
        "resetProfile",
    ):
        assert name not in public_names


def test_optional_stub_modules_can_be_absent(monkeypatch, tmp_path):
    generate_stubs = _load_generate_stubs_module()
    optional_module = "dartpy._dartpy.gui"

    def fake_import_module(module_name):
        if module_name == optional_module:
            raise ModuleNotFoundError(
                f"No module named {module_name!r}", name=module_name
            )
        return object()

    monkeypatch.setattr(generate_stubs.importlib, "import_module", fake_import_module)

    assert not generate_stubs._stub_module_available(optional_module)
    assert generate_stubs._stub_module_available("dartpy._dartpy.common")

    stale_output = tmp_path / "dartpy" / "gui" / "__init__.pyi"
    stale_output.parent.mkdir(parents=True)
    stale_output.write_text("stale\n", encoding="utf-8")

    generate_stubs._remove_stub_output(tmp_path, Path("dartpy/gui/__init__.pyi"))
    generate_stubs._write_top_level_stub(
        tmp_path,
        {"collision": ["CollisionDetector", "cast_result", "getResult"]},
        {"collision", "io", "io"},
    )

    top_level_stub = (tmp_path / "dartpy" / "__init__.pyi").read_text(
        encoding="utf-8"
    )

    assert not stale_output.exists()
    assert "from . import collision" in top_level_stub
    assert "CollisionDetector" in top_level_stub
    assert "cast_result" in top_level_stub
    assert "getResult" not in top_level_stub
    assert "from . import gui" not in top_level_stub
    assert "from . import simulation_experimental" not in top_level_stub


def test_optional_stub_module_dependency_failures_are_not_suppressed(monkeypatch):
    generate_stubs = _load_generate_stubs_module()

    def fake_import_module(module_name):
        raise ModuleNotFoundError(
            "No module named 'missing_dependency'", name="missing_dependency"
        )

    monkeypatch.setattr(generate_stubs.importlib, "import_module", fake_import_module)

    with pytest.raises(ModuleNotFoundError, match="missing_dependency"):
        generate_stubs._stub_module_available("dartpy._dartpy.gui")
