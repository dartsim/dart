"""Ensure the sanitized dartpy.utils stub can be imported by autodoc."""

from __future__ import annotations

import sys
import types
import importlib.util
from pathlib import Path


def test_utils_stub_sanitized_executes(monkeypatch):
    """Exec the sanitized stub to catch missing symbols such as Options aliases."""

    repo_root = Path(__file__).resolve().parents[4]
    stub_path = repo_root / "python" / "stubs" / "dartpy" / "utils" / "__init__.pyi"
    conf_path = repo_root / "docs" / "readthedocs" / "conf.py"
    spec = importlib.util.spec_from_file_location(
        "docs_readthedocs_conf", conf_path
    )
    assert spec is not None and spec.loader is not None
    conf_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(conf_module)
    sanitized = conf_module._sanitize_stub_source(stub_path.read_text())

    # Provide minimal placeholder modules that the stub expects to import.
    modules = {
        "dartpy": types.ModuleType("dartpy"),
        "dartpy.common": types.ModuleType("dartpy.common"),
        "dartpy.dynamics": types.ModuleType("dartpy.dynamics"),
        "dartpy.simulation": types.ModuleType("dartpy.simulation"),
        "dartpy.utils": types.ModuleType("dartpy.utils"),
        "dartpy.utils.MjcfParser": types.ModuleType("dartpy.utils.MjcfParser"),
        "dartpy.utils.SdfParser": types.ModuleType("dartpy.utils.SdfParser"),
        "dartpy.utils.SkelParser": types.ModuleType("dartpy.utils.SkelParser"),
    }
    modules["dartpy"].common = modules["dartpy.common"]
    modules["dartpy"].dynamics = modules["dartpy.dynamics"]
    modules["dartpy"].simulation = modules["dartpy.simulation"]
    modules["dartpy.utils"].__package__ = "dartpy.utils"
    modules["dartpy.utils"].__path__ = []

    for name, mod in modules.items():
        monkeypatch.setitem(sys.modules, name, mod)

    exec(compile(sanitized, str(stub_path), "exec"), modules["dartpy.utils"].__dict__)

    parser_cls = modules["dartpy.utils"].UrdfParser
    assert parser_cls.Options is modules["dartpy.utils"].UrdfParserOptions
    assert parser_cls.RootJointType is modules["dartpy.utils"].UrdfParserRootJointType
    assert modules["dartpy.utils"].DartLoader is parser_cls
