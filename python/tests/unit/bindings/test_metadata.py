"""Metadata and package layout exposed by the built Python module."""

import importlib
import inspect
import sys
import types
from pathlib import Path

import dartpy as dart


def test_module_version_is_nonempty():
    assert isinstance(dart.__version__, str)
    assert dart.__version__


def test_package_wraps_a_private_extension():
    package_dir = Path(dart.__file__).resolve().parent
    assert Path(dart.__file__).name == "__init__.py"
    assert Path(dart._core.__file__).resolve().parent == package_dir
    assert (package_dir / "py.typed").is_file()
    experimental = importlib.import_module("dartpy.experimental")
    assert "outside DART's semantic versioning" in experimental.__doc__


def _public_modules():
    modules = {}
    pending = [dart]
    while pending:
        module = pending.pop()
        if module.__name__ in modules:
            continue
        modules[module.__name__] = module
        for name, value in vars(module).items():
            if (
                not name.startswith("_")
                and isinstance(value, types.ModuleType)
                and value.__name__.startswith("dartpy.")
            ):
                pending.append(value)
    return modules


def test_public_submodules_are_registered_under_their_public_names():
    modules = _public_modules()
    assert {"dartpy.dynamics", "dartpy.simulation", "dartpy.math"} <= modules.keys()
    for name, module in modules.items():
        assert not any(part.startswith("_") for part in name.split(".")), name
        assert sys.modules[name] is module
        assert importlib.import_module(name) is module


def test_public_objects_keep_a_canonical_public_module():
    checked = 0
    for module_name, module in _public_modules().items():
        for name, value in vars(module).items():
            if name.startswith("_") or isinstance(value, types.ModuleType):
                continue
            if not (inspect.isclass(value) or callable(value)):
                continue
            owner = value.__module__
            assert owner.startswith("dartpy"), f"{module_name}.{name}: {owner}"
            assert not any(part.startswith("_") for part in owner.split(".")), (
                f"{module_name}.{name} leaks private module {owner}"
            )
            canonical = sys.modules[owner]
            assert hasattr(canonical, value.__qualname__.partition(".")[0]), (
                f"{module_name}.{name} is not reachable at {owner}.{value.__qualname__}"
            )
            checked += 1
    assert checked > 100
