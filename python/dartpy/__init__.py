"""
Python package entry point for dartpy.

This module re-exports the pybind11 extension located in
``dartpy/dartpy.cpython-*.so`` so users can simply ``import dartpy``.
It also exposes the installed package version via ``dartpy.__version__`` so
wheel tests and downstream tooling can confirm compatibility.
"""

from __future__ import annotations

import importlib
import os
import sys
import types
from pathlib import Path

from importlib import machinery, metadata


def _candidate_runtime_dirs() -> list[Path]:
    """Collect possible locations of the freshly built dartpy extension."""

    candidates: list[Path] = []
    seen: set[Path] = set()

    def _add(path: Path) -> None:
        resolved = path.resolve()
        if resolved in seen:
            return
        seen.add(resolved)
        candidates.append(resolved)

    runtime_hint = os.environ.get("DARTPY_RUNTIME_DIR")
    if runtime_hint:
        _add(Path(runtime_hint))

    extra_paths = os.environ.get("PYTHONPATH")
    if extra_paths:
        for entry in extra_paths.split(os.pathsep):
            entry = entry.strip()
            if not entry:
                continue
            path = Path(entry)
            _add(path if path.name == "dartpy" else path / "dartpy")

    repo_root = Path(__file__).resolve().parents[2]
    pixi_env = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_type = (
        os.environ.get("BUILD_TYPE")
        or os.environ.get("CMAKE_BUILD_TYPE")
        or "Release"
    )
    _add(repo_root / "build" / pixi_env / "cpp" / build_type / "python" / "dartpy")

    return candidates


def _has_native_extension(runtime_dir: Path) -> bool:
    """Check whether dartpy's compiled module exists in the directory."""

    for suffix in machinery.EXTENSION_SUFFIXES:
        pattern = f"dartpy*{suffix}"
        if any(runtime_dir.glob(pattern)):
            return True
    return False


def _extend_package_path() -> None:
    """Ensure the package path points at the compiled bindings."""

    current_dir = Path(__file__).resolve().parent

    for runtime_dir in _candidate_runtime_dirs():
        runtime_dir = runtime_dir.resolve()
        if not runtime_dir.is_dir():
            continue
        if not _has_native_extension(runtime_dir):
            continue

        # If the bindings live next to __init__.py (installed wheels/conda
        # packages) we can stop early.
        if runtime_dir == current_dir:
            return

        runtime_str = str(runtime_dir)
        parent_str = str(runtime_dir.parent)

        if runtime_str not in __path__:
            __path__.insert(0, runtime_str)
        if parent_str not in sys.path:
            sys.path.insert(0, parent_str)

        # Prevent Python from treating the source tree directories (which only
        # contain C++ sources) as namespace packages.
        current_str = str(current_dir)
        while current_str in __path__:
            __path__.remove(current_str)

        return


def _alias_submodules(bindings: types.ModuleType) -> None:
    """Expose pybind-defined submodules as `dartpy.<name>` imports."""

    package_name = __name__
    prefix = bindings.__name__
    if prefix == package_name:
        return

    for attr in dir(bindings):
        value = getattr(bindings, attr)
        if not isinstance(value, types.ModuleType):
            continue
        module_name = value.__name__
        if not module_name.startswith(prefix + "."):
            continue

        alias = package_name + module_name[len(prefix) :]
        sys.modules.setdefault(alias, value)


try:
    __version__ = metadata.version("dartpy")
except metadata.PackageNotFoundError:  # pragma: no cover
    __version__ = ""

_extend_package_path()
_bindings = importlib.import_module(f"{__name__}.dartpy")
_alias_submodules(_bindings)
from .dartpy import *  # type: ignore  # noqa: F401,F403
