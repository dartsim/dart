#!/usr/bin/env python3
"""
Generate Python stub files (.pyi) for dartpy module using nanobind.stubgen.

This script generates type stubs that can be used for:
1. IDE autocompletion and type checking
2. Documentation generation with Sphinx autodoc (on Read the Docs)

Usage:
    python scripts/generate_stubs.py
"""

import ast
import os
import subprocess
import sys
import tempfile
from pathlib import Path

STUB_MODULES = (
    ("dartpy._dartpy.common", Path("dartpy/common.pyi")),
    ("dartpy._dartpy.math", Path("dartpy/math.pyi")),
    ("dartpy._dartpy.dynamics", Path("dartpy/dynamics.pyi")),
    ("dartpy._dartpy.collision", Path("dartpy/collision.pyi")),
    ("dartpy._dartpy.simulation", Path("dartpy/simulation.pyi")),
    ("dartpy._dartpy.constraint", Path("dartpy/constraint.pyi")),
    ("dartpy._dartpy.optimizer", Path("dartpy/optimizer.pyi")),
    (
        "dartpy._dartpy.simulation_experimental",
        Path("dartpy/simulation_experimental.pyi"),
    ),
    ("dartpy._dartpy.gui", Path("dartpy/gui/__init__.pyi")),
    ("dartpy._dartpy.utils", Path("dartpy/utils/__init__.pyi")),
    ("dartpy._dartpy.utils.MjcfParser", Path("dartpy/utils/MjcfParser.pyi")),
    ("dartpy._dartpy.utils.SdfParser", Path("dartpy/utils/SdfParser.pyi")),
    ("dartpy._dartpy.utils.SkelParser", Path("dartpy/utils/SkelParser.pyi")),
)

SUBMODULES = (
    "collision",
    "common",
    "constraint",
    "dynamics",
    "gui",
    "io",
    "math",
    "optimizer",
    "simulation",
    "simulation_experimental",
    "utils",
)

PROMOTED_MODULES = (
    "collision",
    "common",
    "constraint",
    "dynamics",
    "math",
    "optimizer",
    "simulation",
)


def _public_names(source: str) -> list[str]:
    tree = ast.parse(source)
    names = []
    for node in tree.body:
        if isinstance(node, (ast.ClassDef, ast.FunctionDef)):
            names.append(node.name)
        elif isinstance(node, ast.Assign):
            names.extend(
                target.id for target in node.targets if isinstance(target, ast.Name)
            )
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            names.append(node.target.id)
        elif isinstance(node, ast.ImportFrom) and node.level > 0:
            names.extend(alias.asname or alias.name for alias in node.names)

    names = [name for name in names if not name.startswith("_") and name != "*"]
    return sorted(dict.fromkeys(names))


def _all_block(names: list[str]) -> str:
    if not names:
        return "__all__: list[str] = []"

    values = ",\n".join(f'    "{name}"' for name in names)
    return f"__all__: list[str] = [\n{values},\n]"


def _add_future_import(source: str) -> str:
    if source.startswith("from __future__ import annotations\n"):
        return source
    return f"from __future__ import annotations\n\n{source}"


def _insert_all(source: str, names: list[str]) -> str:
    lines = source.splitlines()
    insert_at = 0
    if lines and lines[0] == "from __future__ import annotations":
        insert_at = 1
    return (
        "\n".join(
            lines[:insert_at] + ["", _all_block(names), ""] + lines[insert_at:]
        ).rstrip()
        + "\n"
    )


def _postprocess_stub(source: str) -> str:
    source = _add_future_import(source)
    source = source.replace("dartpy._dartpy.", "dartpy.")
    source = source.replace("dartpy._dartpy", "dartpy")
    return _insert_all(source, _public_names(source))


def _write_top_level_stub(stubs_dir: Path, names_by_module: dict[str, list[str]]):
    dartpy_dir = stubs_dir / "dartpy"
    dartpy_dir.mkdir(parents=True, exist_ok=True)

    lines = [
        '"""',
        "dartpy: Python API of Dynamic Animation and Robotics Toolkit",
        '"""',
        "from __future__ import annotations",
        "",
    ]
    for module in SUBMODULES:
        lines.append(f"from . import {module}")

    promoted_names: list[str] = []
    for module in PROMOTED_MODULES:
        names = names_by_module.get(module, [])
        if not names:
            continue
        lines.extend(["", f"from .{module} import ("])
        lines.extend(f"    {name}," for name in names)
        lines.append(")")
        promoted_names.extend(names)

    all_names = [*SUBMODULES, *promoted_names]
    lines.extend(
        ["", _all_block(sorted(dict.fromkeys(all_names))), "__version__: str = ''"]
    )
    (dartpy_dir / "__init__.pyi").write_text("\n".join(lines) + "\n")


def _write_io_stub(stubs_dir: Path):
    io_dir = stubs_dir / "dartpy" / "io"
    io_dir.mkdir(parents=True, exist_ok=True)
    (io_dir / "__init__.pyi").write_text(
        '"""\n'
        "Alias for ``dartpy.utils`` (preferred parser namespace).\n"
        '"""\n\n'
        "from __future__ import annotations\n\n"
        "from dartpy import utils as _utils\n"
        "from dartpy.utils import *  # noqa: F401,F403\n\n"
        "__all__ = _utils.__all__\n"
    )


def _public_module_name(relative_output: Path) -> str:
    if relative_output.parent == Path("dartpy"):
        return relative_output.stem
    return relative_output.parts[1]


def main():
    # Get the repository root directory
    repo_root = Path(__file__).parent.parent

    # Determine the dartpy module path
    # Check if PYTHONPATH is set (for local builds with compiled module)
    pythonpath = os.environ.get("PYTHONPATH", "")
    if pythonpath:
        for path in reversed(pythonpath.split(os.pathsep)):
            if path:
                sys.path.insert(0, path)

    # Verify dartpy can be imported
    try:
        import dartpy  # type: ignore

        print(f"✓ Found dartpy at: {dartpy.__file__}")
    except ImportError as e:
        print(f"ERROR: Cannot import dartpy: {e}")
        print(
            "Please build dartpy first: pixi run build-py-dev (or ensure dartpy is installed in the current interpreter)"
        )
        sys.exit(1)

    # Output directory for stubs
    stubs_dir = repo_root / "python" / "stubs"
    stubs_dir.mkdir(parents=True, exist_ok=True)

    print(f"Generating stubs in: {stubs_dir}")

    names_by_module: dict[str, list[str]] = {}
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_root = Path(temp_dir)

        for module_name, relative_output in STUB_MODULES:
            temp_output = temp_root / relative_output.name
            cmd = [
                sys.executable,
                "-m",
                "nanobind.stubgen",
                "-m",
                module_name,
                "-o",
                str(temp_output),
                "-D",
                "--exclude-values",
                "-q",
            ]

            try:
                subprocess.run(cmd, check=True, capture_output=True, text=True)
            except subprocess.CalledProcessError as e:
                print(f"ERROR: Failed to generate stub for {module_name}: {e}")
                print(e.stdout)
                print(e.stderr, file=sys.stderr)
                sys.exit(1)

            source = _postprocess_stub(temp_output.read_text())
            output = stubs_dir / relative_output
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(source)

            public_module = _public_module_name(relative_output)
            if public_module != "utils":
                names_by_module[public_module] = _public_names(source)

    _write_top_level_stub(stubs_dir, names_by_module)
    _write_io_stub(stubs_dir)

    # Check that stubs were created
    dartpy_stub = stubs_dir / "dartpy" / "__init__.pyi"
    if dartpy_stub.exists():
        print(f"✓ Verified stub file exists: {dartpy_stub}")
    else:
        print(f"WARNING: Expected stub file not found: {dartpy_stub}")

    print("\nStub generation complete!")
    print(f"Stubs location: {stubs_dir}")


if __name__ == "__main__":
    main()
