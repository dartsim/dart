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
import importlib
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
    ("dartpy._dartpy.gui", Path("dartpy/gui/__init__.pyi")),
    ("dartpy._dartpy.utils", Path("dartpy/utils/__init__.pyi")),
    ("dartpy._dartpy.utils.MjcfParser", Path("dartpy/utils/MjcfParser.pyi")),
    ("dartpy._dartpy.utils.SdfParser", Path("dartpy/utils/SdfParser.pyi")),
    ("dartpy._dartpy.utils.SkelParser", Path("dartpy/utils/SkelParser.pyi")),
)

OPTIONAL_STUB_MODULES = frozenset(
    (
        # simulation (the ECS facade) can be absent when a configure skips the
        # DART 7 World stack because required sibling targets are unavailable;
        # gui is absent in non-GUI builds.
        "dartpy._dartpy.simulation",
        "dartpy._dartpy.gui",
    )
)

SUBMODULES = (
    "collision",
    "common",
    "constraint",
    "diff",
    "dynamics",
    "gui",
    "io",
    "math",
    "optimizer",
    "simulation",
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


def _is_runtime_flat_promoted(name: str) -> bool:
    # Match python/dartpy/_layout.py:_promote_symbols. Legacy lowerCamel
    # compatibility names stay on their submodules; the flat namespace promotes
    # classes, enums/constants, and snake_case helpers.
    return bool(name) and (name[0].isupper() or not any(ch.isupper() for ch in name))


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


def _remove_all_block(source: str) -> str:
    lines = source.splitlines()
    for index, line in enumerate(lines):
        if not line.startswith("__all__: list[str] = ["):
            continue
        end = index + 1
        while end < len(lines) and lines[end] != "]":
            end += 1
        if end < len(lines):
            end += 1
        if end < len(lines) and lines[end] == "":
            end += 1
        return "\n".join(lines[:index] + lines[end:]).rstrip() + "\n"
    return source


def _postprocess_stub(source: str) -> str:
    source = _add_future_import(source)
    source = source.replace("dartpy._dartpy.", "dartpy.")
    source = source.replace("dartpy._dartpy", "dartpy")
    # The diff namespace is a runtime-attached pure-Python module, not a real
    # dartpy.simulation.diff submodule file. Point the simulation stub's diff
    # import at the hand-written top-level dartpy.diff stub so it resolves.
    source = source.replace(
        "import dartpy.simulation.diff as diff", "from dartpy import diff as diff"
    )
    return _insert_all(source, _public_names(source))


def _postprocess_public_module_stub(source: str, public_module: str) -> str:
    if public_module != "gui":
        return source

    helper_names = [
        "DescriptorRenderScene",
        "WorldRenderBridge",
        "look_at",
        "orbit_camera",
        "render",
        "world_render_frame",
    ]
    helper_import = (
        "from dartpy._world_render_bridge import (\n"
        "    DescriptorRenderScene as DescriptorRenderScene,\n"
        "    WorldRenderBridge as WorldRenderBridge,\n"
        "    look_at as look_at,\n"
        "    orbit_camera as orbit_camera,\n"
        "    render as render,\n"
        "    world_render_frame as world_render_frame,\n"
        ")\n"
    )

    source = _remove_all_block(source)
    updated = source
    if "from dartpy._world_render_bridge import" not in updated:
        lines = updated.splitlines()
        insert_at = 0
        while insert_at < len(lines):
            line = lines[insert_at]
            if line.startswith("from ") or line.startswith("import ") or not line:
                insert_at += 1
                continue
            break

        updated = "\n".join(
            lines[:insert_at] + ["", helper_import.rstrip()] + lines[insert_at:]
        )

    names = sorted(dict.fromkeys([*_public_names(updated), *helper_names]))
    return _insert_all(updated, names)


def _requested_module_is_missing(error: ModuleNotFoundError, module_name: str) -> bool:
    missing_name = error.name
    return missing_name == module_name or (
        missing_name is not None and module_name.startswith(f"{missing_name}.")
    )


def _stub_module_available(module_name: str) -> bool:
    if module_name not in OPTIONAL_STUB_MODULES:
        return True

    try:
        importlib.import_module(module_name)
    except ModuleNotFoundError as e:
        if _requested_module_is_missing(e, module_name):
            return False
        raise

    return True


def _remove_stub_output(stubs_dir: Path, relative_output: Path):
    output = stubs_dir / relative_output
    if output.exists():
        output.unlink()


def _write_top_level_stub(
    stubs_dir: Path,
    names_by_module: dict[str, list[str]],
    available_submodules: set[str],
):
    dartpy_dir = stubs_dir / "dartpy"
    dartpy_dir.mkdir(parents=True, exist_ok=True)

    submodules = [module for module in SUBMODULES if module in available_submodules]
    lines = [
        '"""',
        "dartpy: Python API of Dynamic Animation and Robotics Toolkit",
        '"""',
        "from __future__ import annotations",
        "",
    ]
    for module in submodules:
        lines.append(f"from . import {module}")

    promoted_names: list[str] = []
    for module in PROMOTED_MODULES:
        names = [
            name
            for name in names_by_module.get(module, [])
            if _is_runtime_flat_promoted(name)
        ]
        if not names:
            continue
        lines.extend(["", f"from .{module} import ("])
        lines.extend(f"    {name}," for name in names)
        lines.append(")")
        promoted_names.extend(names)

    all_names = [*submodules, *promoted_names]
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


def _write_diff_stub(stubs_dir: Path):
    dartpy_dir = stubs_dir / "dartpy"
    dartpy_dir.mkdir(parents=True, exist_ok=True)

    # DART_BUILD_DIFF=ON also re-exports the framework-neutral C++ rollout API
    # onto the runtime dartpy.diff module (dartpy.__init__ copies it from
    # dartpy.simulation). Mirror those re-exports in the stub when they exist so
    # typed callers and the generated docs see dartpy.diff.rollout /
    # RolloutTrajectory; the default DART_BUILD_DIFF=OFF build emits timestep only.
    reexports: list[str] = []
    try:
        import dartpy  # type: ignore

        diff_module = getattr(dartpy, "diff", None)
        for name in ("RolloutTrajectory", "rollout"):
            if diff_module is not None and hasattr(diff_module, name):
                reexports.append(name)
    except Exception:
        pass

    header = (
        '"""\n'
        "PyTorch autograd bridge for differentiable simulation.\n"
        "\n"
        "Attached at runtime as ``dartpy.diff`` (and ``dartpy.simulation.diff``).\n"
        "Wraps a single differentiable ``World`` step as a\n"
        "``torch.autograd.Function``; torch is imported lazily so importing\n"
        "dartpy stays torch-free.\n"
        '"""\n\n'
        "from __future__ import annotations\n\n"
    )
    body = ""
    if reexports:
        body += "from dartpy.simulation import (\n"
        body += "".join(f"    {name} as {name},\n" for name in sorted(reexports))
        body += ")\n\n"
    body += _all_block(sorted(["timestep", *reexports])) + "\n\n"
    body += "def timestep(world, state, action): ...\n"
    (dartpy_dir / "diff.pyi").write_text(header + body)


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

        available_submodules: set[str] = set()
        for module_name, relative_output in STUB_MODULES:
            if not _stub_module_available(module_name):
                print(f"Skipping optional stub module not available: {module_name}")
                _remove_stub_output(stubs_dir, relative_output)
                continue

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
            public_module = _public_module_name(relative_output)
            source = _postprocess_public_module_stub(source, public_module)
            output = stubs_dir / relative_output
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(source)

            available_submodules.add(public_module)
            if public_module != "utils":
                names_by_module[public_module] = _public_names(source)

    available_submodules.add("io")
    # The diff namespace (dartpy.diff / dartpy.simulation.diff) is attached at
    # runtime by dartpy.__init__ only when the ECS simulation module is present.
    diff_available = "simulation" in available_submodules
    if diff_available:
        available_submodules.add("diff")
    _write_top_level_stub(stubs_dir, names_by_module, available_submodules)
    _write_io_stub(stubs_dir)
    if diff_available:
        _write_diff_stub(stubs_dir)
    else:
        _remove_stub_output(stubs_dir, Path("dartpy/diff.pyi"))

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
