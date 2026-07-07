#!/usr/bin/env python3
"""Check the DART 7 dartpy simulation import layout.

This is the PLAN-042 import-layout gate for the Python side of World
promotion. It is mostly static so it can run in lint jobs before a local dartpy
extension has been built. Pass ``--require-runtime`` from a built/test
environment to require the live import checks too.
"""

from __future__ import annotations

import argparse
import ast
import importlib
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Violation:
    path: str
    message: str


def _rel(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return path.as_posix()


def _read(path: Path, root: Path, violations: list[Violation]) -> str:
    if not path.is_file():
        violations.append(Violation(_rel(path, root), "required file is missing"))
        return ""
    return path.read_text(encoding="utf-8")


def _literal_tuple_assignment(text: str, name: str) -> tuple[str, ...] | None:
    tree = ast.parse(text)
    for node in tree.body:
        targets: list[ast.expr] = []
        value: ast.expr | None = None
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
            value = node.value
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
            value = node.value
        if value is None:
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == name for target in targets
        ):
            continue
        if not isinstance(value, ast.Tuple):
            return None
        values: list[str] = []
        for item in value.elts:
            if not isinstance(item, ast.Constant) or not isinstance(item.value, str):
                return None
            values.append(item.value)
        return tuple(values)
    return None


def _literal_all(text: str) -> tuple[str, ...] | None:
    tree = ast.parse(text)
    for node in tree.body:
        value: ast.expr | None = None
        targets: list[ast.expr] = []
        if isinstance(node, ast.Assign):
            targets = list(node.targets)
            value = node.value
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
            value = node.value
        if value is None:
            continue
        if not any(
            isinstance(target, ast.Name) and target.id == "__all__"
            for target in targets
        ):
            continue
        if not isinstance(value, ast.List):
            return None
        values: list[str] = []
        for item in value.elts:
            if not isinstance(item, ast.Constant) or not isinstance(item.value, str):
                return None
            values.append(item.value)
        return tuple(values)
    return None


def _require_contains(
    violations: list[Violation],
    path: Path,
    root: Path,
    text: str,
    needle: str,
    message: str,
) -> None:
    if needle not in text:
        violations.append(Violation(_rel(path, root), message))


def _require_absent(
    violations: list[Violation],
    path: Path,
    root: Path,
    text: str,
    needle: str,
    message: str,
) -> None:
    if needle in text:
        violations.append(Violation(_rel(path, root), message))


def _require_regex(
    violations: list[Violation],
    path: Path,
    root: Path,
    text: str,
    pattern: str,
    message: str,
) -> None:
    if not re.search(pattern, text, re.MULTILINE | re.DOTALL):
        violations.append(Violation(_rel(path, root), message))


def _require_no_regex(
    violations: list[Violation],
    path: Path,
    root: Path,
    text: str,
    pattern: str,
    message: str,
) -> None:
    if re.search(pattern, text, re.MULTILINE | re.DOTALL):
        violations.append(Violation(_rel(path, root), message))


def find_static_violations(root: Path = REPO_ROOT) -> list[Violation]:
    violations: list[Violation] = []

    dartpy_cpp = root / "python" / "dartpy" / "dartpy.cpp"
    dartpy_cpp_text = _read(dartpy_cpp, root, violations)
    _require_regex(
        violations,
        dartpy_cpp,
        root,
        dartpy_cpp_text,
        r"\bdef_submodule\(\s*\"simulation\"\s*,",
        "dartpy must define the canonical simulation submodule",
    )
    _require_contains(
        violations,
        dartpy_cpp,
        root,
        dartpy_cpp_text,
        "defSimulationModule(simulation)",
        "ECS World bindings must register into dartpy.simulation",
    )
    _require_no_regex(
        violations,
        dartpy_cpp,
        root,
        dartpy_cpp_text,
        r"\bdef_submodule\(\s*\"simulation_experimental\"",
        "dartpy.simulation_experimental must not be a second C++ submodule",
    )

    for removed_path in (
        root / "python" / "dartpy" / "simulation" / "world.cpp",
        root / "python" / "dartpy" / "simulation" / "constraint_solver.cpp",
        root / "python" / "dartpy" / "simulation_experimental" / "module.cpp",
    ):
        if removed_path.exists():
            violations.append(
                Violation(
                    _rel(removed_path, root),
                    "DART 7 dartpy must not retain DART 6 or experimental "
                    "simulation binding sources",
                )
            )
    _require_absent(
        violations,
        dartpy_cpp,
        root,
        dartpy_cpp_text,
        "defSimulationModule(gui)",
        "simulation bindings must not be registered into dartpy.gui",
    )

    layout_py = root / "python" / "dartpy" / "_layout.py"
    layout_text = _read(layout_py, root, violations)
    try:
        legacy_modules = _literal_tuple_assignment(layout_text, "_LEGACY_MODULES")
    except SyntaxError as exc:
        violations.append(Violation(_rel(layout_py, root), f"cannot parse: {exc}"))
        legacy_modules = None
    if legacy_modules is None:
        violations.append(
            Violation(_rel(layout_py, root), "_LEGACY_MODULES must be a literal tuple")
        )
    elif "simulation" in legacy_modules:
        violations.append(
            Violation(
                _rel(layout_py, root),
                "simulation must not be installed as a deprecated legacy module",
            )
        )
    elif "utils" in legacy_modules or "io" in legacy_modules:
        violations.append(
            Violation(
                _rel(layout_py, root),
                "dartpy.io must be the official IO module, not a deprecated "
                "utils/io legacy wrapper",
            )
        )
    _require_contains(
        violations,
        layout_py,
        root,
        layout_text,
        '_PROMOTE_MODULES: tuple[str, ...] = ("simulation",) +',
        "simulation must be promoted first so dartpy.World is the ECS World",
    )

    init_py = root / "python" / "dartpy" / "__init__.py"
    init_text = _read(init_py, root, violations)
    _require_contains(
        violations,
        init_py,
        root,
        init_text,
        'module_name = f"{__name__}.simulation.diff"',
        "diff bridge must attach to dartpy.simulation.diff",
    )
    _require_contains(
        violations,
        init_py,
        root,
        init_text,
        'setattr(sys.modules[__name__], "diff", diff_module)',
        "diff bridge must also be available as dartpy.diff",
    )
    world_bridge_py = root / "python" / "dartpy" / "_world_render_bridge.py"
    world_bridge_text = _read(world_bridge_py, root, violations)
    _require_contains(
        violations,
        world_bridge_py,
        root,
        world_bridge_text,
        "def renderable_provider",
        "Python GUI bridge must provide descriptor renderables",
    )
    _require_absent(
        violations,
        world_bridge_py,
        root,
        world_bridge_text,
        "dart.gui.RenderWorld",
        "Python GUI bridge must not depend on the removed DART 6 RenderWorld",
    )

    stubs_root = root / "python" / "stubs" / "dartpy"
    simulation_experimental_stub = stubs_root / "simulation_experimental.pyi"
    if simulation_experimental_stub.exists():
        violations.append(
            Violation(
                _rel(simulation_experimental_stub, root),
                "stubs must not publish dartpy.simulation_experimental",
            )
        )

    top_stub = stubs_root / "__init__.pyi"
    top_stub_text = _read(top_stub, root, violations)
    _require_contains(
        violations,
        top_stub,
        root,
        top_stub_text,
        "from . import simulation",
        "top-level stubs must import the canonical simulation module",
    )
    _require_contains(
        violations,
        top_stub,
        root,
        top_stub_text,
        "from . import diff",
        "top-level stubs must import the canonical diff alias",
    )
    _require_contains(
        violations,
        top_stub,
        root,
        top_stub_text,
        "from . import io",
        "top-level stubs must import the canonical io module",
    )
    _require_absent(
        violations,
        top_stub,
        root,
        top_stub_text,
        "from . import utils",
        "top-level stubs must not import the removed utils module",
    )
    _require_contains(
        violations,
        top_stub,
        root,
        top_stub_text,
        "from .simulation import (",
        "top-level World must be re-exported from dartpy.simulation",
    )
    _require_absent(
        violations,
        top_stub,
        root,
        top_stub_text,
        "simulation_experimental",
        "top-level stubs must not reference simulation_experimental",
    )
    try:
        top_all = _literal_all(top_stub_text)
    except SyntaxError as exc:
        violations.append(Violation(_rel(top_stub, root), f"cannot parse: {exc}"))
        top_all = None
    if top_all is None:
        violations.append(Violation(_rel(top_stub, root), "__all__ must be literal"))
    else:
        for name in ("World", "simulation", "diff"):
            if name not in top_all:
                violations.append(
                    Violation(_rel(top_stub, root), f"__all__ must include {name}")
                )
        if "io" not in top_all:
            violations.append(
                Violation(_rel(top_stub, root), "__all__ must include io")
            )
        if "utils" in top_all:
            violations.append(
                Violation(
                    _rel(top_stub, root),
                    "__all__ must not include the removed utils module",
                )
            )
        if "simulation_experimental" in top_all:
            violations.append(
                Violation(
                    _rel(top_stub, root),
                    "__all__ must not include simulation_experimental",
                )
            )

    simulation_stub = stubs_root / "simulation.pyi"
    simulation_stub_text = _read(simulation_stub, root, violations)
    _require_contains(
        violations,
        simulation_stub,
        root,
        simulation_stub_text,
        "class World:",
        "dartpy.simulation stubs must define the official World",
    )
    _require_contains(
        violations,
        simulation_stub,
        root,
        simulation_stub_text,
        "from dartpy import diff as diff",
        "dartpy.simulation stubs must expose the canonical diff alias",
    )
    try:
        simulation_all = _literal_all(simulation_stub_text)
    except SyntaxError as exc:
        violations.append(
            Violation(_rel(simulation_stub, root), f"cannot parse: {exc}")
        )
        simulation_all = None
    if simulation_all is None:
        violations.append(
            Violation(_rel(simulation_stub, root), "__all__ must be literal")
        )
    elif "World" not in simulation_all:
        violations.append(
            Violation(_rel(simulation_stub, root), "__all__ must include World")
        )

    io_module_cpp = root / "python" / "dartpy" / "io" / "module.cpp"
    io_module_cpp_text = _read(io_module_cpp, root, violations)
    _require_contains(
        violations,
        io_module_cpp,
        root,
        io_module_cpp_text,
        "defIoRead(m)",
        "dartpy.io must register the unified readSkeleton binding",
    )

    io_stub = stubs_root / "io" / "__init__.pyi"
    io_stub_text = _read(io_stub, root, violations)
    for name in (
        "class ModelFormat",
        "class RootJointType",
        "class ReadOptions",
        "def readSkeleton",
        "read_skeleton = readSkeleton",
    ):
        _require_contains(
            violations,
            io_stub,
            root,
            io_stub_text,
            name,
            "dartpy.io stubs must expose the unified read_skeleton API",
        )

    removed_world_loader_stubs = {
        stubs_root / "io" / "SdfParser.pyi": ("readWorld", "read_world"),
        stubs_root / "io" / "MjcfParser.pyi": ("readWorld", "read_world"),
        io_stub: (
            "parseWorld",
            "parse_world",
            "parseWorldString",
            "parse_world_string",
        ),
    }
    for stub_path, names in removed_world_loader_stubs.items():
        stub_text = _read(stub_path, root, violations)
        for name in names:
            _require_absent(
                violations,
                stub_path,
                root,
                stub_text,
                name,
                f"dartpy.io must not stub DART 6 whole-world loader {name}",
            )

    removed_urdf_loader_sources = {
        root
        / "python"
        / "dartpy"
        / "io"
        / "urdf_parser.cpp": (
            "DartLoader",
            "DartLoaderTestAccess",
        ),
        io_stub: ("DartLoader", "DartLoaderTestAccess"),
    }
    for source_path, names in removed_urdf_loader_sources.items():
        source_text = _read(source_path, root, violations)
        for name in names:
            _require_absent(
                violations,
                source_path,
                root,
                source_text,
                name,
                "dartpy.io must expose UrdfParser only, not retired DART 6 "
                f"URDF loader {name}",
            )

    return violations


def find_runtime_violations(require_runtime: bool = False) -> list[Violation]:
    violations: list[Violation] = []
    try:
        dartpy = importlib.import_module("dartpy")
    except ModuleNotFoundError as exc:
        if require_runtime:
            violations.append(
                Violation(
                    "runtime",
                    f"dartpy import failed but --require-runtime was set: {exc}",
                )
            )
        else:
            print("runtime import check skipped: dartpy is not importable")
        return violations

    simulation = getattr(dartpy, "simulation", None)
    if simulation is None:
        violations.append(Violation("runtime", "dartpy.simulation is missing"))
        return violations
    if getattr(dartpy, "World", None) is not getattr(simulation, "World", None):
        violations.append(
            Violation("runtime", "dartpy.World must be dartpy.simulation.World")
        )
    if hasattr(dartpy, "simulation_experimental"):
        violations.append(
            Violation("runtime", "dartpy.simulation_experimental attribute exists")
        )
    if "dartpy.simulation_experimental" in sys.modules:
        violations.append(
            Violation("runtime", "dartpy.simulation_experimental module is loaded")
        )
    if getattr(dartpy, "diff", None) is not getattr(simulation, "diff", None):
        violations.append(
            Violation("runtime", "dartpy.diff must be dartpy.simulation.diff")
        )
    gui = getattr(dartpy, "gui", None)
    if gui is not None and hasattr(gui, "World"):
        violations.append(
            Violation(
                "runtime",
                "classic render World must not be exposed as dartpy.gui.World",
            )
        )
    if gui is not None and hasattr(gui, "RenderWorld"):
        violations.append(
            Violation("runtime", "classic render World must not be gui.RenderWorld")
        )
    if gui is not None and not hasattr(gui, "WorldRenderBridge"):
        violations.append(
            Violation("runtime", "descriptor WorldRenderBridge must be available")
        )
    io = getattr(dartpy, "io", None)
    if io is not None:
        for name in ("ModelFormat", "RootJointType", "ReadOptions", "read_skeleton"):
            if not hasattr(io, name):
                violations.append(
                    Violation("runtime", f"dartpy.io.{name} must be exposed")
                )

        for name in ("DartLoader", "DartLoaderTestAccess"):
            if hasattr(io, name):
                violations.append(
                    Violation(
                        "runtime",
                        "dartpy.io must expose UrdfParser only, not retired "
                        f"DART 6 URDF loader {name}",
                    )
                )

        removed_world_loader_attrs = {
            "SdfParser": ("readWorld", "read_world"),
            "MjcfParser": ("readWorld", "read_world"),
        }
        for module_name, names in removed_world_loader_attrs.items():
            module = getattr(io, module_name, None)
            if module is None:
                continue
            for name in names:
                if hasattr(module, name):
                    violations.append(
                        Violation(
                            "runtime",
                            f"dartpy.io.{module_name}.{name} must be removed",
                        )
                    )
        urdf_parser = getattr(io, "UrdfParser", None)
        if urdf_parser is not None:
            for name in (
                "parseWorld",
                "parse_world",
                "parseWorldString",
                "parse_world_string",
            ):
                if hasattr(urdf_parser, name):
                    violations.append(
                        Violation(
                            "runtime",
                            f"dartpy.io.UrdfParser.{name} must be removed",
                        )
                    )
    return violations


def find_violations(
    root: Path = REPO_ROOT, require_runtime: bool = False
) -> list[Violation]:
    return find_static_violations(root) + find_runtime_violations(require_runtime)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--require-runtime",
        action="store_true",
        help="Fail if dartpy cannot be imported and run live identity checks.",
    )
    args = parser.parse_args()

    violations = find_violations(REPO_ROOT, require_runtime=args.require_runtime)
    if violations:
        print("dartpy import-layout check failed:")
        for violation in violations:
            print(f"  - {violation.path}: {violation.message}")
        return 1

    print("dartpy import-layout check passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
