#!/usr/bin/env python3
"""Audit retained legacy collision names as native-backed facades.

This is a static completion guard for the native collision migration. It checks
that compatibility factory keys, public C++ detector/group names, package
components, and dartpy's clean API boundary do not reintroduce FCL, Bullet, or
ODE as runtime backends.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class EngineFacade:
    engine: str
    detector: str
    group: str
    factory_keys: tuple[str, ...]
    public_headers: tuple[str, ...]


FACADES = (
    EngineFacade(
        engine="fcl",
        detector="FCLCollisionDetector",
        group="FCLCollisionGroup",
        factory_keys=("fcl", "fcl_mesh"),
        public_headers=(
            "All.hpp",
            "FCLCollisionDetector.hpp",
            "FCLCollisionGroup.hpp",
            "fcl.hpp",
            "fcl_collision_detector.hpp",
            "fcl_collision_group.hpp",
        ),
    ),
    EngineFacade(
        engine="bullet",
        detector="BulletCollisionDetector",
        group="BulletCollisionGroup",
        factory_keys=("bullet",),
        public_headers=(
            "All.hpp",
            "BulletCollisionDetector.hpp",
            "BulletCollisionGroup.hpp",
            "bullet.hpp",
            "bullet_collision_detector.hpp",
            "bullet_collision_group.hpp",
        ),
    ),
    EngineFacade(
        engine="ode",
        detector="OdeCollisionDetector",
        group="OdeCollisionGroup",
        factory_keys=("ode",),
        public_headers=(
            "All.hpp",
            "OdeCollisionDetector.hpp",
            "OdeCollisionGroup.hpp",
            "ode.hpp",
            "ode_collision_detector.hpp",
            "ode_collision_group.hpp",
        ),
    ),
)

LEGACY_FACTORY_KEYS = ("experimental",)


def read_text(repo_root: Path, relative_path: str) -> str:
    path = repo_root / relative_path
    if not path.exists():
        raise AssertionError(f"{relative_path}: missing required artifact")
    return path.read_text(encoding="utf-8", errors="replace")


def require_contains(failures: list[str], text: str, needle: str, label: str) -> None:
    if needle not in text:
        failures.append(f"{label}: expected to find {needle!r}")


def require_not_contains(
    failures: list[str], text: str, needle: str, label: str
) -> None:
    if needle in text:
        failures.append(f"{label}: expected not to find {needle!r}")


def registration_returns_native(source: str, key: str) -> bool:
    for match in re.finditer(rf'"{re.escape(key)}"', source):
        block = source[match.start() : match.start() + 520]
        if "DartCollisionDetector::create()" in block:
            return True
        if "createLegacyAliasDetector(" in block:
            return "return DartCollisionDetector::create();" in source
    return False


def check_factory_aliases(repo_root: Path) -> list[str]:
    failures: list[str] = []
    source = read_text(repo_root, "dart/collision/dart/dart_collision_detector.cpp")
    for key in LEGACY_FACTORY_KEYS:
        if not registration_returns_native(source, key):
            failures.append(
                f'factory key "{key}" must register DartCollisionDetector::create()'
            )

    for facade in FACADES:
        for key in facade.factory_keys:
            if not registration_returns_native(source, key):
                failures.append(
                    f'factory key "{key}" must register '
                    "DartCollisionDetector::create()"
                )
    return failures


def check_cpp_facade(repo_root: Path, facade: EngineFacade) -> list[str]:
    failures: list[str] = []
    compat_header = (
        f"dart/collision/{facade.engine}/compat/"
        f"{facade.engine}_collision_detector.hpp"
    )
    compat_text = read_text(repo_root, compat_header)
    class_pattern = re.compile(
        rf"class\s+(?:DART_COLLISION_LEGACY_NAME_DEPRECATED\s+)?"
        rf"{facade.detector}\s*:\s*public\s+DartCollisionDetector"
    )
    if not class_pattern.search(compat_text):
        failures.append(
            f"{compat_header}: expected {facade.detector} to inherit "
            "DartCollisionDetector"
        )
    require_contains(
        failures,
        compat_text,
        "DART_COLLISION_LEGACY_NAME_DEPRECATED",
        compat_header,
    )
    require_contains(
        failures,
        compat_text,
        f"using {facade.group} = DartCollisionGroup;",
        compat_header,
    )
    require_contains(
        failures,
        compat_text,
        "dart/collision/dart/dart_collision_detector.hpp",
        compat_header,
    )

    for header in facade.public_headers:
        relative_path = f"dart/collision/{facade.engine}/{header}"
        text = read_text(repo_root, relative_path)
        expected = f"#include <dart/collision/{facade.engine}/compat/" f"{header}>"
        if header in {
            "All.hpp",
            f"{facade.engine}.hpp",
            f"{facade.engine}_collision_group.hpp",
            f"{facade.group}.hpp",
        }:
            # Some aggregate compatibility headers forward through the local
            # public facade names to preserve legacy include spelling.
            expected_prefix = f"#include <dart/collision/{facade.engine}/"
            if expected_prefix not in text:
                failures.append(
                    f"{relative_path}: expected a public facade forwarding include"
                )
        elif expected not in text:
            failures.append(
                f"{relative_path}: expected to forward to native-backed "
                f"compat header {expected!r}"
            )
    return failures


def check_python_clean_api(repo_root: Path) -> list[str]:
    failures: list[str] = []
    source = read_text(repo_root, "python/dartpy/collision/collision_detector.cpp")
    require_contains(
        failures,
        source,
        'm, "DartCollisionDetector"',
        "python dartpy collision API",
    )
    for alias in (
        "DARTCollisionDetector",
        *(facade.detector for facade in FACADES),
    ):
        require_not_contains(
            failures,
            source,
            f'm.attr("{alias}")',
            "python dartpy collision API",
        )
    return failures


def check_package_components(repo_root: Path) -> list[str]:
    failures: list[str] = []
    source = read_text(repo_root, "dart/CMakeLists.txt")
    require_contains(
        failures,
        source,
        "foreach(_dart_legacy_collision_component fcl bullet ode)",
        "dart/CMakeLists.txt",
    )
    pattern = re.compile(
        r"add_library\(\$\{_dart_legacy_collision_target\} INTERFACE\).*?"
        r"target_link_libraries\(\$\{_dart_legacy_collision_target\} "
        r"INTERFACE dart\)",
        re.DOTALL,
    )
    if not pattern.search(source):
        failures.append(
            "dart/CMakeLists.txt: legacy collision package components must be "
            "INTERFACE targets that link only dart"
        )
    return failures


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    failures: list[str] = []
    failures.extend(check_factory_aliases(repo_root))
    for facade in FACADES:
        failures.extend(check_cpp_facade(repo_root, facade))
    failures.extend(check_python_clean_api(repo_root))
    failures.extend(check_package_components(repo_root))

    if failures:
        print("Collision compatibility facade audit failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Collision compatibility facade audit passed.")
    print("  factory keys: experimental, fcl, fcl_mesh, bullet, ode -> dart")
    print(
        "  C++ facades: FCLCollisionDetector, BulletCollisionDetector, "
        "OdeCollisionDetector -> DartCollisionDetector"
    )
    print("  dartpy API: DartCollisionDetector only; legacy detector aliases absent")
    print("  package components: collision-fcl/bullet/ode -> dart")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
