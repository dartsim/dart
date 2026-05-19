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


def check_deprecation_policy(repo_root: Path) -> list[str]:
    failures: list[str] = []
    cmake = read_text(repo_root, "CMakeLists.txt")
    option_pattern = re.compile(
        r"dart_option\(\s*"
        r"DART_COLLISION_DEPRECATE_LEGACY_NAMES\s+"
        r'"Emit deprecation warnings for legacy collision detector names '
        r'that now route to the built-in DART detector"\s+'
        r"ON\s+CATEGORY build\s*\)",
        re.DOTALL,
    )
    if not option_pattern.search(cmake):
        failures.append(
            "CMakeLists.txt: DART_COLLISION_DEPRECATE_LEGACY_NAMES must "
            "default ON for the DART 7 migration window"
        )

    source = read_text(repo_root, "dart/collision/dart/dart_collision_detector.cpp")
    require_contains(
        failures,
        source,
        "#if DART_COLLISION_DEPRECATE_LEGACY_NAMES",
        "legacy factory key deprecation warning",
    )
    require_contains(
        failures,
        source,
        "DART_WARN_ONCE(",
        "legacy factory key deprecation warning",
    )
    require_contains(
        failures,
        source,
        "Collision detector factory key '{}' is deprecated",
        "legacy factory key deprecation warning",
    )
    require_contains(
        failures,
        source,
        "use 'dart' or the ",
        "legacy factory key deprecation warning",
    )
    require_contains(
        failures,
        source,
        "default collision detector instead",
        "legacy factory key deprecation warning",
    )
    require_contains(
        failures,
        source,
        "return DartCollisionDetector::create();",
        "legacy factory key deprecation warning",
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


def check_facade_header_installs(repo_root: Path, facade: EngineFacade) -> list[str]:
    failures: list[str] = []
    cmake_path = f"dart/collision/{facade.engine}/CMakeLists.txt"
    source = read_text(repo_root, cmake_path)

    facade_install_pattern = re.compile(
        r"install\(\s*"
        r"FILES\s+\$\{facade_hdrs\}\s+"
        rf"DESTINATION\s+include/dart/collision/{facade.engine}\s+"
        r"COMPONENT\s+headers\s*"
        r"\)",
        re.DOTALL,
    )
    if not facade_install_pattern.search(source):
        failures.append(
            f"{cmake_path}: public facade headers must install to "
            f"include/dart/collision/{facade.engine}"
        )

    compat_install_pattern = re.compile(
        r"install\(\s*"
        r"FILES\s+\$\{compat_hdrs\}\s+"
        rf"DESTINATION\s+include/dart/collision/{facade.engine}/compat\s+"
        r"COMPONENT\s+headers\s*"
        r"\)",
        re.DOTALL,
    )
    if not compat_install_pattern.search(source):
        failures.append(
            f"{cmake_path}: compat implementation headers must install to "
            f"include/dart/collision/{facade.engine}/compat"
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


def check_legacy_facade_raycast_contract(repo_root: Path) -> list[str]:
    failures: list[str] = []
    unsupported_facade_headers = (
        (
            "dart/collision/dart/DARTCollisionDetector.hpp",
            "gz-physics, where",
        ),
        (
            "dart/collision/fcl/compat/fcl_collision_detector.hpp",
            'legacy "fcl" facade',
        ),
        (
            "dart/collision/ode/compat/ode_collision_detector.hpp",
            'legacy "ode" facade',
        ),
    )

    for relative_path, comment_marker in unsupported_facade_headers:
        source = read_text(repo_root, relative_path)
        require_contains(
            failures,
            source,
            "CollisionDetector::raycast(",
            f"{relative_path} raycast facade",
        )
        require_contains(
            failures,
            source,
            comment_marker,
            f"{relative_path} raycast facade",
        )

    bullet_source = read_text(
        repo_root, "dart/collision/bullet/compat/bullet_collision_detector.hpp"
    )
    require_not_contains(
        failures,
        bullet_source,
        "CollisionDetector::raycast(",
        "dart/collision/bullet/compat/bullet_collision_detector.hpp raycast facade",
    )
    return failures


def check_package_components(repo_root: Path) -> list[str]:
    failures: list[str] = []
    source = read_text(repo_root, "dart/CMakeLists.txt")
    require_not_contains(
        failures,
        source,
        "collision-reference-",
        "dart/CMakeLists.txt",
    )
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
    native_link_pattern = re.compile(
        r"target_link_libraries\(\s*"
        r"dart\s+PUBLIC\s+\$\{PROJECT_NAME\}-collision-native\s*\)",
        re.DOTALL,
    )
    if not native_link_pattern.search(source):
        failures.append(
            "dart/CMakeLists.txt: dart must explicitly link "
            "${PROJECT_NAME}-collision-native"
        )
    pkgconfig_link_pattern = re.compile(
        r"dart_pkgconfig_append_libraries\(\s*"
        r"\"-l\$\{PROJECT_NAME\}-collision-native\"\s*\)",
        re.DOTALL,
    )
    if not pkgconfig_link_pattern.search(source):
        failures.append(
            "dart/CMakeLists.txt: dart pkg-config metadata must include "
            "-ldart-collision-native"
        )
    return failures


def check_pkg_config_external_deps(repo_root: Path) -> list[str]:
    failures: list[str] = []
    source = read_text(repo_root, "CMakeLists.txt")
    match = re.search(r'set\(DART_PKG_EXTERNAL_DEPS\s+"([^"]*)"\)', source)
    if not match:
        failures.append("CMakeLists.txt: missing DART_PKG_EXTERNAL_DEPS")
        return failures

    deps = [dep.strip() for dep in match.group(1).split(",")]
    if "ccd" in deps:
        failures.append(
            "CMakeLists.txt: dart pkg-config Requires must not advertise ccd "
            "for the native-only runtime"
        )
    return failures


def check_reference_tree(repo_root: Path) -> list[str]:
    failures: list[str] = []
    for facade in FACADES:
        old_dir = repo_root / "dart" / "collision" / facade.engine / "reference"
        if old_dir.exists():
            failures.append(
                f"{old_dir.relative_to(repo_root).as_posix()}: reference "
                "implementation must live under tests/dart/test/reference_collision/"
            )

        new_dir = (
            repo_root
            / "tests"
            / "dart"
            / "test"
            / "reference_collision"
            / facade.engine
        )
        if not new_dir.is_dir():
            failures.append(
                f"{new_dir.relative_to(repo_root).as_posix()}: missing "
                "test-only reference implementation directory"
            )

        cmake = read_text(
            repo_root,
            f"tests/dart/test/reference_collision/{facade.engine}/CMakeLists.txt",
        )
        require_contains(
            failures,
            cmake,
            f"-test-reference-{facade.engine}",
            f"tests/dart/test/reference_collision/{facade.engine}/CMakeLists.txt",
        )
        require_not_contains(
            failures,
            cmake,
            "INSTALL_INTERFACE",
            f"tests/dart/test/reference_collision/{facade.engine}/CMakeLists.txt",
        )
    return failures


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    failures: list[str] = []
    failures.extend(check_factory_aliases(repo_root))
    failures.extend(check_deprecation_policy(repo_root))
    for facade in FACADES:
        failures.extend(check_cpp_facade(repo_root, facade))
        failures.extend(check_facade_header_installs(repo_root, facade))
    failures.extend(check_python_clean_api(repo_root))
    failures.extend(check_legacy_facade_raycast_contract(repo_root))
    failures.extend(check_package_components(repo_root))
    failures.extend(check_pkg_config_external_deps(repo_root))
    failures.extend(check_reference_tree(repo_root))

    if failures:
        print("Collision compatibility facade audit failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Collision compatibility facade audit passed.")
    print("  factory keys: experimental, fcl, fcl_mesh, bullet, ode -> dart")
    print("  deprecation policy: legacy C++ names warn by default")
    print(
        "  C++ facades: FCLCollisionDetector, BulletCollisionDetector, "
        "OdeCollisionDetector -> DartCollisionDetector"
    )
    print("  C++ facade headers: public and compat headers install in place")
    print("  dartpy API: DartCollisionDetector only; legacy detector aliases absent")
    print(
        "  raycast facades: dart/fcl/ode preserve gz unsupported behavior; "
        "bullet uses native raycast"
    )
    print("  package components: collision-fcl/bullet/ode -> dart")
    print("  link interface: dart -> dart-collision-native")
    print("  pkg-config: dart -> -ldart-collision-native without ccd Requires")
    print("  reference engines: tests/dart/test/reference_collision only")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
