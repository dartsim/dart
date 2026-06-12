#!/usr/bin/env python3
"""Check the DART 7 simulation promotion package contract.

PLAN-041 promotes the ECS-backed World as the DART 7 simulation package. This
gate is static by design: it runs in lint jobs and guards the CMake/package
facts that must stay true after the namespace and target promotion.
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SIMULATION_CMAKE = Path("dart/simulation/CMakeLists.txt")
ROOT_CMAKE = Path("CMakeLists.txt")

PRIVATE_IMPLEMENTATION_DEPS = {
    "EnTT::EnTT",
    "Taskflow",
    "Taskflow::Taskflow",
    "spdlog::spdlog",
}
STATIC_LINK_ONLY_DEP_PACKAGES = {"EnTT", "Taskflow", "spdlog"}


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


def _strip_comments(text: str) -> str:
    return re.sub(r"#[^\n]*", "", text)


def _cmake_calls(text: str, name: str) -> list[tuple[int, str]]:
    """Return ``(start_offset, body)`` for simple CMake function calls."""
    calls: list[tuple[int, str]] = []
    pattern = re.compile(rf"\b{re.escape(name)}\s*\(", re.IGNORECASE)
    for match in pattern.finditer(text):
        depth = 1
        index = match.end()
        while index < len(text) and depth:
            char = text[index]
            if char == "(":
                depth += 1
            elif char == ")":
                depth -= 1
            index += 1
        if depth == 0:
            calls.append((match.start(), text[match.end() : index - 1]))
    return calls


def _tokens(body: str) -> list[str]:
    body = re.sub(r'"(?:\\.|[^"])*"', '""', body)
    return re.findall(r"[A-Za-z0-9_:.${}/+-]+", body)


def _dart_option_default(text: str, option_name: str) -> str | None:
    for _, body in _cmake_calls(text, "dart_option"):
        tokens = _tokens(body)
        if not tokens or tokens[0] != option_name:
            continue
        for token in tokens[1:]:
            upper = token.upper()
            if upper in {"ON", "OFF"}:
                return upper
    return None


def _active_conditions_before(text: str, offset: int) -> list[str]:
    stack: list[str] = []
    for match in re.finditer(
        r"\b(if|elseif|else|endif)\s*\(([^)]*)\)",
        text[:offset],
        re.IGNORECASE | re.DOTALL,
    ):
        kind = match.group(1).lower()
        condition = re.sub(r"\s+", "", match.group(2)).upper()
        if kind == "if":
            stack.append(condition)
        elif kind == "endif" and stack:
            stack.pop()
        elif kind in {"elseif", "else"} and stack:
            stack[-1] = condition if kind == "elseif" else "ELSE"
    return stack


def _target_link_modes(cmake_text: str) -> dict[str, set[str]]:
    """Return dependency -> CMake visibility modes for target_link_libraries."""
    modes: dict[str, set[str]] = {}
    for _, body in _cmake_calls(cmake_text, "target_link_libraries"):
        tokens = _tokens(body)
        if not tokens or tokens[0] != "${target_name}":
            continue
        current_mode = "PRIVATE"
        for token in tokens[1:]:
            if token in {"PUBLIC", "PRIVATE", "INTERFACE"}:
                current_mode = token
                continue
            modes.setdefault(token, set()).add(current_mode)
    return modes


def _dependency_package_calls(cmake_text: str) -> list[tuple[set[str], list[str]]]:
    calls: list[tuple[set[str], list[str]]] = []
    for offset, body in _cmake_calls(cmake_text, "add_component_dependency_packages"):
        tokens = _tokens(body)
        packages = {token for token in tokens if token in STATIC_LINK_ONLY_DEP_PACKAGES}
        if packages:
            calls.append((packages, _active_conditions_before(cmake_text, offset)))
    return calls


def find_violations(root: Path = REPO_ROOT) -> list[Violation]:
    violations: list[Violation] = []
    root_cmake = root / ROOT_CMAKE
    root_text = _read(root_cmake, root, violations)
    simulation_cmake = root / SIMULATION_CMAKE
    simulation_text = _read(simulation_cmake, root, violations)
    simulation_no_comments = _strip_comments(simulation_text)

    if "DART_BUILD_SIMULATION_EXPERIMENTAL" in root_text:
        violations.append(
            Violation(
                ROOT_CMAKE.as_posix(),
                "DART_BUILD_SIMULATION_EXPERIMENTAL was retired; the DART 7 "
                "World stack must stay non-optional on main",
            )
        )

    for option in ("DART_BUILD_DIFF", "DART_ENABLE_EXPERIMENTAL_CUDA"):
        default = _dart_option_default(root_text, option)
        if default is None:
            violations.append(Violation(ROOT_CMAKE.as_posix(), f"{option} is missing"))
        elif default != "OFF":
            violations.append(
                Violation(
                    ROOT_CMAKE.as_posix(),
                    f"{option} must stay opt-in until it has its own public "
                    "package contract",
                )
            )

    if re.search(r"\binstall\s*\(\s*DIRECTORY\b", simulation_no_comments):
        violations.append(
            Violation(
                SIMULATION_CMAKE.as_posix(),
                "simulation promotion must use explicit install(FILES ...) "
                "allowlists, not recursive install(DIRECTORY ...)",
            )
        )

    if "DART_SIMULATION_PUBLIC_HEADERS" not in simulation_no_comments:
        violations.append(
            Violation(
                SIMULATION_CMAKE.as_posix(),
                "simulation public header allowlist must be exported for the "
                "public-header smoke test",
            )
        )

    link_modes = _target_link_modes(simulation_no_comments)
    for dependency in sorted(PRIVATE_IMPLEMENTATION_DEPS):
        modes = link_modes.get(dependency, set())
        public_modes = modes & {"PUBLIC", "INTERFACE"}
        if public_modes:
            violations.append(
                Violation(
                    SIMULATION_CMAKE.as_posix(),
                    f"{dependency} must remain a PRIVATE implementation "
                    "dependency, not {'/'.join(sorted(public_modes))}",
                )
            )

    for packages, conditions in _dependency_package_calls(simulation_no_comments):
        if "NOTBUILD_SHARED_LIBS" not in conditions:
            violations.append(
                Violation(
                    SIMULATION_CMAKE.as_posix(),
                    "EnTT/Taskflow/spdlog component dependency packages are "
                    "allowed only under if(NOT BUILD_SHARED_LIBS) for static "
                    f"link-only exports; found {', '.join(sorted(packages))}",
                )
            )

    return violations


def _final_debt(root: Path = REPO_ROOT) -> list[Violation]:
    """Return facts that are expected to fail until the final promotion PR."""
    violations: list[Violation] = []
    simulation_text = (root / SIMULATION_CMAKE).read_text(encoding="utf-8")
    if "simulation" in simulation_text:
        violations.append(
            Violation(
                SIMULATION_CMAKE.as_posix(),
                "final DART 7 package still has a transitional target/component name",
            )
        )
    return violations


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=REPO_ROOT,
        help="Repository root to check.",
    )
    parser.add_argument(
        "--strict-final",
        action="store_true",
        help=(
            "Also fail on any remaining final-promotion debt such as "
            "transitional target/component names."
        ),
    )
    args = parser.parse_args()

    root = args.root.resolve()
    violations = find_violations(root)
    if args.strict_final and not violations:
        violations.extend(_final_debt(root))

    if violations:
        for violation in violations:
            print(f"{violation.path}: {violation.message}")
        return 1

    print("DART 7 simulation promotion package contract check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
