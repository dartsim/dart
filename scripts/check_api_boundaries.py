#!/usr/bin/env python3
"""Check source-level public/internal API boundaries.

Covers four boundaries:
- Python bindings (``python/dartpy``) must not reach into C++ detail/internal.
- Generated Doxygen output must exclude private/detail paths.
- The dartsim editor UI layer (``dartsim/ui``) must stay renderer- and
  windowing-agnostic: no ImGui/GLFW/Filament/OpenGL/Vulkan/Metal symbols,
  headers, or references. Editor panels talk to the backend-hidden
  ``dart::gui::PanelBuilder`` seam, so the toolkit can be re-implemented without
  rewriting the UI layer. This guard keeps that boundary from quietly becoming a
  one-way door (see ``docs/design/dartsim_gui_toolkit_decisions.md`` Decision 2,
  Action 1).
- The dartsim editor engine (``dartsim/engine``) is headless: it must not
  reference any renderer/windowing backend symbol *or* include a ``dart/gui``
  header at all (the engine depends only on the DART 7 simulation API).
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
ALLOWLIST_PATH = REPO_ROOT / "scripts" / "check_api_boundaries_allowlist.txt"
PYTHON_BINDING_ROOT = REPO_ROOT / "python" / "dartpy"
DARTSIM_UI_ROOT = REPO_ROOT / "dartsim" / "ui"
DARTSIM_ENGINE_ROOT = REPO_ROOT / "dartsim" / "engine"
DOXYFILE_PATH = REPO_ROOT / "docs" / "doxygen" / "Doxyfile.in"
GENERATED_CPP_API_ROOT = REPO_ROOT / "docs" / "readthedocs" / "_generated" / "cpp-api"
SOURCE_SUFFIXES = {".cpp", ".h", ".hpp"}
REQUIRED_DOXYFILE_PATTERNS = (
    "*/detail/*",
    "*/internal/*",
    "*/simulation/experimental/comps/*",
    "*/*-impl.hpp",
    "*/*_impl.hpp",
    "*::detail",
    "*::detail::*",
    "*::internal",
    "*::internal::*",
)
GENERATED_DOC_PRIVATE_MARKERS = ("detail", "internal", "-impl", "_impl")


@dataclass(frozen=True)
class AllowEntry:
    check_id: str
    path: str
    substring: str
    replacement: str
    remove_by: str
    tracking: str
    reason: str


@dataclass(frozen=True)
class Violation:
    check_id: str
    path: str
    line: int
    text: str
    message: str


LINE_CHECKS: tuple[tuple[str, re.Pattern[str], str], ...] = (
    (
        "python-private-include",
        re.compile(
            r"^\s*#\s*include\s*[<\"]dart/[^>\"]*/(?:detail|internal)/[^>\"]+[>\"]"
        ),
        "Python bindings must not include C++ detail/internal headers.",
    ),
    (
        "python-experimental-comps-include",
        re.compile(r"^\s*#\s*include\s*[<\"]dart/simulation/comps/[^>\"]+[>\"]"),
        "Python bindings must not include DART 7 component storage headers.",
    ),
    (
        "python-private-namespace-reference",
        re.compile(r"\bdart(?:::[A-Za-z_]\w*)*::(?:detail|internal)\b"),
        "Python bindings must not bind or reference C++ detail/internal namespace symbols.",
    ),
)

# Boundary before the token, open suffix after, so prefixed identifiers such as
# ``GLFWwindow``, ``glfwGetKey``, ``ImGui::Begin``, ``filament::Engine``,
# ``vulkan``-prefixed helpers, and ``opengl`` references are all caught. A
# legitimate physics "filament" term (none today) would use the allowlist. Bare
# ``gl``/``vk`` are intentionally excluded to avoid false positives (``global``,
# ``glance``); raw backend headers are caught by the include-path pattern below.
DARTSIM_UI_BACKEND_PATTERN = re.compile(
    r"(?i)\b(?:imgui|glfw|filament|filagui|opengl|vulkan|glad|epoxy|bgfx)\w*"
)
# Raw renderer/windowing headers whose include path is the reliable leak signal
# even when the API uses generic-looking spellings (``GL/gl.h``, ``Metal``,
# ``vulkan/vulkan.h``, ``EGL``). Matched against ``#include`` lines only.
DARTSIM_UI_BACKEND_INCLUDE_PATTERN = re.compile(
    r"^\s*#\s*include\s*[<\"]"
    r"(?:GL/|GLES[0-9]*/|GLFW/|EGL/|vulkan/|Metal/|MetalKit/|glad/|epoxy/|bgfx/"
    r"|filament/|filagui/|imgui|backends/imgui)"
)
DARTSIM_UI_BACKEND_MESSAGE = (
    "dartsim/ui must stay renderer/windowing-agnostic: no ImGui/GLFW/Filament/"
    "OpenGL/Vulkan/Metal symbols, headers, or references (build UI against "
    "dart::gui::PanelBuilder)."
)
DARTSIM_ENGINE_GUI_INCLUDE_PATTERN = re.compile(
    r"^\s*#\s*include\s*[<\"]dart/gui/[^>\"]+[>\"]"
)
DARTSIM_ENGINE_BACKEND_MESSAGE = (
    "dartsim/engine is headless: no renderer/windowing backend symbols and no "
    "dart/gui include (depend only on dart::simulation + math)."
)


def line_has_backend_leak(line: str) -> bool:
    return bool(
        DARTSIM_UI_BACKEND_PATTERN.search(line)
        or DARTSIM_UI_BACKEND_INCLUDE_PATTERN.search(line)
    )


JOINT_PROPERTIES_TYPE_PATTERN_TEXT = (
    r"(?:::)?(?:[A-Za-z_]\w*::)*(?:[A-Za-z_]\w*)?Joint[A-Za-z_0-9]*::Properties"
)
JOINT_PROPERTIES_TYPE_PATTERN = re.compile(JOINT_PROPERTIES_TYPE_PATTERN_TEXT)
JOINT_PROPERTIES_ALIAS_PATTERN = re.compile(
    rf"\busing\s+(?P<alias>[A-Za-z_]\w*)\s*=\s*"
    rf"(?:typename\s+)?{JOINT_PROPERTIES_TYPE_PATTERN_TEXT}\s*;"
)
CLASS_BINDING_TYPE_PATTERN = re.compile(
    rf"\bnb::class_<\s*(?P<type>{JOINT_PROPERTIES_TYPE_PATTERN_TEXT}|[A-Za-z_]\w*)\b"
)
PROPERTIES_BINDING_MESSAGE = (
    "Python bindings must not expose legacy C++ Properties aliases without an "
    "allowlist entry."
)


def load_allowlist() -> list[AllowEntry]:
    entries: list[AllowEntry] = []
    for line_number, raw_line in enumerate(ALLOWLIST_PATH.read_text().splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue

        parts = line.split("|", 6)
        if len(parts) != 7:
            raise ValueError(
                f"{ALLOWLIST_PATH}:{line_number}: expected "
                "check_id|path|substring|replacement|remove_by|tracking|reason"
            )

        check_id, path, substring, replacement, remove_by, tracking, reason = (
            part.strip() for part in parts
        )
        if not all(
            (check_id, path, substring, replacement, remove_by, tracking, reason)
        ):
            raise ValueError(
                f"{ALLOWLIST_PATH}:{line_number}: allowlist fields cannot be empty"
            )
        entries.append(
            AllowEntry(
                check_id, path, substring, replacement, remove_by, tracking, reason
            )
        )
    return entries


def iter_python_binding_sources() -> list[Path]:
    return sorted(
        path
        for path in PYTHON_BINDING_ROOT.rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def find_python_boundary_violations_in_lines(
    rel_path: str, lines: list[str]
) -> list[Violation]:
    violations: list[Violation] = []
    properties_aliases: dict[str, str] = {}

    for line_number, line in enumerate(lines, 1):
        stripped_line = line.strip()
        for check_id, pattern, message in LINE_CHECKS:
            if pattern.search(line):
                violations.append(
                    Violation(check_id, rel_path, line_number, stripped_line, message)
                )

        alias_match = JOINT_PROPERTIES_ALIAS_PATTERN.search(line)
        if alias_match:
            properties_aliases[alias_match.group("alias")] = stripped_line

        binding_match = CLASS_BINDING_TYPE_PATTERN.search(line)
        if binding_match:
            bound_type = binding_match.group("type")
            alias_line = properties_aliases.get(bound_type)
            if alias_line:
                violations.append(
                    Violation(
                        "python-properties-alias-binding",
                        rel_path,
                        line_number,
                        f"{alias_line} -> {stripped_line}",
                        PROPERTIES_BINDING_MESSAGE,
                    )
                )
            elif JOINT_PROPERTIES_TYPE_PATTERN.fullmatch(bound_type):
                violations.append(
                    Violation(
                        "python-properties-alias-binding",
                        rel_path,
                        line_number,
                        stripped_line,
                        PROPERTIES_BINDING_MESSAGE,
                    )
                )

    return violations


def find_python_boundary_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_python_binding_sources():
        rel_path = path.relative_to(REPO_ROOT).as_posix()
        violations.extend(
            find_python_boundary_violations_in_lines(
                rel_path, path.read_text().splitlines()
            )
        )
    return violations


def iter_source_files(root: Path) -> list[Path]:
    return sorted(
        path
        for path in root.rglob("*")
        if path.is_file() and path.suffix in SOURCE_SUFFIXES
    )


def find_dartsim_ui_boundary_violations_in_lines(
    rel_path: str, lines: list[str]
) -> list[Violation]:
    violations: list[Violation] = []
    for line_number, line in enumerate(lines, 1):
        if line_has_backend_leak(line):
            violations.append(
                Violation(
                    "dartsim-ui-backend-leak",
                    rel_path,
                    line_number,
                    line.strip(),
                    DARTSIM_UI_BACKEND_MESSAGE,
                )
            )
    return violations


def find_dartsim_ui_boundary_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_source_files(DARTSIM_UI_ROOT):
        rel_path = path.relative_to(REPO_ROOT).as_posix()
        violations.extend(
            find_dartsim_ui_boundary_violations_in_lines(
                rel_path, path.read_text().splitlines()
            )
        )
    return violations


def find_dartsim_engine_boundary_violations_in_lines(
    rel_path: str, lines: list[str]
) -> list[Violation]:
    violations: list[Violation] = []
    for line_number, line in enumerate(lines, 1):
        if line_has_backend_leak(line) or DARTSIM_ENGINE_GUI_INCLUDE_PATTERN.search(
            line
        ):
            violations.append(
                Violation(
                    "dartsim-engine-backend-leak",
                    rel_path,
                    line_number,
                    line.strip(),
                    DARTSIM_ENGINE_BACKEND_MESSAGE,
                )
            )
    return violations


def find_dartsim_engine_boundary_violations() -> list[Violation]:
    violations: list[Violation] = []
    for path in iter_source_files(DARTSIM_ENGINE_ROOT):
        rel_path = path.relative_to(REPO_ROOT).as_posix()
        violations.extend(
            find_dartsim_engine_boundary_violations_in_lines(
                rel_path, path.read_text().splitlines()
            )
        )
    return violations


def find_doxygen_boundary_violations() -> list[Violation]:
    violations: list[Violation] = []
    doxyfile_text = DOXYFILE_PATH.read_text()
    rel_doxyfile_path = DOXYFILE_PATH.relative_to(REPO_ROOT).as_posix()

    for required_pattern in REQUIRED_DOXYFILE_PATTERNS:
        if required_pattern not in doxyfile_text:
            violations.append(
                Violation(
                    "doxygen-boundary-config",
                    rel_doxyfile_path,
                    0,
                    required_pattern,
                    "Doxygen must exclude implementation-detail paths and symbols.",
                )
            )

    if GENERATED_CPP_API_ROOT.exists():
        for path in sorted(GENERATED_CPP_API_ROOT.rglob("*")):
            if not path.is_file():
                continue
            path_text = path.relative_to(GENERATED_CPP_API_ROOT).as_posix()
            if any(marker in path_text for marker in GENERATED_DOC_PRIVATE_MARKERS):
                violations.append(
                    Violation(
                        "doxygen-generated-private-file",
                        path.relative_to(REPO_ROOT).as_posix(),
                        0,
                        path_text,
                        "Generated C++ API docs must not include private/detail files.",
                    )
                )

    return violations


def is_allowed(violation: Violation, entry: AllowEntry) -> bool:
    return (
        violation.check_id == entry.check_id
        and violation.path == entry.path
        and entry.substring in violation.text
    )


def run_self_tests() -> None:
    fixture_path = "python/dartpy/_api_boundary_self_test.cpp"
    cases = (
        (
            "#include <dart/dynamics/detail/joint_aspect.hpp>",
            "python-private-include",
        ),
        (
            "#include <dart/simulation/comps/world.hpp>",
            "python-experimental-comps-include",
        ),
        (
            "namespace joint_detail = dart::dynamics::detail;",
            "python-private-namespace-reference",
        ),
        (
            "using namespace dart::dynamics::internal;",
            "python-private-namespace-reference",
        ),
        (
            "using dart::dynamics::detail::JointProperties;",
            "python-private-namespace-reference",
        ),
        (
            'nb::class_<ZeroDofJoint::Properties>(m, "ZeroDofJointProperties");',
            "python-properties-alias-binding",
        ),
        (
            'nb::class_<dart::dynamics::FreeJoint::Properties>(m, "FreeJointProperties");',
            "python-properties-alias-binding",
        ),
        (
            'nb::class_<Joint::Properties>(m, "JointProperties");',
            "python-properties-alias-binding",
        ),
        (
            'nb::class_<dart::dynamics::Joint::Properties>(m, "JointProperties");',
            "python-properties-alias-binding",
        ),
    )

    for source_line, expected_check_id in cases:
        violations = find_python_boundary_violations_in_lines(
            fixture_path, [source_line]
        )
        if not any(v.check_id == expected_check_id for v in violations):
            raise AssertionError(
                f"self-test failed for {expected_check_id}: {source_line}"
            )

    properties_violations = find_python_boundary_violations_in_lines(
        fixture_path,
        [
            "using Properties = dart::dynamics::RevoluteJoint::Properties;",
            'nb::class_<Properties>(m, "RevoluteJointProperties");',
        ],
    )
    if not any(
        v.check_id == "python-properties-alias-binding" for v in properties_violations
    ):
        raise AssertionError("self-test failed for Properties alias binding")

    renamed_alias_violations = find_python_boundary_violations_in_lines(
        fixture_path,
        [
            "using RevoluteProps = dart::dynamics::RevoluteJoint::Properties;",
            'nb::class_<RevoluteProps>(m, "RevoluteJointProperties");',
        ],
    )
    if not any(
        v.check_id == "python-properties-alias-binding"
        for v in renamed_alias_violations
    ):
        raise AssertionError("self-test failed for renamed Properties alias binding")

    base_joint_alias_violations = find_python_boundary_violations_in_lines(
        fixture_path,
        [
            "using JointProps = dart::dynamics::Joint::Properties;",
            'nb::class_<JointProps>(m, "JointProperties");',
        ],
    )
    if not any(
        v.check_id == "python-properties-alias-binding"
        for v in base_joint_alias_violations
    ):
        raise AssertionError("self-test failed for Joint Properties alias binding")

    ui_fixture_path = "dartsim/ui/_backend_boundary_self_test.cpp"
    for leak_line in (
        "#include <imgui.h>",
        'ImGui::Begin("Inspector");',
        "GLFWwindow* window = nullptr;",
        "filament::Engine* engine = nullptr;",
        "/// Label used for an ImGui-style tree row button.",
        "#include <vulkan/vulkan.h>",
        "#include <GL/gl.h>",
        "#include <Metal/Metal.h>",
        "#include <EGL/egl.h>",
        "auto* ctx = openGLContext();",
    ):
        leak_violations = find_dartsim_ui_boundary_violations_in_lines(
            ui_fixture_path, [leak_line]
        )
        if not any(v.check_id == "dartsim-ui-backend-leak" for v in leak_violations):
            raise AssertionError(f"dartsim-ui self-test failed for: {leak_line}")

    clean_violations = find_dartsim_ui_boundary_violations_in_lines(
        ui_fixture_path,
        [
            "ProjectActionResult result = newProject(engine);",
            "app.note(buildInspectorStatus(engine).selectionSummary);",
            "int global = computeGlobalBounds();",
        ],
    )
    if clean_violations:
        raise AssertionError(
            "dartsim-ui self-test false positive on backend-free lines"
        )

    engine_fixture_path = "dartsim/engine/_backend_boundary_self_test.cpp"
    for leak_line in (
        "#include <dart/gui/application.hpp>",
        "#include <filament/Engine.h>",
        "#include <GL/gl.h>",
        'ImGui::Text("nope");',
    ):
        engine_violations = find_dartsim_engine_boundary_violations_in_lines(
            engine_fixture_path, [leak_line]
        )
        if not any(
            v.check_id == "dartsim-engine-backend-leak" for v in engine_violations
        ):
            raise AssertionError(f"dartsim-engine self-test failed for: {leak_line}")

    engine_clean_violations = find_dartsim_engine_boundary_violations_in_lines(
        engine_fixture_path,
        [
            "#include <dart/simulation/world.hpp>",
            "SceneModel model = engine.sceneModel();",
        ],
    )
    if engine_clean_violations:
        raise AssertionError(
            "dartsim-engine self-test false positive on backend-free lines"
        )


def main() -> int:
    run_self_tests()

    allowlist = load_allowlist()
    used_allowlist: set[AllowEntry] = set()
    unallowed: list[Violation] = []

    for violation in find_python_boundary_violations():
        matched_entry = next(
            (entry for entry in allowlist if is_allowed(violation, entry)),
            None,
        )
        if matched_entry is None:
            unallowed.append(violation)
        else:
            used_allowlist.add(matched_entry)

    for violation in find_doxygen_boundary_violations():
        matched_entry = next(
            (entry for entry in allowlist if is_allowed(violation, entry)),
            None,
        )
        if matched_entry is None:
            unallowed.append(violation)
        else:
            used_allowlist.add(matched_entry)

    for violation in find_dartsim_ui_boundary_violations():
        matched_entry = next(
            (entry for entry in allowlist if is_allowed(violation, entry)),
            None,
        )
        if matched_entry is None:
            unallowed.append(violation)
        else:
            used_allowlist.add(matched_entry)

    for violation in find_dartsim_engine_boundary_violations():
        matched_entry = next(
            (entry for entry in allowlist if is_allowed(violation, entry)),
            None,
        )
        if matched_entry is None:
            unallowed.append(violation)
        else:
            used_allowlist.add(matched_entry)

    stale_allowlist = [entry for entry in allowlist if entry not in used_allowlist]

    if not unallowed and not stale_allowlist:
        print("API boundary check passed.")
        return 0

    if unallowed:
        print("API boundary check failed:", file=sys.stderr)
        for violation in unallowed:
            print(
                f"{violation.path}:{violation.line}: {violation.check_id}: "
                f"{violation.message}\n  {violation.text}",
                file=sys.stderr,
            )

    if stale_allowlist:
        print("Stale API boundary allowlist entries:", file=sys.stderr)
        for entry in stale_allowlist:
            print(
                f"{entry.check_id}|{entry.path}|{entry.substring}|{entry.tracking}",
                file=sys.stderr,
            )

    return 1


if __name__ == "__main__":
    sys.exit(main())
