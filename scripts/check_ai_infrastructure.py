#!/usr/bin/env python3
"""Validate and diagnose DART 6.20's repository-local AI infrastructure."""

from __future__ import annotations

import argparse
import ast
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path
from typing import Any
from urllib.parse import unquote

EXPECTED_AGENTS = {"dart_release_auditor", "dart_reviewer", "dart_scout"}
DIRECT_PIXI_COMMANDS = {"bash", "c++", "cmake", "ctest", "python", "python3", "sh"}
CONFIG_ONLY_CACHE_VARIABLES = {
    "BUILD_SHARED_LIBS",
    "BUILD_TESTING",
    "CMAKE_BUILD_TYPE",
    "CMAKE_INSTALL_PREFIX",
    "CMAKE_PREFIX_PATH",
    "DART_BUILD_DARTPY",
    "DART_BUILD_PROFILE",
    "DART_DISABLE_COMPILER_CACHE",
    "DART_MSVC_FORCE_RELEASE_RUNTIME",
    "DART_USE_SYSTEM_GOOGLEBENCHMARK",
    "DART_USE_SYSTEM_GOOGLETEST",
    "DART_USE_SYSTEM_IMGUI",
    "DART_USE_SYSTEM_PYBIND11",
    "DART_USE_SYSTEM_TRACY",
    "DART_VERBOSE",
}
APPROVED_TASK_SHELL_VARIABLES = {
    "BUILD_TYPE",
    "CONDA_PREFIX",
    "DART_DISABLE_COMPILER_CACHE",
    "DART_VERBOSE",
    "PIXI_ENVIRONMENT_NAME",
    "PIXI_PROJECT_ROOT",
}
REFERENCE_ROOTS = (
    ".agents/",
    ".claude/",
    ".codex/",
    ".opencode/",
    "dart/",
    "docs/",
    "python/",
    "scripts/",
    "tests/",
)
MAX_AGENT_INSTRUCTION_BYTES = 32 * 1024
BRANCH_PROFILE_KEYS = {
    "schema_version",
    "profile",
    "base_ref",
    "cpp_standard",
    "python_binding",
    "io_namespace",
    "gui_backend",
    "required_markers",
    "required_paths",
    "forbidden_markers",
    "forbidden_paths",
    "downstream_gates",
}
SCENARIO_TOP_LEVEL_KEYS = {"schema_version", "profile", "scenarios"}
SCENARIO_KEYS = {
    "id",
    "prompt_class",
    "start_dir",
    "instruction_chain",
    "expected_route",
    "specialist_agent",
    "owner_docs",
    "permitted_scopes",
    "focused_gates",
    "full_gates",
    "recovery",
    "forbidden_paths",
}
SCENARIO_OPTIONAL_KEYS = {"evidence_policy", "semantic_review_policy"}
ROUTE_KEYS = {"kind", "name", "path"}
APPROVED_INACTIVE_CPP_TESTS = {
    "tests/regression/test_Issue2516.cpp": {
        "owner": "tests/regression/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": (
            "if:TARGET dart-external-imgui",
            "if:NOT DART_USE_SYSTEM_IMGUI",
        ),
        "cache": {"DART_USE_SYSTEM_IMGUI": "ON"},
    },
    "tests/regression/test_Issue2668.cpp": {
        "owner": "tests/regression/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": (
            "if:TARGET dart-external-imgui",
            "if:NOT DART_USE_SYSTEM_IMGUI",
        ),
        "cache": {"DART_USE_SYSTEM_IMGUI": "ON"},
    },
    "tests/integration/test_VskParser.cpp": {
        "owner": "tests/integration/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": ("if:TARGET dart-utils", "if:NOT MSVC"),
        "msvc": True,
    },
    "tests/integration/test_DartLoader.cpp": {
        "owner": "tests/integration/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": ("if:TARGET dart-utils-urdf", "if:NOT MSVC"),
        "msvc": True,
    },
    "tests/integration/test_IkFast.cpp": {
        "owner": "tests/integration/CMakeLists.txt",
        "command": "dart_add_test",
        "scopes": (
            "if:TARGET dart-utils-urdf",
            "if:BUILD_SHARED_LIBS",
        ),
        "cache": {"BUILD_SHARED_LIBS": "OFF"},
    },
}
APPROVED_CTEST_SELECTIONS = {
    "test_ConstraintSolver": {
        "arguments": (),
        "environment": ("GTEST_FILTER=ConstraintSolver.*",),
    },
    "test_AdaptiveSoftContactModel": {
        "arguments": ("--gtest_filter=AdaptiveSoftContactModelTest.*",),
        "environment": (),
    },
    "test_SoftWormModel": {
        "arguments": ("--gtest_filter=SoftWormModelTest.*",),
        "environment": (),
    },
    "test_ContactArrowLayout": {
        "arguments": ("--gtest_filter=ContactArrowLayoutTest.*",),
        "environment": (),
    },
}
SAFE_ZERO_TEST_PATTERN = r"(^|[^0-9])0 tests"
RESULT_NEUTRALIZING_CTEST_PROPERTIES = {
    "PASS_REGULAR_EXPRESSION",
    "SKIP_REGULAR_EXPRESSION",
    "SKIP_RETURN_CODE",
    "WILL_FAIL",
}
EXPECTED_PYTEST_INI_OPTIONS = {
    "minversion": "6.0",
    "addopts": ["-ra", "--showlocals", "--strict-markers", "--strict-config"],
    "xfail_strict": True,
    "filterwarnings": ["error"],
    "testpaths": ["tests"],
}
CODEX_HOOK_COMMAND = (
    'repo_root="$(git rev-parse --show-toplevel)" && '
    'CLAUDE_PROJECT_DIR="$repo_root" CODEX_PROJECT_DIR="$repo_root" '
    '"$repo_root/.claude/hooks/pre-commit-guard.sh"'
)
CODEX_HOOK_COMMAND_WINDOWS = (
    "powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass -Command "
    '"$payload = [Console]::In.ReadToEnd(); $payload | & (Join-Path (git rev-parse --show-toplevel) '
    "'.claude/hooks/pre-commit-guard.ps1'); "
    'if (-not $?) { exit 2 }; if ($LASTEXITCODE -ne 0) { exit 2 }; exit 0"'
)
CODEX_HOOK_TIMEOUT = 15
CODEX_HOOK_STATUS = "Checking DART commit command"
CLAUDE_HOOK_COMMAND = '"${CLAUDE_PROJECT_DIR}/.claude/hooks/pre-commit-guard.sh"'
WINDOWS_LAUNCHER_MARKERS = (
    "scripts/pretool_guard_bridge.py",
    "--root",
    ".pixi/envs/default/python.exe",
    "Get-Command py",
    "Get-Command python",
    "pixi run python scripts/setup_ai.py",
    "$payload = [Console]::In.ReadToEnd()",
    "$pipelineInput = @($input | ForEach-Object { $_ })",
    "$pipelineInput.Count -gt 0",
    "$payload | &",
    "$OutputEncoding = New-Object System.Text.UTF8Encoding($false)",
    "$OutputEncoding = $previousOutputEncoding",
    '$ErrorActionPreference = "Continue"',
    "$ErrorActionPreference = $previousErrorActionPreference",
    "$global:LASTEXITCODE = $null",
    "$nativeExitCode = $global:LASTEXITCODE",
    "$null -eq $nativeExitCode",
    "[Console]::Error.WriteLine",
    "exit 2",
)
WINDOWS_BRIDGE_MARKERS = (
    "def validate_payload(payload: bytes)",
    "def may_invoke_git_commit(command: str) -> bool",
    "if not may_invoke_git_commit(command):",
    'env["CLAUDE_PROJECT_DIR"] = str(root)',
    'env["CODEX_PROJECT_DIR"] = str(root)',
    'env["DART_HOOK_PYTHON"]',
    "return 0 if result.returncode == 0 else 2",
)


def read_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def read_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as stream:
        return tomllib.load(stream)


def nonempty_string_list(value: Any) -> bool:
    """Return whether value is a non-empty list of non-empty strings."""
    return (
        isinstance(value, list)
        and bool(value)
        and all(isinstance(item, str) and item.strip() for item in value)
    )


def repository_relative_path(root: Path, value: Any) -> Path | None:
    """Return a normalized repository-relative path, or None when unsafe."""
    if not isinstance(value, str) or not value.strip():
        return None
    candidate = Path(value)
    if (
        candidate.is_absolute()
        or ".." in candidate.parts
        or candidate.as_posix() != value
    ):
        return None
    try:
        (root / candidate).resolve().relative_to(root.resolve())
    except (OSError, ValueError):
        return None
    return candidate


def required_paths(root: Path) -> list[Path]:
    paths = [
        root / ".agents" / "AGENTS.md",
        root / ".agents" / "skills" / ".dart-generated.json",
        root / ".codex" / "AGENTS.md",
        root / ".codex" / "config.toml",
        root / ".codex" / "hooks.json",
        root / ".claude" / "hooks" / "pre-commit-guard.ps1",
        root / "docs" / "ai" / "agent-scenarios.json",
        root / "docs" / "ai" / "branch-profile.json",
        root / "docs" / "onboarding" / "architecture.md",
        root / "scripts" / "check_agent_hook.py",
        root / "scripts" / "check_ai_infrastructure.py",
        root / "scripts" / "pretool_guard_bridge.py",
        root / "scripts" / "run_pytest.py",
        root / "scripts" / "setup_ai.py",
    ]
    paths.extend(
        root / ".codex" / "agents" / f"{name}.toml" for name in sorted(EXPECTED_AGENTS)
    )
    return paths


def collect_task_names(value: Any) -> set[str]:
    tasks: set[str] = set()
    if not isinstance(value, dict):
        return tasks
    for key, child in value.items():
        if key == "tasks" and isinstance(child, dict):
            tasks.update(child)
        tasks.update(collect_task_names(child))
    return tasks


def collect_task_commands(value: Any, task_name: str) -> list[str]:
    commands: list[str] = []
    if not isinstance(value, dict):
        return commands
    tasks = value.get("tasks")
    if isinstance(tasks, dict):
        task = tasks.get(task_name)
        if isinstance(task, dict):
            command = task.get("cmd")
            if isinstance(command, str):
                commands.append(command)
            elif isinstance(command, list) and all(
                isinstance(item, str) for item in command
            ):
                commands.append(" ".join(command))
    for child in value.values():
        commands.extend(collect_task_commands(child, task_name))
    return commands


def collect_task_definitions(value: Any, task_name: str) -> list[dict[str, Any]]:
    """Return every platform/feature definition of a named Pixi task."""
    definitions: list[dict[str, Any]] = []
    if not isinstance(value, dict):
        return definitions
    tasks = value.get("tasks")
    if isinstance(tasks, dict):
        task = tasks.get(task_name)
        if isinstance(task, dict):
            definitions.append(task)
    for child in value.values():
        definitions.extend(collect_task_definitions(child, task_name))
    return definitions


def shell_tokens(command: str) -> list[str]:
    try:
        return shlex.split(command)
    except ValueError:
        return []


def cmake_cache_definition_values(command: str) -> dict[str, list[str]]:
    """Return every value assigned to each CMake cache variable in order."""
    definitions: dict[str, list[str]] = {}
    for token in shell_tokens(command):
        if not token.startswith("-D") or "=" not in token:
            continue
        name_with_type, value = token[2:].split("=", maxsplit=1)
        name = name_with_type.split(":", maxsplit=1)[0]
        definitions.setdefault(name, []).append(value)
    return definitions


def cmake_command_details(text: str) -> list[tuple[str, str, int]]:
    """Parse CMake command names, normalized arguments, and source lines."""

    def bracket_end(offset: int) -> int | None:
        match = re.match(r"\[(=*)\[", text[offset:])
        if match is None:
            return None
        delimiter = "]" + match.group(1) + "]"
        end = text.find(delimiter, offset + match.end())
        return len(text) if end < 0 else end + len(delimiter)

    def skip_comment(offset: int) -> int:
        bracket = bracket_end(offset + 1) if offset + 1 < len(text) else None
        if bracket is not None:
            return bracket
        newline = text.find("\n", offset + 1)
        return len(text) if newline < 0 else newline + 1

    commands: list[tuple[str, str, int]] = []
    identifier = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
    offset = 0
    while offset < len(text):
        if text[offset] == "#":
            offset = skip_comment(offset)
            continue
        match = identifier.match(text, offset)
        if match is None:
            offset += 1
            continue
        name = match.group(0).lower()
        cursor = match.end()
        while cursor < len(text):
            if text[cursor].isspace():
                cursor += 1
                continue
            if text[cursor] == "#":
                cursor = skip_comment(cursor)
                continue
            break
        if cursor >= len(text) or text[cursor] != "(":
            offset = match.end()
            continue

        depth = 1
        cursor += 1
        arguments: list[str] = []
        while cursor < len(text) and depth:
            character = text[cursor]
            if character == "#":
                cursor = skip_comment(cursor)
                arguments.append(" ")
                continue
            if character == '"':
                start = cursor
                cursor += 1
                while cursor < len(text):
                    if text[cursor] == "\\" and cursor + 1 < len(text):
                        cursor += 2
                        continue
                    cursor += 1
                    if text[cursor - 1] == '"':
                        break
                arguments.append(text[start:cursor])
                continue
            bracket = bracket_end(cursor) if character == "[" else None
            if bracket is not None:
                arguments.append(text[cursor:bracket])
                cursor = bracket
                continue
            if character == "(":
                depth += 1
                arguments.append(character)
                cursor += 1
                continue
            if character == ")":
                depth -= 1
                if depth:
                    arguments.append(character)
                cursor += 1
                continue
            arguments.append(character)
            cursor += 1
        normalized = " ".join("".join(arguments).split())
        line = text.count("\n", 0, match.start()) + 1
        commands.append((name, normalized, line))
        offset = cursor
    return commands


def cmake_commands(text: str) -> list[tuple[str, str]]:
    """Parse CMake command names and normalized arguments."""
    return [(name, arguments) for name, arguments, _ in cmake_command_details(text)]


def cmake_scoped_command_details(
    text: str,
) -> list[tuple[str, str, tuple[str, ...], int]]:
    """Attach each CMake command and line to its lexical control scopes."""
    records: list[tuple[str, str, tuple[str, ...], int]] = []
    stack: list[tuple[str, str]] = []
    openers = {"block", "foreach", "function", "if", "macro", "while"}
    closers = {f"end{name}" for name in openers}
    for name, arguments, line in cmake_command_details(text):
        if name in openers:
            stack.append((name, arguments))
            continue
        if name in closers:
            if stack:
                stack.pop()
            continue
        if name in {"else", "elseif"}:
            if stack and stack[-1][0] in {"if", "else", "elseif"}:
                condition = stack[-1][1]
                stack[-1] = (name, condition)
            continue
        context = tuple(f"{kind}:{value}" for kind, value in stack)
        records.append((name, arguments, context, line))
    return records


def cmake_scoped_commands(text: str) -> list[tuple[str, str, tuple[str, ...]]]:
    """Attach each CMake command to its enclosing lexical control scopes."""
    return [
        (name, arguments, context)
        for name, arguments, context, _ in cmake_scoped_command_details(text)
    ]


def cmake_scoped_command_origin_details(
    text: str,
) -> list[tuple[str, str, tuple[tuple[str, str, int], ...], int]]:
    """Attach commands to lexical scopes including each opener's source line."""
    records: list[tuple[str, str, tuple[tuple[str, str, int], ...], int]] = []
    stack: list[tuple[str, str, int]] = []
    openers = {"block", "foreach", "function", "if", "macro", "while"}
    closers = {f"end{name}" for name in openers}
    for name, arguments, line in cmake_command_details(text):
        if name in openers:
            stack.append((name, arguments, line))
            continue
        if name in closers:
            if stack:
                stack.pop()
            continue
        if name in {"else", "elseif"}:
            if stack and stack[-1][0] in {"if", "else", "elseif"}:
                condition = stack[-1][1]
                stack[-1] = (name, condition, line)
            continue
        records.append((name, arguments, tuple(stack), line))
    return records


def cmake_argument_tokens(arguments: str) -> list[str]:
    """Split normalized CMake arguments while keeping bracket values opaque."""

    def bracket_end(offset: int) -> int | None:
        match = re.match(r"\[(=*)\[", arguments[offset:])
        if match is None:
            return None
        delimiter = "]" + match.group(1) + "]"
        end = arguments.find(delimiter, offset + match.end())
        return len(arguments) if end < 0 else end + len(delimiter)

    tokens: list[str] = []
    offset = 0
    while offset < len(arguments):
        while offset < len(arguments) and arguments[offset].isspace():
            offset += 1
        if offset >= len(arguments):
            break

        token: list[str] = []
        while offset < len(arguments) and not arguments[offset].isspace():
            if arguments[offset] == '"':
                offset += 1
                while offset < len(arguments):
                    if arguments[offset] == "\\" and offset + 1 < len(arguments):
                        token.append(arguments[offset + 1])
                        offset += 2
                        continue
                    if arguments[offset] == '"':
                        offset += 1
                        break
                    token.append(arguments[offset])
                    offset += 1
                continue

            bracket = bracket_end(offset) if arguments[offset] == "[" else None
            if bracket is not None:
                token.append(arguments[offset:bracket])
                offset = bracket
                continue

            token.append(arguments[offset])
            offset += 1
        tokens.append("".join(token))
    return tokens


def contains_token_sequence(tokens: list[str], sequence: tuple[str, ...]) -> bool:
    """Return whether one exact token sequence occurs in order."""
    if not sequence or len(sequence) > len(tokens):
        return False
    return any(
        tokens[offset : offset + len(sequence)] == list(sequence)
        for offset in range(len(tokens) - len(sequence) + 1)
    )


def has_shell_control_syntax(command: str) -> bool:
    """Reject syntax that can hide another command from token validation."""
    if re.search(r"[\r\n;&|<>`]|[$][(]", command.strip()) is not None:
        return True

    variable_pattern = re.compile(
        r"[$](?:[{]([A-Za-z_][A-Za-z0-9_]*)[}]|" r"([A-Za-z_][A-Za-z0-9_]*))"
    )
    matches = {match.start(): match for match in variable_pattern.finditer(command)}
    for position, character in enumerate(command):
        if character != "$":
            continue
        match = matches.get(position)
        if match is None:
            return True
        variable = match.group(1) or match.group(2)
        if variable not in APPROVED_TASK_SHELL_VARIABLES:
            return True
        if not is_inside_double_quotes(command, position):
            return True
    return False


def is_inside_double_quotes(value: str, position: int) -> bool:
    in_single_quotes = False
    in_double_quotes = False
    escaped = False
    for character in value[:position]:
        if escaped:
            escaped = False
        elif character == "\\" and not in_single_quotes:
            escaped = True
        elif character == "'" and not in_double_quotes:
            in_single_quotes = not in_single_quotes
        elif character == '"' and not in_single_quotes:
            in_double_quotes = not in_double_quotes
    return in_double_quotes


def is_single_cmake_target_build(command: str, target: str) -> bool:
    """Return whether a task is exactly one CMake build of one named target."""
    tokens = shell_tokens(command)
    return (
        not has_shell_control_syntax(command)
        and tokens[:2] == ["cmake", "--build"]
        and tokens.count("cmake") == 1
        and tokens.count("--target") == 1
        and tokens[-2:] == ["--target", target]
        and "--" not in tokens
    )


def is_single_cmake_all_build(command: str) -> bool:
    """Return whether a task is exactly one CMake build of only target ALL."""
    return is_single_cmake_target_build(command, "ALL")


def is_configuration_only_command(command: str) -> bool:
    """Return whether a prerequisite only configures and never builds/tests."""
    tokens = shell_tokens(command)
    if not tokens or has_shell_control_syntax(command) or tokens[0] != "cmake":
        return False

    index = 1
    if index < len(tokens) and tokens[index] == "-G":
        if tokens[index : index + 2] != ["-G", "Ninja"]:
            return False
        index += 2
    if tokens[index : index + 1] != ["-S"] or index + 1 >= len(tokens):
        return False
    if tokens[index + 1] != ".":
        return False
    index += 2
    if tokens[index : index + 1] != ["-B"] or index + 1 >= len(tokens):
        return False
    index += 2
    definitions = tokens[index:]
    return all(
        token.startswith("-D")
        and "=" in token
        and token[2:].split("=", maxsplit=1)[0].split(":", maxsplit=1)[0]
        in CONFIG_ONLY_CACHE_VARIABLES
        for token in definitions
    )


def source_paths(root: Path) -> list[Path]:
    paths = [
        root / "AGENTS.md",
        root / "docs" / "AGENTS.md",
        root / "docs" / "README.md",
        root / "docs" / "information-architecture.md",
        root / "docs" / "dev_tasks" / "README.md",
        root / "docs" / "onboarding" / "ai-tools.md",
        root / "docs" / "onboarding" / "architecture.md",
        root / "docs" / "onboarding" / "contributing.md",
        root / "docs" / "onboarding" / "release-management.md",
        root / "docs" / "onboarding" / "testing.md",
    ]
    paths.extend(sorted((root / ".claude" / "commands").glob("*.md")))
    paths.extend(sorted((root / ".claude" / "skills").glob("*/SKILL.md")))
    paths.extend(sorted((root / "docs" / "ai").glob("*.md")))
    paths.extend(
        path
        for path in root.rglob("AGENTS.md")
        if not any(
            part in {".git", ".pixi", "build", "external", "node_modules"}
            for part in path.relative_to(root).parts
        )
    )
    return sorted(set(paths))


def operational_context_paths(root: Path) -> list[Path]:
    """Return durable session/handoff surfaces that may direct agent commands."""
    paths = list(source_paths(root))
    task_root = root / "docs" / "dev_tasks"
    if task_root.is_dir():
        for task in sorted(path for path in task_root.iterdir() if path.is_dir()):
            paths.extend(sorted(task.glob("*.md")))
    return sorted(set(paths))


def check_required_files(root: Path, errors: list[str]) -> None:
    for path in required_paths(root):
        if not path.is_file():
            errors.append(f"missing required file: {path.relative_to(root)}")


def check_required_files_are_not_ignored(root: Path, errors: list[str]) -> None:
    """Prevent a locally green check for runtime files that a PR cannot carry."""
    inside_worktree = subprocess.run(
        ["git", "rev-parse", "--is-inside-work-tree"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    if inside_worktree.returncode != 0:
        return
    for path in required_paths(root):
        relative = path.relative_to(root)
        result = subprocess.run(
            ["git", "check-ignore", "--quiet", "--", str(relative)],
            cwd=root,
        )
        if result.returncode == 0:
            errors.append(
                f"{relative}: ignored by git; add a scoped repository exception"
            )


def check_branch_profile(
    root: Path, errors: list[str], profile_data: dict[str, Any] | None = None
) -> None:
    path = root / "docs" / "ai" / "branch-profile.json"
    if not path.exists():
        return
    if profile_data is None:
        try:
            profile = read_json(path)
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f"{path.relative_to(root)}: invalid JSON: {error}")
            return
    else:
        profile = profile_data

    expected = {
        "schema_version": 1,
        "profile": "release-6.20",
        "base_ref": "origin/release-6.20",
        "cpp_standard": "C++17",
        "python_binding": "pybind11",
        "io_namespace": "dart::utils",
        "gui_backend": "OSG",
    }
    if not isinstance(profile, dict):
        errors.append(f"{path.relative_to(root)}: top level must be an object")
        return
    if set(profile) != BRANCH_PROFILE_KEYS:
        errors.append(
            f"{path.relative_to(root)}: keys must be {sorted(BRANCH_PROFILE_KEYS)}"
        )
    for key, value in expected.items():
        actual = profile.get(key)
        if type(actual) is not type(value) or actual != value:
            errors.append(
                f"{path.relative_to(root)}: `{key}` must be {value!r}, "
                f"got {actual!r}"
            )
    if profile.get("downstream_gates") != ["pixi run -e gazebo test-gz"]:
        errors.append(
            f"{path.relative_to(root)}: downstream_gates must contain the "
            "runnable Gazebo command"
        )
    string_lists: dict[str, list[str]] = {}
    for field in (
        "required_markers",
        "required_paths",
        "forbidden_markers",
        "forbidden_paths",
        "downstream_gates",
    ):
        value = profile.get(field)
        if not nonempty_string_list(value):
            errors.append(
                f"{path.relative_to(root)}: `{field}` must be a non-empty "
                "string list"
            )
            string_lists[field] = []
        else:
            string_lists[field] = value

    marker_owners = (
        root / "AGENTS.md",
        root / "docs" / "ai" / "README.md",
        root / "docs" / "onboarding" / "architecture.md",
    )
    marker_content = "\n".join(
        owner.read_text(encoding="utf-8") for owner in marker_owners if owner.is_file()
    )
    for marker in string_lists["required_markers"]:
        if marker not in marker_content:
            errors.append(
                f"{path.relative_to(root)}: required marker not visible `{marker}`"
            )
    for marker in string_lists["forbidden_markers"]:
        if marker in marker_content:
            errors.append(
                f"{path.relative_to(root)}: forbidden marker is visible `{marker}`"
            )
    for required in string_lists["required_paths"]:
        relative = repository_relative_path(root, required)
        if relative is None:
            errors.append(
                f"{path.relative_to(root)}: invalid repository-relative "
                f"required path `{required}`"
            )
        elif not (root / relative).exists():
            errors.append(
                f"{path.relative_to(root)}: missing required path `{required}`"
            )
    for forbidden in string_lists["forbidden_paths"]:
        relative = repository_relative_path(root, forbidden)
        if relative is None:
            errors.append(
                f"{path.relative_to(root)}: invalid repository-relative "
                f"forbidden path `{forbidden}`"
            )
        elif (root / relative).exists():
            errors.append(
                f"{path.relative_to(root)}: forbidden DART 7 path exists `{forbidden}`"
            )


def check_codex_config(root: Path, errors: list[str]) -> None:
    config_path = root / ".codex" / "config.toml"
    if config_path.exists():
        try:
            config = read_toml(config_path)
        except (OSError, tomllib.TOMLDecodeError) as error:
            errors.append(f"{config_path.relative_to(root)}: invalid TOML: {error}")
        else:
            if set(config) != {"agents"}:
                errors.append(".codex/config.toml: root keys must equal agents")
            agents = config.get("agents", {})
            if not isinstance(agents, dict):
                errors.append(".codex/config.toml: agents must be a table")
                agents = {}
            if set(agents) != {"max_threads", "max_depth"}:
                errors.append(
                    ".codex/config.toml: agents keys must equal max_threads, "
                    "max_depth (the compatibility spelling keeps Codex 0.144 "
                    "strict config support)"
                )
            if (
                type(agents.get("max_threads")) is not int
                or agents.get("max_threads") != 4
            ):
                errors.append(
                    ".codex/config.toml: agents.max_threads must be 4 for "
                    "Codex 0.144 compatibility"
                )
            if type(agents.get("max_depth")) is not int or agents.get("max_depth") != 1:
                errors.append(".codex/config.toml: agents.max_depth must be 1")
            forbidden = {
                "model",
                "model_reasoning_effort",
                "review_model",
                "approval_policy",
                "sandbox_mode",
            } & set(config)
            if forbidden:
                errors.append(
                    ".codex/config.toml: project config must not pin "
                    f"{', '.join(sorted(forbidden))}"
                )

    agents_dir = root / ".codex" / "agents"
    actual = {path.stem for path in agents_dir.glob("*.toml")}
    if actual != EXPECTED_AGENTS:
        errors.append(
            ".codex/agents: expected "
            f"{sorted(EXPECTED_AGENTS)}, got {sorted(actual)}"
        )

    for name in sorted(EXPECTED_AGENTS):
        path = agents_dir / f"{name}.toml"
        if not path.exists():
            continue
        try:
            profile = read_toml(path)
        except (OSError, tomllib.TOMLDecodeError) as error:
            errors.append(f"{path.relative_to(root)}: invalid TOML: {error}")
            continue
        expected_keys = {
            "name",
            "description",
            "sandbox_mode",
            "developer_instructions",
        }
        if set(profile) != expected_keys:
            errors.append(
                f"{path.relative_to(root)}: keys must equal description, "
                "developer_instructions, name, sandbox_mode"
            )
        for field in ("name", "description", "developer_instructions"):
            if not isinstance(profile.get(field), str) or not profile[field].strip():
                errors.append(f"{path.relative_to(root)}: missing `{field}`")
        if profile.get("name") != name:
            errors.append(f"{path.relative_to(root)}: name must be `{name}`")
        if profile.get("sandbox_mode") != "read-only":
            errors.append(f"{path.relative_to(root)}: sandbox_mode must be read-only")
        if {"model", "model_reasoning_effort", "review_model"} & set(profile):
            errors.append(f"{path.relative_to(root)}: inherit the parent model")
        instructions = profile.get("developer_instructions", "")
        if not isinstance(instructions, str):
            instructions = ""
        if "Inputs" not in instructions or "Output" not in instructions:
            errors.append(
                f"{path.relative_to(root)}: instructions need Inputs and Output contracts"
            )


def check_hooks(root: Path, errors: list[str]) -> None:
    hooks_path = root / ".codex" / "hooks.json"
    if hooks_path.exists():
        try:
            data = read_json(hooks_path)
        except (OSError, json.JSONDecodeError) as error:
            errors.append(
                f"{hooks_path.relative_to(root)}: invalid hook schema: {error}"
            )
        else:
            if not isinstance(data, dict) or set(data) != {"hooks"}:
                errors.append(".codex/hooks.json: root keys must equal hooks")
            hooks = data.get("hooks") if isinstance(data, dict) else None
            if not isinstance(hooks, dict) or set(hooks) != {"PreToolUse"}:
                errors.append(".codex/hooks.json: only a PreToolUse hook is allowed")
            entries = hooks.get("PreToolUse") if isinstance(hooks, dict) else None
            entry = entries[0] if isinstance(entries, list) and entries else None
            handlers = entry.get("hooks") if isinstance(entry, dict) else None
            hook = handlers[0] if isinstance(handlers, list) and handlers else None
            if (
                not isinstance(entries, list)
                or len(entries) != 1
                or not isinstance(entry, dict)
                or not isinstance(handlers, list)
                or len(handlers) != 1
                or not isinstance(hook, dict)
            ):
                errors.append(".codex/hooks.json: expected one bounded PreToolUse hook")
                hook = None
            if isinstance(entry, dict) and set(entry) != {"matcher", "hooks"}:
                errors.append(
                    ".codex/hooks.json: matcher keys must equal hooks, matcher"
                )
            if not isinstance(entry, dict) or entry.get("matcher") != "^Bash$":
                errors.append(".codex/hooks.json: matcher must be `^Bash$`")
            command = hook.get("command", "") if hook else ""
            expected_keys = {
                "type",
                "command",
                "commandWindows",
                "timeout",
                "statusMessage",
            }
            if isinstance(hook, dict) and set(hook) != expected_keys:
                errors.append(
                    ".codex/hooks.json: handler keys must equal command, "
                    "commandWindows, statusMessage, timeout, type"
                )
            if not hook or hook.get("type") != "command":
                errors.append(".codex/hooks.json: hook type must be command")
            if command != CODEX_HOOK_COMMAND:
                errors.append(".codex/hooks.json: hook command wiring is stale")
            command_windows = hook.get("commandWindows", "") if hook else ""
            if command_windows != CODEX_HOOK_COMMAND_WINDOWS:
                errors.append(".codex/hooks.json: Windows hook command wiring is stale")
            timeout = hook.get("timeout") if hook else None
            if type(timeout) is not int or timeout != CODEX_HOOK_TIMEOUT:
                errors.append(
                    f".codex/hooks.json: timeout must equal {CODEX_HOOK_TIMEOUT}"
                )
            status = hook.get("statusMessage") if hook else None
            if status != CODEX_HOOK_STATUS:
                errors.append(
                    ".codex/hooks.json: statusMessage must equal the canonical text"
                )

    settings_path = root / ".claude" / "settings.json"
    if settings_path.exists():
        try:
            settings = read_json(settings_path)
        except (OSError, json.JSONDecodeError) as error:
            errors.append(
                f"{settings_path.relative_to(root)}: invalid hook schema: {error}"
            )
        else:
            hooks = settings.get("hooks") if isinstance(settings, dict) else None
            entries = hooks.get("PreToolUse") if isinstance(hooks, dict) else None
            entry = entries[0] if isinstance(entries, list) and entries else None
            handlers = entry.get("hooks") if isinstance(entry, dict) else None
            handler = handlers[0] if isinstance(handlers, list) and handlers else None
            command = handler.get("command") if isinstance(handler, dict) else None
            if (
                not isinstance(handler, dict)
                or set(handler) != {"type", "command", "shell"}
                or handler.get("type") != "command"
                or command != CLAUDE_HOOK_COMMAND
                or handler.get("shell") != "bash"
            ):
                errors.append(".claude/settings.json: invalid or stale hook wiring")

    installer = root / "scripts" / "install_git_hooks.py"
    guard = root / ".claude" / "hooks" / "pre-commit-guard.sh"
    for path in (installer, guard):
        content = path.read_text(encoding="utf-8") if path.exists() else ""
        if (
            "scripts/check_agent_hook.py" not in content
            or "--profile staged" not in content
        ):
            errors.append(
                f"{path.relative_to(root)}: must invoke the staged agent hook profile"
            )
    launcher = root / ".claude" / "hooks" / "pre-commit-guard.ps1"
    launcher_text = launcher.read_text(encoding="utf-8") if launcher.exists() else ""
    for marker in WINDOWS_LAUNCHER_MARKERS:
        if marker not in launcher_text:
            errors.append(
                ".claude/hooks/pre-commit-guard.ps1: missing required marker "
                f"{marker!r}"
            )
    bridge = root / "scripts" / "pretool_guard_bridge.py"
    bridge_text = bridge.read_text(encoding="utf-8") if bridge.exists() else ""
    for marker in WINDOWS_BRIDGE_MARKERS:
        if marker not in bridge_text:
            errors.append(
                "scripts/pretool_guard_bridge.py: missing required marker "
                f"{marker!r}"
            )


def check_pixi_references(root: Path, errors: list[str]) -> None:
    try:
        pixi = read_toml(root / "pixi.toml")
    except (OSError, tomllib.TOMLDecodeError) as error:
        errors.append(f"pixi.toml: invalid TOML: {error}")
        return
    tasks = collect_task_names(pixi)
    run_pattern = re.compile(
        r"pixi run"
        r"(?:\s+(?:-e|--environment)\s+\S+)?"
        r"(?:\s+--skip-deps)?"
        r"(?:\s+--)?"
        r"\s+([A-Za-z0-9_][A-Za-z0-9_+.-]*)"
    )
    for path in operational_context_paths(root):
        if not path.exists():
            continue
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            for task in run_pattern.findall(line):
                if task in tasks or task in DIRECT_PIXI_COMMANDS:
                    continue
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: unknown Pixi task `{task}`"
                )


def test_graph_scope_requirements() -> dict[str, tuple[tuple[Any, ...], ...]]:
    """Return the source contracts that own the broad CMake runtime graph."""
    return {
        "CMakeLists.txt": (
            (
                "add_custom_target",
                "ALL DEPENDS ${all_targets}",
                (),
                (),
                "add_custom_target(ALL DEPENDS ${all_targets})",
            ),
            (
                "list",
                "APPEND all_target_candidates tests_and_run pytest",
                (),
                ("if:BUILD_TESTING",),
                "list(APPEND all_target_candidates tests_and_run pytest)",
            ),
        ),
        "tests/CMakeLists.txt": (
            (
                "add_custom_target",
                None,
                (
                    (
                        "tests_and_run",
                        "COMMAND",
                        "${CMAKE_COMMAND}",
                        "-DDART_CTEST_COMMAND=${CMAKE_CTEST_COMMAND}",
                        "-DDART_CTEST_CONFIGURATION=$<CONFIG>",
                        "-P",
                        "${PROJECT_SOURCE_DIR}/cmake/DARTRunCTest.cmake",
                    ),
                    (
                        "DEPENDS",
                        "${integration_tests}",
                        "${regression_tests}",
                        "${unit_tests}",
                    ),
                ),
                (),
                "cmake/DARTRunCTest.cmake",
            ),
        ),
        "python/tests/CMakeLists.txt": (
            (
                "add_custom_target",
                None,
                (
                    ("pytest",),
                    (
                        "COMMAND",
                        "${Python3_EXECUTABLE}",
                        "-I",
                        "${PROJECT_SOURCE_DIR}/scripts/run_pytest.py",
                        "--pythonpath",
                        "${DART_PYTHONPATH}",
                        "${dartpy_test_files}",
                        "-v",
                    ),
                    ("DEPENDS", "dartpy"),
                ),
                ("if:DARTPY_PYTEST_FOUND",),
                "scripts/run_pytest.py",
            ),
            (
                "add_custom_target",
                None,
                (
                    ("pytest", "COMMAND", "${CMAKE_COMMAND}", "-E", "echo"),
                    ("Warning: Failed to run pytest because pytest is not found!",),
                ),
                ("else:DARTPY_PYTEST_FOUND",),
                "pytest",
            ),
        ),
    }


def cmake_graph_command_matches(
    record: tuple[str, str, tuple[str, ...]],
    requirement: tuple[Any, ...],
) -> bool:
    """Return whether one parsed command satisfies a graph source contract."""
    record_name, arguments, context = record
    (
        name,
        exact_arguments,
        argument_token_sequences,
        expected_scope,
        _,
    ) = requirement
    if record_name != name or context != expected_scope:
        return False
    if exact_arguments is not None:
        return arguments == exact_arguments
    if not argument_token_sequences:
        return False
    tokens = cmake_argument_tokens(arguments)
    return tokens[: len(argument_token_sequences[0])] == list(
        argument_token_sequences[0]
    ) and all(
        contains_token_sequence(tokens, sequence)
        for sequence in argument_token_sequences[1:]
    )


def check_ctest_runner_contract(root: Path, errors: list[str]) -> None:
    """Require the broad CTest wrapper to clear every ambient GTEST_* variable."""
    prefix = "cmake/DARTRunCTest.cmake"
    path = root / prefix
    try:
        records = cmake_command_details(path.read_text(encoding="utf-8"))
    except OSError as error:
        errors.append(f"{prefix}: unable to inspect clean-environment runner: {error}")
        return
    actual = tuple(
        (name, tuple(cmake_argument_tokens(arguments)))
        for name, arguments, _ in records
    )
    expected = (
        (
            "if",
            (
                "NOT",
                "DEFINED",
                "DART_CTEST_COMMAND",
                "OR",
                "DART_CTEST_COMMAND",
                "STREQUAL",
                "",
            ),
        ),
        (
            "message",
            (
                "FATAL_ERROR",
                "DART_CTEST_COMMAND must name the configured CTest executable",
            ),
        ),
        ("endif", ()),
        (
            "execute_process",
            (
                "COMMAND",
                "${CMAKE_COMMAND}",
                "-E",
                "environment",
                "OUTPUT_VARIABLE",
                "_dart_test_environment",
                "RESULT_VARIABLE",
                "_dart_environment_result",
            ),
        ),
        ("if", ("NOT", "_dart_environment_result", "EQUAL", "0")),
        ("message", ("FATAL_ERROR", "Failed to inspect the test environment")),
        ("endif", ()),
        (
            "string",
            (
                "REPLACE",
                "rn",
                "n",
                "_dart_test_environment",
                "${_dart_test_environment}",
            ),
        ),
        (
            "string",
            (
                "REPLACE",
                "r",
                "n",
                "_dart_test_environment",
                "${_dart_test_environment}",
            ),
        ),
        (
            "string",
            (
                "REPLACE",
                ";",
                "\\;",
                "_dart_test_environment",
                "${_dart_test_environment}",
            ),
        ),
        (
            "string",
            (
                "REPLACE",
                "n",
                ";",
                "_dart_test_environment",
                "${_dart_test_environment}",
            ),
        ),
        ("set", ("_dart_gtest_unsets",)),
        (
            "foreach",
            ("_dart_environment_entry", "IN", "LISTS", "_dart_test_environment"),
        ),
        (
            "string",
            ("FIND", "${_dart_environment_entry}", "=", "_dart_separator"),
        ),
        ("if", ("_dart_separator", "GREATER", "0")),
        (
            "string",
            (
                "SUBSTRING",
                "${_dart_environment_entry}",
                "0",
                "${_dart_separator}",
                "_dart_name",
            ),
        ),
        ("string", ("TOUPPER", "${_dart_name}", "_dart_name_upper")),
        ("if", ("_dart_name_upper", "MATCHES", "^GTEST_")),
        ("list", ("APPEND", "_dart_gtest_unsets", "--unset=${_dart_name}")),
        ("endif", ()),
        ("endif", ()),
        ("endforeach", ()),
        ("set", ("_dart_ctest_arguments", "--output-on-failure")),
        (
            "if",
            (
                "DEFINED",
                "DART_CTEST_CONFIGURATION",
                "AND",
                "NOT",
                "DART_CTEST_CONFIGURATION",
                "STREQUAL",
                "",
            ),
        ),
        (
            "list",
            (
                "APPEND",
                "_dart_ctest_arguments",
                "-C",
                "${DART_CTEST_CONFIGURATION}",
            ),
        ),
        ("endif", ()),
        (
            "execute_process",
            (
                "COMMAND",
                "${CMAKE_COMMAND}",
                "-E",
                "env",
                "${_dart_gtest_unsets}",
                "${DART_CTEST_COMMAND}",
                "${_dart_ctest_arguments}",
                "COMMAND_ERROR_IS_FATAL",
                "ANY",
            ),
        ),
    )
    if actual != expected:
        errors.append(
            f"{prefix}: commands and lexical control flow must exactly match "
            "the canonical clean-environment runner"
        )


def check_dartpy_runtime_path_contract(root: Path, errors: list[str]) -> None:
    """Keep pytest imports tied to the active dartpy target configuration."""
    relative = "python/CMakeLists.txt"
    path = root / relative
    try:
        text = path.read_text(encoding="utf-8")
    except OSError as error:
        errors.append(f"{relative}: unable to read dartpy runtime path: {error}")
        return

    records = cmake_scoped_commands(text)
    requirements = (
        (
            ("add_subdirectory", "dartpy", ()),
            "dartpy_target",
            "`add_subdirectory(dartpy)` must define the binding target",
        ),
        (
            (
                "set",
                'DART_DARTPY_BUILD_DIR "$<TARGET_FILE_DIR:dartpy>"',
                (),
            ),
            "dartpy_output",
            "`DART_DARTPY_BUILD_DIR` must derive from `$<TARGET_FILE_DIR:dartpy>`",
        ),
        (
            (
                "set",
                "DART_PYTHONPATH "
                '"${DART_DARTPY_BUILD_DIR}\\\\;${DART_PYTHON_BUILD_DIR}"',
                ("if:WIN32",),
            ),
            "windows_path",
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under `if(WIN32)`",
        ),
        (
            (
                "set",
                "DART_PYTHONPATH "
                '"${DART_DARTPY_BUILD_DIR}:${DART_PYTHON_BUILD_DIR}"',
                ("else:WIN32",),
            ),
            "posix_path",
            "`DART_PYTHONPATH` must combine the target output directory and "
            "Python build root under the non-Windows branch",
        ),
        (
            ("add_subdirectory", "tests", ()),
            "pytest_target",
            "`add_subdirectory(tests)` must define the dartpy test target",
        ),
    )
    positions: dict[str, int] = {}
    for required, key, message in requirements:
        try:
            positions[key] = records.index(required)
        except ValueError:
            errors.append(f"{relative}: {message}")

    if len(positions) != len(requirements):
        return
    path_positions = (positions["windows_path"], positions["posix_path"])
    if not (
        positions["dartpy_target"] < positions["dartpy_output"] < min(path_positions)
        and max(path_positions) < positions["pytest_target"]
    ):
        errors.append(
            f"{relative}: define `dartpy`, derive its configuration-aware "
            "output path, compose both platform paths, and then define tests"
        )


def check_test_gate_contract(root: Path, errors: list[str]) -> None:
    """Keep the Release ALL graph and focused test gates unambiguous."""
    try:
        pixi = read_toml(root / "pixi.toml")
    except (OSError, tomllib.TOMLDecodeError) as error:
        errors.append(f"pixi.toml: invalid TOML: {error}")
        return

    task_markers = {
        "test": "--target tests_and_run",
        "test-py": "pytest",
        "test-all": "--target ALL",
    }
    for task, marker in task_markers.items():
        commands = collect_task_commands(pixi, task)
        if not commands:
            errors.append(f"pixi.toml: missing required test task `{task}`")
        elif any(marker not in command for command in commands):
            errors.append(
                f"pixi.toml: `{task}` task must retain command marker `{marker}`"
            )

    test_definitions = collect_task_definitions(pixi, "test")
    test_commands = collect_task_commands(pixi, "test")
    if test_definitions and len(test_commands) != len(test_definitions):
        errors.append("pixi.toml: every `test` variant must define one command")
    if test_commands and any(
        not is_single_cmake_target_build(command, "tests_and_run")
        for command in test_commands
    ):
        errors.append(
            "pixi.toml: every `test` command must be one CMake build with "
            "the single target `--target tests_and_run`"
        )
    for definition in test_definitions:
        dependencies = definition.get("depends-on", [])
        if isinstance(dependencies, str):
            dependencies = [dependencies]
        if dependencies != ["config"]:
            errors.append(
                "pixi.toml: every `test` variant must depend only on "
                "the build configuration task `config`"
            )
            break
        if definition.get("env") != {"BUILD_TYPE": "Release"}:
            errors.append(
                "pixi.toml: every `test` variant must set only "
                '`BUILD_TYPE = "Release"`'
            )
            break

    test_all_definitions = collect_task_definitions(pixi, "test-all")
    test_all_commands = collect_task_commands(pixi, "test-all")
    if test_all_definitions and len(test_all_commands) != len(test_all_definitions):
        errors.append("pixi.toml: every `test-all` variant must define one command")
    if (
        test_all_commands
        and all("--target ALL" in command for command in test_all_commands)
        and any(not is_single_cmake_all_build(command) for command in test_all_commands)
    ):
        errors.append(
            "pixi.toml: every `test-all` command must be one CMake build with "
            "the single target `--target ALL`"
        )
    if any(
        forbidden in command.lower()
        for command in test_all_commands
        for forbidden in ("ctest", "pytest")
    ):
        errors.append(
            "pixi.toml: `test-all` runtime coverage must remain owned by "
            "the CMake `ALL` graph, not appended to the Pixi command"
        )
    for definition in test_all_definitions:
        environment = definition.get("env")
        if environment != {"BUILD_TYPE": "Release"}:
            errors.append(
                "pixi.toml: every `test-all` variant must set only "
                '`BUILD_TYPE = "Release"`'
            )
            break

    exact_pytest_tasks = {
        "test-ai-infra": {
            "tokens": [
                "python",
                "-I",
                "scripts/run_pytest.py",
                "tests/test_sync_ai_commands.py",
                "tests/test_ai_infrastructure.py",
                "tests/test_install_git_hooks.py",
                "-q",
            ],
            "depends-on": [],
            "env": None,
        },
        "test-agent-debug-overlay": {
            "tokens": [
                "python",
                "-I",
                "scripts/run_pytest.py",
                "--pythonpath",
                (
                    "$PIXI_PROJECT_ROOT/build/$PIXI_ENVIRONMENT_NAME/cpp/"
                    "$BUILD_TYPE/python/dartpy"
                ),
                "--pythonpath",
                "$PIXI_PROJECT_ROOT/scripts",
                "python/tests/unit/gui/test_agent_debug_overlay.py::"
                "test_contacts_layer_marks_points_and_normals",
                "python/tests/unit/gui/test_agent_debug_overlay.py::"
                "test_engine_rendered_overlay_changes_pixels",
                "python/tests/unit/gui/test_agent_capture.py::"
                "test_run_capture_smoke_writes_stills_and_sidecar",
                "python/tests/unit/gui/test_agent_capture.py::"
                "test_run_capture_debug_layers_change_pixels_end_to_end",
                "-q",
            ],
            "depends-on": ["build-py-dev"],
            "env": {"BUILD_TYPE": "Release"},
        },
    }
    for task, expected in exact_pytest_tasks.items():
        definitions = collect_task_definitions(pixi, task)
        commands = collect_task_commands(pixi, task)
        if len(definitions) != 1 or len(commands) != 1:
            errors.append(f"pixi.toml: `{task}` must have one exact task definition")
            continue
        if shell_tokens(commands[0]) != expected["tokens"]:
            errors.append(
                f"pixi.toml: `{task}` must use the guarded repository pytest runner"
            )
        dependencies = definitions[0].get("depends-on", [])
        if isinstance(dependencies, str):
            dependencies = [dependencies]
        if dependencies != expected["depends-on"]:
            errors.append(f"pixi.toml: `{task}` has unexpected prerequisite tasks")
        if definitions[0].get("env") != expected["env"]:
            errors.append(f"pixi.toml: `{task}` has unexpected environment settings")
    for definition in test_all_definitions:
        dependencies = definition.get("depends-on", [])
        if isinstance(dependencies, str):
            dependencies = [dependencies]
        if dependencies != ["config"]:
            errors.append(
                "pixi.toml: every `test-all` variant must depend only on "
                "the build configuration task `config`"
            )
            break

    ai_check_definitions = collect_task_definitions(pixi, "check-ai-infra")
    if not ai_check_definitions:
        errors.append("pixi.toml: missing required AI infrastructure check task")
    for definition in ai_check_definitions:
        command = definition.get("cmd")
        try:
            tokens = shlex.split(command) if isinstance(command, str) else []
        except ValueError:
            tokens = []
        if tokens != [
            "python",
            "scripts/check_ai_infrastructure.py",
            "--check",
            "--semantic-cmake",
        ]:
            errors.append(
                "pixi.toml: `check-ai-infra` must run the semantic CMake "
                "infrastructure check exactly"
            )
            break
        dependencies = definition.get("depends-on", [])
        if isinstance(dependencies, str):
            dependencies = [dependencies]
        if dependencies != ["config"]:
            errors.append("pixi.toml: `check-ai-infra` must depend only on `config`")
            break
        if definition.get("env") != {"BUILD_TYPE": "Release"}:
            errors.append(
                "pixi.toml: `check-ai-infra` must select only the Release "
                "configuration"
            )
            break

    config_definitions = collect_task_definitions(pixi, "config")
    if not config_definitions:
        errors.append("pixi.toml: missing required build task `config`")
    for definition in config_definitions:
        dependencies = definition.get("depends-on", [])
        if isinstance(dependencies, str):
            dependencies = [dependencies]
        if dependencies:
            errors.append(
                "pixi.toml: `config` must not depend on other tasks because "
                "`test-all` requires a direct configuration prerequisite"
            )
            break
        environment = definition.get("env")
        if environment not in (
            {"DART_VERBOSE": "OFF"},
            {"DART_VERBOSE": "OFF", "BUILD_TYPE": "Release"},
        ):
            errors.append(
                "pixi.toml: every `config` variant must use the exact "
                "branch-owned environment"
            )
            break
    config_commands = collect_task_commands(pixi, "config")
    if config_definitions and len(config_commands) != len(config_definitions):
        errors.append("pixi.toml: every `config` variant must define one command")
    elif config_commands and any(
        not is_configuration_only_command(command) for command in config_commands
    ):
        errors.append(
            "pixi.toml: every `config` command used by `test-all` must only "
            "configure CMake"
        )
    required_runtime_config_values = (
        ("BUILD_TESTING", "ON"),
        ("DART_BUILD_DARTPY", "ON"),
        ("DART_USE_SYSTEM_PYBIND11", "ON"),
    )
    for variable, expected_value in required_runtime_config_values:
        if config_commands and any(
            cmake_cache_definition_values(command).get(variable) != [expected_value]
            for command in config_commands
        ):
            errors.append(
                "pixi.toml: every `config` command used by `test-all` must pin "
                f"`{variable}={expected_value}` exactly once"
            )

    graph_scope_requirements = test_graph_scope_requirements()
    for relative, requirements in graph_scope_requirements.items():
        path = root / relative
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{relative}: unable to read test graph: {error}")
            continue
        if any(name == "return" for name, _ in cmake_commands(text)):
            errors.append(
                f"{relative}: `return()` may bypass the required `test-all` graph"
            )
        records = cmake_scoped_commands(text)
        for (
            name,
            exact_arguments,
            argument_token_sequences,
            expected_scope,
            marker,
        ) in requirements:
            requirement = (
                name,
                exact_arguments,
                argument_token_sequences,
                expected_scope,
                marker,
            )
            if not any(
                cmake_graph_command_matches(record, requirement) for record in records
            ):
                errors.append(f"{relative}: missing `test-all` graph marker `{marker}`")
    check_ctest_runner_contract(root, errors)
    check_dartpy_runtime_path_contract(root, errors)

    dependencies = pixi.get("dependencies")
    if not isinstance(dependencies, dict) or "pytest" not in dependencies:
        errors.append(
            "pixi.toml: the default environment must provide pytest for "
            "the CMake `ALL` graph"
        )

    required_doc_markers = {
        "AGENTS.md": (
            "pixi run test         # Build and run C++ tests",
            "pixi run test-py      # Run dartpy tests",
            "pixi run test-all     # Build defaults and run C++/Python tests",
        ),
        "docs/onboarding/testing.md": (
            "`pixi run test` builds the `tests_and_run` target",
            "`pixi run test-py`",
            "`pixi run test-all` builds the",
            "`tests_and_run` and `pytest`",
            "`scripts/run_pytest.py`",
            "does not run lint",
            "CMake's File API",
            "$<TARGET_FILE_DIR:dartpy>",
        ),
        "docs/onboarding/release-management.md": (
            "`pixi run test-all` for the complete default CMake graph",
            "`tests_and_run` and `pytest`",
            "`test-all` does not format or check lint",
        ),
    }
    for relative, markers in required_doc_markers.items():
        path = root / relative
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{relative}: unable to read test-gate guidance: {error}")
            continue
        for marker in markers:
            if marker not in text:
                errors.append(f"{relative}: missing branch test-gate marker `{marker}`")

    completion_gate_surfaces = (
        ".codex/AGENTS.md",
        "docs/ai/README.md",
        "docs/ai/verification.md",
    )
    structural_only_command = (
        "pixi run python scripts/check_ai_infrastructure.py --check"
    )
    for relative in completion_gate_surfaces:
        path = root / relative
        if not path.is_file():
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(
                f"{relative}: unable to read AI completion-gate guidance: {error}"
            )
            continue
        if "pixi run check-ai-infra" not in text:
            errors.append(
                f"{relative}: missing semantic AI completion gate "
                "`pixi run check-ai-infra`"
            )
        if structural_only_command in text:
            errors.append(
                f"{relative}: structural-only AI checker is presented as a "
                "completion gate"
            )

    list_item = re.compile(r"^\s*(?:[-*+]|\d+\.)\s+")
    for path in operational_context_paths(root):
        if not path.exists():
            continue
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            errors.append(
                f"{path.relative_to(root)}: unable to read operational "
                f"test-gate guidance: {error}"
            )
            continue
        blocks: list[tuple[int, list[str]]] = []
        block_start = 1
        block_lines: list[str] = []
        for line_number, line in enumerate(lines, start=1):
            if not line.strip() or (list_item.match(line) and block_lines):
                if block_lines:
                    blocks.append((block_start, block_lines))
                    block_lines = []
                if not line.strip():
                    continue
            if not block_lines:
                block_start = line_number
            block_lines.append(line)
        if block_lines:
            blocks.append((block_start, block_lines))

        for line_number, block in blocks:
            original = re.sub(r"[`*_]", "", " ".join(block))
            plain = original.lower()
            if re.search(
                r"\bnanobind\b[^.!?]{0,120}\b(?:known|poisoned)\b"
                r"[^.!?]{0,80}\b(?:cmake)?cache\b",
                plain,
            ):
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: remove stale "
                    "main-only nanobind cache guidance"
                )
            if "test-all" not in plain:
                continue
            stale_debug_claim = re.search(
                r"\btest-all\b[^.!?]{0,80}\b(?:reports?|runs?|builds?|"
                r"uses?|is)\b[^.!?]{0,40}\bdebug\b",
                plain,
            )
            if stale_debug_claim or "nanobind" in plain:
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: `test-all` is a "
                    "Release-only build task; remove Debug/nanobind guidance"
                )
            false_no_runtime_claim = any(
                re.search(pattern, plain)
                for pattern in (
                    r"\btest-all\b[^.!?]{0,100}\b(?:does\s+not|doesn.t|"
                    r"never)\s+(?:run|execute|provide|cover|include)\b"
                    r"[^.!?]{0,100}\b(?:ctest|pytest|runtime|tests?)\b",
                    r"\btest-all\b[^.!?]{0,100}\bonly\s+(?:builds?|" r"compiles?)\b",
                    r"\btest-all\b[^.!?]{0,100}\b(?:is|provides?)\s+not\b"
                    r"[^.!?]{0,60}\bruntime coverage\b",
                )
            )
            if false_no_runtime_claim:
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: `test-all` must "
                    "not be described as build-only or lacking runtime coverage"
                )
            false_lint_claim = any(
                re.search(pattern, plain)
                for pattern in (
                    r"\btest-all\s+(?:runs?|executes?|includes?|covers?)\b"
                    r"[^.!?]{0,80}\blint\b",
                    r"\btest-all\s+(?:is|provides?)\s+(?:the\s+)?full\s+"
                    r"(?:lint[/,+ ]+build[/,+ ]+test|aggregate)\b",
                )
            )
            if false_lint_claim:
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: `test-all` must "
                    "not be described as providing lint coverage"
                )


def cmake_paths_match(first: str | Path, second: str | Path) -> bool:
    """Compare CMake paths across slash and Windows case conventions."""

    def normalize(path: str | Path) -> str:
        value = str(path).replace("\\", "/").rstrip("/")
        return value.casefold() if re.match(r"^[A-Za-z]:/", value) else value

    return normalize(first) == normalize(second)


def discover_cmake_build_dir(root: Path) -> Path | None:
    """Find the Pixi default environment's configured CMake tree."""
    environment = os.environ.get("PIXI_ENVIRONMENT_NAME", "default")
    build_type = os.environ.get("BUILD_TYPE", "Release")
    base = root / "build" / environment / "cpp"
    candidates = (base,) if sys.platform == "win32" else (base / build_type,)
    for candidate in candidates:
        cache = candidate / "CMakeCache.txt"
        if not cache.is_file():
            continue
        try:
            values = cmake_cache_values(cache)
        except OSError:
            continue
        home = values.get("CMAKE_HOME_DIRECTORY")
        if isinstance(home, str) and cmake_paths_match(home, root.resolve()):
            return candidate
    return None


def cmake_cache_values(path: Path) -> dict[str, str]:
    """Read scalar values from one CMake cache."""
    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.startswith(("//", "#")) or "=" not in line:
            continue
        key_with_type, value = line.split("=", maxsplit=1)
        key = key_with_type.split(":", maxsplit=1)[0]
        values[key] = value
    return values


def cmake_target_backtrace(
    target: dict[str, Any],
) -> tuple[str, str, int] | None:
    """Return the command, source path, and line that created a target."""
    graph = target.get("backtraceGraph")
    node_index = target.get("backtrace")
    if not isinstance(graph, dict) or type(node_index) is not int:
        return None
    nodes = graph.get("nodes")
    commands = graph.get("commands")
    files = graph.get("files")
    if (
        not isinstance(nodes, list)
        or not isinstance(commands, list)
        or not isinstance(files, list)
        or not 0 <= node_index < len(nodes)
    ):
        return None
    node = nodes[node_index]
    if not isinstance(node, dict):
        return None
    command_index = node.get("command")
    file_index = node.get("file")
    line = node.get("line")
    if (
        type(command_index) is not int
        or type(file_index) is not int
        or type(line) is not int
        or not 0 <= command_index < len(commands)
        or not 0 <= file_index < len(files)
        or not isinstance(commands[command_index], str)
        or not isinstance(files[file_index], str)
    ):
        return None
    return commands[command_index], files[file_index], line


def cmake_trace_records(output: str) -> list[dict[str, Any]]:
    """Return JSON-v1 CMake trace records from mixed configure output."""
    records: list[dict[str, Any]] = []
    for line in output.splitlines():
        if not line.startswith("{"):
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(record, dict) and isinstance(record.get("cmd"), str):
            records.append(record)
    return records


def cmake_compiler_uses_msvc_abi(
    build_dir: Path, cache: dict[str, str], errors: list[str]
) -> bool | None:
    """Return CMake's MSVC-family compiler semantics from configured state."""
    prefix = "CMake semantic test graph"
    version_parts = [
        cache.get("CMAKE_CACHE_MAJOR_VERSION"),
        cache.get("CMAKE_CACHE_MINOR_VERSION"),
        cache.get("CMAKE_CACHE_PATCH_VERSION"),
    ]
    if not all(isinstance(part, str) and part.isdigit() for part in version_parts):
        errors.append(f"{prefix}: configured CMake version is unavailable")
        return None
    cmake_version = ".".join(part for part in version_parts if part is not None)
    compiler_file = build_dir / "CMakeFiles" / cmake_version / "CMakeCXXCompiler.cmake"
    if not compiler_file.is_file():
        errors.append(
            f"{prefix}: configured C++ compiler description is unavailable "
            f"at `{compiler_file}`"
        )
        return None
    try:
        records = cmake_commands(compiler_file.read_text(encoding="utf-8"))
    except OSError as error:
        errors.append(f"{prefix}: unable to inspect C++ compiler state: {error}")
        return None
    values: dict[str, str] = {}
    for name, arguments in records:
        if name != "set":
            continue
        tokens = cmake_argument_tokens(arguments)
        if len(tokens) >= 2 and tokens[0] in {
            "CMAKE_CXX_COMPILER_ID",
            "CMAKE_CXX_SIMULATE_ID",
        }:
            values[tokens[0]] = tokens[1]
    if "CMAKE_CXX_COMPILER_ID" not in values:
        errors.append(f"{prefix}: configured C++ compiler identity is unavailable")
        return None
    return (
        values["CMAKE_CXX_COMPILER_ID"] == "MSVC"
        or values.get("CMAKE_CXX_SIMULATE_ID") == "MSVC"
    )


def inactive_cpp_test_is_approved(
    relative: str,
    root: Path,
    build_dir: Path,
    cache: dict[str, str],
    trace_records: list[dict[str, Any]],
    errors: list[str],
) -> bool:
    """Validate one explicit, predicate-proven inactive C++ test contract."""
    policy = APPROVED_INACTIVE_CPP_TESTS.get(relative)
    if policy is None:
        return False
    cache_predicates = policy.get("cache", {})
    if not isinstance(cache_predicates, dict) or not all(
        cache.get(variable) == value for variable, value in cache_predicates.items()
    ):
        return False
    if policy.get("msvc"):
        uses_msvc = cmake_compiler_uses_msvc_abi(build_dir, cache, errors)
        if uses_msvc is not True:
            return False

    owner_value = policy.get("owner")
    command = policy.get("command")
    scopes = policy.get("scopes")
    if (
        not isinstance(owner_value, str)
        or not isinstance(command, str)
        or not isinstance(scopes, tuple)
    ):
        return False
    owner = (root / owner_value).resolve()
    try:
        owner_records = cmake_scoped_command_origin_details(
            owner.read_text(encoding="utf-8")
        )
    except OSError as error:
        errors.append(
            f"CMake semantic test graph: unable to inspect inactive test "
            f"owner `{owner_value}`: {error}"
        )
        return False
    source_stem = Path(relative).stem
    owners = [
        record
        for record in owner_records
        if record[0] == command
        and source_stem in cmake_argument_tokens(record[1])
        and tuple(f"{kind}:{arguments}" for kind, arguments, _ in record[2]) == scopes
    ]
    if len(owners) != 1:
        errors.append(
            f"CMake semantic test graph: inactive test `{relative}` does not "
            "have its exact branch-owned conditional declaration"
        )
        return False

    for scope_kind, scope_arguments, scope_line in owners[0][2]:
        if scope_kind != "if":
            errors.append(
                f"CMake semantic test graph: inactive test `{relative}` uses "
                f"unsupported `{scope_kind}` ownership"
            )
            return False
        condition = tuple(cmake_argument_tokens(scope_arguments))
        matches = []
        for record in trace_records:
            source = record.get("file")
            arguments = record.get("args")
            if not isinstance(source, str) or not isinstance(arguments, list):
                continue
            source_path = Path(source)
            if not source_path.is_absolute():
                source_path = root / source_path
            if (
                record.get("cmd") == "if"
                and source_path.resolve() == owner
                and tuple(arguments) == condition
                and record.get("line") == scope_line
            ):
                matches.append(record)
        if len(matches) != 1:
            errors.append(
                f"CMake semantic test graph: inactive test `{relative}` "
                f"condition {list(condition)} was not reached exactly once"
            )
            return False
    return True


def executable_paths_match(actual: str, expected: str) -> bool:
    """Compare configured executable paths across platform spelling variants."""
    return os.path.normcase(str(Path(actual).resolve())) == os.path.normcase(
        str(Path(expected).resolve())
    )


def normalized_absolute_path(path: str, base: Path) -> str:
    """Return one comparable absolute path for generated inventory records."""
    candidate = Path(path)
    if not candidate.is_absolute():
        candidate = base / candidate
    return os.path.normcase(str(candidate.resolve()))


def split_cmake_path_list(value: str, separator: str) -> list[str]:
    """Split a path list without treating generator-expression colons as separators."""
    entries: list[str] = []
    entry: list[str] = []
    depth = 0
    offset = 0
    while offset < len(value):
        if value.startswith("$<", offset):
            depth += 1
            entry.extend(("$", "<"))
            offset += 2
            continue
        character = value[offset]
        if (
            character == "\\"
            and depth == 0
            and offset + 1 < len(value)
            and value[offset + 1] == separator
        ):
            entries.append("".join(entry))
            entry = []
            offset += 2
            continue
        if character == ">" and depth:
            depth -= 1
        if character == separator and depth == 0:
            entries.append("".join(entry))
            entry = []
        else:
            entry.append(character)
        offset += 1
    if depth:
        return []
    entries.append("".join(entry))
    return entries


def check_pytest_module_provenance(
    python_executable: str,
    working_directory: Path,
    pythonpath: str,
    errors: list[str],
) -> None:
    """Resolve pytest without importing repository-controlled Python code."""
    prefix = "CMake semantic test graph"
    probe = """
import importlib.machinery
import json
import os
import sys

trusted = importlib.machinery.PathFinder.find_spec("pytest", sys.path)
target_path = [
    sys.argv[1],
    *[entry for entry in sys.argv[2].split(os.pathsep) if entry],
    *sys.path,
]
target = importlib.machinery.PathFinder.find_spec("pytest", target_path)
print(
    json.dumps(
        {
            "prefix": sys.prefix,
            "target": None if target is None else target.origin,
            "target_locations": (
                None
                if target is None or target.submodule_search_locations is None
                else list(target.submodule_search_locations)
            ),
            "trusted": None if trusted is None else trusted.origin,
            "trusted_locations": (
                None
                if trusted is None or trusted.submodule_search_locations is None
                else list(trusted.submodule_search_locations)
            ),
        }
    )
)
""".strip()
    try:
        result = subprocess.run(
            [
                python_executable,
                "-I",
                "-c",
                probe,
                str(working_directory),
                pythonpath,
            ],
            cwd=working_directory.parent,
            capture_output=True,
            text=True,
            timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        errors.append(f"{prefix}: pytest provenance probe failed: {error}")
        return
    if result.returncode != 0:
        output = (result.stdout + result.stderr).strip().splitlines()
        detail = output[-1] if output else f"exit {result.returncode}"
        errors.append(f"{prefix}: pytest provenance probe failed: {detail}")
        return
    output = [line for line in result.stdout.splitlines() if line.strip()]
    if len(output) != 1:
        errors.append(f"{prefix}: pytest provenance probe returned invalid output")
        return
    try:
        data = json.loads(output[0])
    except json.JSONDecodeError as error:
        errors.append(
            f"{prefix}: pytest provenance probe returned invalid JSON: {error}"
        )
        return
    if not isinstance(data, dict):
        errors.append(f"{prefix}: pytest provenance probe returned invalid data")
        return
    target = data.get("target")
    target_locations = data.get("target_locations")
    trusted = data.get("trusted")
    trusted_locations = data.get("trusted_locations")
    environment_prefix = data.get("prefix")
    if not all(
        isinstance(value, str) and value for value in (trusted, environment_prefix)
    ):
        errors.append(f"{prefix}: trusted pytest package is unavailable")
        return
    trusted_path = Path(trusted).resolve()
    try:
        trusted_path.relative_to(Path(environment_prefix).resolve())
    except ValueError:
        errors.append(
            f"{prefix}: trusted pytest resolves outside the configured Python "
            f"environment at `{trusted_path}`"
        )
        return
    if (
        not isinstance(trusted_locations, list)
        or not trusted_locations
        or not all(isinstance(path, str) and path for path in trusted_locations)
    ):
        errors.append(f"{prefix}: trusted pytest package locations are unavailable")
        return
    trusted_location_paths = {
        normalized_absolute_path(path, Path(environment_prefix))
        for path in trusted_locations
    }
    environment_path = Path(environment_prefix).resolve()
    try:
        for path in trusted_location_paths:
            Path(path).resolve().relative_to(environment_path)
    except ValueError:
        errors.append(
            f"{prefix}: trusted pytest package locations escape the configured "
            "Python environment"
        )
        return
    target_location_paths = (
        {normalized_absolute_path(path, working_directory) for path in target_locations}
        if isinstance(target_locations, list)
        and all(isinstance(path, str) for path in target_locations)
        else set()
    )
    if (
        not isinstance(target, str)
        or not executable_paths_match(target, trusted)
        or target_location_paths != trusted_location_paths
    ):
        errors.append(
            f"{prefix}: pytest target resolution does not match the trusted "
            f"active-environment package (resolved `{target}`, expected "
            f"`{trusted_path}`)"
        )


def check_pytest_execution_options(root: Path, errors: list[str]) -> None:
    """Require the pinned pytest configuration to match the safe branch contract."""
    prefix = "CMake semantic test graph"
    config = root / "pyproject.toml"
    try:
        data = read_toml(config)
    except (OSError, tomllib.TOMLDecodeError) as error:
        errors.append(f"{prefix}: invalid pytest configuration: {error}")
        return
    tool = data.get("tool")
    pytest = tool.get("pytest") if isinstance(tool, dict) else None
    options = pytest.get("ini_options") if isinstance(pytest, dict) else None
    if options != EXPECTED_PYTEST_INI_OPTIONS:
        errors.append(
            f"{prefix}: pinned pytest options do not match the safe branch contract"
        )


def check_pytest_source_policies(root: Path, errors: list[str]) -> None:
    """Reject test-module declarations that can load unvalidated local plugins."""
    prefix = "CMake semantic test graph"
    test_sources = {
        path
        for test_root in (root / "tests", root / "python" / "tests")
        if test_root.is_dir()
        for path in test_root.rglob("*.py")
    }
    for path in sorted(test_sources):
        if not path.is_file():
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        except (OSError, SyntaxError) as error:
            errors.append(
                f"{prefix}: unable to inspect pytest source `{path}`: {error}"
            )
            continue
        declares_plugins = any(
            (isinstance(node, ast.Name) and node.id == "pytest_plugins")
            or (
                isinstance(node, (ast.Import, ast.ImportFrom))
                and any(
                    alias.name == "pytest_plugins" or alias.asname == "pytest_plugins"
                    for alias in node.names
                )
            )
            for node in ast.walk(tree)
        )
        if declares_plugins:
            errors.append(
                f"{prefix}: pytest source `{path.relative_to(root)}` declares "
                "`pytest_plugins`; local plugin loading is not allowed"
            )


def check_test_runner_semantics(root: Path, errors: list[str]) -> None:
    """Execute controlled probes for sanitization, test bodies, and failures."""
    prefix = "Test runner semantic probe"
    cmake = shutil.which("cmake")
    ctest = shutil.which("ctest")
    pytest_runner = root / "scripts" / "run_pytest.py"
    ctest_runner = root / "cmake" / "DARTRunCTest.cmake"
    if cmake is None or ctest is None:
        errors.append(f"{prefix}: active CMake and CTest executables are required")
        return
    if not pytest_runner.is_file() or not ctest_runner.is_file():
        errors.append(f"{prefix}: guarded CTest and pytest runners are required")
        return

    probe_parent = root / "build"
    try:
        probe_parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix="dart-test-runner-",
            dir=probe_parent,
        ) as directory:
            probe_root = Path(directory)
            pytest_marker = probe_root / "pytest-body-ran"
            pytest_source = probe_root / "test_runner_probe.py"
            pytest_source.write_text(
                "from pathlib import Path\n\n"
                "def test_runner_body():\n"
                f"    Path({str(pytest_marker)!r}).write_text('ran')\n",
                encoding="utf-8",
            )
            (probe_root / "conftest.py").write_text(
                "def pytest_collection_modifyitems(items):\n"
                "    items.clear()\n\n"
                "def pytest_sessionfinish(session):\n"
                "    session.exitstatus = 0\n",
                encoding="utf-8",
            )
            (probe_root / "runner_probe_plugin.py").write_text(
                "def pytest_collection_modifyitems(items):\n"
                "    items.clear()\n\n"
                "def pytest_sessionfinish(session):\n"
                "    session.exitstatus = 0\n",
                encoding="utf-8",
            )
            (probe_root / "pytest.py").write_text(
                "raise RuntimeError('ambient pytest shadow loaded')\n",
                encoding="utf-8",
            )
            hostile_pytest_environment = os.environ.copy()
            hostile_pytest_environment.update(
                {
                    "PYTEST_ADDOPTS": "--collect-only",
                    "PYTEST_FUTURE_SELECTOR": "skip-everything",
                    "PYTEST_PLUGINS": "runner_probe_plugin",
                    "PYTHONPATH": str(probe_root),
                }
            )
            pytest_result = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    str(pytest_runner),
                    str(pytest_source),
                    "-q",
                ],
                cwd=root,
                env=hostile_pytest_environment,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if pytest_result.returncode != 0 or not pytest_marker.is_file():
                errors.append(
                    f"{prefix}: pytest did not execute a body after clearing "
                    "ambient controls and local plugins"
                )

            collect_result = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    str(pytest_runner),
                    str(pytest_source),
                    "--collect-only",
                    "-q",
                ],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if collect_result.returncode == 0:
                errors.append(f"{prefix}: pytest accepted a successful zero-body run")

            failing_source = probe_root / "test_runner_failure.py"
            failing_source.write_text(
                "def test_runner_failure():\n"
                "    raise AssertionError('semantic failure probe')\n",
                encoding="utf-8",
            )
            failure_result = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    str(pytest_runner),
                    str(failing_source),
                    "-q",
                ],
                cwd=root,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if failure_result.returncode == 0:
                errors.append(f"{prefix}: pytest did not propagate a test failure")

            ctest_source = probe_root / "ctest-project"
            ctest_build = ctest_source / "build"
            ctest_source.mkdir()
            ctest_marker = ctest_build / "ctest-body-ran"
            body_probe = ctest_source / "body_probe.py"
            body_probe.write_text(
                "import os\n"
                "import sys\n"
                "from pathlib import Path\n\n"
                "if any(name.upper().startswith('GTEST_') for name in os.environ):\n"
                "    raise SystemExit(0)\n"
                "Path(sys.argv[1]).write_text('ran')\n",
                encoding="utf-8",
            )

            def cmake_path(path: Path | str) -> str:
                return str(path).replace("\\", "/").replace('"', '\\"')

            (ctest_source / "CMakeLists.txt").write_text(
                "cmake_minimum_required(VERSION 3.22)\n"
                "project(dart_test_runner_probe NONE)\n"
                "enable_testing()\n"
                "add_test(\n"
                "  NAME runner_body\n"
                f'  COMMAND "{cmake_path(sys.executable)}" '
                f'"{cmake_path(body_probe)}" "{cmake_path(ctest_marker)}"\n'
                ")\n",
                encoding="utf-8",
            )
            configure_result = subprocess.run(
                [
                    cmake,
                    "-S",
                    str(ctest_source),
                    "-B",
                    str(ctest_build),
                    "-DCMAKE_BUILD_TYPE=Release",
                ],
                capture_output=True,
                text=True,
                timeout=60,
            )
            if configure_result.returncode != 0:
                errors.append(f"{prefix}: controlled CTest project did not configure")
                return
            hostile_ctest_environment = os.environ.copy()
            hostile_ctest_environment["GTEST_FUTURE_SELECTOR"] = "skip;future=controls"
            ctest_result = subprocess.run(
                [
                    cmake,
                    f"-DDART_CTEST_COMMAND={ctest}",
                    "-DDART_CTEST_CONFIGURATION=Release",
                    "-P",
                    str(ctest_runner),
                ],
                cwd=ctest_build,
                env=hostile_ctest_environment,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if ctest_result.returncode != 0 or not ctest_marker.is_file():
                errors.append(
                    f"{prefix}: CTest did not execute a body after clearing "
                    "ambient GTest controls"
                )

            ctest_failure_result = subprocess.run(
                [
                    cmake,
                    f"-DDART_CTEST_COMMAND={sys.executable}",
                    "-P",
                    str(ctest_runner),
                ],
                cwd=ctest_build,
                capture_output=True,
                text=True,
                timeout=60,
            )
            if ctest_failure_result.returncode == 0:
                errors.append(
                    f"{prefix}: CTest wrapper did not propagate command failure"
                )
    except (OSError, subprocess.TimeoutExpired) as error:
        errors.append(f"{prefix}: controlled execution failed: {error}")


def cmake_unquote_bracket(value: str) -> str:
    """Remove CMake bracket quoting from one generated argument."""
    match = re.fullmatch(r"\[(=*)\[(.*)\]\1\]", value, re.DOTALL)
    return match.group(2) if match is not None else value


def generated_ctest_condition_matches(
    arguments: str, configuration: str
) -> bool | None:
    """Evaluate the configuration predicates emitted by multi-config CMake."""
    tokens = cmake_argument_tokens(arguments)
    if len(tokens) != 3 or tokens[0] != "CTEST_CONFIGURATION_TYPE":
        return None
    if tokens[1] == "STREQUAL":
        return configuration == tokens[2]
    if tokens[1] != "MATCHES":
        return None
    try:
        return re.search(tokens[2], configuration) is not None
    except re.error:
        return None


def generated_ctest_registrations(
    build_dir: Path, configuration: str, errors: list[str]
) -> dict[str, tuple[list[str], Path]]:
    """Read freshly configured CTest commands without requiring built artifacts."""
    prefix = "CMake semantic test graph"
    registrations: dict[str, tuple[list[str], Path]] = {}
    test_root = (build_dir / "tests").resolve()
    root_file = test_root / "CTestTestfile.cmake"
    if not root_file.is_file():
        errors.append(f"{prefix}: configured CTest files are unavailable")
        return registrations
    pending = [root_file]
    visited: set[Path] = set()
    while pending:
        path = pending.pop()
        if path in visited:
            errors.append(f"{prefix}: generated CTest subdirectory cycle at `{path}`")
            continue
        visited.add(path)
        try:
            records = cmake_command_details(path.read_text(encoding="utf-8"))
        except OSError as error:
            errors.append(f"{prefix}: unable to read `{path}`: {error}")
            continue
        branches: list[dict[str, bool]] = []
        for command_name, arguments, _ in records:
            if command_name == "if":
                matched = generated_ctest_condition_matches(arguments, configuration)
                if matched is None:
                    errors.append(
                        f"{prefix}: generated CTest file uses unsupported "
                        f"condition `{arguments}`"
                    )
                    matched = False
                parent_active = all(branch["active"] for branch in branches)
                branches.append(
                    {
                        "active": parent_active and matched,
                        "parent_active": parent_active,
                        "taken": matched,
                    }
                )
                continue
            if command_name == "elseif":
                if not branches:
                    errors.append(
                        f"{prefix}: generated CTest file has unmatched `elseif()`"
                    )
                    continue
                matched = generated_ctest_condition_matches(arguments, configuration)
                if matched is None:
                    errors.append(
                        f"{prefix}: generated CTest file uses unsupported "
                        f"condition `{arguments}`"
                    )
                    matched = False
                branch = branches[-1]
                branch["active"] = (
                    branch["parent_active"] and not branch["taken"] and matched
                )
                branch["taken"] = branch["taken"] or matched
                continue
            if command_name == "else":
                if not branches:
                    errors.append(
                        f"{prefix}: generated CTest file has unmatched `else()`"
                    )
                    continue
                branch = branches[-1]
                branch["active"] = branch["parent_active"] and not branch["taken"]
                branch["taken"] = True
                continue
            if command_name == "endif":
                if not branches:
                    errors.append(
                        f"{prefix}: generated CTest file has unmatched `endif()`"
                    )
                else:
                    branches.pop()
                continue
            if not all(branch["active"] for branch in branches):
                continue
            tokens = [
                cmake_unquote_bracket(token)
                for token in cmake_argument_tokens(arguments)
            ]
            if command_name == "subdirs":
                for directory in tokens:
                    child = (path.parent / directory / "CTestTestfile.cmake").resolve()
                    try:
                        child.relative_to(test_root)
                    except ValueError:
                        errors.append(
                            f"{prefix}: generated CTest subdirectory escapes "
                            f"the build tree: `{directory}`"
                        )
                        continue
                    if not child.is_file():
                        errors.append(
                            f"{prefix}: generated CTest subdirectory is missing "
                            f"`{child}`"
                        )
                        continue
                    pending.append(child)
                continue
            if command_name != "add_test":
                continue
            if tokens[:1] == ["NAME"]:
                try:
                    command_index = tokens.index("COMMAND")
                except ValueError:
                    command_index = -1
                name = tokens[1] if len(tokens) > 1 else ""
                command = tokens[command_index + 1 :] if command_index >= 0 else []
            else:
                name = tokens[0] if tokens else ""
                command = tokens[1:]
            if not name or not command:
                errors.append(f"{prefix}: generated CTest registration is invalid")
                continue
            if name in registrations:
                errors.append(f"{prefix}: generated CTest test `{name}` is duplicated")
                continue
            registrations[name] = (command, path.parent)
        if branches:
            errors.append(f"{prefix}: generated CTest file has unterminated `if()`")
    if not registrations:
        errors.append(f"{prefix}: generated CTest files register no tests")
    return registrations


def ctest_nonexecution_reason(command: list[str]) -> str | None:
    """Return why a configured CTest command cannot execute its test body."""
    forbidden = {
        "--benchmark_list_tests",
        "--co",
        "--collect-only",
        "--fixtures",
        "--gtest_help",
        "--gtest_list_tests",
        "--help",
        "--list",
        "--list-content",
        "--list-labels",
        "--list-test-names-only",
        "--list-tests",
        "--markers",
        "--setup-only",
        "--setup-plan",
        "--version",
        "-h",
        "-V",
    }
    for argument in command[1:]:
        option, _, value = argument.partition("=")
        if option in forbidden:
            return f"uses non-executing option `{argument}`"
        if option == "--gtest_repeat" and value == "0":
            return "uses non-executing option `--gtest_repeat=0`"
        if option == "--gtest_filter" and (not value or value.startswith("-")):
            return f"uses empty-selection option `{argument}`"
    return None


def ctest_property_values(test: dict[str, Any], name: str) -> list[Any]:
    """Return all values for one CTest JSON property."""
    properties = test.get("properties", [])
    if not isinstance(properties, list):
        return []
    return [
        prop.get("value")
        for prop in properties
        if isinstance(prop, dict) and prop.get("name") == name
    ]


def check_cmake_ctest_inventory(
    build_dir: Path,
    configuration: str,
    ctest_executable: str,
    test_targets: dict[str, dict[str, Any]],
    errors: list[str],
) -> None:
    """Require configured C++ targets to have executing CTest registrations."""
    prefix = "CMake semantic test graph"
    try:
        result = subprocess.run(
            [
                ctest_executable,
                "--test-dir",
                str(build_dir / "tests"),
                "--show-only=json-v1",
                "-C",
                configuration,
            ],
            capture_output=True,
            text=True,
            timeout=60,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        errors.append(f"{prefix}: CTest inventory probe failed: {error}")
        return
    if result.returncode != 0:
        output = (result.stdout + result.stderr).strip().splitlines()
        detail = output[-1] if output else f"exit {result.returncode}"
        errors.append(f"{prefix}: CTest inventory probe failed: {detail}")
        return
    try:
        inventory = json.loads(result.stdout)
    except json.JSONDecodeError as error:
        errors.append(f"{prefix}: CTest inventory is invalid JSON: {error}")
        return
    tests = inventory.get("tests") if isinstance(inventory, dict) else None
    if not isinstance(tests, list) or not tests:
        errors.append(f"{prefix}: CTest inventory exposes no registered tests")
        return

    generated = generated_ctest_registrations(build_dir, configuration, errors)
    inventory_names = {
        test.get("name")
        for test in tests
        if isinstance(test, dict) and isinstance(test.get("name"), str)
    }
    generated_names = set(generated)
    if inventory_names != generated_names:
        errors.append(f"{prefix}: CTest JSON and generated registration names disagree")

    registered_tests: dict[str, tuple[list[str], Path]] = {}
    for test in tests:
        if not isinstance(test, dict):
            errors.append(f"{prefix}: CTest inventory contains an invalid test")
            continue
        name = test.get("name")
        command = test.get("command")
        if not isinstance(name, str) or not name:
            errors.append(f"{prefix}: CTest inventory contains an unnamed test")
            continue
        if name in registered_tests:
            errors.append(f"{prefix}: CTest test name `{name}` is duplicated")
            continue
        if any(
            value in (True, 1, "1", "ON", "TRUE")
            for value in ctest_property_values(test, "DISABLED")
        ):
            errors.append(f"{prefix}: CTest test `{name}` is disabled")
        for property_name in sorted(RESULT_NEUTRALIZING_CTEST_PROPERTIES):
            if ctest_property_values(test, property_name):
                errors.append(
                    f"{prefix}: CTest test `{name}` uses result-neutralizing "
                    f"property `{property_name}`"
                )
        generated_registration = generated.get(name)
        if generated_registration is None:
            continue
        generated_command, command_base = generated_registration
        effective_command = generated_command
        if command is not None:
            if (
                not isinstance(command, list)
                or not command
                or not all(isinstance(argument, str) for argument in command)
            ):
                errors.append(f"{prefix}: CTest test `{name}` has an invalid command")
                continue
            if command[1:] != generated_command[1:]:
                errors.append(
                    f"{prefix}: CTest test `{name}` command disagrees with "
                    "the generated registration"
                )
            effective_command = command
        reason = ctest_nonexecution_reason(generated_command)
        if reason is not None:
            errors.append(f"{prefix}: CTest test `{name}` {reason}")
        selection_policy = APPROVED_CTEST_SELECTIONS.get(
            name, {"arguments": (), "environment": ()}
        )
        expected_arguments = tuple(selection_policy["arguments"])
        if tuple(generated_command[1:]) != expected_arguments:
            errors.append(
                f"{prefix}: CTest test `{name}` has unapproved command arguments "
                f"{generated_command[1:]}"
            )
        environment_values = [
            item
            for value in ctest_property_values(test, "ENVIRONMENT")
            for item in (value if isinstance(value, list) else [value])
            if isinstance(item, str)
        ]
        selection_environment = tuple(
            item
            for item in environment_values
            if item.partition("=")[0].upper().startswith("GTEST_")
        )
        expected_environment = tuple(selection_policy["environment"])
        if selection_environment != expected_environment:
            errors.append(
                f"{prefix}: CTest test `{name}` has unapproved GTest "
                f"environment {list(selection_environment)}"
            )
        environment_modifications = [
            item
            for value in ctest_property_values(test, "ENVIRONMENT_MODIFICATION")
            for item in (value if isinstance(value, list) else [value])
            if isinstance(item, str)
            and item.partition("=")[0].upper().startswith("GTEST_")
        ]
        if environment_modifications:
            errors.append(
                f"{prefix}: CTest test `{name}` has unapproved GTest environment "
                f"modifications {environment_modifications}"
            )
        fail_patterns = [
            item
            for value in ctest_property_values(test, "FAIL_REGULAR_EXPRESSION")
            for item in (value if isinstance(value, list) else [value])
            if isinstance(item, str)
        ]
        selected = bool(expected_arguments or expected_environment)
        if selected and fail_patterns != [SAFE_ZERO_TEST_PATTERN]:
            errors.append(
                f"{prefix}: selected CTest test `{name}` does not fail on zero tests "
                f"with exact pattern `{SAFE_ZERO_TEST_PATTERN}`"
            )
        elif not selected and fail_patterns:
            errors.append(
                f"{prefix}: CTest test `{name}` has unapproved failure patterns "
                f"{fail_patterns}"
            )
        effective_registration = (effective_command, command_base)
        registered_tests[name] = effective_registration

    target_artifacts: dict[str, set[str]] = {}
    for name, target in test_targets.items():
        artifacts = target.get("artifacts")
        if not isinstance(artifacts, list):
            artifacts = []
        paths = {
            normalized_absolute_path(artifact["path"], build_dir)
            for artifact in artifacts
            if isinstance(artifact, dict) and isinstance(artifact.get("path"), str)
        }
        if not paths:
            errors.append(f"{prefix}: C++ test target `{name}` has no artifact")
            continue
        target_artifacts[name] = paths

    def registration_matches_target(
        test_name: str,
        registration: tuple[list[str], Path],
        target_name: str,
        artifacts: set[str],
    ) -> bool:
        command, command_base = registration
        if normalized_absolute_path(command[0], command_base) in artifacts:
            return True
        return test_name == target_name and command[0] == target_name

    matched_registrations: set[str] = set()
    unregistered = []
    for name, artifacts in target_artifacts.items():
        matches = {
            test_name
            for test_name, registration in registered_tests.items()
            if registration_matches_target(
                test_name,
                registration,
                name,
                artifacts,
            )
        }
        matched_registrations.update(matches)
        if not matches:
            unregistered.append(name)
    unregistered.sort()
    if unregistered:
        errors.append(
            f"{prefix}: {len(unregistered)} configured C++ test targets are not "
            f"registered with CTest, including {unregistered[:5]}"
        )
    unmapped_commands = sorted(set(registered_tests) - matched_registrations)
    if unmapped_commands:
        errors.append(
            f"{prefix}: {len(unmapped_commands)} CTest commands do not map to "
            f"configured C++ test targets, including {unmapped_commands[:5]}"
        )


def check_cmake_test_target_trace(
    root: Path,
    build_dir: Path,
    dartpy_output_dir: str,
    cache: dict[str, str],
    records: list[dict[str, Any]],
    origins: dict[str, tuple[str, int]],
    cmake_executable: str,
    errors: list[str],
) -> None:
    """Validate expanded commands for the configured CTest and pytest targets."""
    prefix = "CMake semantic test graph"

    def target_arguments(name: str) -> list[str] | None:
        relative, line = origins[name]
        expected_file = (root / relative).resolve()
        matches = []
        for record in records:
            arguments = record.get("args")
            source = record.get("file")
            if (
                record.get("cmd") != "add_custom_target"
                or not isinstance(arguments, list)
                or not arguments
                or arguments[0] != name
                or not all(isinstance(argument, str) for argument in arguments)
                or not isinstance(source, str)
                or Path(source).resolve() != expected_file
                or record.get("line") != line
            ):
                continue
            matches.append(arguments)
        if len(matches) != 1:
            errors.append(
                f"{prefix}: expected one expanded `{name}` target command at "
                f"`{relative}:{line}`, found {len(matches)}"
            )
            return None
        return matches[0]

    tests_arguments = target_arguments("tests_and_run")
    ctest_executable = cache.get("CMAKE_CTEST_COMMAND")
    trusted_ctest = shutil.which("ctest")
    if not isinstance(ctest_executable, str):
        errors.append(f"{prefix}: configured CTest executable is unavailable")
    elif trusted_ctest is None:
        errors.append(f"{prefix}: trusted CTest executable is unavailable")
    elif not executable_paths_match(ctest_executable, trusted_ctest):
        errors.append(
            f"{prefix}: configured CTest executable does not match the "
            "active Pixi environment"
        )
    elif tests_arguments is not None:
        ctest_definition = (
            tests_arguments[3].partition("=")[2] if len(tests_arguments) > 3 else ""
        )
        configuration_definition = (
            tests_arguments[4].partition("=")[2] if len(tests_arguments) > 4 else ""
        )
        suffix = tests_arguments[7:]
        if (
            tests_arguments[:2] != ["tests_and_run", "COMMAND"]
            or len(tests_arguments) < 8
            or not executable_paths_match(tests_arguments[2], cmake_executable)
            or not tests_arguments[3].startswith("-DDART_CTEST_COMMAND=")
            or not executable_paths_match(ctest_definition, trusted_ctest)
            or not tests_arguments[4].startswith("-DDART_CTEST_CONFIGURATION=")
            or configuration_definition not in {"$<CONFIG>", "Release"}
            or tests_arguments[5] != "-P"
            or not cmake_paths_match(
                tests_arguments[6], root / "cmake" / "DARTRunCTest.cmake"
            )
            or tests_arguments.count("COMMAND") != 1
            or not suffix
            or suffix[0] != "DEPENDS"
        ):
            errors.append(
                f"{prefix}: expanded `tests_and_run` command does not invoke "
                "the validated clean-environment CTest runner"
            )

    pytest_arguments = target_arguments("pytest")
    python_executable = cache.get("_Python3_EXECUTABLE") or cache.get(
        "Python3_EXECUTABLE"
    )
    if not isinstance(python_executable, str):
        errors.append(f"{prefix}: configured Python executable is unavailable")
    elif not executable_paths_match(python_executable, sys.executable):
        errors.append(
            f"{prefix}: configured Python executable does not match the "
            "active Pixi environment"
        )
    elif pytest_arguments is not None:
        command_indexes = [
            index
            for index, argument in enumerate(pytest_arguments)
            if argument == "COMMAND"
        ]
        valid = len(command_indexes) == 2
        if valid:
            echo_index, pytest_index = command_indexes
            valid = (
                len(pytest_arguments) > echo_index + 3
                and executable_paths_match(
                    pytest_arguments[echo_index + 1], cmake_executable
                )
                and pytest_arguments[echo_index + 2 : echo_index + 4] == ["-E", "echo"]
                and len(pytest_arguments) > pytest_index + 6
                and executable_paths_match(
                    pytest_arguments[pytest_index + 1], sys.executable
                )
                and pytest_arguments[pytest_index + 2] == "-I"
                and cmake_paths_match(
                    pytest_arguments[pytest_index + 3],
                    root / "scripts" / "run_pytest.py",
                )
                and pytest_arguments[pytest_index + 4] == "--pythonpath"
            )
        if valid:
            pythonpath_value = pytest_arguments[pytest_index + 5]
            path_separator = ";" if r"\;" in pythonpath_value else os.pathsep
            pythonpath_entries = split_cmake_path_list(pythonpath_value, path_separator)
            resolved_pythonpath_entries = [
                (dartpy_output_dir if entry == "$<TARGET_FILE_DIR:dartpy>" else entry)
                for entry in pythonpath_entries
            ]
            normalized_pythonpath = {
                normalized_absolute_path(entry, root / "python" / "tests")
                for entry in resolved_pythonpath_entries
                if entry
            }
            allowed_pythonpath = {
                normalized_absolute_path(dartpy_output_dir, build_dir),
                normalized_absolute_path(str(build_dir / "python"), root),
            }
            valid = (
                bool(pythonpath_entries)
                and all(pythonpath_entries)
                and pythonpath_entries[0] == "$<TARGET_FILE_DIR:dartpy>"
                and len(pythonpath_entries) == len(normalized_pythonpath)
                and normalized_pythonpath == allowed_pythonpath
            )
            pythonpath_value = os.pathsep.join(resolved_pythonpath_entries)
        if valid:
            try:
                working_index = pytest_arguments.index("WORKING_DIRECTORY")
                sources_index = pytest_arguments.index("SOURCES")
                depends_index = pytest_arguments.index("DEPENDS")
            except ValueError:
                valid = False
            else:
                valid = (
                    pytest_index < working_index < sources_index < depends_index
                    and len(pytest_arguments) > working_index + 1
                    and Path(pytest_arguments[working_index + 1]).resolve()
                    == (root / "python" / "tests").resolve()
                )
        if valid:
            command_tail = pytest_arguments[pytest_index + 6 : working_index]
            valid = bool(command_tail) and command_tail[-1] == "-v"
        if valid:
            traced_sources = [
                normalized_absolute_path(source, root / "python" / "tests")
                for argument in command_tail[:-1]
                for source in argument.split(";")
                if source
            ]
            expected_sources = [
                normalized_absolute_path(str(path), root)
                for path in sorted((root / "python" / "tests").rglob("test_*.py"))
                if path.is_file()
            ]
            valid = len(traced_sources) == len(expected_sources) and set(
                traced_sources
            ) == set(expected_sources)
        if valid:
            check_pytest_execution_options(root, errors)
            check_pytest_source_policies(root, errors)
            check_pytest_module_provenance(
                sys.executable,
                root / "python" / "tests",
                pythonpath_value,
                errors,
            )
        if not valid:
            errors.append(
                f"{prefix}: expanded `pytest` command does not invoke the "
                "configured Python interpreter over the configured test sources"
            )


def check_cmake_test_graph(
    root: Path,
    build_dir: Path,
    errors: list[str],
) -> None:
    """Validate the configured test graph through CMake's File API."""
    prefix = "CMake semantic test graph"
    root = root.resolve()
    build_dir = build_dir.resolve()
    build_root = (root / "build").resolve()
    try:
        build_dir.relative_to(build_root)
    except ValueError:
        errors.append(f"{prefix}: build directory must be inside `{build_root}`")
        return

    cache_path = build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        errors.append(f"{prefix}: missing configured cache `{cache_path}`")
        return

    reply = build_dir / ".cmake" / "api" / "v1" / "reply"
    try:
        previous_indexes = {}
        for path in reply.glob("index-*.json"):
            stat = path.stat()
            previous_indexes[path.name] = (stat.st_mtime_ns, stat.st_size)
    except OSError as error:
        errors.append(f"{prefix}: unable to inspect prior File API replies: {error}")
        return

    query = build_dir / ".cmake" / "api" / "v1" / "query" / "codemodel-v2"
    try:
        query.parent.mkdir(parents=True, exist_ok=True)
        query.write_text("", encoding="utf-8")
    except OSError as error:
        errors.append(f"{prefix}: unable to create File API query: {error}")
        return

    executable = shutil.which("cmake")
    if executable is None:
        errors.append(f"{prefix}: `cmake` is unavailable")
        return
    trace_candidates = {
        root / "cmake" / "DARTMacros.cmake",
        root / "python" / "tests" / "CMakeLists.txt",
        *(root / "tests").rglob("CMakeLists.txt"),
    }
    traced_sources = sorted(path for path in trace_candidates if path.is_file())
    trace_arguments = [
        "--trace-expand",
        "--trace-format=json-v1",
        *(f"--trace-source={source}" for source in traced_sources),
    ]
    try:
        result = subprocess.run(
            [
                executable,
                "-S",
                str(root),
                "-B",
                str(build_dir),
                *trace_arguments,
            ],
            cwd=root,
            capture_output=True,
            text=True,
            timeout=120,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        errors.append(f"{prefix}: configure probe failed: {error}")
        return
    if result.returncode != 0:
        output = (result.stdout + result.stderr).strip().splitlines()
        detail = output[-1] if output else f"exit {result.returncode}"
        errors.append(f"{prefix}: configure probe failed: {detail}")
        return
    trace_records = cmake_trace_records(result.stderr)

    try:
        cache = cmake_cache_values(cache_path)
    except OSError as error:
        errors.append(f"{prefix}: unable to read `{cache_path}`: {error}")
        return
    for variable in (
        "BUILD_TESTING",
        "DART_BUILD_DARTPY",
        "DART_USE_SYSTEM_PYBIND11",
    ):
        if cache.get(variable) != "ON":
            errors.append(f"{prefix}: configured `{variable}` must be `ON`")
    cache_home = cache.get("CMAKE_HOME_DIRECTORY")
    if not isinstance(cache_home, str) or not cmake_paths_match(cache_home, root):
        errors.append(f"{prefix}: configured cache belongs to another source tree")
    desired = "Release"
    configuration_types = cache.get("CMAKE_CONFIGURATION_TYPES")
    if isinstance(configuration_types, str) and configuration_types:
        if desired not in configuration_types.split(";"):
            errors.append(f"{prefix}: multi-config cache does not provide `{desired}`")
    elif cache.get("CMAKE_BUILD_TYPE") != desired:
        errors.append(
            f"{prefix}: single-config cache must use `CMAKE_BUILD_TYPE={desired}`"
        )

    try:
        fresh_indexes = []
        for path in reply.glob("index-*.json"):
            stat = path.stat()
            current = (stat.st_mtime_ns, stat.st_size)
            if previous_indexes.get(path.name) != current:
                fresh_indexes.append((stat.st_mtime_ns, path.name, path))
    except OSError as error:
        errors.append(f"{prefix}: unable to inspect File API replies: {error}")
        return
    if not fresh_indexes:
        errors.append(f"{prefix}: CMake File API did not emit a fresh index")
        return
    fresh_indexes.sort()
    index_path = fresh_indexes[-1][2]
    try:
        index = read_json(index_path)
        codemodel_reply = index["reply"]["codemodel-v2"]
        codemodel = read_json(reply / codemodel_reply["jsonFile"])
    except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
        errors.append(f"{prefix}: invalid CMake File API reply: {error}")
        return

    paths = codemodel.get("paths")
    source_path = paths.get("source") if isinstance(paths, dict) else None
    if not isinstance(source_path, str) or not cmake_paths_match(source_path, root):
        errors.append(f"{prefix}: File API reply belongs to another source tree")
        return
    configurations = codemodel.get("configurations")
    if not isinstance(configurations, list) or not configurations:
        errors.append(f"{prefix}: File API reply has no configurations")
        return
    configuration = next(
        (
            item
            for item in configurations
            if isinstance(item, dict) and item.get("name") == desired
        ),
        configurations[0] if len(configurations) == 1 else None,
    )
    if not isinstance(configuration, dict):
        errors.append(f"{prefix}: File API has no `{desired}` configuration")
        return

    targets = configuration.get("targets")
    directories = configuration.get("directories")
    if not isinstance(targets, list) or not isinstance(directories, list):
        errors.append(f"{prefix}: File API codemodel is incomplete")
        return
    named_targets: dict[str, list[dict[str, Any]]] = {}
    id_to_name: dict[str, str] = {}
    for entry in targets:
        if not isinstance(entry, dict):
            continue
        name = entry.get("name")
        target_id = entry.get("id")
        if isinstance(name, str):
            named_targets.setdefault(name, []).append(entry)
        if isinstance(name, str) and isinstance(target_id, str):
            id_to_name[target_id] = name

    required_entries: dict[str, dict[str, Any]] = {}
    for name in ("ALL", "dartpy", "tests_and_run", "pytest"):
        entries = named_targets.get(name, [])
        if len(entries) != 1:
            errors.append(
                f"{prefix}: expected exactly one configured `{name}` target, "
                f"found {len(entries)}"
            )
            continue
        required_entries[name] = entries[0]
    if len(required_entries) != 4:
        return

    target_data: dict[str, dict[str, Any]] = {}
    for name, entry in required_entries.items():
        json_file = entry.get("jsonFile")
        if not isinstance(json_file, str):
            errors.append(f"{prefix}: target `{name}` has no File API object")
            continue
        try:
            data = read_json(reply / json_file)
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f"{prefix}: unable to read target `{name}`: {error}")
            continue
        if not isinstance(data, dict):
            errors.append(f"{prefix}: target `{name}` object is invalid")
            continue
        target_data[name] = data
    if len(target_data) != 4:
        return

    dartpy_artifacts = target_data["dartpy"].get("artifacts", [])
    if not isinstance(dartpy_artifacts, list):
        dartpy_artifacts = []
    dartpy_output_dirs = {
        str(Path(normalized_absolute_path(artifact["path"], build_dir)).parent)
        for artifact in dartpy_artifacts
        if isinstance(artifact, dict) and isinstance(artifact.get("path"), str)
    }
    if len(dartpy_output_dirs) != 1:
        errors.append(
            f"{prefix}: configured `dartpy` target must expose one artifact "
            "directory"
        )
        return
    dartpy_output_dir = next(iter(dartpy_output_dirs))

    def dependency_names(name: str) -> set[str]:
        dependencies = target_data[name].get("dependencies", [])
        if not isinstance(dependencies, list):
            return set()
        names: set[str] = set()
        for dependency in dependencies:
            if not isinstance(dependency, dict):
                continue
            target_id = dependency.get("id")
            if not isinstance(target_id, str):
                continue
            names.add(id_to_name.get(target_id, target_id.split("::@", 1)[0]))
        return names

    all_dependencies = dependency_names("ALL")
    missing_all = {"tests_and_run", "pytest"} - all_dependencies
    if missing_all:
        errors.append(
            f"{prefix}: `ALL` is missing direct dependencies " f"{sorted(missing_all)}"
        )
    if "dartpy" not in dependency_names("pytest"):
        errors.append(f"{prefix}: `pytest` does not depend on `dartpy`")

    test_directories = ("tests/integration", "tests/regression", "tests/unit")

    def repository_directory(source: str) -> str | None:
        candidate = Path(source)
        if candidate.is_absolute():
            try:
                candidate = candidate.resolve().relative_to(root)
            except ValueError:
                return None
        return candidate.as_posix().rstrip("/")

    configured_directories = {
        relative
        for directory in directories
        if isinstance(directory, dict)
        and isinstance(directory.get("source"), str)
        and (relative := repository_directory(directory["source"])) is not None
    }
    expected_directories = {
        path.parent.relative_to(root).as_posix()
        for directory in test_directories
        for path in (root / directory).rglob("CMakeLists.txt")
    }
    missing_directories = expected_directories - configured_directories
    if missing_directories:
        errors.append(
            f"{prefix}: configured test graph omits {len(missing_directories)} "
            f"CMake directories, including {sorted(missing_directories)[:5]}"
        )

    test_target_names: set[str] = set()
    test_target_data: dict[str, dict[str, Any]] = {}
    configured_test_sources: set[str] = set()
    configured_source_targets: dict[str, list[tuple[str, dict[str, Any]]]] = {}
    executable_test_sources: set[str] = set()
    for entry in targets:
        if not isinstance(entry, dict):
            continue
        directory_index = entry.get("directoryIndex")
        name = entry.get("name")
        if (
            type(directory_index) is not int
            or not 0 <= directory_index < len(directories)
            or not isinstance(name, str)
        ):
            continue
        directory = directories[directory_index]
        source = directory.get("source") if isinstance(directory, dict) else None
        relative_source = (
            repository_directory(source) if isinstance(source, str) else None
        )
        if isinstance(relative_source, str) and any(
            relative_source == directory or relative_source.startswith(f"{directory}/")
            for directory in test_directories
        ):
            json_file = entry.get("jsonFile")
            if not isinstance(json_file, str):
                errors.append(
                    f"{prefix}: C++ test candidate `{name}` has no File API object"
                )
                continue
            try:
                candidate = read_json(reply / json_file)
            except (OSError, json.JSONDecodeError) as error:
                errors.append(
                    f"{prefix}: unable to inspect C++ test candidate "
                    f"`{name}`: {error}"
                )
                continue
            if not isinstance(candidate, dict):
                continue
            candidate_sources: set[str] = set()
            for candidate_source in candidate.get("sources", []):
                path = (
                    candidate_source.get("path")
                    if isinstance(candidate_source, dict)
                    else None
                )
                if not isinstance(path, str):
                    continue
                relative = repository_directory(path)
                if relative is not None:
                    configured_test_sources.add(relative)
                    candidate_sources.add(relative)
                    configured_source_targets.setdefault(relative, []).append(
                        (name, candidate)
                    )
            if candidate.get("type") == "EXECUTABLE":
                test_target_names.add(name)
                test_target_data[name] = candidate
                executable_test_sources.update(candidate_sources)
    if not test_target_names:
        errors.append(f"{prefix}: File API exposes no C++ test targets")
    else:
        missing_tests = test_target_names - dependency_names("tests_and_run")
        if missing_tests:
            sample = sorted(missing_tests)[:5]
            errors.append(
                f"{prefix}: `tests_and_run` omits {len(missing_tests)} configured "
                f"C++ test targets, including {sample}"
            )

    source_extensions = {".c", ".cc", ".cpp", ".cxx"}
    expected_test_sources = {
        path.relative_to(root).as_posix(): path
        for directory in test_directories
        for path in (root / directory).rglob("test_*")
        if path.is_file() and path.suffix.lower() in source_extensions
    }
    unowned_sources = []
    library_only_sources = []
    invalid_compile_only_sources = []
    for relative in expected_test_sources:
        if relative in executable_test_sources:
            continue
        if relative in configured_test_sources:
            owners = configured_source_targets.get(relative, [])
            compile_only_valid = False
            if (
                relative == "tests/unit/math/test_LegacyConvexHullC.c"
                and len(owners) == 1
            ):
                owner_name, owner_target = owners[0]
                backtrace = cmake_target_backtrace(owner_target)
                compile_only_valid = (
                    owner_name == "UNIT_math_LegacyConvexHullC"
                    and owner_target.get("type") == "OBJECT_LIBRARY"
                    and owner_name in dependency_names("tests_and_run")
                    and backtrace is not None
                    and backtrace[0] == "add_library"
                    and backtrace[1].replace("\\", "/")
                    == "tests/unit/math/CMakeLists.txt"
                    and backtrace[2] == 11
                )
            if compile_only_valid:
                continue
            if relative == "tests/unit/math/test_LegacyConvexHullC.c":
                invalid_compile_only_sources.append(relative)
            else:
                library_only_sources.append(relative)
            continue
        if inactive_cpp_test_is_approved(
            relative,
            root,
            build_dir,
            cache,
            trace_records,
            errors,
        ):
            continue
        unowned_sources.append(relative)
    if library_only_sources:
        errors.append(
            f"{prefix}: {len(library_only_sources)} C++ runtime test sources "
            f"belong only to non-executable targets, including "
            f"{library_only_sources[:5]}"
        )
    if invalid_compile_only_sources:
        errors.append(
            f"{prefix}: compile-only C++ test sources do not match their exact "
            f"object-target contract, including {invalid_compile_only_sources[:5]}"
        )
    if unowned_sources:
        errors.append(
            f"{prefix}: {len(unowned_sources)} C++ test sources are neither "
            f"configured nor explicitly approved as inactive, including "
            f"{unowned_sources[:5]}"
        )

    ctest_executable = shutil.which("ctest")
    if ctest_executable is None:
        errors.append(f"{prefix}: trusted CTest executable is unavailable")
    elif test_target_data:
        check_cmake_ctest_inventory(
            build_dir,
            desired,
            ctest_executable,
            test_target_data,
            errors,
        )

    expected_python_sources = {
        path.relative_to(root).as_posix()
        for path in (root / "python" / "tests").rglob("test_*.py")
        if path.is_file()
    }
    actual_python_sources = {
        source["path"]
        for source in target_data["pytest"].get("sources", [])
        if isinstance(source, dict) and isinstance(source.get("path"), str)
    }
    missing_python_sources = expected_python_sources - actual_python_sources
    if missing_python_sources:
        errors.append(
            f"{prefix}: `pytest` omits {len(missing_python_sources)} Python "
            f"test sources, including {sorted(missing_python_sources)[:5]}"
        )

    requirements = test_graph_scope_requirements()
    target_requirements = {
        "ALL": ("CMakeLists.txt", (requirements["CMakeLists.txt"][0],)),
        "tests_and_run": (
            "tests/CMakeLists.txt",
            requirements["tests/CMakeLists.txt"],
        ),
        "pytest": (
            "python/tests/CMakeLists.txt",
            (requirements["python/tests/CMakeLists.txt"][0],),
        ),
    }
    configured_origins: dict[str, tuple[str, int]] = {}
    for name, (relative, allowed_requirements) in target_requirements.items():
        data = target_data[name]
        if data.get("type") != "UTILITY":
            errors.append(f"{prefix}: `{name}` is not a CMake utility target")
            continue
        backtrace = cmake_target_backtrace(data)
        if backtrace is None:
            errors.append(f"{prefix}: `{name}` has no creation backtrace")
            continue
        command, source, line = backtrace
        if command != "add_custom_target" or Path(source).as_posix() != relative:
            errors.append(
                f"{prefix}: `{name}` was created by `{command}` at "
                f"`{source}:{line}`, not its owning graph command"
            )
            continue
        try:
            records = cmake_scoped_command_details(
                (root / relative).read_text(encoding="utf-8")
            )
        except OSError as error:
            errors.append(f"{prefix}: unable to inspect `{relative}`: {error}")
            continue
        record = next(
            (
                (record_name, arguments, context)
                for record_name, arguments, context, record_line in records
                if record_line == line
            ),
            None,
        )
        if record is None or not any(
            cmake_graph_command_matches(record, requirement)
            for requirement in allowed_requirements
        ):
            errors.append(
                f"{prefix}: configured `{name}` does not originate from its "
                f"validated source contract at `{relative}:{line}`"
            )
            continue
        configured_origins[name] = (relative, line)

    if {"tests_and_run", "pytest"} <= configured_origins.keys():
        check_cmake_test_target_trace(
            root,
            build_dir,
            dartpy_output_dir,
            cache,
            trace_records,
            configured_origins,
            executable,
            errors,
        )


def check_path_references(root: Path, errors: list[str]) -> None:
    required_pattern = re.compile(r"@([A-Za-z0-9_.-]+(?:/[A-Za-z0-9_.-]+)+)")
    backtick_pattern = re.compile(
        r"`((?:\.agents/|\.claude/|\.codex/|\.opencode/|dart/|docs/|"
        r"python/|scripts/|tests/)[^`\s]+)`"
    )
    for path in source_paths(root):
        if not path.exists():
            continue
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
            candidates = required_pattern.findall(line)
            if not re.search(
                r"\b(?:intentionally not|does not exist|no longer exists|"
                r"retired|removed|has no)\b",
                line,
                re.IGNORECASE,
            ):
                candidates += backtick_pattern.findall(line)
            for candidate in candidates:
                candidate = candidate.rstrip(".,;:)")
                if any(marker in candidate for marker in ("<", ">", "*", "$", "...")):
                    continue
                if not candidate.startswith(REFERENCE_ROOTS):
                    continue
                if not candidate.endswith("/") and not Path(candidate).suffix:
                    # Conceptual or historical directory names can be valid
                    # prose even after a source tree is retired. Directory
                    # links that promise current existence must include `/`.
                    continue
                relative = repository_relative_path(root, candidate.rstrip("/"))
                if relative is None:
                    errors.append(
                        f"{path.relative_to(root)}:{line_number}: path escapes "
                        f"repository `{candidate}`"
                    )
                elif not (root / relative).exists():
                    errors.append(
                        f"{path.relative_to(root)}:{line_number}: missing path `{candidate}`"
                    )


def markdown_link_errors(root: Path, path: Path) -> list[str]:
    """Return broken repository-relative Markdown links for one source file."""
    errors: list[str] = []
    link_pattern = re.compile(r"\[[^\]]+\]\(([^)]+)\)")
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        for raw_target in link_pattern.findall(line):
            target = raw_target.strip().strip("<>").split(maxsplit=1)[0]
            if not target or target.startswith(
                ("#", "/", "http://", "https://", "mailto:")
            ):
                continue
            relative = unquote(target.split("#", 1)[0])
            resolved = (path.parent / relative).resolve()
            try:
                resolved.relative_to(root)
            except ValueError:
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: link escapes repository "
                    f"`{target}`"
                )
                continue
            if not resolved.exists():
                errors.append(
                    f"{path.relative_to(root)}:{line_number}: broken link `{target}`"
                )
    return errors


def check_markdown_links(root: Path, errors: list[str]) -> None:
    for path in source_paths(root):
        if path.is_file() and path.suffix == ".md":
            errors.extend(markdown_link_errors(root, path))


def tracked_agent_files(root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "*AGENTS.md"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        paths = [root / line for line in result.stdout.splitlines() if line]
    else:
        paths = [
            path
            for path in root.rglob("AGENTS.md")
            if not any(
                part in {".git", ".pixi", "build", "external", "node_modules"}
                for part in path.relative_to(root).parts
            )
        ]
    for runtime_path in (
        root / ".agents" / "AGENTS.md",
        root / ".codex" / "AGENTS.md",
    ):
        if runtime_path.is_file() and runtime_path not in paths:
            paths.append(runtime_path)
    return paths


def check_instruction_budget(root: Path, errors: list[str]) -> None:
    for target in tracked_agent_files(root):
        chain: list[Path] = []
        directory = target.parent
        while directory == root or root in directory.parents:
            candidate = directory / "AGENTS.md"
            if candidate.exists():
                chain.append(candidate)
            if directory == root:
                break
            directory = directory.parent
        total = sum(path.stat().st_size for path in chain)
        if total > MAX_AGENT_INSTRUCTION_BYTES:
            errors.append(
                f"{target.relative_to(root)}: instruction chain is {total} bytes "
                f"(limit {MAX_AGENT_INSTRUCTION_BYTES})"
            )


def check_release_guidance(root: Path, errors: list[str]) -> None:
    python_skill = (root / ".claude" / "skills" / "dart-python" / "SKILL.md").read_text(
        encoding="utf-8"
    )
    python_frontmatter = python_skill.split("---", 2)[1]
    if "nanobind" in python_frontmatter or "pybind11" not in python_skill:
        errors.append("dart-python: DART 6.20 metadata must name pybind11")

    ci_skill_path = root / ".claude" / "skills" / "dart-ci" / "SKILL.md"
    ci_skill = ci_skill_path.read_text(encoding="utf-8")
    if "Expected CI Times" in ci_skill or "reduces build time" in ci_skill:
        errors.append(
            "dart-ci: volatile timing/cache claims belong in live run evidence"
        )
    for workflow in re.findall(r"`(ci_[a-z0-9_]+\.yml)`", ci_skill):
        if not (root / ".github" / "workflows" / workflow).is_file():
            errors.append(f"dart-ci: nonexistent workflow `{workflow}`")

    release_fix = (root / ".claude" / "commands" / "dart-release-ci-fix.md").read_text(
        encoding="utf-8"
    )
    if "release-6.20" not in release_fix or "release-6.19" in release_fix:
        errors.append("dart-release-ci-fix: release default must be release-6.20")

    for path in source_paths(root):
        if not path.exists():
            continue
        content = path.read_text(encoding="utf-8")
        if ".codex/skills" in content:
            errors.append(f"{path.relative_to(root)}: use current `.agents/skills/`")
        if re.search(r"generated[^\n]*`?\.codex/?`?", content, re.IGNORECASE):
            errors.append(
                f"{path.relative_to(root)}: `.codex/` is maintained, not generated"
            )
    if (root / ".codex" / "skills").exists():
        errors.append("legacy generated directory `.codex/skills/` must be absent")

    agents = (root / "AGENTS.md").read_text(encoding="utf-8")
    for marker in (
        "## Quick Commands",
        "## Task-specific Context",
        "docs/onboarding/architecture.md",
        "docs/onboarding/building.md",
        "docs/onboarding/testing.md",
    ):
        if marker not in agents:
            errors.append(f"AGENTS.md: missing pointer-board marker `{marker}`")
    for stale in ("Building for Codex", "manual CMake"):
        if stale in agents:
            errors.append(f"AGENTS.md: stale duplicated guidance `{stale}`")


def check_ci_wiring(root: Path, errors: list[str]) -> None:
    workflow = root / ".github" / "workflows" / "ci_ubuntu.yml"
    if not workflow.exists():
        errors.append(".github/workflows/ci_ubuntu.yml: missing workflow")
    else:
        content = workflow.read_text(encoding="utf-8")
        expected_commands = (
            "pixi run check-ai-commands",
            "pixi run check-ai-infra",
            "pixi run test-ai-infra",
            "scripts/check_ai_infrastructure.py --scenarios",
        )
        for command in expected_commands:
            if command not in content:
                errors.append(
                    f".github/workflows/ci_ubuntu.yml: missing AI check `{command}`"
                )
        visual_step_name = "- name: Agent visual verification smoke"
        if visual_step_name not in content:
            errors.append(
                ".github/workflows/ci_ubuntu.yml: missing visual smoke marker "
                "`Agent visual verification smoke`"
            )
        visual_section = content.partition(visual_step_name)[2].partition(
            "\n      - name:"
        )[0]
        conditions = re.findall(r"(?m)^\s*if:\s*(.*?)\s*$", visual_section)
        if conditions != ["matrix.build_type == 'Release'"]:
            errors.append(
                ".github/workflows/ci_ubuntu.yml: visual smoke must run for "
                "exactly the Release matrix entry"
            )
        if "continue-on-error" in visual_section:
            errors.append(
                ".github/workflows/ci_ubuntu.yml: visual smoke must not use "
                "continue-on-error"
            )
        expected_visual_markers = (
            ("xvfb-run", 2),
            ("bash -eu -o pipefail <<'VISUAL_SMOKE'", 1),
            ("pixi run agent-capture", 2),
            (
                "--scene box_on_ground --steps 250 --focus box --auto-views 1",
                1,
            ),
            ("--scene dart_shape_contacts --steps 500", 1),
            ("--camera-azimuth 1.5708 --camera-elevation 0.35", 1),
            ("--camera-distance 4.5 --camera-target 0 0 0.2", 1),
            ("--layers contacts collision_bounds labels", 2),
            ("--width 320 --height 240", 1),
            ("--width 640 --height 480", 1),
            ("--out /tmp/dart-agent-visual-smoke --prefix smoke", 1),
            ("--out /tmp/dart-agent-visual-smoke --prefix dart", 1),
            ("pixi run image-verdict", 2),
            ("/tmp/dart-agent-visual-smoke/smoke_auto0.png", 1),
            ("/tmp/dart-agent-visual-smoke/dart_main.png", 1),
            ("pixi run test-agent-debug-overlay", 1),
        )
        for marker, expected_count in expected_visual_markers:
            actual_count = visual_section.count(marker)
            if actual_count != expected_count:
                errors.append(
                    ".github/workflows/ci_ubuntu.yml: missing visual smoke "
                    f"marker `{marker}` exactly {expected_count} time(s); "
                    f"found {actual_count}"
                )
        overlay_test = root / "python/tests/unit/gui/test_agent_debug_overlay.py"
        try:
            overlay_text = overlay_test.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{overlay_test.relative_to(root)}: unable to read: {error}")
        else:
            for marker in (
                'pytest.fail("no off-screen GL context despite a configured DISPLAY")',
                "assert with_overlay != base",
                "assert after_clear == base",
            ):
                if marker not in overlay_text:
                    errors.append(
                        f"{overlay_test.relative_to(root)}: missing non-skippable "
                        f"overlay marker `{marker}`"
                    )
        capture_test = root / "python/tests/unit/gui/test_agent_capture.py"
        try:
            capture_text = capture_test.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"{capture_test.relative_to(root)}: unable to read: {error}")
        else:
            for marker in (
                "test_run_capture_smoke_writes_stills_and_sidecar",
                "test_run_capture_debug_layers_change_pixels_end_to_end",
                'factory="claim_capture_scene:make_world"',
                "pytest.fail(str(error))",
                'assert "--factory claim_capture_scene:make_world"',
                'debug_layers = ["contacts", "collision_bounds", "labels"]',
                'plain_artifact["camera"] == combined_artifact["camera"]',
                "assert _changed_pixel_count(plain_image, combined_image) >= 128",
                "agent_debug_overlay.CONTACT_POINT_RGB",
                "assert contact_pixels >= 4",
                "for layer in debug_layers",
                'assert debug["layers"] == [layer]',
                "assert _changed_pixel_count(plain_image, debug_image) >= 32",
            ):
                if marker not in capture_text:
                    errors.append(
                        f"{capture_test.relative_to(root)}: missing non-skippable "
                        f"factory-capture marker `{marker}`"
                    )
        pixi_path = root / "pixi.toml"
        try:
            pixi_text = pixi_path.read_text(encoding="utf-8")
        except OSError as error:
            errors.append(f"pixi.toml: unable to read: {error}")
        else:
            for marker in (
                "test-agent-debug-overlay =",
                "test_contacts_layer_marks_points_and_normals",
                "test_engine_rendered_overlay_changes_pixels",
                "test_run_capture_smoke_writes_stills_and_sidecar",
                "test_run_capture_debug_layers_change_pixels_end_to_end",
            ):
                if marker not in pixi_text:
                    errors.append(
                        f"pixi.toml: missing visual verification task marker `{marker}`"
                    )
    windows = root / ".github" / "workflows" / "ci_windows.yml"
    if not windows.exists():
        errors.append(".github/workflows/ci_windows.yml: missing workflow")
        return
    windows_content = windows.read_text(encoding="utf-8")
    for marker in (
        "Native Windows hook smoke",
        'pixi run python -c "import sys; print(sys.executable)"',
        "$launcher",
        "$hookCommand",
        "& (Join-Path (git rev-parse --show-toplevel)",
        '$ErrorActionPreference = "Continue"',
        '"git status"',
        "DART_HOOK_DRY_RUN",
        '"git commit --no-verify -m x"',
        "$diagnosticOutput",
        "$diagnosticExit -ne 0",
        "$successExit -ne 0",
        "$blockedExit -ne 2",
        "-File $launcher",
        "$rawSuccessExit",
        "$rawBlockedExit",
        "cmd.exe /c exit 0",
    ):
        if marker not in windows_content:
            errors.append(
                ".github/workflows/ci_windows.yml: missing native hook smoke "
                f"marker `{marker}`"
            )


def run_checks(root: Path) -> list[str]:
    errors: list[str] = []
    check_required_files(root, errors)
    check_required_files_are_not_ignored(root, errors)
    check_branch_profile(root, errors)
    check_codex_config(root, errors)
    check_hooks(root, errors)
    check_pixi_references(root, errors)
    check_test_gate_contract(root, errors)
    check_pytest_source_policies(root, errors)
    check_path_references(root, errors)
    check_markdown_links(root, errors)
    check_instruction_budget(root, errors)
    check_release_guidance(root, errors)
    check_ci_wiring(root, errors)
    return errors


def instruction_chain(root: Path, start_dir: str) -> list[str]:
    """Return the root-to-start AGENTS.md discovery chain."""
    start = (root / start_dir).resolve()
    if start.is_file():
        start = start.parent
    try:
        relative = start.relative_to(root)
    except ValueError:
        return []

    directories = [root]
    current = root
    for part in relative.parts:
        current /= part
        directories.append(current)
    return [
        (directory / "AGENTS.md").relative_to(root).as_posix()
        for directory in directories
        if (directory / "AGENTS.md").is_file()
    ]


def scenario_gate_error(root: Path, command: Any, pixi_tasks: set[str]) -> str | None:
    """Validate one non-interactive scenario gate without executing it."""
    if not isinstance(command, str) or re.search(
        r"(?:&&|\|\||[;|<>`\n]|\$\()", command
    ):
        return f"invalid gate command `{command}`"
    try:
        tokens = shlex.split(command)
    except ValueError:
        return f"invalid gate quoting `{command}`"
    if tokens[:2] != ["pixi", "run"]:
        return f"non-Pixi gate command `{command}`"

    index = 2
    if tokens[index : index + 1] == ["-e"]:
        if len(tokens) <= index + 2:
            return f"incomplete Pixi environment gate `{command}`"
        index += 2
    if tokens[index : index + 1] == ["--skip-deps"]:
        index += 1
    if index >= len(tokens):
        return f"missing Pixi task or command `{command}`"

    task = tokens[index]
    arguments = tokens[index + 1 :]
    if task in pixi_tasks:
        if arguments:
            return f"Pixi task gate has unexpected arguments `{command}`"
        return None
    if task != "python":
        return f"unknown gate task `{task}`"
    if not arguments:
        return f"Python gate is missing a script or module `{command}`"
    if (
        arguments[0] == "scripts/check_ai_infrastructure.py"
        and "--check" in arguments[1:]
    ):
        return (
            "structural-only AI checker is not completion evidence; "
            "use `pixi run check-ai-infra`"
        )

    repo_references: list[str] = []
    if arguments[:2] == ["-m", "pytest"]:
        repo_references = [
            token for token in arguments[2:] if not token.startswith("-")
        ]
        if not repo_references:
            return f"pytest gate names no repository test `{command}`"
    elif arguments[0].startswith(("scripts/", "tests/")):
        repo_references = [arguments[0]]
    else:
        return f"unsupported direct Python gate `{command}`"

    for reference in repo_references:
        path_token = reference.split("::", 1)[0]
        relative = repository_relative_path(root, path_token)
        if relative is None:
            return f"Python gate path escapes repository `{path_token}`"
        if not (root / relative).is_file():
            return f"Python gate references missing file `{path_token}`"
    return None


def exercise_scenarios(
    root: Path, data: dict[str, Any] | None = None, *, emit: bool = True
) -> list[str]:
    errors: list[str] = []
    path = root / "docs" / "ai" / "agent-scenarios.json"
    if data is None and not path.exists():
        return ["docs/ai/agent-scenarios.json: missing scenario inventory"]
    try:
        data = data if data is not None else read_json(path)
        capabilities = read_json(root / "docs" / "ai" / "capabilities.json")
        branch_profile = read_json(root / "docs" / "ai" / "branch-profile.json")
        pixi_tasks = collect_task_names(read_toml(root / "pixi.toml"))
    except (OSError, json.JSONDecodeError, tomllib.TOMLDecodeError) as error:
        return [f"scenario inventory: invalid JSON: {error}"]
    if not isinstance(data, dict):
        return ["docs/ai/agent-scenarios.json: top level must be an object"]
    if not isinstance(capabilities, dict):
        return ["docs/ai/capabilities.json: top level must be an object"]
    if not isinstance(branch_profile, dict):
        return ["docs/ai/branch-profile.json: top level must be an object"]
    if set(data) != SCENARIO_TOP_LEVEL_KEYS:
        errors.append(
            "scenario inventory: top-level keys must be "
            f"{sorted(SCENARIO_TOP_LEVEL_KEYS)}"
        )

    capability_names: dict[str, set[str]] = {}
    for kind, field in (
        ("workflow", "workflows"),
        ("domain_skill", "domain_skills"),
    ):
        items = capabilities.get(field)
        if not isinstance(items, list) or not all(
            isinstance(item, dict)
            and isinstance(item.get("name"), str)
            and item["name"].strip()
            for item in items
        ):
            errors.append(
                f"capability inventory: `{field}` must be a list of named objects"
            )
            capability_names[kind] = set()
        else:
            capability_names[kind] = {item["name"] for item in items}
    scenarios = data.get("scenarios")
    if (
        type(data.get("schema_version")) is not int
        or data["schema_version"] != 1
        or not isinstance(scenarios, list)
    ):
        return ["docs/ai/agent-scenarios.json: invalid schema"]
    if data.get("profile") != branch_profile.get("profile"):
        errors.append("scenario inventory: profile does not match branch profile")

    expected_ids = {
        "orientation",
        "small-change",
        "failure-diagnosis",
        "documentation-update",
        "model-upgrade",
        "component-work",
        "simulation-verification",
        "release-maintenance",
    }
    actual_ids = {
        scenario.get("id")
        for scenario in scenarios
        if isinstance(scenario, dict) and isinstance(scenario.get("id"), str)
    }
    if actual_ids != expected_ids or len(scenarios) != len(expected_ids):
        errors.append(
            "scenario inventory: expected exactly "
            f"{sorted(expected_ids)}, got {sorted(str(item) for item in actual_ids)}"
        )

    profile_forbidden_value = branch_profile.get("forbidden_paths")
    if not nonempty_string_list(profile_forbidden_value):
        errors.append(
            "branch profile: `forbidden_paths` must be a non-empty string list"
        )
        profile_forbidden: set[str] = set()
    else:
        profile_forbidden = set(profile_forbidden_value)

    seen: set[str] = set()
    for index, scenario in enumerate(scenarios):
        if not isinstance(scenario, dict):
            errors.append(f"scenario index {index}: must be an object")
            continue
        local_errors: list[str] = []
        raw_scenario_id = scenario.get("id")
        scenario_id = (
            raw_scenario_id
            if isinstance(raw_scenario_id, str) and raw_scenario_id.strip()
            else f"index {index}"
        )
        missing_keys = SCENARIO_KEYS - set(scenario)
        extra_keys = set(scenario) - SCENARIO_KEYS - SCENARIO_OPTIONAL_KEYS
        if missing_keys or extra_keys:
            local_errors.append(
                f"keys must contain {sorted(SCENARIO_KEYS)} and only optional "
                f"{sorted(SCENARIO_OPTIONAL_KEYS)}"
            )
        if not isinstance(raw_scenario_id, str) or not raw_scenario_id.strip():
            local_errors.append("id must be a non-empty string")
        elif raw_scenario_id in seen:
            local_errors.append("is duplicated")
        else:
            seen.add(raw_scenario_id)

        prompt_class = scenario.get("prompt_class")
        if not isinstance(prompt_class, str) or not prompt_class.strip():
            local_errors.append("prompt_class must be a non-empty string")
        start_dir = scenario.get("start_dir")
        start_relative = repository_relative_path(root, start_dir)
        if start_relative is None:
            local_errors.append(f"invalid repository-relative start_dir `{start_dir}`")
        elif not (root / start_relative).exists():
            local_errors.append(f"missing start_dir `{start_dir}`")
        else:
            actual_chain = instruction_chain(root, start_dir)
            configured_chain = scenario.get("instruction_chain")
            if not nonempty_string_list(configured_chain):
                local_errors.append("instruction_chain must be a non-empty string list")
            elif not all(
                repository_relative_path(root, item) is not None
                for item in configured_chain
            ):
                local_errors.append(
                    "instruction_chain must contain repository-relative paths"
                )
            elif configured_chain != actual_chain:
                local_errors.append(
                    f"instruction_chain must be {actual_chain}, "
                    f"got {configured_chain!r}"
                )

        route = scenario.get("expected_route")
        if not isinstance(route, dict):
            local_errors.append("missing expected_route")
            route = {}
        elif set(route) != ROUTE_KEYS:
            local_errors.append(f"expected_route keys must be {sorted(ROUTE_KEYS)}")
        kind = route.get("kind")
        route_name = route.get("name")
        route_path = route.get("path")
        if kind == "agent":
            if not isinstance(route_name, str) or route_name not in EXPECTED_AGENTS:
                local_errors.append(f"unknown agent route `{route_name}`")
                expected_route_path = None
            else:
                expected_route_path = f".codex/agents/{route_name}.toml"
        elif isinstance(kind, str) and kind in capability_names:
            if (
                not isinstance(route_name, str)
                or route_name not in capability_names[kind]
            ):
                local_errors.append(f"unknown {kind} route `{route_name}`")
                expected_route_path = None
            else:
                expected_route_path = f".agents/skills/{route_name}/SKILL.md"
        else:
            local_errors.append(f"unknown route kind `{kind}`")
            expected_route_path = None
        route_relative = repository_relative_path(root, route_path)
        if route_relative is None:
            local_errors.append(
                f"invalid repository-relative route path `{route_path}`"
            )
        elif not (root / route_relative).is_file():
            local_errors.append(f"missing route path `{route_path}`")
        if expected_route_path and route_path != expected_route_path:
            local_errors.append(
                f"route path must be `{expected_route_path}`, got `{route_path}`"
            )
        specialist_agent = scenario.get("specialist_agent")
        if specialist_agent is not None and (
            not isinstance(specialist_agent, str)
            or specialist_agent not in EXPECTED_AGENTS
        ):
            local_errors.append(f"unknown specialist agent `{specialist_agent}`")

        owner_docs = scenario.get("owner_docs")
        if not nonempty_string_list(owner_docs):
            local_errors.append("owner_docs must be a non-empty string list")
            owner_docs = []
        for doc in owner_docs:
            relative = repository_relative_path(root, doc)
            if relative is None:
                local_errors.append(f"invalid repository-relative owner doc `{doc}`")
            elif not (root / relative).is_file():
                local_errors.append(f"missing owner doc `{doc}`")

        permitted = scenario.get("permitted_scopes")
        if not nonempty_string_list(permitted):
            local_errors.append("permitted_scopes must be a non-empty string list")
            permitted = []

        for gate_field in ("focused_gates", "full_gates"):
            gates = scenario.get(gate_field)
            if not nonempty_string_list(gates):
                local_errors.append(f"{gate_field} must be a non-empty string list")
                continue
            for command in gates:
                gate_error = scenario_gate_error(root, command, pixi_tasks)
                if gate_error:
                    local_errors.append(f"{gate_field}: {gate_error}")

        if scenario_id == "model-upgrade":
            expected_prompt = (
                "audit or update DART 6.20 AI infrastructure for a named model, "
                "reasoning mode, or coding-agent release"
            )
            if scenario.get("prompt_class") != expected_prompt:
                local_errors.append("model upgrade has the wrong trigger prompt")
            if (kind, route_name) != ("workflow", "dart-model-upgrade"):
                local_errors.append("model upgrade must route to dart-model-upgrade")
            if "docs/ai/verification.md" not in owner_docs:
                local_errors.append(
                    "model upgrade is missing its simulation evidence owner"
                )
            if scenario.get("focused_gates") != ["pixi run check-ai-infra"]:
                local_errors.append("model upgrade is missing its focused AI gate")
            if scenario.get("full_gates") != [
                "pixi run test-ai-infra",
                "pixi run lint",
            ]:
                local_errors.append("model upgrade is missing its full AI gates")
            workflow_path = root / ".claude/commands/dart-model-upgrade.md"
            try:
                workflow_text = workflow_path.read_text(encoding="utf-8")
            except OSError:
                workflow_text = ""
            for marker in (
                "Use these verdicts",
                "existing model/settings",
                "one lower effort",
                "structural",
                "Deeper reasoning modes give one difficult task more time",
                "explicit user authorization for delegation",
                '`docs/ai/README.md` § "Updating Models And Coding Agents"',
                "Do not substitute a model",
                "durable project context",
                "`docs/plans/dashboard.md`",
                "without hidden",
                "apply/adapt/omit",
                "`audit-only` is read-only",
                "workflow is itself an audit surface",
                "do not clone the workflow",
                "representative DART 6 physics investigation",
                "text correctness",
                "Images are never the",
                "text/image disagreement",
                "C++17",
                "pybind11",
                "`dart::utils`",
                "OSG",
                "configured CMake File API result",
                "source-marker matches",
            ):
                if marker not in workflow_text:
                    local_errors.append(
                        "dart-model-upgrade source is missing contract marker "
                        f"{marker!r}"
                    )

        if scenario_id == "simulation-verification":
            expected_scopes = {
                "dart/simulation",
                "dart/dynamics",
                "dart/collision",
                "dart/constraint",
                "dart/gui",
                "dart/utils",
                "python",
                "examples",
                "tutorials",
                "matching tests",
                "temporary claim-tied evidence",
            }
            expected_prompt = (
                "verify claim-dependent DART 6.20 simulation, dynamics, "
                "collision/contact/constraints, model/scene, GUI, or OSG behavior"
            )
            if scenario.get("prompt_class") != expected_prompt:
                local_errors.append(
                    "simulation verification has the wrong claim-dependent prompt"
                )
            if (kind, route_name) != ("domain_skill", "dart-verify-sim"):
                local_errors.append(
                    "simulation verification must route to dart-verify-sim"
                )
            if "docs/ai/verification.md" not in owner_docs:
                local_errors.append(
                    "simulation verification is missing its visual evidence owner"
                )
            if not expected_scopes.issubset(set(permitted)):
                local_errors.append(
                    "simulation verification must cover simulation, dynamics, "
                    "collision, constraints, GUI, model loading, Python, "
                    "examples, tests, and temporary evidence"
                )
            if scenario.get("evidence_policy") != (
                "text-first-with-claim-tied-visual-or-documented-exception"
            ):
                local_errors.append(
                    "simulation verification has the wrong evidence policy"
                )
            if scenario.get("semantic_review_policy") != (
                "native-image-inspection-or-image-capable-handoff-with-"
                "explicit-limitation"
            ):
                local_errors.append(
                    "simulation verification has the wrong semantic review policy"
                )
            if scenario.get("focused_gates") != ["pixi run test"]:
                local_errors.append(
                    "simulation verification is missing its focused correctness gate"
                )
            if scenario.get("full_gates") != [
                "pixi run test-py",
                "pixi run test-all",
            ]:
                local_errors.append(
                    "simulation verification is missing its full correctness gates"
                )
            skill_path = root / ".claude/skills/dart-verify-sim/SKILL.md"
            try:
                skill_text = skill_path.read_text(encoding="utf-8")
            except OSError:
                skill_text = ""
            for marker in (
                "text correctness oracle",
                "agent-capture",
                "image-verdict",
                "sole correctness oracle",
                "bm-boxes-headless",
                "Xvfb",
                "--factory module:callable",
                "model/scene loading",
                "collision/contact/constraints",
                "simulation stepping",
                "OSG rendering",
                "visual example",
                "OSG capture is unavailable",
                "replacement evidence",
                "settled-contact",
                "text/geometry oracle",
                "test-agent-debug-overlay",
                "/tmp/dart-visual-evidence/capture_auto0.png",
                "semantic inspection",
                "native image viewer",
                "original detail",
                "do not average",
                "verification-bundle",
                "limiting horizontal or",
                "no-bounded-renderable",
            ):
                if marker not in skill_text:
                    local_errors.append(
                        f"dart-verify-sim source is missing contract marker {marker!r}"
                    )
            try:
                new_task_text = (root / ".claude/commands/dart-new-task.md").read_text(
                    encoding="utf-8"
                )
            except OSError:
                new_task_text = ""
            for marker in (
                "route through `dart-verify-sim`",
                "record why it is unavailable or not applicable",
            ):
                if marker not in new_task_text:
                    local_errors.append(
                        "dart-new-task is missing the conditional simulation "
                        f"marker {marker!r}"
                    )
            consumer_markers = {
                "AGENTS.md": ("docs/ai/verification.md", "dart-verify-sim"),
                ".codex/agents/dart_scout.toml": (
                    "text correctness oracle",
                    "claim-tied assessed visual",
                ),
                ".codex/agents/dart_reviewer.toml": (
                    "text-first evidence",
                    "OSG/debug-overlay",
                ),
                ".claude/commands/dart-new-task.md": (
                    "route through `dart-verify-sim`",
                    "record why it is unavailable or not applicable",
                ),
                ".claude/commands/dart-ultrawork.md": (
                    "routes through `dart-verify-sim`",
                ),
                ".claude/commands/dart-resume.md": ("route through `dart-verify-sim`",),
                ".claude/commands/dart-pr.md": ("use `dart-verify-sim`",),
                ".claude/commands/dart-manage-pr.md": ("Visual verification",),
                ".claude/commands/dart-review-pr.md": (
                    "require the `dart-verify-sim` text oracle",
                    "justified replacement",
                ),
                ".claude/skills/dart-build/SKILL.md": ("dart-verify-sim",),
                ".claude/skills/dart-test/SKILL.md": (
                    "load `dart-verify-sim`",
                    "capture is unavailable or not applicable",
                ),
                ".claude/skills/dart-io/SKILL.md": (
                    "also load `dart-verify-sim`",
                    "claim-tied OSG visual corroboration",
                ),
                ".claude/skills/dart-python/SKILL.md": (
                    "load `dart-verify-sim`",
                    "focused Python text/behavior oracle",
                    "collision/contact/constraints",
                    "GUI/OSG output",
                    "visual exception",
                ),
                ".claude/skills/dart-ci/SKILL.md": (
                    "also load `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-downstream-fix.md": (
                    "route through `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-backport-pr.md": (
                    "release branch's `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-release-ci-fix.md": (
                    "use `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-fix-ci.md": (
                    "use `dart-verify-sim`",
                    "collision/contact/constraints",
                    "visual exception",
                ),
                "docs/ai/verification.md": (
                    "DebugOverlay",
                    "text correctness",
                    "required renderer",
                    "Name the replacement",
                ),
            }
            for relative, markers in consumer_markers.items():
                try:
                    consumer_text = (root / relative).read_text(encoding="utf-8")
                except OSError:
                    consumer_text = ""
                for marker in markers:
                    if marker not in consumer_text:
                        local_errors.append(
                            f"{relative}: missing simulation route marker {marker!r}"
                        )

        recovery = scenario.get("recovery")
        recovery_relative = repository_relative_path(root, recovery)
        if recovery_relative is None:
            local_errors.append(
                f"invalid repository-relative recovery pointer `{recovery}`"
            )
        elif not (root / recovery_relative).is_file():
            local_errors.append(f"missing recovery pointer `{recovery}`")

        forbidden_paths = scenario.get("forbidden_paths")
        if not nonempty_string_list(forbidden_paths):
            local_errors.append("forbidden_paths must be a non-empty string list")
            forbidden_paths = []
        if not set(forbidden_paths).issubset(profile_forbidden):
            local_errors.append("forbidden_paths are not release-profile paths")
        route_and_scope = json.dumps(
            {
                "expected_route": route,
                "specialist_agent": specialist_agent,
                "owner_docs": owner_docs,
                "permitted_scopes": permitted,
                "focused_gates": scenario.get("focused_gates"),
                "full_gates": scenario.get("full_gates"),
                "recovery": recovery,
            }
        )
        for forbidden in forbidden_paths:
            relative = repository_relative_path(root, forbidden)
            if relative is None:
                local_errors.append(
                    f"invalid repository-relative forbidden path `{forbidden}`"
                )
                continue
            if (root / relative).exists():
                local_errors.append(f"forbidden main-only path exists `{forbidden}`")
            if forbidden in route_and_scope:
                local_errors.append(
                    f"forbidden path leaks into route/scope `{forbidden}`"
                )

        errors.extend(f"scenario `{scenario_id}`: {error}" for error in local_errors)
        if not local_errors and emit:
            suffix = f" -> {specialist_agent}" if specialist_agent else ""
            print(f"PASS {scenario_id}: {kind}:{route_name}{suffix}")
    return errors


def version(command: str) -> str:
    executable = shutil.which(command)
    if not executable:
        return "unavailable"
    try:
        result = subprocess.run(
            [executable, "--version"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return "unavailable"
    return (result.stdout or result.stderr).strip().splitlines()[0]


def _path_inventory(paths: list[Path], root: Path) -> dict[str, Any]:
    relative = sorted(
        path.relative_to(root).as_posix() for path in paths if path.exists()
    )
    return {"count": len(relative), "paths": relative}


def _line_inventory(paths: list[Path], root: Path) -> dict[str, Any]:
    entries = [
        {
            "path": path.relative_to(root).as_posix(),
            "lines": len(
                path.read_text(encoding="utf-8", errors="replace").splitlines()
            ),
        }
        for path in paths
        if path.is_file()
    ]
    longest = (
        max(entries, key=lambda entry: (entry["lines"], entry["path"]))
        if entries
        else {"path": "", "lines": 0}
    )
    return {
        "count": len(entries),
        "total_lines": sum(entry["lines"] for entry in entries),
        "longest": longest,
    }


def _instruction_context_inventory(root: Path) -> dict[str, Any]:
    chains: list[dict[str, Any]] = []
    for target in tracked_agent_files(root):
        directory = target.parent
        chain: list[Path] = []
        while directory == root or root in directory.parents:
            candidate = directory / "AGENTS.md"
            if candidate.is_file():
                chain.append(candidate)
            if directory == root:
                break
            directory = directory.parent
        relative_chain = [path.relative_to(root).as_posix() for path in reversed(chain)]
        chains.append(
            {
                "target": target.relative_to(root).as_posix(),
                "paths": relative_chain,
                "bytes": sum(path.stat().st_size for path in chain),
            }
        )
    largest = (
        max(chains, key=lambda entry: (entry["bytes"], entry["target"]))
        if chains
        else {"target": "", "paths": [], "bytes": 0}
    )
    return {
        "file_count": len(chains),
        "largest_chain": largest,
        "repository_limit_bytes": MAX_AGENT_INSTRUCTION_BYTES,
    }


def _managed_generated_skills(root: Path) -> list[Path]:
    manifest_path = root / ".agents" / "skills" / ".dart-generated.json"
    try:
        manifest = read_json(manifest_path)
    except (OSError, json.JSONDecodeError):
        return []
    if (
        not isinstance(manifest, dict)
        or type(manifest.get("schema_version")) is not int
        or manifest["schema_version"] != 1
        or manifest.get("generator") != "scripts/sync_ai_commands.py"
        or not isinstance(manifest.get("paths"), list)
    ):
        return []
    managed: list[Path] = []
    for relative in manifest["paths"]:
        if not isinstance(relative, str) or not re.fullmatch(
            r"[a-z0-9][a-z0-9-]*/SKILL\.md", relative
        ):
            continue
        path = manifest_path.parent / relative
        if path.is_file() and not path.is_symlink() and not path.parent.is_symlink():
            managed.append(path)
    return sorted(managed)


def _generated_skill_metadata_chars(paths: list[Path]) -> int:
    total = 0
    for path in paths:
        text = path.read_text(encoding="utf-8", errors="replace")
        frontmatter = text.split("---", 2)
        if len(frontmatter) < 3:
            continue
        for line in frontmatter[1].splitlines():
            key, separator, value = line.partition(":")
            if separator and key.strip() in {"name", "description"}:
                total += len(value.strip())
    return total


def _model_harness_inventory(root: Path) -> dict[str, Any]:
    commands = sorted((root / ".claude" / "commands").glob("*.md"))
    skills = sorted((root / ".claude" / "skills").glob("*/SKILL.md"))
    agents = sorted((root / ".codex" / "agents").glob("*.toml"))
    try:
        config = read_toml(root / ".codex" / "config.toml")
    except (OSError, tomllib.TOMLDecodeError):
        config = {}

    project_pins = sorted(
        key
        for key in ("model", "model_reasoning_effort", "review_model")
        if key in config
    )
    agent_config = config.get("agents")
    displayed_agent_config = (
        {
            str(key): (
                value
                if isinstance(value, (str, int, float, bool)) or value is None
                else str(value)
            )
            for key, value in agent_config.items()
        }
        if isinstance(agent_config, dict)
        else {}
    )
    project_agent_pins = sorted(
        key
        for key in (
            "default_subagent_model",
            "default_subagent_reasoning_effort",
        )
        if isinstance(agent_config, dict) and key in agent_config
    )
    custom_agent_pins: list[dict[str, Any]] = []
    for path in agents:
        try:
            profile = read_toml(path)
        except (OSError, tomllib.TOMLDecodeError):
            profile = {}
        pins = sorted(
            key
            for key in ("model", "model_reasoning_effort", "review_model")
            if key in profile
        )
        if pins:
            custom_agent_pins.append(
                {"path": path.relative_to(root).as_posix(), "keys": pins}
            )

    return {
        "model_pins": {
            "project": project_pins,
            "project_agents": project_agent_pins,
            "custom_agents": custom_agent_pins,
        },
        "project_agent_config": displayed_agent_config,
        "instruction_context": _instruction_context_inventory(root),
        "workflow_sources": _line_inventory(commands, root),
        "domain_skill_sources": _line_inventory(skills, root),
        "generated_skill_metadata_chars": _generated_skill_metadata_chars(
            _managed_generated_skills(root)
        ),
    }


def _durable_context_inventory(root: Path) -> dict[str, Any]:
    owner_paths = [
        root / "docs" / "ai" / "north-star.md",
        root / "docs" / "ai" / "sessions.md",
        root / "docs" / "plans" / "dashboard.md",
        root / "docs" / "plans" / "north-star-roadmap.md",
        root / "docs" / "dev_tasks" / "README.md",
    ]
    plans = sorted((root / "docs" / "plans").glob("*.md"))
    dev_tasks_root = root / "docs" / "dev_tasks"
    active_tasks = (
        sorted(
            path
            for path in dev_tasks_root.iterdir()
            if path.is_dir() and not path.is_symlink()
        )
        if dev_tasks_root.is_dir()
        else []
    )
    resume_surfaces = sorted(
        path
        for task in active_tasks
        for path in task.glob("RESUME.md")
        if path.is_file()
    )
    handoff_surfaces = sorted(
        path
        for task in active_tasks
        for path in task.glob("HANDOFF*.md")
        if path.is_file()
    )
    return {
        "owners": _path_inventory(owner_paths, root),
        "plans": _path_inventory(plans, root),
        "active_dev_tasks": {
            "count": len(active_tasks),
            "paths": [path.relative_to(root).as_posix() for path in active_tasks],
        },
        "resume_surfaces": _path_inventory(resume_surfaces, root),
        "handoff_surfaces": _path_inventory(handoff_surfaces, root),
    }


def _visual_verification_inventory(root: Path) -> dict[str, Any]:
    paths = [
        root / ".claude" / "skills" / "dart-verify-sim" / "SKILL.md",
        root / "docs" / "ai" / "verification.md",
        root / "scripts" / "agent_capture.py",
        root / "scripts" / "agent_view_quality.py",
        root / "scripts" / "evidence_select.py",
        root / "scripts" / "evidence_publish.py",
        root / "scripts" / "image_verdict.py",
        root / "scripts" / "verification_bundle.py",
        root / "python" / "dartpy" / "gui" / "osg",
    ]
    try:
        tasks = collect_task_names(read_toml(root / "pixi.toml"))
    except (OSError, tomllib.TOMLDecodeError):
        tasks = set()
    visual_tasks = {
        "agent-capture",
        "evidence-publish",
        "evidence-select",
        "image-verdict",
        "test-agent-debug-overlay",
        "verification-bundle",
    }
    return {
        "backend": "OSG",
        "paths": _path_inventory(paths, root),
        "tasks": sorted(visual_tasks & tasks),
        "display": os.environ.get("DISPLAY", ""),
        "xvfb_run": shutil.which("xvfb-run") or "unavailable",
        "semantic_review_contract": (
            "native-image-inspection-or-image-capable-handoff"
        ),
    }


def _working_tree_state(root: Path) -> str:
    result = subprocess.run(
        ["git", "status", "--short"],
        cwd=root,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return "unavailable"
    return "dirty" if result.stdout.strip() else "clean"


def doctor_report(root: Path) -> dict[str, Any]:
    root = root.resolve()
    branch = subprocess.run(
        ["git", "branch", "--show-current"],
        cwd=root,
        capture_output=True,
        text=True,
    ).stdout.strip()
    errors = run_checks(root)
    commands = sorted((root / ".claude" / "commands").glob("*.md"))
    skills = sorted((root / ".claude" / "skills").glob("*/SKILL.md"))
    generated = sorted((root / ".agents" / "skills").glob("*/SKILL.md"))
    agents = sorted((root / ".codex" / "agents").glob("*.toml"))
    return {
        "schema_version": 1,
        "root": str(root),
        "branch": branch or "(detached)",
        "profile": {
            "name": "release-6.20",
            "cpp_standard": "C++17",
            "python_binding": "pybind11",
            "io_namespace": "dart::utils",
            "gui_backend": "OSG",
        },
        "working_tree": _working_tree_state(root),
        "tools": {
            "python": sys.version.split()[0],
            "pixi": version("pixi"),
            "codex": version("codex"),
            "claude": version("claude"),
            "opencode": version("opencode"),
        },
        "inventory": {
            "source_commands": _path_inventory(commands, root),
            "source_skills": _path_inventory(skills, root),
            "generated_skills": {
                **_path_inventory(generated, root),
                "manifest": ".agents/skills/.dart-generated.json",
            },
            "custom_agents": _path_inventory(agents, root),
            "model_harness": _model_harness_inventory(root),
            "durable_context": _durable_context_inventory(root),
            "visual_verification": _visual_verification_inventory(root),
        },
        "errors": errors,
        "ok": not errors,
    }


def print_doctor(data: dict[str, Any]) -> None:
    state = "PASS" if data["ok"] else "FAIL"
    profile = data["profile"]
    inventory = data["inventory"]
    model_harness = inventory["model_harness"]
    print(f"DART 6 AI doctor: {state}")
    print(f"  repository: {data['root']}")
    print(f"  branch: {data['branch']}")
    print(
        "  profile: "
        f"{profile['name']} ({profile['cpp_standard']}, "
        f"{profile['python_binding']}, {profile['io_namespace']}, "
        f"{profile['gui_backend']})"
    )
    print(f"  working tree: {data['working_tree']}")
    print(
        "  inventory: "
        f"{inventory['source_commands']['count']} commands, "
        f"{inventory['source_skills']['count']} source skills, "
        f"{inventory['generated_skills']['count']} generated skills, "
        f"{inventory['custom_agents']['count']} custom agents, "
        f"{inventory['durable_context']['active_dev_tasks']['count']} "
        "active dev tasks"
    )
    project_pins = len(model_harness["model_pins"]["project"]) + len(
        model_harness["model_pins"]["project_agents"]
    )
    print(
        "  model harness: "
        f"{project_pins} project pins, "
        f"{len(model_harness['model_pins']['custom_agents'])} custom-agent pins, "
        f"{model_harness['workflow_sources']['longest']['lines']}/"
        f"{model_harness['domain_skill_sources']['longest']['lines']} "
        "lines in longest workflow/domain skill, "
        f"{model_harness['generated_skill_metadata_chars']} generated "
        "skill-metadata chars"
    )
    visual = inventory["visual_verification"]
    print(
        "  visual verification: "
        f"{visual['backend']}, {len(visual['tasks'])} tasks, "
        f"DISPLAY={visual['display'] or '(unset)'}, xvfb-run={visual['xvfb_run']}"
    )
    for tool, tool_version in data["tools"].items():
        print(f"  {tool}: {tool_version}")
    print("Trust: project agents and hooks load only after the repository is trusted")
    print("Hook inspection: use `/hooks` in Codex; git hook: `pixi run install-hooks`")
    for error in data["errors"]:
        print(f"  ERROR: {error}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="run structural checks")
    modes.add_argument("--doctor", action="store_true", help="print diagnostics")
    modes.add_argument(
        "--scenarios", action="store_true", help="exercise deterministic scenarios"
    )
    parser.add_argument(
        "--json", action="store_true", help="emit machine-readable doctor JSON"
    )
    parser.add_argument(
        "--semantic-cmake",
        action="store_true",
        help="reconfigure and validate the effective CMake test graph",
    )
    parser.add_argument(
        "--cmake-build-dir",
        type=Path,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--repo-root", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = (args.repo_root or Path(__file__).resolve().parents[1]).resolve()

    if args.json and not args.doctor:
        parser.error("--json requires --doctor")
    if args.semantic_cmake and (args.doctor or args.scenarios):
        parser.error("--semantic-cmake is only valid with --check")

    if args.doctor:
        data = doctor_report(root)
        if args.json:
            print(json.dumps(data, indent=2, sort_keys=True))
        else:
            print_doctor(data)
        return 0 if data["ok"] else 1
    if args.scenarios:
        errors = exercise_scenarios(root)
    else:
        errors = run_checks(root)
        if args.semantic_cmake:
            check_test_runner_semantics(root, errors)
            build_dir = args.cmake_build_dir or discover_cmake_build_dir(root)
            if build_dir is None:
                errors.append(
                    "CMake semantic test graph: no configured Pixi build tree found"
                )
            else:
                check_cmake_test_graph(root, build_dir, errors)

    if errors:
        for error in errors:
            print(f"ERROR {error}", file=sys.stderr)
        return 1
    print("AI infrastructure checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
