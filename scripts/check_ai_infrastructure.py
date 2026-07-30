#!/usr/bin/env python3
"""Validate and diagnose DART 6.20's repository-local AI infrastructure."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import shutil
import subprocess
import sys
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


def is_single_cmake_all_build(command: str) -> bool:
    """Return whether a task is exactly one CMake build of only target ALL."""
    tokens = shell_tokens(command)
    return (
        not has_shell_control_syntax(command)
        and tokens[:2] == ["cmake", "--build"]
        and tokens.count("cmake") == 1
        and tokens.count("--target") == 1
        and tokens[-2:] == ["--target", "ALL"]
        and "--" not in tokens
    )


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
                (
                    "tests_and_run COMMAND ${CMAKE_CTEST_COMMAND} "
                    "--output-on-failure -C $<CONFIG> DEPENDS "
                    "${integration_tests} ${regression_tests} ${unit_tests}"
                ),
                (),
                ("if:MSVC",),
                "${CMAKE_CTEST_COMMAND} --output-on-failure -C $<CONFIG>",
            ),
            (
                "add_custom_target",
                (
                    "tests_and_run COMMAND ${CMAKE_CTEST_COMMAND} "
                    "--output-on-failure DEPENDS ${integration_tests} "
                    "${regression_tests} ${unit_tests}"
                ),
                (),
                ("else:MSVC",),
                "${CMAKE_CTEST_COMMAND} --output-on-failure",
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
                        "${CMAKE_COMMAND}",
                        "-E",
                        "env",
                        "PYTHONPATH=${DART_PYTHONPATH}",
                        "${Python3_EXECUTABLE}",
                        "-m",
                        "pytest",
                        "${dartpy_test_files}",
                        "-v",
                    ),
                    ("DEPENDS", "dartpy"),
                ),
                ("if:DARTPY_PYTEST_FOUND",),
                '"${Python3_EXECUTABLE}" -m pytest',
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


def check_test_gate_contract(root: Path, errors: list[str]) -> None:
    """Keep the Release ALL graph and focused test gates unambiguous."""
    try:
        pixi = read_toml(root / "pixi.toml")
    except (OSError, tomllib.TOMLDecodeError) as error:
        errors.append(f"pixi.toml: invalid TOML: {error}")
        return

    task_markers = {
        "test": "ctest",
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
            "`pixi run test` runs the C++ test suite",
            "`pixi run test-py`",
            "`pixi run test-all` builds the",
            "`tests_and_run` and `pytest`",
            "does not run lint",
            "CMake's File API",
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
    for candidate in (base / build_type, base):
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


def check_pytest_module_provenance(
    python_executable: str,
    working_directory: Path,
    pythonpath: str,
    errors: list[str],
) -> None:
    """Require target-context pytest imports to resolve inside the Pixi prefix."""
    prefix = "CMake semantic test graph"
    environment = {**os.environ, "PYTHONPATH": pythonpath}
    try:
        result = subprocess.run(
            [
                python_executable,
                "-c",
                (
                    "from pathlib import Path; import pytest; "
                    "print(Path(pytest.__file__).resolve())"
                ),
            ],
            cwd=working_directory,
            env=environment,
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
    module_path = Path(output[0]).resolve()
    try:
        module_path.relative_to(Path(sys.prefix).resolve())
    except ValueError:
        errors.append(
            f"{prefix}: pytest resolves outside the active Pixi environment "
            f"at `{module_path}`"
        )


def check_pytest_execution_options(root: Path, errors: list[str]) -> None:
    """Reject repository or ambient pytest options that suppress execution."""
    prefix = "CMake semantic test graph"
    options: list[str] = []
    config = root / "pyproject.toml"
    if config.is_file():
        try:
            data = read_toml(config)
        except (OSError, tomllib.TOMLDecodeError) as error:
            errors.append(f"{prefix}: invalid pytest configuration: {error}")
            return
        tool = data.get("tool")
        pytest = tool.get("pytest") if isinstance(tool, dict) else None
        ini = pytest.get("ini_options") if isinstance(pytest, dict) else None
        addopts = ini.get("addopts", []) if isinstance(ini, dict) else []
        if isinstance(addopts, str):
            try:
                options.extend(shlex.split(addopts))
            except ValueError as error:
                errors.append(f"{prefix}: invalid pytest addopts: {error}")
                return
        elif isinstance(addopts, list) and all(
            isinstance(option, str) for option in addopts
        ):
            options.extend(addopts)
        else:
            errors.append(f"{prefix}: pytest addopts must be strings")
            return
    ambient = os.environ.get("PYTEST_ADDOPTS")
    if ambient:
        try:
            options.extend(shlex.split(ambient))
        except ValueError as error:
            errors.append(f"{prefix}: invalid ambient PYTEST_ADDOPTS: {error}")
            return
    forbidden = {
        "--co",
        "--collect-only",
        "--fixtures",
        "--fixtures-per-test",
        "--help",
        "--markers",
        "--setup-only",
        "--setup-plan",
        "--version",
        "-h",
        "-V",
    }
    suppressed = sorted(forbidden.intersection(options))
    if suppressed:
        errors.append(
            f"{prefix}: pytest execution is suppressed by options {suppressed}"
        )


def check_cmake_ctest_inventory(
    build_dir: Path,
    configuration: str,
    ctest_executable: str,
    test_targets: dict[str, dict[str, Any]],
    errors: list[str],
) -> None:
    """Require every configured C++ test target to be registered with CTest."""
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

    registered_commands: set[str] = set()
    registered_tests: dict[str, list[str] | None] = {}
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
        properties = test.get("properties", [])
        if isinstance(properties, list) and any(
            isinstance(prop, dict)
            and prop.get("name") == "DISABLED"
            and prop.get("value") in (True, 1, "1", "ON", "TRUE")
            for prop in properties
        ):
            errors.append(f"{prefix}: CTest test `{name}` is disabled")
        if command is None:
            # CTest omits the command for an unbuilt target but still reports
            # its name and add_test() backtrace.
            registered_tests[name] = None
            continue
        if (
            not isinstance(command, list)
            or not command
            or not isinstance(command[0], str)
        ):
            errors.append(f"{prefix}: CTest test `{name}` has an invalid command")
            continue
        command_path = normalized_absolute_path(command[0], build_dir)
        registered_tests[name] = command
        registered_commands.add(command_path)

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

    unregistered = []
    for name, artifacts in target_artifacts.items():
        named_command = registered_tests.get(name, "")
        named_registration = name in registered_tests and (
            named_command is None
            or (
                len(named_command) == 1
                and normalized_absolute_path(named_command[0], build_dir) in artifacts
            )
        )
        command_registration = not artifacts.isdisjoint(registered_commands)
        if not named_registration and not command_registration:
            unregistered.append(name)
    unregistered.sort()
    if unregistered:
        errors.append(
            f"{prefix}: {len(unregistered)} configured C++ test targets are not "
            f"registered with CTest, including {unregistered[:5]}"
        )
    configured_artifacts = {
        artifact for artifacts in target_artifacts.values() for artifact in artifacts
    }
    unmapped_commands = registered_commands - configured_artifacts
    if unmapped_commands:
        errors.append(
            f"{prefix}: {len(unmapped_commands)} CTest commands do not map to "
            "configured C++ test targets"
        )


def check_cmake_test_target_trace(
    root: Path,
    build_dir: Path,
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
        suffix = tests_arguments[4:]
        valid_suffix = (bool(suffix) and suffix[0] == "DEPENDS") or (
            len(suffix) >= 3 and suffix[:3] == ["-C", "$<CONFIG>", "DEPENDS"]
        )
        if (
            tests_arguments[:2] != ["tests_and_run", "COMMAND"]
            or len(tests_arguments) < 4
            or tests_arguments[3] != "--output-on-failure"
            or tests_arguments.count("COMMAND") != 1
            or not executable_paths_match(tests_arguments[2], trusted_ctest)
            or not valid_suffix
        ):
            errors.append(
                f"{prefix}: expanded `tests_and_run` command does not invoke "
                "the configured CTest executable"
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
                and len(pytest_arguments) > pytest_index + 3
                and executable_paths_match(
                    pytest_arguments[pytest_index + 1], cmake_executable
                )
                and pytest_arguments[pytest_index + 2 : pytest_index + 4]
                == ["-E", "env"]
                and len(pytest_arguments) > pytest_index + 8
            )
        if valid:
            pythonpath = pytest_arguments[pytest_index + 4]
            traced_python = pytest_arguments[pytest_index + 5]
            pythonpath_value = pythonpath.partition("=")[2]
            pythonpath_entries = pythonpath_value.split(os.pathsep)
            normalized_pythonpath = {
                normalized_absolute_path(entry, root / "python" / "tests")
                for entry in pythonpath_entries
                if entry
            }
            allowed_pythonpath = {
                normalized_absolute_path(str(build_dir / "python" / "dartpy"), root),
                normalized_absolute_path(str(build_dir / "python"), root),
            }
            required_pythonpath = normalized_absolute_path(
                str(build_dir / "python" / "dartpy"), root
            )
            valid = (
                pythonpath.startswith("PYTHONPATH=")
                and all(pythonpath_entries)
                and len(pythonpath_entries) == len(normalized_pythonpath)
                and required_pythonpath in normalized_pythonpath
                and normalized_pythonpath <= allowed_pythonpath
                and executable_paths_match(traced_python, sys.executable)
                and pytest_arguments[pytest_index + 6 : pytest_index + 8]
                == ["-m", "pytest"]
            )
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
            command_tail = pytest_arguments[pytest_index + 8 : working_index]
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
    traced_sources = (
        root / "tests" / "CMakeLists.txt",
        root / "python" / "tests" / "CMakeLists.txt",
    )
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
    for name in ("ALL", "tests_and_run", "pytest"):
        entries = named_targets.get(name, [])
        if len(entries) != 1:
            errors.append(
                f"{prefix}: expected exactly one configured `{name}` target, "
                f"found {len(entries)}"
            )
            continue
        required_entries[name] = entries[0]
    if len(required_entries) != 3:
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
    if len(target_data) != 3:
        return

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
    ownership_commands = {
        "add_executable",
        "add_library",
        "dart_add_test",
        "dart_build_tests",
        "target_sources",
    }
    unowned_sources = []
    library_only_sources = []
    for relative, path in expected_test_sources.items():
        if relative in executable_test_sources:
            continue
        if relative in configured_test_sources:
            # This C translation unit is an intentional compile/link ABI check.
            if relative == "tests/unit/math/test_LegacyConvexHullC.c":
                continue
            library_only_sources.append(relative)
            continue
        owner = path.parent / "CMakeLists.txt"
        try:
            owner_commands = cmake_commands(owner.read_text(encoding="utf-8"))
        except OSError:
            owner_commands = []
        if any(
            name in ownership_commands
            and (path.name in arguments or path.stem in arguments)
            for name, arguments in owner_commands
        ):
            continue
        unowned_sources.append(relative)
    if library_only_sources:
        errors.append(
            f"{prefix}: {len(library_only_sources)} C++ runtime test sources "
            f"belong only to non-executable targets, including "
            f"{library_only_sources[:5]}"
        )
    if unowned_sources:
        errors.append(
            f"{prefix}: {len(unowned_sources)} C++ test sources are neither "
            f"configured nor conditionally owned, including {unowned_sources[:5]}"
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
            cache,
            cmake_trace_records(result.stderr),
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
            "scripts/check_ai_infrastructure.py --check",
            "tests/test_sync_ai_commands.py",
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
                "Max gives one hard task more reasoning time",
                "Ultra is for independently",
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
