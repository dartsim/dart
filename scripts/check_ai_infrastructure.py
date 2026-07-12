#!/usr/bin/env python3
"""Validate and diagnose DART 6.20's repository-local AI infrastructure."""

from __future__ import annotations

import argparse
import json
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
DIRECT_PIXI_COMMANDS = {"bash", "cmake", "ctest", "python", "python3", "sh"}
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
SCENARIO_OPTIONAL_KEYS = {"evidence_policy"}
ROUTE_KEYS = {"kind", "name", "path"}
CODEX_HOOK_COMMAND = (
    'repo_root="$(git rev-parse --show-toplevel)" && '
    'CLAUDE_PROJECT_DIR="$repo_root" CODEX_PROJECT_DIR="$repo_root" '
    '"$repo_root/.claude/hooks/pre-commit-guard.sh"'
)
CODEX_HOOK_COMMAND_WINDOWS = (
    "powershell -NoProfile -NonInteractive -ExecutionPolicy Bypass -Command "
    '"& (Join-Path (git rev-parse --show-toplevel) '
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
    '$ErrorActionPreference = "Continue"',
    "$ErrorActionPreference = $previousErrorActionPreference",
    "$LASTEXITCODE = $null",
    "$nativeExitCode = $LASTEXITCODE",
    "$null -eq $nativeExitCode",
    "[Console]::Error.WriteLine",
    "exit 2",
)
WINDOWS_BRIDGE_MARKERS = (
    "def validate_payload(payload: bytes)",
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
        owner.read_text(encoding="utf-8")
        for owner in marker_owners
        if owner.is_file()
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
            agents = config.get("agents", {})
            if not isinstance(agents, dict):
                errors.append(".codex/config.toml: agents must be a table")
                agents = {}
            if (
                type(agents.get("max_threads")) is not int
                or agents.get("max_threads") != 4
            ):
                errors.append(".codex/config.toml: agents.max_threads must be 4")
            if type(agents.get("max_depth")) is not int or agents.get("max_depth") != 1:
                errors.append(".codex/config.toml: agents.max_depth must be 1")
            if "model" in config:
                errors.append(".codex/config.toml: project config must not pin a model")

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
        for field in ("name", "description", "developer_instructions"):
            if not isinstance(profile.get(field), str) or not profile[field].strip():
                errors.append(f"{path.relative_to(root)}: missing `{field}`")
        if profile.get("name") != name:
            errors.append(f"{path.relative_to(root)}: name must be `{name}`")
        if profile.get("sandbox_mode") != "read-only":
            errors.append(f"{path.relative_to(root)}: sandbox_mode must be read-only")
        if "model" in profile:
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
    launcher_text = (
        launcher.read_text(encoding="utf-8") if launcher.exists() else ""
    )
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
        r"pixi run(?:\s+-e\s+[A-Za-z0-9_-]+)?(?:\s+--skip-deps)?" r"\s+([A-Za-z0-9_-]+)"
    )
    for path in source_paths(root):
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
    python_skill = (
        root / ".claude" / "skills" / "dart-python" / "SKILL.md"
    ).read_text(encoding="utf-8")
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

    release_fix = (
        root / ".claude" / "commands" / "dart-release-ci-fix.md"
    ).read_text(encoding="utf-8")
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
        if visual_section.count("xvfb-run") != 2:
            errors.append(
                ".github/workflows/ci_ubuntu.yml: missing visual smoke marker "
                "`xvfb-run` exactly twice"
            )
        for marker in (
            "pixi run agent-capture",
            "--scene box_on_ground --steps 250 --focus box --auto-views 1",
            "--layers contacts collision_bounds labels",
            "--width 320 --height 240",
            "--out /tmp/dart-agent-visual-smoke --prefix smoke",
            "pixi run image-verdict",
            "/tmp/dart-agent-visual-smoke/smoke_auto0.png",
            "pixi run test-agent-debug-overlay",
        ):
            if marker not in visual_section:
                errors.append(
                    ".github/workflows/ci_ubuntu.yml: missing visual smoke "
                    f"marker `{marker}`"
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
        '$ErrorActionPreference = "Continue"',
        '"git status"',
        "DART_HOOK_DRY_RUN",
        '"git commit --no-verify -m x"',
        "$diagnosticExit -ne 0",
        "$successExit -ne 0",
        "$blockedExit -ne 2",
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
            ):
                if marker not in skill_text:
                    local_errors.append(
                        f"dart-verify-sim source is missing contract marker {marker!r}"
                    )
            try:
                new_task_text = (
                    root / ".claude/commands/dart-new-task.md"
                ).read_text(encoding="utf-8")
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
                ".claude/commands/dart-resume.md": (
                    "route through `dart-verify-sim`",
                ),
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
                    "required renderer is unavailable",
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


def print_doctor(root: Path) -> None:
    branch = subprocess.run(
        ["git", "branch", "--show-current"],
        cwd=root,
        capture_output=True,
        text=True,
    ).stdout.strip()
    print(f"Repository: {root}")
    print(f"Branch: {branch or '(detached)'}")
    print("Profile: DART 6.20 compatibility (C++17, pybind11, dart::utils, OSG)")
    print(f"Python: {sys.version.split()[0]}")
    print(f"Pixi: {version('pixi')}")
    print(f"Codex: {version('codex')}")
    print("Trust: project agents and hooks load only after the repository is trusted")
    print("Hook inspection: use `/hooks` in Codex; git hook: `pixi run install-hooks`")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true", help="run structural checks")
    modes.add_argument("--doctor", action="store_true", help="print diagnostics")
    modes.add_argument(
        "--scenarios", action="store_true", help="exercise deterministic scenarios"
    )
    parser.add_argument("--repo-root", type=Path, help=argparse.SUPPRESS)
    args = parser.parse_args()
    root = (args.repo_root or Path(__file__).resolve().parents[1]).resolve()

    if args.scenarios:
        errors = exercise_scenarios(root)
    else:
        if args.doctor:
            print_doctor(root)
        errors = run_checks(root)

    if errors:
        for error in errors:
            print(f"ERROR {error}", file=sys.stderr)
        return 1
    print("AI infrastructure checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
