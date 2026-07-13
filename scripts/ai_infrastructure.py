#!/usr/bin/env python3
"""Shared, read-only checks for DART's repository AI infrastructure."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tomllib
from pathlib import Path
from typing import Iterable

PROFILE_READ_ERRORS = (OSError, json.JSONDecodeError, AttributeError)
PATH_RESOLUTION_ERRORS = (OSError, ValueError)
JSON_READ_ERRORS = (OSError, json.JSONDecodeError)

PROFILES = ("main", "release-6.20")
MAX_AGENTS_BYTES = 32 * 1024
EXCLUDED_DIRS = {".git", ".pixi", "build", "external", "node_modules"}
AGENT_NAMES = ("dart_scout", "dart_reviewer", "dart_release_auditor")
SCENARIO_IDS = (
    "orientation",
    "small-change",
    "failure-diagnosis",
    "documentation-update",
    "component-work",
    "simulation-verification",
    "release-maintenance",
)
SCENARIO_MANIFEST = "docs/ai/agent-scenarios.json"
BRANCH_PROFILE = "docs/ai/branch-profile.json"
SCENARIO_REQUIRED_KEYS = {
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
PROFILE_FINGERPRINTS = {
    "main": {
        "base_ref": "origin/main",
        "cpp_standard": "C++23",
        "python_binding": "nanobind",
        "io_namespace": "dart::io",
        "gui_backend": "Filament/GLFW3/Dear ImGui",
    },
    "release-6.20": {
        "base_ref": "origin/release-6.20",
        "cpp_standard": "C++17",
        "python_binding": "pybind11",
        "io_namespace": "dart::utils",
        "gui_backend": "OSG",
    },
}
AI_PREFIXES = (
    "AGENTS.md",
    ".gitignore",
    ".agents/",
    ".claude/",
    ".codex/",
    ".opencode/",
    ".github/workflows/ci_lint.yml",
    ".github/workflows/ci_ubuntu.yml",
    ".github/workflows/ci_windows.yml",
    "docs/README.md",
    "docs/ai/",
    "docs/design/ai_spec_kit_assessment.md",
    "docs/onboarding/ai-tools.md",
    "docs/onboarding/contributing.md",
    "docs/onboarding/release-management.md",
    "scripts/ai_",
    "scripts/check_ai_",
    "scripts/check_agent_hook.py",
    "scripts/exercise_agent_scenarios.py",
    "scripts/install_git_hooks.py",
    "scripts/pretool_guard_bridge.py",
    "scripts/setup_ai.py",
    "scripts/sync_ai_commands.py",
    "python/tests/unit/test_agent_capture.py",
    "python/tests/unit/gui/test_offscreen_render.py",
    "tests/test_ai_",
    "tests/test_check_agent_hook.py",
    "tests/test_install_git_hooks.py",
    "tests/test_sync_ai_commands.py",
    "pixi.toml",
)

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
CODEX_HOOK_STATUS = "Checking DART commit gate"
CLAUDE_HOOK_COMMAND = '"${CLAUDE_PROJECT_DIR}/.claude/hooks/pre-commit-guard.sh"'
WINDOWS_LAUNCHER_MARKERS = (
    "scripts/pretool_guard_bridge.py",
    "--root",
    ".pixi/envs/default/python.exe",
    "Get-Command py",
    "Get-Command python",
    "pixi run python scripts/setup_ai.py",
    "$payload = [Console]::In.ReadToEnd()",
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

PIXIRUN_RE = re.compile(
    r"pixi\s+run\s+"
    r"(?:(?:(?:--locked|--frozen)\s+)|"
    r"(?:(?:-e|--environment)\s+[A-Za-z0-9_.-]+\s+))*"
    r"(?P<task>[A-Za-z0-9][A-Za-z0-9_.-]*)"
)
PATH_RE = re.compile(
    r"(?<![A-Za-z0-9_~/])"
    r"(?:AGENTS\.md|CONTRIBUTING\.md|CHANGELOG\.md|"
    r"(?:docs|scripts|tests|dart|python|\.agents|\.claude|\.codex|\.github)"
    r"/[A-Za-z0-9_./*{}<>$?-]+)"
)
INLINE_CODE_RE = re.compile(r"`([^`\n]+)`")
PLACEHOLDER_PATH_NAMES = {"relevant-doc.md", "x.md"}


def repository_root(start: Path | None = None) -> Path:
    """Return the repository root without changing repository state."""
    if start is None:
        start = Path(__file__).resolve().parents[1]
    result = subprocess.run(
        ["git", "-C", str(start), "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
    )
    if result.returncode == 0:
        return Path(result.stdout.strip()).resolve()
    return start.resolve()


def _git(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", "-C", str(root), *args], capture_output=True, text=True
    )


def detect_profile(root: Path, requested: str = "auto") -> str:
    if requested != "auto":
        if requested not in PROFILES:
            raise ValueError(f"unknown branch profile: {requested}")
        return requested
    profile_path = root / BRANCH_PROFILE
    try:
        declared = json.loads(profile_path.read_text()).get("profile")
    except PROFILE_READ_ERRORS:
        declared = None
    if declared in PROFILES:
        return declared
    branch = _git(root, "branch", "--show-current").stdout.strip()
    if "release-6.20" in branch:
        return "release-6.20"
    release_ref = "origin/release-6.20"
    if _git(root, "rev-parse", "--verify", "--quiet", release_ref).returncode == 0:
        if (
            _git(root, "merge-base", "--is-ancestor", release_ref, "HEAD").returncode
            == 0
        ):
            return "release-6.20"
    return "main"


def _load_toml(path: Path, errors: list[str]) -> dict:
    try:
        with path.open("rb") as stream:
            return tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as exc:
        errors.append(f"{path}: invalid or unreadable TOML: {exc}")
        return {}


def _load_json(path: Path, errors: list[str]) -> dict:
    try:
        data = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        errors.append(f"{path}: invalid or unreadable JSON: {exc}")
        return {}
    if not isinstance(data, dict):
        errors.append(f"{path}: JSON root must be an object")
        return {}
    return data


def check_codex_config(root: Path) -> list[str]:
    errors: list[str] = []
    path = root / ".codex" / "config.toml"
    data = _load_toml(path, errors)
    if set(data) != {"agents"}:
        errors.append(".codex/config.toml: root keys must equal agents")
    agents = data.get("agents", {})
    if not isinstance(agents, dict):
        errors.append(".codex/config.toml: agents must be a table")
        return errors
    if set(agents) != {"max_threads", "max_depth"}:
        errors.append(
            ".codex/config.toml: agents keys must equal max_depth, max_threads"
        )
    if type(agents.get("max_threads")) is not int or agents.get("max_threads") != 4:
        errors.append(".codex/config.toml: agents.max_threads must equal 4")
    if type(agents.get("max_depth")) is not int or agents.get("max_depth") != 1:
        errors.append(".codex/config.toml: agents.max_depth must equal 1")
    forbidden = {"model", "approval_policy", "sandbox_mode"} & set(data)
    if forbidden:
        errors.append(
            ".codex/config.toml: project config must not pin "
            + ", ".join(sorted(forbidden))
        )
    return errors


def check_custom_agents(root: Path) -> list[str]:
    errors: list[str] = []
    agents_dir = root / ".codex" / "agents"
    actual = {path.stem for path in agents_dir.glob("*.toml")}
    missing = set(AGENT_NAMES) - actual
    extra = actual - set(AGENT_NAMES)
    if missing:
        errors.append(f".codex/agents: missing agents: {', '.join(sorted(missing))}")
    if extra:
        errors.append(f".codex/agents: unexpected agents: {', '.join(sorted(extra))}")
    for name in AGENT_NAMES:
        path = agents_dir / f"{name}.toml"
        if not path.is_file():
            continue
        data = _load_toml(path, errors)
        if set(data) != {
            "name",
            "description",
            "sandbox_mode",
            "developer_instructions",
        }:
            errors.append(
                f"{path}: keys must equal description, developer_instructions, "
                "name, sandbox_mode"
            )
        if data.get("name") != name:
            errors.append(f"{path}: name must equal {name!r}")
        description = data.get("description", "")
        if not isinstance(description, str) or not description.strip():
            errors.append(f"{path}: description must be non-empty")
        elif len(description) > 200:
            errors.append(f"{path}: description exceeds 200 characters")
        instructions = data.get("developer_instructions", "")
        if not isinstance(instructions, str) or not instructions.strip():
            errors.append(f"{path}: developer_instructions must be non-empty")
        else:
            lowered = instructions.lower()
            for required in ("read-only", "input", "output", "evidence"):
                if required not in lowered:
                    errors.append(f"{path}: instructions must name {required!r}")
        if data.get("sandbox_mode") != "read-only":
            errors.append(f"{path}: sandbox_mode must equal 'read-only'")
        if "model" in data:
            errors.append(f"{path}: model must inherit from the parent session")
    return errors


def check_codex_hooks(root: Path) -> list[str]:
    errors: list[str] = []
    path = root / ".codex" / "hooks.json"
    data = _load_json(path, errors)
    if set(data) != {"hooks"}:
        errors.append(".codex/hooks.json: root keys must equal hooks")
    hooks = data.get("hooks", {})
    if not isinstance(hooks, dict) or set(hooks) != {"PreToolUse"}:
        errors.append(".codex/hooks.json: only a PreToolUse hook is allowed")
        return errors
    entries = hooks.get("PreToolUse", [])
    if not isinstance(entries, list) or len(entries) != 1:
        errors.append(".codex/hooks.json: PreToolUse must contain one matcher")
        return errors
    entry = entries[0]
    if not isinstance(entry, dict):
        errors.append(".codex/hooks.json: PreToolUse matcher must be an object")
        return errors
    if set(entry) != {"matcher", "hooks"}:
        errors.append(".codex/hooks.json: matcher keys must equal hooks, matcher")
    if entry.get("matcher") != "^Bash$":
        errors.append(".codex/hooks.json: matcher must be '^Bash$'")
    handlers = entry.get("hooks", [])
    if not isinstance(handlers, list) or len(handlers) != 1:
        errors.append(".codex/hooks.json: matcher must contain one handler")
        return errors
    handler = handlers[0]
    if not isinstance(handler, dict):
        errors.append(".codex/hooks.json: handler must be an object")
        return errors
    if set(handler) != {
        "type",
        "command",
        "commandWindows",
        "timeout",
        "statusMessage",
    }:
        errors.append(
            ".codex/hooks.json: handler keys must equal command, commandWindows, "
            "statusMessage, timeout, type"
        )
    if handler.get("type") != "command":
        errors.append(".codex/hooks.json: handler type must be 'command'")
    if handler.get("command") != CODEX_HOOK_COMMAND:
        errors.append(
            ".codex/hooks.json: handler command must equal the canonical "
            "read-only guard invocation"
        )
    if handler.get("commandWindows") != CODEX_HOOK_COMMAND_WINDOWS:
        errors.append(
            ".codex/hooks.json: handler commandWindows must equal the canonical "
            "Windows staged-guard invocation"
        )
    if handler.get("timeout") != 10:
        errors.append(".codex/hooks.json: timeout must equal 10 seconds")
    if handler.get("statusMessage") != CODEX_HOOK_STATUS:
        errors.append(
            ".codex/hooks.json: statusMessage must equal " f"{CODEX_HOOK_STATUS!r}"
        )
    launcher = root / ".claude" / "hooks" / "pre-commit-guard.ps1"
    try:
        launcher_text = launcher.read_text()
    except OSError:
        errors.append(".claude/hooks/pre-commit-guard.ps1: missing Windows launcher")
    else:
        for marker in WINDOWS_LAUNCHER_MARKERS:
            if marker not in launcher_text:
                errors.append(
                    ".claude/hooks/pre-commit-guard.ps1: missing required marker "
                    f"{marker!r}"
                )
    bridge = root / "scripts" / "pretool_guard_bridge.py"
    try:
        bridge_text = bridge.read_text()
    except OSError:
        errors.append("scripts/pretool_guard_bridge.py: missing Windows bridge")
    else:
        for marker in WINDOWS_BRIDGE_MARKERS:
            if marker not in bridge_text:
                errors.append(
                    "scripts/pretool_guard_bridge.py: missing required marker "
                    f"{marker!r}"
                )
    return errors


def check_claude_hooks(root: Path) -> list[str]:
    errors: list[str] = []
    path = root / ".claude" / "settings.json"
    data = _load_json(path, errors)
    hooks = data.get("hooks", {})
    entries = hooks.get("PreToolUse", []) if isinstance(hooks, dict) else []
    entry = entries[0] if isinstance(entries, list) and len(entries) == 1 else None
    handlers = entry.get("hooks", []) if isinstance(entry, dict) else []
    handler = handlers[0] if isinstance(handlers, list) and len(handlers) == 1 else None
    if not isinstance(entry, dict) or entry.get("matcher") != "Bash":
        errors.append(".claude/settings.json: expected one Bash PreToolUse matcher")
    if not isinstance(handler, dict):
        errors.append(".claude/settings.json: expected one command hook")
        return errors
    if set(handler) != {"type", "command", "shell"}:
        errors.append(
            ".claude/settings.json: handler keys must equal command, shell, type"
        )
    if handler.get("type") != "command":
        errors.append(".claude/settings.json: handler type must be 'command'")
    if handler.get("command") != CLAUDE_HOOK_COMMAND:
        errors.append(
            ".claude/settings.json: command must equal the project-root guard "
            "invocation"
        )
    if handler.get("shell") != "bash":
        errors.append(".claude/settings.json: shell must equal 'bash'")
    return errors


def _iter_agent_files(root: Path) -> Iterable[Path]:
    tracked = _git(
        root,
        "ls-files",
        "--cached",
        "--others",
        "--exclude-standard",
        "--",
        "*AGENTS.md",
    )
    if tracked.returncode == 0:
        for relative in tracked.stdout.splitlines():
            path = root / relative
            if path.is_file():
                yield path
        return
    for path in root.rglob("AGENTS.md"):
        relative = path.relative_to(root)
        if not any(part in EXCLUDED_DIRS for part in relative.parts):
            yield path


def _agent_chain(root: Path, leaf: Path) -> list[Path]:
    chain = [root / "AGENTS.md"]
    relative_parent = leaf.parent.relative_to(root)
    current = root
    for part in relative_parent.parts:
        current /= part
        candidate = current / "AGENTS.md"
        if candidate.is_file() and candidate != chain[0]:
            chain.append(candidate)
    return chain


def agents_chain_for_directory(root: Path, start_dir: str) -> list[str]:
    target = (root / start_dir).resolve()
    try:
        relative = target.relative_to(root.resolve())
    except ValueError:
        return []
    if target.is_file():
        relative = relative.parent
    chain = [root / "AGENTS.md"]
    current = root
    for part in relative.parts:
        current /= part
        candidate = current / "AGENTS.md"
        if candidate.is_file() and candidate != chain[0]:
            chain.append(candidate)
    return [str(path.relative_to(root)) for path in chain if path.is_file()]


def check_agents_chains(root: Path) -> list[str]:
    errors: list[str] = []
    root_agents = root / "AGENTS.md"
    if not root_agents.is_file():
        return ["AGENTS.md: missing repository-root instructions"]
    if root_agents.is_symlink():
        return ["AGENTS.md: repository-root instructions must not be a symlink"]
    files = sorted(_iter_agent_files(root))
    for leaf in files:
        if leaf.is_symlink():
            errors.append(f"{leaf.relative_to(root)}: AGENTS.md must not be a symlink")
            continue
        chain = _agent_chain(root, leaf)
        size = sum(path.stat().st_size for path in chain)
        if size > MAX_AGENTS_BYTES:
            names = " -> ".join(str(path.relative_to(root)) for path in chain)
            errors.append(f"AGENTS chain exceeds {MAX_AGENTS_BYTES} bytes: {names}")
        for path in chain:
            try:
                path.read_text(encoding="utf-8")
            except UnicodeError:
                errors.append(f"{path.relative_to(root)}: AGENTS.md is not UTF-8")
    return errors


def instruction_files(root: Path) -> list[str]:
    return sorted(str(path.relative_to(root)) for path in _iter_agent_files(root))


def _is_repo_relative(root: Path, value: str) -> bool:
    candidate = Path(value)
    if candidate.is_absolute() or not value:
        return False
    try:
        (root / candidate).resolve().relative_to(root.resolve())
    except PATH_RESOLUTION_ERRORS:
        return False
    return True


def check_branch_profile(root: Path, profile: str) -> list[str]:
    errors: list[str] = []
    data = _load_json(root / BRANCH_PROFILE, errors)
    expected_keys = {
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
    if set(data) != expected_keys:
        errors.append(
            f"{BRANCH_PROFILE}: keys must equal {', '.join(sorted(expected_keys))}"
        )
    if type(data.get("schema_version")) is not int or data.get("schema_version") != 1:
        errors.append(f"{BRANCH_PROFILE}: schema_version must equal 1")
    if data.get("profile") != profile:
        errors.append(f"{BRANCH_PROFILE}: profile must equal {profile!r}")
    for field, expected in PROFILE_FINGERPRINTS[profile].items():
        if data.get(field) != expected:
            errors.append(f"{BRANCH_PROFILE}: {field} must equal {expected!r}")
    try:
        agents_text = (root / "AGENTS.md").read_text(errors="replace")
    except OSError:
        agents_text = ""
        errors.append("AGENTS.md: unable to read branch-profile markers")
    list_fields = (
        "required_markers",
        "required_paths",
        "forbidden_markers",
        "forbidden_paths",
        "downstream_gates",
    )
    for field in list_fields:
        values = data.get(field)
        if (
            not isinstance(values, list)
            or not values
            or not all(isinstance(value, str) and value for value in values)
        ):
            errors.append(f"{BRANCH_PROFILE}: {field} must be a non-empty string list")
    required_markers = data.get("required_markers")
    forbidden_markers = data.get("forbidden_markers")
    required_paths = data.get("required_paths")
    forbidden_paths = data.get("forbidden_paths")
    downstream_gates = data.get("downstream_gates")
    for marker in required_markers if isinstance(required_markers, list) else []:
        if isinstance(marker, str) and marker not in agents_text:
            errors.append(f"AGENTS.md: {profile} profile is missing marker {marker!r}")
    for marker in forbidden_markers if isinstance(forbidden_markers, list) else []:
        if isinstance(marker, str) and marker in agents_text:
            errors.append(
                f"AGENTS.md: {profile} profile contains forbidden marker {marker!r}"
            )
    for path in required_paths if isinstance(required_paths, list) else []:
        if not isinstance(path, str):
            continue
        if not _is_repo_relative(root, path):
            errors.append(
                f"{profile} profile: required path escapes repository: {path}"
            )
        elif not (root / path).exists():
            errors.append(f"{profile} profile: missing branch-owned path {path}")
    for path in forbidden_paths if isinstance(forbidden_paths, list) else []:
        if not isinstance(path, str):
            continue
        if not _is_repo_relative(root, path):
            errors.append(
                f"{profile} profile: forbidden path escapes repository: {path}"
            )
        elif (root / path).exists():
            errors.append(
                f"{profile} profile: forbidden branch path is present: {path}"
            )
    pixi_errors: list[str] = []
    tasks = _collect_task_names(_load_toml(root / "pixi.toml", pixi_errors))
    errors.extend(pixi_errors)
    for gate in downstream_gates if isinstance(downstream_gates, list) else []:
        _validate_gate(gate, tasks, "downstream_gates", errors)
    return errors


def _source_files(root: Path) -> list[Path]:
    candidates = [root / path for path in instruction_files(root)]
    candidates.extend((root / "docs" / "ai").glob("*.md"))
    candidates.append(root / "docs" / "onboarding" / "ai-tools.md")
    candidates.extend((root / ".claude" / "commands").glob("*.md"))
    candidates.extend((root / ".claude" / "skills").glob("*/SKILL.md"))
    return sorted({path for path in candidates if path.is_file()})


def _collect_task_names(data: object) -> set[str]:
    names: set[str] = set()
    if isinstance(data, dict):
        tasks = data.get("tasks")
        if isinstance(tasks, dict):
            names.update(str(name) for name in tasks)
        for value in data.values():
            names.update(_collect_task_names(value))
    elif isinstance(data, list):
        for value in data:
            names.update(_collect_task_names(value))
    return names


def pixi_task_names(root: Path) -> list[str]:
    errors: list[str] = []
    tasks = _collect_task_names(_load_toml(root / "pixi.toml", errors))
    return sorted(tasks) if not errors else []


def _profile_lines(text: str, profile: str) -> Iterable[str]:
    import sync_ai_commands as sync

    return (line for _, line in sync.profile_skill_lines(text, profile))


def check_pixi_references(root: Path, profile: str | None = None) -> list[str]:
    errors: list[str] = []
    pixi = _load_toml(root / "pixi.toml", errors)
    tasks = _collect_task_names(pixi)
    lint_tasks = (
        "lint-yaml",
        "lint-md",
        "check-lint-yaml",
        "check-lint-md",
    )
    if set(lint_tasks).issubset(tasks):
        task_table = pixi.get("tasks", {})
        if isinstance(task_table, dict):
            for task_name in lint_tasks:
                task = task_table.get(task_name, {})
                command = task.get("cmd", "") if isinstance(task, dict) else task
                if isinstance(command, list):
                    command = " ".join(str(part) for part in command)
                if not isinstance(command, str) or (
                    "!**/.claude/worktrees/**" not in command
                ):
                    errors.append(
                        f"pixi.toml: task {task_name!r} must exclude "
                        ".claude/worktrees/**"
                    )
    for path in _source_files(root):
        text = path.read_text(errors="replace")
        for line in _profile_lines(text, profile or "main"):
            for match in PIXIRUN_RE.finditer(line):
                if match.end() < len(line) and line[match.end()] in "*<{":
                    continue
                task = match.group("task")
                if task not in tasks:
                    rel = path.relative_to(root)
                    errors.append(f"{rel}: references missing Pixi task {task!r}")
    return sorted(set(errors))


def check_path_references(root: Path, profile: str | None = None) -> list[str]:
    errors: list[str] = []
    for path in _source_files(root):
        text = path.read_text(errors="replace")
        # Check repository paths presented as inline code. Prose fragments,
        # fenced examples, API URLs, and placeholders are deliberately outside
        # this lightweight check; the docs-policy checker owns Markdown links.
        selected_text = "\n".join(_profile_lines(text, profile or "main"))
        for code_match in INLINE_CODE_RE.finditer(selected_text):
            code = code_match.group(1)
            if "://" in code or "\n" in code:
                continue
            for match in PATH_RE.finditer(code):
                reference = match.group(0).rstrip(".,:;")
                if any(char in reference for char in "*{}<>$?"):
                    continue
                if Path(reference).name.casefold() in PLACEHOLDER_PATH_NAMES:
                    continue
                if not _is_repo_relative(root, reference):
                    rel = path.relative_to(root)
                    errors.append(
                        f"{rel}: referenced path escapes repository {reference!r}"
                    )
                elif not (root / reference).exists():
                    rel = path.relative_to(root)
                    errors.append(f"{rel}: references missing path {reference!r}")
    return sorted(set(errors))


def check_generated_adapters(root: Path) -> list[str]:
    """Check generated Codex/OpenCode adapters and their ownership manifest."""
    errors: list[str] = []
    skills_root = root / ".agents" / "skills"
    try:
        skills_root.resolve().relative_to(root.resolve())
    except PATH_RESOLUTION_ERRORS:
        return [".agents/skills: generated root escapes repository"]
    if skills_root.is_symlink():
        return [".agents/skills: generated root must not be a symlink"]
    if skills_root.exists() and not skills_root.is_dir():
        return [".agents/skills: generated root must be a directory"]
    manifest_path = skills_root / ".dart-generated.json"
    if manifest_path.is_symlink():
        return [".agents/skills/.dart-generated.json: manifest must not be a symlink"]
    manifest = _load_json(manifest_path, errors)
    if (
        type(manifest.get("schema_version")) is not int
        or manifest["schema_version"] != 1
    ):
        errors.append(
            ".agents/skills/.dart-generated.json: schema_version must equal 1"
        )
    if set(manifest) != {"schema_version", "generator", "paths"}:
        errors.append(
            ".agents/skills/.dart-generated.json: keys must equal generator, "
            "paths, schema_version"
        )
    if manifest.get("generator") != "scripts/sync_ai_commands.py":
        errors.append(
            ".agents/skills/.dart-generated.json: generator must name "
            "scripts/sync_ai_commands.py"
        )
    declared = manifest.get("paths")
    if not isinstance(declared, list):
        errors.append(
            ".agents/skills/.dart-generated.json: paths must be a string list"
        )
        declared_names: set[str] = set()
    else:
        parsed = [
            match.group(1)
            for path in declared
            if isinstance(path, str)
            and (match := re.fullmatch(r"([a-z0-9][a-z0-9-]*)/SKILL\.md", path))
        ]
        if len(parsed) != len(declared):
            errors.append(
                ".agents/skills/.dart-generated.json: paths must contain safe "
                "lowercase `<skill>/SKILL.md` entries"
            )
        declared_names = set(parsed)
        expected_paths = [f"{name}/SKILL.md" for name in sorted(declared_names)]
        if declared != expected_paths:
            errors.append(
                ".agents/skills/.dart-generated.json: paths must be sorted and unique"
            )
    commands = sorted((root / ".claude" / "commands").glob("*.md"))
    skills = sorted((root / ".claude" / "skills").glob("*/SKILL.md"))
    command_names = {path.stem for path in commands}
    skill_names = {path.parent.name for path in skills}
    collisions = sorted(command_names & skill_names)
    if collisions:
        errors.append(
            "editable command/skill name collisions: " + ", ".join(collisions)
        )
    expected_names = command_names | skill_names
    if declared_names != expected_names:
        missing = sorted(expected_names - declared_names)
        extra = sorted(declared_names - expected_names)
        if missing:
            errors.append(
                ".agents/skills/.dart-generated.json: missing managed skills: "
                + ", ".join(missing)
            )
        if extra:
            errors.append(
                ".agents/skills/.dart-generated.json: stale managed skills: "
                + ", ".join(extra)
            )
    previous_bytecode_policy = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        import sync_ai_commands as sync
    except (ImportError, SyntaxError) as exc:
        return [*errors, f"scripts/sync_ai_commands.py: import failed: {exc}"]
    finally:
        sys.dont_write_bytecode = previous_bytecode_policy
    for generated in sorted((root / ".agents" / "skills").glob("*/SKILL.md")):
        if generated.parent.is_symlink() or generated.is_symlink():
            if generated.parent.name in expected_names:
                errors.append(
                    f"{generated.relative_to(root)}: generated skill path must "
                    "not be a symlink"
                )
            continue
        if (
            generated.parent.name not in expected_names
            and sync.is_generated_skill_adapter(generated.read_text())
        ):
            errors.append(
                f"{generated.relative_to(root)}: orphaned generated skill is not "
                "owned by the manifest"
            )
    for source in commands:
        relative = str(source.relative_to(root))
        opencode = root / ".opencode" / "command" / source.name
        if not opencode.is_file():
            errors.append(f"{opencode.relative_to(root)}: missing generated command")
        elif opencode.is_symlink():
            errors.append(
                f"{opencode.relative_to(root)}: generated command is a symlink"
            )
        else:
            opencode_content = opencode.read_text()
            if not sync.has_auto_gen_header(opencode_content) or (
                sync.strip_auto_gen_header(opencode_content) != source.read_text()
            ):
                errors.append(
                    f"{opencode.relative_to(root)}: generated command is stale"
                )
        codex = root / ".agents" / "skills" / source.stem / "SKILL.md"
        expected_codex = sync.add_auto_gen_header(
            sync.render_codex_command_skill(source), relative
        )
        if not codex.is_file():
            errors.append(f"{codex.relative_to(root)}: missing generated skill")
        elif codex.is_symlink() or codex.parent.is_symlink():
            errors.append(
                f"{codex.relative_to(root)}: generated skill path is a symlink"
            )
        elif codex.read_text() != expected_codex:
            errors.append(f"{codex.relative_to(root)}: generated skill is stale")
    for source in skills:
        relative = str(source.relative_to(root))
        codex = root / ".agents" / "skills" / source.parent.name / "SKILL.md"
        expected_codex = sync.add_auto_gen_header(source.read_text(), relative)
        if not codex.is_file():
            errors.append(f"{codex.relative_to(root)}: missing generated skill")
        elif codex.is_symlink() or codex.parent.is_symlink():
            errors.append(
                f"{codex.relative_to(root)}: generated skill path is a symlink"
            )
        elif codex.read_text() != expected_codex:
            errors.append(f"{codex.relative_to(root)}: generated skill is stale")
    expected_commands = {source.name for source in commands}
    actual_commands = {
        path.name for path in (root / ".opencode" / "command").glob("*.md")
    }
    for orphan in sorted(actual_commands - expected_commands):
        errors.append(f".opencode/command/{orphan}: orphaned generated command")
    legacy_root = root / ".codex" / "skills"
    if legacy_root.is_symlink():
        errors.append(".codex/skills: legacy skill root must not be a symlink")
    elif legacy_root.exists() and not legacy_root.is_dir():
        errors.append(".codex/skills: legacy skill root must be a directory")
    elif legacy_root.is_dir():
        for child in sorted(legacy_root.iterdir()):
            if child.is_symlink():
                errors.append(
                    f"{(child / 'SKILL.md').relative_to(root)}: legacy skill "
                    "path must not be a symlink"
                )
    legacy_paths = [] if legacy_root.is_symlink() else legacy_root.glob("*/SKILL.md")
    for legacy in sorted(legacy_paths):
        if legacy.is_symlink() or legacy.parent.is_symlink():
            errors.append(
                f"{legacy.relative_to(root)}: legacy skill path must not be a symlink"
            )
            continue
        legacy_name = legacy.parent.name
        if legacy_name in expected_names:
            errors.append(
                f"{legacy.relative_to(root)}: legacy path collides with canonical "
                ".agents/skills adapter"
            )
        elif "AUTO-GENERATED FILE" in legacy.read_text(errors="replace"):
            errors.append(
                f"{legacy.relative_to(root)}: legacy generated adapter must be removed"
            )
    return errors


def check_ci_wiring(root: Path) -> list[str]:
    path = root / ".github" / "workflows" / "ci_lint.yml"
    try:
        content = path.read_text()
    except OSError as exc:
        return [f".github/workflows/ci_lint.yml: unable to read: {exc}"]
    errors: list[str] = []
    for command in ("pixi run check-lint", "pixi run test-ai-infra"):
        if command not in content:
            errors.append(
                f".github/workflows/ci_lint.yml: missing AI CI command {command!r}"
            )
    windows_path = root / ".github" / "workflows" / "ci_windows.yml"
    try:
        windows_content = windows_path.read_text()
    except OSError as exc:
        errors.append(f".github/workflows/ci_windows.yml: unable to read: {exc}")
    else:
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
                    f"marker {marker!r}"
                )
    linux_path = root / ".github" / "workflows" / "ci_ubuntu.yml"
    capture_test_path = root / "python/tests/unit/test_agent_capture.py"
    for visual_path, markers in (
        (
            capture_test_path,
            (
                "test_run_capture_smoke_writes_stills_and_sidecar",
                'layers=["contacts", "labels"]',
                "test_run_capture_debug_layers_change_pixels_end_to_end",
                'debug_layers = ["contacts", "collision_bounds", "labels"]',
                "for layer in debug_layers",
                "assert _changed_pixel_count(plain_image, combined_image) >= 128",
                "assert _changed_pixel_count(plain_image, debug_image) >= 32",
                "DART_REQUIRE_VISUAL_E2E",
                "dartpy.gui.OffscreenRenderer is unavailable",
                'pytest.fail("visual e2e is required but DISPLAY is unavailable")',
            ),
        ),
    ):
        try:
            visual_content = visual_path.read_text()
        except OSError as exc:
            errors.append(f"{visual_path.relative_to(root)}: unable to read: {exc}")
            continue
        for marker in markers:
            if marker not in visual_content:
                errors.append(
                    f"{visual_path.relative_to(root)}: missing visual CI marker "
                    f"{marker!r}"
                )
    try:
        linux_content = linux_path.read_text()
    except OSError:
        linux_content = ""
    visual_step_name = "- name: Agent visual verification smoke"
    if visual_step_name not in linux_content:
        errors.append(
            ".github/workflows/ci_ubuntu.yml: missing visual CI marker "
            "'Agent visual verification smoke'"
        )
    visual_section = linux_content.partition(visual_step_name)[2].partition(
        "\n  filament-gui-smoke:"
    )[0]
    if re.search(r"(?m)^\s*if\s*:", visual_section):
        errors.append(
            ".github/workflows/ci_ubuntu.yml: visual smoke must not use an "
            "if condition"
        )
    if "continue-on-error" in visual_section:
        errors.append(
            ".github/workflows/ci_ubuntu.yml: visual smoke must not use "
            "continue-on-error"
        )
    if visual_section.count("xvfb-run") != 2:
        errors.append(
            ".github/workflows/ci_ubuntu.yml: missing visual CI marker "
            "'xvfb-run' exactly twice"
        )
    for marker in (
        "pixi run agent-capture",
        "--scene box_on_ground --steps 250 --focus box --auto-views 1",
        "--layers contacts collision_bounds labels",
        "--width 320 --height 240",
        "--out /tmp/dart-agent-visual-smoke --prefix smoke",
        "pixi run image-verdict",
        "/tmp/dart-agent-visual-smoke/smoke_auto0.png",
        "xvfb-run -a -s '-screen 0 640x480x24' \\\n"
        "            env DART_REQUIRE_VISUAL_E2E=1 "
        "pixi run test-agent-visual-e2e",
    ):
        if marker not in visual_section:
            errors.append(
                ".github/workflows/ci_ubuntu.yml: missing visual CI marker "
                f"{marker!r}"
            )
    overlay_test_path = root / "python/tests/unit/gui/test_offscreen_render.py"
    try:
        overlay_test_text = overlay_test_path.read_text()
    except OSError as exc:
        errors.append(f"{overlay_test_path.relative_to(root)}: unable to read: {exc}")
    else:
        for marker in (
            "test_debug_scene_overlay_lifecycle_on_shared_renderer",
            'layers=("grid", "body_frames", "contacts")',
            "_bytes_over_tolerance(debugged, plain) >= 256",
            "_bytes_over_tolerance(emptied, plain) <= 64",
        ):
            if marker not in overlay_test_text:
                errors.append(
                    f"{overlay_test_path.relative_to(root)}: missing visual CI "
                    f"marker {marker!r}"
                )
    pixi_path = root / "pixi.toml"
    try:
        pixi_text = pixi_path.read_text()
    except OSError as exc:
        errors.append(f"pixi.toml: unable to read: {exc}")
    else:
        for marker in (
            "test-agent-visual-e2e =",
            "test_run_capture_debug_layers_change_pixels_end_to_end",
        ):
            if marker not in pixi_text:
                errors.append(f"pixi.toml: missing visual CI marker {marker!r}")
    return errors


def scenario_path(root: Path, kind: str, route: str) -> Path:
    if kind in {"workflow", "domain_skill"}:
        return root / ".agents" / "skills" / route / "SKILL.md"
    if kind == "agent":
        return root / ".codex" / "agents" / f"{route}.toml"
    raise ValueError(f"unknown route kind: {kind}")


def _scenario_contracts(root: Path, profile: str, errors: list[str]) -> list[dict]:
    manifest = _load_json(root / SCENARIO_MANIFEST, errors)
    if set(manifest) != {"schema_version", "profile", "scenarios"}:
        errors.append(
            f"{SCENARIO_MANIFEST}: keys must equal profile, scenarios, schema_version"
        )
    if (
        type(manifest.get("schema_version")) is not int
        or manifest.get("schema_version") != 1
    ):
        errors.append(f"{SCENARIO_MANIFEST}: schema_version must equal 1")
    if manifest.get("profile") != profile:
        errors.append(f"{SCENARIO_MANIFEST}: profile must equal {profile!r}")
    contracts = manifest.get("scenarios")
    if not isinstance(contracts, list):
        errors.append(f"{SCENARIO_MANIFEST}: scenarios must be a list")
        return []
    for index, contract in enumerate(contracts):
        if not isinstance(contract, dict):
            errors.append(f"{SCENARIO_MANIFEST}: scenarios[{index}] must be an object")
    return [contract for contract in contracts if isinstance(contract, dict)]


def _validate_gate(
    gate: object, tasks: set[str], field: str, case_errors: list[str]
) -> None:
    if not isinstance(gate, str):
        case_errors.append(f"{field} entries must be strings")
        return
    match = PIXIRUN_RE.fullmatch(gate.strip())
    if not match:
        case_errors.append(
            f"{field} entry must be one exact 'pixi run' command: {gate!r}"
        )
        return
    task = match.group("task")
    if task not in tasks:
        case_errors.append(f"{field} references missing Pixi task {task!r}")


def validate_scenarios(
    root: Path, profile: str
) -> tuple[list[dict[str, object]], list[str]]:
    errors: list[str] = []
    contracts = _scenario_contracts(root, profile, errors)
    ids = [contract.get("id") for contract in contracts]
    if ids != list(SCENARIO_IDS):
        errors.append(
            f"{SCENARIO_MANIFEST}: {profile} scenario ids/order must equal "
            + ", ".join(SCENARIO_IDS)
        )
    pixi_errors: list[str] = []
    tasks = _collect_task_names(_load_toml(root / "pixi.toml", pixi_errors))
    errors.extend(pixi_errors)
    results: list[dict[str, object]] = []
    for index, contract in enumerate(contracts):
        identifier = contract.get("id")
        label = str(identifier) if identifier else f"index-{index}"
        case_errors: list[str] = []
        missing_keys = SCENARIO_REQUIRED_KEYS - set(contract)
        extra_keys = set(contract) - SCENARIO_REQUIRED_KEYS - SCENARIO_OPTIONAL_KEYS
        if missing_keys:
            case_errors.append(
                "missing required fields: " + ", ".join(sorted(missing_keys))
            )
        if extra_keys:
            case_errors.append("unknown fields: " + ", ".join(sorted(extra_keys)))
        prompt = contract.get("prompt_class")
        if not isinstance(prompt, str) or not prompt.strip():
            case_errors.append("prompt_class must be non-empty")
        start_dir = contract.get("start_dir")
        actual_chain: list[str] = []
        if not isinstance(start_dir, str) or not start_dir:
            case_errors.append("start_dir must be a non-empty relative path")
        else:
            start_path = (root / start_dir).resolve()
            try:
                start_path.relative_to(root.resolve())
            except ValueError:
                case_errors.append("start_dir must stay inside the repository")
            else:
                if not start_path.is_dir():
                    case_errors.append(f"start_dir does not exist: {start_dir}")
                else:
                    actual_chain = agents_chain_for_directory(root, start_dir)
        expected_chain = contract.get("instruction_chain")
        if not isinstance(expected_chain, list) or not all(
            isinstance(item, str) for item in expected_chain
        ):
            case_errors.append("instruction_chain must be a string list")
        elif actual_chain != expected_chain:
            case_errors.append(
                "AGENTS chain mismatch: expected "
                f"{expected_chain!r}, found {actual_chain!r}"
            )
        owner_paths = contract.get("owner_docs")
        if not isinstance(owner_paths, list) or not owner_paths:
            case_errors.append("owner_docs must be a non-empty list")
        else:
            for owner in owner_paths:
                if not isinstance(owner, str) or not _is_repo_relative(root, owner):
                    case_errors.append(f"owner path escapes repository: {owner!r}")
                elif not (root / owner).is_file():
                    case_errors.append(f"owner path is missing: {owner!r}")
        workflow = contract.get("expected_route")
        workflow_path = ""
        if not isinstance(workflow, dict):
            case_errors.append("expected_route must be an object")
        else:
            kind = workflow.get("kind")
            name = workflow.get("name")
            if set(workflow) != {"kind", "name", "path"}:
                case_errors.append("expected_route keys must equal kind, name, path")
            if (
                not isinstance(kind, str)
                or kind not in {"workflow", "domain_skill", "agent"}
                or not isinstance(name, str)
            ):
                case_errors.append("workflow kind/name is invalid")
            elif kind in {"workflow", "domain_skill"} and not re.fullmatch(
                r"dart-[a-z0-9-]+", name
            ):
                case_errors.append("workflow name must be a bounded dart-* identifier")
            elif kind == "agent" and name not in AGENT_NAMES:
                case_errors.append(f"unknown custom agent route: {name!r}")
            else:
                route_path = scenario_path(root, kind, name)
                workflow_path = str(route_path.relative_to(root))
                if workflow.get("path") != workflow_path:
                    case_errors.append(
                        f"expected_route path must equal {workflow_path!r}"
                    )
                if not route_path.is_file():
                    case_errors.append(f"workflow route is missing: {workflow_path}")
                if (
                    kind == "workflow"
                    and not (root / ".claude" / "commands" / f"{name}.md").is_file()
                ):
                    case_errors.append(f"workflow source is missing: {name!r}")
                if (
                    kind == "domain_skill"
                    and not (root / ".claude" / "skills" / name / "SKILL.md").is_file()
                ):
                    case_errors.append(f"domain-skill source is missing: {name!r}")
        specialist = contract.get("specialist_agent")
        if specialist is not None:
            if specialist not in AGENT_NAMES:
                case_errors.append(f"unknown specialist_agent: {specialist!r}")
            elif not (root / ".codex" / "agents" / f"{specialist}.toml").is_file():
                case_errors.append(f"specialist agent is missing: {specialist}")
        scope = contract.get("permitted_scopes")
        if (
            not isinstance(scope, list)
            or not scope
            or not all(isinstance(item, str) and item.strip() for item in scope)
        ):
            case_errors.append("permitted_scopes must be a non-empty string list")
        elif any(
            item.strip() in {"/", ".", "**", "whole repository"} for item in scope
        ):
            case_errors.append("permitted_scopes contains an unbounded entry")
        elif any(
            Path(item).is_absolute() or ".." in Path(item).parts for item in scope
        ):
            case_errors.append("permitted_scopes contains a path escape")
        for gate_field in ("focused_gates", "full_gates"):
            gates = contract.get(gate_field)
            if not isinstance(gates, list) or not gates:
                case_errors.append(f"{gate_field} must be a non-empty list")
                continue
            for gate in gates:
                _validate_gate(gate, tasks, gate_field, case_errors)
        if identifier == "simulation-verification":
            expected_owner = (
                "docs/onboarding/agent-sim-verification.md"
                if profile == "main"
                else "docs/ai/verification.md"
            )
            expected_focused = (
                {"pixi run test-unit"} if profile == "main" else {"pixi run test"}
            )
            expected_evidence_scopes = {
                "dart/simulation",
                "dart/dynamics",
                "dart/collision",
                "dart/gui",
                "matching tests",
                "temporary claim-tied evidence",
            }
            if profile == "main":
                expected_evidence_scopes.update(
                    {
                        "dart/constraint",
                        "dart/io",
                        "python",
                        "examples",
                        "tutorials",
                    }
                )
                expected_prompt = (
                    "verify claim-dependent DART 7 simulation, dynamics, "
                    "collision/contact/constraints, model/scene, or GUI behavior"
                )
                if contract.get("prompt_class") != expected_prompt:
                    case_errors.append(
                        "simulation verification has the wrong claim-dependent prompt"
                    )
            if not isinstance(workflow, dict) or (
                workflow.get("kind"),
                workflow.get("name"),
            ) != ("domain_skill", "dart-verify-sim"):
                case_errors.append(
                    "simulation verification must route to dart-verify-sim"
                )
            if not isinstance(owner_paths, list) or expected_owner not in owner_paths:
                case_errors.append(
                    "simulation verification is missing its visual evidence owner"
                )
            focused = contract.get("focused_gates")
            if not isinstance(focused, list) or set(focused) != expected_focused:
                case_errors.append(
                    "simulation verification is missing its focused correctness gate"
                )
            if profile == "main" and contract.get("full_gates") != [
                "pixi run test-py",
                "pixi run test-all",
            ]:
                case_errors.append(
                    "simulation verification is missing its full correctness gates"
                )
            if not isinstance(scope, list) or not expected_evidence_scopes.issubset(
                set(scope)
            ):
                case_errors.append(
                    "simulation verification must cover simulation, dynamics, "
                    "collision, constraints, GUI, model loading, Python, "
                    "examples, tests, and temporary evidence"
                )
            if contract.get("evidence_policy") != (
                "text-first-with-claim-tied-visual-or-documented-exception"
            ):
                case_errors.append(
                    "simulation verification has the wrong evidence policy"
                )
            skill_source = root / ".claude/skills/dart-verify-sim/SKILL.md"
            try:
                skill_text = skill_source.read_text()
            except OSError:
                skill_text = ""
            skill_markers = [
                "text oracle",
                "agent-capture",
                "image-verdict",
                "sole correctness oracle",
            ]
            if profile == "main":
                skill_markers.extend(
                    [
                        "model/scene loading",
                        "collision/contact/constraints",
                        "stepping, GUI/rendering",
                        "GUI/rendering",
                        "visual examples",
                        "rendering is unavailable",
                        "replacement evidence",
                    ]
                )
            for marker in skill_markers:
                if marker not in skill_text:
                    case_errors.append(
                        f"dart-verify-sim source is missing contract marker {marker!r}"
                    )
            new_task_source = root / ".claude/commands/dart-new-task.md"
            try:
                new_task_text = new_task_source.read_text()
            except OSError:
                new_task_text = ""
            new_task_markers = ["route through `dart-verify-sim`"]
            if profile == "main":
                new_task_markers.append("record why it is not applicable")
            for marker in new_task_markers:
                if marker not in new_task_text:
                    case_errors.append(
                        "dart-new-task is missing the conditional simulation "
                        f"marker {marker!r}"
                    )
            consumer_markers = {
                "AGENTS.md": (
                    "docs/onboarding/agent-sim-verification.md",
                    "dart-verify-sim",
                ),
                ".codex/agents/dart_scout.toml": (
                    "collision/contact/constraints",
                    "text correctness oracle",
                    "claim-tied assessed visual",
                ),
                ".codex/agents/dart_reviewer.toml": (
                    "text-first evidence",
                    "visual/debug-layer",
                ),
                ".claude/commands/dart-new-task.md": (
                    "route through `dart-verify-sim`",
                    "record why it is not applicable",
                ),
                ".claude/commands/dart-ultrawork.md": (
                    "routes through `dart-verify-sim`",
                ),
                ".claude/commands/dart-resume.md": ("route through `dart-verify-sim`",),
                ".claude/commands/dart-pr.md": ("use `dart-verify-sim`",),
                ".claude/commands/dart-manage-pr.md": ("Visual verification",),
                ".claude/commands/dart-review-pr.md": (
                    "require the `dart-verify-sim` text oracle",
                    "accepting a screenshot alone",
                ),
                ".claude/skills/dart-build/SKILL.md": ("dart-verify-sim",),
                ".claude/skills/dart-test/SKILL.md": (
                    "load `dart-verify-sim`",
                    "unavailable or not applicable",
                ),
                ".claude/skills/dart-io/SKILL.md": (
                    "also load `dart-verify-sim`",
                    "claim-tied visual corroboration",
                ),
                ".claude/skills/dart-python/SKILL.md": (
                    "also load `dart-verify-sim`",
                    "collision/contact/constraints",
                    "GUI/rendering output",
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
                    "target branch's `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-fix-ci.md": (
                    "use `dart-verify-sim`",
                    "visual exception",
                ),
                ".claude/commands/dart-fix-issue.md": (
                    "route through `dart-verify-sim`",
                    "collision/contact/constraints",
                    "visual exception",
                ),
                "docs/ai/verification.md": (
                    "agent-capture",
                    "text correctness",
                    "If rendering is unavailable",
                    "name replacement evidence",
                ),
            }
            for relative, markers in consumer_markers.items():
                try:
                    consumer_text = (root / relative).read_text()
                except OSError:
                    consumer_text = ""
                for marker in markers:
                    if marker not in consumer_text:
                        case_errors.append(
                            f"{relative}: missing simulation route marker {marker!r}"
                        )
        recovery = contract.get("recovery")
        if not isinstance(recovery, str) or not recovery:
            case_errors.append("recovery must be a non-empty path")
        elif not _is_repo_relative(root, recovery.partition("#")[0]):
            case_errors.append(f"recovery pointer escapes repository: {recovery}")
        elif not (root / recovery.partition("#")[0]).is_file():
            case_errors.append(f"recovery pointer is missing: {recovery}")
        forbidden = contract.get("forbidden_paths")
        if not isinstance(forbidden, list) or not forbidden:
            case_errors.append("forbidden_paths must be a non-empty list")
        else:
            for forbidden_path in forbidden:
                if not isinstance(forbidden_path, str):
                    case_errors.append("forbidden_paths entries must be strings")
                elif not _is_repo_relative(root, forbidden_path):
                    case_errors.append(
                        f"forbidden path escapes repository: {forbidden_path}"
                    )
                elif (root / forbidden_path).exists():
                    case_errors.append(
                        f"forbidden branch path is present: {forbidden_path}"
                    )
        result = dict(contract)
        result.update(
            {
                "actual_agents_chain": actual_chain,
                "workflow_path": workflow_path,
                "valid": not case_errors,
                "errors": case_errors,
            }
        )
        results.append(result)
        errors.extend(f"scenario {label}: {error}" for error in case_errors)
    return results, errors


def scenario_results(root: Path, profile: str) -> list[dict[str, object]]:
    return validate_scenarios(root, profile)[0]


def check_scenarios(root: Path, profile: str) -> list[str]:
    return validate_scenarios(root, profile)[1]


def run_checks(root: Path, profile: str) -> list[str]:
    checks = (
        check_codex_config,
        check_custom_agents,
        check_codex_hooks,
        check_claude_hooks,
        check_agents_chains,
        lambda path: check_branch_profile(path, profile),
        lambda path: check_pixi_references(path, profile),
        lambda path: check_path_references(path, profile),
        check_generated_adapters,
        check_ci_wiring,
        lambda path: check_scenarios(path, profile),
    )
    errors: list[str] = []
    for check in checks:
        errors.extend(check(root))
    return sorted(set(errors))


def is_ai_infrastructure_path(path: str) -> bool:
    return path.endswith("/AGENTS.md") or any(
        path == prefix or path.startswith(prefix) for prefix in AI_PREFIXES
    )


def is_related_worktree_ai_path(root: Path, path: str) -> bool:
    """Exclude unrelated personal/plugin skills from worktree consistency gates."""
    if not path.startswith(".agents/skills/"):
        return is_ai_infrastructure_path(path)
    parts = Path(path).parts
    if len(parts) < 3 or parts[2] == ".dart-generated.json":
        return True
    skill_name = parts[2]
    if (root / ".claude" / "commands" / f"{skill_name}.md").is_file() or (
        root / ".claude" / "skills" / skill_name / "SKILL.md"
    ).is_file():
        return True
    manifest_path = root / ".agents" / "skills" / ".dart-generated.json"
    try:
        manifest = json.loads(manifest_path.read_text())
    except JSON_READ_ERRORS:
        manifest = {}
    owned_paths = manifest.get("paths", []) if isinstance(manifest, dict) else []
    if isinstance(owned_paths, list) and f"{skill_name}/SKILL.md" in owned_paths:
        return True
    skill_path = root / ".agents" / "skills" / skill_name / "SKILL.md"
    try:
        content = skill_path.read_text()
    except OSError:
        return False
    return "<!-- AUTO-GENERATED FILE" in content and (
        "<!-- Sync script: scripts/sync_ai_commands.py -->" in content
    )


def tool_versions() -> dict[str, str]:
    versions: dict[str, str] = {}
    for tool, args in (
        ("git", ("--version",)),
        ("pixi", ("--version",)),
        ("codex", ("--version",)),
    ):
        executable = shutil.which(tool)
        if not executable:
            versions[tool] = "not found"
            continue
        result = subprocess.run(
            [executable, *args], capture_output=True, text=True, timeout=10
        )
        versions[tool] = (result.stdout or result.stderr).strip().splitlines()[0]
    versions["python"] = sys.version.split()[0]
    return versions


def working_tree_state(root: Path) -> str:
    result = _git(root, "status", "--short")
    if result.returncode != 0:
        return "unavailable"
    count = len(result.stdout.splitlines())
    return "clean" if count == 0 else f"dirty ({count} paths)"


def format_errors(errors: Iterable[str]) -> str:
    return "\n".join(f"ERROR: {error}" for error in errors)


__all__ = [
    "AGENT_NAMES",
    "BRANCH_PROFILE",
    "MAX_AGENTS_BYTES",
    "PIXIRUN_RE",
    "PROFILE_FINGERPRINTS",
    "PROFILES",
    "SCENARIO_IDS",
    "SCENARIO_MANIFEST",
    "agents_chain_for_directory",
    "check_agents_chains",
    "check_branch_profile",
    "check_claude_hooks",
    "check_codex_config",
    "check_codex_hooks",
    "check_custom_agents",
    "check_path_references",
    "check_pixi_references",
    "check_scenarios",
    "detect_profile",
    "format_errors",
    "check_generated_adapters",
    "check_ci_wiring",
    "instruction_files",
    "is_ai_infrastructure_path",
    "is_related_worktree_ai_path",
    "repository_root",
    "run_checks",
    "pixi_task_names",
    "scenario_results",
    "tool_versions",
    "validate_scenarios",
    "working_tree_state",
]
