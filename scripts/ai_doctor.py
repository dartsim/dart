#!/usr/bin/env python3
"""Report DART AI-infrastructure health using read-only probes."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import tomllib
from pathlib import Path

from ai_infrastructure import (
    BRANCH_PROFILE,
    PROFILES,
    agents_chain_for_directory,
    detect_profile,
    instruction_files,
    pixi_task_names,
    repository_root,
    run_checks,
    scenario_results,
    tool_versions,
    working_tree_state,
)
from install_git_hooks import HOOK_TEMPLATE, SENTINEL

AI_TASKS = {
    "ai-doctor",
    "ai-setup",
    "check-agent-hook",
    "check-ai-commands",
    "check-ai-infra",
    "exercise-agent-scenarios",
    "install-hooks",
    "sync-ai-commands",
    "test-ai-infra",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("auto", *PROFILES), default="auto")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def _read_json(path: Path) -> dict:
    try:
        data = json.loads(path.read_text())
    except OSError, json.JSONDecodeError:
        return {}
    return data if isinstance(data, dict) else {}


def _codex_trust(root: Path) -> dict[str, str]:
    codex_home = Path(os.environ.get("CODEX_HOME", Path.home() / ".codex"))
    config_path = codex_home / "config.toml"
    try:
        with config_path.open("rb") as stream:
            config = tomllib.load(stream)
    except OSError, tomllib.TOMLDecodeError:
        return {
            "user_config": str(config_path),
            "project": "not-observed",
            "project_hook": "not-observed",
        }
    projects = config.get("projects") or {}
    project = (
        projects.get(str(root.resolve()), {}) if isinstance(projects, dict) else {}
    )
    trust_level = (
        project.get("trust_level", "not-observed")
        if isinstance(project, dict)
        else "not-observed"
    )
    hooks = config.get("hooks") or {}
    hook_state = hooks.get("state") or {} if isinstance(hooks, dict) else {}
    project_hook = "not-observed"
    project_hook_path = str((root / ".codex" / "hooks.json").resolve())
    if isinstance(hook_state, dict):
        hashed_sources = [
            str(source)
            for source, state in hook_state.items()
            if isinstance(state, dict)
            and isinstance(state.get("trusted_hash"), str)
            and state["trusted_hash"].strip()
        ]
        if any(project_hook_path in source for source in hashed_sources):
            project_hook = "trusted-hash-observed"
        elif any(".codex/hooks.json" in source for source in hashed_sources):
            project_hook = "ambiguous-hook-hash-observed"
    return {
        "user_config": str(config_path),
        "project": str(trust_level),
        "project_hook": project_hook,
    }


def _base_relationship(root: Path, base_ref: object) -> str:
    if not isinstance(base_ref, str):
        return "unavailable"
    exists = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "--verify", "--quiet", base_ref],
        capture_output=True,
        text=True,
    )
    if exists.returncode != 0:
        return "base-ref-unavailable"
    ancestor = subprocess.run(
        ["git", "-C", str(root), "merge-base", "--is-ancestor", base_ref, "HEAD"],
        capture_output=True,
        text=True,
    )
    return "base-is-ancestor" if ancestor.returncode == 0 else "base-not-ancestor"


def _path_inventory(paths: list[Path], root: Path) -> dict[str, object]:
    relative = sorted(str(path.relative_to(root)) for path in paths)
    return {"count": len(relative), "paths": relative}


def _git_hook_inventory(root: Path) -> dict[str, object]:
    custom = subprocess.run(
        ["git", "-C", str(root), "config", "--get", "core.hooksPath"],
        capture_output=True,
        text=True,
    )
    configured_path = custom.stdout.strip() if custom.returncode == 0 else ""
    resolved = subprocess.run(
        ["git", "-C", str(root), "rev-parse", "--git-path", "hooks/pre-commit"],
        capture_output=True,
        text=True,
    )
    if resolved.returncode != 0:
        return {
            "path": "unavailable",
            "core_hooks_path": configured_path,
            "exists": False,
            "executable": False,
            "managed": False,
            "current": False,
        }
    hook_path = Path(resolved.stdout.strip())
    if not hook_path.is_absolute():
        hook_path = (root / hook_path).resolve()
    try:
        content = hook_path.read_text(errors="replace")
    except OSError:
        content = ""
    exists = hook_path.is_file()
    executable = exists and os.access(hook_path, os.X_OK)
    managed = SENTINEL in content
    current = managed and executable and content == HOOK_TEMPLATE
    return {
        "path": str(hook_path),
        "core_hooks_path": configured_path,
        "exists": exists,
        "executable": executable,
        "managed": managed,
        "current": current,
    }


def _inventory(root: Path, profile: str) -> dict[str, object]:
    profile_data = _read_json(root / BRANCH_PROFILE)
    scenarios = scenario_results(root, profile)
    scenario_chains = [
        {
            "id": result.get("id"),
            "start_dir": result.get("start_dir"),
            "paths": agents_chain_for_directory(
                root, str(result.get("start_dir", "."))
            ),
        }
        for result in scenarios
    ]
    commands = sorted((root / ".claude" / "commands").glob("*.md"))
    skills = sorted((root / ".claude" / "skills").glob("*/SKILL.md"))
    generated = sorted((root / ".agents" / "skills").glob("*/SKILL.md"))
    agents = sorted((root / ".codex" / "agents").glob("*.toml"))
    tasks = pixi_task_names(root)
    hooks = _read_json(root / ".codex" / "hooks.json").get("hooks") or {}
    return {
        "effective_profile": profile_data,
        "base_relationship": _base_relationship(root, profile_data.get("base_ref")),
        "instructions": {
            "count": len(instruction_files(root)),
            "paths": instruction_files(root),
            "scenario_chains": scenario_chains,
        },
        "source_commands": _path_inventory(commands, root),
        "source_skills": _path_inventory(skills, root),
        "generated_skills": {
            **_path_inventory(generated, root),
            "manifest": ".agents/skills/.dart-generated.json",
        },
        "custom_agents": _path_inventory(agents, root),
        "hooks": {
            "path": ".codex/hooks.json",
            "events": sorted(hooks) if isinstance(hooks, dict) else [],
            "shared_guard": ".claude/hooks/pre-commit-guard.sh",
            "windows_launcher": ".claude/hooks/pre-commit-guard.ps1",
            "windows_bridge": "scripts/pretool_guard_bridge.py",
        },
        "git_hook": _git_hook_inventory(root),
        "setup": {
            "path": "scripts/setup_ai.py",
            "steps": [
                "scripts/sync_ai_commands.py",
                "scripts/install_git_hooks.py",
            ],
        },
        "tasks": {
            "manifest": "pixi.toml",
            "count": len(tasks),
            "ai_tasks": sorted(AI_TASKS & set(tasks)),
        },
    }


def _recovery(
    errors: list[str], warnings: list[str], trust: dict[str, str]
) -> list[dict[str, str]]:
    suggestions: list[dict[str, str]] = []

    def add(reason: str, command: str) -> None:
        item = {"reason": reason, "command": command}
        if not any(existing["command"] == command for existing in suggestions):
            suggestions.append(item)

    joined = "\n".join(errors)
    if any(token in joined for token in ("generated", ".dart-generated.json")):
        add(
            "generated adapters or ownership metadata are stale",
            "pixi run sync-ai-commands",
        )
    if any(
        token in joined
        for token in (
            ".codex/hooks.json",
            ".claude/hooks/pre-commit-guard.sh",
            ".claude/hooks/pre-commit-guard.ps1",
            "scripts/pretool_guard_bridge.py",
            "commit guard",
        )
    ):
        add(
            "inspect and restore the tracked hook sources before rechecking",
            "git diff -- .codex/hooks.json .claude/hooks/pre-commit-guard.sh "
            ".claude/hooks/pre-commit-guard.ps1 scripts/pretool_guard_bridge.py",
        )
    if "AGENTS chain" in joined or "instruction discovery" in joined:
        add("instruction discovery or context budget failed", "pixi run check-ai-infra")
    if "Pixi task" in joined:
        add("a documented Pixi task alias is missing", "pixi task list")
    if any(token in joined for token in ("scenario", "branch-profile", "missing path")):
        add("tracked AI contract validation failed", "pixi run check-ai-infra")
    if errors:
        add(
            "rerun focused infrastructure tests after repairs", "pixi run test-ai-infra"
        )
    custom_hooks = any(
        "core.hooksPath is configured" in warning for warning in warnings
    )
    if custom_hooks:
        add(
            "wire the DART gate into the configured hook manager",
            "python3 scripts/check_agent_hook.py --profile staged",
        )
    elif any("managed Git pre-commit hook" in warning for warning in warnings):
        add("the cross-tool Git hook is missing or stale", "pixi run install-hooks")
    if trust.get("project") != "trusted":
        add("Codex project trust is not observed", "review project trust in Codex")
    if trust.get("project_hook") != "trusted-hash-observed":
        add("Codex project hook trust is not observed", "/hooks")
    return suggestions


def report(root: Path, requested_profile: str) -> dict:
    root = root.resolve()
    profile = detect_profile(root, requested_profile)
    errors = run_checks(root, profile)
    tools = tool_versions()
    trust = _codex_trust(root)
    inventory = _inventory(root, profile)
    warnings = [
        f"{tool} is not available"
        for tool, version in tools.items()
        if version == "not found"
    ]
    if trust["project"] != "trusted":
        warnings.append("Codex project trust is not observed as trusted")
    if trust["project_hook"] != "trusted-hash-observed":
        warnings.append(
            "Codex project hook trust is not observed; use /hooks after review"
        )
    git_hook = inventory["git_hook"]
    if git_hook["core_hooks_path"]:
        warnings.append(
            "core.hooksPath is configured; the managed Git pre-commit hook "
            "cannot be installed automatically"
        )
    elif not git_hook["current"]:
        warnings.append("managed Git pre-commit hook is missing, stale, or disabled")
    return {
        "schema_version": 1,
        "root": str(root),
        "profile": profile,
        "working_tree": working_tree_state(root),
        "tools": tools,
        "trust": trust,
        "inventory": inventory,
        "errors": errors,
        "warnings": warnings,
        "recovery": _recovery(errors, warnings, trust),
        "ok": not errors,
    }


def main() -> int:
    args = parse_args()
    data = report(repository_root(args.root), args.profile)
    if args.json:
        print(json.dumps(data, indent=2, sort_keys=True))
    else:
        state = "PASS" if data["ok"] else "FAIL"
        print(f"DART AI doctor: {state}")
        print(f"  root: {data['root']}")
        print(f"  profile: {data['profile']}")
        print(f"  working tree: {data['working_tree']}")
        inventory = data["inventory"]
        print(
            "  inventory: "
            f"{inventory['instructions']['count']} instructions, "
            f"{inventory['source_commands']['count']} commands, "
            f"{inventory['source_skills']['count']} source skills, "
            f"{inventory['generated_skills']['count']} generated skills, "
            f"{inventory['custom_agents']['count']} custom agents"
        )
        for tool, version in data["tools"].items():
            print(f"  {tool}: {version}")
        for warning in data["warnings"]:
            print(f"  WARN: {warning}")
        for error in data["errors"]:
            print(f"  ERROR: {error}")
        for item in data["recovery"]:
            print(f"  RECOVER: {item['command']} ({item['reason']})")
    return 0 if data["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
