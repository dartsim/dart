#!/usr/bin/env python3
"""Sync AI commands and skills across tool directories.

Different AI coding tools read from different directories:
- Claude Code: .claude/commands/, .claude/skills/
- OpenCode: .opencode/command/
- Codex: .codex/skills/ (domain skills plus command-derived workflow skills)

This script keeps them in sync using .claude/ as the current editable source
for workflow commands and domain skills.
It also verifies that every supported agent has the same effective DART
capability set, even when tools expose workflows through different UI concepts
(commands vs skills).

Usage:
    python scripts/sync_ai_commands.py          # Sync and report
    python scripts/sync_ai_commands.py --check  # Check only (CI mode)
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import sys
from pathlib import Path

CODEX_NAME_LIMIT = 100
CODEX_DESC_LIMIT = 500
MAX_SKILL_LINES = 500
MAX_COMMAND_LINES = 200
CAPABILITY_SCHEMA_VERSION = 1
CAPABILITY_STATUS_VALUES = {"active", "deprecated", "parked", "proposed"}
CAPABILITY_WORKFLOW_GATE_PROFILE_VALUES = {
    "approval-boundary",
    "code-gate",
    "docs-ai",
    "draft-only",
    "maintainer-only",
    "pr-ready",
    "read-only",
    "release-gate",
    "reproduction",
    "routed",
    "state-check",
    "task-specific",
}
CAPABILITY_DOMAIN_GATE_PROFILE_VALUES = {"command", "reference"}
CAPABILITY_WORKFLOW_GATE_PROFILE_MARKERS = {
    "approval-boundary": ("explicit approval",),
    "code-gate": ("`pixi run lint`", "focused build/test", "regression test"),
    "docs-ai": ("docs/AI checks", "docs/ai/verification.md", "principle audit"),
    "draft-only": ("draft text",),
    "maintainer-only": ("Maintainer-only",),
    "pr-ready": ("CHANGELOG", "milestone"),
    "read-only": ("Read-only", "read-only"),
    "release-gate": ("release verification", "release-target", "release gates"),
    "reproduction": ("local reproduction", "reproduction command"),
    "routed": ("task-specific gates",),
    "state-check": ("`git status --short --branch`",),
    "task-specific": ("task-specific gates",),
}

# Header added to auto-generated files
AUTO_GEN_HEADER = """\
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: {source_path} -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

"""


def get_repo_root() -> Path:
    return Path(__file__).parent.parent


def display_path(path: Path) -> str:
    """Return a repo-relative path when possible, otherwise a readable path."""
    try:
        return str(path.relative_to(get_repo_root()))
    except ValueError:
        return str(path)


def parse_skill_frontmatter(content: str) -> dict[str, str]:
    """Extract name and description from YAML frontmatter."""
    return parse_frontmatter(content, ["name", "description"])


def parse_command_frontmatter(content: str) -> dict[str, str]:
    """Extract supported metadata from command YAML frontmatter."""
    return parse_frontmatter(content, ["description", "argument-hint"])


def parse_frontmatter(content: str, keys: list[str]) -> dict[str, str]:
    """Extract selected single-line fields from YAML frontmatter."""
    match = re.match(r"^---\s*\n(.*?)\n---", content, re.DOTALL)
    if not match:
        return {}

    frontmatter = match.group(1)
    result = {}

    for key in keys:
        pattern = rf"^{key}:\s*(.+)$"
        m = re.search(pattern, frontmatter, re.MULTILINE)
        if m:
            result[key] = normalize_frontmatter_value(m.group(1).strip())

    return result


def normalize_frontmatter_value(value: str) -> str:
    """Normalize the single-line YAML scalars used in our frontmatter."""
    if len(value) < 2:
        return value
    if value[0] == '"' and value[-1] == '"':
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return value[1:-1]
    if value[0] == "'" and value[-1] == "'":
        return value[1:-1].replace("''", "'")
    return value


def validate_frontmatter_yaml_scalars(content: str, path_label: str) -> list[str]:
    """Catch YAML-invalid unquoted scalar patterns in frontmatter."""
    match = re.match(r"^---\s*\n(.*?)\n---", content, re.DOTALL)
    if not match:
        return []

    errors = []
    frontmatter = match.group(1)

    for line_number, line in enumerate(frontmatter.splitlines(), start=2):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        m = re.match(r"^([A-Za-z0-9_-]+):\s*(.*)$", line)
        if not m:
            continue

        key = m.group(1)
        value = m.group(2).strip()
        if not value or value[0] in {'"', "'", "|", ">"}:
            continue

        if ": " in value:
            errors.append(
                f"{path_label}:{line_number}: frontmatter field `{key}` contains "
                "`: ` and must be quoted for strict YAML parsers"
            )

    return errors


def strip_frontmatter(content: str) -> str:
    """Remove YAML frontmatter from a Markdown document."""
    match = re.match(r"^---\s*\n.*?\n---\s*\n?", content, re.DOTALL)
    if not match:
        return content
    return content[match.end() :]


def skill_display_name(skill_name: str) -> str:
    """Convert a skill id into the display name used by Codex."""
    acronyms = {"dart": "DART", "ci": "CI", "io": "IO", "pr": "PR"}
    return " ".join(acronyms.get(part, part.title()) for part in skill_name.split("-"))


def sentence_case(text: str) -> str:
    """Lowercase only the first character for compact skill descriptions."""
    if not text:
        return text
    return text[0].lower() + text[1:]


def validate_codex_skill(skill_path: Path) -> list[str]:
    """Validate skill meets Codex format requirements. Returns list of warnings."""
    warnings = []
    content = skill_path.read_text()
    meta = parse_skill_frontmatter(content)

    name = meta.get("name", "")
    desc = meta.get("description", "")

    if len(name) > CODEX_NAME_LIMIT:
        warnings.append(
            f"name exceeds Codex limit ({len(name)} > {CODEX_NAME_LIMIT} chars)"
        )
    if "\n" in name:
        warnings.append("name contains newline (Codex requires single line)")
    if len(desc) > CODEX_DESC_LIMIT:
        warnings.append(
            f"description exceeds Codex limit ({len(desc)} > {CODEX_DESC_LIMIT} chars)"
        )
    if "\n" in desc:
        warnings.append("description contains newline (Codex requires single line)")

    return warnings


def list_command_names(command_dir: Path) -> set[str]:
    """Return command names from a flat command directory."""
    if not command_dir.exists():
        return set()
    return {path.stem for path in command_dir.glob("*.md")}


def list_skill_names(skill_dir: Path) -> tuple[set[str], list[str]]:
    """Return declared skill names and frontmatter/folder consistency errors."""
    if not skill_dir.exists():
        return set(), []

    names: set[str] = set()
    errors: list[str] = []

    for skill_path in sorted(skill_dir.glob("*/SKILL.md")):
        folder_name = skill_path.parent.name
        rel_path = display_path(skill_path)
        meta = parse_skill_frontmatter(skill_path.read_text())
        declared_name = meta.get("name", "")

        if not declared_name:
            errors.append(f"{rel_path}: missing required frontmatter field `name`")
            continue
        if declared_name != folder_name:
            errors.append(
                f"{rel_path}: folder name `{folder_name}` does not match "
                f"declared skill name `{declared_name}`"
            )
        if declared_name in names:
            errors.append(
                f"{rel_path}: duplicate declared skill name `{declared_name}`"
            )

        names.add(declared_name)

    return names, errors


def print_skill_name_errors(label: str, errors: list[str]) -> None:
    """Print skill name validation errors with an agent/tool label."""
    for error in errors:
        print(f"  SKILL ERROR: {label}: {error}")


def format_names(names: set[str]) -> str:
    """Format a set of names for readable mismatch output."""
    if not names:
        return "(none)"
    return ", ".join(sorted(names))


def check_capability_parity(repo_root: Path) -> bool:
    """Verify supported agents expose the same effective DART capabilities."""
    claude_commands = list_command_names(repo_root / ".claude" / "commands")
    opencode_commands = list_command_names(repo_root / ".opencode" / "command")
    shared_skills, shared_skill_errors = list_skill_names(
        repo_root / ".claude" / "skills"
    )
    codex_skills, codex_skill_errors = list_skill_names(repo_root / ".codex" / "skills")

    expected = claude_commands | shared_skills
    agent_sets = {
        "Claude Code": claude_commands | shared_skills,
        "OpenCode": opencode_commands | shared_skills,
        "Codex": codex_skills,
    }

    ok = True

    if shared_skill_errors:
        ok = False
        print_skill_name_errors(".claude/skills", shared_skill_errors)
    if codex_skill_errors:
        ok = False
        print_skill_name_errors(".codex/skills", codex_skill_errors)

    if claude_commands & shared_skills:
        ok = False
        print(
            "  COLLISION: commands and skills share names: "
            f"{format_names(claude_commands & shared_skills)}"
        )

    for agent, capabilities in agent_sets.items():
        missing = expected - capabilities
        extra = capabilities - expected
        if missing or extra:
            ok = False
            print(f"  MISMATCH: {agent}")
            if missing:
                print(f"    missing: {format_names(missing)}")
            if extra:
                print(f"    extra:   {format_names(extra)}")

    if ok:
        print(f"  OK: {len(expected)} shared capabilities: {format_names(expected)}")

    return ok


def validate_style_and_budget(repo_root: Path) -> bool:
    """Validate concise, consistent command and skill metadata across agents."""
    errors: list[str] = []

    skill_dirs = [
        (".claude/skills", repo_root / ".claude" / "skills"),
        (".codex/skills", repo_root / ".codex" / "skills"),
    ]
    command_dirs = [
        (".claude/commands", repo_root / ".claude" / "commands"),
        (".opencode/command", repo_root / ".opencode" / "command"),
    ]

    for label, skill_dir in skill_dirs:
        for skill_path in sorted(skill_dir.glob("*/SKILL.md")):
            content = skill_path.read_text()
            meta = parse_skill_frontmatter(content)
            name = meta.get("name", "")
            desc = meta.get("description", "")
            path_label = display_path(skill_path)

            errors.extend(validate_frontmatter_yaml_scalars(content, path_label))

            if not desc:
                errors.append(f"{path_label}: missing required description")
            elif name and not desc.startswith(f"{skill_display_name(name)}: "):
                errors.append(
                    f"{path_label}: description should start with "
                    f"`{skill_display_name(name)}: `"
                )

            line_count = len(content.splitlines())
            if line_count > MAX_SKILL_LINES:
                errors.append(
                    f"{path_label}: {line_count} lines exceeds "
                    f"{MAX_SKILL_LINES}-line skill budget"
                )

            if label == ".codex/skills":
                for warning in validate_codex_skill(skill_path):
                    errors.append(f"{path_label}: {warning}")

    for label, command_dir in command_dirs:
        for command_path in sorted(command_dir.glob("*.md")):
            content = command_path.read_text()
            meta = parse_command_frontmatter(content)
            desc = meta.get("description", "")
            path_label = display_path(command_path)

            errors.extend(validate_frontmatter_yaml_scalars(content, path_label))

            if not desc:
                errors.append(f"{path_label}: missing required description")
            elif desc[0].isupper():
                errors.append(
                    f"{path_label}: description should be a lowercase sentence "
                    "fragment; Codex adds the display name"
                )
            elif desc.endswith("."):
                errors.append(f"{path_label}: description should not end with a period")

            line_count = len(content.splitlines())
            if line_count > MAX_COMMAND_LINES:
                errors.append(
                    f"{path_label}: {line_count} lines exceeds "
                    f"{MAX_COMMAND_LINES}-line command budget"
                )

    if errors:
        for error in errors:
            print(f"  STYLE ERROR: {error}")
        return False

    print(
        "  OK: skill descriptions use display-name prefixes; command descriptions "
        "stay concise; files are within context budgets"
    )
    return True


def parse_workflow_rows(workflow_content: str) -> dict[str, list[str]]:
    """Extract user-invoked workflow table rows keyed by capability name."""
    rows: dict[str, list[str]] = {}
    in_user_table = False

    for line in workflow_content.splitlines():
        if line.startswith("## User-Invoked Workflows"):
            in_user_table = True
            continue
        if in_user_table and line.startswith("## "):
            break
        if not in_user_table or not line.startswith("| `dart-"):
            continue

        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 5:
            continue
        match = re.fullmatch(r"`(dart-[a-z0-9-]+)`", cells[0])
        if match:
            rows[match.group(1)] = cells

    return rows


def parse_domain_skill_rows(workflow_content: str) -> dict[str, list[str]]:
    """Extract domain skill table rows keyed by capability name."""
    rows: dict[str, list[str]] = {}
    in_domain_table = False

    for line in workflow_content.splitlines():
        if line.startswith("## Domain Skills"):
            in_domain_table = True
            continue
        if in_domain_table and line.startswith("## "):
            break
        if not in_domain_table or not line.startswith("| `dart-"):
            continue

        cells = [cell.strip() for cell in line.strip().strip("|").split("|")]
        if len(cells) < 3:
            continue
        match = re.fullmatch(r"`(dart-[a-z0-9-]+)`", cells[0])
        if match:
            rows[match.group(1)] = cells

    return rows


def extract_required_reading_from_content(content: str) -> list[str]:
    """Extract @file entries from a command Required Reading section."""
    readings: list[str] = []
    in_required_reading = False

    for line in content.splitlines():
        if line.startswith("## Required Reading"):
            in_required_reading = True
            continue
        if in_required_reading and line.startswith("## "):
            break
        if not in_required_reading:
            continue

        match = re.match(r"@([^\s]+)", line.strip())
        if match:
            readings.append(match.group(1))

    return readings


def extract_required_reading(command_path: Path) -> list[str]:
    """Extract @file entries from a command's Required Reading section."""
    return extract_required_reading_from_content(command_path.read_text())


def required_reading_path_errors(repo_root: Path, command_path: Path) -> list[str]:
    """Return errors for @file required-reading entries that do not exist."""
    errors: list[str] = []
    for required in extract_required_reading(command_path):
        required_path = repo_root / required
        if not required_path.exists():
            errors.append(
                f"{display_path(command_path)}: required reading `{required}` "
                "does not exist"
            )
    return errors


def missing_required_reading_errors(
    workflow_path_label: str,
    name: str,
    public_path: str,
    required_reading: list[str],
) -> list[str]:
    """Return workflow row errors for required reading not in public path."""
    return [
        f"{workflow_path_label}: `{name}` missing required reading `{required}`"
        for required in required_reading
        if required != "AGENTS.md" and required not in public_path
    ]


def has_gate_evidence(gate_cell: str) -> bool:
    """Return whether a workflow gate cell names a local gate or exception."""
    accepted_markers = [
        "pixi run",
        "git status",
        "local reproduction",
        "focused build/test",
        "focused tests",
        "target-specific gates",
        "release-target focused tests",
        "regression test",
        "CI/review green",
        "CI is green",
        "Read-only",
        "read-only",
        "No local gate",
        "Maintainer-only",
        "release verification",
        "Relevant docs/AI checks",
    ]
    return any(marker in gate_cell for marker in accepted_markers)


def _validate_capability_entries(
    section: str, entries: object, expected_names: set[str]
) -> tuple[dict[str, str], list[str]]:
    """Validate one capability manifest section and return declared profiles."""
    errors: list[str] = []
    declared_profiles: dict[str, str] = {}

    if not isinstance(entries, list):
        return declared_profiles, [
            f"docs/ai/capabilities.json: `{section}` must be a list"
        ]

    allowed_gate_profiles = (
        CAPABILITY_DOMAIN_GATE_PROFILE_VALUES
        if section == "domain_skills"
        else CAPABILITY_WORKFLOW_GATE_PROFILE_VALUES
    )

    for index, entry in enumerate(entries, start=1):
        label = f"docs/ai/capabilities.json:{section}[{index}]"
        if not isinstance(entry, dict):
            errors.append(f"{label}: entry must be an object")
            continue

        name = entry.get("name")
        category = entry.get("category")
        status = entry.get("status")
        gate_profile = entry.get("gate_profile")

        if not isinstance(name, str) or not name:
            errors.append(f"{label}: missing string `name`")
            continue
        if not re.fullmatch(r"dart-[a-z0-9-]+", name):
            errors.append(f"{label}: invalid capability name `{name}`")
        if name in declared_profiles:
            errors.append(f"{label}: duplicate capability `{name}`")
        declared_profiles[name] = gate_profile if isinstance(gate_profile, str) else ""

        if not isinstance(category, str) or not re.fullmatch(
            r"[a-z][a-z0-9-]*", category or ""
        ):
            errors.append(f"{label}: missing or invalid `category`")
        if status not in CAPABILITY_STATUS_VALUES:
            errors.append(
                f"{label}: status must be one of "
                f"{format_names(CAPABILITY_STATUS_VALUES)}"
            )
        if gate_profile not in allowed_gate_profiles:
            errors.append(
                f"{label}: gate_profile must be one of "
                f"{format_names(allowed_gate_profiles)}"
            )

    declared_names = set(declared_profiles)
    missing = expected_names - declared_names
    extra = declared_names - expected_names
    if missing:
        errors.append(
            f"docs/ai/capabilities.json: `{section}` missing "
            f"{format_names(missing)}"
        )
    if extra:
        errors.append(
            f"docs/ai/capabilities.json: `{section}` has unknown "
            f"{format_names(extra)}"
        )

    return declared_profiles, errors


def validate_capability_manifest(
    repo_root: Path,
    command_names: set[str],
    skill_names: set[str],
    workflow_rows: dict[str, list[str]],
    domain_skill_rows: dict[str, list[str]],
) -> list[str]:
    """Validate the machine-readable AI capability manifest."""
    manifest_path = repo_root / "docs" / "ai" / "capabilities.json"
    if not manifest_path.exists():
        return [f"{display_path(manifest_path)}: missing capability manifest"]

    try:
        manifest = json.loads(manifest_path.read_text())
    except json.JSONDecodeError as error:
        return [f"{display_path(manifest_path)}:{error.lineno}: invalid JSON"]

    errors: list[str] = []
    if not isinstance(manifest, dict):
        return [f"{display_path(manifest_path)}: manifest must be an object"]
    if manifest.get("schema_version") != CAPABILITY_SCHEMA_VERSION:
        errors.append(
            f"{display_path(manifest_path)}: schema_version must be "
            f"{CAPABILITY_SCHEMA_VERSION}"
        )

    workflow_profiles, workflow_errors = _validate_capability_entries(
        "workflows", manifest.get("workflows"), command_names
    )
    skill_profiles, skill_errors = _validate_capability_entries(
        "domain_skills", manifest.get("domain_skills"), skill_names
    )
    errors.extend(workflow_errors)
    errors.extend(skill_errors)

    for name in sorted(workflow_profiles):
        cells = workflow_rows.get(name)
        if not cells:
            errors.append(
                f"{display_path(manifest_path)}: workflow `{name}` lacks "
                "docs/ai/workflows.md row"
            )
            continue
        if not cells[3] or cells[3] == "-":
            errors.append(
                f"{display_path(manifest_path)}: workflow `{name}` lacks public path"
            )
        if not cells[4] or not has_gate_evidence(cells[4]):
            errors.append(
                f"{display_path(manifest_path)}: workflow `{name}` lacks gate evidence"
            )
        profile_markers = CAPABILITY_WORKFLOW_GATE_PROFILE_MARKERS.get(
            workflow_profiles[name]
        )
        if profile_markers and not any(
            marker in cells[4] for marker in profile_markers
        ):
            errors.append(
                f"{display_path(manifest_path)}: workflow `{name}` gate evidence "
                f"does not match `{workflow_profiles[name]}` profile"
            )

    for name in sorted(skill_profiles):
        cells = domain_skill_rows.get(name)
        if not cells:
            errors.append(
                f"{display_path(manifest_path)}: domain skill `{name}` lacks "
                "docs/ai/workflows.md row"
            )
            continue
        public_path = cells[2] if len(cells) > 2 else ""
        if not public_path or public_path == "-":
            errors.append(
                f"{display_path(manifest_path)}: domain skill `{name}` lacks "
                "required docs/command path"
            )
        elif not any(
            marker in public_path for marker in ("docs/", "CONTRIBUTING.md", "pixi run")
        ):
            errors.append(
                f"{display_path(manifest_path)}: domain skill `{name}` lacks "
                "usable docs/command path"
            )

    return errors


def validate_approval_boundary(repo_root: Path) -> list[str]:
    """Check GitHub/remote mutation commands explicitly require approval."""
    errors: list[str] = []
    mutation_pattern_strings = [
        r"\bgit push\b",
        r"\bgh pr create\b",
        r"\bgh pr comment\b",
        r"\bgh pr edit\b",
        r"\bgh pr merge\b",
        r"\bgh pr ready\b",
        r"\bgh pr review\b",
        r"\bgh issue comment\b",
        r"\bgh issue close\b",
        r"\bgh issue edit\b",
        r"\bgh run rerun\b",
        r"\bgh pr update-branch\b",
        r"\bgh api\b(?!\s+graphql\b)[^\n;|&]*(?:(?:--method(?:=|\s+)|-X\s*)"
        + r"(?:POST|PATCH|PUT|DELETE)\b|(?:--field|-f)\s+\w+=)",
        r"\bgh api graphql\b[^\n;|&]*\bmutation\b",
        r"^\s*mutation(?:\s+\w+)?\s*\{",
        r"\bgit fetch\b[^\n;|&]*(?:--prune\b|-[A-Za-z]*p[A-Za-z]*\b)",
        r"\bgit remote update\b[^\n;|&]*(?:--prune\b|-[A-Za-z]*p[A-Za-z]*\b)",
        r"\bgit remote prune\b",
        r"\bgit branch\b[^\n;|&]*(?:-[A-Za-z]*[dD][A-Za-z]*\b|--delete\b)",
        r"\bresolveReviewThread\b",
        r"--delete-branch\b",
        r"\borigin --delete\b",
        r"^\s*(?:\d+\.\s*)?(?:[-*]\s*)?(?:if approved,\s*)?push\b",
        r"\bpush(?:ing)? (?:the )?(?:fix(?:es)?|change|changes|commit|commits|branch)\b",
        r"\bpush(?:ing)? and (?:ask|create|creating|monitor|open|opening|watch)\b",
        r"\bpush(?:ing)?, (?:comment|resolve|re-trigger|retrigger)\b",
        r"\bpush(?:es|ing)?\b.{0,120}\b(?:PRs?|pull requests?)\b",
        r"\b(?:create|creating|open|opening)\b.{0,80}\b(?:PRs?|pull requests?)\b",
        r"\bcreate or update (?:a |the )?.{0,120}(?:PRs?|pull requests?)\b",
        r"\bcreate/update (?:a |the )?.{0,120}(?:PRs?|pull requests?)\b",
        r"\bupdate (?:a |the )?.{0,120}(?:PRs?|pull requests?)\b",
        r"\bupdating (?:a |the )?.{0,120}(?:PRs?|pull requests?)\b",
        r"\b(?:PRs?|pull requests?) update\b",
        r"\bset(?:ting)? (?:the )?.{0,120}milestones?\b",
        r"\brerun(?:ning)? (?:the )?(?:failed )?(?:CI|check|job|jobs|workflow)\b",
        r"\bmerg(?:e|ing) (?:the )?(?:PR|pull request)\b",
        r"\bresolv(?:e|ing) (?:only )?(?:the )?(?:addressed |reviewed |reviewed and addressed |reviewed, addressed )?(?:review )?(?:thread IDs?|threads?|conversations?)\b",
        r"\bdelet(?:e|ing)\b.{0,80}\bbranch(?:es)?\b",
        r"\bbranch(?:es)? (?:is |are )?(?:a )?deletion candidate(?:s)?\b",
        r"\bdeletion candidate(?:s)?\b",
        r"\bprun(?:e|ing)\b.{0,80}\brefs?\b",
        r"\bforce-delete locally\b",
    ]
    mutation_pattern = re.compile(
        "|".join(f"(?:{pattern})" for pattern in mutation_pattern_strings), re.I
    )
    mutation_examples = {
        "Open a PR with the release milestone": "uppercase PR opening prose",
        "opening the release-branch PR": "gerund PR opening prose",
        "After approval, push the fix": "non-line-initial push prose",
        "pushing fixes to the PR": "gerund push prose",
        "pushes to PRs": "plural push/PR prose",
        "open a pull request": "pull request opening prose",
        "update the pull request": "pull request update prose",
        "open a DART pull request": "qualified pull request opening prose",
        "Create release packaging PR": "qualified PR opening prose",
        "gh pr update-branch": "PR update-branch command",
        "gh api --method POST repos/dartsim/dart/issues/1/comments": "mutating gh api command",
        "gh api --method=PATCH repos/dartsim/dart/issues/1": "mutating gh api equals method command",
        "gh api -X DELETE repos/dartsim/dart/git/refs/heads/tmp": "mutating gh api short method command",
        "gh api -XPOST repos/dartsim/dart/issues": "mutating gh api attached short method command",
        "gh api repos/dartsim/dart/issues/1/comments -f body=x": "implicit mutating gh api field command",
        'gh api graphql -f query="mutation { foo }"': "mutating gh api graphql command",
        "mutation { resolveReviewThread": "GraphQL mutation block",
        "git fetch --all --prune": "fetch prune command",
        "git fetch origin --prune": "fetch remote prune command",
        "git fetch origin -p": "fetch remote short prune command",
        "git fetch -p": "fetch short prune command",
        "git remote update --prune": "remote update prune command",
        "git remote update origin --prune": "remote update remote prune command",
        "git remote update -p": "remote update short prune command",
        "git remote prune origin": "remote prune command",
        "pruning refs": "pruning refs prose",
        "Deleting stale branches": "branch deletion prose",
        "delete any branch": "any branch deletion prose",
        "deleting any local or remote branch": "local-or-remote branch deletion prose",
        "git branch -D <HEAD_BRANCH>": "local branch deletion command",
        "git branch -df <HEAD_BRANCH>": "combined branch force-delete command",
        "git branch -rd origin/foo": "combined remote branch delete command",
        "git branch --delete --force <HEAD_BRANCH>": "long-form branch deletion command",
        "Branch is a deletion candidate": "branch deletion candidate prose",
        "resolve the thread": "singular thread resolution prose",
        "resolve only reviewed and addressed thread IDs": "reviewed thread ID resolution prose",
        "update the PR after editing": "PR update prose",
        "PR update after changelog edit": "noun PR update prose",
        "After opening the PR, merge PR": "compound PR mutation prose",
        "Rerunning failed jobs": "gerund CI rerun prose",
        "Merging PR after checks pass": "gerund merge prose",
        "Resolving addressed threads via GraphQL": "gerund thread-resolution prose",
    }
    for example, label in mutation_examples.items():
        if not mutation_pattern.search(example):
            errors.append(f"internal mutation pattern self-check failed: {label}")

    approval_pattern = re.compile(r"explicit\s+(?:maintainer/user\s+)?approval", re.I)

    def build_scan_text(lines: list[str], index: int, in_fence: bool) -> str:
        """Join a Markdown paragraph or wrapped list item for prose scanning."""
        current = lines[index]
        current_stripped = current.strip()
        if not current_stripped:
            return ""

        parts = [current_stripped]
        current_indent = len(current) - len(current.lstrip())
        current_is_list = bool(re.match(r"(?:[-*]|\d+\.)\s", current_stripped))
        current_is_block = current_stripped.startswith(("#", "|", "```", "---"))

        if current_is_block or in_fence:
            return current_stripped

        for next_line in lines[index + 1 : index + 5]:
            next_stripped = next_line.strip()
            if not next_stripped or next_stripped.startswith(("#", "|", "```", "---")):
                break

            next_indent = len(next_line) - len(next_line.lstrip())
            next_is_list = bool(re.match(r"(?:[-*]|\d+\.)\s", next_stripped))
            if current_is_list:
                if next_indent <= current_indent:
                    break
            elif next_is_list:
                break

            parts.append(next_stripped)

        return " ".join(parts)

    def strip_safe_prune_commands(scan_text: str) -> str:
        """Remove explicitly safe prune inspection commands before scanning."""
        scan_text = re.sub(
            r"\bgit fetch\b[^\n;|&]*--no-prune\b",
            "",
            scan_text,
            flags=re.I,
        )
        scan_text = re.sub(
            r"\bgit remote prune\b[^\n;|&]*--dry-run\b",
            "",
            scan_text,
            flags=re.I,
        )
        return scan_text

    def scan_lines(label: str, lines: list[str]) -> None:
        frontmatter_end = 0
        if lines and lines[0].strip() == "---":
            for line_index, frontmatter_line in enumerate(lines[1:], start=1):
                if frontmatter_line.strip() == "---":
                    frontmatter_end = line_index + 1
                    break

        in_fence = False
        for index, line in enumerate(lines):
            stripped = line.strip()
            if stripped.startswith("```"):
                in_fence = not in_fence
                continue
            if index < frontmatter_end:
                continue
            if re.search(r"\bwhy resolve after\b", line, re.I):
                continue
            scan_text = strip_safe_prune_commands(
                build_scan_text(lines, index, in_fence).replace("$ARGUMENTS", "")
            )
            if not mutation_pattern.search(scan_text):
                continue

            start = max(0, index - 6)
            end = min(len(lines), index + 8)
            context = "\n".join(lines[start:end])
            if approval_pattern.search(context):
                continue

            errors.append(
                f"{label}:{index + 1}: mutation command lacks "
                "nearby explicit approval boundary"
            )

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        ["Push changes: $ARGUMENTS"],
    )
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: same-line "
            "$ARGUMENTS hid mutation prose"
        )
    else:
        del errors[before_self_check:]

    before_self_check = len(errors)
    scan_lines("internal approval-boundary self-check", ["git fetch --all --prune"])
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: fetch prune command "
            "was not detected"
        )
    else:
        del errors[before_self_check:]

    before_self_check = len(errors)
    scan_lines("internal approval-boundary self-check", ["git remote prune origin"])
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: remote prune command "
            "was not detected"
        )
    else:
        del errors[before_self_check:]

    before_self_check = len(errors)
    scan_lines("internal approval-boundary self-check", ["git fetch --all --no-prune"])
    if len(errors) != before_self_check:
        del errors[before_self_check:]
        errors.append(
            "internal approval-boundary self-check failed: --no-prune command "
            "was flagged"
        )

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        [
            "```bash",
            "git fetch origin <RELEASE_BRANCH>",
            "git checkout -B release/<NEW_VERSION>-version-bump origin/<RELEASE_BRANCH>",
            "```",
        ],
    )
    if len(errors) != before_self_check:
        del errors[before_self_check:]
        errors.append(
            "internal approval-boundary self-check failed: fenced safe fetch "
            "was flagged"
        )

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        ["git fetch --all --no-prune && git push"],
    )
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: --no-prune hid "
            "compound mutation"
        )
    else:
        del errors[before_self_check:]

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        ["git remote prune origin --dry-run"],
    )
    if len(errors) != before_self_check:
        del errors[before_self_check:]
        errors.append(
            "internal approval-boundary self-check failed: prune dry-run command "
            "was flagged"
        )

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        ["git remote prune origin --dry-run; gh pr merge"],
    )
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: prune dry-run hid "
            "compound mutation"
        )
    else:
        del errors[before_self_check:]

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        ["gh api graphql -f query='", "  query { repository { id } }"],
    )
    if len(errors) != before_self_check:
        del errors[before_self_check:]
        errors.append(
            "internal approval-boundary self-check failed: read-only GraphQL "
            "query was flagged"
        )

    before_self_check = len(errors)
    scan_lines(
        "internal approval-boundary self-check",
        [
            "- `action`: delete only after ownership is clear",
            "  and the branch deletion is requested",
        ],
    )
    if len(errors) == before_self_check:
        errors.append(
            "internal approval-boundary self-check failed: wrapped branch "
            "deletion prose was not detected"
        )
    else:
        del errors[before_self_check:]

    scan_paths = [
        *sorted((repo_root / ".claude" / "commands").glob("*.md")),
        *sorted((repo_root / ".claude" / "skills").glob("*/SKILL.md")),
        *sorted((repo_root / "docs" / "ai").glob("*.md")),
        repo_root / "docs" / "onboarding" / "ai-tools.md",
    ]

    for path in scan_paths:
        scan_lines(display_path(path), path.read_text().splitlines())

    for command_path in sorted((repo_root / ".claude" / "commands").glob("*.md")):
        rendered_wrapper = render_codex_command_skill(command_path).split(
            "\n## Command Body\n", 1
        )[0]
        scan_lines(
            f"generated Codex wrapper for {command_path.stem}",
            rendered_wrapper.splitlines(),
        )

    ai_tools = repo_root / "docs" / "onboarding" / "ai-tools.md"
    if ai_tools.exists() and re.search(
        r"resolve (?:all|every) unresolved thread", ai_tools.read_text(), re.I
    ):
        errors.append(
            f"{display_path(ai_tools)}: bulk review-thread resolution is forbidden"
        )

    return errors


def validate_ai_docs(repo_root: Path) -> bool:
    """Validate the checked AI-native docs that index generated capabilities."""
    errors: list[str] = []
    docs_dir = repo_root / "docs" / "ai"
    required_files = [
        docs_dir / "README.md",
        docs_dir / "principles.md",
        docs_dir / "north-star.md",
        docs_dir / "workflows.md",
        docs_dir / "verification.md",
        docs_dir / "sessions.md",
        docs_dir / "components.md",
        docs_dir / "capabilities.json",
    ]

    for path in required_files:
        if not path.exists():
            errors.append(f"{display_path(path)}: missing required AI doc")

    agents_content = (repo_root / "AGENTS.md").read_text()
    docs_readme_content = (repo_root / "docs" / "README.md").read_text()
    if "docs/ai/README.md" not in agents_content:
        errors.append("AGENTS.md: missing docs/ai/README.md pointer")
    if "docs/ai/principles.md" not in agents_content:
        errors.append("AGENTS.md: missing docs/ai/principles.md pointer")
    if "ai/README.md" not in docs_readme_content:
        errors.append("docs/README.md: missing ai/README.md index link")

    workflows_path = docs_dir / "workflows.md"
    if workflows_path.exists():
        workflow_content = workflows_path.read_text()
        command_names = list_command_names(repo_root / ".claude" / "commands")
        skill_names, skill_errors = list_skill_names(repo_root / ".claude" / "skills")
        errors.extend(skill_errors)
        expected = command_names | skill_names
        workflow_rows = parse_workflow_rows(workflow_content)
        domain_skill_rows = parse_domain_skill_rows(workflow_content)

        if has_gate_evidence("explicit approval before push"):
            errors.append(
                "internal workflow gate self-check failed: approval-only gate accepted"
            )

        fake_required = extract_required_reading_from_content("""
## Required Reading

@AGENTS.md
@docs/onboarding/required.md (inline note)
@docs/onboarding/extra.md

## Workflow
""")
        fake_missing = missing_required_reading_errors(
            "docs/ai/workflows.md",
            "dart-fake",
            "docs/onboarding/required.md",
            fake_required,
        )
        if len(fake_missing) != 1 or "docs/onboarding/extra.md" not in fake_missing[0]:
            errors.append(
                "internal workflow docs self-check failed: missing required "
                "reading was not detected"
            )

        for name in sorted(expected):
            if f"`{name}`" not in workflow_content:
                errors.append(
                    f"{display_path(workflows_path)}: missing capability `{name}`"
                )

        for name in sorted(command_names):
            command_path = repo_root / ".claude" / "commands" / f"{name}.md"
            cells = workflow_rows.get(name)
            if not cells:
                errors.append(
                    f"{display_path(workflows_path)}: missing workflow row `{name}`"
                )
                continue
            public_path = cells[3]
            gate = cells[4]
            if not public_path or public_path == "-":
                errors.append(
                    f"{display_path(workflows_path)}: `{name}` missing public path"
                )
            if not gate or not has_gate_evidence(gate):
                errors.append(
                    f"{display_path(workflows_path)}: `{name}` missing gate evidence"
                )

            errors.extend(required_reading_path_errors(repo_root, command_path))
            errors.extend(
                missing_required_reading_errors(
                    display_path(workflows_path),
                    name,
                    public_path,
                    extract_required_reading(command_path),
                )
            )

        for name in sorted(skill_names):
            if name not in domain_skill_rows:
                errors.append(
                    f"{display_path(workflows_path)}: missing domain skill row `{name}`"
                )

        errors.extend(
            validate_capability_manifest(
                repo_root,
                command_names,
                skill_names,
                workflow_rows,
                domain_skill_rows,
            )
        )

        approval_workflows = {
            "dart-backport-pr",
            "dart-branch-cleanup",
            "dart-close-issue",
            "dart-docs-update",
            "dart-downstream-fix",
            "dart-fix-ci",
            "dart-fix-issue",
            "dart-manage-pr",
            "dart-merge-pr",
            "dart-mechanical-refactor",
            "dart-new-task",
            "dart-next",
            "dart-pr",
            "dart-release-ci-fix",
            "dart-release-merge-main",
            "dart-release-packaging",
            "dart-resume",
            "dart-review-pr",
            "dart-triage-issue",
        }
        for name in sorted(approval_workflows & command_names):
            cells = workflow_rows.get(name)
            row_text = " ".join(cells) if cells else ""
            if cells and not ("explicit" in row_text and "approval" in row_text):
                errors.append(
                    f"{display_path(workflows_path)}: `{name}` missing approval gate"
                )

        documented = set(re.findall(r"`(dart-[a-z0-9-]+)`", workflow_content))
        extra = documented - expected
        for name in sorted(extra):
            errors.append(
                f"{display_path(workflows_path)}: unknown capability `{name}`"
            )

        if "docs/ai/workflows.md" not in agents_content:
            errors.append("AGENTS.md: missing docs/ai/workflows.md catalog pointer")
        if ".claude/commands/" not in agents_content:
            errors.append("AGENTS.md: missing .claude/commands/ source pointer")
        if ".codex/skills/" not in agents_content:
            errors.append("AGENTS.md: missing .codex/skills/ generated pointer")

    errors.extend(validate_approval_boundary(repo_root))

    private_pattern = re.compile(r"(?:/home/|/Users/|~/|fbsource|arvr/libraries)")
    for path in docs_dir.glob("*.md"):
        for line_number, line in enumerate(path.read_text().splitlines(), start=1):
            if private_pattern.search(line):
                errors.append(
                    f"{display_path(path)}:{line_number}: private path reference"
                )

    if errors:
        for error in errors:
            print(f"  AI DOC ERROR: {error}")
        return False

    print("  OK: docs/ai entrypoint, workflow index, and public paths are valid")
    return True


def has_auto_gen_header(content: str) -> bool:
    """Check if content has the auto-generated header."""
    return "<!-- AUTO-GENERATED FILE" in content


def strip_auto_gen_header(content: str) -> str:
    """Remove auto-generated header from content for comparison."""
    lines = content.split("\n")
    result_lines = []
    in_header = False

    for line in lines:
        if line.startswith("<!-- AUTO-GENERATED FILE"):
            in_header = True
            continue
        if in_header:
            if line.startswith("<!--"):
                continue
            in_header = False
        result_lines.append(line)

    return "\n".join(result_lines)


def add_auto_gen_header(content: str, source_path: str) -> str:
    """Add auto-generated header after YAML frontmatter (if present) to preserve tool parsing."""
    header = AUTO_GEN_HEADER.format(source_path=source_path).rstrip("\n")

    if content.startswith("---"):
        lines = content.split("\n")
        for i, line in enumerate(lines[1:], start=1):
            if line.strip() == "---":
                frontmatter = "\n".join(lines[: i + 1])
                rest = "\n".join(lines[i + 1 :])
                return frontmatter + "\n" + header + "\n" + rest
    return header + "\n\n" + content


def sync_flat_files(
    source_dir: Path,
    target_dir: Path,
    pattern: str,
    label: str,
    check_only: bool = False,
) -> tuple[bool, int, int, int]:
    """Sync flat files from source to target directory.

    Returns:
        Tuple of (all_synced, synced_count, skipped_count, orphan_count)
    """
    if not source_dir.exists():
        print(f"Source directory not found: {source_dir}")
        return False, -1, 0, 0

    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_files = sorted(source_dir.glob(pattern))
    all_synced = True
    synced_count = 0
    skipped_count = 0

    for source_file in source_files:
        target_file = target_dir / source_file.name
        source_content = source_file.read_text()
        source_rel_path = source_file.relative_to(get_repo_root())

        if target_file.exists():
            target_content = target_file.read_text()
            target_content_stripped = strip_auto_gen_header(target_content)
            if source_content == target_content_stripped and has_auto_gen_header(
                target_content
            ):
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            status = "MISMATCH" if target_file.exists() else "MISSING"
            print(f"  {status}: {source_file.name}")
        else:
            content_with_header = add_auto_gen_header(
                source_content, str(source_rel_path)
            )
            target_file.write_text(content_with_header)
            print(f"  SYNCED:   {source_file.name}")
            synced_count += 1

    # Check for orphaned files
    if target_dir.exists():
        target_files = set(f.name for f in target_dir.glob(pattern))
        source_names = set(f.name for f in source_files)
        orphaned = target_files - source_names

        for orphan in sorted(orphaned):
            all_synced = False
            orphan_path = target_dir / orphan
            if check_only:
                print(f"  ORPHAN:   {orphan}")
            else:
                orphan_path.unlink()
                print(f"  REMOVED:  {orphan}")
    else:
        orphaned = set()

    return all_synced, synced_count, skipped_count, len(orphaned)


def render_codex_command_skill(command_path: Path) -> str:
    """Render a Claude/OpenCode command as a Codex skill."""
    command_name = command_path.stem
    command_content = command_path.read_text()
    meta = parse_command_frontmatter(command_content)
    description = meta.get("description", f"Run the {command_name} workflow")
    command_body = strip_frontmatter(command_content).strip()

    skill_description = (
        f"{skill_display_name(command_name)}: {sentence_case(description)}"
    )

    return f"""\
---
name: {command_name}
description: {json.dumps(skill_description)}
---

# {command_name}

Use this skill in Codex to run the DART `{command_name}` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/{command_name} <arguments>`
- Codex: `${command_name} <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

{command_body}
"""


def sync_codex_skills(
    skill_source_dir: Path,
    command_source_dir: Path,
    target_dir: Path,
    check_only: bool = False,
) -> tuple[bool, int, int, int]:
    """Sync explicit skills plus command-derived workflow skills to Codex."""
    if not skill_source_dir.exists():
        print(f"Source directory not found: {skill_source_dir}")
        return False, -1, 0, 0
    if not command_source_dir.exists():
        print(f"Source directory not found: {command_source_dir}")
        return False, -1, 0, 0

    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_skill_names_set, source_skill_errors = list_skill_names(skill_source_dir)
    if source_skill_errors:
        for error in source_skill_errors:
            print(f"  ERROR:    {error}")
        return False, -1, 0, 0

    source_skill_names = sorted(source_skill_names_set)
    source_command_files = sorted(command_source_dir.glob("*.md"))
    source_command_names = [f.stem for f in source_command_files]

    collisions = sorted(set(source_skill_names) & set(source_command_names))
    if collisions:
        for name in collisions:
            print(
                "  ERROR:    "
                f"{name} exists as both .claude/skills and .claude/commands"
            )
        return False, -1, 0, 0

    expected_names = set(source_skill_names) | set(source_command_names)
    all_synced = True
    synced_count = 0
    skipped_count = 0

    items: list[tuple[str, Path, str]] = []

    for skill_name in source_skill_names:
        source_skill = skill_source_dir / skill_name / "SKILL.md"
        source_content = source_skill.read_text()

        warnings = validate_codex_skill(source_skill)
        for warning in warnings:
            print(f"  WARNING:  {skill_name}: {warning}")

        items.append((skill_name, source_skill, source_content))

    for command_file in source_command_files:
        items.append(
            (command_file.stem, command_file, render_codex_command_skill(command_file))
        )

    for skill_name, source_path, content in items:
        target_skill_dir = target_dir / skill_name
        target_skill = target_skill_dir / "SKILL.md"
        source_rel_path = source_path.relative_to(get_repo_root())

        if target_skill.exists():
            target_content = target_skill.read_text()
            target_content_stripped = strip_auto_gen_header(target_content)
            if content == target_content_stripped and has_auto_gen_header(
                target_content
            ):
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            status = "MISMATCH" if target_skill.exists() else "MISSING"
            print(f"  {status}: {skill_name}/SKILL.md")
        else:
            target_skill_dir.mkdir(parents=True, exist_ok=True)
            content_with_header = add_auto_gen_header(content, str(source_rel_path))
            target_skill.write_text(content_with_header)
            print(f"  SYNCED:   {skill_name}/SKILL.md")
            synced_count += 1

    if target_dir.exists():
        target_skills = set(d.parent.name for d in target_dir.glob("*/SKILL.md"))
        orphaned = target_skills - expected_names

        for orphan in sorted(orphaned):
            all_synced = False
            orphan_path = target_dir / orphan
            if check_only:
                print(f"  ORPHAN:   {orphan}/")
            else:
                shutil.rmtree(orphan_path)
                print(f"  REMOVED:  {orphan}/")
    else:
        orphaned = set()

    return all_synced, synced_count, skipped_count, len(orphaned)


def sync_all(check_only: bool = False) -> bool:
    """Sync all AI tool directories.

    Args:
        check_only: If True, only check if files match (don't modify).

    Returns:
        True if all files are in sync, False otherwise.
    """
    repo_root = get_repo_root()
    all_synced = True

    # 1. Sync commands: .claude/commands/ -> .opencode/command/
    print("Commands (.claude/commands/ -> .opencode/command/):")
    synced, s, k, o = sync_flat_files(
        repo_root / ".claude" / "commands",
        repo_root / ".opencode" / "command",
        "*.md",
        "commands",
        check_only,
    )
    if s < 0 or (check_only and not synced):
        all_synced = False
    if not check_only:
        print(f"  Total: {s} synced, {k} unchanged, {o} orphans removed")
    print()

    print("Skills + command workflows (.claude/ -> .codex/skills/):")
    synced, s, k, o = sync_codex_skills(
        repo_root / ".claude" / "skills",
        repo_root / ".claude" / "commands",
        repo_root / ".codex" / "skills",
        check_only=check_only,
    )
    if s < 0 or (check_only and not synced):
        all_synced = False
    if not check_only:
        print(f"  Total: {s} synced, {k} unchanged, {o} orphans removed")
    print()

    print("Effective capability parity (Claude Code, OpenCode, Codex):")
    if not check_capability_parity(repo_root):
        all_synced = False
    print()

    print("Skill and command style:")
    if not validate_style_and_budget(repo_root):
        all_synced = False
    print()

    print("AI docs:")
    if not validate_ai_docs(repo_root):
        all_synced = False
    print()

    # Summary
    if check_only:
        if all_synced:
            print("✓ All AI tool files are in sync")
        else:
            print("✗ Files are out of sync. Run: pixi run sync-ai-commands")

    return all_synced


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Sync AI commands and skills across tool directories"
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check if files are in sync without modifying (for CI)",
    )
    args = parser.parse_args()

    success = sync_all(check_only=args.check)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
