#!/usr/bin/env python3
"""Sync AI commands and skills across tool directories.

Different AI coding tools read from different directories:
- Claude Code: .claude/commands/, .claude/skills/
- OpenCode: .opencode/command/
- Codex: .agents/skills/ (domain-skill adapters plus workflow skill adapters)

This script keeps them in sync using .claude/ as the current editable source
for workflow sources and domain skills.
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
import subprocess
import sys
import tomllib
from pathlib import Path

PROFILE_READ_ERRORS = (OSError, json.JSONDecodeError, AttributeError)
PATH_RESOLUTION_ERRORS = (OSError, ValueError)

CODEX_NAME_LIMIT = 100
CODEX_DESC_LIMIT = 500
MAX_SKILL_LINES = 500
MAX_COMMAND_LINES = 200
REQUIRED_COMMAND_SECTIONS = ("Required Reading", "Workflow", "Output")
DOCS_UPDATE_REQUIRED_READING = (
    "docs/AGENTS.md",
    "docs/information-architecture.md",
)
NEW_TASK_REQUIRED_READING = ("docs/information-architecture.md",)
CAPABILITY_SCHEMA_VERSION = 1
GENERATED_SKILLS_SCHEMA_VERSION = 1
GENERATED_SKILLS_MANIFEST = ".dart-generated.json"
GENERATED_BY = "scripts/sync_ai_commands.py"
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
    content = skill_path.read_text(encoding="utf-8")
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


def list_skill_names(
    skill_dir: Path, include_names: set[str] | None = None
) -> tuple[set[str], list[str]]:
    """Return declared skill names and frontmatter/folder consistency errors."""
    if not skill_dir.exists():
        return set(), []

    names: set[str] = set()
    errors: list[str] = []

    for skill_path in sorted(skill_dir.glob("*/SKILL.md")):
        folder_name = skill_path.parent.name
        if include_names is not None and folder_name not in include_names:
            continue
        rel_path = display_path(skill_path)
        meta = parse_skill_frontmatter(skill_path.read_text(encoding="utf-8"))
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
    codex_target = repo_root / ".agents" / "skills"
    managed_codex_skills, manifest_errors, manifest_exists = (
        read_generated_skills_manifest(codex_target)
    )
    codex_skills, codex_skill_errors = list_skill_names(
        codex_target, managed_codex_skills
    )

    expected = claude_commands | shared_skills
    agent_sets = {
        "Claude Code": claude_commands | shared_skills,
        "OpenCode": opencode_commands | shared_skills,
        "Codex": codex_skills,
    }

    ok = True

    if not manifest_exists:
        ok = False
        print(
            f"  MANIFEST ERROR: {display_path(codex_target / GENERATED_SKILLS_MANIFEST)}: "
            "missing generated-skill ownership manifest"
        )
    for error in manifest_errors:
        ok = False
        print(f"  MANIFEST ERROR: {error}")
    if shared_skill_errors:
        ok = False
        print_skill_name_errors(".claude/skills", shared_skill_errors)
    if codex_skill_errors:
        ok = False
        print_skill_name_errors(".agents/skills", codex_skill_errors)

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

    codex_target = repo_root / ".agents" / "skills"
    managed_codex_skills, manifest_errors, manifest_exists = (
        read_generated_skills_manifest(codex_target)
    )
    if not manifest_exists:
        errors.append(
            f"{display_path(codex_target / GENERATED_SKILLS_MANIFEST)}: "
            "missing generated-skill ownership manifest"
        )
    errors.extend(manifest_errors)

    skill_dirs = [
        (".claude/skills", repo_root / ".claude" / "skills", None),
        (".agents/skills", codex_target, managed_codex_skills),
    ]
    command_dirs = [
        (".claude/commands", repo_root / ".claude" / "commands"),
        (".opencode/command", repo_root / ".opencode" / "command"),
    ]

    for label, skill_dir, include_names in skill_dirs:
        for skill_path in sorted(skill_dir.glob("*/SKILL.md")):
            if (
                include_names is not None
                and skill_path.parent.name not in include_names
            ):
                continue
            content = skill_path.read_text(encoding="utf-8")
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

            if label == ".agents/skills":
                for warning in validate_codex_skill(skill_path):
                    errors.append(f"{path_label}: {warning}")

    for label, command_dir in command_dirs:
        for command_path in sorted(command_dir.glob("*.md")):
            content = command_path.read_text(encoding="utf-8")
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


def collect_pixi_task_names(pixi_path: Path) -> set[str]:
    """Collect task names from every Pixi task table in the project manifest."""
    manifest = tomllib.loads(pixi_path.read_text(encoding="utf-8"))
    names: set[str] = set()

    def visit(value: object, key: str | None = None) -> None:
        if not isinstance(value, dict):
            return
        if key == "tasks":
            names.update(str(name) for name in value)
        for child_key, child_value in value.items():
            visit(child_value, str(child_key))

    visit(manifest)
    return names


def profile_skill_lines(content: str, profile: str) -> list[tuple[int, str]]:
    """Return numbered lines outside sections owned by the other branch."""
    if profile not in {"main", "release-6.20"}:
        raise ValueError(f"unknown branch profile: {profile}")

    excluded_level: int | None = None
    included: list[tuple[int, str]] = []
    excluded_marker = "dart 6" if profile == "main" else "dart 7"
    fence_char = ""
    fence_length = 0

    for line_number, line in enumerate(content.splitlines(), start=1):
        stripped = line.lstrip()
        if fence_char:
            if re.match(rf"^{re.escape(fence_char)}{{{fence_length},}}\s*$", stripped):
                fence_char = ""
                fence_length = 0
            if excluded_level is None:
                included.append((line_number, line))
            continue
        fence = re.match(r"^(`{3,}|~{3,})", stripped)
        if fence:
            fence_char = fence.group(1)[0]
            fence_length = len(fence.group(1))
            if excluded_level is None:
                included.append((line_number, line))
            continue
        heading = re.match(r"^(#{1,6})\s+(.+?)\s*$", stripped)
        if heading:
            level = len(heading.group(1))
            if excluded_level is not None and level <= excluded_level:
                excluded_level = None
            if excluded_marker in heading.group(2).casefold():
                excluded_level = level
        if excluded_level is None:
            included.append((line_number, line))

    return included


def detect_branch_profile(repo_root: Path) -> str:
    """Return the branch profile used for branch-headed skill sections."""
    profile_path = repo_root / "docs" / "ai" / "branch-profile.json"
    try:
        declared = json.loads(profile_path.read_text(encoding="utf-8")).get("profile")
    except PROFILE_READ_ERRORS:
        declared = None
    if declared in {"main", "release-6.20"}:
        return declared
    result = subprocess.run(
        ["git", "-C", str(repo_root), "branch", "--show-current"],
        capture_output=True,
        text=True,
    )
    return "release-6.20" if "release-6.20" in result.stdout else "main"


def is_repo_relative_path(repo_root: Path, candidate: str) -> bool:
    path = Path(candidate)
    if path.is_absolute() or not candidate:
        return False
    try:
        (repo_root / path).resolve().relative_to(repo_root.resolve())
    except PATH_RESOLUTION_ERRORS:
        return False
    return True


def skill_repository_reference_errors(
    repo_root: Path, profile: str | None = None
) -> list[str]:
    """Return errors for literal Pixi tasks and repo paths in source skills."""
    errors: list[str] = []
    profile = profile or detect_branch_profile(repo_root)
    pixi_path = repo_root / "pixi.toml"
    if not pixi_path.exists():
        return [f"{display_path(pixi_path)}: missing Pixi manifest"]

    try:
        pixi_tasks = collect_pixi_task_names(pixi_path)
    except tomllib.TOMLDecodeError as error:
        return [f"{display_path(pixi_path)}: invalid TOML: {error}"]

    pixi_run_pattern = re.compile(
        r"\bpixi\s+run\s+"
        r"(?:(?:--locked|--frozen)\s+|(?:-e|--environment)\s+\S+\s+)*"
        r"([A-Za-z0-9][A-Za-z0-9_.-]*)"
    )
    inline_code_pattern = re.compile(r"`([^`\n]+)`")
    repo_path_prefixes = (
        ".agents/",
        ".claude/",
        ".github/",
        "dart/",
        "docs/",
        "scripts/",
        "tests/",
    )

    for skill_path in sorted((repo_root / ".claude" / "skills").glob("*/SKILL.md")):
        for line_number, line in profile_skill_lines(
            skill_path.read_text(encoding="utf-8"), profile
        ):
            for match in pixi_run_pattern.finditer(line):
                task_name = match.group(1)
                if task_name not in pixi_tasks:
                    errors.append(
                        f"{display_path(skill_path)}:{line_number}: Pixi task "
                        f"`{task_name}` does not exist"
                    )

            for inline_value in inline_code_pattern.findall(line):
                candidate = inline_value.rstrip(".,:;")
                if not candidate.startswith(repo_path_prefixes):
                    continue
                if any(char in candidate for char in " <>*{}$[]"):
                    continue
                if not is_repo_relative_path(repo_root, candidate):
                    errors.append(
                        f"{display_path(skill_path)}:{line_number}: repo path "
                        f"`{candidate}` escapes the repository"
                    )
                elif not (repo_root / candidate).exists():
                    errors.append(
                        f"{display_path(skill_path)}:{line_number}: repo path "
                        f"`{candidate}` does not exist"
                    )

    return errors


def validate_skill_repository_references(repo_root: Path) -> bool:
    """Validate source-skill task and path references against the checkout."""
    errors = skill_repository_reference_errors(repo_root)
    if errors:
        for error in errors:
            print(f"  REFERENCE ERROR: {error}")
        return False

    print("  OK: source-skill Pixi tasks and literal repo paths exist")
    return True


def command_structure_errors(path_label: str, content: str) -> list[str]:
    """Return structural errors for one command's frontmatter and sections."""
    errors: list[str] = []

    meta = parse_command_frontmatter(content)
    if not meta.get("argument-hint"):
        errors.append(
            f"{path_label}: missing non-empty `argument-hint` frontmatter key"
        )

    positions: dict[str, int] = {}
    in_fence = False
    for index, line in enumerate(content.splitlines()):
        if line.strip().startswith("```"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        heading = re.match(r"^##\s+(.+?)\s*$", line)
        if not heading:
            continue
        title = heading.group(1).strip()
        if title in REQUIRED_COMMAND_SECTIONS and title not in positions:
            positions[title] = index

    for section in REQUIRED_COMMAND_SECTIONS:
        if section not in positions:
            errors.append(f"{path_label}: missing required `## {section}` section")

    if all(section in positions for section in REQUIRED_COMMAND_SECTIONS):
        order = [positions[section] for section in REQUIRED_COMMAND_SECTIONS]
        if order != sorted(order):
            errors.append(
                f"{path_label}: sections must appear in order "
                "## Required Reading, ## Workflow, ## Output"
            )

    return errors


def validate_command_structure(repo_root: Path) -> bool:
    """Require consistent section structure and argument hints in commands."""
    errors: list[str] = []

    good = (
        '---\nargument-hint: "<x>"\n---\n\n## Required Reading\n\n'
        "## Workflow\n\n## Output\n"
    )
    if command_structure_errors("internal command-structure self-check", good):
        errors.append(
            "internal command-structure self-check failed: valid command was flagged"
        )

    bad_order = "---\n---\n\n## Workflow\n\n## Required Reading\n\n## Output\n"
    order_errors = command_structure_errors("internal", bad_order)
    if not any("argument-hint" in error for error in order_errors):
        errors.append(
            "internal command-structure self-check failed: missing argument-hint "
            "was not detected"
        )
    if not any("order" in error for error in order_errors):
        errors.append(
            "internal command-structure self-check failed: out-of-order sections "
            "were not detected"
        )

    bad_missing = (
        '---\nargument-hint: "<x>"\n---\n\n## Required Reading\n\n## Workflow\n'
    )
    if not any(
        "## Output" in error
        for error in command_structure_errors("internal", bad_missing)
    ):
        errors.append(
            "internal command-structure self-check failed: missing Output section "
            "was not detected"
        )

    for command_path in sorted((repo_root / ".claude" / "commands").glob("*.md")):
        errors.extend(
            command_structure_errors(
                display_path(command_path), command_path.read_text(encoding="utf-8")
            )
        )

    if errors:
        for error in errors:
            print(f"  STRUCTURE ERROR: {error}")
        return False

    print(
        "  OK: commands declare argument hints and the Required Reading, "
        "Workflow, and Output sections in order"
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
    return extract_required_reading_from_content(
        command_path.read_text(encoding="utf-8")
    )


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


def docs_update_required_reading_errors(command_path: Path) -> list[str]:
    """Return errors when the docs workflow can bypass placement policy."""
    required_reading = set(extract_required_reading(command_path))
    return [
        f"{display_path(command_path)}: dart-docs-update must require `{required}`"
        for required in DOCS_UPDATE_REQUIRED_READING
        if required not in required_reading
    ]


def new_task_required_reading_errors(command_path: Path) -> list[str]:
    """Return errors when task cleanup can bypass placement policy."""
    required_reading = set(extract_required_reading(command_path))
    return [
        f"{display_path(command_path)}: dart-new-task must require `{required}`"
        for required in NEW_TASK_REQUIRED_READING
        if required not in required_reading
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
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
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
        r"\bmerg(?:e|es|ing)\b[^\n;|&]{0,80}\b(?:PRs?|pull requests?)\b",
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
        "merge a ready PR": "PR merge prose",
        "merging any bot PR": "bot PR merge prose",
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
        scan_lines(display_path(path), path.read_text(encoding="utf-8").splitlines())

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
        r"resolve (?:all|every) unresolved thread",
        ai_tools.read_text(encoding="utf-8"),
        re.I,
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
        docs_dir / "terminology.md",
        docs_dir / "capabilities.json",
    ]

    for path in required_files:
        if not path.exists():
            errors.append(f"{display_path(path)}: missing required AI doc")

    agents_content = (repo_root / "AGENTS.md").read_text(encoding="utf-8")
    docs_readme_content = (repo_root / "docs" / "README.md").read_text(encoding="utf-8")
    if "docs/ai/README.md" not in agents_content:
        errors.append("AGENTS.md: missing docs/ai/README.md pointer")
    if "docs/ai/principles.md" not in agents_content:
        errors.append("AGENTS.md: missing docs/ai/principles.md pointer")
    if "docs/ai/terminology.md" not in agents_content:
        errors.append("AGENTS.md: missing docs/ai/terminology.md pointer")
    if "ai/README.md" not in docs_readme_content:
        errors.append("docs/README.md: missing ai/README.md index link")

    workflows_path = docs_dir / "workflows.md"
    if workflows_path.exists():
        workflow_content = workflows_path.read_text(encoding="utf-8")
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

            command_required_reading = extract_required_reading(command_path)
            errors.extend(required_reading_path_errors(repo_root, command_path))
            if name == "dart-docs-update":
                errors.extend(docs_update_required_reading_errors(command_path))
            if name == "dart-new-task":
                errors.extend(new_task_required_reading_errors(command_path))
            errors.extend(
                missing_required_reading_errors(
                    display_path(workflows_path),
                    name,
                    public_path,
                    command_required_reading,
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
            "dart-benchmark-packet",
            "dart-branch-cleanup",
            "dart-changelog",
            "dart-close-issue",
            "dart-deps",
            "dart-docs-update",
            "dart-downstream-fix",
            "dart-fix-ci",
            "dart-fix-issue",
            "dart-manage-pr",
            "dart-mechanical-refactor",
            "dart-new-task",
            "dart-next",
            "dart-pr",
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
        if ".agents/skills/" not in agents_content:
            errors.append("AGENTS.md: missing .agents/skills/ generated pointer")

    errors.extend(validate_approval_boundary(repo_root))

    private_pattern = re.compile(r"(?:/home/|/Users/|~/|fbsource|arvr/libraries)")
    for path in docs_dir.glob("*.md"):
        for line_number, line in enumerate(
            path.read_text(encoding="utf-8").splitlines(), start=1
        ):
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
    lines = content.splitlines()
    header_start = next(
        (
            index
            for index, line in enumerate(lines)
            if line.startswith("<!-- AUTO-GENERATED FILE")
        ),
        None,
    )
    if header_start is None:
        return content

    header_end = header_start
    while header_end < len(lines) and lines[header_end].startswith("<!--"):
        header_end += 1

    before = lines[:header_start]
    after = lines[header_end:]
    while before and not before[-1]:
        before.pop()
    while after and not after[0]:
        after.pop(0)

    sections = ["\n".join(section) for section in (before, after) if section]
    result = "\n\n".join(sections)
    if content.endswith("\n"):
        result += "\n"
    return result


def add_auto_gen_header(content: str, source_path: str) -> str:
    """Add auto-generated header after YAML frontmatter (if present) to preserve tool parsing."""
    header = AUTO_GEN_HEADER.format(source_path=source_path).rstrip("\n")

    if content.startswith("---"):
        lines = content.split("\n")
        for i, line in enumerate(lines[1:], start=1):
            if line.strip() == "---":
                frontmatter = "\n".join(lines[: i + 1])
                rest = "\n".join(lines[i + 1 :]).lstrip("\n")
                return frontmatter + "\n\n" + header + "\n\n" + rest
    return header + "\n\n" + content.lstrip("\n")


def generated_target_root_error(target_dir: Path) -> str | None:
    """Return an error when a generated root escapes the repository via symlinks."""
    if target_dir.exists() and not target_dir.is_dir():
        return f"{display_path(target_dir)}: generated target must be a directory"
    repo_root = get_repo_root().resolve()
    try:
        target_dir.resolve().relative_to(repo_root)
    except PATH_RESOLUTION_ERRORS:
        return f"{display_path(target_dir)}: generated target escapes repository"
    current = target_dir
    while current != repo_root:
        if current.is_symlink():
            return f"{display_path(current)}: generated target must not be a symlink"
        if current.parent == current:
            break
        current = current.parent
    return None


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

    target_error = generated_target_root_error(target_dir)
    if target_error:
        print(f"  ERROR:    {target_error}")
        return False, -1, 0, 0

    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_files = sorted(source_dir.glob(pattern))
    target_files = sorted(target_dir.glob(pattern)) if target_dir.exists() else []
    unsafe_targets = [path for path in target_files if path.is_symlink()]
    if unsafe_targets:
        for path in unsafe_targets:
            print(f"  ERROR:    {display_path(path)}: generated file is a symlink")
        return False, -1, 0, 0
    all_synced = True
    synced_count = 0
    skipped_count = 0

    for source_file in source_files:
        target_file = target_dir / source_file.name
        source_content = source_file.read_text(encoding="utf-8")
        source_rel_path = source_file.relative_to(get_repo_root())

        if target_file.exists():
            target_content = target_file.read_text(encoding="utf-8")
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
            target_file.write_text(content_with_header, encoding="utf-8")
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
    command_content = command_path.read_text(encoding="utf-8")
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
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/{command_name} <arguments>`
- Codex: `${command_name} <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

{command_body}
"""


def is_generated_skill_adapter(content: str) -> bool:
    """Return whether content carries this generator's ownership markers."""
    return has_auto_gen_header(content) and (
        f"<!-- Sync script: {GENERATED_BY} -->" in content
    )


def generated_skills_manifest_content(managed_skills: set[str]) -> str:
    """Render the ownership manifest for generated skill directories."""
    manifest = {
        "schema_version": GENERATED_SKILLS_SCHEMA_VERSION,
        "generator": GENERATED_BY,
        "paths": [f"{name}/SKILL.md" for name in sorted(managed_skills)],
    }
    return json.dumps(manifest, indent=2) + "\n"


def read_generated_skills_manifest(
    target_dir: Path,
) -> tuple[set[str], list[str], bool]:
    """Read and validate generated-skill ownership metadata."""
    manifest_path = target_dir / GENERATED_SKILLS_MANIFEST
    if manifest_path.is_symlink():
        return (
            set(),
            [f"{display_path(manifest_path)}: manifest must not be a symlink"],
            True,
        )
    if not manifest_path.exists():
        return set(), [], False

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        return (
            set(),
            [f"{display_path(manifest_path)}:{error.lineno}: invalid JSON"],
            True,
        )

    if not isinstance(manifest, dict):
        return (
            set(),
            [f"{display_path(manifest_path)}: manifest must be an object"],
            True,
        )

    errors: list[str] = []
    if (
        type(manifest.get("schema_version")) is not int
        or manifest["schema_version"] != GENERATED_SKILLS_SCHEMA_VERSION
    ):
        errors.append(
            f"{display_path(manifest_path)}: schema_version must be "
            f"{GENERATED_SKILLS_SCHEMA_VERSION}"
        )
    legacy_schema = set(manifest) == {
        "schema_version",
        "generated_by",
        "managed_skills",
    }
    if not legacy_schema and set(manifest) != {"schema_version", "generator", "paths"}:
        errors.append(
            f"{display_path(manifest_path)}: keys must equal generator, paths, "
            "schema_version"
        )
    generator = manifest.get("generated_by" if legacy_schema else "generator")
    if generator != GENERATED_BY:
        errors.append(
            f"{display_path(manifest_path)}: generator must be `{GENERATED_BY}`"
        )

    declared = manifest.get("managed_skills" if legacy_schema else "paths")
    if not isinstance(declared, list):
        errors.append(f"{display_path(manifest_path)}: paths must be a list")
        return set(), errors, True
    if legacy_schema:
        declared = [
            f"{name}/SKILL.md" if isinstance(name, str) else name for name in declared
        ]

    managed: set[str] = set()
    for index, path in enumerate(declared):
        match = (
            re.fullmatch(r"([a-z0-9][a-z0-9-]*)/SKILL\.md", path)
            if isinstance(path, str)
            else None
        )
        if not match:
            errors.append(
                f"{display_path(manifest_path)}: paths[{index}] must be a safe "
                "lowercase `<skill>/SKILL.md` path"
            )
            continue
        name = match.group(1)
        if name in managed:
            errors.append(
                f"{display_path(manifest_path)}: duplicate generated path `{path}`"
            )
        managed.add(name)

    return managed, errors, True


def cleanup_legacy_codex_skills(
    legacy_dir: Path,
    canonical_names: set[str],
    check_only: bool = False,
) -> tuple[bool, int, int]:
    """Remove generated legacy adapters and report unowned name collisions."""
    if legacy_dir.is_symlink():
        print("  CONFLICT: legacy skill root is a symlink; preserved without reading")
        return False, 0, 1
    if legacy_dir.exists() and not legacy_dir.is_dir():
        print("  CONFLICT: legacy skill root is not a directory; preserved")
        return False, 0, 1
    generated_paths: list[Path] = []
    conflicting_paths: list[Path] = []
    unsafe_paths: list[Path] = []
    if legacy_dir.exists():
        unsafe_paths.extend(
            child / "SKILL.md"
            for child in sorted(legacy_dir.iterdir())
            if child.is_symlink()
        )
        for path in sorted(legacy_dir.glob("*/SKILL.md")):
            if path.is_symlink() or path.parent.is_symlink():
                if path not in unsafe_paths:
                    unsafe_paths.append(path)
                continue
            if is_generated_skill_adapter(path.read_text(encoding="utf-8")):
                generated_paths.append(path)
            elif path.parent.name in canonical_names:
                conflicting_paths.append(path)

    for skill_path in conflicting_paths:
        rel_path = f"{skill_path.parent.name}/SKILL.md"
        print(
            f"  CONFLICT: {rel_path} is unowned and collides with the canonical "
            ".agents/skills catalog; preserved"
        )

    for skill_path in unsafe_paths:
        rel_path = f"{skill_path.parent.name}/SKILL.md"
        print(f"  CONFLICT: {rel_path} is a symlink; preserved without reading")

    for skill_path in generated_paths:
        rel_path = f"{skill_path.parent.name}/SKILL.md"
        if check_only:
            print(f"  LEGACY:   {rel_path}")
            continue
        skill_path.unlink()
        try:
            skill_path.parent.rmdir()
        except OSError:
            # Cleanup is best effort; the directory may be non-empty or raced.
            pass
        print(f"  REMOVED:  {rel_path}")

    if not check_only:
        try:
            legacy_dir.rmdir()
        except OSError:
            # Cleanup is best effort; the directory may be non-empty or raced.
            pass

    clean = not generated_paths and not conflicting_paths and not unsafe_paths
    return clean, len(generated_paths), len(conflicting_paths) + len(unsafe_paths)


def sync_codex_skills(
    skill_source_dir: Path,
    command_source_dir: Path,
    target_dir: Path,
    check_only: bool = False,
) -> tuple[bool, int, int, int]:
    """Sync domain skills and workflow adapters without deleting user skills."""
    if not skill_source_dir.exists():
        print(f"Source directory not found: {skill_source_dir}")
        return False, -1, 0, 0
    if not command_source_dir.exists():
        print(f"Source directory not found: {command_source_dir}")
        return False, -1, 0, 0

    target_error = generated_target_root_error(target_dir)
    if target_error:
        print(f"  ERROR:    {target_error}")
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
    owned_names, manifest_errors, manifest_exists = read_generated_skills_manifest(
        target_dir
    )
    if manifest_errors:
        for error in manifest_errors:
            print(f"  ERROR:    {error}")
        return False, -1, 0, 0
    unsafe_owned_dirs = [
        target_dir / name
        for name in sorted(expected_names | owned_names)
        if (target_dir / name).is_symlink()
    ]
    if unsafe_owned_dirs:
        for path in unsafe_owned_dirs:
            print(
                f"  ERROR:    {display_path(path)}: generated skill directory is a symlink"
            )
        return False, -1, 0, 0
    unsafe_owned_files = [
        target_dir / name / "SKILL.md"
        for name in sorted(expected_names | owned_names)
        if (target_dir / name / "SKILL.md").is_symlink()
    ]
    if unsafe_owned_files:
        for path in unsafe_owned_files:
            print(
                f"  ERROR:    {display_path(path)}: generated skill file is a symlink"
            )
        return False, -1, 0, 0

    all_synced = True
    synced_count = 0
    skipped_count = 0

    items: list[tuple[str, Path, str]] = []

    for skill_name in source_skill_names:
        source_skill = skill_source_dir / skill_name / "SKILL.md"
        source_content = source_skill.read_text(encoding="utf-8")

        warnings = validate_codex_skill(source_skill)
        for warning in warnings:
            print(f"  WARNING:  {skill_name}: {warning}")

        items.append((skill_name, source_skill, source_content))

    for command_file in source_command_files:
        items.append(
            (command_file.stem, command_file, render_codex_command_skill(command_file))
        )

    marker_owned_names = {
        path.parent.name
        for path in target_dir.glob("*/SKILL.md")
        if not path.parent.is_symlink()
        and not path.is_symlink()
        and is_generated_skill_adapter(path.read_text(encoding="utf-8"))
    }

    # Refuse to overwrite or delete entries that do not carry this generator's
    # marker. Marker-owned leftovers are safe to remove even if a damaged or
    # older manifest omitted their names; unrelated unowned skills are kept.
    conflicts: list[str] = []
    for skill_name, _, _ in items:
        target_skill_dir = target_dir / skill_name
        target_skill = target_skill_dir / "SKILL.md"
        if target_skill.is_symlink():
            conflicts.append(
                f"{display_path(target_skill)}: generated skill file is a symlink"
            )
            continue
        if target_skill.exists():
            if not is_generated_skill_adapter(target_skill.read_text(encoding="utf-8")):
                conflicts.append(
                    f"{display_path(target_skill)}: refusing to overwrite "
                    "an unowned skill"
                )
        elif (
            target_skill_dir.exists()
            and any(target_skill_dir.iterdir())
            and skill_name not in owned_names
        ):
            conflicts.append(
                f"{display_path(target_skill_dir)}: refusing to add a generated "
                "skill inside a non-empty unowned directory"
            )

    orphaned = (owned_names | marker_owned_names) - expected_names
    for orphan in sorted(orphaned):
        orphan_skill = target_dir / orphan / "SKILL.md"
        if orphan_skill.exists() and not is_generated_skill_adapter(
            orphan_skill.read_text(encoding="utf-8")
        ):
            conflicts.append(
                f"{display_path(orphan_skill)}: ownership manifest names this "
                "orphan, but its generator marker is missing; refusing to delete"
            )

    if conflicts:
        for error in conflicts:
            print(f"  ERROR:    {error}")
        return False, -1, 0, len(orphaned)

    for skill_name, source_path, content in items:
        target_skill_dir = target_dir / skill_name
        target_skill = target_skill_dir / "SKILL.md"
        source_rel_path = source_path.relative_to(get_repo_root())
        content_with_header = add_auto_gen_header(content, str(source_rel_path))

        if target_skill.exists():
            target_content = target_skill.read_text(encoding="utf-8")
            if target_content == content_with_header:
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            status = "MISMATCH" if target_skill.exists() else "MISSING"
            print(f"  {status}: {skill_name}/SKILL.md")
        else:
            target_skill_dir.mkdir(parents=True, exist_ok=True)
            target_skill.write_text(content_with_header, encoding="utf-8")
            print(f"  SYNCED:   {skill_name}/SKILL.md")
            synced_count += 1

    for orphan in sorted(orphaned):
        all_synced = False
        orphan_path = target_dir / orphan
        orphan_skill = orphan_path / "SKILL.md"
        if check_only:
            print(f"  ORPHAN:   {orphan}/SKILL.md")
            continue
        if orphan_skill.exists():
            orphan_skill.unlink()
        try:
            orphan_path.rmdir()
        except OSError:
            # Cleanup is best effort; the directory may be non-empty or raced.
            pass
        print(f"  REMOVED:  {orphan}/SKILL.md")

    manifest_path = target_dir / GENERATED_SKILLS_MANIFEST
    expected_manifest = generated_skills_manifest_content(expected_names)
    current_manifest = (
        manifest_path.read_text(encoding="utf-8") if manifest_exists else None
    )
    if current_manifest != expected_manifest:
        all_synced = False
        if check_only:
            status = "MISMATCH" if manifest_exists else "MISSING"
            print(f"  {status}: {GENERATED_SKILLS_MANIFEST}")
        else:
            manifest_path.write_text(expected_manifest, encoding="utf-8")
            print(f"  SYNCED:   {GENERATED_SKILLS_MANIFEST}")

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

    print("Domain skills + workflow adapters (.claude/ -> .agents/skills/):")
    synced, s, k, o = sync_codex_skills(
        repo_root / ".claude" / "skills",
        repo_root / ".claude" / "commands",
        repo_root / ".agents" / "skills",
        check_only=check_only,
    )
    if s < 0 or (check_only and not synced):
        all_synced = False
    codex_sync_preflight_ok = s >= 0
    if not check_only and codex_sync_preflight_ok:
        print(f"  Total: {s} synced, {k} unchanged, {o} orphans removed")
    elif not check_only:
        print("  Total: canonical skill sync failed")
    print()

    print("Legacy generated Codex adapters (.codex/skills/):")
    if codex_sync_preflight_ok:
        canonical_names = list_command_names(repo_root / ".claude" / "commands")
        canonical_names.update(list_skill_names(repo_root / ".claude" / "skills")[0])
        legacy_clean, legacy_count, legacy_conflicts = cleanup_legacy_codex_skills(
            repo_root / ".codex" / "skills",
            canonical_names,
            check_only=check_only,
        )
        if legacy_conflicts:
            all_synced = False
        if not legacy_clean:
            if check_only:
                all_synced = False
            else:
                print(f"  Total: {legacy_count} generated adapters removed")
                if legacy_conflicts:
                    print(
                        f"  Total: {legacy_conflicts} unowned name conflicts preserved"
                    )
        elif not check_only:
            print("  Total: no generated adapters remain")
    else:
        print("  SKIPPED: canonical skill sync failed; legacy catalog preserved")
    print()

    print("Effective capability parity (Claude Code, OpenCode, Codex):")
    if not check_capability_parity(repo_root):
        all_synced = False
    print()

    print("Skill and command style:")
    if not validate_style_and_budget(repo_root):
        all_synced = False
    print()

    print("Source-skill repository references:")
    if not validate_skill_repository_references(repo_root):
        all_synced = False
    print()

    print("Command structure:")
    if not validate_command_structure(repo_root):
        all_synced = False
    print()

    print("AI docs:")
    if not validate_ai_docs(repo_root):
        all_synced = False
    print()

    # Summary
    if check_only:
        if all_synced:
            print("OK: All AI adapter entrypoints are in sync")
        else:
            print("ERROR: Files are out of sync. Run: pixi run sync-ai-commands")

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
