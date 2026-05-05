#!/usr/bin/env python3
"""Sync AI commands and skills across tool directories.

Different AI coding tools read from different directories:
- Claude Code: .claude/commands/, .claude/skills/
- OpenCode: .opencode/command/
- Codex: .codex/skills/ (domain skills plus command-derived workflow skills)

This script keeps them in sync using .claude/ as the source of truth.
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

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/{command_name}`.

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
