#!/usr/bin/env python3
"""Sync AI commands and skills across tool directories.

Different AI coding tools read from different directories:
- Claude Code: .claude/commands/, .claude/skills/
- OpenCode: .opencode/command/
- Codex: .codex/skills/

This script keeps them in sync using .claude/ as the source of truth.

Usage:
    python scripts/sync_ai_commands.py          # Sync and report
    python scripts/sync_ai_commands.py --check  # Check only (CI mode)
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path

CODEX_NAME_LIMIT = 100
CODEX_DESC_LIMIT = 500


def get_repo_root() -> Path:
    return Path(__file__).parent.parent


def parse_skill_frontmatter(content: str) -> dict[str, str]:
    """Extract name and description from YAML frontmatter."""
    match = re.match(r"^---\s*\n(.*?)\n---", content, re.DOTALL)
    if not match:
        return {}

    frontmatter = match.group(1)
    result = {}

    for key in ["name", "description"]:
        pattern = rf"^{key}:\s*(.+)$"
        m = re.search(pattern, frontmatter, re.MULTILINE)
        if m:
            result[key] = m.group(1).strip()

    return result


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
        return False, 0, 0, 0

    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_files = sorted(source_dir.glob(pattern))
    all_synced = True
    synced_count = 0
    skipped_count = 0

    for source_file in source_files:
        target_file = target_dir / source_file.name

        if target_file.exists():
            source_content = source_file.read_text()
            target_content = target_file.read_text()
            if source_content == target_content:
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            status = "MISMATCH" if target_file.exists() else "MISSING"
            print(f"  {status}: {source_file.name}")
        else:
            shutil.copy2(source_file, target_file)
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


def sync_skills(
    source_dir: Path,
    target_dir: Path,
    target_tool: str = "codex",
    check_only: bool = False,
) -> tuple[bool, int, int, int]:
    """Sync skill directories (each skill is a folder with SKILL.md).

    Returns:
        Tuple of (all_synced, synced_count, skipped_count, orphan_count)
    """
    if not source_dir.exists():
        print(f"Source directory not found: {source_dir}")
        return False, 0, 0, 0

    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_skills = sorted(d.parent.name for d in source_dir.glob("*/SKILL.md"))
    all_synced = True
    synced_count = 0
    skipped_count = 0

    for skill_name in source_skills:
        source_skill = source_dir / skill_name / "SKILL.md"
        target_skill_dir = target_dir / skill_name
        target_skill = target_skill_dir / "SKILL.md"

        if target_tool == "codex":
            warnings = validate_codex_skill(source_skill)
            for w in warnings:
                print(f"  WARNING:  {skill_name}: {w}")

        if target_skill.exists():
            source_content = source_skill.read_text()
            target_content = target_skill.read_text()
            if source_content == target_content:
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            status = "MISMATCH" if target_skill.exists() else "MISSING"
            print(f"  {status}: {skill_name}/SKILL.md")
        else:
            target_skill_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source_skill, target_skill)
            print(f"  SYNCED:   {skill_name}/SKILL.md")
            synced_count += 1

    # Check for orphaned skills
    if target_dir.exists():
        target_skills = set(d.parent.name for d in target_dir.glob("*/SKILL.md"))
        orphaned = target_skills - set(source_skills)

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
    if not synced:
        all_synced = False
    if not check_only:
        print(f"  Total: {s} synced, {k} unchanged, {o} orphans removed")
    print()

    print("Skills (.claude/skills/ -> .codex/skills/):")
    synced, s, k, o = sync_skills(
        repo_root / ".claude" / "skills",
        repo_root / ".codex" / "skills",
        target_tool="codex",
        check_only=check_only,
    )
    if not synced:
        all_synced = False
    if not check_only:
        print(f"  Total: {s} synced, {k} unchanged, {o} orphans removed")
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
