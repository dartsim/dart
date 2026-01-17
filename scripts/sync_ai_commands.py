#!/usr/bin/env python3
"""Sync AI commands between .claude/commands/ and .opencode/command/.

Claude Code and OpenCode read commands from different directories.
This script ensures they stay in sync.

Usage:
    python scripts/sync_ai_commands.py        # Sync and report
    python scripts/sync_ai_commands.py --check  # Check only (CI mode)
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path


def get_repo_root() -> Path:
    """Get the repository root directory."""
    return Path(__file__).parent.parent


def sync_commands(check_only: bool = False) -> bool:
    """Sync commands from .claude/commands/ to .opencode/command/.

    Args:
        check_only: If True, only check if files match (don't modify).

    Returns:
        True if all files are in sync, False otherwise.
    """
    repo_root = get_repo_root()
    source_dir = repo_root / ".claude" / "commands"
    target_dir = repo_root / ".opencode" / "command"

    if not source_dir.exists():
        print(f"Source directory not found: {source_dir}")
        return False

    # Ensure target directory exists
    if not check_only:
        target_dir.mkdir(parents=True, exist_ok=True)

    source_files = sorted(source_dir.glob("*.md"))
    all_synced = True
    synced_count = 0
    skipped_count = 0

    for source_file in source_files:
        target_file = target_dir / source_file.name

        # Check if files match
        if target_file.exists():
            source_content = source_file.read_text()
            target_content = target_file.read_text()
            if source_content == target_content:
                skipped_count += 1
                continue

        all_synced = False

        if check_only:
            if target_file.exists():
                print(f"MISMATCH: {source_file.name}")
            else:
                print(f"MISSING:  {source_file.name} (not in .opencode/command/)")
        else:
            shutil.copy2(source_file, target_file)
            print(f"SYNCED:   {source_file.name}")
            synced_count += 1

    # Check for orphaned files in target
    target_files = set(f.name for f in target_dir.glob("*.md"))
    source_names = set(f.name for f in source_files)
    orphaned = target_files - source_names

    for orphan in sorted(orphaned):
        all_synced = False
        orphan_path = target_dir / orphan
        if check_only:
            print(f"ORPHAN:   {orphan} (not in .claude/commands/)")
        else:
            orphan_path.unlink()
            print(f"REMOVED:  {orphan}")

    # Summary
    if check_only:
        if all_synced:
            print(f"\n✓ All {len(source_files)} commands are in sync")
        else:
            print(f"\n✗ Commands are out of sync. Run: pixi run sync-ai-commands")
    else:
        print(
            f"\nSynced: {synced_count}, Skipped: {skipped_count}, Orphans removed: {len(orphaned)}"
        )

    return all_synced


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Sync AI commands between Claude Code and OpenCode directories"
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check if commands are in sync without modifying files (for CI)",
    )
    args = parser.parse_args()

    success = sync_commands(check_only=args.check)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
