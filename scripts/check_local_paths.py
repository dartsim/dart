#!/usr/bin/env python3
"""Reject private or machine-specific paths in repository and published text."""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

# Keep the publication policy and its narrowly scoped exceptions here.
PATH_TAIL = r"[^\s`\"'<>\[\](){};,]*"
PUBLIC_URL = re.compile(r"https?://[^\s`\"'<>\[\](){}]+", re.IGNORECASE)
PATTERNS = tuple(
    re.compile(pattern + PATH_TAIL, re.IGNORECASE)
    for pattern in (
        r"(?<![\w.-])\.sisyphus[/\\]",
        r"(?<![\w.-])\.ab[/\\]",
        r"/tmp/claude\-[^\s/\\`\"'<>]+",
        r"(?<![\w.-])\.claude[/\\]projects[/\\]",
        r"(?<![\w.-])scratchpad[/\\]",
        r"(?<![\w.-])task_\d+(?:[/\\]|-[\w-]+)",
        # A host, path or drive character before /home or /Users is not a home.
        r"(?<![\w.:-])/(?:home|Users)/[^\s/\\`\"'<>\[\](){};,|]+",
        # Unix root homes are case-sensitive; PDF /Root entries are not paths.
        r"(?<![\w.:-])/(?-i:root)(?=[/\\]|$|[\s`\"'<>\[\](){};,.:|])",
        r"(?<![\w.:-])[A-Za-z]:[/\\]+Users[/\\]+[^/\\\r\n`\"'<>\[\](){};,|]+",
    )
)
ALLOWLIST = {
    # Only file/staged scans may exempt checker fixtures; free text never does.
    "tests/test_check_local_paths.py": re.compile(r".*"),
    ".gitignore": re.compile(r"\.sisyphus[/]"),
}
HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")


def scan_line(line: str, number: int | str, filename: str | None = None) -> bool:
    allowed = ALLOWLIST.get(filename)
    if allowed and allowed.fullmatch(line):
        return False
    line = PUBLIC_URL.sub(" ", line)
    matches = sorted(
        {match.group() for pattern in PATTERNS for match in pattern.finditer(line)}
    )
    location = f"{filename}:{number}" if filename else str(number)
    for match in matches:
        print(f"{location}: {match}")
    return bool(matches)


def scan_text(text: str, filename: str | None = None) -> bool:
    found = False
    for number, line in enumerate(text.splitlines(), 1):
        found |= scan_line(line, number, filename)
    return found


def scan_commit_message(text: str) -> bool:
    found = False
    for number, line in enumerate(text.splitlines(), 1):
        if line == "# ------------------------ >8 ------------------------":
            break
        if not line.startswith("#"):
            found |= scan_line(line, number)
    return found


def git_output(root: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True
    ).stdout


def scan_staged(root: Path) -> bool:
    paths = git_output(
        root,
        "diff",
        "--cached",
        "--no-renames",
        "--name-only",
        "-z",
        "--diff-filter=ACMRT",
    )
    found = False
    for encoded in paths.split(b"\0"):
        if not encoded:
            continue
        filename = os.fsdecode(encoded)
        found |= scan_line(filename, filename)
        diff = git_output(
            root,
            "diff",
            "--cached",
            "--no-renames",
            "--no-ext-diff",
            "--no-textconv",
            "--no-color",
            # Binary blobs publish their bytes too; diff them as text.
            "--text",
            "--unified=0",
            "--",
            filename,
        ).decode("utf-8", errors="replace")
        number = None
        for line in diff.splitlines():
            hunk = HUNK.match(line)
            if hunk:
                number = int(hunk.group(1))
            elif number is not None and line.startswith("+"):
                found |= scan_line(line[1:], number, filename)
                number += 1
            elif number is not None and line.startswith(" "):
                number += 1
    return found


def scan_file(path: Path, filename: str) -> bool:
    found = scan_line(filename, filename)
    # A tracked symlink publishes its target, not the external file's contents.
    data = os.fsencode(os.readlink(path)) if path.is_symlink() else path.read_bytes()
    return scan_text(data.decode("utf-8", errors="replace"), filename) | found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--staged", action="store_true", help="scan index names and added lines"
    )
    mode.add_argument("--files", nargs="+", type=Path)
    mode.add_argument("--all-tracked", action="store_true")
    mode.add_argument(
        "--text-file", type=Path, help="scan free text without exceptions"
    )
    mode.add_argument("--stdin", action="store_true", help="scan free text from stdin")
    mode.add_argument(
        "--commit-msg-file",
        type=Path,
        help="scan commit text before comments and scissors",
    )
    args = parser.parse_args()
    try:
        if args.stdin:
            return int(scan_text(sys.stdin.read()))
        if args.text_file:
            return int(scan_text(args.text_file.read_text(encoding="utf-8")))
        if args.commit_msg_file:
            return int(
                scan_commit_message(args.commit_msg_file.read_text(encoding="utf-8"))
            )
        try:
            root = Path(
                os.fsdecode(
                    git_output(Path.cwd(), "rev-parse", "--show-toplevel")
                ).strip()
            )
        except subprocess.CalledProcessError:
            if not args.files:
                raise
            root = Path.cwd()
        if args.staged:
            return int(scan_staged(root))
        paths = args.files
        if args.all_tracked:
            paths = [
                root / os.fsdecode(path)
                for path in git_output(root, "ls-files", "-z").split(b"\0")
                if path
            ]
        found = False
        for path in paths:
            absolute = Path(os.path.abspath(path))
            filename = (
                absolute.relative_to(root).as_posix()
                if absolute.is_relative_to(root)
                else str(path)
            )
            found |= scan_file(path, filename)
        return int(found)
    except (OSError, UnicodeError, subprocess.CalledProcessError) as error:
        print(f"Local path check failed: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
