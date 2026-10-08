#!/usr/bin/env python3
"""Reject private or machine-specific paths in repository and published text.

These checks catch accidental publication by contributors and agents. Local
hooks and the agent guard are conveniences that explicit bypasses can skip.
The PR Text workflow checks the title, body and every PR commit message using
the base branch's checker as the backstop before merge.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import re
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlsplit

# Keep the publication policy and its narrowly scoped exceptions here.
PATH_TAIL = r"[^\s`\"'<>\[\](){};,]*"
PUBLIC_URL = re.compile(
    r"https?://(?:[^\s`\"'<>\[\](){}/@]+@)?"
    r"(?:\[[^\s`\"'<>\[\](){}]+\])?[^\s`\"'<>\[\](){}]*",
    re.IGNORECASE,
)
PATTERNS = tuple(
    re.compile(pattern + PATH_TAIL, re.IGNORECASE)
    for pattern in (
        r"(?<![\w.-])\.sisyphus[/\\]",
        r"(?<![\w.-])\.ab[/\\]",
        r"/tmp/claude\-[^\s/\\`\"'<>]+",
        r"(?<![\w.-])\.claude[/\\]projects[/\\]",
        r"(?<![\w.-])scratchpad[/\\]",
        r"(?<![\w.-])task_\d+(?:[/\\]|-[\w-]+)",
        # Generic home-relative paths such as ~/.config name no user or machine.
        # Private agent project dirs, scratchpads and numbered worktrees still
        # match above, so ~/ alone is not reported.
        # A host, path or drive character before /home or /Users is not a home.
        r"(?<![\w.:-])/(?:home|Users|mnt/[A-Za-z]/Users)/[^\s/\\`\"'<>\[\](){};,|]+",
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
UTF16_BOMS = (b"\xff\xfe", b"\xfe\xff")


def is_public_host(host: str | None) -> bool:
    host = (host or "").rstrip(".")
    try:
        return ipaddress.ip_address(host).is_global
    except ValueError:
        return "." in host and host != "localhost" and not host.endswith(".localhost")


def scan_line(line: str, number: int | str, filename: str | None = None) -> bool:
    allowed = ALLOWLIST.get(filename)
    if allowed and allowed.fullmatch(line):
        return False
    url_paths = []

    def mask_public_url(match: re.Match[str]) -> str:
        try:
            url = urlsplit(match.group())
            if is_public_host(url.hostname):
                return " "
        except ValueError:
            return match.group()
        # Scan local URL paths at their root without relaxing home lookbehinds.
        url_paths.append(url.path)
        return match.group()

    line = PUBLIC_URL.sub(mask_public_url, line)
    matches = sorted(
        {
            match.group()
            for text in (line, *url_paths)
            for pattern in PATTERNS
            for match in pattern.finditer(text)
        }
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
    # The scissors stop matches git commit -v. Typing a literal scissors line
    # is deliberate; the PR Text backstop still scans the entire message.
    found = False
    for number, line in enumerate(text.splitlines(), 1):
        if line == "# ------------------------ >8 ------------------------":
            break
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
        "--raw",
        "-z",
        "--diff-filter=ACMRT",
    )
    found = False
    entries = paths.split(b"\0")
    for metadata, encoded in zip(entries[::2], entries[1::2]):
        if not encoded:
            continue
        filename = os.fsdecode(encoded)
        found |= scan_line(filename, filename)
        # Gitlinks publish a commit ID, not a file blob.
        if metadata.split()[1] == b"160000":
            continue
        data = git_output(root, "show", f":{filename}")
        if data.startswith(UTF16_BOMS):
            found |= scan_text(data.decode("utf-16", errors="replace"), filename)
            continue
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
    # A submodule gitlink publishes only its name and commit, not this checkout.
    if path.is_dir() and not path.is_symlink():
        return found
    # A tracked symlink publishes its target, not the external file's contents.
    data = os.fsencode(os.readlink(path)) if path.is_symlink() else path.read_bytes()
    encoding = "utf-16" if data.startswith(UTF16_BOMS) else "utf-8"
    return scan_text(data.decode(encoding, errors="replace"), filename) | found


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
        help="scan every commit line before Git scissors",
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
