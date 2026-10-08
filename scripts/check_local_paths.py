#!/usr/bin/env python3
"""Reject private or machine-specific paths in repository and published text.

These checks catch accidental publication by contributors and agents. Local
hooks and the agent guard are conveniences that explicit bypasses can skip.
The PR Text workflow checks the title, body, messages and changes of every PR
commit using the base branch's checker as the backstop before merge.
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
URI = re.compile(
    r"[a-z][a-z0-9+.-]*://(?:[^\s`\"'<>\[\](){}/@]+@)?"
    r"(?:\[[^\s`\"'<>\[\](){}]+\])?"
    r"(?:[^\s`\"'<>\[\](){}]|\([^\s`\"'<>\[\](){}]*\))*",
    re.IGNORECASE,
)
# Covers private planning/harness dirs, agent scratch/project dirs, numbered
# worktrees, Linux/macOS/root homes, Windows drive/WSL/Git Bash profiles and
# local URLs. Other representations are out of scope unless contributors' tools
# produce them; the PR Text workflow remains the backstop.
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
        # A host or path character before /home or /Users is not a home, and a
        # drive letter (C:/Users) is left to the Windows pattern below; other
        # labels such as cwd: or file: still precede a reported home.
        r"(?:(?<=\.\.)|(?<![\w.-])(?<!\b[A-Za-z]:))/(?:home|Users|(?:mnt/)?[A-Za-z]/Users)/[^\s/\\`\"'<>\[\](){};,|]+",
        # Unix root homes are case-sensitive; PDF /Root entries are not paths.
        r"(?<![\w.-])(?<!\b[A-Za-z]:)/(?-i:root)(?=[/\\]|$|[\s`\"'<>\[\](){};,.:|])",
        r"(?<![\w.:-])[A-Za-z]:[/\\]+Users[/\\]+[^/\\\r\n`\"'<>\[\](){};,|]+",
        r"\\\\[^\\/\s]+\\Users\\[^\\/\r\n`\"'<>\[\](){};,|]+",
    )
)
ALLOWLIST = {
    # Only file/staged scans may exempt checker fixtures; free text never does.
    "tests/test_check_local_paths.py": re.compile(
        r".*(?:\bexample\b|# path-fixture).*", re.IGNORECASE
    ),
    ".gitignore": re.compile(r"\.sisyphus[/]"),
}
HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")
UTF32_BOMS = (b"\xff\xfe\x00\x00", b"\x00\x00\xfe\xff")
UTF16_BOMS = (b"\xff\xfe", b"\xfe\xff")


def decode(data: bytes) -> str:
    # UTF-32LE starts with the UTF-16LE mark, so test the longer marks first.
    if data.startswith(UTF32_BOMS):
        return data.decode("utf-32", errors="replace")
    if data.startswith(UTF16_BOMS):
        return data.decode("utf-16", errors="replace")
    return data.decode("utf-8", errors="replace")


SCISSORS = re.compile(r"\S -{24} >8 -{24}")
GIT_STATUS_LINE = re.compile(
    r"\S\t(?P<status>new file|modified|deleted|renamed|copied|typechange|both \w+|"
    r"added by \w+|deleted by \w+):\s+(?P<paths>\S.*)"
)


def is_public_host(host: str | None) -> bool:
    host = (host or "").rstrip(".")
    try:
        address = ipaddress.ip_address(host)
        # Python counts some multicast ranges (SSDP, link-local) as global.
        return address.is_global and not address.is_multicast
    except ValueError:
        # Shorthand numeric hosts such as 127.1 still reach local addresses.
        if re.fullmatch(r"[0-9.]+|0x[0-9a-f.x]+", host, re.IGNORECASE):
            return False
        # Special-use DNS suffixes identify local machines and private networks.
        return "." in host and not host.endswith(
            (".localhost", ".local", ".home.arpa", ".internal", ".lan", ".localdomain")
        )


def scan_line(
    line: str,
    number: int | str,
    filename: str | None = None,
    commit: str | None = None,
) -> bool:
    allowed = ALLOWLIST.get(filename)
    if allowed and allowed.fullmatch(line):
        return False
    url_paths = []

    def mask_public_url(match: re.Match[str]) -> str:
        try:
            url = urlsplit(match.group())
            if url.scheme in {"http", "https"} and is_public_host(url.hostname):
                return " "
        except ValueError:
            return match.group()
        # Scan local URI paths at their root without relaxing home lookbehinds.
        url_paths.append(url.path)
        return match.group()

    line = URI.sub(mask_public_url, line)
    matches = sorted(
        {
            match.group()
            for text in (line, *url_paths)
            for pattern in PATTERNS
            for match in pattern.finditer(text)
        }
    )
    location = f"{filename}:{number}" if filename else str(number)
    if commit:
        location = f"{commit}:{location}"
    for match in matches:
        print(f"{location}: {match}")
    return bool(matches)


def scan_text(
    text: str, filename: str | None = None, commit: str | None = None
) -> bool:
    found = False
    for number, line in enumerate(text.splitlines(), 1):
        found |= scan_line(line, number, filename, commit)
    return found


def scan_commit_message(text: str) -> bool:
    # The scissors stop matches git commit -v. Typing a literal scissors line
    # is deliberate; the PR Text backstop still scans the entire message.
    found = False
    staged = None
    for number, line in enumerate(text.splitlines(), 1):
        # Git prefixes the scissors with core.commentChar, which may differ.
        if SCISSORS.fullmatch(line):
            break
        # Exempt only actual staged entries, not literal status-shaped messages.
        status = GIT_STATUS_LINE.fullmatch(line)
        if status:
            if staged is None:
                root = Path(
                    os.fsdecode(
                        git_output(Path.cwd(), "rev-parse", "--show-toplevel")
                    ).strip()
                )
                staged = staged_status_entries(root)
            if (status["status"], status["paths"]) in staged:
                continue
        found |= scan_line(line, number)
    return found


def git_output(root: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", *args], cwd=root, check=True, capture_output=True
    ).stdout


def git_path_display(path: bytes, quote_non_ascii: bool) -> str:
    # Git uses C quoting for control bytes and optionally for non-ASCII bytes.
    escapes = {
        7: r"\a",
        8: r"\b",
        9: r"\t",
        10: r"\n",
        11: r"\v",
        12: r"\f",
        13: r"\r",
        34: r"\"",
        92: r"\\",
    }
    if not any(
        byte in escapes or byte < 32 or byte == 127 or (quote_non_ascii and byte >= 128)
        for byte in path
    ):
        return os.fsdecode(path)
    quoted = b"".join(
        (
            escapes[byte].encode()
            if byte in escapes
            else (
                f"\\{byte:03o}".encode()
                if byte < 32 or byte == 127 or (quote_non_ascii and byte >= 128)
                else bytes([byte])
            )
        )
        for byte in path
    )
    return '"' + os.fsdecode(quoted) + '"'


def staged_status_entries(root: Path) -> set[tuple[str, str]]:
    entries = iter(
        git_output(root, "diff", "--cached", "--name-status", "-z").split(b"\0")
    )
    labels = {
        "A": "new file",
        "M": "modified",
        "D": "deleted",
        "R": "renamed",
        "C": "copied",
        "T": "typechange",
    }
    staged = set()
    for status in entries:
        if not status:
            continue
        code = chr(status[0])
        paths = [next(entries)]
        if code in {"R", "C"}:
            paths.append(next(entries))
        if code in labels:
            for quote_non_ascii in (True, False):
                staged.add(
                    (
                        labels[code],
                        " -> ".join(
                            git_path_display(path, quote_non_ascii) for path in paths
                        ),
                    )
                )
    return staged


def scan_changes(
    root: Path, revisions: tuple[str, ...], commit: str | None = None
) -> bool:
    paths = git_output(
        root,
        *revisions,
        "--no-renames",
        "--raw",
        "-z",
        "--diff-filter=ACDMRT" if commit else "--diff-filter=ACMRT",
    )
    found = False
    entries = paths.split(b"\0")
    for metadata, encoded in zip(entries[::2], entries[1::2]):
        if not encoded:
            continue
        filename = os.fsdecode(encoded)
        found |= scan_line(
            filename, f"{filename}:0" if commit else filename, commit=commit
        )
        # Deletions have no blob; gitlinks publish a commit ID, not a file blob.
        if metadata.split()[1] in {b"000000", b"160000"}:
            continue
        data = git_output(root, "show", f"{commit or ''}:{filename}")
        if data.startswith(UTF32_BOMS + UTF16_BOMS):
            found |= scan_text(decode(data), filename, commit)
            continue
        diff = git_output(
            root,
            *revisions,
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
                found |= scan_line(line[1:], number, filename, commit)
                number += 1
            elif number is not None and line.startswith(" "):
                number += 1
    return found


def scan_staged(root: Path) -> bool:
    return scan_changes(root, ("diff", "--cached"))


def scan_commit_range(root: Path, commit_range: str) -> bool:
    found = False
    commits = git_output(
        root,
        "rev-list",
        "--reverse",
        "--topo-order",
        "--parents",
        "--end-of-options",
        commit_range,
        "--",
    )
    for entry in commits.decode("ascii").splitlines():
        commit, *parents = entry.split()
        # First-parent diffs include merge resolutions; side commits are scanned too.
        revisions = (
            ("diff", parents[0], commit)
            if parents
            else ("diff-tree", "--root", "--no-commit-id", "-r", commit)
        )
        found |= scan_changes(root, revisions, commit)
    return found


def scan_file(path: Path, filename: str) -> bool:
    found = scan_line(filename, filename)
    # A submodule gitlink publishes only its name and commit, not this checkout.
    if path.is_dir() and not path.is_symlink():
        return found
    # A tracked symlink publishes its target, not the external file's contents.
    data = os.fsencode(os.readlink(path)) if path.is_symlink() else path.read_bytes()
    return scan_text(decode(data), filename) | found


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--staged", action="store_true", help="scan index names and added lines"
    )
    mode.add_argument("--files", nargs="+", type=Path)
    mode.add_argument("--all-tracked", action="store_true")
    mode.add_argument(
        "--commit-range",
        help="scan names and added lines of every commit in BASE..HEAD",
    )
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
        if args.commit_range:
            return int(scan_commit_range(root, args.commit_range))
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
