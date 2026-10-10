#!/usr/bin/env python3
"""Reject private or machine-specific paths in repository and published text.

These checks catch accidental publication by contributors and agents. Local
hooks are conveniences that explicit bypasses can skip.
The PR Text workflow checks the title, body, messages and changes of every PR
commit using the base branch's checker as the backstop before merge.
"""

from __future__ import annotations

import argparse
import ipaddress
import os
import re
import shlex
import subprocess
import sys
from pathlib import Path
from urllib.parse import urlsplit

# Keep the publication policy and its narrowly scoped exceptions here.
PATH_TAIL = r"[^\s`\"'<>\[\](){};,|]*"
URI = re.compile(
    r"[a-z][a-z0-9+.-]*://(?:[^\s`\"'<>\[\](){}/@;,|]+@)?"
    r"(?:\[[^\s`\"'<>\[\](){};,|]+\])?"
    r"(?:[^\s`\"'<>\[\](){};,|]|\([^\s`\"'<>\[\](){};,|]*\))*",
    re.IGNORECASE,
)
# Covers private planning/harness dirs, agent scratch/project dirs, numbered
# worktrees, hosted-agent workspaces, Linux/macOS/root homes, Windows
# drive/WSL/Git Bash profiles and local URLs. Other representations are out of
# scope unless contributors' tools produce them; PR Text remains the backstop.
PATTERNS = tuple(
    re.compile(pattern + PATH_TAIL, re.IGNORECASE)
    for pattern in (
        r"(?<![\w.-])\.sisyphus[/\\]+",
        r"(?<![\w.-])\.ab[/\\]+",
        r"/+tmp/+claude\-[^\s/\\`\"'<>]+",
        r"(?<![\w.-])\.claude[/\\]+projects[/\\]+",
        r"(?<![\w.-])scratchpad[/\\]+",
        r"(?<![\w.-])task_\d+(?:[/\\]+|-[\w-]+)",
        # Generic home-relative paths such as ~/.config name no user or machine.
        # Private agent project dirs, scratchpads and numbered worktrees still
        # match above, so ~/ alone is not reported.
        # Named-user tilde forms (~name/) are not reported either: binary files
        # are full of the same byte pattern, and contributors' tools expand them.
        # A host or path character before /home or /Users is not a home, and a
        # drive letter (C:/Users) is left to the Windows pattern below; other
        # labels such as cwd: or file: still precede a reported home.
        r"(?:(?<=\.\.)|(?<![\w./-])(?<!\b[A-Za-z]:))/+(?:home|Users|(?:mnt/+)?[A-Za-z]/+Users)/+[^\s/\\`\"'<>\[\](){};,|]+",
        # A standalone Docker WORKDIR naming one workspace directory is generic;
        # deeper paths and the same roots in published prose still identify work.
        r"(?!(?<=^WORKDIR )/+(?:workspace|workspaces)/+[^/\s]+$)"
        r"(?:(?<=\.\.)|(?<![\w./-])(?<!\b[A-Za-z]:))/+(?:workspace|workspaces|__w)/+[^\s/\\`\"'<>\[\](){};,|]+",
        # Unix root homes are case-sensitive; PDF /Root entries are not paths.
        r"(?<![\w./-])(?<!\b[A-Za-z]:)/+(?-i:root)(?=[/\\]|$|[\s`\"'<>\[\](){};,.:|])",
        r"(?<![\w.:-])[A-Za-z]:[/\\]+Users[/\\]+[^/\\\r\n`\"'<>\[\](){};,|]+",
        # Windows Actions checks out the repository in two same-named dirs.
        r"(?<![\w.:-])[A-Za-z]:[/\\]+a[/\\]+(?P<repo>[\w.-]+)[/\\]+(?P=repo)(?=[/\\]|$|[\s`\"'<>\[\](){};,|])",
        # Network (UNC) user profiles, including JSON-escaped separators.
        r"(?<![\w:/\\])[\\/]{2,}[^\\/\s]+[\\/]+Users[\\/]+[^\\/\r\n`\"'<>\[\](){};,|]+",
        r"(?<![\w:/\\])[\\/]{2,}(?:wsl\.localhost|wsl\$)[\\/]+"
        r"[^\\/\s`\"'<>\[\](){};,|]+[\\/]+"
        r"(?:home[\\/]+[^\\/\s`\"'<>\[\](){};,|]+|(?-i:root)(?=[/\\]|$|[\s`\"'<>\[\](){};,.:|]))",
    )
)
ALLOWLIST = {
    # Only file/staged scans may exempt checker fixtures; free text never does.
    "tests/test_check_local_paths.py": re.compile(
        r"(?:^|[/\\])(?:example(?: user|-repo)?(?:\.[\w.-]+)?|path-fixture)(?=$|[/\\:])",
        re.IGNORECASE,
    ),
    ".gitignore": re.compile(r"\.sisyphus[/]"),
}
HUNK = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")
UTF32_BOMS = (b"\xff\xfe\x00\x00", b"\x00\x00\xfe\xff")
UTF16_BOMS = (b"\xff\xfe", b"\xfe\xff")


def text_encoding(data: bytes) -> str | None:
    # UTF-32LE starts with the UTF-16LE mark, so test the longer marks first.
    if data.startswith(UTF32_BOMS):
        return "utf-32"
    if data.startswith(UTF16_BOMS):
        return "utf-16"
    if len(data) >= 8 and len(data) % 2 == 0 and b"\0" in data:
        for nul_bytes, text_bytes, encoding in (
            (data[1::2], data[::2], "utf-16-le"),
            (data[::2], data[1::2], "utf-16-be"),
        ):
            # Alternating NULs plus printable text distinguish UTF-16 from assets.
            if (
                nul_bytes.count(0) / len(nul_bytes) >= 0.3
                and sum(32 <= byte <= 126 or byte in b"\t\n\r" for byte in text_bytes)
                / len(text_bytes)
                >= 0.85
            ):
                return encoding
    return None


def decode(data: bytes) -> str:
    return data.decode(text_encoding(data) or "utf-8", errors="replace")


def content_decodings(data: bytes) -> list[str]:
    texts = [decode(data)]
    if b"\0" in data:
        # NUL-containing bytes are not UTF-8 text, even if UTF-8 accepts them.
        texts.extend(
            data.decode(encoding, errors="replace").removeprefix("\ufeff")
            for encoding in ("utf-16-le", "utf-16-be", "utf-32-le", "utf-32-be")
        )
    return list(dict.fromkeys(texts))


SCISSORS = re.compile(r"(?P<char>[^\r\n]+) -{24} >8 -{24}")
# Git wraps its editor instruction ("... Lines starting" / "<c> with '<c>' will
# be ignored, ..."); accept the wrapped second line and a one-line variant.
GIT_TEMPLATE_INSTRUCTION = re.compile(
    r"^(?P<char>[^\r\n]+?) (?:Lines starting )?with '(?P=char)' will be ignored(?:,|$)",
    re.MULTILINE,
)
GIT_SCISSORS_INSTRUCTION = re.compile(
    r"^(?P<char>[^\r\n]+?) Do not modify or remove the line above\.", re.MULTILINE
)


def is_public_host(host: str | None) -> bool:
    host = (host or "").lower().removesuffix(".")
    try:
        address = ipaddress.ip_address(host)
        # Python counts some multicast ranges (SSDP, link-local) as global.
        return address.is_global and not address.is_multicast
    except ValueError:
        try:
            host = host.encode("idna").decode("ascii")
        except UnicodeError:
            return False
        # Shorthand numeric hosts such as 127.1 still reach local addresses.
        if re.fullmatch(r"[0-9.]+|0x[0-9a-f.x]+", host, re.IGNORECASE):
            return False
        # Special-use DNS suffixes identify local machines and private networks.
        return (
            "." in host
            and len(host) <= 253
            and all(
                re.fullmatch(r"[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?", label)
                for label in host.split(".")
            )
            and not any(
                host == suffix or host.endswith("." + suffix)
                for suffix in (
                    "test",
                    "invalid",
                    "example",
                    "localhost",
                    "local",
                    "home.arpa",
                    "internal",
                    "lan",
                    "localdomain",
                )
            )
        )


def scan_line(
    line: str,
    number: int | str,
    filename: str | None = None,
    commit: str | None = None,
) -> bool:
    # Invalid bytes delimit text; keep any valid path prefix beside them.
    line = line.replace("\ufffd", "\n").replace(r"\/", "/")
    allowed = ALLOWLIST.get(filename)
    if filename == ".gitignore" and allowed.fullmatch(line.removesuffix("\r")):
        return False
    url_paths = []

    def mask_public_url(match: re.Match[str]) -> str:
        try:
            text = match.group()
            # HTTP(S) treats backslashes as slashes, including at the authority.
            if text.lower().startswith(("http://", "https://")):
                text = text.replace("\\", "/")
            url = urlsplit(text)
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
            if not (
                filename != ".gitignore" and allowed and allowed.search(match.group())
            )
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


def read_git_command(pid: int) -> list[str] | None:
    try:
        data = Path(f"/proc/{pid}/cmdline").read_bytes()
        if data:
            return [os.fsdecode(arg) for arg in data.rstrip(b"\0").split(b"\0")]
    except OSError:
        # Procfs may be unavailable; try the portable ps fallback below.
        pass
    try:
        result = subprocess.run(
            ["ps", "-o", "args=", "-p", str(pid)],
            check=True,
            capture_output=True,
            text=True,
        )
        return shlex.split(result.stdout) or None
    except (OSError, ValueError, subprocess.CalledProcessError):
        return None


# None denotes an optional value accepted only with '='.
GIT_GLOBAL_OPTIONS = {
    "-C": 1,
    "-c": 1,
    "--git-dir": 1,
    "--work-tree": 1,
    "--namespace": 1,
    "--exec-path": None,
    "--config-env": 1,
    "--super-prefix": 1,
    "--attr-source": 1,
    "-p": 0,
    "-P": 0,
    "--paginate": 0,
    "--no-pager": 0,
    "--bare": 0,
    "--no-replace-objects": 0,
    "--no-lazy-fetch": 0,
    "--literal-pathspecs": 0,
    "--glob-pathspecs": 0,
    "--noglob-pathspecs": 0,
    "--icase-pathspecs": 0,
    "--no-optional-locks": 0,
    "--no-advice": 0,
}


def commit_cleanup(command: list[str]) -> tuple[str, bool] | None:
    config_args = []
    i = 1
    while i < len(command) and command[i].startswith("-"):
        token = command[i]
        i += 1
        option, sep, value = token.partition("=")
        if token.startswith(("-C", "-c")) and len(token) > 2:
            option, sep, value = token[:2], "attached", token[2:]
        if option not in GIT_GLOBAL_OPTIONS:
            return None
        arity = GIT_GLOBAL_OPTIONS[option]
        if arity == 0 and sep:
            return None
        if arity == 1 and not sep:
            if i == len(command):
                return None
            value = command[i]
            i += 1
        if option == "-c":
            config_args.extend(("-c", value))
    if i >= len(command) or command[i] not in {"commit", "merge"}:
        return None
    subcommand = command[i]
    args = command[i + 1 :]
    # Git exposes hidden and negated options too, keeping prefix matching in sync.
    options = {
        option.rstrip("=")
        for option in subprocess.run(
            ["git", subcommand, "--git-completion-helper-all"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout.split()
        if option != "--"
    }
    cleanup = None
    supplied = verbose = False
    if subcommand == "commit":
        result = subprocess.run(
            ["git", *config_args, "config", "--bool-or-int", "--get", "commit.verbose"],
            capture_output=True,
            text=True,
        )
        if result.returncode not in (0, 1):
            result.check_returncode()
        value = result.stdout.strip()
        verbose = value == "true" or (value not in {"", "false"} and int(value) > 0)
    edit = None
    i = 0
    while i < len(args):
        token = args[i]
        i += 1
        if token == "--":
            break
        option, sep, value = token.partition("=")
        if option.startswith("--") and option not in options:
            matches = [name for name in options if name.startswith(option)]
            if len(matches) == 1:
                option = matches[0]
        if option == "--cleanup":
            if not sep and i < len(args):
                value = args[i]
                i += 1
            cleanup = value
        elif option in {"--edit", "--no-edit"}:
            edit = option == "--edit"
        elif option in {"--verbose", "--no-verbose"}:
            verbose = subcommand == "commit" and option == "--verbose"
        elif option in {"--message", "--file", "--reuse-message", "--reedit-message"}:
            supplied = True
            if option == "--reedit-message" and edit is None:
                edit = True
            i += int(not sep)
        elif option in {
            "--author",
            "--date",
            "--template",
            "--trailer",
            "--fixup",
            "--squash",
            "--pathspec-from-file",
            "--strategy",
            "--strategy-option",
            "--into-name",
        }:
            i += int(not sep)
        elif token.startswith("-") and not token.startswith("--"):
            for offset, short in enumerate(token[1:], 2):
                if subcommand == "merge" and short in "sX":
                    i += int(offset == len(token))
                    break
                if short == "e":
                    edit = True
                verbose |= subcommand == "commit" and short == "v"
                if short in "mFCc":
                    supplied = True
                    if short == "c" and edit is None:
                        edit = True
                    i += int(offset == len(token))
                    break
                if short in "tSuU":
                    i += int(short == "t" and offset == len(token))
                    break
    if cleanup is None:
        # The hook already runs in Git's repository; replay only config overrides.
        result = subprocess.run(
            ["git", *config_args, "config", "--get", "commit.cleanup"],
            capture_output=True,
            text=True,
        )
        if result.returncode not in (0, 1):
            result.check_returncode()
        cleanup = result.stdout.rstrip("\n") if result.returncode == 0 else "default"
    use_editor = edit if edit is not None else not supplied
    if cleanup == "default":
        # Merge editor use depends on environment and interactivity, not just args.
        cleanup = "strip" if subcommand == "commit" and use_editor else "whitespace"
    elif cleanup == "scissors" and not use_editor:
        cleanup = "whitespace"
    return cleanup, verbose


def scan_commit_message(
    text: str, git_command: list[str] | None = None, comment_string: str | None = None
) -> bool:
    lines = text.splitlines()
    instruction = GIT_TEMPLATE_INSTRUCTION.search(text)
    strip_comments = instruction is not None
    if instruction is None:
        for line, following in zip(lines, lines[1:]):
            candidate = GIT_SCISSORS_INSTRUCTION.match(following)
            scissors = SCISSORS.fullmatch(line)
            if scissors and candidate and scissors["char"] == candidate["char"]:
                instruction = candidate
                break
    cleanup = commit_cleanup(git_command) if git_command else None
    if cleanup is not None:
        mode, verbose = cleanup
        if comment_string is None:
            for key in ("core.commentString", "core.commentChar"):
                result = subprocess.run(
                    ["git", "config", "--get", key], capture_output=True, text=True
                )
                if result.returncode == 0 and result.stdout.rstrip("\n"):
                    comment_string = result.stdout.rstrip("\n")
                    break
            if comment_string == "auto":
                # Git's template instructions identify its selected character.
                comment_string = instruction["char"] if instruction else None
            elif not comment_string:
                comment_string = "#"
        strip_comments = mode == "strip" and comment_string is not None
        cut_at_scissors = mode == "scissors" or verbose
    else:
        # Unreadable or unparsable commands fall back to English-template evidence.
        comment_string = instruction["char"] if instruction else None
        cut_at_scissors = instruction is not None
    found = False
    for number, line in enumerate(lines, 1):
        scissors = SCISSORS.fullmatch(line)
        if cut_at_scissors and scissors and scissors["char"] == comment_string:
            break
        if strip_comments and line.startswith(comment_string):
            continue
        found |= scan_line(line, number)
    return found


def git_output(root: Path, *args: str) -> bytes:
    return subprocess.run(
        ["git", "--literal-pathspecs", *args], cwd=root, check=True, capture_output=True
    ).stdout


def scan_changes(
    root: Path, revisions: tuple[str, ...], commit: str | None = None
) -> bool:
    paths = git_output(
        root,
        *revisions,
        "--no-renames",
        "--ignore-submodules=none",
        "--no-ext-diff",
        "--no-textconv",
        "--no-color",
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
        found |= scan_line(
            filename, f"{filename}:0" if commit else filename, commit=commit
        )
        # Gitlinks publish a commit ID, not a file blob.
        if metadata.split()[1] == b"160000":
            continue
        data = git_output(root, "show", f"{commit or ''}:{filename}")
        texts = content_decodings(data)
        if text_encoding(data) or len(texts) > 1:
            for text in texts:
                found |= scan_text(text, filename, commit)
            continue
        diff = git_output(
            root,
            *revisions,
            "--no-renames",
            "--ignore-submodules=none",
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
        # Git patch records end at LF; a bare CR belongs to the added line.
        for line in diff.split("\n"):
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
        message = git_output(root, "show", "-s", "--format=%B", commit)
        for text in content_decodings(message):
            found |= scan_text(text, commit=commit)
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
    for text in content_decodings(data):
        found |= scan_text(text, filename)
    return found


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
        help="scan messages, names and added lines of every commit in BASE..HEAD",
    )
    mode.add_argument(
        "--text-file", type=Path, help="scan free text without exceptions"
    )
    mode.add_argument("--stdin", action="store_true", help="scan free text from stdin")
    mode.add_argument(
        "--commit-msg-file",
        type=Path,
        help="scan commit text, excluding editor template comments and Git scissors",
    )
    parser.add_argument(
        "--git-pid", type=int, help="parent Git process for commit cleanup"
    )
    args = parser.parse_args()
    try:
        if args.stdin:
            return int(
                any(
                    [
                        scan_text(text)
                        for text in content_decodings(sys.stdin.buffer.read())
                    ]
                )
            )
        if args.text_file:
            return int(
                any(
                    [
                        scan_text(text)
                        for text in content_decodings(args.text_file.read_bytes())
                    ]
                )
            )
        if args.commit_msg_file:
            command = read_git_command(args.git_pid) if args.git_pid else None
            return int(
                any(
                    [
                        scan_commit_message(text, command)
                        for text in content_decodings(args.commit_msg_file.read_bytes())
                    ]
                )
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
