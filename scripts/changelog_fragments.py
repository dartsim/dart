#!/usr/bin/env python3
"""Check or assemble the changelog fragments in ``changelog.d/`` on main.

Pull requests on main add one fragment per changelog entry instead of editing
CHANGELOG.md, so open pull requests never conflict over that file; the format
is in changelog.d/README.md. ``check`` validates every fragment. ``assemble``
moves each fragment's entry to the end of its category under the first
(unreleased) release heading of CHANGELOG.md, ordered by the number of the PR
that introduced the fragment, and deletes the fragments. Release branches have
no fragments.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import NamedTuple

ROOT = Path(__file__).resolve().parents[1]

# A fragment may name a category already under the first release heading or
# one of these, which assembly creates in this order. The list is the category
# vocabulary of the current release lines; keep changelog.d/README.md in sync.
CATEGORY_ORDER = (
    "Breaking Changes",
    "Behavior Changes",
    "API",
    "Performance",
    "Collision",
    "Constraint",
    "Simulation",
    "Dynamics",
    "Parsers",
    "Python",
    "GUI",
    "Examples",
    "Math",
    "Build",
)
FRAGMENT_NAME = re.compile(r"[a-z0-9]+(?:-[a-z0-9]+)*\.md")
SECTION_LINE = re.compile(r"Section: (.+)")
LINK_LINE = re.compile(
    r"    \[#(\d+)\]\(https://github\.com/dartsim/dart/(pull|issues)/(\d+)\)(,?)"
)
# The PR number that ends a squash-merge subject, as in "Add x (#123)".
SUBJECT_PR = re.compile(r".*\(#([1-9]\d*)\)")
# Editor swap files, like `.draft.md.swp`; any other hidden file is validated.
EDITOR_ARTIFACT = re.compile(r"\..+\.sw[a-p]")
# A line holding only a reference link, at any indentation.
LINK_ONLY = re.compile(r"\s*\[#[^\]]*\]\([^)]*\),?")


class Fragment(NamedTuple):
    path: Path
    category: str
    body: list[str]
    prs: list[int]


def parse_fragment(path: Path) -> tuple[Fragment | None, list[str]]:
    """Parse one fragment; return it (None if unusable) and its problems."""
    name = f"changelog.d/{path.name}"
    try:
        lines = path.read_text(encoding="utf-8-sig").splitlines()
    except UnicodeDecodeError:
        return None, [f"{name}: save the fragment as UTF-8"]
    while lines and not lines[-1].strip():
        lines.pop()
    errors = [
        f"{name}:{number}: remove trailing whitespace"
        for number, line in enumerate(lines, start=1)
        if line != line.rstrip()
    ]
    if not FRAGMENT_NAME.fullmatch(path.name):
        errors.append(f"{name}: use a lowercase kebab-case name, like `fix-x.md`")
    if re.fullmatch(r"con|prn|aux|nul|com[1-9]|lpt[1-9]", path.stem, re.IGNORECASE):
        errors.append(
            f"{name}: choose another name; `{path.stem}` is reserved on Windows"
        )
    section = SECTION_LINE.fullmatch(lines[0]) if lines else None
    if not section:
        return None, errors + [f"{name}:1: start with `Section: <Category>`"]
    if len(lines) < 3 or lines[1] or not lines[2].strip():
        return None, errors + [
            f"{name}:2: follow the Section line with one blank line, then the entry"
        ]

    body = lines[2:]
    if not body[0].startswith("  * ") or not body[0][4:].strip():
        errors.append(f"{name}:3: start the entry with `  * `, as in CHANGELOG.md")
    for number, line in enumerate(body[1:], start=4):
        if line.startswith("  * "):
            errors.append(f"{name}:{number}: put each entry in its own fragment")
        elif line.strip() and not re.match(r" {4,}\S", line):
            errors.append(f"{name}:{number}: indent continuation lines 4 spaces")

    # The entry ends with its reference links, one per line, comma-separated.
    first_link = len(body)
    while first_link > 1 and body[first_link - 1].lstrip().startswith("[#"):
        first_link -= 1
    errors += [
        f"{name}:{index + 3}: keep the links together as the entry's last lines"
        for index in range(1, first_link)
        if LINK_ONLY.fullmatch(body[index])
    ]
    prs = []
    for index in range(first_link, len(body)):
        match = LINK_LINE.fullmatch(body[index])
        last = index == len(body) - 1
        if not match or match[1] != match[3] or (match[4] == ",") == last:
            errors.append(
                f"{name}:{index + 3}: write each link as "
                "`    [#N](https://github.com/dartsim/dart/pull/N)`, "
                "with a comma after every link but the last"
            )
        elif match[2] == "pull":
            prs.append(int(match[1]))
    return Fragment(path, section[1].strip(), body, prs), errors


def release_section(lines: list[str]) -> tuple[int, int]:
    """Return the line range of the first release section, heading included."""
    start = next((i for i, line in enumerate(lines) if line.startswith("### ")), None)
    if start is None:
        raise ValueError("CHANGELOG.md: no `### ` release heading")
    end = next(
        (i for i in range(start + 1, len(lines)) if lines[i].startswith("#")),
        len(lines),
    )
    return start, end


def categories(lines: list[str], start: int, end: int) -> dict[str, int]:
    """Map each category bullet of a release section to its line index."""
    return {
        lines[i][2:].strip(): i
        for i in range(start + 1, end)
        if lines[i].startswith("* ")
    }


def last_text_line(lines: list[str], begin: int, end: int) -> int:
    return max(i for i in range(begin, end) if lines[i].strip())


def insert_entry(lines: list[str], category: str, body: list[str]) -> bool:
    """Append an entry to a category of the first release section.

    Creates a missing category in CATEGORY_ORDER and returns whether it did.
    """
    start, end = release_section(lines)
    found = categories(lines, start, end)
    created = category not in found
    if created:
        rank = CATEGORY_ORDER.index(category)
        later = [
            index
            for name, index in found.items()
            if name in CATEGORY_ORDER and CATEGORY_ORDER.index(name) > rank
        ]
        if later:
            lines[min(later) : min(later)] = [f"* {category}", ""]
        else:
            at = last_text_line(lines, start, end) + 1
            lines[at:at] = ["", f"* {category}"]
        start, end = release_section(lines)
        found = categories(lines, start, end)
    begin = found[category]
    stop = min((index for index in found.values() if index > begin), default=end)
    at = last_text_line(lines, begin, stop) + 1
    lines[at:at] = ["", *body]
    return created


def load(root: Path) -> tuple[list[Fragment], list[str], list[str]]:
    """Parse every fragment; return them, all problems, and CHANGELOG.md lines."""
    lines = (root / "CHANGELOG.md").read_text(encoding="utf-8").split("\n")
    start, end = release_section(lines)
    existing = list(categories(lines, start, end))
    known = existing + [name for name in CATEGORY_ORDER if name not in existing]
    directory = root / "changelog.d"
    fragments, errors = [], []
    for path in sorted(directory.iterdir()) if directory.is_dir() else []:
        if path.name == "README.md" or EDITOR_ARTIFACT.fullmatch(path.name):
            continue
        if path.suffix != ".md" or not path.is_file():
            errors.append(f"changelog.d/{path.name}: fragments are `<topic>.md` files")
            continue
        fragment, problems = parse_fragment(path)
        errors += problems
        if fragment is None:
            continue
        if fragment.category not in known:
            errors.append(
                f"changelog.d/{path.name}:1: unknown category "
                f"`{fragment.category}`; use one of: {', '.join(known)}"
            )
        fragments.append(fragment)
    return fragments, errors, lines


def report(errors: list[str]) -> int:
    for error in errors:
        print(error, file=sys.stderr)
    return 1


def git(root: Path, *args: str) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(["git", *args], cwd=root, capture_output=True, text=True)
    except OSError as error:
        raise ValueError(
            f"changelog.d/: cannot run git; install Git and retry: {error}"
        ) from error


def target_branch(root: Path) -> str:
    """Return the branch this checkout lands on, or "" if unknown.

    Follows check_ai_infrastructure.check_release_target: an explicit
    GITHUB_BASE_REF wins, even a stacked topic base; otherwise a checked-out
    or CI-reported main/release branch counts.
    """
    base = os.environ.get("GITHUB_BASE_REF")
    if base:
        return base
    try:
        checkout = git(root, "branch", "--show-current").stdout.strip()
    except ValueError:
        checkout = ""
    for candidate in (checkout, os.environ.get("GITHUB_REF_NAME", "")):
        if re.fullmatch(r"main|release-.+", candidate):
            return candidate
    return ""


def release_branch_errors(root: Path, fragments: list[Fragment]) -> list[str]:
    branch = target_branch(root)
    if not (fragments and branch.startswith("release-")):
        return []
    return [
        f"changelog.d/{fragment.path.name}: `{branch}` has no fragments; "
        "write the entry directly in CHANGELOG.md"
        for fragment in fragments
    ]


def check(root: Path) -> int:
    fragments, errors, _ = load(root)
    errors += release_branch_errors(root, fragments)
    if errors:
        return report(errors)
    print(f"{len(fragments)} changelog fragment(s) OK")
    return 0


def fragment_changes(root: Path) -> list[str]:
    """Return paths with staged, unstaged, untracked, or ignored changes in
    CHANGELOG.md or changelog.d/, which assembly must not mix with its own output."""
    result = git(
        root,
        "status",
        "--porcelain",
        "--untracked-files=all",
        "--ignored=matching",
        "--",
        "CHANGELOG.md",
        "changelog.d",
    )
    if result.returncode:
        raise ValueError(
            "changelog.d/: cannot check Git status; use --root pointing to a Git checkout: "
            f"{result.stderr.strip()}"
        )
    return [line[3:] for line in result.stdout.splitlines()]


def fragment_origins(
    root: Path, fragments: list[Fragment]
) -> tuple[dict[str, int], list[str]]:
    """Map each fragment name to the PR that introduced it, from Git history.

    The PR is the terminal (#N) of the subject of the commit that added the
    file, following renames. Link lines are user input, not provenance, so
    missing, shallow, or ambiguous history is an error, never a guess.
    """
    shallow = git(root, "rev-parse", "--is-shallow-repository")
    if shallow.returncode or shallow.stdout.strip() != "false":
        return {}, [
            "changelog.d/: cannot tell which PR introduced each fragment from a "
            "shallow or unreadable Git history; fetch the full history "
            f"(`git fetch --unshallow`) and retry: {shallow.stderr.strip()}"
        ]
    origins, errors = {}, []
    for fragment in fragments:
        relative = f"changelog.d/{fragment.path.name}"
        log = git(
            root,
            "log",
            "--follow",
            "--diff-filter=A",
            "-1",
            "--format=%s",
            "--",
            relative,
        )
        subject = log.stdout.strip()
        match = None if log.returncode else SUBJECT_PR.fullmatch(subject)
        if match:
            origins[fragment.path.name] = int(match[1])
        else:
            errors.append(
                f"{relative}: the commit that added it must have a subject ending "
                "with its PR as (#N), as a squash merge makes; found "
                f"`{subject or log.stderr.strip() or 'no commit'}`"
            )
    return origins, errors


def assemble(root: Path) -> int:
    changes = fragment_changes(root)
    if changes:
        return report(
            [
                f"{path}: commit or remove this change before assembling; after a "
                "failed assembly, review and commit its CHANGELOG.md changes, "
                "then re-run to delete the remaining fragments"
                for path in changes
            ]
        )
    fragments, errors, lines = load(root)
    errors += release_branch_errors(root, fragments)
    origins, problems = fragment_origins(root, fragments) if fragments else ({}, [])
    errors += problems
    errors += [
        f"changelog.d/{fragment.path.name}: end the entry with its own PR link "
        f"line, `    [#{origins[fragment.path.name]}]"
        f"(https://github.com/dartsim/dart/pull/{origins[fragment.path.name]})`; "
        "the subject from `git log --follow --diff-filter=A -1 --format=%s -- "
        f"changelog.d/{fragment.path.name}` ends with the PR as (#N)"
        for fragment in fragments
        if fragment.path.name in origins
        and origins[fragment.path.name] not in fragment.prs
    ]
    if errors:
        return report(errors)
    if not fragments:
        print("No changelog fragments to assemble")
        return 0
    heading = lines[release_section(lines)[0]]
    if "(Unreleased)" not in heading:
        return report(
            [
                f"CHANGELOG.md: `{heading}` is released; to include these "
                "fragments in it, assemble before dating it; for changes after "
                "its tag, roll `main` forward first (row and heading together, "
                "docs/onboarding/release-management.md § Release Target)"
            ]
        )

    for fragment in sorted(
        fragments, key=lambda f: (origins[f.path.name], f.path.name)
    ):
        start, end = release_section(lines)
        entry = "\n".join(fragment.body)
        if f"\n{entry}\n" in "\n".join(lines[start:end]) + "\n":
            print(
                f"changelog.d/{fragment.path.name}: entry already present under `{heading}`; "
                "deleting fragment only"
            )
            continue
        if insert_entry(lines, fragment.category, fragment.body):
            print(f"Created category `{fragment.category}`")
        print(
            f"changelog.d/{fragment.path.name} -> {fragment.category} "
            f"(#{origins[fragment.path.name]})"
        )
    # Write a sibling file and swap it in, so a failed write never truncates
    # CHANGELOG.md.
    staged = root / ".CHANGELOG.md.assembling"
    try:
        staged.write_text("\n".join(lines), encoding="utf-8", newline="\n")
        staged.replace(root / "CHANGELOG.md")
    finally:
        staged.unlink(missing_ok=True)
    for fragment in fragments:
        try:
            fragment.path.unlink()
        except OSError as error:
            raise ValueError(
                f"changelog.d/{fragment.path.name}: {error}; CHANGELOG.md is already "
                "updated, so review and commit the changes, then re-run to delete "
                "the remaining fragments"
            ) from error
    print(f"Assembled {len(fragments)} fragment(s) under `{heading}`")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("check", "assemble"))
    parser.add_argument(
        "--root", type=Path, default=ROOT, help="repository root (default: this one)"
    )
    args = parser.parse_args(argv)
    try:
        return check(args.root) if args.command == "check" else assemble(args.root)
    except (OSError, ValueError) as error:
        return report([f"error: {error}"])


if __name__ == "__main__":
    raise SystemExit(main())
