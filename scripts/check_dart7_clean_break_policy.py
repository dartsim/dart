#!/usr/bin/env python3
"""Check the DART 7 clean-break and DART 6 LTS Gazebo support policy."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GZ_WORKFLOW = REPO_ROOT / ".github" / "workflows" / "ci_gz_physics.yml"
DEFAULT_PIXI = REPO_ROOT / "pixi.toml"
DEFAULT_RELEASE_ROADMAP = REPO_ROOT / "docs" / "onboarding" / "release-roadmap.md"


@dataclass(frozen=True)
class Violation:
    message: str


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gz-workflow", type=Path, default=DEFAULT_GZ_WORKFLOW)
    parser.add_argument("--pixi", type=Path, default=DEFAULT_PIXI)
    parser.add_argument("--release-roadmap", type=Path, default=DEFAULT_RELEASE_ROADMAP)
    return parser.parse_args(argv)


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8")
    except OSError:
        return None


def indented_block(text: str, key: str, indent: int) -> str:
    spaces = " " * indent
    next_key = re.compile(rf"(?m)^{spaces}[A-Za-z0-9_-]+:\s*(?:#.*)?$")
    start_match = re.search(rf"(?m)^{spaces}{re.escape(key)}:\s*(?:#.*)?$", text)
    if start_match is None:
        return ""

    next_match = next_key.search(text, start_match.end())
    end = next_match.start() if next_match is not None else len(text)
    return text[start_match.start() : end]


def extract_branch_entries(block: str) -> list[str]:
    branches = indented_block(block, "branches", 4)
    return [
        match.group(1).strip().strip("\"'")
        for match in re.finditer(r"(?m)^\s*-\s*([^#\n]+)", branches)
    ]


def check_gz_workflow(path: Path) -> list[Violation]:
    text = read_text(path)
    if text is None:
        return [Violation(f"Could not read {path}")]

    violations: list[Violation] = []
    on_block = indented_block(text, "on", 0)
    if not on_block:
        violations.append(Violation("CI gz-physics workflow must define an on block"))
        return violations

    if re.search(r"(?m)^  schedule:\s*(?:#.*)?$", on_block):
        violations.append(
            Violation("CI gz-physics must not run on a main-branch schedule")
        )

    if "  workflow_dispatch:" not in on_block:
        violations.append(
            Violation("CI gz-physics must keep manual workflow_dispatch canary access")
        )

    for event_name in ("push", "pull_request"):
        event_block = indented_block(on_block, event_name, 2)
        if not event_block:
            violations.append(
                Violation(f"CI gz-physics must define release-only {event_name}")
            )
            continue

        branches = extract_branch_entries(event_block)
        if branches != ["release-*"]:
            violations.append(
                Violation(
                    f"CI gz-physics {event_name} branches must be release-* only, "
                    f"got {branches or 'none'}"
                )
            )

    return violations


def pinned_gz_physics_branch(pixi_path: Path) -> str | None:
    text = read_text(pixi_path)
    if text is None:
        return None

    match = re.search(r"git clone --branch\s+([A-Za-z0-9_.\-/]+)\s+", text)
    return match.group(1) if match is not None else None


def check_release_roadmap(path: Path, pinned_branch: str | None) -> list[Violation]:
    text = read_text(path)
    if text is None:
        return [Violation(f"Could not read {path}")]

    required_snippets = [
        "DART 6 LTS: Compatibility Line",
        "active DART 6 LTS branch",
        "highest maintained `release-6.*` branch",
        "Gazebo/gz-physics compatibility fixes",
        "main-branch Gazebo workflow is a migration canary",
        "sunset date or sunset trigger",
    ]
    violations = [
        Violation(f"Release roadmap missing clean-break policy text: {snippet}")
        for snippet in required_snippets
        if snippet not in text
    ]

    if pinned_branch is None:
        violations.append(
            Violation("Could not find pinned gz-physics branch in pixi.toml")
        )
    elif pinned_branch not in text:
        violations.append(
            Violation(
                "Release roadmap must document the pinned gz-physics branch "
                f"used by the support lane: {pinned_branch}"
            )
        )

    return violations


def find_violations(
    gz_workflow: Path = DEFAULT_GZ_WORKFLOW,
    pixi: Path = DEFAULT_PIXI,
    release_roadmap: Path = DEFAULT_RELEASE_ROADMAP,
) -> list[Violation]:
    pinned_branch = pinned_gz_physics_branch(pixi)
    return [
        *check_gz_workflow(gz_workflow),
        *check_release_roadmap(release_roadmap, pinned_branch),
    ]


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    violations = find_violations(args.gz_workflow, args.pixi, args.release_roadmap)
    if not violations:
        print("DART 7 clean-break policy check passed.")
        return 0

    print("DART 7 clean-break policy check failed:", file=sys.stderr)
    for violation in violations:
        print(f"- {violation.message}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
