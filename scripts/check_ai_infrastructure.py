#!/usr/bin/env python3
"""Validate DART's tracked AI infrastructure without modifying it."""

from __future__ import annotations

import argparse
from pathlib import Path

from ai_infrastructure import (
    PROFILES,
    detect_profile,
    format_errors,
    repository_root,
    run_checks,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="affirm the non-mutating check mode (the only supported mode)",
    )
    parser.add_argument("--profile", choices=("auto", *PROFILES), default="auto")
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repository_root(args.root)
    profile = detect_profile(root, args.profile)
    errors = run_checks(root, profile)
    if errors:
        print(format_errors(errors))
        print(
            f"AI infrastructure check FAILED ({len(errors)} errors, profile={profile})"
        )
        return 1
    print(f"AI infrastructure check passed (profile={profile})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
