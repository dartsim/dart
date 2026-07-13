#!/usr/bin/env python3
"""Exercise seven deterministic DART workflow-routing scenarios."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ai_infrastructure import (
    PROFILES,
    SCENARIO_IDS,
    detect_profile,
    repository_root,
    validate_scenarios,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("auto", *PROFILES), default="auto")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = repository_root(args.root)
    profile = detect_profile(root, args.profile)
    results, errors = validate_scenarios(root, profile)
    if args.json:
        print(
            json.dumps(
                {"profile": profile, "scenarios": results, "errors": errors},
                indent=2,
            )
        )
    else:
        for result in results:
            state = "PASS" if result["valid"] else "FAIL"
            workflow = result.get("expected_route") or {}
            chain = " -> ".join(result.get("actual_agents_chain") or [])
            print(
                f"{state}: {result['id']} -> {workflow.get('kind')}:"
                f"{workflow.get('name')} [{chain}]"
            )
            for error in result.get("errors") or []:
                print(f"  ERROR: {error}")
        for error in errors:
            if not error.startswith("scenario "):
                print(f"ERROR: {error}")
        print(f"Exercised {len(results)} scenarios (profile={profile})")
    return (
        0
        if not errors
        and len(results) == len(SCENARIO_IDS)
        and all(result["valid"] for result in results)
        else 1
    )


if __name__ == "__main__":
    raise SystemExit(main())
