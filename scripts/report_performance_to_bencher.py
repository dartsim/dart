#!/usr/bin/env python3
"""Report DART benchmark JSON files to Bencher."""

from __future__ import annotations

import argparse
import glob
import os
import shlex
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        action="append",
        default=[],
        help=(
            "Benchmark JSON file or glob pattern. Defaults to .benchmark_results/*.json."
        ),
    )
    parser.add_argument(
        "--project",
        default=os.environ.get("BENCHER_PROJECT"),
        help="Bencher project slug. Defaults to BENCHER_PROJECT.",
    )
    parser.add_argument(
        "--key",
        default=os.environ.get("BENCHER_API_KEY"),
        help="Bencher project API key. Defaults to BENCHER_API_KEY.",
    )
    parser.add_argument(
        "--host",
        default=os.environ.get("BENCHER_HOST"),
        help="Optional Bencher API host. Defaults to BENCHER_HOST.",
    )
    parser.add_argument(
        "--branch",
        default=os.environ.get("GITHUB_REF_NAME", "local"),
        help="Bencher branch name. Defaults to GITHUB_REF_NAME.",
    )
    parser.add_argument(
        "--sha",
        default=os.environ.get("GITHUB_SHA", "unknown"),
        help="Commit SHA recorded with the report. Defaults to GITHUB_SHA.",
    )
    parser.add_argument(
        "--testbed",
        default=_default_testbed(),
        help="Bencher testbed. Defaults to the GitHub runner OS/arch.",
    )
    parser.add_argument(
        "--bencher-bin",
        default="bencher",
        help="Bencher CLI executable.",
    )
    parser.add_argument(
        "--skip-if-unconfigured",
        action="store_true",
        help="Exit successfully when project or key are missing.",
    )
    parser.add_argument(
        "--skip-if-no-input",
        action="store_true",
        help="Exit successfully when no benchmark JSON files match.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print sanitized Bencher commands without executing them.",
    )
    return parser.parse_args()


def _default_testbed() -> str:
    os_name = os.environ.get("RUNNER_OS")
    arch = os.environ.get("RUNNER_ARCH")
    if os_name and arch:
        return f"github-{os_name}-{arch}"
    return "local"


def _collect_inputs(patterns: list[str]) -> list[Path]:
    paths: list[Path | None] = []
    candidates = patterns or [".benchmark_results/*.json"]
    for pattern in candidates:
        matches = sorted(Path(match) for match in glob.glob(pattern))
        if matches:
            paths.extend(_valid_input(path) for path in matches if path.is_file())
            continue

        path = Path(pattern)
        if path.is_file():
            paths.append(_valid_input(path))
            continue

        if any(character in pattern for character in "*?[]"):
            continue
        raise SystemExit(f"Benchmark input not found: {pattern}")

    return sorted(path for path in dict.fromkeys(paths) if path is not None)


def _valid_input(path: Path) -> Path | None:
    if path.stat().st_size > 0:
        return path
    print(f"Skipping empty benchmark input: {path}", file=sys.stderr)
    return None


def _base_command(args: argparse.Namespace) -> list[str]:
    command = [
        args.bencher_bin,
        "run",
        "--project",
        args.project,
        "--branch",
        args.branch,
        "--hash",
        args.sha,
        "--testbed",
        args.testbed,
        "--adapter",
        "cpp_google",
    ]
    if args.host:
        command.extend(["--host", args.host])
    return command


def _format_command(command: list[str]) -> str:
    return " ".join(shlex.quote(part) for part in command)


def main() -> int:
    args = parse_args()
    if not args.project or not args.key:
        message = "BENCHER_PROJECT or BENCHER_API_KEY is not configured."
        if args.skip_if_unconfigured:
            print(f"{message} Skipping Bencher reporting.")
            return 0
        raise SystemExit(message)

    paths = _collect_inputs(args.input)
    if not paths:
        message = "No benchmark JSON inputs matched."
        if args.skip_if_no_input:
            print(f"{message} Skipping Bencher reporting.")
            return 0
        raise SystemExit(message)

    env = os.environ.copy()
    env["BENCHER_API_KEY"] = args.key
    command = _base_command(args)
    for path in paths:
        report_command = [*command, "--file", str(path)]
        if args.dry_run:
            print(_format_command(report_command))
        else:
            subprocess.run(report_command, check=True, env=env)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
