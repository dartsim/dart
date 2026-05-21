#!/usr/bin/env python3
"""Dry-run the workflow's gh-pages performance dashboard publication path."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SOURCE_URL = "https://github.com/dartsim/dart/actions/runs/workflow-dry-run"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        action="append",
        default=[],
        help=(
            "Google Benchmark JSON file or directory to pass to the generator. "
            "Defaults to .benchmark_results, matching the workflow."
        ),
    )
    parser.add_argument(
        "--seed-root",
        type=Path,
        default=Path(".benchmark_seed"),
        help="Directory whose child directories are passed as --seed-input.",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=None,
        help=(
            "Local dry-run root. Defaults to a temporary directory. The "
            "dashboard is written under <work-dir>/gh-pages/performance."
        ),
    )
    parser.add_argument(
        "--run-id",
        default="workflow-dry-run",
        help="Run id recorded in the dry-run dashboard.",
    )
    parser.add_argument(
        "--run-at",
        default=None,
        help="Optional ISO-8601 timestamp recorded in the dry-run dashboard.",
    )
    parser.add_argument(
        "--branch",
        default="main",
        help="Branch recorded in the dry-run dashboard.",
    )
    parser.add_argument(
        "--sha",
        default="workflow-dry-run",
        help="Commit SHA recorded in the dry-run dashboard.",
    )
    parser.add_argument(
        "--source-url",
        default=DEFAULT_SOURCE_URL,
        help="Source URL recorded in the dry-run dashboard.",
    )
    parser.add_argument(
        "--testbed",
        default="local-workflow-dry-run",
        help="Testbed name recorded in the dry-run dashboard.",
    )
    return parser.parse_args()


def _seed_dirs(seed_root: Path) -> list[Path]:
    if not seed_root.is_dir():
        return []
    return [path for path in sorted(seed_root.iterdir()) if path.is_dir()]


def _run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def _verify_workflow_summary(path: Path) -> None:
    try:
        summary = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Failed to read workflow summary {path}: {exc}") from exc

    required_snippets = [
        "## DART Performance Dashboard",
        "| Canonical website | [Read the Docs](https://dart.readthedocs.io/en/latest/) |",
        "| Dashboard | [Open dashboard](https://dartsim.github.io/dart/performance/) |",
        "| Status endpoint | [status.json](https://dartsim.github.io/dart/performance/status.json) |",
        "| Durable data | [data.json](https://dartsim.github.io/dart/performance/data.json) |",
        "| Text summary | [summary.md](https://dartsim.github.io/dart/performance/summary.md) |",
        "| Dashboard guide | [Read the guide](https://dart.readthedocs.io/en/latest/community/performance_dashboard.html) |",
        "### Service Decision Summary",
        "best_option",
        "### Service Decision",
        "DART-owned GitHub Pages",
        "Bencher Cloud or Self-Hosted",
        "LLVM LNT",
        "### Publication Contract",
        "| Pages source | `gh-pages` / |",
        "| Publish ref | `refs/heads/main` |",
        "schedule_crons",
        "30 3 * * 0,3",
        "`actions: read`<br>`contents: write`<br>`pages: write`",
        "performance-dashboard-site-<run_id>-<run_attempt>",
        "pixi run check-bm-dashboard-launch-live",
        "maintainer approval",
        "### External Competitor Status",
        "live competitor",
        "### Testbed Summary",
        "### Trend Summary",
        "primary_status_counts",
    ]
    missing = [snippet for snippet in required_snippets if snippet not in summary]
    if missing:
        raise RuntimeError(
            "Workflow summary is missing launch metadata: " + ", ".join(missing)
        )


def _generate_command(args: argparse.Namespace, output_dir: Path) -> list[str]:
    command = [
        sys.executable,
        str(SCRIPT_DIR / "generate_performance_dashboard.py"),
    ]
    for input_path in args.input or [Path(".benchmark_results")]:
        command.extend(["--input", str(input_path)])
    command.extend(
        [
            "--output-dir",
            str(output_dir),
            "--history",
            str(output_dir / "data.json"),
            "--clean-output",
        ]
    )
    for seed_dir in _seed_dirs(args.seed_root):
        command.extend(["--seed-input", str(seed_dir)])
    command.extend(
        [
            "--run-id",
            args.run_id,
            "--branch",
            args.branch,
            "--sha",
            args.sha,
            "--source-url",
            args.source_url,
            "--testbed",
            args.testbed,
            "--allow-empty",
        ]
    )
    if args.run_at:
        command.extend(["--run-at", args.run_at])
    return command


def _run_dry_run(args: argparse.Namespace, root: Path) -> Path:
    performance_dir = root / "gh-pages" / "performance"
    performance_dir.mkdir(parents=True, exist_ok=True)
    workflow_summary = root / "workflow-summary.md"
    _run(_generate_command(args, performance_dir))
    _run(
        [
            sys.executable,
            str(SCRIPT_DIR / "verify_performance_dashboard.py"),
            str(performance_dir),
        ]
    )
    _run(
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_endpoint.py"),
            str(performance_dir / "status.json"),
            "--expect-status-file",
            str(performance_dir / "status.json"),
            "--expect-data-file",
            str(performance_dir / "data.json"),
            "--expect-dashboard-file",
            str(performance_dir / "index.html"),
            "--expect-summary-file",
            str(performance_dir / "summary.md"),
            "--require-dashboard-page",
            "--dashboard-url",
            str(performance_dir / "index.html"),
            "--require-summary",
            "--summary-url",
            str(performance_dir / "summary.md"),
        ]
    )
    _run(
        [
            sys.executable,
            str(SCRIPT_DIR / "summarize_performance_dashboard.py"),
            str(performance_dir / "status.json"),
            "--summary-file",
            str(workflow_summary),
        ]
    )
    _verify_workflow_summary(workflow_summary)
    return performance_dir


def main() -> int:
    args = parse_args()
    if args.work_dir is not None:
        args.work_dir.mkdir(parents=True, exist_ok=True)
        performance_dir = _run_dry_run(args, args.work_dir)
        print(f"verified workflow dashboard dry run: {performance_dir}")
        return 0

    temp_root = Path(tempfile.mkdtemp(prefix="dart-bm-dashboard-workflow-"))
    try:
        performance_dir = _run_dry_run(args, temp_root)
        print(f"verified workflow dashboard dry run: {performance_dir}")
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
