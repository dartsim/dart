#!/usr/bin/env python3
"""Run the local launch preflight for the performance dashboard."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        default=os.environ.get("GITHUB_REPOSITORY", "dartsim/dart"),
        help="GitHub repository in owner/name form.",
    )
    parser.add_argument(
        "--work-dir",
        type=Path,
        default=Path("build/performance-dashboard-launch-preflight"),
        help="Ignored work directory for downloaded seed artifacts and dry-run output.",
    )
    parser.add_argument(
        "--require-registered",
        action="store_true",
        help="Require the workflow to be registered in GitHub Actions.",
    )
    parser.add_argument(
        "--require-published",
        action="store_true",
        help="Require the Read the Docs guide and public dashboard endpoint to be live.",
    )
    return parser.parse_args()


def _run(label: str, command: list[str]) -> None:
    print(f"==> {label}", flush=True)
    subprocess.run(command, check=True)


def _failure_message(label: str, exc: subprocess.CalledProcessError) -> str:
    return f"{label} failed with exit code {exc.returncode}: " + " ".join(
        str(part) for part in exc.cmd
    )


def _raise_failures(failures: list[str]) -> None:
    if failures:
        formatted = "\n".join(f"- {failure}" for failure in failures)
        raise RuntimeError(
            "Performance dashboard launch preflight failed:\n" + formatted
        )


def _clean_dir(path: Path) -> None:
    shutil.rmtree(path, ignore_errors=True)
    path.mkdir(parents=True, exist_ok=True)


def _publication_expectations_from_workflow_run(path: Path) -> list[str]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(
            f"Failed to read latest dashboard workflow run identity from {path}: {exc}"
        ) from exc

    expected_run_id = data.get("expected_dashboard_run_id")
    head_sha = data.get("head_sha")
    missing = [
        name
        for name, value in (
            ("expected_dashboard_run_id", expected_run_id),
            ("head_sha", head_sha),
        )
        if not value
    ]
    if missing:
        raise RuntimeError(
            "Latest dashboard workflow run identity is missing: " + ", ".join(missing)
        )
    return ["--expect-run-id", str(expected_run_id), "--expect-sha", str(head_sha)]


def run_preflight(args: argparse.Namespace) -> None:
    seed_dir = args.work_dir / "seed"
    dry_run_dir = args.work_dir / "workflow-dry-run"
    workflow_run_identity = args.work_dir / "latest-workflow-run.json"
    _clean_dir(seed_dir)
    _clean_dir(dry_run_dir)
    failures: list[str] = []

    _run(
        "Verify GitHub Pages configuration",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_pages_config.py"),
            "--repo",
            args.repo,
        ],
    )

    pages_branch_command = [
        sys.executable,
        str(SCRIPT_DIR / "check_performance_dashboard_pages_branch.py"),
        "--repo",
        args.repo,
    ]
    if args.require_published:
        pages_branch_command.append("--require-dashboard-path")
    try:
        _run("Verify GitHub Pages source branch path", pages_branch_command)
    except subprocess.CalledProcessError as exc:
        if not args.require_published:
            raise
        failures.append(_failure_message("Verify GitHub Pages source branch path", exc))

    _run(
        "Verify latest GitHub Pages build",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_pages_build.py"),
            "--repo",
            args.repo,
        ],
    )

    _run(
        "Verify Read the Docs source integration",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_website_sources.py"),
        ],
    )

    _run(
        "Verify dashboard PR body draft",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_pr_body.py"),
        ],
    )

    _run(
        "Verify service decision source URLs",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_service_sources.py"),
        ],
    )

    workflow_registration_command = [
        sys.executable,
        str(SCRIPT_DIR / "check_performance_dashboard_workflow_registration.py"),
        "--repo",
        args.repo,
    ]
    if not args.require_registered:
        workflow_registration_command.append("--allow-missing")
    publication_expectations: list[str] = []
    try:
        _run("Verify workflow registration preflight", workflow_registration_command)
    except subprocess.CalledProcessError as exc:
        if not (args.require_registered and args.require_published):
            raise
        failures.append(_failure_message("Verify workflow registration preflight", exc))

    if args.require_registered and args.require_published:
        workflow_run_command = [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_workflow_run.py"),
            "--repo",
            args.repo,
            "--require-dashboard-artifact",
            "--run-output-json",
            str(workflow_run_identity),
        ]
        try:
            _run("Verify latest dashboard workflow run", workflow_run_command)
        except subprocess.CalledProcessError as exc:
            failures.append(
                _failure_message("Verify latest dashboard workflow run", exc)
            )
        else:
            try:
                publication_expectations = _publication_expectations_from_workflow_run(
                    workflow_run_identity
                )
            except RuntimeError as exc:
                failures.append(str(exc))

    _run(
        "Download seed benchmark artifacts",
        [
            sys.executable,
            str(SCRIPT_DIR / "download_performance_dashboard_artifacts.py"),
            "--repo",
            args.repo,
            "--output-dir",
            str(seed_dir),
            "--branch",
            "main",
            "--skip-if-unavailable",
        ],
    )

    _run(
        "Render workflow-shaped dashboard dry run",
        [
            sys.executable,
            str(SCRIPT_DIR / "check_performance_dashboard_workflow_dry_run.py"),
            "--seed-root",
            str(seed_dir),
            "--work-dir",
            str(dry_run_dir),
        ],
    )

    publication_command = [
        sys.executable,
        str(SCRIPT_DIR / "check_performance_dashboard_publication.py"),
        "--dashboard-dir",
        str(dry_run_dir / "gh-pages" / "performance"),
    ]
    if not args.require_published:
        publication_command.append("--allow-unpublished")
    publication_command.extend(publication_expectations)
    try:
        _run("Verify publication endpoint preflight", publication_command)
    except subprocess.CalledProcessError as exc:
        if not args.require_published:
            raise
        failures.append(_failure_message("Verify publication endpoint preflight", exc))

    _raise_failures(failures)


def main() -> int:
    try:
        run_preflight(parse_args())
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        return 1
    print("performance dashboard launch preflight passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
