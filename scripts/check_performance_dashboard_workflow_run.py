#!/usr/bin/env python3
"""Verify the latest performance dashboard workflow run succeeded on GitHub."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any


def _default_token() -> str | None:
    token = os.environ.get("GITHUB_TOKEN")
    if token:
        return token
    try:
        result = subprocess.run(
            ["gh", "auth", "token"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip() or None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        default=os.environ.get("GITHUB_REPOSITORY", "dartsim/dart"),
        help="GitHub repository in owner/name form.",
    )
    parser.add_argument(
        "--api-url",
        default=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
        help="GitHub API URL.",
    )
    parser.add_argument(
        "--token",
        default=_default_token(),
        help="GitHub token. Defaults to GITHUB_TOKEN or gh auth token.",
    )
    parser.add_argument(
        "--workflow",
        default="performance_dashboard.yml",
        help="Workflow filename or ID to query.",
    )
    parser.add_argument(
        "--branch",
        default="main",
        help="Branch whose latest dashboard workflow run must have succeeded.",
    )
    parser.add_argument(
        "--runs-json",
        type=Path,
        default=None,
        help="Local workflow-runs API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--artifacts-json",
        type=Path,
        default=None,
        help="Local workflow-run artifacts API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--require-dashboard-artifact",
        action=argparse.BooleanOptionalAction,
        default=True,
        help=(
            "Require the latest successful workflow run to expose the hosted-site "
            "dashboard artifact."
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
    )
    parser.add_argument(
        "--run-output-json",
        type=Path,
        default=None,
        help=(
            "Write the verified latest run identity to this JSON file so launch "
            "checks can require the public dashboard to match it."
        ),
    )
    return parser.parse_args()


def _headers(token: str | None) -> dict[str, str]:
    headers = {
        "Accept": "application/vnd.github+json",
        "User-Agent": "dart-performance-dashboard",
    }
    if token:
        headers["Authorization"] = f"Bearer {token}"
    return headers


def _read_json_url(url: str, token: str | None, timeout: float) -> dict[str, Any]:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            data = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            raise RuntimeError(
                "Performance dashboard workflow runs are not available yet. "
                f"GitHub returned HTTP 404 for {url}. This usually means "
                "performance_dashboard.yml has not landed on main or GitHub "
                "Actions has not registered it."
            ) from exc
        raise RuntimeError(f"Failed to read {url}: HTTP Error {exc.code}") from exc
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected JSON object from {url}.")
    return data


def _read_runs(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.runs_json:
        data = json.loads(args.runs_json.read_text(encoding="utf-8"))
    else:
        repo = urllib.parse.quote(args.repo, safe="/")
        workflow = urllib.parse.quote(args.workflow, safe="")
        branch = urllib.parse.quote(args.branch, safe="")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/actions/workflows/"
            f"{workflow}/runs?branch={branch}&per_page=20",
            args.token,
            args.timeout,
        )
    runs = data.get("workflow_runs", [])
    if not isinstance(runs, list):
        raise RuntimeError("Workflow runs API response did not contain a list.")
    return [run for run in runs if isinstance(run, dict)]


def _read_artifacts(
    args: argparse.Namespace, run: dict[str, Any]
) -> list[dict[str, Any]]:
    if args.artifacts_json:
        data = json.loads(args.artifacts_json.read_text(encoding="utf-8"))
    else:
        run_id = run.get("id")
        if run_id is None:
            raise RuntimeError("Latest performance dashboard workflow run has no ID.")
        repo = urllib.parse.quote(args.repo, safe="/")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/actions/runs/"
            f"{run_id}/artifacts?per_page=100",
            args.token,
            args.timeout,
        )
    artifacts = data.get("artifacts", [])
    if not isinstance(artifacts, list):
        raise RuntimeError(
            "Workflow run artifacts API response did not contain a list."
        )
    return [artifact for artifact in artifacts if isinstance(artifact, dict)]


def _latest_run(runs: list[dict[str, Any]], branch: str) -> dict[str, Any]:
    if not runs:
        raise RuntimeError(
            f"No performance dashboard workflow runs found for branch {branch!r}."
        )

    run = runs[0]
    head_branch = run.get("head_branch")
    if head_branch is not None and head_branch != branch:
        raise RuntimeError(
            "Latest performance dashboard workflow run was for branch "
            f"{head_branch!r}, expected {branch!r}."
        )
    return run


def _dashboard_artifact(
    run: dict[str, Any], artifacts: list[dict[str, Any]]
) -> dict[str, Any]:
    run_id = run.get("id")
    if run_id is None:
        raise RuntimeError("Latest performance dashboard workflow run has no ID.")

    prefix = f"performance-dashboard-site-{run_id}-"
    matching = [
        artifact
        for artifact in artifacts
        if isinstance(artifact.get("name"), str) and artifact["name"].startswith(prefix)
    ]
    if not matching:
        raise RuntimeError(
            "Latest performance dashboard workflow run did not upload a hosted-site "
            f"dashboard artifact with prefix {prefix!r}."
        )

    for artifact in matching:
        if not artifact.get("expired") and artifact.get("archive_download_url"):
            return artifact

    names = ", ".join(str(artifact.get("name", "<unnamed>")) for artifact in matching)
    raise RuntimeError(
        "Latest performance dashboard workflow run has no usable hosted-site "
        f"dashboard artifact. Matching artifacts: {names}."
    )


def verify_latest_successful_run(args: argparse.Namespace) -> dict[str, Any]:
    run = _latest_run(_read_runs(args), args.branch)
    status = run.get("status")
    if status != "completed":
        raise RuntimeError(
            "Latest performance dashboard workflow run has status "
            f"{status!r}; expected 'completed'."
        )
    conclusion = run.get("conclusion")
    if conclusion != "success":
        raise RuntimeError(
            "Latest performance dashboard workflow run has conclusion "
            f"{conclusion!r}; expected 'success'."
        )
    if not (run.get("html_url") or run.get("url")):
        raise RuntimeError("Latest performance dashboard workflow run has no URL.")

    if args.require_dashboard_artifact:
        artifact = _dashboard_artifact(run, _read_artifacts(args, run))
        run = dict(run)
        run["dashboard_artifact"] = artifact
    return run


def _expected_dashboard_run_id(run: dict[str, Any]) -> str | None:
    run_id = run.get("id")
    run_attempt = run.get("run_attempt")
    if run_id is None or run_attempt is None:
        return None
    return f"{run_id}-{run_attempt}"


def _run_output(run: dict[str, Any]) -> dict[str, Any]:
    output: dict[str, Any] = {
        "id": run.get("id"),
        "run_attempt": run.get("run_attempt"),
        "expected_dashboard_run_id": _expected_dashboard_run_id(run),
        "head_sha": run.get("head_sha"),
        "head_branch": run.get("head_branch"),
        "html_url": run.get("html_url"),
        "url": run.get("url"),
    }
    artifact = run.get("dashboard_artifact")
    if isinstance(artifact, dict):
        output["dashboard_artifact"] = {
            "name": artifact.get("name"),
            "archive_download_url": artifact.get("archive_download_url"),
            "expired": artifact.get("expired"),
        }
    return output


def _write_run_output(path: Path, run: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_run_output(run), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    try:
        run = verify_latest_successful_run(args)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    if args.run_output_json:
        _write_run_output(args.run_output_json, run)

    artifact = run.get("dashboard_artifact")
    artifact_suffix = ""
    if isinstance(artifact, dict):
        artifact_suffix = f" artifact={artifact.get('name', 'unknown')}"
    print(
        "performance dashboard workflow run succeeded: "
        f"{run.get('id', 'unknown')} {run.get('html_url') or run.get('url')}"
        f"{artifact_suffix}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
