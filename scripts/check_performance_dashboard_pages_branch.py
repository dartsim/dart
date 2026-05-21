#!/usr/bin/env python3
"""Verify the GitHub Pages source branch can host the dashboard path."""

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

EXPECTED_DASHBOARD_FILES = {"data.json", "index.html", "status.json", "summary.md"}


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
        "--branch",
        default="gh-pages",
        help="GitHub Pages source branch to check.",
    )
    parser.add_argument(
        "--dashboard-path",
        default="performance",
        help="Dashboard path on the Pages source branch.",
    )
    parser.add_argument(
        "--branch-json",
        type=Path,
        default=None,
        help="Local branch API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--contents-json",
        type=Path,
        default=None,
        help="Local contents API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--contents-missing",
        action="store_true",
        help="Use a missing dashboard-path fixture. Used by tests.",
    )
    parser.add_argument(
        "--require-dashboard-path",
        action="store_true",
        help=(
            "Require the dashboard path to already exist and contain dashboard "
            "files. Use this after publication; omit it for first-publication "
            "preflight where the path may be absent."
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
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


def _read_json_url(
    url: str, token: str | None, timeout: float, *, allow_missing: bool = False
) -> Any:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if allow_missing and exc.code == 404:
            return None
        raise RuntimeError(f"Failed to read {url}: HTTP Error {exc.code}") from exc
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc


def _read_branch(args: argparse.Namespace) -> dict[str, Any]:
    if args.branch_json:
        data = json.loads(args.branch_json.read_text(encoding="utf-8"))
    else:
        repo = urllib.parse.quote(args.repo, safe="/")
        branch = urllib.parse.quote(args.branch, safe="")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/branches/{branch}",
            args.token,
            args.timeout,
        )
    if not isinstance(data, dict):
        raise RuntimeError("Branch API response did not contain an object.")
    return data


def _read_contents(args: argparse.Namespace) -> Any:
    if args.contents_missing:
        return None
    if args.contents_json:
        return json.loads(args.contents_json.read_text(encoding="utf-8"))

    repo = urllib.parse.quote(args.repo, safe="/")
    dashboard_path = urllib.parse.quote(args.dashboard_path.strip("/"), safe="/")
    branch = urllib.parse.quote(args.branch, safe="")
    return _read_json_url(
        f"{args.api_url.rstrip('/')}/repos/{repo}/contents/{dashboard_path}"
        f"?ref={branch}",
        args.token,
        args.timeout,
        allow_missing=True,
    )


def verify_pages_branch(args: argparse.Namespace) -> str:
    branch = _read_branch(args)
    if branch.get("name") != args.branch:
        raise RuntimeError(
            f"Expected Pages branch {args.branch!r}, got {branch.get('name')!r}."
        )

    contents = _read_contents(args)
    if contents is None:
        if args.require_dashboard_path:
            raise RuntimeError(
                f"Expected {args.branch}:{args.dashboard_path}/ to contain "
                "published dashboard files, but the path is absent."
            )
        return (
            f"pages branch verified: {args.branch} exists; "
            f"{args.dashboard_path}/ is absent and ready for first publication"
        )

    if isinstance(contents, dict):
        raise RuntimeError(
            f"Expected {args.dashboard_path}/ on {args.branch} to be absent "
            f"or a dashboard directory, got {contents.get('type')!r}."
        )

    if not isinstance(contents, list):
        raise RuntimeError("Dashboard path contents response did not contain a list.")

    names = {entry.get("name") for entry in contents if isinstance(entry, dict)}
    missing = sorted(EXPECTED_DASHBOARD_FILES - names)
    if missing:
        raise RuntimeError(
            f"Existing {args.branch}:{args.dashboard_path}/ does not look like "
            f"the performance dashboard; missing {', '.join(missing)}."
        )

    return (
        f"pages branch verified: {args.branch}:{args.dashboard_path}/ "
        "contains dashboard files"
    )


def main() -> int:
    try:
        print(verify_pages_branch(parse_args()))
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
