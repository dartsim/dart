#!/usr/bin/env python3
"""Verify the latest GitHub Pages build is usable for the dashboard host."""

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
        "--source-branch",
        default="gh-pages",
        help="GitHub Pages source branch whose latest build should be current.",
    )
    parser.add_argument(
        "--build-json",
        type=Path,
        default=None,
        help="Local latest Pages build API JSON fixture. Used by tests.",
    )
    parser.add_argument(
        "--branch-json",
        type=Path,
        default=None,
        help="Local branch API JSON fixture. Used by tests.",
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


def _read_json_url(url: str, token: str | None, timeout: float) -> Any:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raise RuntimeError(f"Failed to read {url}: HTTP Error {exc.code}") from exc
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc


def _read_build(args: argparse.Namespace) -> dict[str, Any]:
    if args.build_json:
        data = json.loads(args.build_json.read_text(encoding="utf-8"))
    else:
        repo = urllib.parse.quote(args.repo, safe="/")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/pages/builds/latest",
            args.token,
            args.timeout,
        )
    if not isinstance(data, dict):
        raise RuntimeError("Latest Pages build API response did not contain an object.")
    return data


def _read_branch(args: argparse.Namespace) -> dict[str, Any]:
    if args.branch_json:
        data = json.loads(args.branch_json.read_text(encoding="utf-8"))
    else:
        repo = urllib.parse.quote(args.repo, safe="/")
        branch = urllib.parse.quote(args.source_branch, safe="")
        data = _read_json_url(
            f"{args.api_url.rstrip('/')}/repos/{repo}/branches/{branch}",
            args.token,
            args.timeout,
        )
    if not isinstance(data, dict):
        raise RuntimeError(
            "Pages source branch API response did not contain an object."
        )
    return data


def _branch_sha(branch: dict[str, Any]) -> str | None:
    commit = branch.get("commit")
    if isinstance(commit, dict):
        sha = commit.get("sha")
        if isinstance(sha, str):
            return sha
    sha = branch.get("sha")
    return sha if isinstance(sha, str) else None


def verify_pages_build(args: argparse.Namespace) -> str:
    build = _read_build(args)
    status = build.get("status")
    if status != "built":
        raise RuntimeError(
            f"Expected latest Pages build status 'built', got {status!r}."
        )

    error = build.get("error")
    if isinstance(error, dict) and error.get("message"):
        raise RuntimeError(f"Latest Pages build has error: {error.get('message')}")

    build_commit = build.get("commit")
    if not isinstance(build_commit, str) or not build_commit:
        raise RuntimeError("Latest Pages build did not report a commit SHA.")

    branch = _read_branch(args)
    branch_sha = _branch_sha(branch)
    if branch.get("name") not in {None, args.source_branch}:
        raise RuntimeError(
            f"Expected Pages source branch {args.source_branch!r}, "
            f"got {branch.get('name')!r}."
        )
    if not branch_sha:
        raise RuntimeError("Pages source branch did not report a commit SHA.")
    if build_commit != branch_sha:
        raise RuntimeError(
            "Latest Pages build is not current for the source branch: "
            f"build commit {build_commit}, {args.source_branch} head {branch_sha}."
        )

    updated_at = build.get("updated_at") or "unknown update time"
    return (
        "pages build verified: latest build is built for "
        f"{args.source_branch}@{build_commit} ({updated_at})"
    )


def main() -> int:
    try:
        print(verify_pages_build(parse_args()))
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
