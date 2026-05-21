#!/usr/bin/env python3
"""Verify GitHub Pages is configured for the DART performance dashboard."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        default="dartsim/dart",
        help="GitHub repository in owner/name form.",
    )
    parser.add_argument(
        "--pages-api-url",
        default=None,
        help=(
            "GitHub Pages API URL or local JSON file. Defaults to "
            "https://api.github.com/repos/<repo>/pages."
        ),
    )
    parser.add_argument(
        "--expected-html-url",
        default="https://dartsim.github.io/dart/",
        help="Expected GitHub Pages site URL.",
    )
    parser.add_argument(
        "--expected-branch",
        default="gh-pages",
        help="Expected branch-source publishing branch.",
    )
    parser.add_argument(
        "--expected-path",
        default="/",
        help="Expected branch-source publishing path.",
    )
    parser.add_argument(
        "--expected-build-type",
        default="legacy",
        help="Expected Pages build type.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
    )
    return parser.parse_args()


def _pages_url(args: argparse.Namespace) -> str:
    if args.pages_api_url:
        return args.pages_api_url
    return f"https://api.github.com/repos/{args.repo}/pages"


def _read_json(location: str, timeout: float) -> dict[str, Any]:
    parsed = urllib.parse.urlparse(location)
    if parsed.scheme in {"http", "https"}:
        try:
            headers = {"User-Agent": "dart-performance-dashboard"}
            token = os.environ.get("GITHUB_TOKEN")
            if token:
                headers["Authorization"] = f"Bearer {token}"
            request = urllib.request.Request(location, headers=headers)
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            if parsed.netloc != "api.github.com" or shutil.which("gh") is None:
                raise RuntimeError(f"Failed to fetch {location}: {exc}") from exc
            gh_path = parsed.path
            if parsed.query:
                gh_path = f"{gh_path}?{parsed.query}"
            output = subprocess.check_output(["gh", "api", gh_path], text=True)
            return json.loads(output)
    return json.loads(Path(location).read_text(encoding="utf-8"))


def _expect(label: str, actual: Any, expected: Any) -> None:
    if actual != expected:
        raise RuntimeError(f"Expected Pages {label} {expected!r}, got {actual!r}")


def verify_pages_config(config: dict[str, Any], args: argparse.Namespace) -> None:
    _expect("html_url", config.get("html_url"), args.expected_html_url)
    _expect("build_type", config.get("build_type"), args.expected_build_type)
    status = config.get("status")
    if status not in {"built", "building", "queued"}:
        raise RuntimeError(f"Expected usable Pages status, got {status!r}")

    source = config.get("source")
    if not isinstance(source, dict):
        raise RuntimeError("Expected Pages source object")
    _expect("source.branch", source.get("branch"), args.expected_branch)
    _expect("source.path", source.get("path"), args.expected_path)


def main() -> int:
    args = parse_args()
    config = _read_json(_pages_url(args), args.timeout)
    verify_pages_config(config, args)
    print(
        "pages configuration verified: "
        f"{config.get('html_url')} "
        f"({config.get('build_type')} "
        f"{config.get('source', {}).get('branch')}"
        f"{config.get('source', {}).get('path')})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
