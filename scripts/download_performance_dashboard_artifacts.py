#!/usr/bin/env python3
"""Download recent benchmark artifacts for dashboard history seeding."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from io import BytesIO
from pathlib import Path
from typing import Any

DEFAULT_PREFIXES = [
    "performance-dashboard-raw",
    "collision-benchmark-guard",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo",
        default=os.environ.get("GITHUB_REPOSITORY"),
        help="Repository in owner/name form. Defaults to GITHUB_REPOSITORY.",
    )
    parser.add_argument(
        "--api-url",
        default=os.environ.get("GITHUB_API_URL", "https://api.github.com"),
        help="GitHub API URL. Defaults to GITHUB_API_URL or api.github.com.",
    )
    parser.add_argument(
        "--token",
        default=os.environ.get("GITHUB_TOKEN"),
        help="GitHub token used for artifact API access. Defaults to GITHUB_TOKEN.",
    )
    parser.add_argument(
        "--artifacts-json",
        type=Path,
        default=None,
        help="Local artifacts API JSON fixture. Used for tests and dry runs.",
    )
    parser.add_argument(
        "--name-prefix",
        action="append",
        default=[],
        help=(
            "Artifact name prefix to download. Defaults to performance dashboard "
            "and collision benchmark artifacts."
        ),
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=3,
        help="Maximum number of artifacts to download.",
    )
    parser.add_argument(
        "--branch",
        default=None,
        help=(
            "Only download artifacts produced by workflow runs from this branch. "
            "Artifacts without workflow_run.head_branch metadata are skipped "
            "when this is set."
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(".benchmark_seed"),
        help="Directory where artifact JSON files are extracted.",
    )
    parser.add_argument(
        "--skip-if-unavailable",
        action="store_true",
        help="Exit successfully if API configuration or artifacts are unavailable.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print selected artifacts without downloading them.",
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


class _AuthDroppingRedirectHandler(urllib.request.HTTPRedirectHandler):
    def redirect_request(
        self,
        req: urllib.request.Request,
        fp: Any,
        code: int,
        msg: str,
        headers: Any,
        newurl: str,
    ) -> urllib.request.Request | None:
        redirected = super().redirect_request(req, fp, code, msg, headers, newurl)
        if redirected is None:
            return None
        old_host = urllib.parse.urlparse(req.full_url).netloc
        new_host = urllib.parse.urlparse(newurl).netloc
        if old_host != new_host:
            redirected.remove_header("Authorization")
        return redirected


def _default_token() -> str | None:
    token = os.environ.get("GITHUB_TOKEN")
    if token or shutil.which("gh") is None:
        return token
    try:
        result = subprocess.run(
            ["gh", "auth", "token"],
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    token = result.stdout.strip()
    return token or None


def _read_json_url(url: str, token: str | None) -> dict[str, Any]:
    request = urllib.request.Request(url, headers=_headers(token))
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Failed to read {url}: {exc}") from exc


def _read_artifacts(args: argparse.Namespace) -> list[dict[str, Any]]:
    if args.artifacts_json:
        data = json.loads(args.artifacts_json.read_text(encoding="utf-8"))
    else:
        if not args.repo:
            raise RuntimeError("Repository is not configured.")
        repo = urllib.parse.quote(args.repo, safe="/")
        api_url = args.api_url.rstrip("/")
        data = _read_json_url(
            f"{api_url}/repos/{repo}/actions/artifacts?per_page=100",
            args.token,
        )
    artifacts = data.get("artifacts", [])
    if not isinstance(artifacts, list):
        raise RuntimeError("Artifacts API response did not contain a list.")
    return [artifact for artifact in artifacts if isinstance(artifact, dict)]


def _select_artifacts(
    artifacts: list[dict[str, Any]],
    prefixes: list[str],
    limit: int,
    branch: str | None = None,
) -> list[dict[str, Any]]:
    wanted_prefixes = prefixes or DEFAULT_PREFIXES
    selected = [
        artifact
        for artifact in artifacts
        if not artifact.get("expired")
        and any(
            str(artifact.get("name", "")).startswith(prefix)
            for prefix in wanted_prefixes
        )
        and artifact.get("archive_download_url")
        and (branch is None or _artifact_branch(artifact) == branch)
    ]
    selected.sort(
        key=lambda artifact: str(artifact.get("created_at", "")), reverse=True
    )
    return selected[: max(limit, 0)]


def _artifact_branch(artifact: dict[str, Any]) -> str | None:
    workflow_run = artifact.get("workflow_run")
    if isinstance(workflow_run, dict) and workflow_run.get("head_branch"):
        return str(workflow_run["head_branch"])
    if artifact.get("workflow_run_head_branch"):
        return str(artifact["workflow_run_head_branch"])
    return None


def _safe_member_name(name: str) -> str | None:
    path = Path(name)
    if path.is_absolute() or ".." in path.parts:
        return None
    if path.suffix != ".json":
        return None
    return path.name


def _download_zip(url: str, token: str | None) -> bytes:
    request = urllib.request.Request(url, headers=_headers(token))
    opener = urllib.request.build_opener(_AuthDroppingRedirectHandler)
    try:
        with opener.open(request, timeout=60) as response:
            return response.read()
    except (OSError, urllib.error.URLError) as exc:
        raise RuntimeError(f"Failed to download {url}: {exc}") from exc


def _extract_json_artifact(
    artifact: dict[str, Any],
    output_dir: Path,
    token: str | None,
) -> Path:
    name = str(artifact["name"])
    destination = output_dir / name
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)

    archive = _download_zip(str(artifact["archive_download_url"]), token)
    extracted = 0
    with zipfile.ZipFile(BytesIO(archive)) as zf:
        for member in zf.infolist():
            member_name = _safe_member_name(member.filename)
            if member_name is None:
                continue
            (destination / member_name).write_bytes(zf.read(member))
            extracted += 1

    if not extracted:
        raise RuntimeError(f"Artifact {name} did not contain benchmark JSON files.")
    return destination


def main() -> int:
    args = parse_args()
    if args.token is None:
        args.token = _default_token()
    try:
        artifacts = _read_artifacts(args)
        selected = _select_artifacts(
            artifacts,
            args.name_prefix,
            args.limit,
            branch=args.branch,
        )
    except RuntimeError as exc:
        if args.skip_if_unavailable:
            print(f"{exc} Skipping benchmark artifact history seed.")
            return 0
        raise SystemExit(str(exc)) from exc

    if not selected:
        message = "No matching benchmark artifacts found."
        if args.skip_if_unavailable:
            print(f"{message} Skipping benchmark artifact history seed.")
            return 0
        raise SystemExit(message)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    downloaded = 0
    for artifact in reversed(selected):
        name = artifact.get("name")
        if args.dry_run:
            print(f"would download benchmark artifact: {name}")
            downloaded += 1
            continue
        try:
            destination = _extract_json_artifact(artifact, args.output_dir, args.token)
        except RuntimeError as exc:
            if args.skip_if_unavailable:
                print(f"{exc} Skipping benchmark artifact {name}.")
                continue
            raise SystemExit(str(exc)) from exc
        print(f"downloaded benchmark artifact: {name} -> {destination}")
        downloaded += 1

    if not downloaded:
        message = "No benchmark artifacts were downloaded."
        if args.skip_if_unavailable:
            print(f"{message} Skipping benchmark artifact history seed.")
            return 0
        raise SystemExit(message)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
