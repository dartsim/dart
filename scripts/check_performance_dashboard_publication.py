#!/usr/bin/env python3
"""Check local and public readiness for the performance dashboard."""

from __future__ import annotations

import argparse
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path

from check_performance_dashboard_endpoint import (
    _default_data_url,
    _linked_url,
    _load_json,
    _load_status,
    _load_text,
    _verify_dashboard_page,
    _verify_data,
    _verify_status,
    _verify_summary,
)
from verify_performance_dashboard import verify_dashboard

CANONICAL_DASHBOARD_URL = "https://dartsim.github.io/dart/performance/"
CANONICAL_STATUS_URL = "https://dartsim.github.io/dart/performance/status.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dashboard-dir",
        type=Path,
        default=Path("build/performance-dashboard"),
        help="Generated local dashboard directory to verify.",
    )
    parser.add_argument(
        "--website-url",
        default="https://dart.readthedocs.io/en/latest/",
        help="Canonical DART website URL to check.",
    )
    parser.add_argument(
        "--website-dashboard-url",
        default=CANONICAL_DASHBOARD_URL,
        help="Hosted dashboard URL expected on the canonical website.",
    )
    parser.add_argument(
        "--website-status-url",
        default=CANONICAL_STATUS_URL,
        help="Hosted status.json URL expected on the canonical website.",
    )
    parser.add_argument(
        "--guide-url",
        default=(
            "https://dart.readthedocs.io/en/latest/"
            "community/performance_dashboard.html"
        ),
        help="Read the Docs performance dashboard guide URL to check.",
    )
    parser.add_argument(
        "--status-url",
        default="https://dartsim.github.io/dart/performance/status.json",
        help="Public dashboard status.json URL to check.",
    )
    parser.add_argument(
        "--data-url",
        default=None,
        help=(
            "Public dashboard data.json URL to check. Defaults to replacing "
            "the status URL filename with data.json."
        ),
    )
    parser.add_argument(
        "--dashboard-url",
        default=None,
        help=(
            "Public dashboard HTML URL to check when status.json is live. "
            "Defaults to links.dashboard_url from status.json."
        ),
    )
    parser.add_argument(
        "--summary-url",
        default=None,
        help=(
            "Public summary.md URL to check when status.json is live. "
            "Defaults to links.summary_url from status.json."
        ),
    )
    parser.add_argument(
        "--allow-unpublished",
        action="store_true",
        help="Exit successfully when the public dashboard endpoint is not live yet.",
    )
    parser.add_argument(
        "--expect-local-match",
        action="store_true",
        help=(
            "Require public status, data, dashboard HTML, and summary endpoints "
            "to match the local generated files."
        ),
    )
    parser.add_argument(
        "--expect-run-id",
        default=None,
        help="Expected public status latest_run_id value.",
    )
    parser.add_argument(
        "--expect-sha",
        default=None,
        help="Expected public status sha value.",
    )
    parser.add_argument(
        "--require-data",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require the public dashboard data.json endpoint when status.json is live.",
    )
    parser.add_argument(
        "--require-fresh",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Require the public dashboard endpoint freshness state to be fresh.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
    )
    return parser.parse_args()


def _read_text(location: str, timeout: float) -> str:
    parsed = urllib.parse.urlparse(location)
    if parsed.scheme:
        try:
            request = urllib.request.Request(
                location,
                headers={"User-Agent": "dart-performance-dashboard"},
            )
            with urllib.request.urlopen(request, timeout=timeout) as response:
                return response.read().decode("utf-8", errors="replace")
        except (OSError, ValueError, urllib.error.URLError) as exc:
            raise RuntimeError(f"Failed to fetch {location}: {exc}") from exc
    try:
        return Path(location).read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Failed to read {location}: {exc}") from exc


def _guide_link_snippet(guide_url: str) -> str:
    parsed = urllib.parse.urlparse(guide_url)
    path = parsed.path if parsed.scheme else guide_url
    parts = [part for part in path.split("/") if part]
    if len(parts) >= 2:
        return "/".join(parts[-2:])
    return path


def _check_website(
    website_url: str,
    timeout: float,
    *,
    dashboard_url: str | None = None,
    status_url: str | None = None,
    guide_url: str | None = None,
) -> None:
    body = _read_text(website_url, timeout)
    if not body.strip():
        raise RuntimeError(
            f"Canonical website returned an empty response: {website_url}"
        )
    missing: list[str] = []
    if dashboard_url and dashboard_url not in body:
        missing.append(dashboard_url)
    if status_url and status_url not in body:
        missing.append(status_url)
    if guide_url:
        guide_snippets = {
            guide_url,
            _guide_link_snippet(guide_url),
            "community/performance_dashboard.html",
        }
        if not any(snippet and snippet in body for snippet in guide_snippets):
            missing.append(guide_url)
    if missing:
        raise RuntimeError(
            "Canonical website is missing dashboard links: " + ", ".join(missing)
        )


def _default_dashboard_url(status_url: str) -> str:
    parsed = urllib.parse.urlparse(status_url)
    if parsed.scheme in {"http", "https"}:
        path = parsed.path.rsplit("/", 1)[0] + "/"
        return urllib.parse.urlunparse(
            parsed._replace(path=path, query="", fragment="")
        )
    if parsed.scheme == "file":
        return (
            Path(urllib.request.url2pathname(parsed.path))
            .with_name("index.html")
            .as_uri()
        )
    return str(Path(status_url).with_name("index.html"))


def _default_summary_url(status_url: str) -> str:
    parsed = urllib.parse.urlparse(status_url)
    if parsed.scheme in {"http", "https"}:
        path = parsed.path.rsplit("/", 1)[0] + "/summary.md"
        return urllib.parse.urlunparse(
            parsed._replace(path=path, query="", fragment="")
        )
    if parsed.scheme == "file":
        return (
            Path(urllib.request.url2pathname(parsed.path))
            .with_name("summary.md")
            .as_uri()
        )
    return str(Path(status_url).with_name("summary.md"))


def _record_fetch_failure(label: str, location: str, timeout: float) -> str | None:
    try:
        body = _read_text(location, timeout)
    except RuntimeError as exc:
        return f"{label}: {exc}"
    if not body.strip():
        return f"{label}: endpoint returned an empty response: {location}"
    return None


def _raise_failures(failures: list[str]) -> None:
    if failures:
        formatted = "\n".join(f"- {failure}" for failure in failures)
        raise SystemExit("Public dashboard publication checks failed:\n" + formatted)


def main() -> int:
    args = parse_args()
    verify_dashboard(args.dashboard_dir, require_measurements=False)
    local_status_path = args.dashboard_dir / "status.json"
    local_status = _load_status(str(local_status_path), args.timeout)
    print(f"local dashboard verified: {args.dashboard_dir}")

    failures: list[str] = []
    try:
        _check_website(
            args.website_url,
            args.timeout,
            dashboard_url=(
                None if args.allow_unpublished else args.website_dashboard_url
            ),
            status_url=None if args.allow_unpublished else args.website_status_url,
            guide_url=None if args.allow_unpublished else args.guide_url,
        )
    except RuntimeError as exc:
        if args.allow_unpublished:
            raise SystemExit(f"canonical website: {exc}") from exc
        failures.append(f"canonical website: {exc}")
    else:
        print(f"canonical website reachable: {args.website_url}")

    try:
        _check_website(args.guide_url, args.timeout)
        print(f"dashboard guide reachable: {args.guide_url}")
    except RuntimeError as exc:
        if args.allow_unpublished:
            print(f"dashboard guide not verified yet: {exc}")
        else:
            failures.append(f"dashboard guide: {exc}")

    try:
        public_status = _load_status(args.status_url, args.timeout)
    except RuntimeError as exc:
        if args.allow_unpublished:
            print(f"public dashboard not verified yet: {exc}")
            return 0
        failures.append(f"status.json: {exc}")
        public_data_url = args.data_url or _default_data_url(args.status_url)
        public_dashboard_url = args.dashboard_url or _default_dashboard_url(
            args.status_url
        )
        public_summary_url = args.summary_url or _default_summary_url(args.status_url)
        for label, location in (
            ("data.json", public_data_url),
            ("dashboard HTML", public_dashboard_url),
            ("summary.md", public_summary_url),
        ):
            failure = _record_fetch_failure(label, location, args.timeout)
            if failure:
                failures.append(failure)
        _raise_failures(failures)

    expected_run_id = args.expect_run_id
    if expected_run_id is None and args.expect_local_match:
        expected_run_id = local_status.get("latest_run_id")
    expected_sha = args.expect_sha
    if expected_sha is None and args.expect_local_match:
        expected_sha = local_status.get("sha")
    try:
        _verify_status(
            status=public_status,
            expected_run_id=expected_run_id,
            expected_sha=expected_sha,
            require_fresh=args.require_fresh,
        )
        if args.require_data or args.expect_local_match:
            data_url = args.data_url or _default_data_url(args.status_url)
            public_data = _load_json(data_url, args.timeout)
            expected_data = (
                _load_json(str(args.dashboard_dir / "data.json"), args.timeout)
                if args.expect_local_match
                else None
            )
            _verify_data(public_data, public_status, expected_data)
        dashboard_url = _linked_url(public_status, args.dashboard_url, "dashboard_url")
        expected_dashboard = (
            _load_text(str(args.dashboard_dir / "index.html"), args.timeout)
            if args.expect_local_match
            else None
        )
        _verify_dashboard_page(
            _load_text(dashboard_url, args.timeout),
            dashboard_url,
            expected_dashboard,
        )
        summary_url = _linked_url(public_status, args.summary_url, "summary_url")
        expected_summary = (
            _load_text(str(args.dashboard_dir / "summary.md"), args.timeout)
            if args.expect_local_match
            else None
        )
        _verify_summary(
            _load_text(summary_url, args.timeout),
            summary_url,
            expected_summary,
        )
    except RuntimeError as exc:
        failures.append(str(exc))

    _raise_failures(failures)

    freshness = public_status.get("freshness", {})
    print(
        "public dashboard endpoint verified: "
        f"{args.status_url} "
        f"(run={public_status.get('latest_run_id')}, "
        f"freshness={freshness.get('state')}, "
        f"data={'verified' if args.require_data or args.expect_local_match else 'not-checked'}, "
        "page=verified, summary=verified)"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
