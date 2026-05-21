#!/usr/bin/env python3
"""Verify the public performance dashboard status endpoint."""

from __future__ import annotations

import argparse
import json
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

from check_performance_dashboard_service_sources import EXPECTED_URL_SNIPPETS

SCHEMA_VERSION = 1
EXPECTED_LINKS = {
    "canonical_website_url": "https://dart.readthedocs.io/en/latest/",
    "dashboard_url": "https://dartsim.github.io/dart/performance/",
    "status_url": "https://dartsim.github.io/dart/performance/status.json",
    "data_url": "https://dartsim.github.io/dart/performance/data.json",
    "summary_url": "https://dartsim.github.io/dart/performance/summary.md",
    "guide_url": (
        "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html"
    ),
}
EXPECTED_SERVICE_DECISIONS = {
    "primary-dashboard": "DART-owned GitHub Pages",
    "external-history-and-thresholds": "Bencher Cloud or Self-Hosted",
    "microbenchmark-pr-pilot": "CodSpeed",
    "off-the-shelf-pages-action": "github-action-benchmark",
    "static-benchmark-dashboard-framework": "Airspeed Velocity",
    "self-hosted-performance-tracker": "LLVM LNT",
    "self-hosted-fallback": "Conbench",
    "public-ecosystem-campaigns": "OpenBenchmarking.org / Phoronix Test Suite",
}
EXPECTED_SERVICE_DECISION_SUMMARY = {
    "best_option": (
        "Use DART-owned GitHub Pages as the canonical static dashboard and "
        "durable data host, with Bencher as an opt-in external "
        "history/threshold companion after maintainer approval."
    ),
    "selected_host": "DART-owned GitHub Pages",
    "selected_host_role": "primary-dashboard",
    "recommended_external_service": "Bencher Cloud or Self-Hosted",
    "recommended_external_role": "external-history-and-thresholds",
    "free_open_source_fit": (
        "The selected GitHub Pages host is free for DART's public GitHub "
        "repository, and the recommended Bencher companion has a free "
        "public/open-source path."
    ),
}
EXPECTED_SERVICE_EVIDENCE_URLS = sorted(EXPECTED_URL_SNIPPETS)
EXPECTED_PUBLICATION_CONTRACT = {
    "host": "GitHub Pages",
    "site_url": "https://dartsim.github.io/dart/performance/",
    "source_branch": "gh-pages",
    "source_path": "/",
    "dashboard_path": "performance/",
    "build_type": "legacy branch-source",
    "publisher_workflow": ".github/workflows/performance_dashboard.yml",
    "workflow_name": "Performance Dashboard",
    "publish_ref": "refs/heads/main",
    "schedule_crons": ["30 3 * * 0,3"],
    "expected_update_interval_hours": 96.0,
    "stale_after_hours": 120.0,
    "requires_pages_build_request": True,
    "required_permissions": [
        "actions: read",
        "contents: write",
        "pages: write",
    ],
    "required_endpoints": [
        "https://dartsim.github.io/dart/performance/",
        "https://dartsim.github.io/dart/performance/status.json",
        "https://dartsim.github.io/dart/performance/data.json",
        "https://dartsim.github.io/dart/performance/summary.md",
    ],
    "required_website_links": [
        "https://dartsim.github.io/dart/performance/",
        "https://dartsim.github.io/dart/performance/status.json",
        "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html",
    ],
    "required_workflow_artifacts": [
        "performance-dashboard-site-<run_id>-<run_attempt>",
    ],
    "launch_checks": [
        "pixi run check-bm-dashboard-launch-preflight",
        "pixi run check-bm-dashboard-pages-branch",
        "pixi run check-bm-dashboard-pages-build",
        "pixi run check-bm-dashboard-workflow-registration",
        "pixi run check-bm-dashboard-workflow-run",
        "pixi run check-bm-dashboard-launch-live",
    ],
}
EXPECTED_UPDATE_TRIGGERS = [
    "path-scoped main push",
    "scheduled workflow",
    "manual workflow_dispatch",
]
EXPECTED_COMPARISON_SURFACES = {
    "collision",
    "common",
    "dynamics",
    "lcp",
    "math",
    "simd",
    "simulation",
    "compute",
    "gpu",
}
EXPECTED_COVERAGE_STATES = {"live", "primary-only", "queued"}
EXPECTED_EXTERNAL_COMPETITOR_BACKENDS = {
    "Chrono",
    "Drake",
    "Gazebo",
    "GzPhysics",
    "Isaac",
    "IsaacGym",
    "MuJoCo",
    "PyBullet",
    "RaiSim",
    "Simbody",
}
EXPECTED_DASHBOARD_PAGE_SNIPPETS = [
    "DART Performance Dashboard",
    'href="status.json"',
    'href="data.json"',
    'href="summary.md"',
    "Dashboard Service Decision",
    "DART-owned GitHub Pages",
    "Bencher Cloud or Self-Hosted",
    "Airspeed Velocity",
    "LLVM LNT",
    "OpenBenchmarking.org / Phoronix Test Suite",
    "Publication Contract",
    ".github/workflows/performance_dashboard.yml",
    "refs/heads/main",
    "schedule_crons",
    "30 3 * * 0,3",
    "actions: read",
    "pages: write",
    "required_website_links",
    "required_workflow_artifacts",
    "performance-dashboard-site-&lt;run_id&gt;-&lt;run_attempt&gt;",
    "Comparison Metric Contract",
    "backend_ns / primary_ns",
    "comparison_input_contract",
    "comparison_metric_contract",
    "External Competitor Status",
    "external_competitor_status",
    "Testbed Summary",
    "testbed_summary",
    "Service Decision Summary",
    "service_decision_summary",
    "best_option",
    "Trend Summary",
    "trend_summary",
    "BM_Distance_BoxSphere_MuJoCo",
    "Reference And Competitor Coverage",
]
EXPECTED_SUMMARY_SNIPPETS = [
    "DART Performance Dashboard Summary",
    "## Dashboard Service Decision",
    "DART-owned GitHub Pages",
    "Bencher Cloud or Self-Hosted",
    "Airspeed Velocity",
    "LLVM LNT",
    "OpenBenchmarking.org / Phoronix Test Suite",
    "## Publication Contract",
    ".github/workflows/performance_dashboard.yml",
    "refs/heads/main",
    "schedule_crons",
    "30 3 * * 0,3",
    "actions: read",
    "pages: write",
    "Required website links",
    "performance-dashboard-site-<run_id>-<run_attempt>",
    "## Comparison Input Contract",
    "BM_Distance_BoxSphere_MuJoCo",
    "## Comparison Metric Contract",
    "backend_ns / primary_ns",
    "## External Competitor Status",
    "live competitor",
    "## Testbed Summary",
    "## Service Decision Summary",
    "best_option",
    "## Trend Summary",
    "## Reference And Competitor Coverage",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "status_url",
        help="Public dashboard status.json URL to verify.",
    )
    parser.add_argument(
        "--expect-run-id",
        default=None,
        help="Expected latest_run_id value.",
    )
    parser.add_argument(
        "--expect-sha",
        default=None,
        help="Expected latest run SHA value.",
    )
    parser.add_argument(
        "--expect-status-file",
        type=Path,
        default=None,
        help=(
            "Local generated status.json whose latest_run_id and sha should "
            "match the public endpoint."
        ),
    )
    parser.add_argument(
        "--data-url",
        default=None,
        help=(
            "Public dashboard data.json URL to verify. Defaults to replacing "
            "the status URL filename with data.json when data verification is "
            "requested."
        ),
    )
    parser.add_argument(
        "--dashboard-url",
        default=None,
        help=(
            "Public dashboard HTML URL to verify when --require-dashboard-page "
            "is set. Defaults to links.dashboard_url from status.json."
        ),
    )
    parser.add_argument(
        "--summary-url",
        default=None,
        help=(
            "Public summary.md URL to verify when --require-summary is set. "
            "Defaults to links.summary_url from status.json."
        ),
    )
    parser.add_argument(
        "--expect-data-file",
        type=Path,
        default=None,
        help="Local generated data.json expected to match the public data endpoint.",
    )
    parser.add_argument(
        "--expect-dashboard-file",
        type=Path,
        default=None,
        help="Local generated index.html expected to match the public dashboard page.",
    )
    parser.add_argument(
        "--expect-summary-file",
        type=Path,
        default=None,
        help="Local generated summary.md expected to match the public summary page.",
    )
    parser.add_argument(
        "--require-data",
        action="store_true",
        help="Fail unless the public data.json endpoint is reachable and consistent.",
    )
    parser.add_argument(
        "--require-dashboard-page",
        action="store_true",
        help="Fail unless the public dashboard HTML page is reachable.",
    )
    parser.add_argument(
        "--require-summary",
        action="store_true",
        help="Fail unless the public summary.md page is reachable.",
    )
    parser.add_argument(
        "--attempts",
        type=int,
        default=1,
        help="Fetch attempts before failing.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=5.0,
        help="Seconds to wait between attempts.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Per-request timeout in seconds.",
    )
    parser.add_argument(
        "--require-fresh",
        action="store_true",
        help="Fail unless freshness.state is fresh.",
    )
    return parser.parse_args()


def _load_text(location: str, timeout: float) -> str:
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
    path = Path(location)
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise RuntimeError(f"Failed to read {path}: {exc}") from exc


def _load_json(location: str, timeout: float) -> dict[str, Any]:
    body = _load_text(location, timeout)

    try:
        data = json.loads(body)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Invalid JSON at {location}: {exc}") from exc

    if not isinstance(data, dict):
        raise RuntimeError(f"Expected JSON object at {location}.")
    return data


def _load_status(url: str, timeout: float) -> dict[str, Any]:
    return _load_json(url, timeout)


def _default_data_url(status_url: str) -> str:
    parsed = urllib.parse.urlparse(status_url)
    if parsed.scheme in {"http", "https"}:
        path = parsed.path.rsplit("/", 1)[0] + "/data.json"
        return urllib.parse.urlunparse(
            parsed._replace(path=path, query="", fragment="")
        )
    if parsed.scheme == "file":
        return (
            Path(urllib.request.url2pathname(parsed.path))
            .with_name("data.json")
            .as_uri()
        )
    return str(Path(status_url).with_name("data.json"))


def _linked_url(
    status: dict[str, Any],
    explicit_url: str | None,
    link_key: str,
) -> str:
    if explicit_url:
        return explicit_url
    links = status.get("links")
    if not isinstance(links, dict) or not links.get(link_key):
        raise RuntimeError(f"Dashboard status is missing {link_key}.")
    return str(links[link_key])


def _verify_dashboard_page(
    body: str,
    location: str,
    expected_body: str | None = None,
) -> None:
    if not body.strip():
        raise RuntimeError(f"Dashboard page returned an empty response: {location}")
    missing = [
        snippet for snippet in EXPECTED_DASHBOARD_PAGE_SNIPPETS if snippet not in body
    ]
    if missing:
        raise RuntimeError(
            f"Dashboard page does not look like the generated dashboard: {location} "
            f"(missing: {', '.join(missing)})"
        )
    if expected_body is not None and body != expected_body:
        raise RuntimeError(
            "Dashboard page endpoint does not match expected index.html."
        )


def _verify_summary(
    body: str,
    location: str,
    expected_body: str | None = None,
) -> None:
    if not body.strip():
        raise RuntimeError(f"Dashboard summary returned an empty response: {location}")
    missing = [snippet for snippet in EXPECTED_SUMMARY_SNIPPETS if snippet not in body]
    if missing:
        raise RuntimeError(
            f"Dashboard summary does not look like generated summary.md: {location} "
            f"(missing: {', '.join(missing)})"
        )
    if expected_body is not None and body != expected_body:
        raise RuntimeError(
            "Dashboard summary endpoint does not match expected summary.md."
        )


def _verify_status(
    status: dict[str, Any],
    expected_run_id: str | None,
    expected_sha: str | None,
    require_fresh: bool,
) -> None:
    if status.get("schema_version") != SCHEMA_VERSION:
        raise RuntimeError(
            "Unsupported dashboard schema version: " f"{status.get('schema_version')!r}"
        )

    latest_run_id = status.get("latest_run_id")
    if not latest_run_id:
        raise RuntimeError("Dashboard status is missing latest_run_id.")
    if expected_run_id and latest_run_id != expected_run_id:
        raise RuntimeError(
            f"Expected latest_run_id {expected_run_id!r}, got {latest_run_id!r}."
        )

    sha = status.get("sha")
    if expected_sha and sha != expected_sha:
        raise RuntimeError(f"Expected sha {expected_sha!r}, got {sha!r}.")

    files = status.get("dashboard_files")
    required = {"index.html", "data.json", "summary.md", "status.json"}
    if not isinstance(files, list) or not required.issubset(files):
        raise RuntimeError("Dashboard status file manifest is incomplete.")

    freshness = status.get("freshness")
    allowed_freshness = {"fresh", "stale", "unknown", "no-runs"}
    if (
        not isinstance(freshness, dict)
        or freshness.get("state") not in allowed_freshness
    ):
        raise RuntimeError("Dashboard freshness status is missing or invalid.")
    if require_fresh and freshness.get("state") != "fresh":
        raise RuntimeError(
            "Dashboard status is not fresh: "
            f"{freshness.get('state')!r} "
            f"(latest_run_age_hours={freshness.get('latest_run_age_hours')!r})."
        )
    if freshness.get("update_triggers") != EXPECTED_UPDATE_TRIGGERS:
        raise RuntimeError(
            "Dashboard freshness metadata does not advertise the expected "
            "main-push, scheduled, and manual update triggers."
        )

    _verify_links(status)
    _verify_service_decision(status)
    _verify_service_decision_summary(status)
    _verify_publication_contract(status)
    _verify_comparison_input_contract(status)
    _verify_comparison_metric_contract(status)
    _verify_trend_summary(status)
    _verify_testbed_summary(status)


def _verify_service_decision(status: dict[str, Any]) -> None:
    rows = status.get("service_decision")
    if not isinstance(rows, list):
        raise RuntimeError("Dashboard status is missing service decision rows.")

    by_role = {str(item.get("role")): item for item in rows if isinstance(item, dict)}
    missing = [role for role in EXPECTED_SERVICE_DECISIONS if role not in by_role]
    if missing:
        raise RuntimeError(
            "Dashboard status is missing service decision roles: " + ", ".join(missing)
        )

    mismatches = [
        f"{role}={by_role[role].get('service')!r}"
        for role, expected in EXPECTED_SERVICE_DECISIONS.items()
        if by_role[role].get("service") != expected
    ]
    if mismatches:
        raise RuntimeError(
            "Dashboard status service decisions do not match the selected DART "
            "architecture: " + ", ".join(mismatches)
        )
    required_fields = [
        "cost_model",
        "data_owner",
        "ci_integration",
        "approval_gate",
        "evidence_urls",
    ]
    missing_fields = [
        f"{role}.{field}"
        for role, item in by_role.items()
        for field in required_fields
        if field not in item
    ]
    if missing_fields:
        raise RuntimeError(
            "Dashboard status service decisions are missing evaluation fields: "
            + ", ".join(missing_fields)
        )
    for role, item in by_role.items():
        evidence_urls = item.get("evidence_urls")
        if not isinstance(evidence_urls, list) or not evidence_urls:
            raise RuntimeError(
                f"Dashboard status service decision {role!r} is missing evidence URLs."
            )
        if not all(str(url).startswith("https://") for url in evidence_urls):
            raise RuntimeError(
                f"Dashboard status service decision {role!r} has invalid evidence URLs."
            )
        unconfigured = [
            str(url) for url in evidence_urls if str(url) not in EXPECTED_URL_SNIPPETS
        ]
        if unconfigured:
            raise RuntimeError(
                f"Dashboard status service decision {role!r} has unconfigured "
                "evidence URLs: " + ", ".join(unconfigured)
            )
        for field in ("cost_model", "data_owner", "ci_integration", "approval_gate"):
            if not isinstance(item.get(field), str) or not item.get(field):
                raise RuntimeError(
                    f"Dashboard status service decision {role!r} has invalid "
                    f"{field}."
                )
    observed_urls = sorted(
        {str(url) for item in by_role.values() for url in item.get("evidence_urls", [])}
    )
    if observed_urls != EXPECTED_SERVICE_EVIDENCE_URLS:
        missing_urls = sorted(set(EXPECTED_SERVICE_EVIDENCE_URLS) - set(observed_urls))
        extra_urls = sorted(set(observed_urls) - set(EXPECTED_SERVICE_EVIDENCE_URLS))
        details: list[str] = []
        if missing_urls:
            details.append("missing: " + ", ".join(missing_urls))
        if extra_urls:
            details.append("extra: " + ", ".join(extra_urls))
        raise RuntimeError(
            "Dashboard status service decision evidence URLs do not match the "
            "verified service-source contract: " + "; ".join(details)
        )
    for role in ("primary-dashboard", "external-history-and-thresholds"):
        cost_model = str(by_role[role].get("cost_model", "")).lower()
        if "free" not in cost_model:
            raise RuntimeError(
                f"Dashboard status service decision {role!r} does not document "
                "a free/public-open-source cost path."
            )


def _verify_service_decision_summary(payload: dict[str, Any]) -> None:
    summary = payload.get("service_decision_summary")
    if not isinstance(summary, dict):
        raise RuntimeError("Dashboard is missing service decision summary.")
    missing = [key for key in EXPECTED_SERVICE_DECISION_SUMMARY if key not in summary]
    if missing:
        raise RuntimeError(
            "Dashboard service decision summary is missing keys: " + ", ".join(missing)
        )
    mismatches = [
        f"{key}={summary.get(key)!r}"
        for key, expected in EXPECTED_SERVICE_DECISION_SUMMARY.items()
        if summary.get(key) != expected
    ]
    if mismatches:
        raise RuntimeError(
            "Dashboard service decision summary does not match the selected "
            "DART architecture: " + ", ".join(mismatches)
        )

    rows = payload.get("service_decision")
    by_role = {str(item.get("role")): item for item in rows if isinstance(item, dict)}
    selected = by_role.get("primary-dashboard", {})
    recommended = by_role.get("external-history-and-thresholds", {})
    if summary.get("canonical_data_owner") != selected.get("data_owner"):
        raise RuntimeError(
            "Dashboard service decision summary canonical data owner does not "
            "match the selected host row."
        )
    if summary.get("selected_host") != selected.get("service"):
        raise RuntimeError(
            "Dashboard service decision summary selected host does not match "
            "service decision rows."
        )
    if summary.get("recommended_external_service") != recommended.get("service"):
        raise RuntimeError(
            "Dashboard service decision summary recommended external service "
            "does not match service decision rows."
        )
    expected_deferred = [
        str(item.get("service"))
        for item in rows
        if isinstance(item, dict)
        and str(item.get("state")) in {"deferred-pilot", "not-selected"}
    ]
    if summary.get("deferred_services") != expected_deferred:
        raise RuntimeError(
            "Dashboard service decision summary deferred services do not match "
            "service decision rows."
        )
    expected_fallback = [
        str(item.get("service"))
        for item in rows
        if isinstance(item, dict) and str(item.get("state")) == "fallback"
    ]
    if summary.get("fallback_services") != expected_fallback:
        raise RuntimeError(
            "Dashboard service decision summary fallback services do not match "
            "service decision rows."
        )
    approval_boundary = str(summary.get("approval_boundary", "")).lower()
    if (
        "maintainer approval" not in approval_boundary
        or "workflow dispatch" not in approval_boundary
    ):
        raise RuntimeError(
            "Dashboard service decision summary is missing the remote approval "
            "boundary."
        )


def _verify_publication_contract(payload: dict[str, Any]) -> None:
    contract = payload.get("publication_contract")
    if not isinstance(contract, dict):
        raise RuntimeError("Dashboard is missing publication contract.")

    mismatches = [
        f"{key}={contract.get(key)!r}"
        for key, expected in EXPECTED_PUBLICATION_CONTRACT.items()
        if contract.get(key) != expected
    ]
    if mismatches:
        raise RuntimeError(
            "Dashboard publication contract does not match expected Pages "
            "publisher contract: " + ", ".join(mismatches)
        )

    approval_boundary = contract.get("approval_boundary")
    if not isinstance(approval_boundary, str) or not approval_boundary:
        raise RuntimeError("Dashboard publication contract approval boundary missing.")
    boundary_lower = approval_boundary.lower()
    if (
        "maintainer approval" not in boundary_lower
        or "workflow dispatch" not in boundary_lower
    ):
        raise RuntimeError(
            "Dashboard publication contract does not describe the maintainer "
            "approval boundary."
        )

    links = payload.get("links")
    if isinstance(links, dict):
        expected_endpoints = [
            links.get("dashboard_url"),
            links.get("status_url"),
            links.get("data_url"),
            links.get("summary_url"),
        ]
        if contract.get("required_endpoints") != expected_endpoints:
            raise RuntimeError(
                "Dashboard publication contract required endpoints do not match "
                "public links."
            )


def _verify_comparison_input_contract(payload: dict[str, Any]) -> None:
    contract = payload.get("comparison_input_contract")
    if not isinstance(contract, dict):
        raise RuntimeError("Dashboard is missing comparison input contract.")
    if contract.get("format") != "Google Benchmark JSON":
        raise RuntimeError("Dashboard comparison input contract format is invalid.")
    supported = contract.get("supported_external_backends")
    if not isinstance(supported, list):
        raise RuntimeError(
            "Dashboard comparison input contract is missing supported backends."
        )
    missing = sorted(EXPECTED_EXTERNAL_COMPETITOR_BACKENDS - set(supported))
    if missing:
        raise RuntimeError(
            "Dashboard comparison input contract is missing external backends: "
            + ", ".join(missing)
        )
    examples = contract.get("example_rows")
    if not isinstance(examples, list) or not examples:
        raise RuntimeError(
            "Dashboard comparison input contract is missing example rows."
        )
    required_text_fields = [
        "backend_suffix_rule",
        "primary_backend_rule",
        "claim_rule",
    ]
    missing_fields = [
        field
        for field in required_text_fields
        if not isinstance(contract.get(field), str) or not contract.get(field)
    ]
    if missing_fields:
        raise RuntimeError(
            "Dashboard comparison input contract is missing fields: "
            + ", ".join(missing_fields)
        )
    if "_<Backend>" not in contract["backend_suffix_rule"]:
        raise RuntimeError(
            "Dashboard comparison input contract does not describe the backend "
            "suffix rule."
        )
    if "live competitor" not in contract["claim_rule"]:
        raise RuntimeError(
            "Dashboard comparison input contract does not describe the live "
            "competitor claim rule."
        )


def _verify_comparison_metric_contract(payload: dict[str, Any]) -> None:
    contract = payload.get("comparison_metric_contract")
    if not isinstance(contract, dict):
        raise RuntimeError("Dashboard is missing comparison metric contract.")
    expected = {
        "time_unit": "nanoseconds",
        "primary_time_field": "primary_ns",
        "backend_time_field": "backend_ns",
        "ratio_field": "backend_vs_primary_ratio",
        "ratio_formula": "backend_ns / primary_ns",
        "latest_comparison_ratio_field": "ratio",
        "latest_comparison_ratio_formula": "native_ns / best_reference_ns",
    }
    mismatches = [
        f"{key}={contract.get(key)!r}"
        for key, value in expected.items()
        if contract.get(key) != value
    ]
    if mismatches:
        raise RuntimeError(
            "Dashboard comparison metric contract has invalid fields: "
            + ", ".join(mismatches)
        )
    for field in ("interpretation", "latest_comparison_interpretation"):
        value = contract.get(field)
        if not isinstance(value, str) or "ratio < 1" not in value:
            raise RuntimeError(
                f"Dashboard comparison metric contract {field} is missing "
                "ratio direction."
            )
    status_meanings = contract.get("status_meanings")
    if not isinstance(status_meanings, dict):
        raise RuntimeError(
            "Dashboard comparison metric contract status meanings are missing."
        )
    for status in ("lead", "behind", "faster-than-primary", "slower-than-primary"):
        if not status_meanings.get(status):
            raise RuntimeError(
                "Dashboard comparison metric contract is missing status meaning "
                f"for {status}."
            )


def _verify_trend_summary(payload: dict[str, Any]) -> None:
    summary = payload.get("trend_summary")
    if not isinstance(summary, dict):
        raise RuntimeError("Dashboard is missing trend summary.")

    runs = payload.get("runs")
    if isinstance(runs, list):
        expected_history_runs = len(runs)
    else:
        expected_history_runs = payload.get("runs")
    if summary.get("history_runs") != expected_history_runs:
        raise RuntimeError("Dashboard trend summary history run count is invalid.")

    primary_rows = summary.get("primary_trend_rows")
    reference_rows = summary.get("reference_trend_rows")
    if not isinstance(primary_rows, int) or primary_rows < 0:
        raise RuntimeError("Dashboard trend summary primary row count is invalid.")
    if not isinstance(reference_rows, int) or reference_rows < 0:
        raise RuntimeError("Dashboard trend summary reference row count is invalid.")
    if payload.get(
        "reference_trend_rows"
    ) is not None and reference_rows != payload.get("reference_trend_rows"):
        raise RuntimeError(
            "Dashboard trend summary reference row count does not match status."
        )

    for field, expected_rows in (
        ("primary_status_counts", primary_rows),
        ("primary_threshold_counts", primary_rows),
        ("reference_status_counts", reference_rows),
        ("reference_threshold_counts", reference_rows),
    ):
        counts = summary.get(field)
        if not isinstance(counts, dict):
            raise RuntimeError(f"Dashboard trend summary missing {field}.")
        if sum(int(value) for value in counts.values()) != expected_rows:
            raise RuntimeError(f"Dashboard trend summary {field} does not add up.")

    for field in (
        "primary_largest_regression",
        "primary_largest_improvement",
        "reference_largest_regression",
        "reference_largest_improvement",
    ):
        snapshot = summary.get(field)
        if snapshot is None:
            continue
        if not isinstance(snapshot, dict):
            raise RuntimeError(f"Dashboard trend summary {field} is invalid.")
        if "change_percent" not in snapshot or "comparable_group" not in snapshot:
            raise RuntimeError(f"Dashboard trend summary {field} is incomplete.")


def _verify_testbed_summary(payload: dict[str, Any]) -> None:
    summary = payload.get("testbed_summary")
    if not isinstance(summary, dict):
        raise RuntimeError("Dashboard is missing testbed summary.")

    latest_run_id = payload.get("latest_run_id")
    if summary.get("latest_run_id") != latest_run_id:
        raise RuntimeError("Dashboard testbed summary latest run is invalid.")
    runs = payload.get("runs")
    expected_history_runs = len(runs) if isinstance(runs, list) else runs
    if summary.get("history_runs") != expected_history_runs:
        raise RuntimeError("Dashboard testbed summary history run count is invalid.")

    context_count = summary.get("context_count")
    if not isinstance(context_count, int) or context_count < 0:
        raise RuntimeError("Dashboard testbed summary context count is invalid.")
    if not isinstance(summary.get("history_testbeds"), list):
        raise RuntimeError("Dashboard testbed summary history testbeds missing.")
    if not isinstance(summary.get("source_files"), list):
        raise RuntimeError("Dashboard testbed summary source files missing.")
    if not isinstance(summary.get("host_names"), list):
        raise RuntimeError("Dashboard testbed summary host names missing.")
    if not isinstance(summary.get("num_cpus"), list):
        raise RuntimeError("Dashboard testbed summary CPU list missing.")
    source_file_count = summary.get("source_file_count")
    if not isinstance(source_file_count, int) or source_file_count < 0:
        raise RuntimeError("Dashboard testbed summary source file count is invalid.")
    if source_file_count < len(summary.get("source_files", [])):
        raise RuntimeError("Dashboard testbed summary source file count is invalid.")
    for field in ("cpu_scaling_counts", "aslr_counts"):
        counts = summary.get(field)
        if not isinstance(counts, dict):
            raise RuntimeError(f"Dashboard testbed summary missing {field}.")
        if sum(int(value) for value in counts.values()) != context_count:
            raise RuntimeError(f"Dashboard testbed summary {field} does not add up.")
    if "Trend interpretation" not in str(summary.get("interpretation", "")):
        raise RuntimeError("Dashboard testbed summary interpretation is missing.")


def _coverage_by_surface(rows: Any, label: str) -> dict[str, dict[str, Any]]:
    if not isinstance(rows, list):
        raise RuntimeError(f"Dashboard data is missing {label} rows.")
    by_surface = {
        str(item.get("surface")): item for item in rows if isinstance(item, dict)
    }
    missing = sorted(EXPECTED_COMPARISON_SURFACES - set(by_surface))
    if missing:
        raise RuntimeError(
            f"Dashboard data {label} is missing surfaces: " + ", ".join(missing)
        )
    return by_surface


def _expected_external_competitor_status(
    data: dict[str, Any],
    comparison_coverage: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    row_count = sum(
        int(item.get("external_competitor_row_count") or 0)
        for item in comparison_coverage.values()
    )
    backends = sorted(
        {
            str(backend)
            for item in comparison_coverage.values()
            for backend in item.get("external_competitor_backends", [])
        }
    )
    surfaces = sorted(
        surface
        for surface, item in comparison_coverage.items()
        if int(item.get("external_competitor_row_count") or 0) > 0
    )
    sample_groups = sorted(
        {
            str(group)
            for item in comparison_coverage.values()
            for group in item.get("external_competitor_sample_groups", [])
        }
    )
    contract = data.get("comparison_input_contract", {})
    return {
        "state": "live" if row_count else "queued",
        "row_count": row_count,
        "backends": backends,
        "surfaces": surfaces,
        "sample_groups": sample_groups,
        "supported_external_backends": contract.get("supported_external_backends", []),
        "claim_rule": contract.get("claim_rule"),
        "data_source": "comparison_coverage",
    }


def _verify_coverage(data: dict[str, Any], status: dict[str, Any]) -> None:
    coverage = _coverage_by_surface(data.get("coverage"), "coverage")
    comparison_coverage = _coverage_by_surface(
        data.get("comparison_coverage"),
        "comparison coverage",
    )

    if status.get("comparison_coverage_rows") != len(
        data.get("comparison_coverage", [])
    ):
        raise RuntimeError(
            "Dashboard data comparison coverage row count does not match status: "
            f"{len(data.get('comparison_coverage', []))!r} != "
            f"{status.get('comparison_coverage_rows')!r}."
        )
    external_competitor_rows = sum(
        int(item.get("external_competitor_row_count") or 0)
        for item in comparison_coverage.values()
    )
    external_competitor_backends = sorted(
        {
            str(backend)
            for item in comparison_coverage.values()
            for backend in item.get("external_competitor_backends", [])
        }
    )
    external_competitor_surfaces = sorted(
        surface
        for surface, item in comparison_coverage.items()
        if int(item.get("external_competitor_row_count") or 0) > 0
    )
    external_competitor_sample_groups = sorted(
        {
            str(group)
            for item in comparison_coverage.values()
            for group in item.get("external_competitor_sample_groups", [])
        }
    )
    expected_external_status = _expected_external_competitor_status(
        data, comparison_coverage
    )
    if data.get("external_competitor_status") != expected_external_status:
        raise RuntimeError(
            "Dashboard data external competitor status does not match "
            "comparison coverage."
        )
    if status.get("external_competitor_status") != expected_external_status:
        raise RuntimeError(
            "Dashboard status external competitor status does not match "
            "comparison coverage."
        )
    if status.get("external_competitor_rows") != external_competitor_rows:
        raise RuntimeError(
            "Dashboard data external competitor row count does not match status: "
            f"{external_competitor_rows!r} != "
            f"{status.get('external_competitor_rows')!r}."
        )
    if status.get("external_competitor_backends") != external_competitor_backends:
        raise RuntimeError(
            "Dashboard data external competitor backends do not match status: "
            f"{external_competitor_backends!r} != "
            f"{status.get('external_competitor_backends')!r}."
        )
    if status.get("external_competitor_surfaces") != external_competitor_surfaces:
        raise RuntimeError(
            "Dashboard data external competitor surfaces do not match status: "
            f"{external_competitor_surfaces!r} != "
            f"{status.get('external_competitor_surfaces')!r}."
        )
    if (
        status.get("external_competitor_sample_groups")
        != external_competitor_sample_groups
    ):
        raise RuntimeError(
            "Dashboard data external competitor sample groups do not match "
            "status: "
            f"{external_competitor_sample_groups!r} != "
            f"{status.get('external_competitor_sample_groups')!r}."
        )

    for surface, item in comparison_coverage.items():
        state = item.get("state")
        if state not in EXPECTED_COVERAGE_STATES:
            raise RuntimeError(
                f"Dashboard comparison coverage for {surface!r} has invalid "
                f"state {state!r}."
            )
        for field in ("comparison_scope", "entrypoint", "next_step"):
            if not isinstance(item.get(field), str) or not item.get(field):
                raise RuntimeError(
                    f"Dashboard comparison coverage for {surface!r} is "
                    f"missing {field}."
                )
        external_backends = item.get("external_competitor_backends")
        if not isinstance(external_backends, list):
            raise RuntimeError(
                f"Dashboard comparison coverage for {surface!r} is missing "
                "external competitor backends."
            )
        external_groups = item.get("external_competitor_sample_groups")
        if not isinstance(external_groups, list):
            raise RuntimeError(
                f"Dashboard comparison coverage for {surface!r} is missing "
                "external competitor sample groups."
            )
        for field in (
            "external_competitor_row_count",
            "external_competitor_group_count",
        ):
            if not isinstance(item.get(field), int) or item.get(field) < 0:
                raise RuntimeError(
                    f"Dashboard comparison coverage for {surface!r} has invalid "
                    f"{field}."
                )
        candidates = item.get("external_competitor_candidates")
        if not isinstance(candidates, list) or not candidates:
            raise RuntimeError(
                f"Dashboard comparison coverage for {surface!r} is missing "
                "external competitor candidates."
            )
        external_state = str(item.get("external_competitor_state", ""))
        if "reproducible external harness" not in external_state:
            raise RuntimeError(
                f"Dashboard comparison coverage for {surface!r} does not "
                "describe the reproducible external harness requirement."
            )

    live_surfaces = sum(1 for item in coverage.values() if item.get("state") == "live")
    queued_surfaces = sum(
        1 for item in coverage.values() if item.get("state") == "queued"
    )
    if status.get("live_surfaces") != live_surfaces:
        raise RuntimeError(
            "Dashboard coverage live surface count does not match status: "
            f"{live_surfaces!r} != {status.get('live_surfaces')!r}."
        )
    if status.get("queued_surfaces") != queued_surfaces:
        raise RuntimeError(
            "Dashboard coverage queued surface count does not match status: "
            f"{queued_surfaces!r} != {status.get('queued_surfaces')!r}."
        )


def _verify_links(status: dict[str, Any]) -> None:
    links = status.get("links")
    if not isinstance(links, dict):
        raise RuntimeError("Dashboard status is missing public links.")

    missing = [key for key in EXPECTED_LINKS if key not in links]
    if missing:
        raise RuntimeError(
            "Dashboard status is missing public link keys: " + ", ".join(missing)
        )

    mismatches = [
        f"{key}={links.get(key)!r}"
        for key, expected in EXPECTED_LINKS.items()
        if links.get(key) != expected
    ]
    if mismatches:
        raise RuntimeError(
            "Dashboard status public links do not match the DART endpoints: "
            + ", ".join(mismatches)
        )


def _latest_run(data: dict[str, Any]) -> dict[str, Any]:
    latest_run_id = data.get("latest_run_id")
    for run in data.get("runs", []):
        if isinstance(run, dict) and run.get("run_id") == latest_run_id:
            return run
    raise RuntimeError(
        f"Dashboard data latest_run_id has no matching run: {latest_run_id!r}."
    )


def _latest_measurement_count(data: dict[str, Any]) -> int:
    latest_run_id = data.get("latest_run_id")
    return sum(
        1
        for measurement in data.get("measurements", [])
        if isinstance(measurement, dict) and measurement.get("run_id") == latest_run_id
    )


def _verify_data(
    data: dict[str, Any],
    status: dict[str, Any],
    expected_data: dict[str, Any] | None,
) -> None:
    if data.get("schema_version") != SCHEMA_VERSION:
        raise RuntimeError(
            "Unsupported dashboard data schema version: "
            f"{data.get('schema_version')!r}"
        )

    if data.get("latest_run_id") != status.get("latest_run_id"):
        raise RuntimeError(
            "Dashboard data latest_run_id does not match status: "
            f"{data.get('latest_run_id')!r} != {status.get('latest_run_id')!r}."
        )

    latest_run = _latest_run(data)
    if latest_run.get("sha") != status.get("sha"):
        raise RuntimeError(
            "Dashboard data latest run SHA does not match status: "
            f"{latest_run.get('sha')!r} != {status.get('sha')!r}."
        )

    if len(data.get("runs", [])) != status.get("runs"):
        raise RuntimeError(
            "Dashboard data run count does not match status: "
            f"{len(data.get('runs', []))!r} != {status.get('runs')!r}."
        )

    latest_measurements = _latest_measurement_count(data)
    if latest_measurements != status.get("measurements"):
        raise RuntimeError(
            "Dashboard data latest measurement count does not match status: "
            f"{latest_measurements!r} != {status.get('measurements')!r}."
        )

    if data.get("links") != status.get("links"):
        raise RuntimeError("Dashboard data links do not match status links.")

    if data.get("service_decision") != status.get("service_decision"):
        raise RuntimeError(
            "Dashboard data service decision does not match status service decision."
        )
    if data.get("service_decision_summary") != status.get("service_decision_summary"):
        raise RuntimeError(
            "Dashboard data service decision summary does not match status."
        )
    _verify_service_decision_summary(data)

    if data.get("publication_contract") != status.get("publication_contract"):
        raise RuntimeError("Dashboard data publication contract does not match status.")
    _verify_publication_contract(data)

    if data.get("comparison_input_contract") != status.get("comparison_input_contract"):
        raise RuntimeError(
            "Dashboard data comparison input contract does not match status."
        )
    _verify_comparison_input_contract(data)

    if data.get("comparison_metric_contract") != status.get(
        "comparison_metric_contract"
    ):
        raise RuntimeError(
            "Dashboard data comparison metric contract does not match status."
        )
    _verify_comparison_metric_contract(data)

    if data.get("trend_summary") != status.get("trend_summary"):
        raise RuntimeError("Dashboard data trend summary does not match status.")
    _verify_trend_summary(data)
    trend_summary = data["trend_summary"]
    if trend_summary.get("primary_trend_rows") != len(data.get("latest_trends", [])):
        raise RuntimeError(
            "Dashboard data trend summary primary row count does not match trends."
        )
    if trend_summary.get("reference_trend_rows") != len(
        data.get("latest_reference_trends", [])
    ):
        raise RuntimeError(
            "Dashboard data trend summary reference row count does not match trends."
        )

    if data.get("testbed_summary") != status.get("testbed_summary"):
        raise RuntimeError("Dashboard data testbed summary does not match status.")
    _verify_testbed_summary(data)

    _verify_coverage(data, status)

    if expected_data is not None and data != expected_data:
        raise RuntimeError("Dashboard data endpoint does not match expected data.json.")


def main() -> int:
    args = parse_args()
    attempts = max(args.attempts, 1)
    last_error: Exception | None = None
    expected_status = (
        _load_status(str(args.expect_status_file), args.timeout)
        if args.expect_status_file
        else {}
    )
    expected_data = (
        _load_json(str(args.expect_data_file), args.timeout)
        if args.expect_data_file
        else None
    )
    expected_dashboard = (
        _load_text(str(args.expect_dashboard_file), args.timeout)
        if args.expect_dashboard_file
        else None
    )
    expected_summary = (
        _load_text(str(args.expect_summary_file), args.timeout)
        if args.expect_summary_file
        else None
    )
    expected_run_id = args.expect_run_id or expected_status.get("latest_run_id")
    expected_sha = args.expect_sha or expected_status.get("sha")
    data_url = args.data_url or _default_data_url(args.status_url)
    verify_data = args.require_data or expected_data is not None
    verify_dashboard_page = (
        args.require_dashboard_page or expected_dashboard is not None
    )
    verify_summary = args.require_summary or expected_summary is not None

    for attempt in range(1, attempts + 1):
        try:
            status = _load_status(args.status_url, args.timeout)
            _verify_status(
                status=status,
                expected_run_id=expected_run_id,
                expected_sha=expected_sha,
                require_fresh=args.require_fresh,
            )
            if verify_data:
                data = _load_json(data_url, args.timeout)
                _verify_data(data, status, expected_data)
            if verify_dashboard_page:
                dashboard_url = _linked_url(status, args.dashboard_url, "dashboard_url")
                _verify_dashboard_page(
                    _load_text(dashboard_url, args.timeout),
                    dashboard_url,
                    expected_dashboard,
                )
            if verify_summary:
                summary_url = _linked_url(status, args.summary_url, "summary_url")
                _verify_summary(
                    _load_text(summary_url, args.timeout),
                    summary_url,
                    expected_summary,
                )
            print(
                "verified dashboard endpoint: "
                f"{args.status_url} "
                f"(run={status.get('latest_run_id')}, "
                f"measurements={status.get('measurements')}, "
                f"freshness={status.get('freshness', {}).get('state')}, "
                f"data={'verified' if verify_data else 'not-checked'}, "
                f"page={'verified' if verify_dashboard_page else 'not-checked'}, "
                f"summary={'verified' if verify_summary else 'not-checked'})"
            )
            return 0
        except RuntimeError as exc:
            last_error = exc
            if attempt == attempts:
                break
            print(
                f"Attempt {attempt}/{attempts} failed: {exc}. "
                f"Retrying in {args.interval:g}s..."
            )
            time.sleep(args.interval)

    raise SystemExit(str(last_error))


if __name__ == "__main__":
    raise SystemExit(main())
