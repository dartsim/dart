#!/usr/bin/env python3
"""Write a GitHub Actions summary for a generated performance dashboard."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

DEFAULT_DASHBOARD_URL = "https://dartsim.github.io/dart/performance/"
DEFAULT_STATUS_URL = "https://dartsim.github.io/dart/performance/status.json"
DEFAULT_DATA_URL = "https://dartsim.github.io/dart/performance/data.json"
DEFAULT_SUMMARY_URL = "https://dartsim.github.io/dart/performance/summary.md"
DEFAULT_GUIDE_URL = (
    "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html"
)
DEFAULT_WEBSITE_URL = "https://dart.readthedocs.io/en/latest/"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "status_file",
        type=Path,
        help="Generated dashboard status.json file.",
    )
    parser.add_argument(
        "--summary-file",
        type=Path,
        default=_default_summary_file(),
        help="Summary output file. Defaults to GITHUB_STEP_SUMMARY when set.",
    )
    parser.add_argument(
        "--dashboard-url",
        default=None,
        help="Public dashboard URL.",
    )
    parser.add_argument(
        "--status-url",
        default=None,
        help="Public dashboard status.json URL.",
    )
    parser.add_argument(
        "--data-url",
        default=None,
        help="Public dashboard data.json URL.",
    )
    parser.add_argument(
        "--summary-url",
        default=None,
        help="Public dashboard summary.md URL.",
    )
    parser.add_argument(
        "--bencher-configured",
        action="store_true",
        help="Whether Bencher reporting was configured for this run.",
    )
    return parser.parse_args()


def _default_summary_file() -> Path | None:
    summary = os.environ.get("GITHUB_STEP_SUMMARY")
    return Path(summary) if summary else None


def _load_status(path: Path) -> dict[str, Any]:
    try:
        status = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON in {path}: {exc}") from exc
    if not isinstance(status, dict):
        raise SystemExit(f"Expected JSON object in {path}.")
    return status


def _comparison_text(status: dict[str, Any]) -> str:
    states = status.get("comparison_states", {})
    if not isinstance(states, dict) or not states:
        return "none"
    return ", ".join(f"{key}: {value}" for key, value in sorted(states.items()))


def _threshold_text(status: dict[str, Any]) -> str:
    thresholds = status.get("thresholds", {})
    if not isinstance(thresholds, dict) or not thresholds:
        return "none"
    mode = thresholds.get("mode", "unknown")
    primary = thresholds.get("primary_change_percent", "unknown")
    required = "required" if thresholds.get("required_gate") else "informational"
    return f"{mode}, {primary}% primary/reference change, {required}"


def _freshness_text(status: dict[str, Any]) -> str:
    freshness = status.get("freshness", {})
    if not isinstance(freshness, dict) or not freshness:
        return "unknown"
    state = freshness.get("state", "unknown")
    age = freshness.get("latest_run_age_hours")
    stale_after = freshness.get("stale_after_hours", "unknown")
    triggers = freshness.get("update_triggers")
    trigger_text = ""
    if isinstance(triggers, list) and triggers:
        trigger_text = ", triggers: " + ", ".join(str(item) for item in triggers)
    return f"{state}, latest run age {age}h, stale after {stale_after}h{trigger_text}"


def _service_decision_rows(status: dict[str, Any]) -> list[dict[str, Any]]:
    rows = status.get("service_decision", [])
    if not isinstance(rows, list):
        return []
    return [row for row in rows if isinstance(row, dict)]


def _service_decision_summary(status: dict[str, Any]) -> dict[str, Any]:
    summary = status.get("service_decision_summary", {})
    return summary if isinstance(summary, dict) else {}


def _publication_contract(status: dict[str, Any]) -> dict[str, Any]:
    contract = status.get("publication_contract", {})
    return contract if isinstance(contract, dict) else {}


def _comparison_metric_contract(status: dict[str, Any]) -> dict[str, Any]:
    contract = status.get("comparison_metric_contract", {})
    return contract if isinstance(contract, dict) else {}


def _external_competitor_status(status: dict[str, Any]) -> dict[str, Any]:
    competitor_status = status.get("external_competitor_status", {})
    return competitor_status if isinstance(competitor_status, dict) else {}


def _trend_summary(status: dict[str, Any]) -> dict[str, Any]:
    trends = status.get("trend_summary", {})
    return trends if isinstance(trends, dict) else {}


def _testbed_summary(status: dict[str, Any]) -> dict[str, Any]:
    testbed = status.get("testbed_summary", {})
    return testbed if isinstance(testbed, dict) else {}


def _list_text(value: Any) -> str:
    if not isinstance(value, list) or not value:
        return "-"
    return "<br>".join(f"`{item}`" for item in value)


def _summary_value_text(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, dict):
        if not value:
            return "-"
        return "<br>".join(
            f"`{key}`: `{item}`" for key, item in value.items() if item is not None
        )
    if isinstance(value, list):
        return _list_text(value)
    return f"`{value}`"


def render_summary(
    status: dict[str, Any],
    dashboard_url: str,
    status_url: str,
    data_url: str,
    summary_url: str,
    guide_url: str,
    website_url: str,
    bencher_configured: bool,
) -> str:
    rows = [
        ("Canonical website", f"[Read the Docs]({website_url})"),
        ("Dashboard", f"[Open dashboard]({dashboard_url})"),
        ("Status endpoint", f"[status.json]({status_url})"),
        ("Durable data", f"[data.json]({data_url})"),
        ("Text summary", f"[summary.md]({summary_url})"),
        ("Dashboard guide", f"[Read the guide]({guide_url})"),
        ("Latest run", str(status.get("latest_run_id", "unknown"))),
        ("Latest timestamp", str(status.get("latest_run_at", "unknown"))),
        ("Branch", str(status.get("branch", "unknown"))),
        ("SHA", str(status.get("sha", "unknown"))),
        ("Testbed", str(status.get("testbed", "unknown"))),
        ("Measurements", str(status.get("measurements", 0))),
        ("Runs in history", str(status.get("runs", 0))),
        ("Freshness", _freshness_text(status)),
        ("Live surfaces", str(status.get("live_surfaces", 0))),
        ("Queued surfaces", str(status.get("queued_surfaces", 0))),
        ("Comparison states", _comparison_text(status)),
        ("Threshold policy", _threshold_text(status)),
        ("Backend summary rows", str(status.get("backend_summary_rows", 0))),
        ("Backend matrix rows", str(status.get("backend_matrix_rows", 0))),
        ("Reference trend rows", str(status.get("reference_trend_rows", 0))),
        ("Comparison coverage rows", str(status.get("comparison_coverage_rows", 0))),
        ("External competitor rows", str(status.get("external_competitor_rows", 0))),
        (
            "External competitor backends",
            _list_text(status.get("external_competitor_backends")),
        ),
        (
            "External competitor surfaces",
            _list_text(status.get("external_competitor_surfaces")),
        ),
        ("Bencher configured", "yes" if bencher_configured else "no"),
    ]

    lines = [
        "## DART Performance Dashboard",
        "",
        "| Field | Value |",
        "| ----- | ----- |",
    ]
    lines.extend(f"| {field} | {value} |" for field, value in rows)
    lines.append("")
    service_summary = _service_decision_summary(status)
    if service_summary:
        lines.extend(
            [
                "### Service Decision Summary",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
            ]
        )
        lines.extend(
            f"| `{key}` | {_summary_value_text(value)} |"
            for key, value in service_summary.items()
        )
        lines.append("")
    service_rows = _service_decision_rows(status)
    if service_rows:
        lines.extend(
            [
                "### Service Decision",
                "",
                "| Role | Service | State | Fit | Next step |",
                "| ---- | ------- | ----- | --- | --------- |",
            ]
        )
        lines.extend(
            "| {role} | {service} | {state} | {fit} | {next_step} |".format(
                role=row.get("role", "-"),
                service=row.get("service", "-"),
                state=row.get("state", "-"),
                fit=row.get("fit", "-"),
                next_step=row.get("next_step", "-"),
            )
            for row in service_rows
        )
        lines.append("")
    publication = _publication_contract(status)
    if publication:
        lines.extend(
            [
                "### Publication Contract",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
                f"| Host | {publication.get('host', '-')} |",
                f"| Pages source | `{publication.get('source_branch', '-')}` "
                f"{publication.get('source_path', '-')} |",
                f"| Dashboard path | `{publication.get('dashboard_path', '-')}` |",
                f"| Workflow | `{publication.get('workflow_name', '-')}` "
                f"({publication.get('publisher_workflow', '-')}) |",
                f"| Publish ref | `{publication.get('publish_ref', '-')}` |",
                f"| schedule_crons | {_list_text(publication.get('schedule_crons'))} |",
                "| expected_update_interval_hours | "
                f"`{publication.get('expected_update_interval_hours', '-')}` |",
                f"| stale_after_hours | `{publication.get('stale_after_hours', '-')}` |",
                "| Pages build request | "
                f"{publication.get('requires_pages_build_request', '-')} |",
                "| Required permissions | "
                f"{_list_text(publication.get('required_permissions'))} |",
                "| Required endpoints | "
                f"{_list_text(publication.get('required_endpoints'))} |",
                "| Required website links | "
                f"{_list_text(publication.get('required_website_links'))} |",
                "| Required workflow artifacts | "
                f"{_list_text(publication.get('required_workflow_artifacts'))} |",
                f"| Launch checks | {_list_text(publication.get('launch_checks'))} |",
                f"| Approval boundary | {publication.get('approval_boundary', '-')} |",
                "",
            ]
        )
    metric_contract = _comparison_metric_contract(status)
    if metric_contract:
        lines.extend(
            [
                "### Comparison Metric Contract",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
                f"| Time unit | `{metric_contract.get('time_unit', '-')}` |",
                "| Backend matrix ratio | "
                f"`{metric_contract.get('ratio_field', '-')}` = "
                f"`{metric_contract.get('ratio_formula', '-')}` |",
                "| Latest comparison ratio | "
                f"`{metric_contract.get('latest_comparison_ratio_field', '-')}` = "
                f"`{metric_contract.get('latest_comparison_ratio_formula', '-')}` |",
                f"| Interpretation | {metric_contract.get('interpretation', '-')} |",
                "| Latest comparison interpretation | "
                f"{metric_contract.get('latest_comparison_interpretation', '-')} |",
                "",
            ]
        )
    competitor_status = _external_competitor_status(status)
    if competitor_status:
        lines.extend(
            [
                "### External Competitor Status",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
            ]
        )
        lines.extend(
            f"| `{key}` | {_summary_value_text(value)} |"
            for key, value in competitor_status.items()
        )
        lines.append("")
    testbed = _testbed_summary(status)
    if testbed:
        lines.extend(
            [
                "### Testbed Summary",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
            ]
        )
        lines.extend(
            f"| `{key}` | {_summary_value_text(value)} |"
            for key, value in testbed.items()
        )
        lines.append("")
    trends = _trend_summary(status)
    if trends:
        lines.extend(
            [
                "### Trend Summary",
                "",
                "| Field | Value |",
                "| ----- | ----- |",
            ]
        )
        lines.extend(
            f"| `{key}` | {_summary_value_text(value)} |"
            for key, value in trends.items()
        )
        lines.append("")
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    status = _load_status(args.status_file)
    links = status.get("links", {})
    if not isinstance(links, dict):
        links = {}
    summary = render_summary(
        status=status,
        dashboard_url=args.dashboard_url
        or links.get("dashboard_url")
        or DEFAULT_DASHBOARD_URL,
        status_url=args.status_url or links.get("status_url") or DEFAULT_STATUS_URL,
        data_url=args.data_url or links.get("data_url") or DEFAULT_DATA_URL,
        summary_url=args.summary_url or links.get("summary_url") or DEFAULT_SUMMARY_URL,
        guide_url=links.get("guide_url") or DEFAULT_GUIDE_URL,
        website_url=links.get("canonical_website_url") or DEFAULT_WEBSITE_URL,
        bencher_configured=args.bencher_configured,
    )

    if args.summary_file is None:
        raise SystemExit("No summary output file provided.")
    args.summary_file.parent.mkdir(parents=True, exist_ok=True)
    with args.summary_file.open("a", encoding="utf-8") as file:
        file.write(summary)
    print(f"wrote dashboard summary: {args.summary_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
