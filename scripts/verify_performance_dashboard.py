#!/usr/bin/env python3
"""Verify generated performance dashboard files are internally consistent."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any

from check_performance_dashboard_endpoint import (
    EXPECTED_COMPARISON_SURFACES,
    EXPECTED_COVERAGE_STATES,
    EXPECTED_DASHBOARD_PAGE_SNIPPETS,
    EXPECTED_EXTERNAL_COMPETITOR_BACKENDS,
    EXPECTED_LINKS,
    EXPECTED_PUBLICATION_CONTRACT,
    EXPECTED_SERVICE_DECISION_SUMMARY,
    EXPECTED_SERVICE_DECISIONS,
    EXPECTED_SERVICE_EVIDENCE_URLS,
    EXPECTED_SUMMARY_SNIPPETS,
    EXPECTED_UPDATE_TRIGGERS,
    EXPECTED_URL_SNIPPETS,
)

REQUIRED_FILES = ["index.html", "data.json", "summary.md", "status.json"]
SCHEMA_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "dashboard_dir",
        type=Path,
        help="Generated dashboard directory to verify.",
    )
    parser.add_argument(
        "--require-measurements",
        action="store_true",
        help="Fail when the latest run has zero measurements.",
    )
    return parser.parse_args()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise SystemExit(f"Invalid JSON in {path}: {exc}") from exc


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def _latest_run(data: dict[str, Any]) -> dict[str, Any]:
    latest_run_id = data.get("latest_run_id")
    for run in data.get("runs", []):
        if run.get("run_id") == latest_run_id:
            return run
    raise SystemExit(f"latest_run_id does not match any run: {latest_run_id}")


def _verify_public_links(links: Any) -> None:
    _require(isinstance(links, dict), "Dashboard links missing.")
    missing = [key for key in EXPECTED_LINKS if key not in links]
    _require(
        not missing,
        "Dashboard public links are missing keys: " + ", ".join(missing),
    )
    mismatches = [
        f"{key}={links.get(key)!r}"
        for key, expected in EXPECTED_LINKS.items()
        if links.get(key) != expected
    ]
    _require(
        not mismatches,
        "Dashboard public links do not match expected endpoints: "
        + ", ".join(mismatches),
    )


def _verify_rendered_snippets(body: str, snippets: list[str], label: str) -> None:
    missing = [snippet for snippet in snippets if snippet not in body]
    _require(
        not missing,
        f"Dashboard {label} is missing launch metadata: " + ", ".join(missing),
    )


def _verify_service_decision(rows: Any) -> None:
    _require(isinstance(rows, list), "Dashboard service decision missing.")
    by_role = {str(item.get("role")): item for item in rows if isinstance(item, dict)}
    missing = [role for role in EXPECTED_SERVICE_DECISIONS if role not in by_role]
    _require(
        not missing,
        "Dashboard service decision is missing roles: " + ", ".join(missing),
    )
    mismatches = [
        f"{role}={by_role[role].get('service')!r}"
        for role, expected in EXPECTED_SERVICE_DECISIONS.items()
        if by_role[role].get("service") != expected
    ]
    _require(
        not mismatches,
        "Dashboard service decision does not match selected architecture: "
        + ", ".join(mismatches),
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
    _require(
        not missing_fields,
        "Dashboard service decision missing evaluation fields: "
        + ", ".join(missing_fields),
    )
    for role, item in by_role.items():
        evidence_urls = item.get("evidence_urls")
        _require(
            isinstance(evidence_urls, list) and bool(evidence_urls),
            f"Dashboard service decision {role} evidence URLs missing.",
        )
        _require(
            all(str(url).startswith("https://") for url in evidence_urls),
            f"Dashboard service decision {role} evidence URLs invalid.",
        )
        unconfigured = [
            str(url) for url in evidence_urls if str(url) not in EXPECTED_URL_SNIPPETS
        ]
        _require(
            not unconfigured,
            f"Dashboard service decision {role} evidence URLs unconfigured: "
            + ", ".join(unconfigured),
        )
        for field in ("cost_model", "data_owner", "ci_integration", "approval_gate"):
            _require(
                isinstance(item.get(field), str) and bool(item.get(field)),
                f"Dashboard service decision {role} {field} invalid.",
            )
    observed_urls = sorted(
        {str(url) for item in by_role.values() for url in item.get("evidence_urls", [])}
    )
    missing_urls = sorted(set(EXPECTED_SERVICE_EVIDENCE_URLS) - set(observed_urls))
    extra_urls = sorted(set(observed_urls) - set(EXPECTED_SERVICE_EVIDENCE_URLS))
    _require(
        not missing_urls and not extra_urls,
        "Dashboard service decision evidence URLs do not match verified "
        "service-source contract: "
        + "; ".join(
            part
            for part in (
                "missing: " + ", ".join(missing_urls) if missing_urls else "",
                "extra: " + ", ".join(extra_urls) if extra_urls else "",
            )
            if part
        ),
    )
    for role in ("primary-dashboard", "external-history-and-thresholds"):
        _require(
            "free" in str(by_role[role].get("cost_model", "")).lower(),
            f"Dashboard service decision {role} missing free/open-source cost path.",
        )


def _verify_service_decision_summary(summary: Any, rows: Any) -> None:
    _require(isinstance(summary, dict), "Dashboard service decision summary missing.")
    _require(isinstance(rows, list), "Dashboard service decision rows missing.")
    missing = [key for key in EXPECTED_SERVICE_DECISION_SUMMARY if key not in summary]
    _require(
        not missing,
        "Dashboard service decision summary missing keys: " + ", ".join(missing),
    )
    mismatches = [
        f"{key}={summary.get(key)!r}"
        for key, expected in EXPECTED_SERVICE_DECISION_SUMMARY.items()
        if summary.get(key) != expected
    ]
    _require(
        not mismatches,
        "Dashboard service decision summary does not match selected architecture: "
        + ", ".join(mismatches),
    )

    by_role = {str(item.get("role")): item for item in rows if isinstance(item, dict)}
    selected = by_role.get("primary-dashboard", {})
    recommended = by_role.get("external-history-and-thresholds", {})
    _require(
        summary.get("canonical_data_owner") == selected.get("data_owner"),
        "Dashboard service decision summary canonical data owner mismatch.",
    )
    _require(
        summary.get("selected_host") == selected.get("service"),
        "Dashboard service decision summary selected host mismatch.",
    )
    _require(
        summary.get("recommended_external_service") == recommended.get("service"),
        "Dashboard service decision summary recommended external service mismatch.",
    )
    expected_deferred = [
        str(item.get("service"))
        for item in rows
        if isinstance(item, dict)
        and str(item.get("state")) in {"deferred-pilot", "not-selected"}
    ]
    _require(
        summary.get("deferred_services") == expected_deferred,
        "Dashboard service decision summary deferred service list mismatch.",
    )
    expected_fallback = [
        str(item.get("service"))
        for item in rows
        if isinstance(item, dict) and str(item.get("state")) == "fallback"
    ]
    _require(
        summary.get("fallback_services") == expected_fallback,
        "Dashboard service decision summary fallback service list mismatch.",
    )
    approval_boundary = str(summary.get("approval_boundary", "")).lower()
    _require(
        "maintainer approval" in approval_boundary
        and "workflow dispatch" in approval_boundary,
        "Dashboard service decision summary approval boundary missing.",
    )


def _verify_publication_contract(contract: Any, links: Any) -> None:
    _require(isinstance(contract, dict), "Dashboard publication contract missing.")
    mismatches = [
        f"{key}={contract.get(key)!r}"
        for key, expected in EXPECTED_PUBLICATION_CONTRACT.items()
        if contract.get(key) != expected
    ]
    _require(
        not mismatches,
        "Dashboard publication contract does not match expected Pages publisher "
        "contract: " + ", ".join(mismatches),
    )
    boundary = str(contract.get("approval_boundary", "")).lower()
    _require(
        "maintainer approval" in boundary and "workflow dispatch" in boundary,
        "Dashboard publication contract approval boundary missing.",
    )
    if isinstance(links, dict):
        expected_endpoints = [
            links.get("dashboard_url"),
            links.get("status_url"),
            links.get("data_url"),
            links.get("summary_url"),
        ]
        _require(
            contract.get("required_endpoints") == expected_endpoints,
            "Dashboard publication contract endpoint list mismatch.",
        )


def _verify_comparison_input_contract(contract: Any) -> None:
    _require(isinstance(contract, dict), "Dashboard comparison input contract missing.")
    _require(
        contract.get("format") == "Google Benchmark JSON",
        "Dashboard comparison input contract format invalid.",
    )
    supported = contract.get("supported_external_backends")
    _require(
        isinstance(supported, list),
        "Dashboard comparison input contract supported backends missing.",
    )
    missing = sorted(EXPECTED_EXTERNAL_COMPETITOR_BACKENDS - set(supported))
    _require(
        not missing,
        "Dashboard comparison input contract missing backends: " + ", ".join(missing),
    )
    examples = contract.get("example_rows")
    _require(
        isinstance(examples, list) and bool(examples),
        "Dashboard comparison input contract example rows missing.",
    )
    for field in ("backend_suffix_rule", "primary_backend_rule", "claim_rule"):
        _require(
            isinstance(contract.get(field), str) and bool(contract.get(field)),
            f"Dashboard comparison input contract {field} missing.",
        )
    _require(
        "_<Backend>" in contract["backend_suffix_rule"],
        "Dashboard comparison input contract suffix rule missing.",
    )
    _require(
        "live competitor" in contract["claim_rule"],
        "Dashboard comparison input contract claim rule missing.",
    )


def _verify_comparison_metric_contract(contract: Any) -> None:
    _require(
        isinstance(contract, dict), "Dashboard comparison metric contract missing."
    )
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
    _require(
        not mismatches,
        "Dashboard comparison metric contract invalid: " + ", ".join(mismatches),
    )
    for field in ("interpretation", "latest_comparison_interpretation"):
        value = contract.get(field)
        _require(
            isinstance(value, str) and "ratio < 1" in value,
            f"Dashboard comparison metric contract {field} missing ratio direction.",
        )
    status_meanings = contract.get("status_meanings")
    _require(
        isinstance(status_meanings, dict),
        "Dashboard comparison metric contract status meanings missing.",
    )
    for status in ("lead", "behind", "faster-than-primary", "slower-than-primary"):
        _require(
            bool(status_meanings.get(status)),
            "Dashboard comparison metric contract missing status meaning for "
            f"{status}.",
        )


def _verify_trend_summary(summary: Any, data: dict[str, Any]) -> None:
    _require(isinstance(summary, dict), "Dashboard trend summary missing.")
    _require(
        summary.get("latest_run_id") == data.get("latest_run_id"),
        "Dashboard trend summary latest run mismatch.",
    )
    _require(
        summary.get("history_runs") == len(data.get("runs", [])),
        "Dashboard trend summary history count mismatch.",
    )
    primary_rows = len(data.get("latest_trends", []))
    reference_rows = len(data.get("latest_reference_trends", []))
    _require(
        summary.get("primary_trend_rows") == primary_rows,
        "Dashboard trend summary primary row count mismatch.",
    )
    _require(
        summary.get("reference_trend_rows") == reference_rows,
        "Dashboard trend summary reference row count mismatch.",
    )
    for field, expected_rows in (
        ("primary_status_counts", primary_rows),
        ("primary_threshold_counts", primary_rows),
        ("reference_status_counts", reference_rows),
        ("reference_threshold_counts", reference_rows),
    ):
        counts = summary.get(field)
        _require(isinstance(counts, dict), f"Dashboard trend summary {field} missing.")
        _require(
            sum(int(value) for value in counts.values()) == expected_rows,
            f"Dashboard trend summary {field} does not match row count.",
        )
    for field in (
        "primary_largest_regression",
        "primary_largest_improvement",
        "reference_largest_regression",
        "reference_largest_improvement",
    ):
        snapshot = summary.get(field)
        if snapshot is None:
            continue
        _require(isinstance(snapshot, dict), f"Dashboard trend summary {field} bad.")
        _require(
            "change_percent" in snapshot and "comparable_group" in snapshot,
            f"Dashboard trend summary {field} incomplete.",
        )


def _verify_testbed_summary(summary: Any, data: dict[str, Any]) -> None:
    _require(isinstance(summary, dict), "Dashboard testbed summary missing.")
    latest_run = _latest_run(data)
    contexts = latest_run.get("contexts", [])
    if not isinstance(contexts, list):
        contexts = []
    _require(
        summary.get("latest_run_id") == data.get("latest_run_id"),
        "Dashboard testbed summary latest run mismatch.",
    )
    _require(
        summary.get("latest_run_at") == latest_run.get("run_at"),
        "Dashboard testbed summary run time mismatch.",
    )
    _require(
        summary.get("testbed") == latest_run.get("testbed"),
        "Dashboard testbed summary testbed mismatch.",
    )
    _require(
        summary.get("history_runs") == len(data.get("runs", [])),
        "Dashboard testbed summary history count mismatch.",
    )
    _require(
        summary.get("context_count") == len(contexts),
        "Dashboard testbed summary context count mismatch.",
    )
    source_files = summary.get("source_files")
    _require(isinstance(source_files, list), "Dashboard testbed source files missing.")
    source_file_count = summary.get("source_file_count")
    _require(
        isinstance(source_file_count, int) and source_file_count >= 0,
        "Dashboard testbed source file count invalid.",
    )
    _require(
        source_file_count >= len(source_files),
        "Dashboard testbed source file count mismatch.",
    )
    for field in ("cpu_scaling_counts", "aslr_counts"):
        counts = summary.get(field)
        _require(isinstance(counts, dict), f"Dashboard testbed {field} missing.")
        _require(
            sum(int(value) for value in counts.values()) == len(contexts),
            f"Dashboard testbed {field} does not match context count.",
        )
    _require(
        isinstance(summary.get("history_testbeds"), list),
        "Dashboard testbed history list missing.",
    )
    _require(
        isinstance(summary.get("num_cpus"), list),
        "Dashboard testbed CPU list missing.",
    )
    _require(
        "Trend interpretation" in str(summary.get("interpretation", "")),
        "Dashboard testbed summary interpretation missing.",
    )


def _verify_freshness(freshness: Any) -> None:
    _require(isinstance(freshness, dict), "Dashboard freshness status missing.")
    _require(
        freshness.get("update_triggers") == EXPECTED_UPDATE_TRIGGERS,
        "Dashboard freshness metadata is missing expected update triggers.",
    )


def _coverage_by_surface(rows: Any, label: str) -> dict[str, dict[str, Any]]:
    _require(isinstance(rows, list), f"Dashboard {label} rows missing.")
    by_surface = {
        str(item.get("surface")): item for item in rows if isinstance(item, dict)
    }
    missing = sorted(EXPECTED_COMPARISON_SURFACES - set(by_surface))
    _require(
        not missing,
        f"Dashboard {label} is missing surfaces: " + ", ".join(missing),
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


def _verify_comparison_coverage(data: dict[str, Any], status: dict[str, Any]) -> None:
    coverage = _coverage_by_surface(data.get("coverage"), "coverage")
    comparison_coverage = _coverage_by_surface(
        data.get("comparison_coverage"),
        "comparison coverage",
    )
    _require(
        status.get("comparison_coverage_rows")
        == len(data.get("comparison_coverage", [])),
        "Comparison coverage row count mismatch.",
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
    _require(
        data.get("external_competitor_status") == expected_external_status,
        "External competitor status does not match comparison coverage.",
    )
    _require(
        status.get("external_competitor_status") == expected_external_status,
        "External competitor status mismatch.",
    )
    _require(
        status.get("external_competitor_rows") == external_competitor_rows,
        "External competitor row count mismatch.",
    )
    _require(
        status.get("external_competitor_backends") == external_competitor_backends,
        "External competitor backend mismatch.",
    )
    _require(
        status.get("external_competitor_surfaces") == external_competitor_surfaces,
        "External competitor surface mismatch.",
    )
    _require(
        status.get("external_competitor_sample_groups")
        == external_competitor_sample_groups,
        "External competitor sample group mismatch.",
    )

    for surface, item in comparison_coverage.items():
        _require(
            item.get("state") in EXPECTED_COVERAGE_STATES,
            f"Comparison coverage state is invalid for {surface}.",
        )
        for field in ("comparison_scope", "entrypoint", "next_step"):
            _require(
                isinstance(item.get(field), str) and bool(item.get(field)),
                f"Comparison coverage {field} missing for {surface}.",
            )
        external_backends = item.get("external_competitor_backends")
        _require(
            isinstance(external_backends, list),
            f"Comparison coverage external competitor backends missing for {surface}.",
        )
        external_groups = item.get("external_competitor_sample_groups")
        _require(
            isinstance(external_groups, list),
            f"Comparison coverage external competitor sample groups missing for {surface}.",
        )
        for field in (
            "external_competitor_row_count",
            "external_competitor_group_count",
        ):
            _require(
                isinstance(item.get(field), int) and item.get(field) >= 0,
                f"Comparison coverage {field} invalid for {surface}.",
            )
        candidates = item.get("external_competitor_candidates")
        _require(
            isinstance(candidates, list) and bool(candidates),
            f"Comparison coverage competitor candidates missing for {surface}.",
        )
        _require(
            "reproducible external harness"
            in str(item.get("external_competitor_state", "")),
            f"Comparison coverage harness requirement missing for {surface}.",
        )

    live_surfaces = sum(1 for item in coverage.values() if item.get("state") == "live")
    queued_surfaces = sum(
        1 for item in coverage.values() if item.get("state") == "queued"
    )
    _require(status.get("live_surfaces") == live_surfaces, "Live surface mismatch.")
    _require(
        status.get("queued_surfaces") == queued_surfaces,
        "Queued surface mismatch.",
    )


def verify_dashboard(dashboard_dir: Path, require_measurements: bool) -> None:
    _require(dashboard_dir.is_dir(), f"Dashboard directory not found: {dashboard_dir}")
    for name in REQUIRED_FILES:
        path = dashboard_dir / name
        _require(path.is_file(), f"Required dashboard file missing: {path}")
        _require(path.stat().st_size > 0, f"Required dashboard file is empty: {path}")

    data = _load_json(dashboard_dir / "data.json")
    status = _load_json(dashboard_dir / "status.json")
    latest_run = _latest_run(data)
    latest_run_id = data.get("latest_run_id")
    measurements = [
        item
        for item in data.get("measurements", [])
        if item.get("run_id") == latest_run_id
    ]
    coverage_counts = Counter(item.get("state") for item in data.get("coverage", []))

    _require(
        data.get("schema_version") == SCHEMA_VERSION,
        f"Unsupported data schema version: {data.get('schema_version')}",
    )
    _require(
        status.get("schema_version") == SCHEMA_VERSION,
        f"Unsupported status schema version: {status.get('schema_version')}",
    )
    _require(status.get("latest_run_id") == latest_run_id, "Run ID mismatch.")
    _require(
        status.get("latest_run_at") == latest_run.get("run_at"), "Run time mismatch."
    )
    _require(status.get("branch") == latest_run.get("branch"), "Branch mismatch.")
    _require(status.get("sha") == latest_run.get("sha"), "SHA mismatch.")
    _require(status.get("testbed") == latest_run.get("testbed"), "Testbed mismatch.")
    _require(
        status.get("source_url") == latest_run.get("source_url"),
        "Source URL mismatch.",
    )
    _require(
        status.get("measurements") == len(measurements),
        "Latest measurement count mismatch.",
    )
    _require(status.get("runs") == len(data.get("runs", [])), "Run count mismatch.")
    _require(
        status.get("live_surfaces") == coverage_counts.get("live", 0),
        "Live surface count mismatch.",
    )
    _require(
        status.get("queued_surfaces") == coverage_counts.get("queued", 0),
        "Queued surface count mismatch.",
    )
    _require(
        status.get("comparison_states")
        == data.get("latest_summary", {}).get("comparisons", {}),
        "Comparison state count mismatch.",
    )
    _require(
        status.get("thresholds") == data.get("thresholds"),
        "Threshold policy mismatch.",
    )
    _require(
        status.get("freshness") == data.get("freshness"),
        "Freshness status mismatch.",
    )
    _verify_freshness(data.get("freshness"))
    _verify_public_links(data.get("links"))
    _require(status.get("links") == data.get("links"), "Dashboard links mismatch.")
    _verify_service_decision(data.get("service_decision"))
    _require(
        status.get("service_decision") == data.get("service_decision"),
        "Dashboard service decision mismatch.",
    )
    _verify_service_decision_summary(
        data.get("service_decision_summary"),
        data.get("service_decision"),
    )
    _require(
        status.get("service_decision_summary") == data.get("service_decision_summary"),
        "Dashboard service decision summary mismatch.",
    )
    _verify_publication_contract(data.get("publication_contract"), data.get("links"))
    _require(
        status.get("publication_contract") == data.get("publication_contract"),
        "Dashboard publication contract mismatch.",
    )
    _verify_comparison_input_contract(data.get("comparison_input_contract"))
    _require(
        status.get("comparison_input_contract")
        == data.get("comparison_input_contract"),
        "Dashboard comparison input contract mismatch.",
    )
    _verify_comparison_metric_contract(data.get("comparison_metric_contract"))
    _require(
        status.get("comparison_metric_contract")
        == data.get("comparison_metric_contract"),
        "Dashboard comparison metric contract mismatch.",
    )
    _verify_trend_summary(data.get("trend_summary"), data)
    _require(
        status.get("trend_summary") == data.get("trend_summary"),
        "Dashboard trend summary mismatch.",
    )
    _verify_testbed_summary(data.get("testbed_summary"), data)
    _require(
        status.get("testbed_summary") == data.get("testbed_summary"),
        "Dashboard testbed summary mismatch.",
    )
    _require(
        status.get("backend_summary_rows")
        == len(data.get("latest_backend_summary", [])),
        "Backend summary row count mismatch.",
    )
    _require(
        status.get("backend_matrix_rows") == len(data.get("latest_backend_matrix", [])),
        "Backend matrix row count mismatch.",
    )
    _require(
        status.get("reference_trend_rows")
        == len(data.get("latest_reference_trends", [])),
        "Reference trend row count mismatch.",
    )
    _verify_comparison_coverage(data, status)
    _require(
        status.get("dashboard_files") == REQUIRED_FILES,
        "Dashboard file manifest mismatch.",
    )
    if require_measurements:
        _require(measurements, "No latest-run measurements found.")

    index = (dashboard_dir / "index.html").read_text(encoding="utf-8")
    summary = (dashboard_dir / "summary.md").read_text(encoding="utf-8")
    _verify_rendered_snippets(index, EXPECTED_DASHBOARD_PAGE_SNIPPETS, "HTML")
    _verify_rendered_snippets(summary, EXPECTED_SUMMARY_SNIPPETS, "summary")


def main() -> int:
    args = parse_args()
    verify_dashboard(args.dashboard_dir, args.require_measurements)
    print(f"verified dashboard: {args.dashboard_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
