import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_endpoint.py"


def _links():
    return {
        "canonical_website_url": "https://dart.readthedocs.io/en/latest/",
        "dashboard_url": "https://dartsim.github.io/dart/performance/",
        "status_url": "https://dartsim.github.io/dart/performance/status.json",
        "data_url": "https://dartsim.github.io/dart/performance/data.json",
        "summary_url": "https://dartsim.github.io/dart/performance/summary.md",
        "guide_url": (
            "https://dart.readthedocs.io/en/latest/community/"
            "performance_dashboard.html"
        ),
    }


def _service_decision():
    return [
        {
            "role": "primary-dashboard",
            "service": "DART-owned GitHub Pages",
            "state": "selected",
            "fit": "high",
            "cost_model": "Free for DART's public GitHub repository.",
            "data_owner": "DART-owned static files.",
            "ci_integration": "GitHub Actions publisher.",
            "approval_gate": "Maintainer-approved workflow.",
            "rationale": "Primary static dashboard host.",
            "next_step": "Publish gh-pages/performance.",
            "evidence_urls": [
                "https://docs.github.com/en/pages/getting-started-with-github-pages",
                "https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site",
            ],
        },
        {
            "role": "external-history-and-thresholds",
            "service": "Bencher Cloud or Self-Hosted",
            "state": "recommended-after-maintainer-approval",
            "fit": "high",
            "cost_model": "Free for public/open-source projects.",
            "data_owner": "Companion service.",
            "ci_integration": "bencher CLI with cpp_google adapter.",
            "approval_gate": "Maintainer-configured project and key.",
            "rationale": "External history and thresholds.",
            "next_step": "Configure project and key.",
            "evidence_urls": [
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/pages/pricing.astro",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/content/docs-how-to/en/github-actions.mdx",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-how-to/github-actions/en/base-branch.mdx",
                "https://github.com/bencherdev/bencher/blob/main/services/console/src/chunks/docs-explanation/adapters/en/cpp-google.mdx",
            ],
        },
        {
            "role": "microbenchmark-pr-pilot",
            "service": "CodSpeed",
            "state": "deferred-pilot",
            "fit": "medium",
            "cost_model": "Free on public repositories.",
            "data_owner": "External CodSpeed project.",
            "ci_integration": "CodSpeed GitHub Action.",
            "approval_gate": "Maintainer-approved pilot.",
            "rationale": "Microbenchmark regression checks.",
            "next_step": "Pilot later.",
            "evidence_urls": [
                "https://codspeed.io/docs/features/seats-and-billing",
                "https://codspeed.io/docs/integrations/ci/github-actions",
                "https://codspeed.io/docs/guides/how-to-benchmark-cpp-with-google-benchmark",
            ],
        },
        {
            "role": "off-the-shelf-pages-action",
            "service": "github-action-benchmark",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": "Open-source GitHub Action.",
            "data_owner": "Action-owned Pages data.",
            "ci_integration": "Parses googlecpp output.",
            "approval_gate": "Maintainer approval for Pages-writing action.",
            "rationale": "Generic dashboard model.",
            "next_step": "Keep as fallback.",
            "evidence_urls": [
                "https://github.com/benchmark-action/github-action-benchmark",
                "https://github.com/marketplace/actions/continuous-benchmark",
            ],
        },
        {
            "role": "static-benchmark-dashboard-framework",
            "service": "Airspeed Velocity",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": "Open-source static benchmark dashboard.",
            "data_owner": "ASV JSON and generated static HTML.",
            "ci_integration": "Requires ASV benchmark suite or adapter.",
            "approval_gate": "Maintainer-approved pilot.",
            "rationale": "Python benchmark-suite workflow.",
            "next_step": "Keep as future migration candidate.",
            "evidence_urls": [
                "https://github.com/airspeed-velocity/asv",
                "https://asv.readthedocs.io/en/stable/using.html",
                "https://asv.readthedocs.io/en/stable/commands.html",
            ],
        },
        {
            "role": "self-hosted-performance-tracker",
            "service": "LLVM LNT",
            "state": "not-selected",
            "fit": "medium",
            "cost_model": "Open-source self-hosted performance tracking software.",
            "data_owner": "Self-hosted LNT server.",
            "ci_integration": "Translates benchmark outputs into LNT report JSON.",
            "approval_gate": "Infrastructure ownership decision.",
            "rationale": "Heavier server path.",
            "next_step": "Keep as future lab-scale option.",
            "evidence_urls": [
                "https://llvm.org/docs/lnt/",
                "https://llvm.org/docs/lnt/concepts.html",
                "https://llvm.org/docs/lnt/importing_data.html",
                "https://github.com/llvm/llvm-lnt",
            ],
        },
        {
            "role": "self-hosted-fallback",
            "service": "Conbench",
            "state": "fallback",
            "fit": "medium",
            "cost_model": "Open-source self-hosted stack.",
            "data_owner": "Self-hosted Conbench API.",
            "ci_integration": "JSON submissions.",
            "approval_gate": "Infrastructure ownership decision.",
            "rationale": "Heavier fallback.",
            "next_step": "Revisit if needed.",
            "evidence_urls": [
                "https://conbench.github.io/conbench/",
                "https://github.com/conbench/conbench",
            ],
        },
        {
            "role": "public-ecosystem-campaigns",
            "service": "OpenBenchmarking.org / Phoronix Test Suite",
            "state": "not-selected",
            "fit": "low",
            "cost_model": "Free public use.",
            "data_owner": "External OpenBenchmarking result store.",
            "ci_integration": "Requires a Phoronix Test Suite profile.",
            "approval_gate": "Maintainer-approved public campaign.",
            "rationale": "Future ecosystem campaigns.",
            "next_step": "Revisit after launch.",
            "evidence_urls": [
                "https://openbenchmarking.org/",
                "https://openbenchmarking.org/features",
            ],
        },
    ]


def _service_decision_summary():
    return {
        "best_option": (
            "Use DART-owned GitHub Pages as the canonical static dashboard and "
            "durable data host, with Bencher as an opt-in external "
            "history/threshold companion after maintainer approval."
        ),
        "selected_host": "DART-owned GitHub Pages",
        "selected_host_role": "primary-dashboard",
        "recommended_external_service": "Bencher Cloud or Self-Hosted",
        "recommended_external_role": "external-history-and-thresholds",
        "canonical_data_owner": "DART-owned static files.",
        "free_open_source_fit": (
            "The selected GitHub Pages host is free for DART's public GitHub "
            "repository, and the recommended Bencher companion has a free "
            "public/open-source path."
        ),
        "deferred_services": [
            "CodSpeed",
            "github-action-benchmark",
            "Airspeed Velocity",
            "LLVM LNT",
            "OpenBenchmarking.org / Phoronix Test Suite",
        ],
        "fallback_services": ["Conbench"],
        "approval_boundary": (
            "Remote pushes, PR creation, merges, workflow dispatches, Pages "
            "publication, and Bencher setup require explicit maintainer approval."
        ),
    }


def _publication_contract():
    return {
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
        "approval_boundary": (
            "Pushes, PR creation, merges, workflow dispatches, and Pages "
            "publication require explicit maintainer approval."
        ),
    }


def _comparison_input_contract():
    return {
        "format": "Google Benchmark JSON",
        "backend_suffix_rule": (
            "Name reproducible comparison rows as <benchmark>_<Backend> before "
            "optional Google Benchmark parameters."
        ),
        "primary_backend_rule": (
            "DART-owned rows use primary backends; external engines use "
            "supported_external_backends."
        ),
        "supported_external_backends": [
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
        ],
        "example_rows": [
            "BM_Distance_BoxSphere_Native",
            "BM_Distance_BoxSphere_MuJoCo",
        ],
        "claim_rule": "Only matching rows become a live competitor claim.",
    }


def _comparison_metric_contract():
    return {
        "time_unit": "nanoseconds",
        "primary_time_field": "primary_ns",
        "backend_time_field": "backend_ns",
        "ratio_field": "backend_vs_primary_ratio",
        "ratio_formula": "backend_ns / primary_ns",
        "latest_comparison_ratio_field": "ratio",
        "latest_comparison_ratio_formula": "native_ns / best_reference_ns",
        "interpretation": (
            "For backend matrix rows, ratio < 1 means the backend/reference "
            "row is faster than the primary DART row."
        ),
        "latest_comparison_interpretation": (
            "For latest native/reference comparison rows, ratio < 1 is a DART "
            "lead over the best reference."
        ),
        "status_meanings": {
            "primary": "primary row",
            "faster-than-primary": "backend faster",
            "slower-than-primary": "backend slower",
            "same-as-primary": "same",
            "lead": "DART lead",
            "behind": "DART behind",
            "native-only": "no reference",
        },
    }


def _trend_summary():
    return {
        "latest_run_id": "123-1",
        "history_runs": 1,
        "primary_trend_rows": 0,
        "reference_trend_rows": 0,
        "primary_status_counts": {},
        "reference_status_counts": {},
        "primary_threshold_counts": {},
        "reference_threshold_counts": {},
        "primary_largest_regression": None,
        "primary_largest_improvement": None,
        "reference_largest_regression": None,
        "reference_largest_improvement": None,
    }


def _testbed_summary():
    return {
        "latest_run_id": "123-1",
        "latest_run_at": "2026-05-21T00:00:00+00:00",
        "testbed": "unit-testbed",
        "history_runs": 1,
        "history_testbeds": ["unit-testbed"],
        "context_count": 0,
        "source_file_count": 0,
        "source_files": [],
        "host_names": [],
        "num_cpus": [],
        "cpu_scaling_counts": {"enabled": 0, "disabled": 0, "unknown": 0},
        "aslr_counts": {"enabled": 0, "disabled": 0, "unknown": 0},
        "interpretation": (
            "Trend interpretation is strongest when testbed, host, CPU count, "
            "and scaling state remain stable across compared runs."
        ),
    }


def _external_competitor_status(**overrides):
    status = {
        "state": "queued",
        "row_count": 0,
        "backends": [],
        "surfaces": [],
        "sample_groups": [],
        "supported_external_backends": _comparison_input_contract()[
            "supported_external_backends"
        ],
        "claim_rule": _comparison_input_contract()["claim_rule"],
        "data_source": "comparison_coverage",
    }
    status.update(overrides)
    return status


def _coverage_rows():
    return [
        {"surface": "collision", "state": "live"},
        {"surface": "common", "state": "queued"},
        {"surface": "dynamics", "state": "queued"},
        {"surface": "lcp", "state": "queued"},
        {"surface": "math", "state": "queued"},
        {"surface": "simd", "state": "queued"},
        {"surface": "simulation", "state": "queued"},
        {"surface": "compute", "state": "queued"},
        {"surface": "gpu", "state": "queued"},
    ]


def _comparison_coverage_rows():
    return [
        {
            "surface": surface,
            "state": "live" if surface == "collision" else "queued",
            "comparison_scope": f"{surface} comparison target",
            "entrypoint": f"pixi run bm-dashboard-surfaces --surface {surface}",
            "next_step": f"Add {surface} comparison rows.",
            "external_competitor_state": (
                "deferred until a reproducible external harness writes dashboard JSON"
            ),
            "external_competitor_backends": [],
            "external_competitor_row_count": 0,
            "external_competitor_group_count": 0,
            "external_competitor_sample_groups": [],
            "external_competitor_candidates": [f"{surface} external baseline"],
        }
        for surface in [
            "collision",
            "common",
            "dynamics",
            "lcp",
            "math",
            "simd",
            "simulation",
            "compute",
            "gpu",
        ]
    ]


def _status_file(tmp_path: Path, **overrides):
    status = {
        "schema_version": 1,
        "latest_run_id": "123-1",
        "sha": "abc123",
        "measurements": 12,
        "runs": 1,
        "live_surfaces": 1,
        "queued_surfaces": 8,
        "comparison_coverage_rows": 9,
        "external_competitor_rows": 0,
        "external_competitor_backends": [],
        "external_competitor_surfaces": [],
        "external_competitor_sample_groups": [],
        "reference_trend_rows": 0,
        "freshness": {
            "state": "fresh",
            "update_triggers": [
                "path-scoped main push",
                "scheduled workflow",
                "manual workflow_dispatch",
            ],
            "latest_run_age_hours": 4.0,
            "stale_after_hours": 120.0,
        },
        "dashboard_files": [
            "index.html",
            "data.json",
            "summary.md",
            "status.json",
        ],
        "links": _links(),
        "service_decision": _service_decision(),
        "service_decision_summary": _service_decision_summary(),
        "publication_contract": _publication_contract(),
        "comparison_input_contract": _comparison_input_contract(),
        "comparison_metric_contract": _comparison_metric_contract(),
        "trend_summary": _trend_summary(),
        "testbed_summary": _testbed_summary(),
        "external_competitor_status": _external_competitor_status(),
    }
    status.update(overrides)
    path = tmp_path / "status.json"
    path.write_text(json.dumps(status), encoding="utf-8")
    return path


def _data_file(tmp_path: Path, **overrides):
    data = {
        "schema_version": 1,
        "generated_at": "2026-05-21T00:00:00+00:00",
        "latest_run_id": "123-1",
        "runs": [
            {
                "run_id": "123-1",
                "run_at": "2026-05-21T00:00:00+00:00",
                "branch": "main",
                "sha": "abc123",
                "testbed": "unit-testbed",
                "contexts": [],
            }
        ],
        "measurements": [
            {"run_id": "123-1", "benchmark": f"BM_Test_{index}"}
            for index in range(12)
        ],
        "links": _links(),
        "service_decision": _service_decision(),
        "service_decision_summary": _service_decision_summary(),
        "publication_contract": _publication_contract(),
        "comparison_input_contract": _comparison_input_contract(),
        "comparison_metric_contract": _comparison_metric_contract(),
        "trend_summary": _trend_summary(),
        "testbed_summary": _testbed_summary(),
        "external_competitor_status": _external_competitor_status(),
        "coverage": _coverage_rows(),
        "comparison_coverage": _comparison_coverage_rows(),
    }
    data.update(overrides)
    path = tmp_path / "data.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _dashboard_page(tmp_path: Path) -> Path:
    path = tmp_path / "index.html"
    path.write_text(
        """
<html>
<body>
DART Performance Dashboard
<a href="status.json">status.json</a>
<a href="data.json">data.json</a>
<a href="summary.md">summary.md</a>
Service Decision Summary
service_decision_summary
best_option
Dashboard Service Decision
DART-owned GitHub Pages
Bencher Cloud or Self-Hosted
Airspeed Velocity
LLVM LNT
OpenBenchmarking.org / Phoronix Test Suite
Publication Contract
.github/workflows/performance_dashboard.yml
refs/heads/main
schedule_crons
30 3 * * 0,3
actions: read
pages: write
required_website_links
required_workflow_artifacts
performance-dashboard-site-&lt;run_id&gt;-&lt;run_attempt&gt;
Comparison Metric Contract
backend_ns / primary_ns
comparison_input_contract
comparison_metric_contract
External Competitor Status
external_competitor_status
Testbed Summary
testbed_summary
Trend Summary
trend_summary
BM_Distance_BoxSphere_MuJoCo
Reference And Competitor Coverage
</body>
</html>
""",
        encoding="utf-8",
    )
    return path


def _summary_file(tmp_path: Path) -> Path:
    path = tmp_path / "summary.md"
    path.write_text(
        """
# DART Performance Dashboard Summary

## Service Decision Summary
best_option

## Dashboard Service Decision
DART-owned GitHub Pages
Bencher Cloud or Self-Hosted
Airspeed Velocity
LLVM LNT
OpenBenchmarking.org / Phoronix Test Suite

## Publication Contract
.github/workflows/performance_dashboard.yml
refs/heads/main
schedule_crons
30 3 * * 0,3
actions: read
pages: write
Required website links
performance-dashboard-site-<run_id>-<run_attempt>

## Comparison Input Contract
BM_Distance_BoxSphere_MuJoCo

## Comparison Metric Contract
backend_ns / primary_ns

## External Competitor Status
live competitor

## Testbed Summary
testbed_summary

## Trend Summary

## Reference And Competitor Coverage
""",
        encoding="utf-8",
    )
    return path


def test_check_performance_dashboard_endpoint_accepts_status_file(tmp_path):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-run-id",
            "123-1",
            "--expect-sha",
            "abc123",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "verified dashboard endpoint" in result.stdout
    assert "run=123-1" in result.stdout
    assert "measurements=12" in result.stdout
    assert "freshness=fresh" in result.stdout


def test_check_performance_dashboard_endpoint_accepts_plain_file_path(tmp_path):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(path),
            "--expect-run-id",
            "123-1",
            "--expect-sha",
            "abc123",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "verified dashboard endpoint" in result.stdout


def test_check_performance_dashboard_endpoint_accepts_expected_status_file(tmp_path):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-status-file",
            str(path),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "verified dashboard endpoint" in result.stdout
    assert "run=123-1" in result.stdout


def test_check_performance_dashboard_endpoint_accepts_matching_data_file(tmp_path):
    path = _status_file(tmp_path)
    data = _data_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-status-file",
            str(path),
            "--expect-data-file",
            str(data),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "verified dashboard endpoint" in result.stdout
    assert "data=verified" in result.stdout


def test_check_performance_dashboard_endpoint_accepts_required_page_and_summary(
    tmp_path,
):
    path = _status_file(tmp_path)
    page = _dashboard_page(tmp_path)
    summary = _summary_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-dashboard-page",
            "--dashboard-url",
            page.as_uri(),
            "--require-summary",
            "--summary-url",
            summary.as_uri(),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "page=verified" in result.stdout
    assert "summary=verified" in result.stdout


def test_check_performance_dashboard_endpoint_accepts_matching_page_and_summary(
    tmp_path,
):
    path = _status_file(tmp_path)
    page = _dashboard_page(tmp_path)
    summary = _summary_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-dashboard-file",
            str(page),
            "--expect-summary-file",
            str(summary),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "page=verified" in result.stdout
    assert "summary=verified" in result.stdout


def test_check_performance_dashboard_endpoint_rejects_missing_dashboard_page(
    tmp_path,
):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-dashboard-page",
            "--dashboard-url",
            (tmp_path / "missing.html").as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "missing.html" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_incomplete_dashboard_page(
    tmp_path,
):
    path = _status_file(tmp_path)
    page = tmp_path / "index.html"
    page.write_text("<html>DART Performance Dashboard</html>", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-dashboard-page",
            "--dashboard-url",
            page.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "generated dashboard" in result.stderr
    assert "Publication Contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_page_mismatch(tmp_path):
    path = _status_file(tmp_path)
    page = _dashboard_page(tmp_path)
    expected = tmp_path / "expected-index.html"
    expected.write_text("<html>DART Performance Dashboard changed</html>", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--expect-dashboard-file",
            str(expected),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "does not match expected index.html" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_summary_mismatch(tmp_path):
    path = _status_file(tmp_path)
    summary = _summary_file(tmp_path)
    expected = tmp_path / "expected-summary.md"
    expected.write_text(
        "# DART Performance Dashboard Summary\n\nChanged.\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-summary-file",
            str(expected),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "does not match expected summary.md" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_incomplete_summary(tmp_path):
    path = _status_file(tmp_path)
    summary = tmp_path / "summary.md"
    summary.write_text("# DART Performance Dashboard Summary\n", encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-summary",
            "--summary-url",
            summary.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "generated summary.md" in result.stderr
    assert "Publication Contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_data_when_required(
    tmp_path,
):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-data",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "data.json" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_data_mismatch(tmp_path):
    path = _status_file(tmp_path)
    _data_file(tmp_path)
    expected = tmp_path / "expected-data.json"
    expected.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "generated_at": "2026-05-21T01:00:00+00:00",
                "latest_run_id": "123-1",
                "runs": [
                    {
                        "run_id": "123-1",
                        "run_at": "2026-05-21T00:00:00+00:00",
                        "branch": "main",
                        "sha": "abc123",
                        "testbed": "unit-testbed",
                    }
                ],
                "measurements": [
                    {"run_id": "123-1", "benchmark": f"BM_Test_{index}"}
                    for index in range(12)
                ],
                "links": _links(),
                "service_decision": _service_decision(),
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(expected),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "does not match expected data.json" in result.stderr


def test_check_performance_dashboard_endpoint_accepts_required_freshness(tmp_path):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-fresh",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "freshness=fresh" in result.stdout


def test_check_performance_dashboard_endpoint_allows_stale_without_requirement(
    tmp_path,
):
    path = _status_file(
        tmp_path,
        freshness={
            "state": "stale",
            "update_triggers": [
                "path-scoped main push",
                "scheduled workflow",
                "manual workflow_dispatch",
            ],
            "latest_run_age_hours": 130.0,
            "stale_after_hours": 120.0,
        },
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "freshness=stale" in result.stdout


def test_check_performance_dashboard_endpoint_rejects_stale_when_required(
    tmp_path,
):
    path = _status_file(
        tmp_path,
        freshness={
            "state": "stale",
            "update_triggers": [
                "path-scoped main push",
                "scheduled workflow",
                "manual workflow_dispatch",
            ],
            "latest_run_age_hours": 130.0,
            "stale_after_hours": 120.0,
        },
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--require-fresh",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "not fresh" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_run_mismatch(tmp_path):
    path = _status_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-run-id",
            "other-run",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected latest_run_id" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_incomplete_manifest(tmp_path):
    path = _status_file(tmp_path, dashboard_files=["status.json"])

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "file manifest is incomplete" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_freshness(tmp_path):
    path = _status_file(tmp_path, freshness={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "freshness status" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_update_triggers(
    tmp_path,
):
    path = _status_file(
        tmp_path,
        freshness={
            "state": "fresh",
            "latest_run_age_hours": 4.0,
            "stale_after_hours": 120.0,
        },
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "update triggers" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_links(tmp_path):
    path = _status_file(tmp_path, links={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "public link keys" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_link_mismatch(tmp_path):
    links = _links()
    links["canonical_website_url"] = "https://dartsim.github.io/"
    path = _status_file(tmp_path, links=links)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "public links do not match" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_service_decision(
    tmp_path,
):
    path = _status_file(tmp_path, service_decision=[])

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "service decision roles" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_service_decision_mismatch(
    tmp_path,
):
    service_decision = _service_decision()
    service_decision[0]["service"] = "other host"
    path = _status_file(tmp_path, service_decision=service_decision)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "selected DART architecture" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_service_evidence(
    tmp_path,
):
    service_decision = _service_decision()
    service_decision[0].pop("cost_model")
    path = _status_file(tmp_path, service_decision=service_decision)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "evaluation fields" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_unconfigured_service_url(
    tmp_path,
):
    service_decision = _service_decision()
    service_decision[0]["evidence_urls"] = ["https://example.invalid/source"]
    path = _status_file(tmp_path, service_decision=service_decision)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "unconfigured evidence URLs" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_service_summary_mismatch(
    tmp_path,
):
    summary = _service_decision_summary()
    summary["selected_host"] = "other host"
    path = _status_file(tmp_path, service_decision_summary=summary)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "service decision summary" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_publication_contract(
    tmp_path,
):
    path = _status_file(tmp_path, publication_contract={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "publication contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_publication_contract_mismatch(
    tmp_path,
):
    contract = _publication_contract()
    contract["source_branch"] = "main"
    path = _status_file(tmp_path, publication_contract=contract)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Pages publisher contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_data_publication_drift(
    tmp_path,
):
    data_contract = _publication_contract()
    data_contract["required_permissions"] = ["contents: write"]
    path = _status_file(tmp_path)
    data = _data_file(tmp_path, publication_contract=data_contract)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "publication contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_input_contract(
    tmp_path,
):
    path = _status_file(tmp_path, comparison_input_contract={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison input contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_input_contract_mismatch(
    tmp_path,
):
    data_contract = _comparison_input_contract()
    data_contract["example_rows"] = ["BM_Different_Native"]
    path = _status_file(tmp_path)
    data = _data_file(tmp_path, comparison_input_contract=data_contract)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison input contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_metric_contract(
    tmp_path,
):
    path = _status_file(tmp_path, comparison_metric_contract={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison metric contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_metric_contract_mismatch(
    tmp_path,
):
    data_contract = _comparison_metric_contract()
    data_contract["ratio_formula"] = "primary_ns / backend_ns"
    path = _status_file(tmp_path)
    data = _data_file(tmp_path, comparison_metric_contract=data_contract)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison metric contract" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_trend_summary(
    tmp_path,
):
    path = _status_file(tmp_path, trend_summary={})

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "trend summary" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_trend_summary_mismatch(
    tmp_path,
):
    status_summary = _trend_summary()
    data_summary = _trend_summary()
    data_summary["primary_trend_rows"] = 1
    data_summary["primary_status_counts"] = {"slower": 1}
    data_summary["primary_threshold_counts"] = {"informational-regression": 1}
    path = _status_file(tmp_path, trend_summary=status_summary)
    data = _data_file(
        tmp_path,
        trend_summary=data_summary,
        latest_trends=[
            {
                "status": "slower",
                "threshold_status": "informational-regression",
            }
        ],
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "trend summary" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_testbed_summary_mismatch(
    tmp_path,
):
    status_summary = _testbed_summary()
    data_summary = _testbed_summary()
    data_summary["context_count"] = 1
    data_summary["cpu_scaling_counts"] = {
        "enabled": 0,
        "disabled": 0,
        "unknown": 1,
    }
    data_summary["aslr_counts"] = {
        "enabled": 0,
        "disabled": 0,
        "unknown": 1,
    }
    path = _status_file(tmp_path, testbed_summary=status_summary)
    data = _data_file(tmp_path, testbed_summary=data_summary)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "testbed summary" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_comparison_surface(
    tmp_path,
):
    path = _status_file(tmp_path)
    comparison_coverage = [
        row
        for row in _comparison_coverage_rows()
        if row["surface"] != "simulation"
    ]
    data = _data_file(tmp_path, comparison_coverage=comparison_coverage)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison coverage is missing surfaces" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_competitor_candidates(
    tmp_path,
):
    path = _status_file(tmp_path)
    comparison_coverage = _comparison_coverage_rows()
    comparison_coverage[0]["external_competitor_candidates"] = []
    data = _data_file(tmp_path, comparison_coverage=comparison_coverage)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "external competitor candidates" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_external_competitor_row_mismatch(
    tmp_path,
):
    path = _status_file(tmp_path, external_competitor_rows=4)
    data = _data_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "external competitor row count" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_external_status_mismatch(
    tmp_path,
):
    path = _status_file(
        tmp_path,
        external_competitor_status=_external_competitor_status(row_count=4),
    )
    data = _data_file(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "external competitor status" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_comparison_scope(
    tmp_path,
):
    path = _status_file(tmp_path)
    comparison_coverage = _comparison_coverage_rows()
    comparison_coverage[0].pop("comparison_scope")
    data = _data_file(tmp_path, comparison_coverage=comparison_coverage)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "missing comparison_scope" in result.stderr


def test_check_performance_dashboard_endpoint_rejects_missing_external_backends(
    tmp_path,
):
    path = _status_file(tmp_path)
    comparison_coverage = _comparison_coverage_rows()
    comparison_coverage[0].pop("external_competitor_backends")
    data = _data_file(tmp_path, comparison_coverage=comparison_coverage)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            path.as_uri(),
            "--expect-data-file",
            str(data),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "external competitor backends" in result.stderr
