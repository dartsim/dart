import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "summarize_performance_dashboard.py"


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
            "row is faster than the primary DART row, ratio > 1 means the "
            "primary DART row is faster, and ratio == 1 means parity within "
            "the recorded precision."
        ),
        "latest_comparison_interpretation": (
            "For latest native/reference comparison rows, ratio < 1 is a DART "
            "lead over the best reference, ratio > 1 is DART behind the best "
            "reference, and null means no reproducible reference row exists."
        ),
        "status_meanings": {
            "primary": "The row is the selected primary DART-owned baseline.",
            "faster-than-primary": (
                "The backend/reference row is faster than the primary row."
            ),
            "slower-than-primary": (
                "The backend/reference row is slower than the primary row."
            ),
            "same-as-primary": "The backend/reference row matches the primary row.",
            "lead": "The primary DART row is faster than the best reference row.",
            "behind": "The primary DART row is slower than the best reference row.",
            "native-only": "No reproducible reference row exists for that group.",
        },
    }


def _status_file(tmp_path: Path) -> Path:
    status = {
        "latest_run_id": "123-1",
        "latest_run_at": "2026-05-21T12:00:00+00:00",
        "branch": "main",
        "sha": "abc123",
        "testbed": "github-Linux-X64",
        "measurements": 17,
        "runs": 3,
        "live_surfaces": 2,
        "queued_surfaces": 5,
        "comparison_states": {
            "lead": 4,
            "behind": 1,
        },
        "thresholds": {
            "mode": "informational",
            "required_gate": False,
            "primary_change_percent": 1.0,
            "reference_change_percent": 1.0,
        },
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
        "backend_summary_rows": 6,
        "backend_matrix_rows": 19,
        "reference_trend_rows": 9,
        "trend_summary": {
            "latest_run_id": "123-1",
            "history_runs": 3,
            "primary_trend_rows": 7,
            "reference_trend_rows": 9,
            "primary_status_counts": {"slower": 2, "faster": 1, "flat": 4},
            "reference_status_counts": {
                "reference-slower": 3,
                "reference-faster": 2,
                "flat": 4,
            },
            "primary_threshold_counts": {
                "informational-regression": 2,
                "informational-improvement": 1,
                "within-threshold": 4,
            },
            "reference_threshold_counts": {
                "informational-regression": 3,
                "informational-improvement": 2,
                "within-threshold": 4,
            },
            "primary_largest_regression": {
                "surface": "collision",
                "family": "Distance",
                "comparable_group": "BM_Distance_BoxSphere",
                "status": "slower",
                "threshold_status": "informational-regression",
                "change_percent": 12.5,
                "latest_run_id": "123-1",
                "previous_run_id": "122-1",
            },
            "primary_largest_improvement": None,
            "reference_largest_regression": None,
            "reference_largest_improvement": None,
        },
        "testbed_summary": {
            "latest_run_id": "123-1",
            "latest_run_at": "2026-05-21T12:00:00+00:00",
            "testbed": "github-Linux-X64",
            "history_runs": 3,
            "history_testbeds": ["github-Linux-X64"],
            "context_count": 2,
            "source_file_count": 2,
            "source_files": ["collision.json", "simulation.json"],
            "host_names": ["runner-a"],
            "num_cpus": [4],
            "cpu_scaling_counts": {
                "enabled": 0,
                "disabled": 2,
                "unknown": 0,
            },
            "aslr_counts": {
                "enabled": 1,
                "disabled": 1,
                "unknown": 0,
            },
            "interpretation": (
                "Trend interpretation is strongest when testbed, host, CPU "
                "count, and scaling state remain stable across compared runs."
            ),
        },
        "comparison_coverage_rows": 9,
        "external_competitor_rows": 2,
        "external_competitor_backends": ["Drake", "MuJoCo"],
        "external_competitor_surfaces": ["collision", "simulation"],
        "external_competitor_sample_groups": ["BM_Distance_BoxSphere"],
        "external_competitor_status": {
            "state": "live",
            "row_count": 2,
            "backends": ["Drake", "MuJoCo"],
            "surfaces": ["collision", "simulation"],
            "sample_groups": ["BM_Distance_BoxSphere"],
            "supported_external_backends": ["Drake", "MuJoCo"],
            "claim_rule": (
                "A live competitor claim requires a reproducible dashboard "
                "benchmark row."
            ),
            "data_source": "comparison_coverage",
        },
        "links": {
            "canonical_website_url": "https://dart.readthedocs.io/en/latest/",
            "dashboard_url": "https://dartsim.github.io/dart/performance/",
            "status_url": "https://dartsim.github.io/dart/performance/status.json",
            "data_url": "https://dartsim.github.io/dart/performance/data.json",
            "summary_url": "https://dartsim.github.io/dart/performance/summary.md",
            "guide_url": (
                "https://dart.readthedocs.io/en/latest/community/"
                "performance_dashboard.html"
            ),
        },
        "publication_contract": {
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
        },
        "comparison_metric_contract": _comparison_metric_contract(),
        "service_decision_summary": {
            "best_option": (
                "Use DART-owned GitHub Pages as the canonical static dashboard "
                "and durable data host, with Bencher as an opt-in external "
                "history/threshold companion after maintainer approval."
            ),
            "selected_host": "DART-owned GitHub Pages",
            "recommended_external_service": "Bencher Cloud or Self-Hosted",
            "free_open_source_fit": "Free public/open-source path.",
            "deferred_services": ["CodSpeed"],
            "fallback_services": ["Conbench"],
            "approval_boundary": (
                "Remote pushes, PR creation, merges, workflow dispatches, "
                "Pages publication, and Bencher setup require explicit "
                "maintainer approval."
            ),
        },
        "service_decision": [
            {
                "role": "primary-dashboard",
                "service": "DART-owned GitHub Pages",
                "state": "selected",
                "fit": "high",
                "rationale": "Primary static dashboard host.",
                "next_step": "Publish gh-pages/performance.",
            },
            {
                "role": "external-history-and-thresholds",
                "service": "Bencher Cloud or Self-Hosted",
                "state": "recommended-after-maintainer-approval",
                "fit": "high",
                "rationale": "External history and thresholds.",
                "next_step": "Configure project and key.",
            },
        ],
    }
    path = tmp_path / "status.json"
    path.write_text(json.dumps(status), encoding="utf-8")
    return path


def test_summarize_performance_dashboard_writes_step_summary(tmp_path):
    status = _status_file(tmp_path)
    summary = tmp_path / "summary.md"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(status),
            "--summary-file",
            str(summary),
            "--bencher-configured",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    text = summary.read_text(encoding="utf-8")
    assert "wrote dashboard summary" in result.stdout
    assert "## DART Performance Dashboard" in text
    assert "| Latest run | 123-1 |" in text
    assert "| Canonical website | [Read the Docs](https://dart.readthedocs.io/en/latest/) |" in text
    assert (
        "| Dashboard guide | [Read the guide](https://dart.readthedocs.io/en/latest/community/performance_dashboard.html) |"
        in text
    )
    assert "| Text summary | [summary.md](https://dartsim.github.io/dart/performance/summary.md) |" in text
    assert "### Service Decision Summary" in text
    assert "| `selected_host` | `DART-owned GitHub Pages` |" in text
    assert "| `recommended_external_service` | `Bencher Cloud or Self-Hosted` |" in text
    assert "### Service Decision" in text
    assert "| primary-dashboard | DART-owned GitHub Pages | selected | high |" in text
    assert (
        "| external-history-and-thresholds | Bencher Cloud or Self-Hosted |"
        in text
    )
    assert "| Measurements | 17 |" in text
    assert "| Comparison states | behind: 1, lead: 4 |" in text
    assert (
        "| Freshness | fresh, latest run age 4.0h, stale after 120.0h, "
        "triggers: path-scoped main push, scheduled workflow, "
        "manual workflow_dispatch |"
    ) in text
    assert (
        "| Threshold policy | informational, 1.0% primary/reference change, informational |"
        in text
    )
    assert "| Reference trend rows | 9 |" in text
    assert "| Comparison coverage rows | 9 |" in text
    assert "| External competitor rows | 2 |" in text
    assert "| External competitor backends | `Drake`<br>`MuJoCo` |" in text
    assert "| External competitor surfaces | `collision`<br>`simulation` |" in text
    assert "| Bencher configured | yes |" in text
    assert "### Publication Contract" in text
    assert "| Pages source | `gh-pages` / |" in text
    assert "| Publish ref | `refs/heads/main` |" in text
    assert "| schedule_crons | `30 3 * * 0,3` |" in text
    assert "| expected_update_interval_hours | `96.0` |" in text
    assert "| stale_after_hours | `120.0` |" in text
    assert "`actions: read`<br>`contents: write`<br>`pages: write`" in text
    assert "| Required website links |" in text
    assert "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html" in text
    assert "performance-dashboard-site-<run_id>-<run_attempt>" in text
    assert "pixi run check-bm-dashboard-launch-live" in text
    assert "workflow dispatches" in text
    assert "https://dartsim.github.io/dart/performance/status.json" in text
    assert "### Comparison Metric Contract" in text
    assert "| Time unit | `nanoseconds` |" in text
    assert (
        "| Backend matrix ratio | `backend_vs_primary_ratio` = "
        "`backend_ns / primary_ns` |"
    ) in text
    assert (
        "| Latest comparison ratio | `ratio` = "
        "`native_ns / best_reference_ns` |"
    ) in text
    assert "ratio < 1 means the backend/reference row is faster" in text
    assert "### External Competitor Status" in text
    assert "| `state` | `live` |" in text
    assert "| `row_count` | `2` |" in text
    assert "| `backends` | `Drake`<br>`MuJoCo` |" in text
    assert "| `data_source` | `comparison_coverage` |" in text
    assert "A live competitor claim requires" in text
    assert "### Testbed Summary" in text
    assert "| `context_count` | `2` |" in text
    assert "| `host_names` | `runner-a` |" in text
    assert "`disabled`: `2`" in text
    assert "### Trend Summary" in text
    assert "| `primary_trend_rows` | `7` |" in text
    assert "`slower`: `2`" in text
    assert "`change_percent`: `12.5`" in text


def test_summarize_performance_dashboard_can_use_environment_summary(tmp_path):
    status = _status_file(tmp_path)
    summary = tmp_path / "env-summary.md"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(status),
        ],
        check=True,
        env={"GITHUB_STEP_SUMMARY": str(summary)},
    )

    assert "| Bencher configured | no |" in summary.read_text(encoding="utf-8")


def test_summarize_performance_dashboard_uses_default_summary_url(tmp_path):
    status = json.loads(_status_file(tmp_path).read_text(encoding="utf-8"))
    status["links"].pop("summary_url")
    status_path = tmp_path / "status-without-summary-link.json"
    status_path.write_text(json.dumps(status), encoding="utf-8")
    summary = tmp_path / "summary.md"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(status_path),
            "--summary-file",
            str(summary),
        ],
        check=True,
    )

    assert (
        "| Text summary | [summary.md](https://dartsim.github.io/dart/performance/summary.md) |"
        in summary.read_text(encoding="utf-8")
    )
