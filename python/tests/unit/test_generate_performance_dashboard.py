import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "generate_performance_dashboard.py"
FIXTURE = ROOT / "tests/fixtures/performance_dashboard/google_benchmark_sample.json"


def _run_dashboard(
    input_path: Path | list[Path],
    output_dir: Path,
    run_id: str,
    run_at: str = "2026-05-20T08:00:00+00:00",
    *extra_args: str,
) -> None:
    input_paths = input_path if isinstance(input_path, list) else [input_path]
    command = [
        sys.executable,
        str(SCRIPT),
    ]
    for path in input_paths:
        command.extend(["--input", str(path)])
    command.extend(
        [
            "--output-dir",
            str(output_dir),
            "--run-id",
            run_id,
            "--run-at",
            run_at,
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--source-url",
            f"https://example.invalid/runs/{run_id}",
            "--testbed",
            "unit-testbed",
            *extra_args,
        ]
    )
    subprocess.run(
        command,
        check=True,
    )


def test_generate_performance_dashboard_outputs_schema_and_html(tmp_path):
    output_dir = tmp_path / "dashboard"

    _run_dashboard(FIXTURE, output_dir, "sample-run")

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))
    html = (output_dir / "index.html").read_text(encoding="utf-8")
    summary = (output_dir / "summary.md").read_text(encoding="utf-8")

    assert data["schema_version"] == 1
    assert data["latest_run_id"] == "sample-run"
    assert data["links"] == {
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
    service_by_role = {item["role"]: item for item in data["service_decision"]}
    assert service_by_role["primary-dashboard"]["service"] == (
        "DART-owned GitHub Pages"
    )
    assert service_by_role["primary-dashboard"]["state"] == "selected"
    assert "Free" in service_by_role["primary-dashboard"]["cost_model"]
    assert "gh-pages/performance" in service_by_role["primary-dashboard"]["data_owner"]
    assert service_by_role["primary-dashboard"]["evidence_urls"]
    assert service_by_role["external-history-and-thresholds"]["service"] == (
        "Bencher Cloud or Self-Hosted"
    )
    assert service_by_role["external-history-and-thresholds"]["fit"] == "high"
    assert "open-source" in service_by_role["external-history-and-thresholds"][
        "cost_model"
    ]
    assert "cpp_google" in service_by_role["external-history-and-thresholds"][
        "ci_integration"
    ]
    assert service_by_role["external-history-and-thresholds"]["evidence_urls"]
    assert service_by_role["off-the-shelf-pages-action"]["service"] == (
        "github-action-benchmark"
    )
    assert service_by_role["off-the-shelf-pages-action"]["state"] == "not-selected"
    assert "googlecpp" in service_by_role["off-the-shelf-pages-action"][
        "ci_integration"
    ]
    assert service_by_role["static-benchmark-dashboard-framework"]["service"] == (
        "Airspeed Velocity"
    )
    assert (
        service_by_role["static-benchmark-dashboard-framework"]["state"]
        == "not-selected"
    )
    assert "Google Benchmark JSON" in service_by_role[
        "static-benchmark-dashboard-framework"
    ]["ci_integration"]
    assert service_by_role["self-hosted-performance-tracker"]["service"] == "LLVM LNT"
    assert service_by_role["self-hosted-performance-tracker"]["state"] == (
        "not-selected"
    )
    assert "LNT report JSON" in service_by_role[
        "self-hosted-performance-tracker"
    ]["ci_integration"]
    assert "self-hosted" in service_by_role["self-hosted-performance-tracker"][
        "cost_model"
    ]
    assert service_by_role["public-ecosystem-campaigns"]["service"] == (
        "OpenBenchmarking.org / Phoronix Test Suite"
    )
    assert service_by_role["public-ecosystem-campaigns"]["state"] == "not-selected"
    assert "Phoronix Test Suite" in service_by_role["public-ecosystem-campaigns"][
        "ci_integration"
    ]
    service_summary = data["service_decision_summary"]
    assert service_summary == status["service_decision_summary"]
    assert service_summary["selected_host"] == "DART-owned GitHub Pages"
    assert service_summary["recommended_external_service"] == (
        "Bencher Cloud or Self-Hosted"
    )
    assert "GitHub Pages" in service_summary["best_option"]
    assert "Bencher" in service_summary["best_option"]
    assert "free" in service_summary["free_open_source_fit"].lower()
    assert "CodSpeed" in service_summary["deferred_services"]
    assert service_summary["fallback_services"] == ["Conbench"]
    assert "maintainer approval" in service_summary["approval_boundary"]
    assert "Service Decision Summary" in html
    assert "service_decision_summary" in html
    assert "## Service Decision Summary" in summary
    publication = data["publication_contract"]
    assert publication == status["publication_contract"]
    assert publication["host"] == "GitHub Pages"
    assert publication["site_url"] == "https://dartsim.github.io/dart/performance/"
    assert publication["source_branch"] == "gh-pages"
    assert publication["source_path"] == "/"
    assert publication["dashboard_path"] == "performance/"
    assert publication["publisher_workflow"] == (
        ".github/workflows/performance_dashboard.yml"
    )
    assert publication["workflow_name"] == "Performance Dashboard"
    assert publication["publish_ref"] == "refs/heads/main"
    assert publication["schedule_crons"] == ["30 3 * * 0,3"]
    assert publication["expected_update_interval_hours"] == 96.0
    assert publication["stale_after_hours"] == 120.0
    assert publication["requires_pages_build_request"] is True
    assert publication["required_permissions"] == [
        "actions: read",
        "contents: write",
        "pages: write",
    ]
    assert publication["required_endpoints"] == [
        "https://dartsim.github.io/dart/performance/",
        "https://dartsim.github.io/dart/performance/status.json",
        "https://dartsim.github.io/dart/performance/data.json",
        "https://dartsim.github.io/dart/performance/summary.md",
    ]
    assert publication["required_website_links"] == [
        "https://dartsim.github.io/dart/performance/",
        "https://dartsim.github.io/dart/performance/status.json",
        "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html",
    ]
    assert publication["required_workflow_artifacts"] == [
        "performance-dashboard-site-<run_id>-<run_attempt>",
    ]
    assert "pixi run check-bm-dashboard-pages-build" in publication["launch_checks"]
    assert "pixi run check-bm-dashboard-workflow-run" in publication["launch_checks"]
    assert "pixi run check-bm-dashboard-launch-live" in publication["launch_checks"]
    assert "maintainer approval" in publication["approval_boundary"]
    contract = data["comparison_input_contract"]
    assert contract == status["comparison_input_contract"]
    assert contract["format"] == "Google Benchmark JSON"
    assert "MuJoCo" in contract["supported_external_backends"]
    assert "Drake" in contract["supported_external_backends"]
    assert "BM_Distance_BoxSphere_MuJoCo" in contract["example_rows"]
    assert "_<Backend>" in contract["backend_suffix_rule"]
    assert "live competitor" in contract["claim_rule"]
    metric_contract = data["comparison_metric_contract"]
    assert metric_contract == status["comparison_metric_contract"]
    assert metric_contract["time_unit"] == "nanoseconds"
    assert metric_contract["ratio_field"] == "backend_vs_primary_ratio"
    assert metric_contract["ratio_formula"] == "backend_ns / primary_ns"
    assert metric_contract["latest_comparison_ratio_formula"] == (
        "native_ns / best_reference_ns"
    )
    assert "ratio < 1" in metric_contract["interpretation"]
    assert "behind" in metric_contract["status_meanings"]
    assert data["thresholds"] == {
        "mode": "informational",
        "required_gate": False,
        "primary_change_percent": 1.0,
        "reference_change_percent": 1.0,
        "description": (
            "Dashboard trend labels use this threshold for review signals only; "
            "they are not required PR gates until runner variance is characterized."
        ),
    }
    assert data["freshness"]["mode"] == "ci-publisher"
    assert data["freshness"]["update_triggers"] == [
        "path-scoped main push",
        "scheduled workflow",
        "manual workflow_dispatch",
    ]
    assert data["freshness"]["state"] == "fresh"
    assert data["freshness"]["expected_update_interval_hours"] == 96.0
    assert data["freshness"]["stale_after_hours"] == 120.0
    assert data["freshness"]["latest_run_at"] == "2026-05-20T08:00:00+00:00"
    assert len(data["runs"]) == 1
    assert len(data["measurements"]) == 3
    assert data["latest_summary"]["comparisons"] == {
        "lead": 1,
        "native-only": 1,
    }
    assert status["latest_run_id"] == "sample-run"
    assert status["measurements"] == 3
    assert status["links"] == data["links"]
    assert status["service_decision"] == data["service_decision"]
    assert status["publication_contract"] == data["publication_contract"]
    assert status["comparison_metric_contract"] == data["comparison_metric_contract"]
    external_status = data["external_competitor_status"]
    assert external_status == status["external_competitor_status"]
    assert external_status["state"] == "queued"
    assert external_status["row_count"] == 0
    assert external_status["backends"] == []
    assert external_status["surfaces"] == []
    assert external_status["sample_groups"] == []
    assert "MuJoCo" in external_status["supported_external_backends"]
    assert "live competitor" in external_status["claim_rule"]
    assert external_status["data_source"] == "comparison_coverage"
    assert status["thresholds"] == data["thresholds"]
    assert status["freshness"] == data["freshness"]
    assert status["live_surfaces"] == 1
    assert status["backend_summary_rows"] == 1
    assert status["backend_matrix_rows"] == 3
    assert status["reference_trend_rows"] == 1
    assert status["comparison_coverage_rows"] == 9
    assert status["external_competitor_rows"] == 0
    assert status["external_competitor_backends"] == []
    assert status["external_competitor_surfaces"] == []
    assert status["external_competitor_sample_groups"] == []
    assert status["dashboard_files"] == [
        "index.html",
        "data.json",
        "summary.md",
        "status.json",
    ]
    coverage_by_surface = {item["surface"]: item for item in data["coverage"]}
    assert coverage_by_surface["collision"]["state"] == "live"
    assert coverage_by_surface["collision"]["latest_measurements"] == 3
    assert coverage_by_surface["lcp"]["state"] == "queued"
    comparison_coverage_by_surface = {
        item["surface"]: item for item in data["comparison_coverage"]
    }
    assert comparison_coverage_by_surface["collision"]["state"] == "live"
    assert comparison_coverage_by_surface["collision"]["primary_backends"] == ["Native"]
    assert comparison_coverage_by_surface["collision"]["reference_backends"] == [
        "Bullet"
    ]
    assert comparison_coverage_by_surface["collision"]["comparison_group_count"] == 1
    assert (
        comparison_coverage_by_surface["collision"]["external_competitor_state"]
        == "deferred until a reproducible external harness writes dashboard JSON"
    )
    assert comparison_coverage_by_surface["collision"][
        "external_competitor_candidates"
    ] == [
        "MuJoCo collision queries",
        "Drake geometry queries",
        "Gazebo/gz-physics collision queries",
    ]
    assert comparison_coverage_by_surface["collision"]["comparison_scope"] == (
        "Native, FCL, Bullet, ODE where harnesses exist"
    )
    assert comparison_coverage_by_surface["collision"]["entrypoint"] == (
        "pixi run bm-collision-check"
    )
    assert comparison_coverage_by_surface["collision"]["next_step"] == (
        "Live first slice; keep extending reference-backed rows."
    )
    assert comparison_coverage_by_surface["lcp"]["state"] == "queued"
    assert comparison_coverage_by_surface["compute"]["state"] == "queued"
    assert comparison_coverage_by_surface["gpu"]["state"] == "queued"

    comparison = data["latest_comparisons"][0]
    assert comparison["status"] == "lead"
    assert comparison["comparable_group"] == "BM_Distance_BoxBox"
    assert comparison["best_reference_backend"] == "Bullet"
    assert comparison["ratio"] == 0.5
    matrix_by_backend = {
        (item["comparable_group"], item["backend"]): item
        for item in data["latest_backend_matrix"]
    }
    assert matrix_by_backend[("BM_Distance_BoxBox", "Native")]["status"] == "primary"
    assert (
        matrix_by_backend[("BM_Distance_BoxBox", "Native")]["backend_vs_primary_ratio"]
        == 1.0
    )
    assert (
        matrix_by_backend[("BM_Distance_BoxBox", "Bullet")]["status"]
        == "slower-than-primary"
    )
    assert (
        matrix_by_backend[("BM_Distance_BoxBox", "Bullet")]["backend_vs_primary_ratio"]
        == 2.0
    )
    summary_by_backend = {
        (item["surface"], item["backend"]): item
        for item in data["latest_backend_summary"]
    }
    assert summary_by_backend[("collision", "Bullet")]["status"] == "primary-faster"
    assert summary_by_backend[("collision", "Bullet")]["groups"] == 1
    assert summary_by_backend[("collision", "Bullet")]["primary_faster"] == 1
    assert (
        summary_by_backend[("collision", "Bullet")]["geomean_backend_vs_primary_ratio"]
        == 2.0
    )
    assert data["latest_trends"][0]["status"] == "new"
    assert data["latest_trends"][0]["threshold_status"] == "new"
    assert data["latest_trends"][0]["threshold_percent"] == 1.0
    trend_summary = data["trend_summary"]
    assert trend_summary == status["trend_summary"]
    assert trend_summary["latest_run_id"] == "sample-run"
    assert trend_summary["history_runs"] == 1
    assert trend_summary["primary_trend_rows"] == 2
    assert trend_summary["reference_trend_rows"] == 1
    assert trend_summary["primary_status_counts"] == {"new": 2}
    assert trend_summary["reference_status_counts"] == {"new": 1}
    assert trend_summary["primary_threshold_counts"] == {"new": 2}
    assert trend_summary["reference_threshold_counts"] == {"new": 1}
    assert trend_summary["primary_largest_regression"] is None
    assert trend_summary["reference_largest_regression"] is None
    testbed_summary = data["testbed_summary"]
    assert testbed_summary == status["testbed_summary"]
    assert testbed_summary["latest_run_id"] == "sample-run"
    assert testbed_summary["latest_run_at"] == "2026-05-20T08:00:00+00:00"
    assert testbed_summary["testbed"] == "unit-testbed"
    assert testbed_summary["history_runs"] == 1
    assert testbed_summary["history_testbeds"] == ["unit-testbed"]
    assert testbed_summary["context_count"] == 1
    assert testbed_summary["source_file_count"] == 1
    assert testbed_summary["source_files"] == ["google_benchmark_sample.json"]
    assert testbed_summary["host_names"] == ["sample-runner"]
    assert testbed_summary["num_cpus"] == [4]
    assert testbed_summary["cpu_scaling_counts"] == {
        "enabled": 0,
        "disabled": 1,
        "unknown": 0,
    }
    assert testbed_summary["aslr_counts"] == {
        "enabled": 0,
        "disabled": 0,
        "unknown": 1,
    }
    assert "Trend interpretation" in testbed_summary["interpretation"]
    reference_trend = data["latest_reference_trends"][0]
    assert reference_trend["status"] == "new"
    assert reference_trend["threshold_status"] == "new"
    assert reference_trend["threshold_percent"] == 1.0
    assert reference_trend["backend"] == "Bullet"
    assert reference_trend["latest_backend_vs_primary_ratio"] == 2.0

    assert "DART Performance Dashboard" in html
    assert 'href="status.json"' in html
    assert 'href="data.json"' in html
    assert 'href="summary.md"' in html
    assert (
        'href="https://dart.readthedocs.io/en/latest/community/'
        'performance_dashboard.html"'
    ) in html
    assert "Change Threshold" in html
    assert "Dashboard Service Decision" in html
    assert "DART-owned GitHub Pages" in html
    assert "Bencher Cloud or Self-Hosted" in html
    assert "github-action-benchmark" in html
    assert "Airspeed Velocity" in html
    assert "LLVM LNT" in html
    assert "OpenBenchmarking.org / Phoronix Test Suite" in html
    assert "Publication Contract" in html
    assert ".github/workflows/performance_dashboard.yml" in html
    assert "refs/heads/main" in html
    assert "required_website_links" in html
    assert "required_workflow_artifacts" in html
    assert "performance-dashboard-site-&lt;run_id&gt;-&lt;run_attempt&gt;" in html
    assert "pixi run check-bm-dashboard-launch-live" in html
    assert "Benchmark Surface Coverage" in html
    assert "Reference And Competitor Coverage" in html
    assert "External Competitor Status" in html
    assert "external_competitor_status" in html
    assert "Native, FCL, Bullet, ODE where harnesses exist" in html
    assert "MuJoCo collision queries" in html
    assert "Latest Changes vs Previous Run" in html
    assert "Trend Summary" in html
    assert "trend_summary" in html
    assert "Testbed Summary" in html
    assert "testbed_summary" in html
    assert "sample-runner" in html
    assert "Latest Reference Backend Changes vs Previous Run" in html
    assert "Latest Reference Backend Summary" in html
    assert "Latest Backend Matrix" in html
    assert "BM_Distance_BoxBox" in html
    assert "sample-run" in summary
    assert "- Threshold policy: `informational`" in summary
    assert "- Informational change threshold: `1.0%`" in summary
    assert "## Dashboard Service Decision" in summary
    assert "primary-dashboard | DART-owned GitHub Pages" in summary
    assert "Cost / OSS fit" in summary
    assert "cpp_google" in summary
    assert "github-action-benchmark" in summary
    assert "Airspeed Velocity" in summary
    assert "LLVM LNT" in summary
    assert "OpenBenchmarking.org / Phoronix Test Suite" in summary
    assert "## Publication Contract" in summary
    assert "Required website links" in summary
    assert "performance-dashboard-site-<run_id>-<run_attempt>" in summary
    assert "pixi run check-bm-dashboard-launch-live" in summary
    assert "workflow dispatches" in summary
    assert "## Comparison Input Contract" in summary
    assert "BM_Distance_BoxSphere_MuJoCo" in summary
    assert "Comparison Metric Contract" in html
    assert "backend_ns / primary_ns" in html
    assert "Comparison Metric Contract" in summary
    assert "backend_ns / primary_ns" in summary
    assert "## External Competitor Status" in summary
    assert "`claim_rule`" in summary
    assert "## Testbed Summary" in summary
    assert "`context_count`" in summary
    assert "sample-runner" in summary
    assert "BM_KR5ForwardDynamics_Drake/1" in summary
    assert "## Benchmark Surface Coverage" in summary
    assert "## Reference And Competitor Coverage" in summary
    assert "## Trend Summary" in summary
    assert "`primary_status_counts`" in summary
    assert "Native, FCL, Bullet, ODE where harnesses exist" in summary
    assert "candidates: MuJoCo collision queries" in summary
    assert "## Latest Reference Backend Changes vs Previous Run" in summary
    assert "## Latest Reference Backend Summary" in summary
    assert "## Latest Backend Matrix" in summary


def test_generate_performance_dashboard_clean_output_removes_stale_files(tmp_path):
    output_dir = tmp_path / "dashboard"
    stale_dir = output_dir / "old-assets"
    stale_dir.mkdir(parents=True)
    (output_dir / "old.html").write_text("stale", encoding="utf-8")
    (stale_dir / "old.json").write_text("{}", encoding="utf-8")

    _run_dashboard(
        FIXTURE,
        output_dir,
        "sample-run",
        "2026-05-20T08:00:00+00:00",
        "--clean-output",
    )

    assert not (output_dir / "old.html").exists()
    assert not stale_dir.exists()
    assert (output_dir / "data.json").is_file()
    assert (output_dir / "index.html").is_file()
    assert (output_dir / "summary.md").is_file()
    assert (output_dir / "status.json").is_file()


def test_generate_performance_dashboard_recognizes_external_competitor_rows(tmp_path):
    output_dir = tmp_path / "dashboard"
    competitor_fixture = tmp_path / "simulation_external.json"
    competitor_fixture.write_text(
        json.dumps(
            {
                "context": {
                    "date": "2026-05-20T08:00:00+00:00",
                    "executable": "build/sample/cpp/Release/bin/bm_simulation_external",
                    "json_schema_version": 1,
                },
                "benchmarks": [
                    {
                        "name": "BM_WorldStep_DART/1_median",
                        "run_name": "BM_WorldStep_DART/1",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 10.0,
                        "cpu_time": 10.0,
                        "time_unit": "us",
                    },
                    {
                        "name": "BM_WorldStep_MuJoCo/1_median",
                        "run_name": "BM_WorldStep_MuJoCo/1",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 8.0,
                        "cpu_time": 8.0,
                        "time_unit": "us",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )

    _run_dashboard(competitor_fixture, output_dir, "competitor-run")

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))
    html = (output_dir / "index.html").read_text(encoding="utf-8")
    summary = (output_dir / "summary.md").read_text(encoding="utf-8")
    comparison_coverage_by_surface = {
        item["surface"]: item for item in data["comparison_coverage"]
    }
    simulation_coverage = comparison_coverage_by_surface["simulation"]

    assert simulation_coverage["state"] == "live"
    assert simulation_coverage["primary_backends"] == ["DART"]
    assert simulation_coverage["reference_backends"] == ["MuJoCo"]
    assert simulation_coverage["external_competitor_state"] == (
        "live reproducible external harness rows in dashboard JSON"
    )
    assert simulation_coverage["external_competitor_backends"] == ["MuJoCo"]
    assert simulation_coverage["external_competitor_row_count"] == 1
    assert simulation_coverage["external_competitor_group_count"] == 1
    assert simulation_coverage["external_competitor_sample_groups"] == [
        "BM_WorldStep/1"
    ]
    assert status["external_competitor_rows"] == 1
    assert status["external_competitor_backends"] == ["MuJoCo"]
    assert status["external_competitor_surfaces"] == ["simulation"]
    assert status["external_competitor_sample_groups"] == ["BM_WorldStep/1"]
    external_status = data["external_competitor_status"]
    assert external_status == status["external_competitor_status"]
    assert external_status["state"] == "live"
    assert external_status["row_count"] == 1
    assert external_status["backends"] == ["MuJoCo"]
    assert external_status["surfaces"] == ["simulation"]
    assert external_status["sample_groups"] == ["BM_WorldStep/1"]
    comparison = data["latest_comparisons"][0]
    assert comparison["status"] == "behind"
    assert comparison["primary_backend"] == "DART"
    assert comparison["best_reference_backend"] == "MuJoCo"
    assert comparison["ratio"] == 1.25
    assert "live reproducible external harness rows" in html
    assert "MuJoCo" in html
    assert "live: MuJoCo (1 rows)" in summary


def test_generate_performance_dashboard_can_render_empty_hosting_page(tmp_path):
    output_dir = tmp_path / "dashboard"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "empty-run",
            "--run-at",
            "2026-05-20T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--allow-empty",
        ],
        check=True,
        cwd=tmp_path,
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))
    html = (output_dir / "index.html").read_text(encoding="utf-8")

    assert data["latest_run_id"] == "empty-run"
    assert data["latest_summary"]["measurements"] == 0
    assert data["latest_summary"]["surfaces"] == {}
    assert status["latest_run_id"] == "empty-run"
    assert status["measurements"] == 0
    assert status["freshness"]["state"] == "fresh"
    assert status["backend_matrix_rows"] == 0
    assert status["reference_trend_rows"] == 0
    assert status["comparison_coverage_rows"] == 9
    assert all(item["state"] == "queued" for item in data["coverage"])
    assert all(item["state"] == "queued" for item in data["comparison_coverage"])
    assert "DART Performance Dashboard" in html
    assert "Benchmark Surface Coverage" in html


def test_generate_performance_dashboard_allows_explicit_missing_default_input(
    tmp_path,
):
    output_dir = tmp_path / "dashboard"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            ".benchmark_results",
            "--output-dir",
            str(output_dir),
            "--run-id",
            "empty-run",
            "--run-at",
            "2026-05-20T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--allow-empty",
        ],
        check=True,
        cwd=tmp_path,
    )

    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))

    assert status["latest_run_id"] == "empty-run"
    assert status["measurements"] == 0
    assert status["freshness"]["state"] == "fresh"
    assert status["reference_trend_rows"] == 0
    assert status["comparison_coverage_rows"] == 9


def test_generate_performance_dashboard_skips_empty_directory_inputs(tmp_path):
    input_dir = tmp_path / "inputs"
    input_dir.mkdir()
    (input_dir / "empty.json").write_text("", encoding="utf-8")
    (input_dir / "sample.json").write_text(
        FIXTURE.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    output_dir = tmp_path / "dashboard"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(input_dir),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "sample-run",
            "--run-at",
            "2026-05-20T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
        ],
        check=True,
    )

    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))

    assert status["latest_run_id"] == "sample-run"
    assert status["measurements"] == 3


def test_generate_performance_dashboard_preserves_history_on_empty_followup(tmp_path):
    output_dir = tmp_path / "dashboard"

    _run_dashboard(FIXTURE, output_dir, "sample-run")
    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(output_dir),
            "--history",
            str(output_dir / "data.json"),
            "--run-id",
            "empty-followup",
            "--run-at",
            "2026-05-21T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "def456",
            "--allow-empty",
        ],
        check=True,
        cwd=tmp_path,
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    status = json.loads((output_dir / "status.json").read_text(encoding="utf-8"))

    assert data["latest_run_id"] == "sample-run"
    assert [run["run_id"] for run in data["runs"]] == ["sample-run"]
    assert len(data["measurements"]) == 3
    assert status["latest_run_id"] == "sample-run"
    assert status["measurements"] == 3
    assert status["sha"] == "abc123"


def test_generate_performance_dashboard_seeds_history_when_empty(tmp_path):
    output_dir = tmp_path / "dashboard"
    current_fixture = tmp_path / "current.json"

    current = json.loads(FIXTURE.read_text(encoding="utf-8"))
    current["context"]["date"] = "2026-05-21T08:00:00+00:00"
    for row in current["benchmarks"]:
        if row.get("run_name") == "BM_Distance_BoxBox_Native":
            row["real_time"] = 8.0
            row["cpu_time"] = 8.0
    current_fixture.write_text(json.dumps(current), encoding="utf-8")

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--seed-input",
            str(FIXTURE),
            "--input",
            str(current_fixture),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "current-run",
            "--run-at",
            "2026-05-21T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "def456",
        ],
        check=True,
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))

    assert [run["run_id"] for run in data["runs"]] == [
        "seed-2026-05-20T08-00-00-00-00-performance-dashboard",
        "current-run",
    ]
    assert data["latest_run_id"] == "current-run"
    assert len(data["measurements"]) == 6
    trend = next(
        item
        for item in data["latest_trends"]
        if item["comparable_group"] == "BM_Distance_BoxBox"
    )
    assert trend["status"] == "faster"
    assert (
        trend["previous_run_id"]
        == "seed-2026-05-20T08-00-00-00-00-performance-dashboard"
    )
    assert trend["latest_run_id"] == "current-run"
    assert trend["native_change_percent"] == pytest.approx(-20.0)
    assert trend["threshold_status"] == "informational-improvement"
    assert data["trend_summary"]["primary_largest_improvement"]["change_percent"] == (
        pytest.approx(-20.0)
    )


def test_generate_performance_dashboard_keeps_same_timestamp_seed_artifacts(
    tmp_path,
):
    first_seed = tmp_path / "collision-benchmark-guard-1"
    second_seed = tmp_path / "performance-dashboard-raw-2"
    first_seed.mkdir()
    second_seed.mkdir()
    (first_seed / "sample.json").write_text(
        FIXTURE.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    second = json.loads(FIXTURE.read_text(encoding="utf-8"))
    for row in second["benchmarks"]:
        if row.get("run_name") == "BM_Distance_BoxBox_Native":
            row["real_time"] = 11.0
            row["cpu_time"] = 11.0
    (second_seed / "sample.json").write_text(json.dumps(second), encoding="utf-8")
    current_fixture = tmp_path / "current.json"
    current = json.loads(FIXTURE.read_text(encoding="utf-8"))
    current["context"]["date"] = "2026-05-21T08:00:00+00:00"
    current_fixture.write_text(json.dumps(current), encoding="utf-8")
    output_dir = tmp_path / "dashboard"

    subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--seed-input",
            str(first_seed),
            "--seed-input",
            str(second_seed),
            "--input",
            str(current_fixture),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "current-run",
            "--run-at",
            "2026-05-21T08:00:00+00:00",
        ],
        check=True,
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))

    assert [run["run_id"] for run in data["runs"]] == [
        "seed-2026-05-20T08-00-00-00-00-collision-benchmark-guard-1",
        "seed-2026-05-20T08-00-00-00-00-performance-dashboard-raw-2",
        "current-run",
    ]
    assert len(data["measurements"]) == 9


def test_generate_performance_dashboard_marks_non_collision_surfaces_live(tmp_path):
    output_dir = tmp_path / "dashboard"
    dynamics_fixture = tmp_path / "dashboard_dynamics_kinematics.json"
    math_fixture = tmp_path / "dashboard_math_helpers.json"
    simulation_fixture = tmp_path / "dashboard_simulation_world_step.json"
    compute_fixture = tmp_path / "dashboard_compute_graph.json"
    dynamics_fixture.write_text(
        json.dumps(
            {
                "context": {
                    "date": "2026-05-20T08:00:00+00:00",
                    "executable": "build/sample/cpp/Release/bin/bm_kinematics",
                    "json_schema_version": 1,
                },
                "benchmarks": [
                    {
                        "name": "BM_Kinematics/1_median",
                        "run_name": "BM_Kinematics/1",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 5.0,
                        "cpu_time": 5.0,
                        "time_unit": "us",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    math_fixture.write_text(
        json.dumps(
            {
                "context": {
                    "date": "2026-05-20T08:00:00+00:00",
                    "executable": "build/sample/cpp/Release/bin/bm_helpers",
                    "json_schema_version": 1,
                },
                "benchmarks": [
                    {
                        "name": "BM_isNan_Baseline/3_median",
                        "run_name": "BM_isNan_Baseline/3",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 10.0,
                        "cpu_time": 10.0,
                        "time_unit": "us",
                    },
                    {
                        "name": "BM_isNan_Optimized/3_median",
                        "run_name": "BM_isNan_Optimized/3",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 4.0,
                        "cpu_time": 4.0,
                        "time_unit": "us",
                    },
                ],
            }
        ),
        encoding="utf-8",
    )
    simulation_fixture.write_text(
        json.dumps(
            {
                "context": {
                    "date": "2026-05-20T08:00:00+00:00",
                    "executable": "build/sample/cpp/Release/bin/bm_compute_graph",
                    "json_schema_version": 1,
                },
                "benchmarks": [
                    {
                        "name": "BM_WorldStepSequential/32/8_median",
                        "run_name": "BM_WorldStepSequential/32/8",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 177.0,
                        "cpu_time": 177.0,
                        "time_unit": "us",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )
    compute_fixture.write_text(
        json.dumps(
            {
                "context": {
                    "date": "2026-05-20T08:00:00+00:00",
                    "executable": "build/sample/cpp/Release/bin/bm_compute_graph",
                    "json_schema_version": 1,
                },
                "benchmarks": [
                    {
                        "name": "BM_ComputeGraphSequential/1024/32_median",
                        "run_name": "BM_ComputeGraphSequential/1024/32",
                        "run_type": "aggregate",
                        "aggregate_name": "median",
                        "repetitions": 3,
                        "iterations": 3,
                        "real_time": 3.0,
                        "cpu_time": 3.0,
                        "time_unit": "us",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )

    _run_dashboard(
        [FIXTURE, dynamics_fixture, math_fixture, simulation_fixture, compute_fixture],
        output_dir,
        "surface-run",
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    coverage_by_surface = {item["surface"]: item for item in data["coverage"]}

    assert coverage_by_surface["collision"]["state"] == "live"
    assert coverage_by_surface["dynamics"]["state"] == "live"
    assert coverage_by_surface["dynamics"]["latest_measurements"] == 1
    assert coverage_by_surface["math"]["state"] == "live"
    assert coverage_by_surface["math"]["latest_measurements"] == 2
    assert coverage_by_surface["simulation"]["state"] == "live"
    assert coverage_by_surface["simulation"]["latest_measurements"] == 1
    assert coverage_by_surface["compute"]["state"] == "live"
    assert coverage_by_surface["compute"]["latest_measurements"] == 1
    assert coverage_by_surface["common"]["state"] == "queued"
    comparison_coverage_by_surface = {
        item["surface"]: item for item in data["comparison_coverage"]
    }
    assert comparison_coverage_by_surface["dynamics"]["state"] == "primary-only"
    assert comparison_coverage_by_surface["simulation"]["state"] == "primary-only"
    assert comparison_coverage_by_surface["simulation"]["entrypoint"] == (
        "pixi run bm-dashboard-surfaces --surface simulation"
    )
    assert comparison_coverage_by_surface["compute"]["state"] == "primary-only"
    assert comparison_coverage_by_surface["compute"]["entrypoint"] == (
        "pixi run bm-dashboard-surfaces --surface compute"
    )
    assert comparison_coverage_by_surface["math"]["state"] == "live"
    assert comparison_coverage_by_surface["math"]["primary_backends"] == ["Optimized"]
    assert comparison_coverage_by_surface["math"]["reference_backends"] == ["Baseline"]
    assert data["latest_summary"]["surfaces"] == {
        "collision": 3,
        "compute": 1,
        "dynamics": 1,
        "math": 2,
        "simulation": 1,
    }
    math_comparison = next(
        item
        for item in data["latest_comparisons"]
        if item["comparable_group"] == "BM_isNan/3"
    )
    assert math_comparison["status"] == "lead"
    assert math_comparison["primary_backend"] == "Optimized"
    assert math_comparison["best_reference_backend"] == "Baseline"
    assert math_comparison["ratio"] == 0.4
    math_baseline = next(
        item
        for item in data["latest_backend_matrix"]
        if item["comparable_group"] == "BM_isNan/3" and item["backend"] == "Baseline"
    )
    assert math_baseline["status"] == "slower-than-primary"
    assert math_baseline["backend_vs_primary_ratio"] == 2.5
    assert math_baseline["primary_backend"] == "Optimized"
    math_summary = next(
        item
        for item in data["latest_backend_summary"]
        if item["surface"] == "math" and item["backend"] == "Baseline"
    )
    assert math_summary["status"] == "primary-faster"
    assert math_summary["groups"] == 1
    assert math_summary["primary_faster"] == 1
    assert math_summary["geomean_backend_vs_primary_ratio"] == 2.5


def test_generate_performance_dashboard_preserves_history_and_trends(tmp_path):
    output_dir = tmp_path / "dashboard"
    slower_fixture = tmp_path / "slower.json"

    first = json.loads(FIXTURE.read_text(encoding="utf-8"))
    second = json.loads(FIXTURE.read_text(encoding="utf-8"))
    second["context"]["date"] = "2026-05-21T08:00:00+00:00"
    for row in second["benchmarks"]:
        if row.get("run_name") == "BM_Distance_BoxBox_Native":
            row["real_time"] = 12.0
            row["cpu_time"] = 12.0
    slower_fixture.write_text(json.dumps(second), encoding="utf-8")

    _run_dashboard(FIXTURE, output_dir, "sample-run-1")
    _run_dashboard(
        slower_fixture,
        output_dir,
        "sample-run-2",
        run_at="2026-05-21T08:00:00+00:00",
    )

    data = json.loads((output_dir / "data.json").read_text(encoding="utf-8"))
    html = (output_dir / "index.html").read_text(encoding="utf-8")
    summary = (output_dir / "summary.md").read_text(encoding="utf-8")

    assert len(data["runs"]) == 2
    assert len(data["measurements"]) == 6
    assert data["coverage"][0]["history_runs"] == 2

    trend = next(
        item
        for item in data["latest_trends"]
        if item["comparable_group"] == "BM_Distance_BoxBox"
    )
    assert trend["status"] == "slower"
    assert trend["previous_run_id"] == "sample-run-1"
    assert trend["latest_run_id"] == "sample-run-2"
    assert trend["previous_native_ns"] == 10.0
    assert trend["latest_native_ns"] == 12.0
    assert trend["native_change_percent"] == pytest.approx(20.0)
    assert trend["threshold_status"] == "informational-regression"
    assert trend["latest_ratio"] == pytest.approx(0.6)
    assert trend["ratio_change_percent"] == pytest.approx(20.0)
    reference_trend = next(
        item
        for item in data["latest_reference_trends"]
        if item["comparable_group"] == "BM_Distance_BoxBox"
        and item["backend"] == "Bullet"
    )
    assert reference_trend["status"] == "flat"
    assert reference_trend["threshold_status"] == "within-threshold"
    assert reference_trend["previous_backend_ns"] == 20.0
    assert reference_trend["latest_backend_ns"] == 20.0
    assert reference_trend["latest_backend_vs_primary_ratio"] == pytest.approx(
        20.0 / 12.0
    )
    assert reference_trend["ratio_change_percent"] == pytest.approx(-16.6666666667)

    assert '<svg class="spark"' in html
    assert "20.0% slower" in summary
