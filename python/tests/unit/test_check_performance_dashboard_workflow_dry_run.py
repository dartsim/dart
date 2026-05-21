import json
import importlib.util
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_workflow_dry_run.py"
WORKFLOW = ROOT / ".github/workflows/performance_dashboard.yml"
FIXTURE = ROOT / "tests/fixtures/performance_dashboard/google_benchmark_sample.json"

SPEC = importlib.util.spec_from_file_location("dashboard_workflow_dry_run", SCRIPT)
assert SPEC and SPEC.loader
DRY_RUN = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(DRY_RUN)


def test_workflow_dry_run_writes_gh_pages_performance_tree(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input",
            str(FIXTURE),
            "--work-dir",
            str(tmp_path),
            "--seed-root",
            str(tmp_path / "no-seeds"),
            "--run-id",
            "dry-run",
            "--run-at",
            "2026-05-21T00:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--source-url",
            "https://example.invalid/actions/runs/dry-run",
            "--testbed",
            "unit-testbed",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    performance_dir = tmp_path / "gh-pages" / "performance"
    workflow_summary = tmp_path / "workflow-summary.md"
    status = json.loads((performance_dir / "status.json").read_text(encoding="utf-8"))

    assert "verified workflow dashboard dry run" in result.stdout
    assert (performance_dir / "index.html").is_file()
    assert (performance_dir / "data.json").is_file()
    assert (performance_dir / "summary.md").is_file()
    assert workflow_summary.is_file()
    summary = workflow_summary.read_text(encoding="utf-8")
    assert "DART Performance Dashboard" in summary
    assert "### Service Decision Summary" in summary
    assert "best_option" in summary
    assert "### Service Decision" in summary
    assert "### Publication Contract" in summary
    assert "| Pages source | `gh-pages` / |" in summary
    assert "| Publish ref | `refs/heads/main` |" in summary
    assert "schedule_crons" in summary
    assert "30 3 * * 0,3" in summary
    assert "| Required website links |" in summary
    assert "https://dart.readthedocs.io/en/latest/community/performance_dashboard.html" in summary
    assert "pixi run check-bm-dashboard-launch-live" in summary
    assert "performance-dashboard-site-<run_id>-<run_attempt>" in summary
    assert "### External Competitor Status" in summary
    assert "live competitor" in summary
    assert "### Testbed Summary" in summary
    assert "### Trend Summary" in summary
    assert "primary_status_counts" in summary
    assert status["latest_run_id"] == "dry-run"
    assert status["sha"] == "abc123"
    assert status["testbed"] == "unit-testbed"
    assert status["measurements"] == 3


def test_workflow_dry_run_allows_empty_first_publication(tmp_path):
    work_dir = tmp_path / "workflow"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--work-dir",
            str(work_dir),
            "--seed-root",
            str(tmp_path / "no-seeds"),
            "--run-id",
            "empty-dry-run",
            "--run-at",
            "2026-05-21T00:00:00+00:00",
        ],
        check=True,
        cwd=tmp_path,
        text=True,
        stdout=subprocess.PIPE,
    )

    status = json.loads(
        (work_dir / "gh-pages" / "performance" / "status.json").read_text(
            encoding="utf-8"
        )
    )
    workflow_summary = work_dir / "workflow-summary.md"

    assert "verified workflow dashboard dry run" in result.stdout
    assert workflow_summary.is_file()
    assert status["latest_run_id"] == "empty-dry-run"
    assert status["measurements"] == 0
    assert status["freshness"]["state"] == "fresh"


def test_workflow_dry_run_rejects_incomplete_workflow_summary(tmp_path):
    summary = tmp_path / "workflow-summary.md"
    summary.write_text("## DART Performance Dashboard\n", encoding="utf-8")

    try:
        DRY_RUN._verify_workflow_summary(summary)
    except RuntimeError as exc:
        assert "launch metadata" in str(exc)
    else:
        raise AssertionError("Expected incomplete workflow summary to fail")


def test_workflow_requests_pages_build_after_publish_step():
    workflow = WORKFLOW.read_text(encoding="utf-8")

    assert "pages: write" in workflow
    assert "--clean-output" in workflow
    assert "id: publish_dashboard" in workflow
    assert "steps.publish_dashboard.outputs.published == 'true'" not in workflow
    assert "if: github.ref == 'refs/heads/main'" in workflow
    assert '"/repos/${GITHUB_REPOSITORY}/pages/builds"' in workflow
    assert "--expect-data-file gh-pages/performance/data.json" in workflow
    assert "--attempts 60" in workflow
    assert "--interval 15" in workflow
