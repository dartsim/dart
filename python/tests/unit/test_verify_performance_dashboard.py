import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
GENERATE = ROOT / "scripts" / "generate_performance_dashboard.py"
VERIFY = ROOT / "scripts" / "verify_performance_dashboard.py"
FIXTURE = ROOT / "tests/fixtures/performance_dashboard/google_benchmark_sample.json"


def _generate_dashboard(output_dir: Path) -> None:
    subprocess.run(
        [
            sys.executable,
            str(GENERATE),
            "--input",
            str(FIXTURE),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "verify-run",
            "--run-at",
            "2026-05-20T08:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
            "--testbed",
            "unit-testbed",
        ],
        check=True,
    )


def test_verify_performance_dashboard_accepts_generated_output(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir), "--require-measurements"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "verified dashboard" in result.stdout


def test_verify_performance_dashboard_rejects_inconsistent_status(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["measurements"] += 1
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Latest measurement count mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_missing_update_triggers(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    for name in ("data.json", "status.json"):
        path = output_dir / name
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["freshness"].pop("update_triggers")
        path.write_text(json.dumps(payload), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "expected update triggers" in result.stderr


def test_verify_performance_dashboard_rejects_public_link_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    for name in ("data.json", "status.json"):
        path = output_dir / name
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["links"]["canonical_website_url"] = "https://dartsim.github.io/"
        path.write_text(json.dumps(payload), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "public links do not match" in result.stderr


def test_verify_performance_dashboard_rejects_input_contract_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["comparison_input_contract"]["example_rows"] = ["BM_Different_Native"]
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison input contract mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_metric_contract_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["comparison_metric_contract"]["ratio_formula"] = "primary_ns / backend_ns"
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison metric contract mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_trend_summary_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["trend_summary"]["primary_status_counts"] = {"slower": 99}
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "trend summary mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_testbed_summary_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["testbed_summary"]["context_count"] += 1
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "testbed summary mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_publication_contract_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["publication_contract"]["source_branch"] = "main"
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "publication contract mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_service_evidence_url_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    data_path = output_dir / "data.json"
    data = json.loads(data_path.read_text(encoding="utf-8"))
    data["service_decision"][0]["evidence_urls"] = ["https://example.invalid/source"]
    data_path.write_text(json.dumps(data), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "evidence URLs unconfigured" in result.stderr


def test_verify_performance_dashboard_rejects_service_summary_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["service_decision_summary"]["selected_host"] = "other host"
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "service decision summary mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_missing_html_launch_metadata(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    (output_dir / "index.html").write_text(
        "<html>DART Performance Dashboard</html>",
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Dashboard HTML is missing launch metadata" in result.stderr


def test_verify_performance_dashboard_rejects_missing_summary_launch_metadata(
    tmp_path,
):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    (output_dir / "summary.md").write_text(
        "# DART Performance Dashboard Summary\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Dashboard summary is missing launch metadata" in result.stderr


def test_verify_performance_dashboard_rejects_missing_comparison_surface(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    data_path = output_dir / "data.json"
    data = json.loads(data_path.read_text(encoding="utf-8"))
    data["comparison_coverage"] = [
        item
        for item in data["comparison_coverage"]
        if item["surface"] != "simulation"
    ]
    data_path.write_text(json.dumps(data), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison coverage is missing surfaces" in result.stderr


def test_verify_performance_dashboard_rejects_external_competitor_row_mismatch(
    tmp_path,
):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["external_competitor_rows"] += 1
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "External competitor row count mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_external_status_mismatch(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["external_competitor_status"]["row_count"] += 1
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "External competitor status mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_external_competitor_backend_mismatch(
    tmp_path,
):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    status_path = output_dir / "status.json"
    status = json.loads(status_path.read_text(encoding="utf-8"))
    status["external_competitor_backends"] = ["MuJoCo"]
    status_path.write_text(json.dumps(status), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "External competitor backend mismatch" in result.stderr


def test_verify_performance_dashboard_rejects_missing_comparison_scope(tmp_path):
    output_dir = tmp_path / "dashboard"
    _generate_dashboard(output_dir)
    data_path = output_dir / "data.json"
    data = json.loads(data_path.read_text(encoding="utf-8"))
    data["comparison_coverage"][0].pop("comparison_scope")
    data_path.write_text(json.dumps(data), encoding="utf-8")

    result = subprocess.run(
        [sys.executable, str(VERIFY), str(output_dir)],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "comparison_scope missing" in result.stderr
