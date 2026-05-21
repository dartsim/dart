import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
GENERATE = ROOT / "scripts" / "generate_performance_dashboard.py"
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_publication.py"
FIXTURE = ROOT / "tests/fixtures/performance_dashboard/google_benchmark_sample.json"


def _generate_dashboard(tmp_path: Path) -> Path:
    output_dir = tmp_path / "dashboard"
    subprocess.run(
        [
            sys.executable,
            str(GENERATE),
            "--input",
            str(FIXTURE),
            "--output-dir",
            str(output_dir),
            "--run-id",
            "publication-run",
            "--run-at",
            "2026-05-21T00:00:00+00:00",
            "--branch",
            "main",
            "--sha",
            "abc123",
        ],
        check=True,
    )
    return output_dir


def _website(tmp_path: Path, *, include_dashboard_links: bool = True) -> Path:
    path = tmp_path / "website.html"
    links = ""
    if include_dashboard_links:
        links = """
<a href="https://dartsim.github.io/dart/performance/">Dashboard</a>
<a href="https://dartsim.github.io/dart/performance/status.json">Status</a>
<a href="community/performance_dashboard.html">Guide</a>
"""
    path.write_text(f"<html>DART{links}</html>", encoding="utf-8")
    return path


def test_check_performance_dashboard_publication_accepts_matching_public_status(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-local-match",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "local dashboard verified" in result.stdout
    assert "canonical website reachable" in result.stdout
    assert "dashboard guide reachable" in result.stdout
    assert "public dashboard endpoint verified" in result.stdout
    assert "page=verified" in result.stdout
    assert "summary=verified" in result.stdout


def test_check_performance_dashboard_publication_rejects_page_mismatch(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    page = tmp_path / "index.html"
    page.write_text(
        (dashboard / "index.html").read_text(encoding="utf-8")
        + "\n<!-- changed -->\n",
        encoding="utf-8",
    )
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-local-match",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "does not match expected index.html" in result.stderr


def test_check_performance_dashboard_publication_allows_unpublished_endpoint(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            (tmp_path / "missing.json").as_uri(),
            "--allow-unpublished",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "public dashboard not verified yet" in result.stdout


def test_check_performance_dashboard_publication_reports_unpublished_guide(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            (tmp_path / "missing-guide.html").as_uri(),
            "--status-url",
            (tmp_path / "missing.json").as_uri(),
            "--allow-unpublished",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "dashboard guide not verified yet" in result.stdout
    assert "public dashboard not verified yet" in result.stdout


def test_check_performance_dashboard_publication_rejects_unlinked_website(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path, include_dashboard_links=False)
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-local-match",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "missing dashboard links" in result.stderr


def test_check_performance_dashboard_publication_reports_all_unpublished_endpoints(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    public_dir = tmp_path / "public"
    public_dir.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            (tmp_path / "missing-guide.html").as_uri(),
            "--status-url",
            (public_dir / "status.json").as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Public dashboard publication checks failed:" in result.stderr
    assert "missing-guide.html" in result.stderr
    assert "status.json" in result.stderr
    assert "data.json" in result.stderr
    assert "index.html" in result.stderr
    assert "summary.md" in result.stderr


def test_check_performance_dashboard_publication_reports_website_and_endpoint_failures(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path, include_dashboard_links=False)
    public_dir = tmp_path / "public"
    public_dir.mkdir()

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            (tmp_path / "missing-guide.html").as_uri(),
            "--status-url",
            (public_dir / "status.json").as_uri(),
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Public dashboard publication checks failed:" in result.stderr
    assert "canonical website" in result.stderr
    assert "missing dashboard links" in result.stderr
    assert "missing-guide.html" in result.stderr
    assert "status.json" in result.stderr
    assert "data.json" in result.stderr
    assert "index.html" in result.stderr
    assert "summary.md" in result.stderr


def test_check_performance_dashboard_publication_rejects_stale_public_status(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    status = json.loads((dashboard / "status.json").read_text(encoding="utf-8"))
    status["freshness"]["state"] = "stale"
    status["freshness"]["latest_run_age_hours"] = 130.0
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(json.dumps(status), encoding="utf-8")
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-local-match",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "not fresh" in result.stderr


def test_check_performance_dashboard_publication_rejects_missing_public_data(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--allow-unpublished",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "data.json" in result.stderr


def test_check_performance_dashboard_publication_rejects_missing_public_page(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            (tmp_path / "missing-page.html").as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-local-match",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "missing-page.html" in result.stderr


def test_check_performance_dashboard_publication_rejects_unexpected_run_id(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-run-id",
            "newer-main-run",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected latest_run_id 'newer-main-run'" in result.stderr


def test_check_performance_dashboard_publication_rejects_unexpected_sha(
    tmp_path,
):
    dashboard = _generate_dashboard(tmp_path)
    website = _website(tmp_path)
    page = dashboard / "index.html"
    summary = dashboard / "summary.md"
    public_dir = tmp_path / "public"
    public_dir.mkdir()
    public_status = public_dir / "status.json"
    public_data = public_dir / "data.json"
    public_status.write_text(
        (dashboard / "status.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    public_data.write_text(
        (dashboard / "data.json").read_text(encoding="utf-8"),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--dashboard-dir",
            str(dashboard),
            "--website-url",
            website.as_uri(),
            "--guide-url",
            website.as_uri(),
            "--status-url",
            public_status.as_uri(),
            "--dashboard-url",
            page.as_uri(),
            "--summary-url",
            summary.as_uri(),
            "--expect-sha",
            "newer-main-sha",
        ],
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected sha 'newer-main-sha'" in result.stderr
