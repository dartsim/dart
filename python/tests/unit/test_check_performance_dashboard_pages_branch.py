import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_pages_branch.py"


def _json_file(tmp_path: Path, name: str, data) -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _command(tmp_path: Path, branch, *extra: str) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT),
        "--branch-json",
        str(_json_file(tmp_path, "branch.json", branch)),
        *extra,
    ]


def _dashboard_entries():
    return [
        {"name": "data.json", "type": "file"},
        {"name": "index.html", "type": "file"},
        {"name": "status.json", "type": "file"},
        {"name": "summary.md", "type": "file"},
    ]


def test_pages_branch_accepts_missing_dashboard_path(tmp_path):
    result = subprocess.run(
        _command(tmp_path, {"name": "gh-pages"}, "--contents-missing"),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "gh-pages exists" in result.stdout
    assert "ready for first publication" in result.stdout


def test_pages_branch_rejects_missing_dashboard_path_when_required(tmp_path):
    result = subprocess.run(
        _command(
            tmp_path,
            {"name": "gh-pages"},
            "--contents-missing",
            "--require-dashboard-path",
        ),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "path is absent" in result.stderr


def test_pages_branch_accepts_existing_dashboard_directory(tmp_path):
    contents = _json_file(tmp_path, "contents.json", _dashboard_entries())

    result = subprocess.run(
        _command(tmp_path, {"name": "gh-pages"}, "--contents-json", str(contents)),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "contains dashboard files" in result.stdout


def test_pages_branch_rejects_wrong_branch_fixture(tmp_path):
    result = subprocess.run(
        _command(tmp_path, {"name": "main"}, "--contents-missing"),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected Pages branch" in result.stderr


def test_pages_branch_rejects_file_at_dashboard_path(tmp_path):
    contents = _json_file(
        tmp_path,
        "contents.json",
        {"name": "performance", "type": "file"},
    )

    result = subprocess.run(
        _command(tmp_path, {"name": "gh-pages"}, "--contents-json", str(contents)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "dashboard directory" in result.stderr


def test_pages_branch_rejects_non_dashboard_directory(tmp_path):
    contents = _json_file(
        tmp_path,
        "contents.json",
        [{"name": "README.md", "type": "file"}],
    )

    result = subprocess.run(
        _command(tmp_path, {"name": "gh-pages"}, "--contents-json", str(contents)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "does not look like the performance dashboard" in result.stderr
