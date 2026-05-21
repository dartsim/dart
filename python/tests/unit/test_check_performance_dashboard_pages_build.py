import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_pages_build.py"


def _json_file(tmp_path: Path, name: str, data) -> Path:
    path = tmp_path / name
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def _build(**overrides):
    data = {
        "status": "built",
        "commit": "abc123",
        "error": {"message": None},
        "updated_at": "2026-05-21T00:00:00Z",
    }
    data.update(overrides)
    return data


def _branch(**overrides):
    data = {"name": "gh-pages", "commit": {"sha": "abc123"}}
    data.update(overrides)
    return data


def _command(tmp_path: Path, build, branch) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT),
        "--build-json",
        str(_json_file(tmp_path, "build.json", build)),
        "--branch-json",
        str(_json_file(tmp_path, "branch.json", branch)),
    ]


def test_pages_build_accepts_built_current_source_branch(tmp_path):
    result = subprocess.run(
        _command(tmp_path, _build(), _branch()),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "pages build verified" in result.stdout
    assert "gh-pages@abc123" in result.stdout


def test_pages_build_rejects_non_built_status(tmp_path):
    result = subprocess.run(
        _command(tmp_path, _build(status="errored"), _branch()),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected latest Pages build status" in result.stderr


def test_pages_build_rejects_reported_error(tmp_path):
    result = subprocess.run(
        _command(tmp_path, _build(error={"message": "build failed"}), _branch()),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "build failed" in result.stderr


def test_pages_build_rejects_stale_build_commit(tmp_path):
    result = subprocess.run(
        _command(tmp_path, _build(commit="old"), _branch()),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "not current for the source branch" in result.stderr


def test_pages_build_rejects_wrong_source_branch(tmp_path):
    result = subprocess.run(
        _command(tmp_path, _build(), _branch(name="main")),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "Expected Pages source branch" in result.stderr
