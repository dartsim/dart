import json
import importlib.util
import subprocess
import sys
import urllib.error
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_workflow_run.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("workflow_run", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _runs_json(tmp_path: Path, runs) -> Path:
    path = tmp_path / "runs.json"
    path.write_text(json.dumps({"workflow_runs": runs}), encoding="utf-8")
    return path


def _artifacts_json(tmp_path: Path, artifacts) -> Path:
    path = tmp_path / "artifacts.json"
    path.write_text(json.dumps({"artifacts": artifacts}), encoding="utf-8")
    return path


def _command(runs_json: Path, *extra: str) -> list[str]:
    return [
        sys.executable,
        str(SCRIPT),
        "--runs-json",
        str(runs_json),
        *extra,
    ]


def _successful_run(**overrides):
    run = {
        "id": 12345,
        "head_branch": "main",
        "status": "completed",
        "conclusion": "success",
        "run_attempt": 1,
        "html_url": "https://github.com/dartsim/dart/actions/runs/12345",
    }
    run.update(overrides)
    return run


def _site_artifact(**overrides):
    artifact = {
        "name": "performance-dashboard-site-12345-1",
        "expired": False,
        "archive_download_url": (
            "https://api.github.com/repos/dartsim/dart/actions/artifacts/99/zip"
        ),
    }
    artifact.update(overrides)
    return artifact


def test_workflow_run_accepts_latest_successful_main_run(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])
    artifacts_json = _artifacts_json(tmp_path, [_site_artifact()])

    result = subprocess.run(
        _command(runs_json, "--artifacts-json", str(artifacts_json)),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "performance dashboard workflow run succeeded" in result.stdout
    assert "12345" in result.stdout
    assert "performance-dashboard-site-12345-1" in result.stdout


def test_workflow_run_writes_latest_run_identity(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run(head_sha="abc123")])
    artifacts_json = _artifacts_json(tmp_path, [_site_artifact()])
    output_json = tmp_path / "latest-run.json"

    subprocess.run(
        _command(
            runs_json,
            "--artifacts-json",
            str(artifacts_json),
            "--run-output-json",
            str(output_json),
        ),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    output = json.loads(output_json.read_text(encoding="utf-8"))
    assert output["id"] == 12345
    assert output["run_attempt"] == 1
    assert output["expected_dashboard_run_id"] == "12345-1"
    assert output["head_sha"] == "abc123"
    assert output["head_branch"] == "main"
    assert output["dashboard_artifact"]["name"] == "performance-dashboard-site-12345-1"


def test_workflow_run_accepts_success_without_artifact_when_not_required(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])

    result = subprocess.run(
        _command(runs_json, "--no-require-dashboard-artifact"),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "performance dashboard workflow run succeeded" in result.stdout
    assert "artifact=" not in result.stdout


def test_workflow_run_rejects_missing_runs(tmp_path):
    runs_json = _runs_json(tmp_path, [])

    result = subprocess.run(
        _command(runs_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "No performance dashboard workflow runs found" in result.stderr


def test_workflow_run_rejects_non_completed_latest_run(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run(status="in_progress")])

    result = subprocess.run(
        _command(runs_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "expected 'completed'" in result.stderr


def test_workflow_run_rejects_failed_latest_run(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run(conclusion="failure")])

    result = subprocess.run(
        _command(runs_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "expected 'success'" in result.stderr


def test_workflow_run_rejects_wrong_branch_latest_run(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run(head_branch="feature")])

    result = subprocess.run(
        _command(runs_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "expected 'main'" in result.stderr


def test_workflow_run_rejects_missing_url(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run(html_url=None, url=None)])

    result = subprocess.run(
        _command(runs_json),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "has no URL" in result.stderr


def test_workflow_run_rejects_missing_dashboard_artifact(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])
    artifacts_json = _artifacts_json(
        tmp_path,
        [
            _site_artifact(name="performance-dashboard-raw-12345-1"),
        ],
    )

    result = subprocess.run(
        _command(runs_json, "--artifacts-json", str(artifacts_json)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "did not upload a hosted-site dashboard artifact" in result.stderr


def test_workflow_run_rejects_expired_dashboard_artifact(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])
    artifacts_json = _artifacts_json(
        tmp_path,
        [_site_artifact(expired=True)],
    )

    result = subprocess.run(
        _command(runs_json, "--artifacts-json", str(artifacts_json)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "no usable hosted-site dashboard artifact" in result.stderr


def test_workflow_run_rejects_dashboard_artifact_without_download_url(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])
    artifacts_json = _artifacts_json(
        tmp_path,
        [_site_artifact(archive_download_url=None)],
    )

    result = subprocess.run(
        _command(runs_json, "--artifacts-json", str(artifacts_json)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "no usable hosted-site dashboard artifact" in result.stderr


def test_workflow_run_rejects_artifacts_payload_without_list(tmp_path):
    runs_json = _runs_json(tmp_path, [_successful_run()])
    artifacts_json = tmp_path / "artifacts.json"
    artifacts_json.write_text(json.dumps({"artifacts": {}}), encoding="utf-8")

    result = subprocess.run(
        _command(runs_json, "--artifacts-json", str(artifacts_json)),
        text=True,
        stderr=subprocess.PIPE,
    )

    assert result.returncode != 0
    assert "artifacts API response did not contain a list" in result.stderr


def test_workflow_run_reports_unregistered_workflow_404(monkeypatch):
    module = _load_module()

    def fake_urlopen(request, timeout):
        raise urllib.error.HTTPError(
            request.full_url,
            404,
            "Not Found",
            hdrs=None,
            fp=None,
        )

    monkeypatch.setattr(module.urllib.request, "urlopen", fake_urlopen)

    with pytest.raises(RuntimeError) as exc_info:
        module._read_json_url(
            "https://api.github.com/repos/dartsim/dart/actions/workflows/"
            "performance_dashboard.yml/runs?branch=main&per_page=20",
            token=None,
            timeout=1.0,
        )

    message = str(exc_info.value)
    assert "workflow runs are not available yet" in message
    assert "has not landed on main" in message
