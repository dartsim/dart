import importlib.util
import json
import subprocess
import sys
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "download_performance_dashboard_artifacts.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("download_artifacts", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_select_artifacts_filters_and_orders_matching_artifacts():
    module = _load_module()
    artifacts = [
        {
            "name": "ubuntu-latest-py312",
            "expired": False,
            "created_at": "2026-05-21T00:00:00Z",
            "archive_download_url": "https://example.invalid/wheel.zip",
        },
        {
            "name": "collision-benchmark-guard-old",
            "expired": False,
            "created_at": "2026-05-20T08:00:00Z",
            "archive_download_url": "https://example.invalid/old.zip",
        },
        {
            "name": "performance-dashboard-raw-new",
            "expired": False,
            "created_at": "2026-05-21T08:00:00Z",
            "archive_download_url": "https://example.invalid/new.zip",
        },
        {
            "name": "collision-benchmark-guard-expired",
            "expired": True,
            "created_at": "2026-05-22T08:00:00Z",
            "archive_download_url": "https://example.invalid/expired.zip",
        },
    ]

    selected = module._select_artifacts(artifacts, [], 2)

    assert [artifact["name"] for artifact in selected] == [
        "performance-dashboard-raw-new",
        "collision-benchmark-guard-old",
    ]


def test_select_artifacts_filters_by_branch_when_requested():
    module = _load_module()
    artifacts = [
        {
            "name": "performance-dashboard-raw-main",
            "expired": False,
            "created_at": "2026-05-21T08:00:00Z",
            "archive_download_url": "https://example.invalid/main.zip",
            "workflow_run": {"head_branch": "main"},
        },
        {
            "name": "performance-dashboard-raw-pr",
            "expired": False,
            "created_at": "2026-05-21T09:00:00Z",
            "archive_download_url": "https://example.invalid/pr.zip",
            "workflow_run": {"head_branch": "feature/perf"},
        },
        {
            "name": "performance-dashboard-raw-unknown",
            "expired": False,
            "created_at": "2026-05-21T10:00:00Z",
            "archive_download_url": "https://example.invalid/unknown.zip",
        },
    ]

    selected = module._select_artifacts(artifacts, [], 3, branch="main")

    assert [artifact["name"] for artifact in selected] == [
        "performance-dashboard-raw-main"
    ]


def test_default_token_uses_github_token(monkeypatch):
    module = _load_module()
    monkeypatch.setenv("GITHUB_TOKEN", "from-env")
    monkeypatch.setattr(module.shutil, "which", lambda name: None)

    assert module._default_token() == "from-env"


def test_default_token_falls_back_to_gh_auth(monkeypatch):
    module = _load_module()
    monkeypatch.delenv("GITHUB_TOKEN", raising=False)
    monkeypatch.setattr(module.shutil, "which", lambda name: "/usr/bin/gh")

    def fake_run(command, **kwargs):
        assert command == ["gh", "auth", "token"]
        assert kwargs["check"] is True
        return subprocess.CompletedProcess(command, 0, stdout="from-gh\n")

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    assert module._default_token() == "from-gh"


def test_redirect_handler_drops_authorization_on_cross_host_redirect():
    module = _load_module()
    handler = module._AuthDroppingRedirectHandler()
    request = module.urllib.request.Request(
        "https://api.github.com/repos/dartsim/dart/actions/artifacts/1/zip",
        headers={"Authorization": "Bearer token"},
    )

    redirected = handler.redirect_request(
        request,
        None,
        302,
        "Found",
        {},
        "https://artifactcache.actions.githubusercontent.com/archive.zip",
    )

    assert redirected is not None
    assert not redirected.has_header("Authorization")


def test_download_performance_dashboard_artifacts_extracts_json(tmp_path):
    archive = tmp_path / "artifact.zip"
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("nested/benchmark.json", "{}")
        zf.writestr("notes.txt", "ignored")
    artifacts = tmp_path / "artifacts.json"
    artifacts.write_text(
        json.dumps(
            {
                "artifacts": [
                    {
                        "name": "collision-benchmark-guard-1",
                        "expired": False,
                        "created_at": "2026-05-20T08:00:00Z",
                        "archive_download_url": archive.as_uri(),
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    output_dir = tmp_path / "seed"

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--artifacts-json",
            str(artifacts),
            "--output-dir",
            str(output_dir),
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "downloaded benchmark artifact" in result.stdout
    assert (output_dir / "collision-benchmark-guard-1" / "benchmark.json").read_text(
        encoding="utf-8"
    ) == "{}"


def test_download_performance_dashboard_artifacts_skips_when_unavailable(tmp_path):
    artifacts = tmp_path / "artifacts.json"
    artifacts.write_text(json.dumps({"artifacts": []}), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--artifacts-json",
            str(artifacts),
            "--skip-if-unavailable",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "Skipping benchmark artifact history seed" in result.stdout


def test_download_performance_dashboard_artifacts_skips_download_failure(tmp_path):
    artifacts = tmp_path / "artifacts.json"
    artifacts.write_text(
        json.dumps(
            {
                "artifacts": [
                    {
                        "name": "collision-benchmark-guard-1",
                        "expired": False,
                        "created_at": "2026-05-20T08:00:00Z",
                        "archive_download_url": "file:///does/not/exist.zip",
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--artifacts-json",
            str(artifacts),
            "--skip-if-unavailable",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    assert "Skipping benchmark artifact collision-benchmark-guard-1" in result.stdout
    assert "Skipping benchmark artifact history seed" in result.stdout
