import json
import importlib.util
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "check_performance_dashboard_launch_preflight.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("launch_preflight", SCRIPT)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _args(tmp_path, *, require_registered=False, require_published=False):
    return _load_module().argparse.Namespace(
        repo="dartsim/dart",
        work_dir=tmp_path / "preflight",
        require_registered=require_registered,
        require_published=require_published,
    )


def test_launch_preflight_runs_premerge_checks(monkeypatch, tmp_path):
    module = _load_module()
    commands = []

    def fake_run(command, check):
        assert check is True
        commands.append(command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    module.run_preflight(_args(tmp_path))

    assert [Path(command[1]).name for command in commands] == [
        "check_performance_dashboard_pages_config.py",
        "check_performance_dashboard_pages_branch.py",
        "check_performance_dashboard_pages_build.py",
        "check_performance_dashboard_website_sources.py",
        "check_performance_dashboard_pr_body.py",
        "check_performance_dashboard_service_sources.py",
        "check_performance_dashboard_workflow_registration.py",
        "download_performance_dashboard_artifacts.py",
        "check_performance_dashboard_workflow_dry_run.py",
        "check_performance_dashboard_publication.py",
    ]
    assert "--allow-missing" in commands[6]
    assert "--skip-if-unavailable" in commands[7]
    assert "--branch" in commands[7]
    assert commands[7][commands[7].index("--branch") + 1] == "main"
    assert "--seed-root" in commands[8]
    assert "--allow-unpublished" in commands[9]


def test_launch_preflight_can_require_registered_and_published(monkeypatch, tmp_path):
    module = _load_module()
    commands = []

    def fake_run(command, check):
        assert check is True
        commands.append(command)
        if Path(command[1]).name == "check_performance_dashboard_workflow_run.py":
            output_path = Path(command[command.index("--run-output-json") + 1])
            output_path.write_text(
                json.dumps(
                    {
                        "expected_dashboard_run_id": "12345-1",
                        "head_sha": "abc123",
                    }
                ),
                encoding="utf-8",
            )
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    module.run_preflight(
        _args(tmp_path, require_registered=True, require_published=True)
    )

    registration_command = commands[6]
    workflow_run_command = commands[7]
    publication_command = commands[10]
    assert "--require-dashboard-path" in commands[1]
    assert "--allow-missing" not in registration_command
    assert Path(workflow_run_command[1]).name == (
        "check_performance_dashboard_workflow_run.py"
    )
    assert "--require-dashboard-artifact" in workflow_run_command
    assert "--run-output-json" in workflow_run_command
    assert "--allow-unpublished" not in publication_command
    assert "--expect-run-id" in publication_command
    assert publication_command[publication_command.index("--expect-run-id") + 1] == (
        "12345-1"
    )
    assert "--expect-sha" in publication_command
    assert publication_command[publication_command.index("--expect-sha") + 1] == (
        "abc123"
    )


def test_launch_live_reports_registration_and_publication_failures(
    monkeypatch, tmp_path
):
    module = _load_module()
    commands = []

    def fake_run(command, check):
        assert check is True
        commands.append(command)
        script_name = Path(command[1]).name
        if script_name in {
            "check_performance_dashboard_pages_branch.py",
            "check_performance_dashboard_workflow_registration.py",
            "check_performance_dashboard_workflow_run.py",
            "check_performance_dashboard_publication.py",
        }:
            raise subprocess.CalledProcessError(1, command)
        return subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    with pytest.raises(RuntimeError) as exc_info:
        module.run_preflight(
            _args(tmp_path, require_registered=True, require_published=True)
        )

    message = str(exc_info.value)
    assert "Performance dashboard launch preflight failed:" in message
    assert "Verify GitHub Pages source branch path failed" in message
    assert "Verify workflow registration preflight failed" in message
    assert "Verify latest dashboard workflow run failed" in message
    assert "Verify publication endpoint preflight failed" in message
    publication_command = commands[-1]
    assert "--expect-run-id" not in publication_command
    assert "--expect-sha" not in publication_command
    assert [Path(command[1]).name for command in commands] == [
        "check_performance_dashboard_pages_config.py",
        "check_performance_dashboard_pages_branch.py",
        "check_performance_dashboard_pages_build.py",
        "check_performance_dashboard_website_sources.py",
        "check_performance_dashboard_pr_body.py",
        "check_performance_dashboard_service_sources.py",
        "check_performance_dashboard_workflow_registration.py",
        "check_performance_dashboard_workflow_run.py",
        "download_performance_dashboard_artifacts.py",
        "check_performance_dashboard_workflow_dry_run.py",
        "check_performance_dashboard_publication.py",
    ]
