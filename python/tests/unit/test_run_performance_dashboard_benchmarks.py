import importlib.util
import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_performance_dashboard_benchmarks.py"


def _load_runner_module():
    spec = importlib.util.spec_from_file_location(
        "run_performance_dashboard_benchmarks",
        SCRIPT,
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_dashboard_surface_runner_dry_run_lists_bounded_specs(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--output-dir",
            str(tmp_path),
            "--dry-run",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    lines = result.stdout.strip().splitlines()
    assert len(lines) == 5
    assert all("scripts/run_cpp_benchmark.py" in line for line in lines)
    assert all("--benchmark_out_format=json" in line for line in lines)
    assert all("--benchmark_min_time=1ms" in line for line in lines)
    assert all("--benchmark_repetitions=3" in line for line in lines)
    assert "dashboard_empty.json" in result.stdout
    assert "BM_Empty$" in result.stdout
    assert "dashboard_kinematics_dynamics.json" in result.stdout
    assert "BM_(Kinematics|Dynamics)/(1|10|100)$" in result.stdout
    assert "dashboard_inverse_dynamics.json" in result.stdout
    assert "BM_ContactInverseDynamics/(2|4|8)$" in result.stdout
    assert "dashboard_boxes.json" in result.stdout
    assert "BM_RunBoxes/(2|4|8)$" in result.stdout
    assert "dashboard_contact_container.json" in result.stdout
    assert "BM_ContactContainerActive/.*" in result.stdout


def test_dashboard_surface_runner_can_select_specific_surface(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--surface",
            "inverse-dynamics",
            "--output-dir",
            str(tmp_path),
            "--benchmark-min-time",
            "2ms",
            "--benchmark-repetitions",
            "2",
            "--dry-run",
        ],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )

    lines = result.stdout.strip().splitlines()
    assert len(lines) == 1
    assert "dashboard_inverse_dynamics.json" in result.stdout
    assert "--benchmark_min_time=2ms" in result.stdout
    assert "--benchmark_repetitions=2" in result.stdout


def test_contact_container_surface_is_required():
    runner = _load_runner_module()

    spec = next(
        spec for spec in runner.BENCHMARK_SPECS if spec.surface == "contact-container"
    )

    assert spec.target == "BM_INTEGRATION_contact_container"
    assert not spec.optional


def test_dashboard_surface_runner_fails_when_output_has_no_rows(tmp_path, monkeypatch):
    runner = _load_runner_module()
    monkeypatch.setattr(
        runner,
        "BENCHMARK_SPECS",
        [
            runner.BenchmarkSpec(
                surface="fake",
                target="fake_target",
                benchmark_filter="BM_Fake$",
                output_name="fake.json",
            )
        ],
    )

    def fake_run(command, check):
        output = next(
            arg.split("=", 1)[1]
            for arg in command
            if arg.startswith("--benchmark_out=")
        )
        Path(output).write_text(json.dumps({"benchmarks": []}), encoding="utf-8")

    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    with pytest.raises(SystemExit, match="no benchmark rows"):
        runner.main(["--surface", "fake", "--output-dir", str(tmp_path)])


def test_continue_on_error_preserves_required_surface_failure(
    tmp_path,
    monkeypatch,
):
    runner = _load_runner_module()
    monkeypatch.setattr(
        runner,
        "BENCHMARK_SPECS",
        [
            runner.BenchmarkSpec(
                surface="broken",
                target="broken_target",
                benchmark_filter="BM_Broken$",
                output_name="broken.json",
            ),
            runner.BenchmarkSpec(
                surface="passing",
                target="passing_target",
                benchmark_filter="BM_Passing$",
                output_name="passing.json",
            ),
        ],
    )

    def fake_run(command, check):
        output = next(
            arg.split("=", 1)[1] for arg in command if arg.startswith("--benchmark_out=")
        )
        if output.endswith("broken.json"):
            raise subprocess.CalledProcessError(2, command)
        Path(output).write_text(
            json.dumps({"benchmarks": [{"name": "BM_Passing", "real_time": 1.0}]}),
            encoding="utf-8",
        )

    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    result = runner.main(
        [
            "--surface",
            "broken",
            "--surface",
            "passing",
            "--output-dir",
            str(tmp_path),
            "--continue-on-error",
        ]
    )

    assert result == 1
    assert (tmp_path / "passing.json").is_file()


def test_optional_surface_failure_is_tolerated(tmp_path, monkeypatch, capsys):
    runner = _load_runner_module()
    monkeypatch.setattr(
        runner,
        "BENCHMARK_SPECS",
        [
            runner.BenchmarkSpec(
                surface="optional",
                target="optional_target",
                benchmark_filter="BM_Optional$",
                output_name="optional.json",
                optional=True,
            )
        ],
    )

    def fake_run(command, check):
        raise subprocess.CalledProcessError(2, command)

    monkeypatch.setattr(runner, "_target_declared", lambda target, build_type: True)
    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    result = runner.main(["--surface", "optional", "--output-dir", str(tmp_path)])

    assert result == 0
    assert "Skipping optional optional" in capsys.readouterr().err


def test_dashboard_surface_runner_accepts_output_with_rows(tmp_path, monkeypatch):
    runner = _load_runner_module()
    monkeypatch.setattr(
        runner,
        "BENCHMARK_SPECS",
        [
            runner.BenchmarkSpec(
                surface="fake",
                target="fake_target",
                benchmark_filter="BM_Fake$",
                output_name="fake.json",
            )
        ],
    )

    def fake_run(command, check):
        output = next(
            arg.split("=", 1)[1]
            for arg in command
            if arg.startswith("--benchmark_out=")
        )
        Path(output).write_text(
            json.dumps({"benchmarks": [{"name": "BM_Fake", "real_time": 1.0}]}),
            encoding="utf-8",
        )

    monkeypatch.setattr(runner.subprocess, "run", fake_run)

    assert runner.main(["--surface", "fake", "--output-dir", str(tmp_path)]) == 0
