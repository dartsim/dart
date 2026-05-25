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
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
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
    assert len(lines) == 1
    assert all("scripts/run_cpp_benchmark.py" in line for line in lines)
    assert all("--benchmark_out_format=json" in line for line in lines)
    assert all("--benchmark_min_time=1ms" in line for line in lines)
    assert all("--benchmark_repetitions=3" in line for line in lines)
    assert "dashboard_experimental_world.json" in result.stdout
    assert "BM_WorldUpdateKinematics" in result.stdout
    assert "BM_WorldStep(Sequential|Parallel)/.*" in result.stdout
    assert "BM_RigidBodyStep(Sequential|Parallel)/.*" in result.stdout
    assert "BM_ContactShaped(Sequential|Parallel)/.*" in result.stdout
    assert "BM_ContactIslandShaped(Sequential|Parallel)/.*" in result.stdout
    assert "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10" in result.stdout
    assert "BM_Robot_(KR5|Atlas)_WorldStep" not in result.stdout
    assert "BM_LCP_COMPARE_SMOKE" not in result.stdout
    assert "BM_Add_DART_f32(_Baseline)?/1024(/.*)?$" not in result.stdout


def test_dashboard_surface_runner_can_select_specific_surfaces(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--surface",
            "experimental-world",
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
    assert "dashboard_experimental_world.json" in result.stdout
    assert "--benchmark_min_time=2ms" in result.stdout
    assert "--benchmark_repetitions=2" in result.stdout


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
