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
    # One bounded command per curated DART 7 World surface.
    assert len(lines) == 6
    assert all("scripts/run_cpp_benchmark.py" in line for line in lines)
    assert all("--benchmark_out_format=json" in line for line in lines)
    assert all("--benchmark_min_time=1ms" in line for line in lines)
    assert all("--benchmark_repetitions=3" in line for line in lines)
    # Core DART 7 World step & scaling surface (bm_compute_graph).
    assert "dashboard_world.json" in result.stdout
    assert "BM_WorldUpdateKinematics" in result.stdout
    assert "BM_WorldStep(Sequential|Parallel)/.*" in result.stdout
    assert "BM_RigidBodyStep(Sequential|Parallel)/.*" in result.stdout
    assert "BM_ContactShaped(Sequential|Parallel)/.*" in result.stdout
    assert "BM_ContactIslandShaped(Sequential|Parallel)/.*" in result.stdout
    assert "BM_Phase5RigidBodyBatchCpuBaseline/1024/128/10" in result.stdout
    # New DART 7 solver-family end-to-end World-step surfaces.
    assert "dashboard_rigid_world.json" in result.stdout
    assert "BM_RigidWorldStep_(SequentialImpulse|Ipc)/.*" in result.stdout
    assert "dashboard_lcp_solvers.json" in result.stdout
    assert "BM_LcpCompare/Standard/(.*/12|Direct/3)$" in result.stdout
    assert "BM_LcpCompare/Boxed/.*/12$" in result.stdout
    assert "BM_LcpCompare/FrictionIndex/.*/4$" in result.stdout
    assert "BM_LcpWorldContact/FrictionIndex/.*/4$" in result.stdout
    assert "BM_LcpWorldBoxContact/FrictionIndex/.*/4$" in result.stdout
    assert "BM_LcpWorldBilliardsStep_BoxedLcp/(1|4|8)/1$" in result.stdout
    assert "BM_LcpWorldStackStep_BoxedLcp/(3|4)/200$" in result.stdout
    assert "BM_LcpWorldCardPileStep_BoxedLcp/(4|7|12)/200$" in result.stdout
    assert "dashboard_vbd_world.json" in result.stdout
    assert "BM_VbdWorldStep(Default|Vbd)/.*" in result.stdout
    assert "dashboard_deformable_world.json" in result.stdout
    assert "BM_DeformableFemBarStep/.*" in result.stdout
    assert "dashboard_avbd_world.json" in result.stdout
    assert "BM_AvbdEmptyWorldStep$" in result.stdout
    assert "BM_AvbdDemo2dMotorStep$" in result.stdout
    assert "BM_AvbdDemo2dHangingRopeStep$" in result.stdout
    assert "BM_AvbdDemo2dFractureStep$" in result.stdout
    assert "BM_AvbdDemo2dGroundStep$" in result.stdout
    assert "BM_AvbdDemo2dDynamicFrictionStep$" in result.stdout
    assert "BM_AvbdDemo2dFrictionCoefficientSweep/.*" in result.stdout
    assert "BM_AvbdDemo2dStaticFrictionStep$" in result.stdout
    assert "BM_AvbdDemo2dPyramidStep$" in result.stdout
    assert "BM_AvbdDemo2dCardsStep$" in result.stdout
    assert "BM_AvbdDemo2dStackStep$" in result.stdout
    assert "BM_AvbdDemo2dStackRatioStep$" in result.stdout
    assert "BM_AvbdDemo2dRodStep$" in result.stdout
    assert "BM_AvbdDemo2dSoftBodyStep$" in result.stdout
    assert "BM_AvbdDemo2dJointGridStep$" in result.stdout
    assert "BM_AvbdDemo2dRopeStep$" in result.stdout
    assert "BM_AvbdDemo2dHeavyRopeStep$" in result.stdout
    assert "BM_AvbdDemo2dSpringStep$" in result.stdout
    assert "BM_AvbdDemo2dSpringRatioStep$" in result.stdout
    assert "BM_AvbdDemo2dNetStep$" in result.stdout
    assert "BM_AvbdDemo3dGroundStep$" in result.stdout
    assert "BM_AvbdDemo3dDynamicFrictionStep$" in result.stdout
    assert "BM_AvbdDemo3dStaticFrictionStep$" in result.stdout
    assert "BM_AvbdDemo3dPyramidStep$" in result.stdout
    assert "BM_AvbdDemo3dRopeStep$" in result.stdout
    assert "BM_AvbdDemo3dHeavyRopeStep$" in result.stdout
    assert "BM_AvbdDemo3dSpringStep$" in result.stdout
    assert "BM_AvbdDemo3dSpringRatioStep$" in result.stdout
    assert "BM_AvbdDemo3dStackStep$" in result.stdout
    assert "BM_AvbdDemo3dStackRatioStep$" in result.stdout
    assert "BM_AvbdDemo3dSoftBodyStep$" in result.stdout
    assert "BM_AvbdDemo3dBridgeStep$" in result.stdout
    assert "BM_AvbdDemo3dBreakableStep$" in result.stdout
    assert "BM_AvbdArticulatedHighRatioChainStep$" in result.stdout
    assert "BM_AvbdPaperScaleHighRatioChainStep$" in result.stdout
    assert "BM_AvbdPaperScaleHighRatioChainIterationSweep/.*" in result.stdout
    assert (
        "BM_Avbd(Rigid(FixedJoint|RevoluteMotor|PrismaticMotor|BreakableJoint"
        "|SphericalBreakableJoint)"
        "|Articulated((Revolute|World(Revolute|Prismatic)Breakable"
        "|PrismaticBreakable|Prismatic|Breakable)Motor"
        "|BreakableJoint|WorldSphericalBreakableJoint"
        "|SphericalPairBreakableJoint))Step/.*"
        in result.stdout
    )
    # Still excludes unrelated solver, SIMD, and robot-loader surfaces, and the
    # CUDA/GPU and DART_BUILD_DIFF rows the hosted runner cannot produce.
    assert "BM_Robot_(KR5|Atlas)_WorldStep" not in result.stdout
    assert "BM_LCP_COMPARE_SMOKE" not in result.stdout
    assert "BM_Add_DART_f32(_Baseline)?/1024(/.*)?$" not in result.stdout
    assert "Cuda" not in result.stdout
    assert "bm_diff_gradient" not in result.stdout


def test_dashboard_surface_runner_can_select_specific_surfaces(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--surface",
            "world",
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
    assert "dashboard_world.json" in result.stdout
    assert "--benchmark_min_time=2ms" in result.stdout
    assert "--benchmark_repetitions=2" in result.stdout


def test_dashboard_surface_runner_can_select_lcp_solver_surface(tmp_path):
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--surface",
            "lcp-solvers",
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
    assert "--target lcp_compare" in result.stdout
    assert "dashboard_lcp_solvers.json" in result.stdout
    assert "BM_LcpCompare/Standard/(.*/12|Direct/3)$" in result.stdout
    assert "BM_LcpWorldBilliardsStep_BoxedLcp/(1|4|8)/1$" in result.stdout
    assert "BM_LcpWorldStackStep_BoxedLcp/(3|4)/200$" in result.stdout
    assert "BM_LcpWorldCardPileStep_BoxedLcp/(4|7|12)/200$" in result.stdout


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
