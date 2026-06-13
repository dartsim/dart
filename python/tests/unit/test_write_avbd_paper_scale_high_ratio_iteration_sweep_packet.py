from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = (
    ROOT
    / "scripts"
    / "write_avbd_paper_scale_high_ratio_iteration_sweep_packet.py"
)


def _load_packet_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_scale_high_ratio_iteration_sweep_packet",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_row(
    budget: int,
    *,
    run_type: str = "iteration",
    aggregate_name: str | None = None,
    links: float = 50.0,
    mass_ratio: float = 50000.0,
    tolerance: float = 1e-9,
    replay_steps: float = 32.0,
    replay_seconds: float = 0.16,
    finite_replay: float = 1.0,
    max_abs_position: float | None = None,
) -> dict[str, object]:
    suffix = f"/{budget}"
    name = f"BM_AvbdPaperScaleHighRatioChainIterationSweep{suffix}"
    row: dict[str, object] = {
        "cpu_time": float(budget * 1000),
        "iterations": 10,
        "links": links,
        "mass_ratio": mass_ratio,
        "max_iterations": float(budget),
        "name": name,
        "real_time": float(budget * 1100),
        "replay_seconds": replay_seconds,
        "replay_steps": replay_steps,
        "run_name": name,
        "run_type": run_type,
        "time_unit": "ns",
        "tolerance": tolerance,
        "finite_replay": finite_replay,
        "max_abs_position": (
            float(budget) * 0.001
            if max_abs_position is None
            else max_abs_position
        ),
    }
    if aggregate_name is not None:
        row["aggregate_name"] = aggregate_name
        row["aggregate_unit"] = "time"
        row["name"] = f"{name}_{aggregate_name}"
    return row


def _write_benchmark_json(
    tmp_path: Path,
    *,
    budgets: tuple[int, ...] = (25, 50, 100, 200),
    links: float = 50.0,
    wrong_benchmark: bool = False,
) -> Path:
    rows = []
    for budget in budgets:
        row = _benchmark_row(budget, links=links)
        median = _benchmark_row(
            budget,
            run_type="aggregate",
            aggregate_name="median",
            links=links,
        )
        if wrong_benchmark:
            row["name"] = "BM_AvbdPaperScaleHighRatioChainStep"
            row["run_name"] = "BM_AvbdPaperScaleHighRatioChainStep"
            median["name"] = "BM_AvbdPaperScaleHighRatioChainStep_median"
            median["run_name"] = "BM_AvbdPaperScaleHighRatioChainStep"
        rows.extend([row, median])
    benchmark = {
        "benchmarks": rows,
        "context": {
            "executable": "build/default/cpp/Release/bin/bm_avbd_rigid_fixed_joint",
            "json_schema_version": 1,
            "library_build_type": "release",
            "library_version": "v1.9.5",
            "mhz_per_cpu": 3200,
            "num_cpus": 8,
        },
    }
    path = tmp_path / "benchmark.json"
    path.write_text(json.dumps(benchmark), encoding="utf-8")
    return path


def test_avbd_paper_scale_high_ratio_iteration_sweep_packet_records_plot_data(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path)
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--benchmark-json",
                str(benchmark_json),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["schema_version"] == 2
    assert packet["packet"] == "avbd_paper_scale_high_ratio_iteration_sweep"
    assert packet["resolved_solver_identity"] == {
        "avbd_rigid_contact_config_emplaced": False,
        "recorded_from": "paper-scale high-ratio iteration benchmark row family",
        "rigid_contact_solver": "none",
        "rigid_point_joint_solver": "avbd",
    }
    assert packet["scene"] == "avbd_paper_scale_high_ratio_chain"
    assert packet["target"] == {
        "complete_paper_reproduction": False,
        "paper_gap": "iteration-count sweep for the 50-link 50,000:1 chain",
        "scope": (
            "benchmark-only max-iteration sweep over the paper-scale "
            "50-link articulated chain"
        ),
    }
    assert packet["scene_invariants"] == {
        "light_link_mass": 1.0,
        "links": 50,
        "mass_ratio": 50000.0,
        "max_iterations": [25, 50, 100, 200],
        "replay_seconds": 0.16,
        "replay_steps": 32,
        "time_step": 0.005,
        "tip_link_mass": 50000.0,
        "tolerance": 1e-9,
    }
    assert packet["benchmark"]["benchmark"] == (
        "BM_AvbdPaperScaleHighRatioChainIterationSweep"
    )
    assert packet["benchmark"]["invariants"] == {
        "finite_replay": True,
        "links": 50,
        "mass_ratio": 50000.0,
        "max_iterations": [25, 50, 100, 200],
        "replay_seconds": 0.16,
        "replay_steps": 32,
        "time_step": 0.005,
        "tolerance": 1e-9,
    }
    assert packet["benchmark"]["plot_data"] == [
        {
            "cpu_time_per_step_ns": 25000.0,
            "finite_replay": 1.0,
            "max_abs_position_rad": 0.025,
            "max_iterations": 25,
            "real_time_per_step_ns": 27500.0,
            "time_unit": "ns",
        },
        {
            "cpu_time_per_step_ns": 50000.0,
            "finite_replay": 1.0,
            "max_abs_position_rad": 0.05,
            "max_iterations": 50,
            "real_time_per_step_ns": 55000.0,
            "time_unit": "ns",
        },
        {
            "cpu_time_per_step_ns": 100000.0,
            "finite_replay": 1.0,
            "max_abs_position_rad": 0.1,
            "max_iterations": 100,
            "real_time_per_step_ns": 110000.0,
            "time_unit": "ns",
        },
        {
            "cpu_time_per_step_ns": 200000.0,
            "finite_replay": 1.0,
            "max_abs_position_rad": 0.2,
            "max_iterations": 200,
            "real_time_per_step_ns": 220000.0,
            "time_unit": "ns",
        },
    ]
    assert (
        "rendered convergence/stability plot for the iteration-count sweep"
        in packet["remaining_gates"]
    )


def test_avbd_paper_scale_high_ratio_iteration_sweep_packet_records_plot_artifact(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path)
    plot_svg = tmp_path / "plot.svg"
    plot_svg.write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" width="720" height="420"></svg>',
        encoding="utf-8",
    )
    output = tmp_path / "packet.json"

    assert (
        module.main(
            [
                "--benchmark-json",
                str(benchmark_json),
                "--plot-svg",
                str(plot_svg),
                "--output",
                str(output),
            ]
        )
        == 0
    )

    packet = json.loads(output.read_text())
    assert packet["rendered_plot"]["file"] == str(plot_svg)
    assert packet["rendered_plot"]["width"] == 720.0
    assert packet["rendered_plot"]["height"] == 420.0
    assert "sha256" in packet["rendered_plot"]
    assert (
        "rendered convergence/stability plot for the iteration-count sweep"
        not in packet["remaining_gates"]
    )


def test_avbd_paper_scale_high_ratio_iteration_sweep_packet_rejects_wrong_benchmark(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, wrong_benchmark=True)

    with pytest.raises(
        SystemExit,
        match="benchmark JSON missing BM_AvbdPaperScaleHighRatioChainIterationSweep",
    ):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_paper_scale_high_ratio_iteration_sweep_packet_rejects_missing_budget(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, budgets=(25, 50, 100))

    with pytest.raises(SystemExit, match="missing max_iterations budgets: 200"):
        module.main(["--benchmark-json", str(benchmark_json)])


def test_avbd_paper_scale_high_ratio_iteration_sweep_packet_rejects_wrong_counters(
    tmp_path: Path,
) -> None:
    module = _load_packet_module()
    benchmark_json = _write_benchmark_json(tmp_path, links=5.0)

    with pytest.raises(SystemExit, match="expected links=50"):
        module.main(["--benchmark-json", str(benchmark_json)])
