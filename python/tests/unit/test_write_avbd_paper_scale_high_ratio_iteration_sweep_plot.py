from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

SCRIPT = ROOT / "scripts" / "write_avbd_paper_scale_high_ratio_iteration_sweep_plot.py"


def _load_plot_module():
    spec = importlib.util.spec_from_file_location(
        "write_avbd_paper_scale_high_ratio_iteration_sweep_plot",
        SCRIPT,
    )
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _write_packet(
    tmp_path: Path,
    *,
    finite_replay: float = 1.0,
    avbd_solver_evidence: bool = False,
) -> Path:
    packet = {
        "benchmark": {
            "plot_data": [
                {
                    "cpu_time_per_step_ns": 25000.0,
                    "finite_replay": finite_replay,
                    "max_abs_position_rad": 0.025,
                    "max_iterations": 25,
                    "real_time_per_step_ns": 27500.0,
                    "time_unit": "ns",
                },
                {
                    "cpu_time_per_step_ns": 50000.0,
                    "finite_replay": finite_replay,
                    "max_abs_position_rad": 0.05,
                    "max_iterations": 50,
                    "real_time_per_step_ns": 55000.0,
                    "time_unit": "ns",
                },
            ]
        },
        "evidence_boundary": {
            "avbd_performance_claim_supported": False,
            "avbd_solver_evidence": avbd_solver_evidence,
            "current_build_bound": False,
            "measurement_runtime_identity_recorded": False,
            "plan104_avbd_row_closure_supported": False,
        },
        "packet": "avbd_paper_scale_high_ratio_iteration_sweep",
        "resolved_solver_identity": {
            "rigid_contact_solver": "none",
            "rigid_point_joint_solver": "none",
        },
        "schema_version": 3,
    }
    path = tmp_path / "packet.json"
    path.write_text(json.dumps(packet), encoding="utf-8")
    return path


def test_avbd_paper_scale_high_ratio_iteration_sweep_plot_renders_svg(
    tmp_path: Path,
) -> None:
    module = _load_plot_module()
    packet = _write_packet(tmp_path)
    output = tmp_path / "plot.svg"

    assert module.main(["--packet", str(packet), "--output", str(output)]) == 0

    svg = output.read_text()
    assert '<svg xmlns="http://www.w3.org/2000/svg"' in svg
    assert "Variational Multibody High-Ratio Iteration Sweep" in svg
    assert "not AVBD solver evidence" in svg
    assert "CPU time per step" in svg
    assert "Replay stability" in svg
    assert ">25<" in svg
    assert ">50<" in svg


def test_avbd_paper_scale_high_ratio_iteration_sweep_plot_rejects_nonfinite_replay(
    tmp_path: Path,
) -> None:
    module = _load_plot_module()
    packet = _write_packet(tmp_path, finite_replay=0.0)

    with pytest.raises(SystemExit, match="plot_data row is not finite"):
        module.main(["--packet", str(packet)])


def test_avbd_paper_scale_high_ratio_iteration_sweep_plot_rejects_avbd_claim(
    tmp_path: Path,
) -> None:
    module = _load_plot_module()
    packet = _write_packet(tmp_path, avbd_solver_evidence=True)

    with pytest.raises(
        SystemExit,
        match=(
            "must explicitly reject AVBD solver, performance, current-build, "
            "runtime-identity, and PLAN-104"
        ),
    ):
        module.main(["--packet", str(packet)])
