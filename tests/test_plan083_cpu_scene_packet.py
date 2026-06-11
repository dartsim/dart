import importlib.util
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "write_plan083_cpu_scene_packet.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "write_plan083_cpu_scene_packet",
        SCRIPT,
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _benchmark_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_hanging_bridge_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 2.0,
        "cpu_time": 2.0,
        "time_unit": "ms",
        "body_count": 7,
        "dynamic_body_count": 5,
        "fixed_joint_count": 4,
        "active_articulation_constraints": 12,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "traveler_height_m": 0.81,
        "max_board_sag_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _nunchaku_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_nunchaku_single_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 3.0,
        "cpu_time": 3.0,
        "time_unit": "ms",
        "body_count": 2,
        "dynamic_body_count": 1,
        "revolute_joint_count": 1,
        "active_articulation_constraints": 2,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "swinging_tip_radius_m": 0.36,
        "free_axis_angular_velocity_rad_s": 1.5,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _windmill_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_windmill_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_windmill_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 4.0,
        "cpu_time": 4.0,
        "time_unit": "ms",
        "body_count": 3,
        "dynamic_body_count": 2,
        "revolute_joint_count": 1,
        "active_constraints": 4,
        "active_friction_constraints": 2,
        "active_articulation_constraints": 2,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "blade_tip_radius_m": 0.36,
        "striker_height_m": 0.95,
        "striker_blade_clearance_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def test_plan083_cpu_scene_packet_accepts_reduced_hanging_bridge() -> None:
    module = _load_module()

    packet = module.make_packet(
        _benchmark_packet(),
        max_equality_residual=1e-8,
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-02"
    assert row["scene_id"] == "plan083_hanging_bridge"
    assert row["paper_scale"] is False
    assert row["body_count"] == 7
    assert row["dynamic_body_count"] == 5
    assert row["fixed_joint_count"] == 4
    assert row["active_articulation_constraints"] == 12
    assert row["wall_time_ns"] == 2.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_nunchaku() -> None:
    module = _load_module()

    packet = module.make_packet(
        _nunchaku_packet(),
        max_equality_residual=1e-8,
        scene="nunchaku_single",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-13"
    assert row["scene_id"] == "plan083_nunchaku"
    assert row["paper_scale"] is False
    assert row["body_count"] == 2
    assert row["dynamic_body_count"] == 1
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 2
    assert row["wall_time_ns"] == 3.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_windmill() -> None:
    module = _load_module()

    packet = module.make_packet(
        _windmill_packet(),
        max_equality_residual=1e-8,
        scene="windmill",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-20"
    assert row["scene_id"] == "plan083_windmill"
    assert row["paper_scale"] is False
    assert row["body_count"] == 3
    assert row["dynamic_body_count"] == 2
    assert row["revolute_joint_count"] == 1
    assert row["active_articulation_constraints"] == 2
    assert row["wall_time_ns"] == 4.0e6


def test_plan083_cpu_scene_packet_rejects_failed_bridge_step() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="failed steps"):
        module.make_packet(
            _benchmark_packet(failed_steps=1),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_rejects_high_equality_residual() -> None:
    module = _load_module()

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="equality residual",
    ):
        module.make_packet(
            _benchmark_packet(final_equality_residual_norm=1e-5),
            max_equality_residual=1e-8,
        )


def test_plan083_cpu_scene_packet_requires_representative_row() -> None:
    module = _load_module()
    packet = _benchmark_packet(
        run_name="BM_Plan083CpuScene_other_scene",
        name="BM_Plan083CpuScene_other_scene_median",
    )

    with pytest.raises(
        module.Plan083CpuScenePacketError,
        match="unexpected benchmark row",
    ):
        module.make_packet(packet, max_equality_residual=1e-8)


def test_plan083_cpu_scene_packet_rejects_nunchaku_without_hinge() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="revolute joint"):
        module.make_packet(
            _nunchaku_packet(revolute_joint_count=0),
            max_equality_residual=1e-8,
            scene="nunchaku_single",
        )


def test_plan083_cpu_scene_packet_rejects_windmill_penetration() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="penetrated"):
        module.make_packet(
            _windmill_packet(striker_blade_clearance_m=-1e-3),
            max_equality_residual=1e-8,
            scene="windmill",
        )
