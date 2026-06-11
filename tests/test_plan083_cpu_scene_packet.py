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
        "solver_iterations": 0,
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
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "swinging_tip_radius_m": 0.36,
        "free_axis_angular_velocity_rad_s": 1.5,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _nunchaku_scaling_packet(**overrides):
    rows = []
    for size in (20, 40, 60, 80, 100):
        row = {
            "name": f"BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step/{size}_median",
            "run_name": f"BM_Plan083CpuScene_nunchaku_scaling_reduced_world_step/{size}",
            "aggregate_name": "median",
            "real_time": float(size),
            "cpu_time": float(size),
            "time_unit": "ms",
            "nunchaku_pair_count": size,
            "body_count": 2 * size,
            "dynamic_body_count": size,
            "revolute_joint_count": size,
            "active_articulation_constraints": 2 * size,
            "solver_iterations": 0,
            "failed_steps": 0,
            "final_equality_residual_norm": 0.0,
            "free_axis_angular_velocity_rad_s": 1.5,
        }
        if overrides.get("size") == size:
            row.update({k: v for k, v in overrides.items() if k != "size"})
        rows.append(row)
    return {"benchmarks": rows}


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
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "blade_tip_radius_m": 0.36,
        "striker_height_m": 0.95,
        "striker_blade_clearance_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _terrain_vehicle_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_terrain_vehicle_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 5.0,
        "cpu_time": 5.0,
        "time_unit": "ms",
        "body_count": 6,
        "dynamic_body_count": 5,
        "wheel_count": 4,
        "revolute_joint_count": 4,
        "active_constraints": 18,
        "active_friction_constraints": 10,
        "active_articulation_constraints": 8,
        "solver_iterations": 0,
        "failed_steps": 0,
        "final_equality_residual_norm": 1e-10,
        "chassis_height_m": 0.17,
        "min_wheel_ground_clearance_m": 0.01,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _precession_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_precession_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_precession_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 6.0,
        "cpu_time": 6.0,
        "time_unit": "ms",
        "body_count": 2,
        "dynamic_body_count": 1,
        "active_constraints": 8,
        "active_friction_constraints": 8,
        "solver_iterations": 3,
        "failed_steps": 0,
        "final_equality_residual_norm": 0.0,
        "wheel_height_m": 0.16,
        "wheel_ground_clearance_m": 0.0,
        "spin_rate_rad_s": 8.1,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _ragdoll_packet(**overrides):
    row = {
        "name": "BM_Plan083CpuScene_ragdoll_reduced_world_step_median",
        "run_name": "BM_Plan083CpuScene_ragdoll_reduced_world_step",
        "aggregate_name": "median",
        "real_time": 7.0,
        "cpu_time": 7.0,
        "time_unit": "ms",
        "body_count": 7,
        "dynamic_body_count": 6,
        "ragdoll_body_count": 6,
        "revolute_joint_count": 5,
        "active_constraints": 12,
        "active_friction_constraints": 12,
        "active_articulation_constraints": 10,
        "solver_iterations": 3,
        "failed_steps": 0,
        "final_equality_residual_norm": 0.0,
        "torso_height_m": 0.42,
        "min_leg_ground_clearance_m": 0.0,
    }
    row.update(overrides)
    return {"benchmarks": [row]}


def _timing_breakdown_packet(**overrides):
    rows = [
        _benchmark_packet()["benchmarks"][0],
        _nunchaku_packet()["benchmarks"][0],
        _terrain_vehicle_packet()["benchmarks"][0],
        _ragdoll_packet()["benchmarks"][0],
        _windmill_packet()["benchmarks"][0],
        _precession_packet()["benchmarks"][0],
    ]
    if "row_index" in overrides:
        row_index = overrides.pop("row_index")
        rows[row_index].update(overrides)
    return {"benchmarks": rows}


def _table2_packet(**overrides):
    rows = [
        _benchmark_packet()["benchmarks"][0],
        _terrain_vehicle_packet()["benchmarks"][0],
        _ragdoll_packet()["benchmarks"][0],
        _windmill_packet()["benchmarks"][0],
        _precession_packet()["benchmarks"][0],
    ]
    if "row_index" in overrides:
        row_index = overrides.pop("row_index")
        rows[row_index].update(overrides)
    return {"benchmarks": rows}


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


def test_plan083_cpu_scene_packet_accepts_reduced_nunchaku_scaling() -> None:
    module = _load_module()

    packet = module.make_packet(
        _nunchaku_scaling_packet(),
        max_equality_residual=1e-8,
        scene="nunchaku_scaling",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-25"
    assert row["scene_id"] == "plan083_nunchaku"
    assert row["paper_scale"] is False
    assert row["sample_sizes"] == [20, 40, 60, 80, 100]
    assert row["sample_count"] == 5
    assert row["samples"][0]["body_count"] == 40
    assert row["samples"][-1]["revolute_joint_count"] == 100


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


def test_plan083_cpu_scene_packet_accepts_reduced_terrain_vehicle() -> None:
    module = _load_module()

    packet = module.make_packet(
        _terrain_vehicle_packet(),
        max_equality_residual=1e-8,
        scene="terrain_vehicle",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-10"
    assert row["scene_id"] == "plan083_terrain_vehicle"
    assert row["paper_scale"] is False
    assert row["body_count"] == 6
    assert row["dynamic_body_count"] == 5
    assert row["wheel_count"] == 4
    assert row["revolute_joint_count"] == 4
    assert row["active_articulation_constraints"] == 8
    assert row["wall_time_ns"] == 5.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_precession() -> None:
    module = _load_module()

    packet = module.make_packet(
        _precession_packet(),
        max_equality_residual=1e-8,
        scene="precession",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-23"
    assert row["scene_id"] == "plan083_precession"
    assert row["paper_scale"] is False
    assert row["body_count"] == 2
    assert row["dynamic_body_count"] == 1
    assert row["active_constraints"] == 8
    assert row["active_friction_constraints"] == 8
    assert row["wall_time_ns"] == 6.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_ragdoll() -> None:
    module = _load_module()

    packet = module.make_packet(
        _ragdoll_packet(),
        max_equality_residual=1e-8,
        scene="ragdoll_reduced",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-11"
    assert row["scene_id"] == "plan083_ragdolls"
    assert row["paper_scale"] is False
    assert row["body_count"] == 7
    assert row["dynamic_body_count"] == 6
    assert row["ragdoll_body_count"] == 6
    assert row["revolute_joint_count"] == 5
    assert row["active_articulation_constraints"] == 10
    assert row["wall_time_ns"] == 7.0e6


def test_plan083_cpu_scene_packet_accepts_reduced_timing_breakdown() -> None:
    module = _load_module()

    packet = module.make_packet(
        _timing_breakdown_packet(),
        max_equality_residual=1e-8,
        scene="timing_breakdown",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-fig-24"
    assert row["scene_id"] == "plan083_reduced_timing_breakdown"
    assert row["paper_scale"] is False
    assert row["scene_count"] == 6
    assert row["total_body_count"] == 27
    assert row["total_dynamic_body_count"] == 20
    assert row["total_wall_time_ns"] == 27.0e6
    assert row["available_timing_fields"] == ["wall_time_ns"]
    assert "linear_solve" in row["missing_paper_timing_fields"]


def test_plan083_cpu_scene_packet_accepts_reduced_table2() -> None:
    module = _load_module()

    packet = module.make_packet(
        _table2_packet(),
        max_equality_residual=1e-8,
        scene="table_2",
    )

    row = packet["plan083_cpu_scene_packet"]
    assert row["row_id"] == "unb-table-02"
    assert row["scene_id"] == "plan083_reduced_table_2"
    assert row["paper_scale"] is False
    assert row["scene_count"] == 5
    assert row["covered_paper_rows"] == [
        "unb-fig-02",
        "unb-fig-10",
        "unb-fig-11",
        "unb-fig-20",
        "unb-fig-23",
    ]
    assert row["missing_paper_rows"] == [
        "unb-fig-01",
        "unb-fig-03",
        "unb-fig-04",
        "unb-fig-22",
    ]
    assert row["total_body_count"] == 25
    assert row["total_dynamic_body_count"] == 19
    assert row["total_wall_time_ns"] == 24.0e6


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


def test_plan083_cpu_scene_packet_rejects_nunchaku_scaling_without_size() -> None:
    module = _load_module()
    packet = _nunchaku_scaling_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="nunchaku_scaling",
        )


def test_plan083_cpu_scene_packet_rejects_windmill_penetration() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="penetrated"):
        module.make_packet(
            _windmill_packet(striker_blade_clearance_m=-1e-3),
            max_equality_residual=1e-8,
            scene="windmill",
        )


def test_plan083_cpu_scene_packet_rejects_terrain_vehicle_without_wheels() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="passive wheels"):
        module.make_packet(
            _terrain_vehicle_packet(wheel_count=3),
            max_equality_residual=1e-8,
            scene="terrain_vehicle",
        )


def test_plan083_cpu_scene_packet_rejects_precession_without_spin() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="spin rate"):
        module.make_packet(
            _precession_packet(spin_rate_rad_s=0.0),
            max_equality_residual=1e-8,
            scene="precession",
        )


def test_plan083_cpu_scene_packet_rejects_ragdoll_without_ground_contact() -> None:
    module = _load_module()

    with pytest.raises(module.Plan083CpuScenePacketError, match="ground-contact"):
        module.make_packet(
            _ragdoll_packet(active_constraints=0),
            max_equality_residual=1e-8,
            scene="ragdoll_reduced",
        )


def test_plan083_cpu_scene_packet_rejects_timing_breakdown_without_scene() -> None:
    module = _load_module()
    packet = _timing_breakdown_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="timing_breakdown",
        )


def test_plan083_cpu_scene_packet_rejects_table2_without_scene() -> None:
    module = _load_module()
    packet = _table2_packet()
    packet["benchmarks"] = packet["benchmarks"][:-1]

    with pytest.raises(module.Plan083CpuScenePacketError, match="missing median"):
        module.make_packet(
            packet,
            max_equality_residual=1e-8,
            scene="table_2",
        )
