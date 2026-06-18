"""Runtime smoke for unified Newton-barrier py-demos."""

from __future__ import annotations

import pathlib
import sys

import numpy as np
import pytest

_PYTHON_DIR = pathlib.Path(__file__).resolve().parents[2]
if str(_PYTHON_DIR) not in sys.path:
    sys.path.insert(0, str(_PYTHON_DIR))


def _sx():
    try:
        import dartpy as sx
    except ModuleNotFoundError as exc:
        pytest.skip(f"dartpy unavailable: {exc}")
    missing = [
        name
        for name in ("World", "RigidBodySolver", "CollisionShape")
        if not hasattr(sx, name)
    ]
    if missing:
        pytest.skip(f"dartpy symbols unavailable: {', '.join(missing)}")
    return sx


def test_hanging_bridge_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_hanging_bridge")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    traveler = setup.info["traveler"]
    boards = setup.info["boards"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_fixed_joints == len(boards)
    assert traveler.is_kinematic is True
    assert all(board.is_static for board in boards)
    assert "--headless" in setup.info["plan083_smoke_command"]
    assert setup.info["plan083_visual_command"].startswith("pixi run py-demo-capture")
    assert setup.pre_step is not None

    initial = np.asarray(traveler.translation, dtype=float).reshape(3).copy()
    for _ in range(24):
        setup.pre_step()

    final = np.asarray(traveler.translation, dtype=float).reshape(3)
    assert np.all(np.isfinite(final))
    assert final[0] > initial[0] + 0.03
    assert final[2] < initial[2] - 0.03
    for board in boards:
        assert np.all(np.isfinite(np.asarray(board.translation, dtype=float)))


def test_nunchaku_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_nunchaku")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    swinging = setup.info["swinging"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_joints == 1
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-nunchaku-packet"
    )
    assert setup.pre_step is not None

    initial = np.asarray(swinging.transform, dtype=float).reshape(4, 4).copy()
    for _ in range(24):
        setup.pre_step()

    final = np.asarray(swinging.transform, dtype=float).reshape(4, 4)
    assert np.all(np.isfinite(final))
    assert not np.allclose(final[:3, :3], initial[:3, :3])


def test_windmill_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_windmill")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    blade = setup.info["blade"]
    striker = setup.info["striker"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_joints == 1
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-windmill-packet"
    )
    assert setup.pre_step is not None

    initial_blade = np.asarray(blade.transform, dtype=float).reshape(4, 4).copy()
    initial_striker = np.asarray(striker.translation, dtype=float).reshape(3).copy()
    for _ in range(24):
        setup.pre_step()

    final_blade = np.asarray(blade.transform, dtype=float).reshape(4, 4)
    final_striker = np.asarray(striker.translation, dtype=float).reshape(3)
    assert np.all(np.isfinite(final_blade))
    assert np.all(np.isfinite(final_striker))
    assert not np.allclose(final_blade[:3, :3], initial_blade[:3, :3])
    assert final_striker[2] < initial_striker[2]


def test_umbrella_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_umbrella")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    hub = setup.info["hub"]
    ribs = setup.info["ribs"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_joints == 3
    assert sx_world.num_rigid_body_fixed_joints == 2
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-umbrella-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    for _ in range(24):
        setup.pre_step()

    final_hub = np.asarray(hub.transform, dtype=float).reshape(4, 4)
    final_ribs = [
        np.asarray(rib.translation, dtype=float).reshape(3) for rib in ribs
    ]
    final_span = float(np.linalg.norm(final_ribs[1] - final_ribs[0]))
    assert np.all(np.isfinite(final_hub))
    assert all(np.all(np.isfinite(rib)) for rib in final_ribs)
    assert sx_world.time > initial_time
    assert final_span > 0.0


def test_candy_demo_steps_deformable_ipc_world() -> None:
    _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_candy")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    cloth = setup.info["cloth"]

    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["deformable_solver"] == "ipc"
    assert sx_world.num_deformable_bodies == 1
    assert sx_world.num_rigid_bodies == 1
    assert cloth.node_count == 25
    assert cloth.surface_triangle_count == 32
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-candy-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    for _ in range(24):
        setup.pre_step()

    final_positions = [
        np.asarray(cloth.node_position(index), dtype=float).reshape(3)
        for index in range(cloth.node_count)
    ]
    diagnostics = sx_world.last_deformable_solver_diagnostics
    assert all(np.all(np.isfinite(point)) for point in final_positions)
    assert sx_world.time > initial_time
    assert diagnostics.body_count == 1
    assert diagnostics.node_count == cloth.node_count
    assert min(point[2] for point in final_positions) > 0.0


def test_lying_flat_demo_steps_deformable_ipc_world() -> None:
    _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_lying_flat")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    cloth = setup.info["cloth"]

    assert setup.info["runtime_smoke_scene"] is True
    assert setup.info["deformable_solver"] == "ipc"
    assert sx_world.num_deformable_bodies == 1
    assert sx_world.num_rigid_bodies == 4
    assert cloth.node_count == 24
    assert cloth.surface_triangle_count == 30
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-lying-flat-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    for _ in range(24):
        setup.pre_step()

    final_positions = [
        np.asarray(cloth.node_position(index), dtype=float).reshape(3)
        for index in range(cloth.node_count)
    ]
    diagnostics = sx_world.last_deformable_solver_diagnostics
    assert all(np.all(np.isfinite(point)) for point in final_positions)
    assert sx_world.time > initial_time
    assert diagnostics.body_count == 1
    assert diagnostics.node_count == cloth.node_count
    assert min(point[2] for point in final_positions) > 0.0


def test_terrain_vehicle_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(
        scene for scene in PLAN083_SCENES if scene.id == "plan083_terrain_vehicle"
    )
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    chassis = setup.info["chassis"]
    wheels = setup.info["wheels"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_joints == 4
    assert chassis.is_kinematic is True
    assert len(wheels) == 4
    assert all(wheel.is_kinematic for wheel in wheels)
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-terrain-vehicle-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    initial_chassis = np.asarray(chassis.translation, dtype=float).reshape(3).copy()
    initial_wheels = [
        np.asarray(wheel.translation, dtype=float).reshape(3).copy()
        for wheel in wheels
    ]
    for _ in range(24):
        setup.pre_step()

    final_chassis = np.asarray(chassis.translation, dtype=float).reshape(3)
    final_wheels = [
        np.asarray(wheel.translation, dtype=float).reshape(3)
        for wheel in wheels
    ]
    assert sx_world.time > initial_time
    assert np.all(np.isfinite(final_chassis))
    assert all(np.all(np.isfinite(wheel)) for wheel in final_wheels)
    assert final_chassis[0] > initial_chassis[0]
    assert all(
        wheel[0] > initial_wheel[0]
        for wheel, initial_wheel in zip(final_wheels, initial_wheels)
    )


def test_precession_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(
        scene for scene in PLAN083_SCENES if scene.id == "plan083_precession"
    )
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    wheel = setup.info["wheel"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-precession-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    initial_translation = np.asarray(wheel.translation, dtype=float).reshape(3).copy()
    initial_transform = np.asarray(wheel.transform, dtype=float).reshape(4, 4).copy()
    for _ in range(24):
        setup.pre_step()

    final_translation = np.asarray(wheel.translation, dtype=float).reshape(3)
    final_transform = np.asarray(wheel.transform, dtype=float).reshape(4, 4)
    assert sx_world.time > initial_time
    assert np.all(np.isfinite(final_translation))
    assert np.all(np.isfinite(final_transform))
    assert final_translation[0] > initial_translation[0]
    assert not np.allclose(final_transform[:3, :3], initial_transform[:3, :3])


def test_ragdolls_demo_steps_rigid_ipc_world() -> None:
    sx = _sx()
    from examples.demos.scenes.plan083_unified_newton_barrier import PLAN083_SCENES

    scene = next(scene for scene in PLAN083_SCENES if scene.id == "plan083_ragdolls")
    setup = scene.build()
    sx_world = setup.info["sx_world"]
    torso = setup.info["torso"]
    parts = setup.info["ragdoll_parts"]

    assert setup.info["runtime_smoke_scene"] is True
    assert sx_world.rigid_body_solver == sx.RigidBodySolver.IPC
    assert sx_world.num_rigid_body_joints == 5
    assert len(parts) == 5
    assert setup.info["plan083_benchmark_command"] == (
        "pixi run bm-plan083-cpu-ragdoll-packet"
    )
    assert setup.pre_step is not None

    initial_time = sx_world.time
    initial_torso = np.asarray(torso.translation, dtype=float).reshape(3).copy()
    for _ in range(24):
        setup.pre_step()

    final_torso = np.asarray(torso.translation, dtype=float).reshape(3)
    assert sx_world.time > initial_time
    assert np.all(np.isfinite(final_torso))
    assert final_torso[0] > initial_torso[0]
    for part in parts:
        assert np.all(np.isfinite(np.asarray(part.translation, dtype=float)))
