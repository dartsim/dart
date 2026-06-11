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
