"""The ports of the two-dimensional reference demos stay in the XY plane."""

from __future__ import annotations

import math

import numpy as np
import pytest

dart = pytest.importorskip("dartpy")

from examples.demos._avbd_demo2d_plane import (  # noqa: E402
    lock_to_xy_plane,
    plane_locked_pre_step,
)


def _z_rotation(angle: float) -> np.ndarray:
    rotation = np.eye(3)
    rotation[0, 0] = math.cos(angle)
    rotation[0, 1] = -math.sin(angle)
    rotation[1, 0] = math.sin(angle)
    rotation[1, 1] = math.cos(angle)
    return rotation


def _x_rotation(angle: float) -> np.ndarray:
    rotation = np.eye(3)
    rotation[1, 1] = math.cos(angle)
    rotation[1, 2] = -math.sin(angle)
    rotation[2, 1] = math.sin(angle)
    rotation[2, 2] = math.cos(angle)
    return rotation


def _world() -> dart.World:
    return dart.World(
        time_step=1.0 / 60.0,
        gravity=(0.0, -10.0, 0.0),
        rigid_body_solver=dart.RigidBodySolver.AVBD,
        rigid_avbd_parameter_profile=dart.RigidAvbdParameterProfile.SOURCE_DEMO_2D,
    )


def test_lock_removes_out_of_plane_state_and_keeps_the_plane_angle() -> None:
    world = _world()
    body = world.add_rigid_body("tilted", position=(1.5, 2.0, 0.25))
    body.mass = 1.0
    angle = 0.3
    tilted = np.eye(4)
    # A tilt about the body's own x axis keeps that axis in the plane, so the
    # lock's in-plane angle is exactly the z rotation.
    tilted[:3, :3] = _z_rotation(angle) @ _x_rotation(0.05)
    tilted[:3, 3] = (1.5, 2.0, 0.25)
    body.transform = tilted
    body.linear_velocity = (0.5, -1.0, 0.75)
    body.angular_velocity = (0.1, -0.2, 3.0)

    assert lock_to_xy_plane([body]) == 1

    transform = np.array(body.transform, dtype=float).reshape(4, 4)
    assert transform[:3, :3] == pytest.approx(_z_rotation(angle), abs=1e-12)
    assert transform[:3, 3].tolist() == pytest.approx([1.5, 2.0, 0.0])
    assert np.array(body.linear_velocity).tolist() == pytest.approx([0.5, -1.0, 0.0])
    assert np.array(body.angular_velocity).tolist() == pytest.approx([0.0, 0.0, 3.0])
    # A body that already lies in the plane is not written again.
    assert lock_to_xy_plane([body]) == 0


def test_lock_skips_static_bodies() -> None:
    world = _world()
    ground = world.add_rigid_body("ground", position=(0.0, 0.0, 0.4))
    ground.is_static = True
    assert lock_to_xy_plane([ground]) == 0
    assert np.array(ground.translation).tolist() == pytest.approx([0.0, 0.0, 0.4])


def test_locked_pre_step_keeps_the_bridge_reachable_for_replay() -> None:
    calls: list[str] = []

    class _Bridge:
        def pre_step(self) -> None:
            calls.append("stepped")

    bridge = _Bridge()
    world = _world()
    body = world.add_rigid_body("box", position=(0.0, 1.0, 0.1))
    body.mass = 1.0
    pre_step = plane_locked_pre_step(bridge, [body])
    pre_step()
    assert calls == ["stepped"]
    assert np.array(body.translation).tolist() == pytest.approx([0.0, 1.0, 0.0])
    closure = [cell.cell_contents for cell in pre_step.__closure__ or ()]
    assert any(value is bridge for value in closure)


@pytest.mark.parametrize(
    "module_name",
    ["avbd_demo2d_fracture", "avbd_demo2d_static_friction", "avbd_demo2d_heavy_rope"],
)
def test_two_dimensional_ports_stay_planar(module_name: str) -> None:
    import importlib

    module = importlib.import_module(f"examples.demos.scenes.{module_name}")
    setup = module.build()
    assert setup.info["replay_live_step_is_stateless"] is True
    bodies = [
        body
        for key in ("links", "boxes", "chain", "supports", "falling_blocks")
        for body in setup.info.get(key, ())
    ]
    assert bodies
    assert setup.pre_step is not None
    for _ in range(240):
        setup.pre_step()
        setup.world.step()
    for body in bodies:
        if body.is_static:
            continue
        translation = np.array(body.translation, dtype=float).reshape(3)
        rotation = np.array(body.rotation, dtype=float).reshape(3, 3)
        angular = np.array(body.angular_velocity, dtype=float).reshape(3)
        # The lock runs before each step, so at most one step of out-of-plane
        # motion is ever visible.
        assert abs(translation[2]) < 1e-3, body.name
        assert abs(rotation[2, 2] - 1.0) < 1e-6, body.name
        assert abs(angular[0]) < 1e-2 and abs(angular[1]) < 1e-2, body.name
