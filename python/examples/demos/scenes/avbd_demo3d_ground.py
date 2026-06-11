"""AVBD port of the avbd-demo3d Ground source scene."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 60.0
_GRAVITY = -10.0
_FRICTION = 0.5
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_BOX_SIZE = np.array([1.0, 1.0, 1.0])
_BOX_START = np.array([0.0, 0.0, 4.0])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 1,
    "scene_name": "Ground",
    "scene_count": 14,
    "scene_builder": "sceneGround",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "z",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {"size": tuple(_GROUND_SIZE), "density": 0.0, "friction": _FRICTION},
        "box": {"size": tuple(_BOX_SIZE), "density": 1.0, "friction": _FRICTION},
    },
    "expected_counts": {
        "rigid_bodies": 2,
        "joints": 0,
        "collision_shapes": 2,
    },
}


def _box_mass(size: np.ndarray, density: float) -> float:
    return float(np.prod(size) * density)


def _full_box_inertia(size: np.ndarray, mass: float) -> np.ndarray:
    return np.diag(
        [
            mass * float(size[1] * size[1] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[1] * size[1]) / 12.0,
        ]
    )


def _source_row() -> dict[str, Any]:
    return {
        **_SOURCE_ROW,
        "solver_defaults": dict(_SOURCE_ROW["solver_defaults"]),
        "source_shapes": {
            name: dict(shape)
            for name, shape in _SOURCE_ROW["source_shapes"].items()
        },
        "expected_counts": dict(_SOURCE_ROW["expected_counts"]),
    }


def _add_source_box(
    world: sx.World,
    name: str,
    *,
    size: np.ndarray,
    density: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * size))
    if not is_static:
        mass = _box_mass(size, density)
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, _GRAVITY))

    ground = _add_source_box(
        world,
        "avbd_demo3d_ground_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )
    box = _add_source_box(
        world,
        "avbd_demo3d_ground_box",
        size=_BOX_SIZE,
        density=1.0,
        position=_BOX_START,
    )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_ground_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.42, 0.43, 0.45),
        name="avbd_demo3d_ground_ground_visual",
    )
    bridge.add_rigid_body_visual(
        box,
        dart.BoxShape(_BOX_SIZE),
        (0.23, 0.56, 0.86),
        name="avbd_demo3d_ground_box_visual",
    )
    bridge.sync()

    box_z_history: deque[float] = deque(maxlen=160)
    box_speed_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        translation = np.asarray(box.translation, dtype=float).reshape(3)
        velocity = np.asarray(box.linear_velocity, dtype=float).reshape(3)
        box_z = float(translation[2])
        speed = float(np.linalg.norm(velocity))
        box_z_history.append(box_z)
        box_speed_history.append(speed)

        builder.text("source corpus: avbd-demo3d Ground")
        builder.text("source scene: sceneGround, index 1 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {len(ground.collision_shapes) + len(box.collision_shapes)}")
        builder.text(f"box z: {box_z:.3f} m")
        builder.text(f"box speed: {speed:.3f} m/s")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Box z", list(box_z_history))
        builder.plot_lines("Box speed", list(box_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Ground", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "box": box,
            "source_demo_row": "avbd-demo3d ground",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_ground",
    title="AVBD Demo3D Ground (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Ground row port with a falling rigid box and AVBD "
    "rigid contact.",
    build=build,
)
