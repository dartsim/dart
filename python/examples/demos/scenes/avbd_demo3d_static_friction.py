"""AVBD port of the avbd-demo3d Static Friction source scene."""

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
_RAMP_ANGLE = np.deg2rad(30.0)
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_RAMP_SIZE = np.array([40.0, 24.0, 1.0])
_BOX_SIZE = np.array([1.0, 1.0, 1.0])
_BOX_COUNT = 11
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 3,
    "scene_name": "Static Friction",
    "scene_count": 14,
    "scene_builder": "sceneStaticFriction",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "z",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {"size": tuple(_GROUND_SIZE), "density": 0.0, "friction": 0.5},
        "ramp": {
            "size": tuple(_RAMP_SIZE),
            "density": 0.0,
            "friction": 1.0,
            "angle_degrees": 30.0,
        },
        "boxes": {
            "count": _BOX_COUNT,
            "size": tuple(_BOX_SIZE),
            "density": 1.0,
            "friction_range": (0.25, 0.5),
        },
    },
    "expected_counts": {
        "rigid_bodies": 13,
        "joints": 0,
        "collision_shapes": 13,
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


def _y_axis_orientation(angle: float) -> tuple[float, float, float, float]:
    return (float(np.cos(0.5 * angle)), 0.0, float(np.sin(0.5 * angle)), 0.0)


def _rotate_y(angle: float, vector: np.ndarray) -> np.ndarray:
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    x, y, z = vector
    return np.array([c * x + s * z, y, -s * x + c * z])


def _ramp_tangent() -> np.ndarray:
    return _rotate_y(_RAMP_ANGLE, np.array([1.0, 0.0, 0.0]))


def _ramp_normal() -> np.ndarray:
    return _rotate_y(_RAMP_ANGLE, np.array([0.0, 0.0, 1.0]))


def _source_box_position(index: int) -> np.ndarray:
    ramp_position = np.array([0.0, 0.0, 3.0])
    return (
        ramp_position
        + _ramp_tangent() * -12.0
        + np.array([0.0, -10.0 + 2.0 * float(index), 0.0])
        + _ramp_normal() * 1.05
    )


def _source_box_friction(index: int) -> float:
    return float(index) / 10.0 * 0.25 + 0.25


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
    friction: float,
    position: np.ndarray,
    orientation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position), orientation=orientation)
    body.is_static = is_static
    body.friction = friction
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
        "avbd_demo3d_static_friction_ground",
        size=_GROUND_SIZE,
        density=0.0,
        friction=0.5,
        position=np.zeros(3),
        is_static=True,
    )
    ramp = _add_source_box(
        world,
        "avbd_demo3d_static_friction_ramp",
        size=_RAMP_SIZE,
        density=0.0,
        friction=1.0,
        position=np.array([0.0, 0.0, 3.0]),
        orientation=_y_axis_orientation(_RAMP_ANGLE),
        is_static=True,
    )

    boxes: list[sx.RigidBody] = []
    for index in range(_BOX_COUNT):
        boxes.append(
            _add_source_box(
                world,
                f"avbd_demo3d_static_friction_box_{index}",
                size=_BOX_SIZE,
                density=1.0,
                friction=_source_box_friction(index),
                position=_source_box_position(index),
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_static_friction_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_static_friction_ground_visual",
    )
    bridge.add_rigid_body_visual(
        ramp,
        dart.BoxShape(_RAMP_SIZE),
        (0.34, 0.45, 0.38),
        name="avbd_demo3d_static_friction_ramp_visual",
    )
    for index, box in enumerate(boxes):
        blend = float(index) / float(_BOX_COUNT - 1)
        color = (0.86 - 0.50 * blend, 0.40 + 0.20 * blend, 0.18 + 0.46 * blend)
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_BOX_SIZE),
            color,
            name=f"avbd_demo3d_static_friction_box_{index}_visual",
        )
    bridge.sync()

    low_friction_history: deque[float] = deque(maxlen=160)
    high_friction_history: deque[float] = deque(maxlen=160)

    initial_low_position = np.asarray(boxes[0].translation, dtype=float).reshape(3)
    initial_high_position = np.asarray(boxes[-1].translation, dtype=float).reshape(3)
    tangent = _ramp_tangent()

    def build_panel(builder: object, context: object) -> None:
        low_position = np.asarray(boxes[0].translation, dtype=float).reshape(3)
        high_position = np.asarray(boxes[-1].translation, dtype=float).reshape(3)
        low_slide = float(np.dot(low_position - initial_low_position, tangent))
        high_slide = float(np.dot(high_position - initial_high_position, tangent))
        low_friction_history.append(low_slide)
        high_friction_history.append(high_slide)

        builder.text("source corpus: avbd-demo3d Static Friction")
        builder.text("source scene: sceneStaticFriction, index 3 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {2 + len(boxes)}")
        builder.text(f"low-friction slide: {low_slide:.3f} m")
        builder.text(f"high-friction slide: {high_slide:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Friction 0.25 slide", list(low_friction_history))
        builder.plot_lines("Friction 0.50 slide", list(high_friction_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Static Friction", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "ramp": ramp,
            "boxes": boxes,
            "ramp_tangent": _ramp_tangent(),
            "ramp_normal": _ramp_normal(),
            "source_demo_row": "avbd-demo3d static friction",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_static_friction",
    title="AVBD Demo3D Static Friction (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Static Friction row port with rigid boxes sliding "
    "on an inclined ramp.",
    build=build,
)
