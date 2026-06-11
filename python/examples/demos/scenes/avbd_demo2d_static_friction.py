"""AVBD port of the avbd-demo2d Static Friction source scene."""

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
_FRICTION = 1.0
_THICKNESS = 0.2
_SOURCE_RAMP_ANGLE = 3.14159 / 6.0
_GROUND_SIZE_2D = np.array([100.0, 1.0])
_GROUND_SIZE = np.array([100.0, 1.0, _THICKNESS])
_BOX_SIZE_2D = np.array([5.0, 0.5])
_BOX_SIZE = np.array([5.0, 0.5, _THICKNESS])
_BOX_COUNT = 11
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 3,
    "scene_name": "Static Friction",
    "scene_count": 19,
    "scene_builder": "sceneStaticFriction",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "size": tuple(_GROUND_SIZE_2D),
            "density": 0.0,
            "friction": _FRICTION,
            "position": (0.0, 0.0, _SOURCE_RAMP_ANGLE),
        },
        "boxes": {
            "count": _BOX_COUNT,
            "size": tuple(_BOX_SIZE_2D),
            "density": 1.0,
            "friction": _FRICTION,
            "first_position": (0.0, 1.0, _SOURCE_RAMP_ANGLE),
            "y_spacing": 1.0,
        },
    },
    "expected_counts": {
        "rigid_bodies": 12,
        "dynamic_bodies": 11,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 12,
    },
}


def _box_mass_2d(size_xy: np.ndarray, density: float) -> float:
    return float(np.prod(size_xy) * density)


def _full_box_inertia(size: np.ndarray, mass: float) -> np.ndarray:
    return np.diag(
        [
            mass * float(size[1] * size[1] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[1] * size[1]) / 12.0,
        ]
    )


def _z_axis_orientation(angle: float) -> tuple[float, float, float, float]:
    return (float(np.cos(0.5 * angle)), 0.0, 0.0, float(np.sin(0.5 * angle)))


def _rotate_z(angle: float, vector: np.ndarray) -> np.ndarray:
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    x, y, z = vector
    return np.array([c * x - s * y, s * x + c * y, z])


def _ramp_tangent() -> np.ndarray:
    return _rotate_z(_SOURCE_RAMP_ANGLE, np.array([1.0, 0.0, 0.0]))


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


def _source_box_position(index: int) -> np.ndarray:
    return np.array([0.0, float(index) + 1.0, 0.0])


def _add_source_box(
    world: sx.World,
    name: str,
    *,
    size: np.ndarray,
    size_2d: np.ndarray,
    density: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(
        name,
        position=tuple(position),
        orientation=_z_axis_orientation(_SOURCE_RAMP_ANGLE),
    )
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * size))
    if not is_static:
        mass = _box_mass_2d(size_2d, density)
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = _add_source_box(
        world,
        "avbd_demo2d_static_friction_ground",
        size=_GROUND_SIZE,
        size_2d=_GROUND_SIZE_2D,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    boxes: list[sx.RigidBody] = []
    for index in range(_BOX_COUNT):
        boxes.append(
            _add_source_box(
                world,
                f"avbd_demo2d_static_friction_box_{index}",
                size=_BOX_SIZE,
                size_2d=_BOX_SIZE_2D,
                density=1.0,
                position=_source_box_position(index),
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_static_friction_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.42, 0.43),
        name="avbd_demo2d_static_friction_ground_visual",
    )
    for index, box in enumerate(boxes):
        blend = float(index) / float(_BOX_COUNT - 1)
        color = (0.82 - 0.48 * blend, 0.42 + 0.18 * blend, 0.24 + 0.42 * blend)
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_BOX_SIZE),
            color,
            name=f"avbd_demo2d_static_friction_box_{index}_visual",
        )
    bridge.sync()

    low_box_history: deque[float] = deque(maxlen=160)
    high_box_history: deque[float] = deque(maxlen=160)
    initial_low_position = np.asarray(boxes[0].translation, dtype=float).reshape(3)
    initial_high_position = np.asarray(boxes[-1].translation, dtype=float).reshape(3)
    tangent = _ramp_tangent()

    def build_panel(builder: object, context: object) -> None:
        low_position = np.asarray(boxes[0].translation, dtype=float).reshape(3)
        high_position = np.asarray(boxes[-1].translation, dtype=float).reshape(3)
        low_slide = float(np.dot(low_position - initial_low_position, tangent))
        high_slide = float(np.dot(high_position - initial_high_position, tangent))
        low_box_history.append(low_slide)
        high_box_history.append(high_slide)

        builder.text("source corpus: avbd-demo2d Static Friction")
        builder.text("source scene: sceneStaticFriction, index 3 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {1 + len(boxes)}")
        builder.text(f"lowest box slide: {low_slide:.3f} m")
        builder.text(f"highest box slide: {high_slide:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Lowest box slide", list(low_box_history))
        builder.plot_lines("Highest box slide", list(high_box_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Static Friction", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "boxes": boxes,
            "ramp_tangent": _ramp_tangent(),
            "source_demo_row": "avbd-demo2d static friction",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_static_friction",
    title="AVBD Demo2D Static Friction (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Static Friction row port with rotated 2D boxes "
    "on an inclined ground slab.",
    build=build,
)
