"""AVBD port of the avbd-demo2d Dynamic Friction source scene."""

from __future__ import annotations

import os
from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 60.0
_GRAVITY = -10.0
_GROUND_FRICTION = 0.5
_THICKNESS = 0.2
_GROUND_SIZE_2D = np.array([100.0, 1.0])
_GROUND_SIZE = np.array([100.0, 1.0, _THICKNESS])
_BOX_SIZE_2D = np.array([1.0, 0.5])
_BOX_SIZE = np.array([1.0, 0.5, _THICKNESS])
_BOX_COUNT = 11
_BOX_INITIAL_SPEED = 10.0
_MAX_FRICTION_ENV = "DART_AVBD_DEMO2D_DYNAMIC_FRICTION_MAX_FRICTION"
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 2,
    "scene_name": "Dynamic Friction",
    "scene_count": 19,
    "scene_builder": "sceneDynamicFriction",
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
            "friction": _GROUND_FRICTION,
        },
        "boxes": {
            "count": _BOX_COUNT,
            "size": tuple(_BOX_SIZE_2D),
            "density": 1.0,
            "friction_range": (5.0, 0.0),
            "first_position": (-30.0, 0.75, 0.0),
            "x_spacing": 2.0,
            "initial_velocity": (_BOX_INITIAL_SPEED, 0.0, 0.0),
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


def _dynamic_max_friction_from_env() -> float:
    value = os.environ.get(_MAX_FRICTION_ENV)
    if value is None:
        return 5.0
    try:
        max_friction = float(value)
    except ValueError as exc:
        raise ValueError(f"{_MAX_FRICTION_ENV} must be a finite number") from exc
    if not np.isfinite(max_friction):
        raise ValueError(f"{_MAX_FRICTION_ENV} must be finite")
    if max_friction < 0.0:
        raise ValueError(f"{_MAX_FRICTION_ENV} must be non-negative")
    return max_friction


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


def _source_row(max_friction: float) -> dict[str, Any]:
    source_shapes = {
        name: dict(shape) for name, shape in _SOURCE_ROW["source_shapes"].items()
    }
    source_shapes["boxes"]["friction_range"] = (max_friction, 0.0)
    return {
        **_SOURCE_ROW,
        "solver_defaults": dict(_SOURCE_ROW["solver_defaults"]),
        "source_shapes": source_shapes,
        "expected_counts": dict(_SOURCE_ROW["expected_counts"]),
        "parameters": {
            "max_dynamic_box_friction": max_friction,
            "environment_variable": _MAX_FRICTION_ENV,
        },
    }


def _source_box_position(index: int) -> np.ndarray:
    return np.array([-30.0 + 2.0 * float(index), 0.75, 0.0])


def _source_box_friction(index: int, max_friction: float) -> float:
    return max_friction - float(index) / 10.0 * max_friction


def _add_source_box(
    world: sx.World,
    name: str,
    *,
    size: np.ndarray,
    size_2d: np.ndarray,
    density: float,
    friction: float,
    position: np.ndarray,
    linear_velocity: np.ndarray | None = None,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(
        name,
        position=tuple(position),
        linear_velocity=tuple(
            np.zeros(3) if linear_velocity is None else linear_velocity
        ),
    )
    body.is_static = is_static
    body.friction = friction
    body.set_collision_shape(sx.CollisionShape.box(0.5 * size))
    if not is_static:
        mass = _box_mass_2d(size_2d, density)
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def build() -> SceneSetup:
    max_friction = _dynamic_max_friction_from_env()
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = _add_source_box(
        world,
        "avbd_demo2d_dynamic_friction_ground",
        size=_GROUND_SIZE,
        size_2d=_GROUND_SIZE_2D,
        density=0.0,
        friction=_GROUND_FRICTION,
        position=np.zeros(3),
        is_static=True,
    )

    boxes: list[sx.RigidBody] = []
    for index in range(_BOX_COUNT):
        boxes.append(
            _add_source_box(
                world,
                f"avbd_demo2d_dynamic_friction_box_{index}",
                size=_BOX_SIZE,
                size_2d=_BOX_SIZE_2D,
                density=1.0,
                friction=_source_box_friction(index, max_friction),
                position=_source_box_position(index),
                linear_velocity=np.array([_BOX_INITIAL_SPEED, 0.0, 0.0]),
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_dynamic_friction_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.42, 0.43, 0.45),
        name="avbd_demo2d_dynamic_friction_ground_visual",
    )
    for index, box in enumerate(boxes):
        blend = float(index) / float(_BOX_COUNT - 1)
        color = (0.22 + 0.58 * blend, 0.58 - 0.25 * blend, 0.86 - 0.52 * blend)
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_BOX_SIZE),
            color,
            name=f"avbd_demo2d_dynamic_friction_box_{index}_visual",
        )
    bridge.sync()

    high_friction_speed_history: deque[float] = deque(maxlen=160)
    zero_friction_speed_history: deque[float] = deque(maxlen=160)
    high_friction_speed_label = f"Friction {max_friction:.3g} speed"

    def build_panel(builder: object, context: object) -> None:
        high_friction_speed = float(
            np.linalg.norm(np.asarray(boxes[0].linear_velocity, dtype=float)[:2])
        )
        zero_friction_speed = float(
            np.linalg.norm(np.asarray(boxes[-1].linear_velocity, dtype=float)[:2])
        )
        high_friction_speed_history.append(high_friction_speed)
        zero_friction_speed_history.append(zero_friction_speed)

        builder.text("source corpus: avbd-demo2d Dynamic Friction")
        builder.text("source scene: sceneDynamicFriction, index 2 of 19")
        builder.text(f"max dynamic friction: {max_friction:.3g}")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {1 + len(boxes)}")
        builder.text(f"high-friction speed: {high_friction_speed:.3f} m/s")
        builder.text(f"zero-friction speed: {zero_friction_speed:.3f} m/s")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines(high_friction_speed_label, list(high_friction_speed_history))
        builder.plot_lines("Friction 0 speed", list(zero_friction_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Dynamic Friction", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "boxes": boxes,
            "max_dynamic_box_friction": max_friction,
            "high_friction_speed_label": high_friction_speed_label,
            "source_demo_row": "avbd-demo2d dynamic friction",
            "source_demo_reference": _source_row(max_friction),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_dynamic_friction",
    title="AVBD Demo2D Dynamic Friction (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Dynamic Friction row port with sliding 2D boxes "
    "and varying Coulomb friction.",
    build=build,
)
