"""AVBD port of the avbd-demo3d Pyramid source scene."""

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
_PYRAMID_SIZE = 16
_BOX_COUNT = _PYRAMID_SIZE * (_PYRAMID_SIZE + 1) // 2
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_BOX_SIZE = np.array([1.0, 0.5, 0.5])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 4,
    "scene_name": "Pyramid",
    "scene_count": 14,
    "scene_builder": "scenePyramid",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "z",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "size": tuple(_GROUND_SIZE),
            "density": 0.0,
            "friction": 0.5,
            "position": (0.0, 0.0, -0.5),
        },
        "boxes": {
            "pyramid_size": _PYRAMID_SIZE,
            "count": _BOX_COUNT,
            "size": tuple(_BOX_SIZE),
            "density": 1.0,
            "friction": 0.5,
            "x_spacing": 1.01,
            "row_x_offset": 0.5,
            "z_spacing": 0.85,
            "base_z": 0.5,
        },
    },
    "expected_counts": {
        "rigid_bodies": 137,
        "joints": 0,
        "collision_shapes": 137,
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


def _source_box_position(column: int, row: int) -> np.ndarray:
    return np.array(
        [
            float(column) * 1.01 + float(row) * 0.5 - float(_PYRAMID_SIZE) / 2.0,
            0.0,
            float(row) * 0.85 + 0.5,
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
    friction: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
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
        "avbd_demo3d_pyramid_ground",
        size=_GROUND_SIZE,
        density=0.0,
        friction=0.5,
        position=np.array([0.0, 0.0, -0.5]),
        is_static=True,
    )

    boxes: list[sx.RigidBody] = []
    for row in range(_PYRAMID_SIZE):
        for column in range(_PYRAMID_SIZE - row):
            boxes.append(
                _add_source_box(
                    world,
                    f"avbd_demo3d_pyramid_box_{column}_{row}",
                    size=_BOX_SIZE,
                    density=1.0,
                    friction=0.5,
                    position=_source_box_position(column, row),
                )
            )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_pyramid_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_pyramid_ground_visual",
    )
    for index, box in enumerate(boxes):
        row = int((2 * _PYRAMID_SIZE + 1 - np.sqrt((2 * _PYRAMID_SIZE + 1) ** 2 - 8 * index)) // 2)
        blend = float(row) / float(_PYRAMID_SIZE - 1)
        color = (0.82 - 0.30 * blend, 0.50 + 0.20 * blend, 0.28 + 0.34 * blend)
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_BOX_SIZE),
            color,
            name=f"avbd_demo3d_pyramid_box_{index}_visual",
        )
    bridge.sync()

    contact_history: deque[float] = deque(maxlen=160)
    top_box = boxes[-1]
    initial_top_z = float(np.asarray(top_box.translation, dtype=float).reshape(3)[2])

    def build_panel(builder: object, context: object) -> None:
        top_z = float(np.asarray(top_box.translation, dtype=float).reshape(3)[2])
        contacts = len(world.collide())
        contact_history.append(float(contacts))

        builder.text("source corpus: avbd-demo3d Pyramid")
        builder.text("source scene: scenePyramid, index 4 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {1 + len(boxes)}")
        builder.text(f"contacts: {contacts}")
        builder.text(f"top-box dz: {top_z - initial_top_z:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Contacts", list(contact_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Pyramid", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "boxes": boxes,
            "source_demo_row": "avbd-demo3d pyramid",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_pyramid",
    title="AVBD Demo3D Pyramid (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Pyramid row port with a triangular rigid-body pile.",
    build=build,
)
