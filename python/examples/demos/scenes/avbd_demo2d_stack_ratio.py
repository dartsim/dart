"""AVBD port of the avbd-demo2d Stack Ratio source scene."""

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
_THICKNESS = 0.2
_BOX_COUNT = 6
_GROUND_SIZE_2D = np.array([100.0, 1.0])
_GROUND_SIZE = np.array([100.0, 1.0, _THICKNESS])
_SOURCE_BOX_SIZES_2D = (
    (1.0, 1.0),
    (2.0, 2.0),
    (4.0, 4.0),
    (8.0, 8.0),
    (16.0, 16.0),
    (32.0, 32.0),
)
_SOURCE_BOX_CENTERS_Y = (1.0, 2.0, 5.0, 11.0, 23.0, 47.0)
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 12,
    "scene_name": "Stack Ratio",
    "scene_count": 19,
    "scene_builder": "sceneStackRatio",
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
            "position": (0.0, 0.0, 0.0),
        },
        "boxes": {
            "count": _BOX_COUNT,
            "initial_size": 1.0,
            "size_multiplier": 2.0,
            "sizes": _SOURCE_BOX_SIZES_2D,
            "centers_y": _SOURCE_BOX_CENTERS_Y,
            "density": 1.0,
            "friction": _FRICTION,
        },
    },
    "expected_counts": {
        "rigid_bodies": _BOX_COUNT + 1,
        "dynamic_bodies": _BOX_COUNT,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": _BOX_COUNT + 1,
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


def _source_box_specs() -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    specs: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    for size_2d_tuple, center_y in zip(
        _SOURCE_BOX_SIZES_2D, _SOURCE_BOX_CENTERS_Y, strict=True
    ):
        size_2d = np.array(size_2d_tuple)
        size = np.array([size_2d[0], size_2d[1], _THICKNESS])
        position = np.array([0.0, center_y, 0.0])
        specs.append((size_2d, size, position))
    return specs


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
    size_2d: np.ndarray,
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
        mass = _box_mass_2d(size_2d, density)
        body.mass = mass
        body.inertia = _full_box_inertia(size, mass)
    return body


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = _add_source_box(
        world,
        "avbd_demo2d_stack_ratio_ground",
        size_2d=_GROUND_SIZE_2D,
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    boxes: list[sx.RigidBody] = []
    box_sizes: list[np.ndarray] = []
    for index, (size_2d, size, position) in enumerate(_source_box_specs()):
        boxes.append(
            _add_source_box(
                world,
                f"avbd_demo2d_stack_ratio_box_{index}",
                size_2d=size_2d,
                size=size,
                density=1.0,
                position=position,
            )
        )
        box_sizes.append(size)

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_stack_ratio_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo2d_stack_ratio_ground_visual",
    )
    for index, (box, size) in enumerate(zip(boxes, box_sizes, strict=True)):
        blend = float(index) / float(max(1, len(boxes) - 1))
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(size),
            (0.70 - 0.34 * blend, 0.48 + 0.20 * blend, 0.31 + 0.34 * blend),
            name=f"avbd_demo2d_stack_ratio_box_{index}_visual",
        )
    bridge.sync()

    contact_history: deque[float] = deque(maxlen=160)
    top_box = boxes[-1]
    initial_top_y = float(np.asarray(top_box.translation, dtype=float).reshape(3)[1])

    def build_panel(builder: object, context: object) -> None:
        top_y = float(np.asarray(top_box.translation, dtype=float).reshape(3)[1])
        contacts = len(world.collide())
        contact_history.append(float(contacts))

        builder.text("source corpus: avbd-demo2d Stack Ratio")
        builder.text("source scene: sceneStackRatio, index 12 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {1 + len(boxes)}")
        builder.text(f"contacts: {contacts}")
        builder.text(f"top-box dy: {top_y - initial_top_y:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Contacts", list(contact_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Stack Ratio", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "boxes": boxes,
            "box_sizes": box_sizes,
            "source_demo_row": "avbd-demo2d stack ratio",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_stack_ratio",
    title="AVBD Demo2D Stack Ratio (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Stack Ratio row port with six size-ratio 2D boxes.",
    build=build,
)
