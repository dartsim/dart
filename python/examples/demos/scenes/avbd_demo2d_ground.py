"""AVBD port of the avbd-demo2d Ground source scene."""

from __future__ import annotations

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
_GROUND_SIZE_2D = np.array([100.0, 1.0])
_GROUND_SIZE = np.array([100.0, 1.0, _THICKNESS])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 1,
    "scene_name": "Ground",
    "scene_count": 19,
    "scene_builder": "sceneGround",
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
    },
    "expected_counts": {
        "rigid_bodies": 1,
        "dynamic_bodies": 0,
        "static_bodies": 1,
        "joints": 0,
        "collision_shapes": 1,
    },
}


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


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = world.add_rigid_body("avbd_demo2d_ground_slab", position=(0.0, 0.0, 0.0))
    ground.is_static = True
    ground.friction = _FRICTION
    ground.set_collision_shape(sx.CollisionShape.box(0.5 * _GROUND_SIZE))

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_ground_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo2d_ground_slab_visual",
    )
    bridge.sync()

    def build_panel(builder: object, context: object) -> None:
        translation = np.asarray(ground.translation, dtype=float).reshape(3)
        contacts = len(world.collide())

        builder.text("source corpus: avbd-demo2d Ground")
        builder.text("source scene: sceneGround, index 1 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"collision shapes: {len(ground.collision_shapes)}")
        builder.text(f"contacts: {contacts}")
        builder.text(f"ground y: {translation[1]:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Ground", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "source_demo_row": "avbd-demo2d ground",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_ground",
    title="AVBD Demo2D Ground (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Ground row port with the static 2D source slab.",
    build=build,
)
