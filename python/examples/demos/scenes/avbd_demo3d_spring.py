"""AVBD port of the avbd-demo3d Spring source scene."""

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
_ANCHOR_SIZE = np.array([1.0, 1.0, 1.0])
_BLOCK_SIZE = np.array([2.0, 2.0, 2.0])
_GROUND_POSITION = np.array([0.0, 0.0, 0.0])
_ANCHOR_POSITION = np.array([0.0, 0.0, 14.0])
_BLOCK_POSITION = np.array([0.0, 0.0, 8.0])
_CENTER_ANCHOR = np.array([0.0, 0.0, 0.0])
_SPRING_STIFFNESS = 100.0
_SPRING_REST_LENGTH = 4.0
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 7,
    "scene_name": "Spring",
    "scene_count": 14,
    "scene_builder": "sceneSpring",
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
            "friction": _FRICTION,
            "position": tuple(_GROUND_POSITION),
        },
        "anchor": {
            "size": tuple(_ANCHOR_SIZE),
            "density": 0.0,
            "friction": _FRICTION,
            "position": tuple(_ANCHOR_POSITION),
        },
        "block": {
            "size": tuple(_BLOCK_SIZE),
            "density": 1.0,
            "friction": _FRICTION,
            "position": tuple(_BLOCK_POSITION),
        },
    },
    "source_constraints": {
        "radial_distance_springs": {
            "count": 1,
            "parent_anchor": (0.0, 0.0, 0.0),
            "child_anchor": (0.0, 0.0, 0.0),
            "stiffness": _SPRING_STIFFNESS,
            "rest_length": _SPRING_REST_LENGTH,
        },
    },
    "expected_counts": {
        "rigid_bodies": 3,
        "dynamic_bodies": 1,
        "static_bodies": 2,
        "joints": 0,
        "distance_springs": 1,
        "collision_shapes": 3,
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
        "source_constraints": {
            name: dict(constraint)
            for name, constraint in _SOURCE_ROW["source_constraints"].items()
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


def _anchor_world_position(body: sx.RigidBody, local_anchor: np.ndarray) -> np.ndarray:
    rotation = np.asarray(body.rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(body.translation, dtype=float).reshape(3)
    return translation + rotation @ local_anchor


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, _GRAVITY))

    ground = _add_source_box(
        world,
        "avbd_demo3d_spring_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=_GROUND_POSITION,
        is_static=True,
    )
    anchor = _add_source_box(
        world,
        "avbd_demo3d_spring_anchor",
        size=_ANCHOR_SIZE,
        density=0.0,
        position=_ANCHOR_POSITION,
        is_static=True,
    )
    block = _add_source_box(
        world,
        "avbd_demo3d_spring_block",
        size=_BLOCK_SIZE,
        density=1.0,
        position=_BLOCK_POSITION,
    )

    world.add_rigid_body_distance_spring(
        "avbd_demo3d_spring_radial",
        anchor,
        block,
        _SPRING_REST_LENGTH,
        _SPRING_STIFFNESS,
        parent_anchor=tuple(_CENTER_ANCHOR),
        child_anchor=tuple(_CENTER_ANCHOR),
    )
    world.set_collision_pair_ignored(anchor, block)
    ignored_collision_pairs = [(anchor, block)]
    springs = [
        {
            "name": "avbd_demo3d_spring_radial",
            "parent": anchor.name,
            "child": block.name,
            "parent_anchor": tuple(_CENTER_ANCHOR),
            "child_anchor": tuple(_CENTER_ANCHOR),
            "rest_length": _SPRING_REST_LENGTH,
            "stiffness": _SPRING_STIFFNESS,
        }
    ]

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_spring_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_spring_ground_visual",
    )
    bridge.add_rigid_body_visual(
        anchor,
        dart.BoxShape(_ANCHOR_SIZE),
        (0.30, 0.38, 0.44),
        name="avbd_demo3d_spring_anchor_visual",
    )
    bridge.add_rigid_body_visual(
        block,
        dart.BoxShape(_BLOCK_SIZE),
        (0.79, 0.42, 0.22),
        name="avbd_demo3d_spring_block_visual",
    )
    bridge.sync()

    length_history: deque[float] = deque(maxlen=160)
    block_z_history: deque[float] = deque(maxlen=160)
    initial_block_z = float(np.asarray(block.translation, dtype=float).reshape(3)[2])

    def spring_length() -> float:
        return float(
            np.linalg.norm(
                _anchor_world_position(anchor, _CENTER_ANCHOR)
                - _anchor_world_position(block, _CENTER_ANCHOR)
            )
        )

    def build_panel(builder: object, context: object) -> None:
        length = spring_length()
        block_z = float(np.asarray(block.translation, dtype=float).reshape(3)[2])
        length_history.append(length)
        block_z_history.append(block_z)

        builder.text("source corpus: avbd-demo3d Spring")
        builder.text("source scene: sceneSpring, index 7 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"distance springs: {len(springs)}")
        builder.text(f"ignored collision pairs: {len(ignored_collision_pairs)}")
        builder.text(f"spring length: {length:.3f} m")
        builder.text(f"block dz: {block_z - initial_block_z:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Spring length", list(length_history))
        builder.plot_lines("Block z", list(block_z_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Spring", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "anchor": anchor,
            "block": block,
            "springs": springs,
            "ignored_collision_pairs": ignored_collision_pairs,
            "source_demo_row": "avbd-demo3d spring",
            "source_demo_reference": _source_row(),
            "spring_length": spring_length,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_spring",
    title="AVBD Demo3D Spring (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Spring row port with one radial rigid distance spring.",
    build=build,
)
