"""AVBD port of the avbd-demo3d Rope source scene."""

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
_ROPE_LINKS = 20
_ROPE_JOINTS = _ROPE_LINKS - 1
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_LINK_SIZE = np.array([1.0, 0.5, 0.5])
_FRICTION = 0.5
_PARENT_ANCHOR = np.array([0.5, 0.0, 0.0])
_CHILD_ANCHOR = np.array([-0.5, 0.0, 0.0])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 5,
    "scene_name": "Rope",
    "scene_count": 14,
    "scene_builder": "sceneRope",
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
            "position": (0.0, 0.0, -20.0),
        },
        "rope_link": {
            "count": _ROPE_LINKS,
            "size": tuple(_LINK_SIZE),
            "dynamic_density": 1.0,
            "static_anchor_density": 0.0,
            "friction": _FRICTION,
            "first_position": (0.0, 0.0, 10.0),
            "x_spacing": 1.0,
        },
    },
    "source_constraints": {
        "linear_point_joints": {
            "count": _ROPE_JOINTS,
            "parent_anchor": tuple(_PARENT_ANCHOR),
            "child_anchor": tuple(_CHILD_ANCHOR),
            "linear_stiffness": "infinity",
            "angular_stiffness": 0.0,
            "fracture": "infinity",
        },
    },
    "expected_counts": {
        "rigid_bodies": 21,
        "dynamic_bodies": 19,
        "static_bodies": 2,
        "joints": _ROPE_JOINTS,
        "linear_point_joints": _ROPE_JOINTS,
        "collision_shapes": 21,
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


def _source_link_position(index: int) -> np.ndarray:
    return np.array([float(index), 0.0, 10.0])


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
        "avbd_demo3d_rope_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=np.array([0.0, 0.0, -20.0]),
        is_static=True,
    )

    links: list[sx.RigidBody] = []
    for index in range(_ROPE_LINKS):
        links.append(
            _add_source_box(
                world,
                f"avbd_demo3d_rope_link_{index:02d}",
                size=_LINK_SIZE,
                density=0.0 if index == 0 else 1.0,
                position=_source_link_position(index),
                is_static=index == 0,
            )
        )

    joints = []
    for index, (parent, child) in enumerate(zip(links, links[1:])):
        joints.append(
            world.add_joint(
                parent,
                child,
                                sx.JointSpec(
                    name=f"avbd_demo3d_rope_point_joint_{index:02d}",
                    type=sx.JointType.SPHERICAL,
                    parent_anchor=tuple(_PARENT_ANCHOR),
                    child_anchor=tuple(_CHILD_ANCHOR),
                )
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_rope_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_rope_ground_visual",
    )
    for index, link in enumerate(links):
        is_anchor = index == 0
        bridge.add_rigid_body_visual(
            link,
            dart.BoxShape(_LINK_SIZE),
            (0.91, 0.47, 0.18) if is_anchor else (0.23, 0.58, 0.78),
            name=f"avbd_demo3d_rope_link_{index:02d}_visual",
        )
    bridge.sync()

    tail_z_history: deque[float] = deque(maxlen=160)
    endpoint_error_history: deque[float] = deque(maxlen=160)
    tail = links[-1]
    initial_tail_z = float(np.asarray(tail.translation, dtype=float).reshape(3)[2])

    def max_endpoint_error() -> float:
        return max(
            float(
                np.linalg.norm(
                    _anchor_world_position(parent, _PARENT_ANCHOR)
                    - _anchor_world_position(child, _CHILD_ANCHOR)
                )
            )
            for parent, child in zip(links, links[1:])
        )

    def build_panel(builder: object, context: object) -> None:
        tail_z = float(np.asarray(tail.translation, dtype=float).reshape(3)[2])
        endpoint_error = max_endpoint_error()
        tail_z_history.append(tail_z)
        endpoint_error_history.append(endpoint_error)

        builder.text("source corpus: avbd-demo3d Rope")
        builder.text("source scene: sceneRope, index 5 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"linear point joints: {len(joints)}")
        builder.text(f"tail dz: {tail_z - initial_tail_z:.3f} m")
        builder.text(f"max endpoint error: {endpoint_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Tail z", list(tail_z_history))
        builder.plot_lines("Endpoint error", list(endpoint_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Rope", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "links": links,
            "joints": joints,
            "source_demo_row": "avbd-demo3d rope",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_rope",
    title="AVBD Demo3D Rope (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Rope row port with anchored linear-only point joints.",
    build=build,
)
