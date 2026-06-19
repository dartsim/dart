"""AVBD port of the avbd-demo2d Heavy Rope source scene."""

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
_THICKNESS = 0.2
_LINK_SIZE_2D = np.array([1.0, 0.5])
_LINK_SIZE = np.array([1.0, 0.5, _THICKNESS])
_HEAVY_SIZE = 30.0
_HEAVY_BLOCK_SIZE_2D = np.array([_HEAVY_SIZE, _HEAVY_SIZE])
_HEAVY_BLOCK_SIZE = np.array([_HEAVY_SIZE, _HEAVY_SIZE, _THICKNESS])
_FRICTION = 0.5
_PARENT_ANCHOR = np.array([0.5, 0.0, 0.0])
_REGULAR_CHILD_ANCHOR = np.array([-0.5, 0.0, 0.0])
_HEAVY_CHILD_ANCHOR = np.array([-_HEAVY_SIZE * 0.5, 0.0, 0.0])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 7,
    "scene_name": "Heavy Rope",
    "scene_count": 19,
    "scene_builder": "sceneHeavyRope",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "rope_link": {
            "count": _ROPE_LINKS - 1,
            "size": tuple(_LINK_SIZE_2D),
            "dynamic_density": 1.0,
            "static_anchor_density": 0.0,
            "friction": _FRICTION,
            "first_position": (0.0, 10.0, 0.0),
            "x_spacing": 1.0,
        },
        "heavy_block": {
            "count": 1,
            "size": tuple(_HEAVY_BLOCK_SIZE_2D),
            "density": 1.0,
            "friction": _FRICTION,
            "position": (19.0 + _HEAVY_SIZE * 0.5, 10.0, 0.0),
        },
    },
    "source_constraints": {
        "linear_point_joints": {
            "count": _ROPE_JOINTS,
            "parent_anchor": (0.5, 0.0),
            "regular_child_anchor": (-0.5, 0.0),
            "heavy_child_anchor": (-_HEAVY_SIZE * 0.5, 0.0),
            "source_initial_last_endpoint_gap": 0.5,
            "linear_stiffness": "infinity",
            "angular_stiffness": 0.0,
            "fracture": "infinity",
        },
    },
    "expected_counts": {
        "rigid_bodies": _ROPE_LINKS,
        "dynamic_bodies": _ROPE_LINKS - 1,
        "static_bodies": 1,
        "joints": _ROPE_JOINTS,
        "linear_point_joints": _ROPE_JOINTS,
        "collision_shapes": _ROPE_LINKS,
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


def _source_link_size_2d(index: int) -> np.ndarray:
    return _HEAVY_BLOCK_SIZE_2D if index == _ROPE_LINKS - 1 else _LINK_SIZE_2D


def _source_link_size(index: int) -> np.ndarray:
    return _HEAVY_BLOCK_SIZE if index == _ROPE_LINKS - 1 else _LINK_SIZE


def _source_link_position(index: int) -> np.ndarray:
    x = float(index)
    if index == _ROPE_LINKS - 1:
        x += _HEAVY_SIZE * 0.5
    return np.array([x, 10.0, 0.0])


def _child_anchor(index: int) -> np.ndarray:
    return _HEAVY_CHILD_ANCHOR if index == _ROPE_LINKS - 2 else _REGULAR_CHILD_ANCHOR


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


def _anchor_world_position(body: sx.RigidBody, local_anchor: np.ndarray) -> np.ndarray:
    rotation = np.asarray(body.rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(body.translation, dtype=float).reshape(3)
    return translation + rotation @ local_anchor


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    links: list[sx.RigidBody] = []
    link_sizes: list[np.ndarray] = []
    for index in range(_ROPE_LINKS):
        size_2d = _source_link_size_2d(index)
        size = _source_link_size(index)
        link_sizes.append(size)
        links.append(
            _add_source_box(
                world,
                f"avbd_demo2d_heavy_rope_link_{index:02d}",
                size_2d=size_2d,
                size=size,
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
                    name=f"avbd_demo2d_heavy_rope_point_joint_{index:02d}",
                    type=sx.JointType.SPHERICAL,
                    parent_anchor=tuple(_PARENT_ANCHOR),
                    child_anchor=tuple(_child_anchor(index)),
                )
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_heavy_rope_render")
    for index, (link, size) in enumerate(zip(links, link_sizes)):
        if index == 0:
            color = (0.91, 0.47, 0.18)
        elif index == _ROPE_LINKS - 1:
            color = (0.64, 0.22, 0.25)
        else:
            color = (0.22, 0.58, 0.76)
        bridge.add_rigid_body_visual(
            link,
            dart.BoxShape(size),
            color,
            name=f"avbd_demo2d_heavy_rope_link_{index:02d}_visual",
        )
    bridge.sync()

    heavy_y_history: deque[float] = deque(maxlen=160)
    endpoint_error_history: deque[float] = deque(maxlen=160)
    heavy_block = links[-1]
    initial_heavy_y = float(
        np.asarray(heavy_block.translation, dtype=float).reshape(3)[1]
    )

    def max_endpoint_error() -> float:
        return max(
            float(
                np.linalg.norm(
                    _anchor_world_position(parent, _PARENT_ANCHOR)
                    - _anchor_world_position(child, _child_anchor(index))
                )
            )
            for index, (parent, child) in enumerate(zip(links, links[1:]))
        )

    def build_panel(builder: object, context: object) -> None:
        heavy_y = float(np.asarray(heavy_block.translation, dtype=float).reshape(3)[1])
        endpoint_error = max_endpoint_error()
        heavy_y_history.append(heavy_y)
        endpoint_error_history.append(endpoint_error)

        builder.text("source corpus: avbd-demo2d Heavy Rope")
        builder.text("source scene: sceneHeavyRope, index 7 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"linear point joints: {len(joints)}")
        builder.text(f"heavy block dy: {heavy_y - initial_heavy_y:.3f} m")
        builder.text(f"max endpoint error: {endpoint_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Heavy block y", list(heavy_y_history))
        builder.plot_lines("Endpoint error", list(endpoint_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Heavy Rope", build_panel)],
        info={
            "sx_world": world,
            "links": links,
            "link_sizes": link_sizes,
            "joints": joints,
            "source_demo_row": "avbd-demo2d heavy rope",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_heavy_rope",
    title="AVBD Demo2D Heavy Rope (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Heavy Rope row port with a high-ratio endpoint block.",
    build=build,
)
