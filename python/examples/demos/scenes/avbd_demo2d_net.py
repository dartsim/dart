"""AVBD port of the avbd-demo2d Net source scene."""

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
_NET_LINKS = 40
_NET_JOINTS = _NET_LINKS - 1
_FALLING_COLUMNS = _NET_LINKS // 4
_FALLING_ROWS = _NET_LINKS // 8
_FALLING_BLOCKS = _FALLING_COLUMNS * _FALLING_ROWS
_THICKNESS = 0.2
_GROUND_SIZE_2D = np.array([100.0, 0.5])
_GROUND_SIZE = np.array([100.0, 0.5, _THICKNESS])
_LINK_SIZE_2D = np.array([1.0, 0.5])
_LINK_SIZE = np.array([1.0, 0.5, _THICKNESS])
_BLOCK_SIZE_2D = np.array([1.0, 1.0])
_BLOCK_SIZE = np.array([1.0, 1.0, _THICKNESS])
_FRICTION = 0.5
_PARENT_ANCHOR = np.array([0.5, 0.0, 0.0])
_CHILD_ANCHOR = np.array([-0.5, 0.0, 0.0])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 16,
    "scene_name": "Net",
    "scene_count": 19,
    "scene_builder": "sceneNet",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "count": 1,
            "size": tuple(_GROUND_SIZE_2D),
            "density": 0.0,
            "friction": _FRICTION,
            "position": (0.0, 0.0, 0.0),
        },
        "net_link": {
            "count": _NET_LINKS,
            "size": tuple(_LINK_SIZE_2D),
            "dynamic_density": 1.0,
            "static_endpoint_density": 0.0,
            "friction": _FRICTION,
            "first_position": (-_NET_LINKS / 2.0, 10.0, 0.0),
            "x_spacing": 1.0,
        },
        "falling_block": {
            "count": _FALLING_BLOCKS,
            "size": tuple(_BLOCK_SIZE_2D),
            "density": 1.0,
            "friction": _FRICTION,
            "first_position": (-_NET_LINKS / 8.0, 15.0, 0.0),
            "columns": _FALLING_COLUMNS,
            "rows": _FALLING_ROWS,
            "x_spacing": 1.0,
            "y_spacing": 1.0,
        },
    },
    "source_constraints": {
        "linear_point_joints": {
            "count": _NET_JOINTS,
            "parent_anchor": (0.5, 0.0),
            "child_anchor": (-0.5, 0.0),
            "linear_stiffness": "infinity",
            "angular_stiffness": 0.0,
            "fracture": "infinity",
        },
    },
    "expected_counts": {
        "rigid_bodies": 1 + _NET_LINKS + _FALLING_BLOCKS,
        "dynamic_bodies": (_NET_LINKS - 2) + _FALLING_BLOCKS,
        "static_bodies": 3,
        "joints": _NET_JOINTS,
        "linear_point_joints": _NET_JOINTS,
        "collision_shapes": 1 + _NET_LINKS + _FALLING_BLOCKS,
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


def _source_link_position(index: int) -> np.ndarray:
    return np.array([float(index) - _NET_LINKS / 2.0, 10.0, 0.0])


def _source_block_position(x: int, y: int) -> np.ndarray:
    return np.array([float(x) - _NET_LINKS / 8.0, float(y) + 15.0, 0.0])


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

    ground = _add_source_box(
        world,
        "avbd_demo2d_net_ground",
        size_2d=_GROUND_SIZE_2D,
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    links: list[sx.RigidBody] = []
    for index in range(_NET_LINKS):
        is_static = index == 0 or index == _NET_LINKS - 1
        links.append(
            _add_source_box(
                world,
                f"avbd_demo2d_net_link_{index:02d}",
                size_2d=_LINK_SIZE_2D,
                size=_LINK_SIZE,
                density=0.0 if is_static else 1.0,
                position=_source_link_position(index),
                is_static=is_static,
            )
        )

    joints = []
    for index, (parent, child) in enumerate(zip(links, links[1:])):
        joints.append(
            world.add_joint(
                parent,
                child,
                                sx.JointSpec(
                    name=f"avbd_demo2d_net_point_joint_{index:02d}",
                    type=sx.JointType.SPHERICAL,
                    parent_anchor=tuple(_PARENT_ANCHOR),
                    child_anchor=tuple(_CHILD_ANCHOR),
                )
            )
        )

    falling_blocks: list[sx.RigidBody] = []
    for x in range(_FALLING_COLUMNS):
        for y in range(_FALLING_ROWS):
            falling_blocks.append(
                _add_source_box(
                    world,
                    f"avbd_demo2d_net_falling_block_{x:02d}_{y:02d}",
                    size_2d=_BLOCK_SIZE_2D,
                    size=_BLOCK_SIZE,
                    density=1.0,
                    position=_source_block_position(x, y),
                )
            )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_net_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.36, 0.38, 0.42),
        name="avbd_demo2d_net_ground_visual",
    )
    for index, link in enumerate(links):
        color = (0.91, 0.47, 0.18) if link.is_static else (0.22, 0.58, 0.76)
        bridge.add_rigid_body_visual(
            link,
            dart.BoxShape(_LINK_SIZE),
            color,
            name=f"avbd_demo2d_net_link_{index:02d}_visual",
        )
    for index, block in enumerate(falling_blocks):
        bridge.add_rigid_body_visual(
            block,
            dart.BoxShape(_BLOCK_SIZE),
            (0.58, 0.56, 0.20),
            name=f"avbd_demo2d_net_falling_block_{index:02d}_visual",
        )
    bridge.sync()

    lowest_block_history: deque[float] = deque(maxlen=160)
    endpoint_error_history: deque[float] = deque(maxlen=160)
    initial_lowest_block_y = min(
        float(np.asarray(block.translation, dtype=float).reshape(3)[1])
        for block in falling_blocks
    )

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

    def lowest_block_y() -> float:
        return min(
            float(np.asarray(block.translation, dtype=float).reshape(3)[1])
            for block in falling_blocks
        )

    def build_panel(builder: object, context: object) -> None:
        low_y = lowest_block_y()
        endpoint_error = max_endpoint_error()
        lowest_block_history.append(low_y)
        endpoint_error_history.append(endpoint_error)

        builder.text("source corpus: avbd-demo2d Net")
        builder.text("source scene: sceneNet, index 16 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"linear point joints: {len(joints)}")
        builder.text(f"falling blocks: {len(falling_blocks)}")
        builder.text(f"lowest block dy: {low_y - initial_lowest_block_y:.3f} m")
        builder.text(f"max endpoint error: {endpoint_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Lowest block y", list(lowest_block_history))
        builder.plot_lines("Endpoint error", list(endpoint_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Net", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "links": links,
            "joints": joints,
            "falling_blocks": falling_blocks,
            "source_demo_row": "avbd-demo2d net",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_net",
    title="AVBD Demo2D Net (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Net row port with endpoint-pinned point joints and falling blocks.",
    build=build,
)
