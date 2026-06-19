"""AVBD port of the avbd-demo2d Rod source scene."""

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
_ROD_LINKS = 20
_ROD_JOINTS = _ROD_LINKS - 1
_THICKNESS = 0.2
_LINK_SIZE_2D = np.array([1.0, 0.5])
_LINK_SIZE = np.array([1.0, 0.5, _THICKNESS])
_FRICTION = 0.5
_CAPTURED_OFFSET = np.array([1.0, 0.0, 0.0])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 13,
    "scene_name": "Rod",
    "scene_count": 19,
    "scene_builder": "sceneRod",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "rod_link": {
            "count": _ROD_LINKS,
            "size": tuple(_LINK_SIZE_2D),
            "dynamic_density": 1.0,
            "static_anchor_density": 0.0,
            "friction": _FRICTION,
            "first_position": (0.0, 10.0, 0.0),
            "x_spacing": 1.0,
        },
    },
    "source_constraints": {
        "all_axis_fixed_joints": {
            "count": _ROD_JOINTS,
            "parent_anchor": (0.5, 0.0),
            "child_anchor": (-0.5, 0.0),
            "linear_stiffness": "infinity",
            "angular_stiffness": "infinity",
            "fracture": "infinity",
        },
    },
    "expected_counts": {
        "rigid_bodies": _ROD_LINKS,
        "dynamic_bodies": _ROD_LINKS - 1,
        "static_bodies": 1,
        "joints": _ROD_JOINTS,
        "fixed_joints": _ROD_JOINTS,
        "collision_shapes": _ROD_LINKS,
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
    return np.array([float(index), 10.0, 0.0])


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
    density: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * _LINK_SIZE))
    if not is_static:
        mass = _box_mass_2d(_LINK_SIZE_2D, density)
        body.mass = mass
        body.inertia = _full_box_inertia(_LINK_SIZE, mass)
    return body


def _max_fixed_joint_pose_error(links: list[sx.RigidBody]) -> float:
    errors = []
    for parent, child in zip(links, links[1:]):
        parent_translation = np.asarray(parent.translation, dtype=float).reshape(3)
        child_translation = np.asarray(child.translation, dtype=float).reshape(3)
        parent_rotation = np.asarray(parent.rotation, dtype=float).reshape(3, 3)
        child_rotation = np.asarray(child.rotation, dtype=float).reshape(3, 3)
        translation_error = np.linalg.norm(
            child_translation - parent_translation - parent_rotation @ _CAPTURED_OFFSET
        )
        rotation_error = 0.25 * np.linalg.norm(child_rotation - parent_rotation)
        errors.append(float(translation_error + rotation_error))
    return max(errors) if errors else 0.0


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    links: list[sx.RigidBody] = []
    for index in range(_ROD_LINKS):
        links.append(
            _add_source_box(
                world,
                f"avbd_demo2d_rod_link_{index:02d}",
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
                    name=f"avbd_demo2d_rod_fixed_joint_{index:02d}",
                    type=sx.JointType.FIXED,
                )
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_rod_render")
    for index, link in enumerate(links):
        bridge.add_rigid_body_visual(
            link,
            dart.BoxShape(_LINK_SIZE),
            (0.91, 0.47, 0.18) if index == 0 else (0.20, 0.56, 0.80),
            name=f"avbd_demo2d_rod_link_{index:02d}_visual",
        )
    bridge.sync()

    tail_y_history: deque[float] = deque(maxlen=160)
    pose_error_history: deque[float] = deque(maxlen=160)
    tail = links[-1]
    initial_tail_y = float(np.asarray(tail.translation, dtype=float).reshape(3)[1])

    def build_panel(builder: object, context: object) -> None:
        tail_y = float(np.asarray(tail.translation, dtype=float).reshape(3)[1])
        pose_error = _max_fixed_joint_pose_error(links)
        tail_y_history.append(tail_y)
        pose_error_history.append(pose_error)

        builder.text("source corpus: avbd-demo2d Rod")
        builder.text("source scene: sceneRod, index 13 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"fixed joints: {len(joints)}")
        builder.text(f"tail dy: {tail_y - initial_tail_y:.3f} m")
        builder.text(f"max fixed pose error: {pose_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Tail y", list(tail_y_history))
        builder.plot_lines("Fixed pose error", list(pose_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Rod", build_panel)],
        info={
            "sx_world": world,
            "links": links,
            "link_sizes": [np.array(_LINK_SIZE, copy=True) for _ in links],
            "joints": joints,
            "source_demo_row": "avbd-demo2d rod",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_rod",
    title="AVBD Demo2D Rod (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Rod row port with all-axis public fixed joints.",
    build=build,
)
