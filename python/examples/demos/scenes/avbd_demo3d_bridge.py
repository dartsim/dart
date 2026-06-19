"""AVBD port of the avbd-demo3d Bridge source scene."""

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
_PLANKS = 40
_LOAD_X = _PLANKS // 4
_LOAD_Z = _PLANKS // 8
_LOAD_BOXES = _LOAD_X * _LOAD_Z
_PLANK_JOINTS = (_PLANKS - 1) * 2
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_PLANK_SIZE = np.array([1.0, 4.0, 0.5])
_LOAD_BOX_SIZE = np.array([1.0, 1.0, 1.0])
_FRICTION = 0.5
_HALF_LENGTH = 0.5
_HALF_WIDTH = 2.0
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 12,
    "scene_name": "Bridge",
    "scene_count": 14,
    "scene_builder": "sceneBridge",
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
            "position": (0.0, 0.0, 0.0),
        },
        "bridge_plank": {
            "count": _PLANKS,
            "size": tuple(_PLANK_SIZE),
            "dynamic_density": 1.0,
            "static_endpoint_density": 0.0,
            "friction": _FRICTION,
            "first_position": (-20.0, 0.0, 10.0),
            "x_spacing": 1.0,
            "static_indices": (0, _PLANKS - 1),
        },
        "load_box": {
            "count": _LOAD_BOXES,
            "size": tuple(_LOAD_BOX_SIZE),
            "density": 1.0,
            "friction": _FRICTION,
            "x_range": (-5.0, 4.0),
            "z_range": (12.0, 16.0),
        },
    },
    "source_constraints": {
        "linear_point_joints": {
            "count": _PLANK_JOINTS,
            "pairs": _PLANKS - 1,
            "parent_anchors": (
                (_HALF_LENGTH, _HALF_WIDTH, 0.0),
                (_HALF_LENGTH, -_HALF_WIDTH, 0.0),
            ),
            "child_anchors": (
                (-_HALF_LENGTH, _HALF_WIDTH, 0.0),
                (-_HALF_LENGTH, -_HALF_WIDTH, 0.0),
            ),
            "linear_stiffness": "infinity",
            "angular_stiffness": 0.0,
            "fracture": "infinity",
        },
    },
    "expected_counts": {
        "rigid_bodies": 91,
        "dynamic_bodies": 88,
        "static_bodies": 3,
        "joints": _PLANK_JOINTS,
        "linear_point_joints": _PLANK_JOINTS,
        "collision_shapes": 91,
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
        "avbd_demo3d_bridge_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    planks: list[sx.RigidBody] = []
    for index in range(_PLANKS):
        planks.append(
            _add_source_box(
                world,
                f"avbd_demo3d_bridge_plank_{index:02d}",
                size=_PLANK_SIZE,
                density=0.0 if index in (0, _PLANKS - 1) else 1.0,
                position=np.array([float(index) - _PLANKS * 0.5, 0.0, 10.0]),
                is_static=index in (0, _PLANKS - 1),
            )
        )

    joints = []
    parent_anchors = (
        np.array([_HALF_LENGTH, _HALF_WIDTH, 0.0]),
        np.array([_HALF_LENGTH, -_HALF_WIDTH, 0.0]),
    )
    child_anchors = (
        np.array([-_HALF_LENGTH, _HALF_WIDTH, 0.0]),
        np.array([-_HALF_LENGTH, -_HALF_WIDTH, 0.0]),
    )
    for pair_index, (parent, child) in enumerate(zip(planks, planks[1:])):
        for side_index, (parent_anchor, child_anchor) in enumerate(
            zip(parent_anchors, child_anchors)
        ):
            joints.append(
                world.add_joint(
                    parent,
                    child,
                                        sx.JointSpec(
                        name=f"avbd_demo3d_bridge_joint_{pair_index:02d}_{side_index}",
                        type=sx.JointType.SPHERICAL,
                        parent_anchor=tuple(parent_anchor),
                        child_anchor=tuple(child_anchor),
                    )
                )
            )

    load_boxes: list[sx.RigidBody] = []
    for x in range(_LOAD_X):
        for z in range(_LOAD_Z):
            load_boxes.append(
                _add_source_box(
                    world,
                    f"avbd_demo3d_bridge_load_{x:02d}_{z:02d}",
                    size=_LOAD_BOX_SIZE,
                    density=1.0,
                    position=np.array([float(x) - _PLANKS / 8.0, 0.0, float(z) + 12.0]),
                )
            )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_bridge_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_bridge_ground_visual",
    )
    for index, plank in enumerate(planks):
        bridge.add_rigid_body_visual(
            plank,
            dart.BoxShape(_PLANK_SIZE),
            (0.91, 0.47, 0.18) if plank.is_static else (0.25, 0.56, 0.72),
            name=f"avbd_demo3d_bridge_plank_{index:02d}_visual",
        )
    for index, box in enumerate(load_boxes):
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_LOAD_BOX_SIZE),
            (0.58, 0.33, 0.65),
            name=f"avbd_demo3d_bridge_load_{index:02d}_visual",
        )
    bridge.sync()

    load_z_history: deque[float] = deque(maxlen=160)
    endpoint_error_history: deque[float] = deque(maxlen=160)
    initial_mean_load_z = float(
        np.mean([np.asarray(box.translation, dtype=float).reshape(3)[2] for box in load_boxes])
    )

    def max_endpoint_error() -> float:
        errors = []
        for parent, child in zip(planks, planks[1:]):
            for parent_anchor, child_anchor in zip(parent_anchors, child_anchors):
                errors.append(
                    float(
                        np.linalg.norm(
                            _anchor_world_position(parent, parent_anchor)
                            - _anchor_world_position(child, child_anchor)
                        )
                    )
                )
        return max(errors)

    def mean_load_z() -> float:
        return float(
            np.mean(
                [
                    np.asarray(box.translation, dtype=float).reshape(3)[2]
                    for box in load_boxes
                ]
            )
        )

    def build_panel(builder: object, context: object) -> None:
        current_mean_load_z = mean_load_z()
        endpoint_error = max_endpoint_error()
        load_z_history.append(current_mean_load_z)
        endpoint_error_history.append(endpoint_error)

        builder.text("source corpus: avbd-demo3d Bridge")
        builder.text("source scene: sceneBridge, index 12 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"linear point joints: {len(joints)}")
        builder.text(f"load boxes: {len(load_boxes)}")
        builder.text(f"mean load dz: {current_mean_load_z - initial_mean_load_z:.3f} m")
        builder.text(f"max endpoint error: {endpoint_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Mean load z", list(load_z_history))
        builder.plot_lines("Endpoint error", list(endpoint_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Bridge", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "planks": planks,
            "load_boxes": load_boxes,
            "joints": joints,
            "source_demo_row": "avbd-demo3d bridge",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_bridge",
    title="AVBD Demo3D Bridge (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Bridge row port with paired point joints and load boxes.",
    build=build,
)
