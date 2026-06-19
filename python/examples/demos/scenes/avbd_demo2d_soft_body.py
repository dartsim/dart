"""AVBD port of the avbd-demo2d Soft Body source scene."""

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
_GRID_WIDTH = 15
_GRID_HEIGHT = 5
_GRID_STACKS = 2
_GRID_CELLS_PER_STACK = _GRID_WIDTH * _GRID_HEIGHT
_GRID_CELLS = _GRID_STACKS * _GRID_CELLS_PER_STACK
_HORIZONTAL_JOINTS_PER_STACK = (_GRID_WIDTH - 1) * _GRID_HEIGHT
_VERTICAL_JOINTS_PER_STACK = _GRID_WIDTH * (_GRID_HEIGHT - 1)
_GRID_JOINTS_PER_STACK = _HORIZONTAL_JOINTS_PER_STACK + _VERTICAL_JOINTS_PER_STACK
_GRID_JOINTS = _GRID_STACKS * _GRID_JOINTS_PER_STACK
_DIAGONAL_IGNORE_COLLISION_PAIRS = (
    _GRID_STACKS * 2 * (_GRID_WIDTH - 1) * (_GRID_HEIGHT - 1)
)
_THICKNESS = 0.2
_GROUND_SIZE_2D = np.array([100.0, 0.5])
_GROUND_SIZE = np.array([100.0, 0.5, _THICKNESS])
_CELL_SIZE_2D = np.array([1.0, 1.0])
_CELL_SIZE = np.array([1.0, 1.0, _THICKNESS])
_FRICTION = 0.5
_LINEAR_STIFFNESS = 1000.0
_ANGULAR_STIFFNESS = 100.0
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 14,
    "scene_name": "Soft Body",
    "scene_count": 19,
    "scene_builder": "sceneSoftBody",
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
        "soft_cell": {
            "count": _GRID_CELLS,
            "size": tuple(_CELL_SIZE_2D),
            "density": 1.0,
            "friction": _FRICTION,
            "width": _GRID_WIDTH,
            "height": _GRID_HEIGHT,
            "stacks": _GRID_STACKS,
            "first_position": (0.0, 5.0, 0.0),
            "stack_y_spacing": float(_GRID_HEIGHT * 2),
        },
    },
    "source_constraints": {
        "finite_all_axis_fixed_joints": {
            "count": _GRID_JOINTS,
            "horizontal_count": _GRID_STACKS * _HORIZONTAL_JOINTS_PER_STACK,
            "vertical_count": _GRID_STACKS * _VERTICAL_JOINTS_PER_STACK,
            "horizontal_parent_anchor": (0.5, 0.0),
            "horizontal_child_anchor": (-0.5, 0.0),
            "vertical_parent_anchor": (0.0, 0.5),
            "vertical_child_anchor": (0.0, -0.5),
            "linear_stiffness": _LINEAR_STIFFNESS,
            "angular_stiffness": _ANGULAR_STIFFNESS,
            "fracture": "infinity",
        },
    },
    "source_collision_filters": {
        "diagonal_ignore_collision_pairs": _DIAGONAL_IGNORE_COLLISION_PAIRS,
    },
    "expected_counts": {
        "rigid_bodies": 1 + _GRID_CELLS,
        "dynamic_bodies": _GRID_CELLS,
        "static_bodies": 1,
        "joints": _GRID_JOINTS,
        "fixed_joints": _GRID_JOINTS,
        "finite_stiffness_fixed_joints": _GRID_JOINTS,
        "collision_shapes": 1 + _GRID_CELLS,
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


def _source_cell_position(stack: int, x: int, y: int) -> np.ndarray:
    return np.array([float(x), float(y) + _GRID_HEIGHT * stack * 2.0 + 5.0, 0.0])


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
        "source_collision_filters": dict(_SOURCE_ROW["source_collision_filters"]),
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


def _configure_finite_joint(joint: sx.Joint) -> sx.Joint:
    policy = joint.constraint_projection_policy
    policy.linear_stiffness = _LINEAR_STIFFNESS
    joint.constraint_projection_policy = policy
    policy = joint.constraint_projection_policy
    policy.angular_stiffness = _ANGULAR_STIFFNESS
    joint.constraint_projection_policy = policy
    return joint


def _fixed_pose_error(
    parent: sx.RigidBody,
    child: sx.RigidBody,
    captured_offset: np.ndarray,
) -> float:
    parent_translation = np.asarray(parent.translation, dtype=float).reshape(3)
    child_translation = np.asarray(child.translation, dtype=float).reshape(3)
    parent_rotation = np.asarray(parent.rotation, dtype=float).reshape(3, 3)
    child_rotation = np.asarray(child.rotation, dtype=float).reshape(3, 3)
    translation_error = np.linalg.norm(
        child_translation - parent_translation - parent_rotation @ captured_offset
    )
    rotation_error = 0.25 * np.linalg.norm(child_rotation - parent_rotation)
    return float(translation_error + rotation_error)


def _max_soft_body_pose_error(
    grids: list[list[list[sx.RigidBody]]],
) -> float:
    errors: list[float] = []
    for grid in grids:
        for x in range(1, _GRID_WIDTH):
            for y in range(_GRID_HEIGHT):
                errors.append(
                    _fixed_pose_error(
                        grid[x - 1][y], grid[x][y], np.array([1.0, 0.0, 0.0])
                    )
                )
        for x in range(_GRID_WIDTH):
            for y in range(1, _GRID_HEIGHT):
                errors.append(
                    _fixed_pose_error(
                        grid[x][y - 1], grid[x][y], np.array([0.0, 1.0, 0.0])
                    )
                )
    return max(errors) if errors else 0.0


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = _add_source_box(
        world,
        "avbd_demo2d_soft_body_ground",
        size_2d=_GROUND_SIZE_2D,
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    grids: list[list[list[sx.RigidBody]]] = []
    cells: list[sx.RigidBody] = []
    for stack in range(_GRID_STACKS):
        grid: list[list[sx.RigidBody]] = []
        for x in range(_GRID_WIDTH):
            column: list[sx.RigidBody] = []
            for y in range(_GRID_HEIGHT):
                cell = _add_source_box(
                    world,
                    f"avbd_demo2d_soft_body_cell_{stack:02d}_{x:02d}_{y:02d}",
                    size_2d=_CELL_SIZE_2D,
                    size=_CELL_SIZE,
                    density=1.0,
                    position=_source_cell_position(stack, x, y),
                )
                column.append(cell)
                cells.append(cell)
            grid.append(column)
        grids.append(grid)

    horizontal_joints = []
    vertical_joints = []
    for stack, grid in enumerate(grids):
        for x in range(1, _GRID_WIDTH):
            for y in range(_GRID_HEIGHT):
                horizontal_joints.append(
                    _configure_finite_joint(
                        world.add_joint(
                            grid[x - 1][y],
                            grid[x][y],
                                                        sx.JointSpec(
                                name=f"avbd_demo2d_soft_body_fixed_h_"
                            f"{stack:02d}_{x - 1:02d}_{y:02d}",
                                type=sx.JointType.FIXED,
                            )
                        )
                    )
                )
        for x in range(_GRID_WIDTH):
            for y in range(1, _GRID_HEIGHT):
                vertical_joints.append(
                    _configure_finite_joint(
                        world.add_joint(
                            grid[x][y - 1],
                            grid[x][y],
                                                        sx.JointSpec(
                                name=f"avbd_demo2d_soft_body_fixed_v_"
                            f"{stack:02d}_{x:02d}_{y - 1:02d}",
                                type=sx.JointType.FIXED,
                            )
                        )
                    )
                )

    ignored_collision_pairs: list[tuple[sx.RigidBody, sx.RigidBody]] = []
    for grid in grids:
        for x in range(_GRID_WIDTH - 1):
            for y in range(_GRID_HEIGHT - 1):
                diagonal_pairs = (
                    (grid[x][y], grid[x + 1][y + 1]),
                    (grid[x + 1][y], grid[x][y + 1]),
                )
                for first, second in diagonal_pairs:
                    world.set_collision_pair_ignored(first, second)
                    ignored_collision_pairs.append((first, second))

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_soft_body_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo2d_soft_body_ground_visual",
    )
    palette = ((0.24, 0.55, 0.78), (0.86, 0.53, 0.20))
    for stack, grid in enumerate(grids):
        for x, column in enumerate(grid):
            for y, cell in enumerate(column):
                bridge.add_rigid_body_visual(
                    cell,
                    dart.BoxShape(_CELL_SIZE),
                    palette[stack % len(palette)],
                    name=(
                        f"avbd_demo2d_soft_body_cell_"
                        f"{stack:02d}_{x:02d}_{y:02d}_visual"
                    ),
                )
    bridge.sync()

    center = grids[0][_GRID_WIDTH // 2][_GRID_HEIGHT // 2]
    center_y_history: deque[float] = deque(maxlen=160)
    pose_error_history: deque[float] = deque(maxlen=160)
    initial_center_y = float(np.asarray(center.translation, dtype=float).reshape(3)[1])
    joints = [*horizontal_joints, *vertical_joints]

    def build_panel(builder: object, context: object) -> None:
        center_y = float(np.asarray(center.translation, dtype=float).reshape(3)[1])
        pose_error = _max_soft_body_pose_error(grids)
        center_y_history.append(center_y)
        pose_error_history.append(pose_error)

        builder.text("source corpus: avbd-demo2d Soft Body")
        builder.text("source scene: sceneSoftBody, index 14 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"finite fixed joints: {len(joints)}")
        builder.text(f"ignored collision pairs: {len(ignored_collision_pairs)}")
        builder.text(f"center dy: {center_y - initial_center_y:.3f} m")
        builder.text(f"max finite pose error: {pose_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Center y", list(center_y_history))
        builder.plot_lines("Finite pose error", list(pose_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Soft Body", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "cells": cells,
            "cell_grids": grids,
            "cell_size": np.array(_CELL_SIZE, copy=True),
            "joints": joints,
            "horizontal_joints": horizontal_joints,
            "vertical_joints": vertical_joints,
            "ignored_collision_pairs": ignored_collision_pairs,
            "source_demo_row": "avbd-demo2d soft body",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_soft_body",
    title="AVBD Demo2D Soft Body (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Soft Body row port with 260 finite-stiffness joints.",
    build=build,
)
