"""AVBD port of the avbd-demo2d Joint Grid source scene."""

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
_GRID_WIDTH = 25
_GRID_HEIGHT = 25
_GRID_CELLS = _GRID_WIDTH * _GRID_HEIGHT
_STATIC_CORNERS = 2
_DYNAMIC_CELLS = _GRID_CELLS - _STATIC_CORNERS
_HORIZONTAL_JOINTS = (_GRID_WIDTH - 1) * _GRID_HEIGHT
_VERTICAL_JOINTS = _GRID_WIDTH * (_GRID_HEIGHT - 1)
_GRID_JOINTS = _HORIZONTAL_JOINTS + _VERTICAL_JOINTS
_DIAGONAL_IGNORE_COLLISION_PAIRS = 2 * (_GRID_WIDTH - 1) * (_GRID_HEIGHT - 1)
_THICKNESS = 0.2
_CELL_SIZE_2D = np.array([1.0, 1.0])
_CELL_SIZE = np.array([1.0, 1.0, _THICKNESS])
_FRICTION = 0.5
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 15,
    "scene_name": "Joint Grid",
    "scene_count": 19,
    "scene_builder": "sceneJointGrid",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "grid_cell": {
            "count": _GRID_CELLS,
            "size": tuple(_CELL_SIZE_2D),
            "dynamic_density": 1.0,
            "static_corner_density": 0.0,
            "friction": _FRICTION,
            "width": _GRID_WIDTH,
            "height": _GRID_HEIGHT,
            "first_position": (0.0, 0.0, 0.0),
            "static_corner_positions": (
                (0.0, float(_GRID_HEIGHT - 1), 0.0),
                (float(_GRID_WIDTH - 1), float(_GRID_HEIGHT - 1), 0.0),
            ),
        },
    },
    "source_constraints": {
        "all_axis_fixed_joints": {
            "count": _GRID_JOINTS,
            "horizontal_count": _HORIZONTAL_JOINTS,
            "vertical_count": _VERTICAL_JOINTS,
            "horizontal_parent_anchor": (0.5, 0.0),
            "horizontal_child_anchor": (-0.5, 0.0),
            "vertical_parent_anchor": (0.0, 0.5),
            "vertical_child_anchor": (0.0, -0.5),
            "linear_stiffness": "infinity",
            "angular_stiffness": "infinity",
            "fracture": "infinity",
        },
    },
    "source_collision_filters": {
        "diagonal_ignore_collision_pairs": _DIAGONAL_IGNORE_COLLISION_PAIRS,
    },
    "expected_counts": {
        "rigid_bodies": _GRID_CELLS,
        "dynamic_bodies": _DYNAMIC_CELLS,
        "static_bodies": _STATIC_CORNERS,
        "joints": _GRID_JOINTS,
        "fixed_joints": _GRID_JOINTS,
        "collision_shapes": _GRID_CELLS,
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


def _source_cell_position(x: int, y: int) -> np.ndarray:
    return np.array([float(x), float(y), 0.0])


def _is_static_corner(x: int, y: int) -> bool:
    return y == _GRID_HEIGHT - 1 and x in (0, _GRID_WIDTH - 1)


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
    density: float,
    position: np.ndarray,
    is_static: bool = False,
) -> sx.RigidBody:
    body = world.add_rigid_body(name, position=tuple(position))
    body.is_static = is_static
    body.friction = _FRICTION
    body.set_collision_shape(sx.CollisionShape.box(0.5 * _CELL_SIZE))
    if not is_static:
        mass = _box_mass_2d(_CELL_SIZE_2D, density)
        body.mass = mass
        body.inertia = _full_box_inertia(_CELL_SIZE, mass)
    return body


def _max_grid_fixed_pose_error(grid: list[list[sx.RigidBody]]) -> float:
    errors: list[float] = []
    for x in range(1, _GRID_WIDTH):
        for y in range(_GRID_HEIGHT):
            parent = grid[x - 1][y]
            child = grid[x][y]
            errors.append(_fixed_pose_error(parent, child, np.array([1.0, 0.0, 0.0])))
    for x in range(_GRID_WIDTH):
        for y in range(1, _GRID_HEIGHT):
            parent = grid[x][y - 1]
            child = grid[x][y]
            errors.append(_fixed_pose_error(parent, child, np.array([0.0, 1.0, 0.0])))
    return max(errors) if errors else 0.0


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


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    grid: list[list[sx.RigidBody]] = []
    cells: list[sx.RigidBody] = []
    for x in range(_GRID_WIDTH):
        column: list[sx.RigidBody] = []
        for y in range(_GRID_HEIGHT):
            is_static = _is_static_corner(x, y)
            cell = _add_source_box(
                world,
                f"avbd_demo2d_joint_grid_cell_{x:02d}_{y:02d}",
                density=0.0 if is_static else 1.0,
                position=_source_cell_position(x, y),
                is_static=is_static,
            )
            column.append(cell)
            cells.append(cell)
        grid.append(column)

    horizontal_joints = []
    vertical_joints = []
    for x in range(1, _GRID_WIDTH):
        for y in range(_GRID_HEIGHT):
            horizontal_joints.append(
                world.add_joint(
                    grid[x - 1][y],
                    grid[x][y],
                                        sx.JointSpec(
                        name=f"avbd_demo2d_joint_grid_fixed_h_{x - 1:02d}_{y:02d}",
                        type=sx.JointType.FIXED,
                    )
                )
            )
    for x in range(_GRID_WIDTH):
        for y in range(1, _GRID_HEIGHT):
            vertical_joints.append(
                world.add_joint(
                    grid[x][y - 1],
                    grid[x][y],
                                        sx.JointSpec(
                        name=f"avbd_demo2d_joint_grid_fixed_v_{x:02d}_{y - 1:02d}",
                        type=sx.JointType.FIXED,
                    )
                )
            )

    ignored_collision_pairs: list[tuple[sx.RigidBody, sx.RigidBody]] = []
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

    bridge = WorldRenderBridge(world, name="avbd_demo2d_joint_grid_render")
    for x, column in enumerate(grid):
        for y, cell in enumerate(column):
            color = (0.91, 0.47, 0.18) if _is_static_corner(x, y) else (0.24, 0.55, 0.78)
            bridge.add_rigid_body_visual(
                cell,
                dart.BoxShape(_CELL_SIZE),
                color,
                name=f"avbd_demo2d_joint_grid_cell_{x:02d}_{y:02d}_visual",
            )
    bridge.sync()

    center = grid[_GRID_WIDTH // 2][_GRID_HEIGHT // 2]
    center_y_history: deque[float] = deque(maxlen=160)
    pose_error_history: deque[float] = deque(maxlen=160)
    initial_center_y = float(np.asarray(center.translation, dtype=float).reshape(3)[1])
    joints = [*horizontal_joints, *vertical_joints]

    def build_panel(builder: object, context: object) -> None:
        center_y = float(np.asarray(center.translation, dtype=float).reshape(3)[1])
        pose_error = _max_grid_fixed_pose_error(grid)
        center_y_history.append(center_y)
        pose_error_history.append(pose_error)

        builder.text("source corpus: avbd-demo2d Joint Grid")
        builder.text("source scene: sceneJointGrid, index 15 of 19")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"fixed joints: {len(joints)}")
        builder.text(f"ignored collision pairs: {len(ignored_collision_pairs)}")
        builder.text(f"center dy: {center_y - initial_center_y:.3f} m")
        builder.text(f"max fixed pose error: {pose_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Center y", list(center_y_history))
        builder.plot_lines("Fixed pose error", list(pose_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Joint Grid", build_panel)],
        info={
            "sx_world": world,
            "cells": cells,
            "cell_grid": grid,
            "cell_size": np.array(_CELL_SIZE, copy=True),
            "joints": joints,
            "horizontal_joints": horizontal_joints,
            "vertical_joints": vertical_joints,
            "ignored_collision_pairs": ignored_collision_pairs,
            "source_demo_row": "avbd-demo2d joint grid",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_joint_grid",
    title="AVBD Demo2D Joint Grid (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Joint Grid row port with 1200 all-axis fixed joints.",
    build=build,
)
