"""AVBD port of the avbd-demo3d Soft Body source scene."""

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
_GRID_WIDTH = 4
_GRID_DEPTH = 4
_GRID_HEIGHT = 4
_GRID_STACKS = 3
_CELLS_PER_STACK = _GRID_WIDTH * _GRID_DEPTH * _GRID_HEIGHT
_GRID_CELLS = _GRID_STACKS * _CELLS_PER_STACK
_X_JOINTS_PER_STACK = (_GRID_WIDTH - 1) * _GRID_DEPTH * _GRID_HEIGHT
_Y_JOINTS_PER_STACK = _GRID_WIDTH * (_GRID_DEPTH - 1) * _GRID_HEIGHT
_Z_JOINTS_PER_STACK = _GRID_WIDTH * _GRID_DEPTH * (_GRID_HEIGHT - 1)
_JOINTS_PER_STACK = _X_JOINTS_PER_STACK + _Y_JOINTS_PER_STACK + _Z_JOINTS_PER_STACK
_GRID_JOINTS = _GRID_STACKS * _JOINTS_PER_STACK
_DIAGONAL_IGNORE_COLLISION_PAIRS = _GRID_STACKS * (
    2 * (_GRID_WIDTH - 1) * _GRID_DEPTH * (_GRID_HEIGHT - 1)
    + 2 * _GRID_WIDTH * (_GRID_DEPTH - 1) * (_GRID_HEIGHT - 1)
    + 2 * (_GRID_WIDTH - 1) * (_GRID_DEPTH - 1) * _GRID_HEIGHT
)
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_CELL_SIZE = np.array([0.8, 0.8, 0.8])
_CELL_HALF = 0.4
_BASE_Z = 8.0
_STACK_GAP = 2.0
_FRICTION = 0.5
_LINEAR_STIFFNESS = 1000.0
_ANGULAR_STIFFNESS = 250.0
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 11,
    "scene_name": "Soft Body",
    "scene_count": 14,
    "scene_builder": "sceneSoftBody",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "z",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {
            "count": 1,
            "size": tuple(_GROUND_SIZE),
            "density": 0.0,
            "friction": _FRICTION,
            "position": (0.0, 0.0, 0.0),
        },
        "soft_cell": {
            "count": _GRID_CELLS,
            "size": tuple(_CELL_SIZE),
            "density": 1.0,
            "friction": _FRICTION,
            "width": _GRID_WIDTH,
            "depth": _GRID_DEPTH,
            "height": _GRID_HEIGHT,
            "stacks": _GRID_STACKS,
            "base_z": _BASE_Z,
            "stack_gap": _STACK_GAP,
            "first_position": (-1.2, -1.2, 8.0),
        },
    },
    "source_constraints": {
        "finite_all_axis_fixed_joints": {
            "count": _GRID_JOINTS,
            "x_count": _GRID_STACKS * _X_JOINTS_PER_STACK,
            "y_count": _GRID_STACKS * _Y_JOINTS_PER_STACK,
            "z_count": _GRID_STACKS * _Z_JOINTS_PER_STACK,
            "x_parent_anchor": (_CELL_HALF, 0.0, 0.0),
            "x_child_anchor": (-_CELL_HALF, 0.0, 0.0),
            "y_parent_anchor": (0.0, _CELL_HALF, 0.0),
            "y_child_anchor": (0.0, -_CELL_HALF, 0.0),
            "z_parent_anchor": (0.0, 0.0, _CELL_HALF),
            "z_child_anchor": (0.0, 0.0, -_CELL_HALF),
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


def _source_cell_position(stack: int, x: int, y: int, z: int) -> np.ndarray:
    stack_z = stack * (_GRID_HEIGHT * float(_CELL_SIZE[2]) + _STACK_GAP)
    return np.array(
        [
            (x - (_GRID_WIDTH - 1) * 0.5) * float(_CELL_SIZE[0]),
            (y - (_GRID_DEPTH - 1) * 0.5) * float(_CELL_SIZE[1]),
            _BASE_Z + stack_z + z * float(_CELL_SIZE[2]),
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
        "source_collision_filters": dict(_SOURCE_ROW["source_collision_filters"]),
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


def _configure_finite_joint(joint: sx.Joint) -> sx.Joint:
    policy = joint.constraint_projection_policy
    policy.linear_stiffness = _LINEAR_STIFFNESS
    joint.constraint_projection_policy = policy
    policy = joint.constraint_projection_policy
    policy.angular_stiffness = _ANGULAR_STIFFNESS
    joint.constraint_projection_policy = policy
    return joint


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, _GRAVITY))

    ground = _add_source_box(
        world,
        "avbd_demo3d_soft_body_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=np.zeros(3),
        is_static=True,
    )

    grids: list[list[list[list[sx.RigidBody]]]] = []
    cells: list[sx.RigidBody] = []
    for stack in range(_GRID_STACKS):
        grid: list[list[list[sx.RigidBody]]] = []
        for x in range(_GRID_WIDTH):
            yz_plane: list[list[sx.RigidBody]] = []
            for y in range(_GRID_DEPTH):
                column: list[sx.RigidBody] = []
                for z in range(_GRID_HEIGHT):
                    cell = _add_source_box(
                        world,
                        f"avbd_demo3d_soft_body_cell_{stack:02d}_"
                        f"{x:02d}_{y:02d}_{z:02d}",
                        size=_CELL_SIZE,
                        density=1.0,
                        position=_source_cell_position(stack, x, y, z),
                    )
                    column.append(cell)
                    cells.append(cell)
                yz_plane.append(column)
            grid.append(yz_plane)
        grids.append(grid)

    x_joints = []
    y_joints = []
    z_joints = []
    for stack, grid in enumerate(grids):
        for x in range(1, _GRID_WIDTH):
            for y in range(_GRID_DEPTH):
                for z in range(_GRID_HEIGHT):
                    x_joints.append(
                        _configure_finite_joint(
                            world.add_joint(
                                grid[x - 1][y][z],
                                grid[x][y][z],
                                                                sx.JointSpec(
                                    name=f"avbd_demo3d_soft_body_fixed_x_"
                                f"{stack:02d}_{x - 1:02d}_{y:02d}_{z:02d}",
                                    type=sx.JointType.FIXED,
                                )
                            )
                        )
                    )
        for x in range(_GRID_WIDTH):
            for y in range(1, _GRID_DEPTH):
                for z in range(_GRID_HEIGHT):
                    y_joints.append(
                        _configure_finite_joint(
                            world.add_joint(
                                grid[x][y - 1][z],
                                grid[x][y][z],
                                                                sx.JointSpec(
                                    name=f"avbd_demo3d_soft_body_fixed_y_"
                                f"{stack:02d}_{x:02d}_{y - 1:02d}_{z:02d}",
                                    type=sx.JointType.FIXED,
                                )
                            )
                        )
                    )
        for x in range(_GRID_WIDTH):
            for y in range(_GRID_DEPTH):
                for z in range(1, _GRID_HEIGHT):
                    z_joints.append(
                        _configure_finite_joint(
                            world.add_joint(
                                grid[x][y][z - 1],
                                grid[x][y][z],
                                                                sx.JointSpec(
                                    name=f"avbd_demo3d_soft_body_fixed_z_"
                                f"{stack:02d}_{x:02d}_{y:02d}_{z - 1:02d}",
                                    type=sx.JointType.FIXED,
                                )
                            )
                        )
                    )

    ignored_collision_pairs: list[tuple[sx.RigidBody, sx.RigidBody]] = []
    for grid in grids:
        for x in range(1, _GRID_WIDTH):
            for y in range(_GRID_DEPTH):
                for z in range(1, _GRID_HEIGHT):
                    for first, second in (
                        (grid[x - 1][y][z - 1], grid[x][y][z]),
                        (grid[x][y][z - 1], grid[x - 1][y][z]),
                    ):
                        world.set_collision_pair_ignored(first, second)
                        ignored_collision_pairs.append((first, second))
        for x in range(_GRID_WIDTH):
            for y in range(1, _GRID_DEPTH):
                for z in range(1, _GRID_HEIGHT):
                    for first, second in (
                        (grid[x][y - 1][z - 1], grid[x][y][z]),
                        (grid[x][y][z - 1], grid[x][y - 1][z]),
                    ):
                        world.set_collision_pair_ignored(first, second)
                        ignored_collision_pairs.append((first, second))
        for x in range(1, _GRID_WIDTH):
            for y in range(1, _GRID_DEPTH):
                for z in range(_GRID_HEIGHT):
                    for first, second in (
                        (grid[x - 1][y - 1][z], grid[x][y][z]),
                        (grid[x][y - 1][z], grid[x - 1][y][z]),
                    ):
                        world.set_collision_pair_ignored(first, second)
                        ignored_collision_pairs.append((first, second))

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_soft_body_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.40, 0.41, 0.43),
        name="avbd_demo3d_soft_body_ground_visual",
    )
    palette = ((0.24, 0.55, 0.78), (0.86, 0.53, 0.20), (0.42, 0.63, 0.33))
    for stack, grid in enumerate(grids):
        for x, yz_plane in enumerate(grid):
            for y, column in enumerate(yz_plane):
                for z, cell in enumerate(column):
                    bridge.add_rigid_body_visual(
                        cell,
                        dart.BoxShape(_CELL_SIZE),
                        palette[stack % len(palette)],
                        name=(
                            f"avbd_demo3d_soft_body_cell_"
                            f"{stack:02d}_{x:02d}_{y:02d}_{z:02d}_visual"
                        ),
                    )
    bridge.sync()

    center = grids[0][_GRID_WIDTH // 2][_GRID_DEPTH // 2][_GRID_HEIGHT // 2]
    center_z_history: deque[float] = deque(maxlen=160)
    contact_history: deque[float] = deque(maxlen=160)
    initial_center_z = float(np.asarray(center.translation, dtype=float).reshape(3)[2])
    joints = [*x_joints, *y_joints, *z_joints]

    def build_panel(builder: object, context: object) -> None:
        center_z = float(np.asarray(center.translation, dtype=float).reshape(3)[2])
        contacts = len(world.collide())
        center_z_history.append(center_z)
        contact_history.append(float(contacts))

        builder.text("source corpus: avbd-demo3d Soft Body")
        builder.text("source scene: sceneSoftBody, index 11 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"finite fixed joints: {len(joints)}")
        builder.text(f"ignored collision pairs: {len(ignored_collision_pairs)}")
        builder.text(f"contacts: {contacts}")
        builder.text(f"center dz: {center_z - initial_center_z:.3f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Center z", list(center_z_history))
        builder.plot_lines("Contacts", list(contact_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Soft Body", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "cells": cells,
            "cell_grids": grids,
            "cell_size": np.array(_CELL_SIZE, copy=True),
            "joints": joints,
            "x_joints": x_joints,
            "y_joints": y_joints,
            "z_joints": z_joints,
            "ignored_collision_pairs": ignored_collision_pairs,
            "source_demo_row": "avbd-demo3d soft body",
            "source_demo_reference": _source_row(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_soft_body",
    title="AVBD Demo3D Soft Body (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Soft Body row port with 432 finite-stiffness joints.",
    build=build,
)
