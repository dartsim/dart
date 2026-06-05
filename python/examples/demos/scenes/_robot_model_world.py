"""Shared World-loaded robot-model helpers for py-demos scenes."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._world_bridge import WorldRenderBridge

_ROOT = Path(__file__).resolve().parents[4]
ATLAS_URDF = _ROOT / "data" / "sdf" / "atlas" / "atlas_v5_no_head.urdf"
HUBO_URDF = _ROOT / "data" / "urdf" / "drchubo" / "drchubo.urdf"
HUBO_PACKAGE_DIR = _ROOT / "data" / "urdf" / "drchubo"


def translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _as_matrix(transform: Any) -> np.ndarray:
    if hasattr(transform, "matrix"):
        return np.asarray(transform.matrix(), dtype=float)
    return np.asarray(transform, dtype=float)


def _sanitize(name: str) -> str:
    return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in name)


def _link_color(link_name: str) -> tuple[float, float, float]:
    lower = link_name.lower()
    if lower.startswith(("l_", "body_l")):
        return (0.20, 0.55, 0.90)
    if lower.startswith(("r_", "body_r")):
        return (0.90, 0.40, 0.22)
    if "torso" in lower or "pelvis" in lower or lower.endswith("_tsy"):
        return (0.70, 0.70, 0.68)
    return (0.34, 0.78, 0.48)


def _box_shape_from_mesh(
    collision_shape: sx.CollisionShape,
) -> tuple[dart.BoxShape, np.ndarray] | None:
    vertices = np.asarray(collision_shape.vertices, dtype=float)
    if vertices.ndim != 2 or vertices.shape[0] == 0 or vertices.shape[1] != 3:
        return None
    lower = vertices.min(axis=0)
    upper = vertices.max(axis=0)
    center = 0.5 * (lower + upper)
    size = np.maximum(upper - lower, np.array([0.035, 0.035, 0.035]))
    local = _as_matrix(collision_shape.local_transform) @ translation(*center)
    return dart.BoxShape(size), local


def _render_shape_from_collision(
    collision_shape: sx.CollisionShape,
) -> tuple[dart.Shape, np.ndarray] | None:
    local = _as_matrix(collision_shape.local_transform)
    kind = collision_shape.type
    if kind == sx.CollisionShapeType.SPHERE:
        return dart.SphereShape(float(collision_shape.radius)), local
    if kind == sx.CollisionShapeType.BOX:
        return dart.BoxShape(2.0 * np.asarray(collision_shape.half_extents)), local
    if kind == sx.CollisionShapeType.CAPSULE:
        return (
            dart.CapsuleShape(
                float(collision_shape.radius), float(collision_shape.height)
            ),
            local,
        )
    if kind == sx.CollisionShapeType.CYLINDER:
        return (
            dart.CylinderShape(
                float(collision_shape.radius), float(collision_shape.height)
            ),
            local,
        )
    if kind == sx.CollisionShapeType.MESH:
        return _box_shape_from_mesh(collision_shape)
    return None


def _fallback_shape(link: sx.Link) -> tuple[dart.Shape, np.ndarray]:
    mass = max(float(link.mass), 0.1)
    size = min(0.18, max(0.045, 0.035 * mass ** (1.0 / 3.0)))
    return dart.SphereShape(size), np.eye(4)


def add_robot_link_visuals(
    bridge: WorldRenderBridge,
    robot: sx.Multibody,
    *,
    prefix: str,
) -> int:
    visual_count = 0
    for link in robot.links:
        link_name = str(link.name)
        if link_name.startswith("root_anchor_"):
            continue

        render = None
        for collision_shape in link.collision_shapes:
            render = _render_shape_from_collision(collision_shape)
            if render is not None:
                break
        shape, local_transform = render if render is not None else _fallback_shape(link)
        bridge.add_link_visual(
            link,
            shape,
            _link_color(link_name),
            name=f"{prefix}_{_sanitize(link_name)}_visual",
            local_transform=local_transform,
        )
        visual_count += 1
    return visual_count


def add_ground_visual(
    bridge: WorldRenderBridge,
    *,
    name: str,
    z: float = 0.0,
    size: tuple[float, float, float] = (2.4, 1.6, 0.02),
) -> None:
    frame = dart.SimpleFrame(dart.Frame.world(), name, translation(0.0, 0.0, z))
    frame.set_shape(dart.BoxShape(np.array(size)))
    frame.create_visual_aspect().set_color([0.45, 0.47, 0.50])
    bridge.render_world.add_simple_frame(frame)


def apply_joint_targets(robot: sx.Multibody, targets: Mapping[str, float]) -> None:
    for name, value in targets.items():
        joint = robot.get_joint(name)
        if joint is None or int(joint.num_dofs) != 1:
            continue
        position = np.asarray(joint.position, dtype=float).copy()
        lower = np.asarray(joint.position_lower_limits, dtype=float)
        upper = np.asarray(joint.position_upper_limits, dtype=float)
        target = float(value)
        if lower.size == 1 and upper.size == 1:
            if np.isfinite(lower[0]) and np.isfinite(upper[0]):
                target = float(np.clip(target, lower[0], upper[0]))
        position[0] = target
        joint.position = position
        joint.velocity = np.zeros_like(np.asarray(joint.velocity, dtype=float))
        joint.force = np.zeros_like(np.asarray(joint.force, dtype=float))


def _shape_world_min_z(link: sx.Link) -> float | None:
    mins: list[float] = []
    link_tf = _as_matrix(link.transform)
    for shape in link.collision_shapes:
        local = link_tf @ _as_matrix(shape.local_transform)
        if shape.type == sx.CollisionShapeType.MESH and len(shape.vertices):
            vertices = np.asarray(shape.vertices, dtype=float)
            homogeneous = np.c_[vertices, np.ones(vertices.shape[0])]
            mins.append(float((homogeneous @ local.T)[:, 2].min()))
        elif shape.type == sx.CollisionShapeType.SPHERE:
            mins.append(float(local[2, 3] - shape.radius))
        elif shape.type == sx.CollisionShapeType.BOX:
            half = np.asarray(shape.half_extents, dtype=float)
            corners = np.array(
                [
                    [sx_, sy_, sz_, 1.0]
                    for sx_ in (-half[0], half[0])
                    for sy_ in (-half[1], half[1])
                    for sz_ in (-half[2], half[2])
                ]
            )
            mins.append(float((corners @ local.T)[:, 2].min()))
        elif shape.type in (
            sx.CollisionShapeType.CAPSULE,
            sx.CollisionShapeType.CYLINDER,
        ):
            mins.append(float(local[2, 3] - shape.half_height - shape.radius))
    return min(mins) if mins else None


def lift_root_to_ground(world: sx.World, robot: sx.Multibody, clearance: float = 0.02) -> float:
    world.update_kinematics()
    min_z = None
    for link in robot.links:
        if str(link.name).startswith("root_anchor_"):
            continue
        link_min = _shape_world_min_z(link)
        if link_min is not None:
            min_z = link_min if min_z is None else min(min_z, link_min)

    root = robot.get_joint("rootJoint")
    if root is None or int(root.num_dofs) < 3 or min_z is None:
        return 0.0
    root_position = np.asarray(root.position, dtype=float).copy()
    root_position[2] += -float(min_z) + clearance
    root.position = root_position
    root.velocity = np.zeros_like(np.asarray(root.velocity, dtype=float))
    root.force = np.zeros_like(np.asarray(root.force, dtype=float))
    world.update_kinematics()
    return -float(min_z) + clearance


def load_atlas_world(initial_targets: Mapping[str, float] | None = None) -> tuple[sx.World, sx.Multibody]:
    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 1.0 / 240.0
    robot = sx.add_skeleton(world, ATLAS_URDF.resolve().as_uri())
    robot.name = "atlas"
    if initial_targets:
        apply_joint_targets(robot, initial_targets)
    world.enter_simulation_mode()
    lift_root_to_ground(world, robot)
    return world, robot


def load_hubo_world(initial_targets: Mapping[str, float] | None = None) -> tuple[sx.World, sx.Multibody]:
    world = sx.World()
    world.gravity = (0.0, 0.0, 0.0)
    world.time_step = 1.0 / 240.0
    read_options = sx.ReadOptions()
    read_options.format = sx.ModelFormat.URDF
    read_options.add_package_directory("drchubo", str(HUBO_PACKAGE_DIR.resolve()))
    robot = sx.add_skeleton(world, HUBO_URDF.resolve().as_uri(), read_options)
    robot.name = "hubo"
    if initial_targets:
        apply_joint_targets(robot, initial_targets)
    world.enter_simulation_mode()
    lift_root_to_ground(world, robot)
    return world, robot
