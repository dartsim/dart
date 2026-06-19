"""AVBD port of the avbd-demo3d Breakable source scene."""

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
_CHAIN_LINKS = 11
_BREAKABLE_JOINTS = _CHAIN_LINKS - 1
_FALLING_BLOCKS = 5
_BREAK_FORCE = 90.0
_GROUND_SIZE = np.array([100.0, 100.0, 1.0])
_CHAIN_SIZE = np.array([1.0, 1.0, 0.5])
_SUPPORT_SIZE = np.array([1.0, 1.0, 5.0])
_FALLING_BLOCK_SIZE = np.array([2.0, 1.0, 1.0])
_FRICTION = 0.5
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo3d",
    "repository": "https://github.com/savant117/avbd-demo3d",
    "revision": "7701bd427d55",
    "dimension": 3,
    "scene_index": 13,
    "scene_name": "Breakable",
    "scene_count": 14,
    "scene_builder": "sceneBreakable",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "z",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {"size": tuple(_GROUND_SIZE), "density": 0.0, "friction": _FRICTION},
        "chain": {"size": tuple(_CHAIN_SIZE), "density": 1.0, "friction": _FRICTION},
        "support": {
            "size": tuple(_SUPPORT_SIZE),
            "density": 0.0,
            "friction": _FRICTION,
        },
        "falling_block": {
            "size": tuple(_FALLING_BLOCK_SIZE),
            "density": 1.0,
            "friction": _FRICTION,
        },
    },
    "source_constraints": {
        "breakable_fixed_joints": {
            "count": _BREAKABLE_JOINTS,
            "anchor_a": (0.5, 0.0, 0.0),
            "anchor_b": (-0.5, 0.0, 0.0),
            "linear_stiffness": "infinity",
            "angular_stiffness": "infinity",
            "break_force": _BREAK_FORCE,
        },
    },
    "expected_counts": {
        "rigid_bodies": 19,
        "joints": _BREAKABLE_JOINTS,
        "breakable_joints": _BREAKABLE_JOINTS,
        "collision_shapes": 19,
    },
}


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _full_box_inertia(size: np.ndarray, mass: float) -> np.ndarray:
    return np.diag(
        [
            mass * float(size[1] * size[1] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[2] * size[2]) / 12.0,
            mass * float(size[0] * size[0] + size[1] * size[1]) / 12.0,
        ]
    )


def _box_mass(size: np.ndarray, density: float) -> float:
    return float(np.prod(size) * density)


def _normalized_or(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return fallback
    return vector / norm


def _connector_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    x_axis = _normalized_or(end - start, np.array([1.0, 0.0, 0.0]))
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ x_axis)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = _normalized_or(np.cross(reference, x_axis), np.array([0.0, 1.0, 0.0]))
    z_axis = np.cross(x_axis, y_axis)

    transform = _translation(0.5 * (start + end))
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    return transform


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


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, _GRAVITY))

    ground = _add_source_box(
        world,
        "avbd_demo3d_breakable_ground",
        size=_GROUND_SIZE,
        density=0.0,
        position=np.array([0.0, 0.0, 0.0]),
        is_static=True,
    )

    chain = []
    for index in range(_CHAIN_LINKS):
        chain.append(
            _add_source_box(
                world,
                f"avbd_demo3d_breakable_chain_{index:02d}",
                size=_CHAIN_SIZE,
                density=1.0,
                position=np.array([float(index - _BREAKABLE_JOINTS / 2), 0.0, 6.0]),
            )
        )

    joints = []
    for index, (parent, child) in enumerate(zip(chain, chain[1:])):
        joint = world.add_joint(
            parent,
            child,
                        sx.JointSpec(
                name=f"avbd_demo3d_breakable_joint_{index:02d}",
                type=sx.JointType.FIXED,
            )
        )
        joint.break_force = _BREAK_FORCE
        joints.append(joint)

    supports = [
        _add_source_box(
            world,
            "avbd_demo3d_breakable_left_support",
            size=_SUPPORT_SIZE,
            density=0.0,
            position=np.array([-_BREAKABLE_JOINTS / 2.0, 0.0, 2.5]),
            is_static=True,
        ),
        _add_source_box(
            world,
            "avbd_demo3d_breakable_right_support",
            size=_SUPPORT_SIZE,
            density=0.0,
            position=np.array([_BREAKABLE_JOINTS / 2.0, 0.0, 2.5]),
            is_static=True,
        ),
    ]

    falling_blocks = []
    for index in range(_FALLING_BLOCKS):
        falling_blocks.append(
            _add_source_box(
                world,
                f"avbd_demo3d_breakable_falling_block_{index:02d}",
                size=_FALLING_BLOCK_SIZE,
                density=1.0,
                position=np.array([0.0, 0.0, index * 2.0 + 8.0]),
            )
        )

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo3d_breakable_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.42, 0.43, 0.45),
        name="avbd_demo3d_breakable_ground_visual",
    )
    for body in chain:
        bridge.add_rigid_body_visual(
            body,
            dart.BoxShape(_CHAIN_SIZE),
            (0.21, 0.48, 0.78),
            name=f"{body.name}_visual",
        )
    for body in supports:
        bridge.add_rigid_body_visual(
            body,
            dart.BoxShape(_SUPPORT_SIZE),
            (0.28, 0.30, 0.33),
            name=f"{body.name}_visual",
        )
    for body in falling_blocks:
        bridge.add_rigid_body_visual(
            body,
            dart.BoxShape(_FALLING_BLOCK_SIZE),
            (0.92, 0.52, 0.18),
            name=f"{body.name}_visual",
        )

    connector_frames = []
    connector_visuals = []
    for index, (parent, child) in enumerate(zip(chain, chain[1:])):
        parent_pos = np.asarray(parent.translation, dtype=float).reshape(3)
        child_pos = np.asarray(child.translation, dtype=float).reshape(3)
        connector = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"avbd_demo3d_breakable_connector_{index:02d}",
            _connector_transform(parent_pos, child_pos),
        )
        connector.set_shape(dart.BoxShape(np.array([1.0, 0.045, 0.045])))
        visual = connector.create_visual_aspect()
        visual.set_color([0.80, 0.78, 0.64])
        bridge.render_world.add_simple_frame(connector)
        connector_frames.append(connector)
        connector_visuals.append(visual)

    def sync_connectors() -> None:
        for joint, connector, visual, parent, child in zip(
            joints, connector_frames, connector_visuals, chain, chain[1:]
        ):
            parent_pos = np.asarray(parent.translation, dtype=float).reshape(3)
            child_pos = np.asarray(child.translation, dtype=float).reshape(3)
            connector.set_transform(_connector_transform(parent_pos, child_pos))
            if joint.is_broken:
                visual.set_color([0.95, 0.18, 0.13])
            else:
                visual.set_color([0.80, 0.78, 0.64])

    def replay_sync() -> None:
        bridge.sync()
        sync_connectors()

    def pre_step() -> None:
        bridge.pre_step()
        sync_connectors()

    bridge.sync()
    sync_connectors()

    min_chain_z_history: deque[float] = deque(maxlen=160)
    max_falling_z_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        chain_z = [
            float(np.asarray(body.translation, dtype=float).reshape(3)[2])
            for body in chain
        ]
        falling_z = [
            float(np.asarray(body.translation, dtype=float).reshape(3)[2])
            for body in falling_blocks
        ]
        broken = sum(1 for joint in joints if joint.is_broken)
        min_chain_z_history.append(min(chain_z))
        max_falling_z_history.append(max(falling_z))
        broken_history.append(float(broken))

        builder.text("source corpus: avbd-demo3d Breakable")
        builder.text("source scene: sceneBreakable, index 13 of 14")
        builder.text(f"rigid bodies: {world.num_rigid_bodies}")
        builder.text(f"breakable joints: {len(joints)}")
        builder.text(f"broken joints: {broken}")
        builder.text(f"break force: {_BREAK_FORCE:.1f} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"min chain z: {min(chain_z):.3f} m")
        builder.text(f"top block z: {max(falling_z):.3f} m")
        builder.plot_lines("Min chain z", list(min_chain_z_history))
        builder.plot_lines("Top block z", list(max_falling_z_history))
        builder.plot_lines("Broken joints", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo3D Breakable", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "chain": tuple(chain),
            "joints": tuple(joints),
            "supports": tuple(supports),
            "falling_blocks": tuple(falling_blocks),
            "connectors": tuple(connector_frames),
            "source_demo_row": "avbd-demo3d breakable",
            "source_demo_reference": _source_row(),
            "break_force": _BREAK_FORCE,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo3d_breakable",
    title="AVBD Demo3D Breakable (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Breakable row port with rigid boxes, contact shapes, "
    "and AVBD breakable fixed joints.",
    build=build,
)
