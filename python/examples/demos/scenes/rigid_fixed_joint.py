"""Rigid-body fixed-joint scene for the World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_PAYLOAD_OFFSET = np.array([0.85, 0.0, 0.0])
_BASE_HALF = np.array([0.16, 0.16, 0.16])
_PAYLOAD_HALF = np.array([0.22, 0.14, 0.14])


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, -9.81))

    base = world.add_rigid_body("fixed_joint_base", position=(0.0, 0.0, 1.0))
    base.is_static = True

    payload = world.add_rigid_body(
        "fixed_joint_payload", position=tuple(base.translation + _PAYLOAD_OFFSET)
    )
    payload.mass = 1.0
    payload.angular_velocity = (0.0, 0.0, 1.2)

    fixed_joint = world.add_rigid_body_fixed_joint(
        "fixed_joint_base_to_payload", base, payload
    )
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="rigid_fixed_joint_render")
    bridge.add_rigid_body_visual(
        base,
        dart.BoxShape(2.0 * _BASE_HALF),
        (0.32, 0.34, 0.38),
        name="fixed_joint_base_visual",
    )
    bridge.add_rigid_body_visual(
        payload,
        dart.BoxShape(2.0 * _PAYLOAD_HALF),
        (0.88, 0.42, 0.18),
        name="fixed_joint_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.Frame.world(),
        "fixed_joint_connector_visual",
        _translation(0.5 * _PAYLOAD_OFFSET[0], 0.0, 1.0),
    )
    connector.set_shape(dart.BoxShape(np.array([_PAYLOAD_OFFSET[0], 0.035, 0.035])))
    connector.create_visual_aspect().set_color([0.78, 0.78, 0.72])
    bridge.render_world.add_simple_frame(connector)
    bridge.sync()

    error_history: deque[float] = deque(maxlen=120)
    speed_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        base_position = np.asarray(base.translation, dtype=float).reshape(3)
        payload_position = np.asarray(payload.translation, dtype=float).reshape(3)
        error = float(np.linalg.norm((payload_position - base_position) - _PAYLOAD_OFFSET))
        speed = float(np.linalg.norm(np.asarray(payload.linear_velocity, dtype=float)))
        error_history.append(error)
        speed_history.append(speed)

        builder.text("joint: fixed")
        builder.text(f"name: {fixed_joint.name}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"offset error: {error:.4f} m")
        builder.text(f"payload speed: {speed:.3f} m/s")
        builder.plot_lines("Offset error", list(error_history))
        builder.plot_lines("Payload speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Rigid Fixed Joint", build_panel)],
        info={
            "sx_world": world,
            "joint": fixed_joint,
            "base": base,
            "payload": payload,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_fixed_joint",
    title="Rigid Fixed Joint",
    category="World Rigid Body",
    summary="A fixed joint holds a free rigid body at a captured offset.",
    build=build,
)
