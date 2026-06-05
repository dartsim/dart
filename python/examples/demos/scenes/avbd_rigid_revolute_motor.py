"""AVBD revolute-motor scene for the experimental World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_HUB_HALF = np.array([0.16, 0.16, 0.16])
_ROTOR_HALF = np.array([0.52, 0.08, 0.08])
_BASE_POS = np.array([0.0, 0.0, 1.0])
_ROTOR_POS = _BASE_POS + np.array([0.52, 0.0, 0.0])
_TARGET_SPEED = 1.2
_MAX_TORQUE = 800.0


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))

    hub = world.add_rigid_body("avbd_motor_hub", position=tuple(_BASE_POS))
    hub.is_static = True

    rotor = world.add_rigid_body("avbd_motor_rotor", position=tuple(_ROTOR_POS))
    rotor.mass = 1.0

    motor_joint = world.add_rigid_body_revolute_joint(
        "avbd_motor_hinge",
        hub,
        rotor,
        axis=(0.0, 0.0, 1.0),
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_TORQUE], [_MAX_TORQUE])

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_rigid_revolute_motor_render")
    bridge.add_rigid_body_visual(
        hub,
        dart.BoxShape(_full(_HUB_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_motor_hub_visual",
    )
    bridge.add_rigid_body_visual(
        rotor,
        dart.BoxShape(_full(_ROTOR_HALF)),
        (0.94, 0.62, 0.18),
        name="avbd_motor_rotor_visual",
    )

    axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_motor_axis_visual",
        _translation(_BASE_POS + np.array([0.0, 0.0, 0.33])),
    )
    axis.set_shape(dart.CylinderShape(0.024, 0.72))
    axis.create_visual_aspect().set_color([0.25, 0.66, 0.92])
    bridge.render_world.add_simple_frame(axis)
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=160)
    error_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        angular_velocity = np.asarray(rotor.angular_velocity, dtype=float).reshape(3)
        measured_speed = float(angular_velocity[2])
        speed_error = _TARGET_SPEED - measured_speed
        speed_history.append(measured_speed)
        error_history.append(speed_error)

        builder.text("solver: AVBD revolute velocity motor")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"target speed: {_TARGET_SPEED:.2f} rad/s")
        builder.text(f"measured speed: {measured_speed:.2f} rad/s")
        builder.text(f"max torque: {_MAX_TORQUE:.1f} N m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Angular speed", list(speed_history))
        builder.plot_lines("Speed error", list(error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Revolute Motor", build_panel)],
        info={
            "sx_world": world,
            "hub": hub,
            "rotor": rotor,
            "joint": motor_joint,
            "target_speed": _TARGET_SPEED,
            "max_torque": _MAX_TORQUE,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_revolute_motor",
    title="AVBD Revolute Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A bounded revolute velocity motor drives a free rigid-body hinge.",
    build=build,
)
