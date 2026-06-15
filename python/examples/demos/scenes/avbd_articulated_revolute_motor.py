"""AVBD articulated revolute-motor scene for the experimental World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_HUB_HALF = np.array([0.16, 0.16, 0.16])
_ROTOR_HALF = np.array([0.58, 0.08, 0.08])
_TARGET_SPEED = 1.0
_COMMAND_SWITCH_TIME = 0.15
_MAX_TORQUE = 900.0


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _yaw(link: object) -> float:
    rotation = np.asarray(link.rotation, dtype=float).reshape(3, 3)
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def _command_for_time(time: float) -> float:
    return _TARGET_SPEED if time < _COMMAND_SWITCH_TIME else -_TARGET_SPEED


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    arm = world.add_multibody("avbd_articulated_motor_arm")
    base = arm.add_link("avbd_articulated_motor_base")
    rotor = arm.add_link(
        "avbd_articulated_motor_rotor",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_motor_floating_rotor",
            type=sx.JointType.FLOATING,
        ),
    )
    rotor.mass = 2.0
    rotor.inertia = ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.3))

    motor_joint = world.add_articulated_revolute_joint(
        "avbd_articulated_motor_hinge",
        base,
        rotor,
        axis=(0.0, 0.0, 1.0),
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_TORQUE], [_MAX_TORQUE])

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_articulated_revolute_motor_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_HUB_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_motor_base_visual",
    )
    bridge.add_link_visual(
        rotor,
        dart.BoxShape(_full(_ROTOR_HALF)),
        (0.25, 0.66, 0.92),
        name="avbd_articulated_motor_rotor_visual",
    )

    axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_motor_axis_visual",
        _translation(np.array([0.0, 0.0, 0.34])),
    )
    axis.set_shape(dart.CylinderShape(0.024, 0.72))
    axis.create_visual_aspect().set_color([0.92, 0.55, 0.18])
    bridge.render_world.add_simple_frame(axis)

    def apply_command() -> float:
        target = _command_for_time(float(world.time))
        motor_joint.command_velocity = [target]
        return target

    def pre_step() -> None:
        apply_command()
        bridge.pre_step()

    bridge.sync()

    yaw_history: deque[float] = deque(maxlen=160)
    command_history: deque[float] = deque(maxlen=160)
    error_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        target = _command_for_time(float(world.time))
        yaw = _yaw(rotor)
        command = float(np.asarray(motor_joint.command_velocity, dtype=float)[0])
        yaw_history.append(yaw)
        command_history.append(command)
        error_history.append(target - command)

        builder.text("solver: AVBD articulated revolute velocity motor")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"target speed: {target:.2f} rad/s")
        builder.text(f"active command: {command:.2f} rad/s")
        builder.text(f"command switch: {_COMMAND_SWITCH_TIME:.2f} s")
        builder.text(f"yaw: {yaw:.3f} rad")
        builder.text(f"max torque: {_MAX_TORQUE:.1f} N m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Yaw", list(yaw_history))
        builder.plot_lines("Command", list(command_history))
        builder.plot_lines("Command error", list(error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Revolute Motor", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "rotor": rotor,
            "joint": motor_joint,
            "target_speed": _TARGET_SPEED,
            "command_switch_time": _COMMAND_SWITCH_TIME,
            "max_torque": _MAX_TORQUE,
            "apply_command": apply_command,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_revolute_motor",
    title="AVBD Articulated Revolute Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public articulated revolute velocity motor reverses command "
    "through the AVBD variational bridge.",
    build=build,
)
