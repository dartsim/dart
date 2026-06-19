"""AVBD world-anchored articulated breakable revolute-motor scene."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BASE_HALF = np.array([0.16, 0.16, 0.16])
_ROTOR_HALF = np.array([0.58, 0.08, 0.08])
_TARGET_SPEED = 0.75
_MAX_TORQUE = 900.0
_PULL_FORCE = np.array([0.0, 5.0, 0.0])
_BREAK_FORCE = 1.0e-18
_RESET_BREAK_FORCE = 1.0e12


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _yaw(link: object) -> float:
    rotation = np.asarray(link.rotation, dtype=float).reshape(3, 3)
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    arm = world.add_multibody("avbd_articulated_world_revolute_breakable_motor")
    base = arm.add_link("avbd_articulated_world_revolute_breakable_base")
    rotor = arm.add_link(
        "avbd_articulated_world_revolute_breakable_rotor",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_world_revolute_breakable_floating_rotor",
            type=sx.JointType.FLOATING,
        ),
    )
    rotor.mass = 2.0
    rotor.inertia = ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.3))

    motor_joint = world.add_joint(
        rotor,
                sx.JointSpec(
            name="avbd_articulated_world_revolute_breakable_hinge",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 0.0, 1.0),
        )
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_TORQUE], [_MAX_TORQUE])
    motor_joint.break_force = _BREAK_FORCE

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_world_revolute_breakable_motor_render"
    )
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_world_revolute_breakable_base_visual",
    )
    bridge.add_link_visual(
        rotor,
        dart.BoxShape(_full(_ROTOR_HALF)),
        (0.25, 0.66, 0.92),
        name="avbd_articulated_world_revolute_breakable_rotor_visual",
    )

    axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_world_revolute_breakable_axis_visual",
        _translation(np.array([0.0, 0.0, 0.34])),
    )
    axis.set_shape(dart.CylinderShape(0.024, 0.72))
    axis_visual = axis.create_visual_aspect()
    axis_visual.set_color([0.92, 0.55, 0.18])
    bridge.render_world.add_simple_frame(axis)

    command_state = {"target_speed": _TARGET_SPEED}
    force_state = {"enabled": True}

    def set_target_speed(speed: float) -> None:
        command_state["target_speed"] = float(speed)
        motor_joint.command_velocity = [float(speed)]

    def reset_joint(force: float) -> None:
        rotor.parent_joint.velocity = [0.0] * 6
        motor_joint.break_force = force
        motor_joint.reset_breakage()

    def rearm_weak_joint() -> None:
        reset_joint(_BREAK_FORCE)

    def reset_strong_joint() -> None:
        reset_joint(_RESET_BREAK_FORCE)

    def sync_axis() -> None:
        if motor_joint.is_broken:
            axis_visual.set_color([0.95, 0.18, 0.13])
        else:
            axis_visual.set_color([0.92, 0.55, 0.18])

    def replay_sync() -> None:
        bridge.sync()
        sync_axis()

    def pre_step() -> None:
        motor_joint.command_velocity = [float(command_state["target_speed"])]
        if force_state["enabled"]:
            rotor.apply_force(_PULL_FORCE)
        bridge.pre_step()
        sync_axis()

    bridge.sync()
    sync_axis()

    yaw_history: deque[float] = deque(maxlen=160)
    drift_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        translation = np.asarray(rotor.translation, dtype=float).reshape(3)
        yaw = _yaw(rotor)
        drift = float(np.linalg.norm(translation))
        broken = 1.0 if motor_joint.is_broken else 0.0
        yaw_history.append(yaw)
        drift_history.append(drift)
        broken_history.append(broken)

        builder.text("solver: AVBD world revolute motor break/reset")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"state: {'broken' if motor_joint.is_broken else 'intact'}")
        builder.text(f"target speed: {float(command_state['target_speed']):.2f} rad/s")
        builder.text(f"break force: {motor_joint.break_force:.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"yaw: {yaw:.3f} rad")
        builder.text(f"anchor drift: {drift:.4f} m")
        if builder.button("Reset strong motor"):
            reset_strong_joint()
        if builder.button("Re-arm weak motor"):
            rearm_weak_joint()
        changed, enabled = builder.checkbox(
            "Apply lateral pull",
            bool(force_state["enabled"]),
        )
        if changed:
            force_state["enabled"] = bool(enabled)
        builder.plot_lines("Yaw", list(yaw_history))
        builder.plot_lines("Anchor drift", list(drift_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[
            ScenePanel(
                "AVBD Articulated World Revolute Breakable Motor", build_panel
            )
        ],
        info={
            "sx_world": world,
            "base": base,
            "rotor": rotor,
            "joint": motor_joint,
            "axis": axis,
            "target_speed": _TARGET_SPEED,
            "max_torque": _MAX_TORQUE,
            "break_force": _BREAK_FORCE,
            "reset_break_force": _RESET_BREAK_FORCE,
            "pull_force": _PULL_FORCE.copy(),
            "set_target_speed": set_target_speed,
            "reset_joint": reset_joint,
            "rearm_weak_joint": rearm_weak_joint,
            "reset_strong_joint": reset_strong_joint,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_world_revolute_motor_breakable_joint",
    title="AVBD Articulated World Revolute Breakable Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public world-anchored articulated revolute velocity motor breaks, "
    "releases, and re-engages through the AVBD variational bridge.",
    build=build,
)
