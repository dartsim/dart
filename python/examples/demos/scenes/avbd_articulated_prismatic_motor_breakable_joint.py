"""AVBD articulated breakable prismatic-motor scene for experimental World."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BASE_HALF = np.array([0.16, 0.16, 0.16])
_CARRIAGE_HALF = np.array([0.20, 0.14, 0.14])
_RAIL_HALF = np.array([0.92, 0.025, 0.025])
_TARGET_SPEED = 0.35
_MAX_FORCE = 900.0
_PULL_FORCE = np.array([0.0, 5.0, 0.0])
_BREAK_FORCE = 1.0e-18
_RESET_BREAK_FORCE = 1.0e12


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _orthogonal(position: np.ndarray, axis: np.ndarray) -> np.ndarray:
    return position - float(position @ axis) * axis


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    slider = world.add_multibody("avbd_articulated_prismatic_breakable_motor")
    base = slider.add_link("avbd_articulated_prismatic_breakable_base")
    carriage = slider.add_link(
        "avbd_articulated_prismatic_breakable_carriage",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_prismatic_breakable_floating_carriage",
            type=sx.JointType.FLOATING,
        ),
    )
    carriage.mass = 1.4
    carriage.inertia = ((0.12, 0.0, 0.0), (0.0, 0.18, 0.0), (0.0, 0.0, 0.18))

    axis = np.array([1.0, 0.0, 0.0])
    motor_joint = world.add_articulated_prismatic_joint(
        "avbd_articulated_prismatic_breakable_axis",
        carriage,
        axis=axis.tolist(),
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_FORCE], [_MAX_FORCE])
    motor_joint.break_force = _BREAK_FORCE

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_prismatic_breakable_motor_render"
    )
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_prismatic_breakable_base_visual",
    )
    bridge.add_link_visual(
        carriage,
        dart.BoxShape(_full(_CARRIAGE_HALF)),
        (0.25, 0.66, 0.92),
        name="avbd_articulated_prismatic_breakable_carriage_visual",
    )

    rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_prismatic_breakable_rail_visual",
        _translation(np.zeros(3)),
    )
    rail.set_shape(dart.BoxShape(_full(_RAIL_HALF)))
    rail_visual = rail.create_visual_aspect()
    rail_visual.set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(rail)

    command_state = {"target_speed": _TARGET_SPEED}
    force_state = {"enabled": True}

    def set_target_speed(speed: float) -> None:
        command_state["target_speed"] = float(speed)
        motor_joint.command_velocity = [float(speed)]

    def reset_joint(force: float) -> None:
        carriage.parent_joint.velocity = [0.0] * 6
        motor_joint.break_force = force
        motor_joint.reset_breakage()

    def rearm_weak_joint() -> None:
        reset_joint(_BREAK_FORCE)

    def reset_strong_joint() -> None:
        reset_joint(_RESET_BREAK_FORCE)

    def sync_rail() -> None:
        if motor_joint.is_broken:
            rail_visual.set_color([0.95, 0.18, 0.13])
        else:
            rail_visual.set_color([0.78, 0.78, 0.70])

    def replay_sync() -> None:
        bridge.sync()
        sync_rail()

    def pre_step() -> None:
        motor_joint.command_velocity = [float(command_state["target_speed"])]
        if force_state["enabled"]:
            carriage.apply_force(_PULL_FORCE)
        bridge.pre_step()
        sync_rail()

    bridge.sync()
    sync_rail()

    axis_history: deque[float] = deque(maxlen=160)
    orthogonal_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        position = np.asarray(carriage.translation, dtype=float).reshape(3)
        axis_position = float(position @ axis)
        orthogonal_drift = float(np.linalg.norm(_orthogonal(position, axis)))
        broken = 1.0 if motor_joint.is_broken else 0.0
        axis_history.append(axis_position)
        orthogonal_history.append(orthogonal_drift)
        broken_history.append(broken)

        builder.text("solver: AVBD articulated prismatic motor break/reset")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"state: {'broken' if motor_joint.is_broken else 'intact'}")
        builder.text(f"target speed: {float(command_state['target_speed']):.2f} m/s")
        builder.text(f"break force: {motor_joint.break_force:.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"axis position: {axis_position:.3f} m")
        builder.text(f"orthogonal drift: {orthogonal_drift:.4f} m")
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
        builder.plot_lines("Axis position", list(axis_history))
        builder.plot_lines("Orthogonal drift", list(orthogonal_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Breakable Prismatic Motor", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "carriage": carriage,
            "joint": motor_joint,
            "axis": axis.copy(),
            "rail": rail,
            "target_speed": _TARGET_SPEED,
            "max_force": _MAX_FORCE,
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
    id="avbd_articulated_prismatic_motor_breakable_joint",
    title="AVBD Articulated Breakable Prismatic Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public world-anchored articulated prismatic velocity motor breaks, "
    "releases, and re-engages through the AVBD variational bridge.",
    build=build,
)
