"""AVBD articulated prismatic-motor scene for the experimental World facade."""

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
_TARGET_SPEED = 0.45
_COMMAND_SWITCH_TIME = 0.15
_MAX_FORCE = 700.0


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _command_for_time(time: float) -> float:
    return _TARGET_SPEED if time < _COMMAND_SWITCH_TIME else -_TARGET_SPEED


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    slider = world.add_multibody("avbd_articulated_prismatic_motor")
    base = slider.add_link("avbd_articulated_prismatic_base")
    carriage = slider.add_link(
        "avbd_articulated_prismatic_carriage",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_prismatic_floating_carriage",
            type=sx.JointType.FLOATING,
        ),
    )
    carriage.mass = 1.4
    carriage.inertia = ((0.12, 0.0, 0.0), (0.0, 0.18, 0.0), (0.0, 0.0, 0.18))

    motor_joint = world.add_joint(
        base,
        carriage,
                sx.JointSpec(
            name="avbd_articulated_prismatic_axis",
            type=sx.JointType.PRISMATIC,
            axis=(1.0, 0.0, 0.0),
        )
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_FORCE], [_MAX_FORCE])

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_articulated_prismatic_motor_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_prismatic_base_visual",
    )
    bridge.add_link_visual(
        carriage,
        dart.BoxShape(_full(_CARRIAGE_HALF)),
        (0.25, 0.66, 0.92),
        name="avbd_articulated_prismatic_carriage_visual",
    )

    rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_prismatic_rail_visual",
        _translation(np.zeros(3)),
    )
    rail.set_shape(dart.BoxShape(_full(_RAIL_HALF)))
    rail.create_visual_aspect().set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(rail)

    def apply_command() -> float:
        target = _command_for_time(float(world.time))
        motor_joint.command_velocity = [target]
        return target

    def pre_step() -> None:
        apply_command()
        bridge.pre_step()

    bridge.sync()

    position_history: deque[float] = deque(maxlen=160)
    command_history: deque[float] = deque(maxlen=160)
    orthogonal_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        position = np.asarray(carriage.translation, dtype=float).reshape(3)
        target = _command_for_time(float(world.time))
        command = float(np.asarray(motor_joint.command_velocity, dtype=float)[0])
        orthogonal = float(np.linalg.norm(position[1:]))
        position_history.append(float(position[0]))
        command_history.append(command)
        orthogonal_history.append(orthogonal)

        builder.text("solver: AVBD articulated prismatic velocity motor")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"target speed: {target:.2f} m/s")
        builder.text(f"active command: {command:.2f} m/s")
        builder.text(f"command switch: {_COMMAND_SWITCH_TIME:.2f} s")
        builder.text(f"axis position: {position[0]:.3f} m")
        builder.text(f"orthogonal drift: {orthogonal:.6f} m")
        builder.text(f"max force: {_MAX_FORCE:.1f} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Axis position", list(position_history))
        builder.plot_lines("Command", list(command_history))
        builder.plot_lines("Orthogonal drift", list(orthogonal_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Prismatic Motor", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "carriage": carriage,
            "joint": motor_joint,
            "target_speed": _TARGET_SPEED,
            "command_switch_time": _COMMAND_SWITCH_TIME,
            "max_force": _MAX_FORCE,
            "apply_command": apply_command,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_prismatic_motor",
    title="AVBD Articulated Prismatic Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public articulated prismatic velocity motor reverses command "
    "through the AVBD variational bridge.",
    build=build,
)
