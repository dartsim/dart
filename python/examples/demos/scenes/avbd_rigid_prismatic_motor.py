"""AVBD prismatic-motor scene for the DART 7 World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BASE_HALF = np.array([0.16, 0.16, 0.16])
_SLIDER_HALF = np.array([0.22, 0.13, 0.13])
_RAIL_HALF = np.array([0.95, 0.025, 0.025])
_BASE_POS = np.array([0.0, 0.0, 1.0])
_SLIDER_POS = _BASE_POS + np.array([0.42, 0.0, 0.0])
_TARGET_SPEED = 0.8
_MAX_FORCE = 800.0


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))

    base = world.add_rigid_body("avbd_prismatic_motor_base", position=tuple(_BASE_POS))
    base.is_static = True

    slider = world.add_rigid_body(
        "avbd_prismatic_motor_slider", position=tuple(_SLIDER_POS)
    )
    slider.mass = 1.0

    motor_joint = world.add_rigid_body_prismatic_joint(
        "avbd_prismatic_motor_axis",
        base,
        slider,
        axis=(1.0, 0.0, 0.0),
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_FORCE], [_MAX_FORCE])

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_rigid_prismatic_motor_render")
    bridge.add_rigid_body_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_prismatic_motor_base_visual",
    )
    bridge.add_rigid_body_visual(
        slider,
        dart.BoxShape(_full(_SLIDER_HALF)),
        (0.25, 0.66, 0.92),
        name="avbd_prismatic_motor_slider_visual",
    )

    rail = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_prismatic_motor_rail_visual",
        _translation(_BASE_POS),
    )
    rail.set_shape(dart.BoxShape(_full(_RAIL_HALF)))
    rail.create_visual_aspect().set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(rail)
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=160)
    position_history: deque[float] = deque(maxlen=160)
    drift_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        position = np.asarray(slider.translation, dtype=float).reshape(3)
        velocity = np.asarray(slider.linear_velocity, dtype=float).reshape(3)
        measured_speed = float(velocity[0])
        axis_position = float(position[0] - _BASE_POS[0])
        orthogonal_drift = float(np.linalg.norm(position[1:] - _BASE_POS[1:]))
        speed_history.append(measured_speed)
        position_history.append(axis_position)
        drift_history.append(orthogonal_drift)

        builder.text("solver: AVBD prismatic velocity motor")
        builder.text(f"joint: {motor_joint.name}")
        builder.text(f"target speed: {_TARGET_SPEED:.2f} m/s")
        builder.text(f"measured speed: {measured_speed:.2f} m/s")
        builder.text(f"axis position: {axis_position:.3f} m")
        builder.text(f"orthogonal drift: {orthogonal_drift:.6f} m")
        builder.text(f"max force: {_MAX_FORCE:.1f} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Linear speed", list(speed_history))
        builder.plot_lines("Axis position", list(position_history))
        builder.plot_lines("Orthogonal drift", list(drift_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Prismatic Motor", build_panel)],
        info={
            "sx_world": world,
            "base": base,
            "slider": slider,
            "joint": motor_joint,
            "target_speed": _TARGET_SPEED,
            "max_force": _MAX_FORCE,
            "base_position": _BASE_POS.copy(),
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_prismatic_motor",
    title="AVBD Prismatic Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A bounded prismatic velocity motor drives a free rigid-body slider.",
    build=build,
)
