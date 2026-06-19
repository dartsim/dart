"""AVBD prismatic-motor scene for the DART 7 World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
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
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))

    base = world.add_rigid_body("avbd_prismatic_motor_base", position=tuple(_BASE_POS))
    base.is_static = True

    slider = world.add_rigid_body(
        "avbd_prismatic_motor_slider", position=tuple(_SLIDER_POS)
    )
    slider.mass = 1.0

    motor_joint = world.add_joint(
        base,
        slider,
                sx.JointSpec(
            name="avbd_prismatic_motor_axis",
            type=sx.JointType.PRISMATIC,
            axis=(1.0, 0.0, 0.0),
        )
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
    _last_metrics: dict[str, float] = {}

    def sample_metrics() -> dict[str, float]:
        position = np.asarray(slider.translation, dtype=float).reshape(3)
        velocity = np.asarray(slider.linear_velocity, dtype=float).reshape(3)
        measured_speed = float(velocity[0])
        speed_error = float(_TARGET_SPEED - measured_speed)
        axis_position = float(position[0] - _BASE_POS[0])
        orthogonal_drift = float(np.linalg.norm(position[1:] - _BASE_POS[1:]))
        return {
            "measured_speed": measured_speed,
            "speed_error": speed_error,
            "abs_speed_error": abs(speed_error),
            "axis_position": axis_position,
            "orthogonal_drift": orthogonal_drift,
            "world_time": float(world.time),
        }

    def record_metrics() -> dict[str, float]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        speed_history.append(_last_metrics["measured_speed"])
        position_history.append(_last_metrics["axis_position"])
        drift_history.append(_last_metrics["orthogonal_drift"])
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        speed_values = list(speed_history)
        abs_error_values = [
            abs(float(_TARGET_SPEED) - float(value)) for value in speed_history
        ]
        position_values = list(position_history)
        drift_values = list(drift_history)
        return {
            "row": "avbd_rigid_prismatic_motor",
            "solver": "avbd_rigid_joints",
            "executor": "World.step default",
            "actuator": "prismatic_velocity_motor",
            "related_source_row": "rigid_joint_motor_limits",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(motor_joint.name),
            "target_speed": float(_TARGET_SPEED),
            "max_force": float(_MAX_FORCE),
            "measured_speed": float(_last_metrics["measured_speed"]),
            "speed_error": float(_last_metrics["speed_error"]),
            "abs_speed_error": float(_last_metrics["abs_speed_error"]),
            "axis_position": float(_last_metrics["axis_position"]),
            "orthogonal_drift": float(_last_metrics["orthogonal_drift"]),
            "metrics": dict(_last_metrics),
            "history": {
                "samples": float(len(speed_values)),
                "max_measured_speed": max(speed_values, default=0.0),
                "max_abs_speed_error": max(abs_error_values, default=0.0),
                "max_axis_position": max(position_values, default=0.0),
                "max_orthogonal_drift": max(drift_values, default=0.0),
            },
        }

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()

    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        measured_speed = float(metrics["measured_speed"])
        axis_position = float(metrics["axis_position"])
        orthogonal_drift = float(metrics["orthogonal_drift"])

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
        pre_step=pre_step,
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
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_prismatic_motor",
    title="AVBD Prismatic Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A bounded prismatic velocity motor drives a free rigid-body slider.",
    build=build,
)
