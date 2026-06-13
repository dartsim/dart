"""AVBD revolute-motor scene for the DART 7 World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
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
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))

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
    _last_metrics: dict[str, float] = {}

    def sample_metrics() -> dict[str, float]:
        angular_velocity = np.asarray(rotor.angular_velocity, dtype=float).reshape(3)
        measured_speed = float(angular_velocity[2])
        speed_error = float(_TARGET_SPEED - measured_speed)
        return {
            "measured_speed": measured_speed,
            "speed_error": speed_error,
            "abs_speed_error": abs(speed_error),
            "world_time": float(world.time),
        }

    def record_metrics() -> dict[str, float]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        speed_history.append(_last_metrics["measured_speed"])
        error_history.append(_last_metrics["speed_error"])
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        speed_values = list(speed_history)
        abs_error_values = [abs(value) for value in error_history]
        return {
            "row": "avbd_rigid_revolute_motor",
            "solver": "avbd_rigid_joints",
            "executor": "World.step default",
            "actuator": "revolute_velocity_motor",
            "related_source_row": "rigid_joint_motor_limits",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(motor_joint.name),
            "target_speed": float(_TARGET_SPEED),
            "max_torque": float(_MAX_TORQUE),
            "measured_speed": float(_last_metrics["measured_speed"]),
            "speed_error": float(_last_metrics["speed_error"]),
            "abs_speed_error": float(_last_metrics["abs_speed_error"]),
            "metrics": dict(_last_metrics),
            "history": {
                "samples": float(len(speed_values)),
                "max_measured_speed": max(speed_values, default=0.0),
                "min_measured_speed": min(speed_values, default=0.0),
                "max_abs_speed_error": max(abs_error_values, default=0.0),
            },
        }

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()

    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        measured_speed = float(metrics["measured_speed"])

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
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Revolute Motor", build_panel)],
        info={
            "sx_world": world,
            "hub": hub,
            "rotor": rotor,
            "joint": motor_joint,
            "target_speed": _TARGET_SPEED,
            "max_torque": _MAX_TORQUE,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_revolute_motor",
    title="AVBD Revolute Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A bounded revolute velocity motor drives a free rigid-body hinge.",
    build=build,
)
