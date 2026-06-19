"""AVBD port of the avbd-demo2d motor source scene."""

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
_TARGET_SPEED = 20.0
_MAX_TORQUE = 50.0
_GROUND_POS = np.array([0.0, -10.0, 0.0])
_BAR_POS = np.array([0.0, 0.0, 0.0])
_GROUND_SIZE = np.array([100.0, 0.5, 0.2])
_BAR_SIZE = np.array([5.0, 0.5, 0.2])
_SOURCE_ROW: dict[str, Any] = {
    "demo": "avbd-demo2d",
    "repository": "https://github.com/savant117/avbd-demo2d",
    "revision": "74699a11f858",
    "dimension": 2,
    "scene_index": 17,
    "scene_name": "Motor",
    "scene_count": 19,
    "scene_builder": "sceneMotor",
    "solver_defaults": {
        "time_step": _TIME_STEP,
        "gravity_axis": "y",
        "gravity": _GRAVITY,
        "iterations": 10,
    },
    "source_shapes": {
        "ground": {"size": (100.0, 0.5), "mass": 0.0, "friction": 0.5},
        "bar": {"size": (5.0, 0.5), "mass": 1.0, "friction": 0.5},
    },
    "source_constraints": {
        "world_joint": {
            "body_a": None,
            "body_b": "bar",
            "anchor_a": (0.0, 0.0),
            "anchor_b": (0.0, 0.0),
            "stiffness": ("infinity", "infinity", 0.0),
        },
        "motor": {
            "body_a": None,
            "body_b": "bar",
            "speed": _TARGET_SPEED,
            "max_torque": _MAX_TORQUE,
        },
    },
    "expected_counts": {
        "rigid_bodies": 2,
        "joints": 1,
        "motors": 1,
    },
}


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
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


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, _GRAVITY, 0.0))

    ground = world.add_rigid_body("avbd_demo2d_motor_ground", position=tuple(_GROUND_POS))
    ground.is_static = True

    bar = world.add_rigid_body("avbd_demo2d_motor_bar", position=tuple(_BAR_POS))
    bar.mass = 1.0

    motor_joint = world.add_joint(
        ground,
        bar,
                sx.JointSpec(
            name="avbd_demo2d_motor_pin",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 0.0, 1.0),
        )
    )
    motor_joint.actuator_type = sx.ActuatorType.VELOCITY
    motor_joint.command_velocity = [_TARGET_SPEED]
    motor_joint.set_effort_limits([-_MAX_TORQUE], [_MAX_TORQUE])

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_demo2d_motor_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_GROUND_SIZE),
        (0.30, 0.31, 0.34),
        name="avbd_demo2d_motor_ground_visual",
    )
    bridge.add_rigid_body_visual(
        bar,
        dart.BoxShape(_BAR_SIZE),
        (0.92, 0.56, 0.18),
        name="avbd_demo2d_motor_bar_visual",
    )

    axis = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_demo2d_motor_axis_visual",
        _translation(_BAR_POS + np.array([0.0, 0.0, 0.36])),
    )
    axis.set_shape(dart.CylinderShape(0.035, 0.72))
    axis.create_visual_aspect().set_color([0.23, 0.64, 0.88])
    bridge.render_world.add_simple_frame(axis)
    bridge.sync()

    speed_history: deque[float] = deque(maxlen=160)
    y_error_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        angular_velocity = np.asarray(bar.angular_velocity, dtype=float).reshape(3)
        measured_speed = float(angular_velocity[2])
        y_error = float(np.asarray(bar.translation, dtype=float).reshape(3)[1])
        speed_history.append(measured_speed)
        y_error_history.append(y_error)

        builder.text("source corpus: avbd-demo2d Motor")
        builder.text("source scene: sceneMotor, index 17 of 19")
        builder.text(f"target speed: {_TARGET_SPEED:.1f} rad/s")
        builder.text(f"max torque: {_MAX_TORQUE:.1f} N m")
        builder.text(f"measured speed: {measured_speed:.3f} rad/s")
        builder.text(f"bar y: {y_error:.4f} m")
        builder.text(f"world time: {world.time:.3f} s")
        builder.plot_lines("Angular speed", list(speed_history))
        builder.plot_lines("Pinned y", list(y_error_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Demo2D Motor", build_panel)],
        info={
            "sx_world": world,
            "ground": ground,
            "bar": bar,
            "joint": motor_joint,
            "source_demo_row": "avbd-demo2d motor",
            "source_demo_reference": _source_row(),
            "target_speed": _TARGET_SPEED,
            "max_torque": _MAX_TORQUE,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_demo2d_motor",
    title="AVBD Demo2D Motor (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A source-demo Motor row port using a pinned rigid bar and AVBD "
    "revolute velocity actuator.",
    build=build,
)
