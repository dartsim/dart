"""Articulated dynamics scene: a 2-DOF arm on the World.

Builds a World (revolute shoulder + universal wrist) for physics, plus a
parallel render World (via WorldRenderBridge) of SimpleFrame box
visuals so the C++ viewer can render the result.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_HISTORY = 120


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _joint_scalar(value: object) -> float:
    values = np.asarray(value, dtype=float).reshape(-1)
    return float(values[0]) if values.size else 0.0


def _set_joint_scalar(joint: object, attr: str, value: float) -> None:
    current = np.asarray(getattr(joint, attr), dtype=float)
    setattr(joint, attr, np.full(current.shape, float(value)))


def build() -> SceneSetup:
    world = sx.World()
    robot = world.add_multibody("arm")
    base = robot.add_link("base")
    upper = robot.add_link(
        "upper_arm",
        parent=base,
        joint=sx.JointSpec(
            name="shoulder",
            type=sx.JointType.REVOLUTE,
            axis=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    upper.mass = 2.0
    upper.inertia = ((0.10, 0.0, 0.0), (0.0, 0.20, 0.0), (0.0, 0.0, 0.30))

    fore = robot.add_link(
        "forearm",
        parent=upper,
        joint=sx.JointSpec(
            name="wrist",
            type=sx.JointType.UNIVERSAL,
            axis=(0.0, 0.0, 1.0),
            axis2=(0.0, 1.0, 0.0),
            transform_from_parent=_translation(1.0, 0.0, 0.0),
        ),
    )
    fore.mass = 1.0
    fore.inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))

    world.enter_simulation_mode()

    # Render bridge: a small box per link tracking the link's world transform.
    bridge = WorldRenderBridge(world, name="articulated_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(np.array([0.18, 0.18, 0.18])),
        (0.65, 0.65, 0.65),
        name="base_visual",
    )
    bridge.add_link_visual(
        upper,
        dart.BoxShape(np.array([0.9, 0.12, 0.12])),
        (0.20, 0.55, 0.90),
        name="upper_visual",
    )
    bridge.add_link_visual(
        fore,
        dart.BoxShape(np.array([0.9, 0.10, 0.10])),
        (0.30, 0.80, 0.45),
        name="fore_visual",
    )
    bridge.sync()

    joint_speed_history: deque[float] = deque(maxlen=_HISTORY)
    forearm_height_history: deque[float] = deque(maxlen=_HISTORY)
    _last_metrics: dict[str, float] = {}

    def sample_metrics() -> dict[str, float]:
        shoulder = upper.parent_joint
        wrist = fore.parent_joint
        shoulder_speed = float(abs(np.asarray(shoulder.velocity, dtype=float)[0]))
        wrist_speed = float(np.linalg.norm(np.asarray(wrist.velocity, dtype=float)))
        forearm_height = float(np.asarray(fore.translation, dtype=float)[2])
        return {
            "shoulder_speed": shoulder_speed,
            "wrist_speed": wrist_speed,
            "max_joint_speed": max(shoulder_speed, wrist_speed),
            "forearm_height": forearm_height,
            "shoulder_damping": _joint_scalar(shoulder.damping_coefficient),
            "wrist_damping": _joint_scalar(wrist.damping_coefficient),
            "shoulder_position": _joint_scalar(shoulder.position),
            "wrist_position_norm": float(
                np.linalg.norm(np.asarray(wrist.position, dtype=float))
            ),
            "world_time": float(world.time),
        }

    def record_metrics() -> dict[str, float]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        joint_speed_history.append(_last_metrics["max_joint_speed"])
        forearm_height_history.append(_last_metrics["forearm_height"])
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        metrics = sample_metrics()
        speed_values = list(joint_speed_history)
        height_values = list(forearm_height_history)
        return {
            "row": "articulated",
            "category": "World Rigid Body",
            "related_source_row": "rigid_multibody_dynamics_terms",
            "solver": "articulated_sx_world",
            "executor": "World.step default",
            "scope": "broader_two_link_arm",
            "time_step_ms": float(world.time_step) * 1000.0,
            "world_time": float(world.time),
            "dofs": float(robot.num_dofs),
            "link_count": 3.0,
            "controls": {
                "shoulder_damping": float(metrics["shoulder_damping"]),
                "wrist_damping": float(metrics["wrist_damping"]),
            },
            "shoulder_speed": float(metrics["shoulder_speed"]),
            "wrist_speed": float(metrics["wrist_speed"]),
            "max_joint_speed": float(metrics["max_joint_speed"]),
            "forearm_height": float(metrics["forearm_height"]),
            "shoulder_damping": float(metrics["shoulder_damping"]),
            "wrist_damping": float(metrics["wrist_damping"]),
            "shoulder_position": float(metrics["shoulder_position"]),
            "wrist_position_norm": float(metrics["wrist_position_norm"]),
            "metrics": dict(metrics),
            "history": {
                "samples": float(len(speed_values)),
                "max_joint_speed": max(speed_values, default=0.0),
                "min_joint_speed": min(speed_values, default=0.0),
                "max_forearm_height": max(height_values, default=0.0),
                "min_forearm_height": min(height_values, default=0.0),
            },
        }

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()

    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        shoulder = upper.parent_joint
        wrist = fore.parent_joint
        metrics = _last_metrics or record_metrics()
        shoulder_speed = float(metrics["shoulder_speed"])
        wrist_speed = float(metrics["wrist_speed"])

        builder.text("solver: articulated sx world")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"dofs: {robot.num_dofs}")
        builder.text(f"shoulder speed: {shoulder_speed:.3f} rad/s")
        builder.text(f"wrist speed: {wrist_speed:.3f} rad/s")
        changed, shoulder_damping = builder.slider(
            "Shoulder damping",
            _joint_scalar(shoulder.damping_coefficient),
            0.0,
            2.0,
        )
        if changed:
            _set_joint_scalar(shoulder, "damping_coefficient", shoulder_damping)
        changed, wrist_damping = builder.slider(
            "Wrist damping", _joint_scalar(wrist.damping_coefficient), 0.0, 2.0
        )
        if changed:
            _set_joint_scalar(wrist, "damping_coefficient", wrist_damping)
        builder.plot_lines("Max joint speed", list(joint_speed_history))
        builder.plot_lines("Forearm height", list(forearm_height_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Articulated", build_panel)],
        info={
            "sx_world": world,
            "dofs": robot.num_dofs,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="articulated",
    title="Articulated Dynamics",
    category="World Rigid Body",
    summary="A revolute + universal arm exercising the World.",
    build=build,
)
