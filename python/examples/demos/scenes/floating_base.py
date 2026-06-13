"""Floating-base scene: a free body drifts and spins under SE(3) integration.

World owns the physics; WorldRenderBridge mirrors the body's world transform
onto a SimpleFrame box visual each frame so the viewer renders the motion.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.01
_HISTORY = 120


def build() -> SceneSetup:
    world = sx.World()
    # The C++ scene runs in zero-G so the body drifts/spins on its own; match.
    world.gravity = (0.0, 0.0, 0.0)
    robot = world.add_multibody("floating")
    base = robot.add_link("base")
    body = robot.add_link(
        "body",
        parent=base,
        joint=sx.JointSpec(name="floating", type=sx.JointType.FLOATING),
    )
    body.mass = 1.5
    body.inertia = ((0.1, 0.0, 0.0), (0.0, 0.1, 0.0), (0.0, 0.0, 0.1))

    joint = body.parent_joint
    # Free-joint velocity is [linear; angular]: drift along +X while spinning.
    joint.velocity = [1.0, 0.0, 0.0, 0.0, 0.0, 2.0]

    world.time_step = _TIME_STEP
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="floating_base_render")
    bridge.add_link_visual(
        body, dart.BoxShape(np.array([0.4, 0.3, 0.2])),
        (0.95, 0.50, 0.16), name="floating_body_visual")
    bridge.sync()

    linear_speed_history: deque[float] = deque(maxlen=_HISTORY)
    angular_speed_history: deque[float] = deque(maxlen=_HISTORY)
    x_position_history: deque[float] = deque(maxlen=_HISTORY)
    _last_metrics: dict[str, float] = {}

    def sample_metrics() -> dict[str, float]:
        velocity = np.asarray(joint.velocity, dtype=float).reshape(-1)
        translation = np.asarray(body.translation, dtype=float).reshape(3)
        linear_speed = float(np.linalg.norm(velocity[:3]))
        angular_speed = float(np.linalg.norm(velocity[3:]))
        return {
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "body_x": float(translation[0]),
            "body_y": float(translation[1]),
            "body_z": float(translation[2]),
            "spin_command": float(velocity[5]) if velocity.size > 5 else 0.0,
            "world_time": float(world.time),
        }

    def record_metrics() -> dict[str, float]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        linear_speed_history.append(_last_metrics["linear_speed"])
        angular_speed_history.append(_last_metrics["angular_speed"])
        x_position_history.append(_last_metrics["body_x"])
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        linear_values = list(linear_speed_history)
        angular_values = list(angular_speed_history)
        x_values = list(x_position_history)
        return {
            "row": "floating_base",
            "category": "World Rigid Body",
            "related_source_row": "rigid_free_flight",
            "solver": "floating_joint_se3",
            "executor": "World.step default",
            "scope": "broader_floating_joint_drift_spin",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "dofs": float(robot.num_dofs),
            "gravity_z": float(world.gravity[2]),
            "linear_speed": float(_last_metrics["linear_speed"]),
            "angular_speed": float(_last_metrics["angular_speed"]),
            "body_x": float(_last_metrics["body_x"]),
            "body_y": float(_last_metrics["body_y"]),
            "body_z": float(_last_metrics["body_z"]),
            "spin_command": float(_last_metrics["spin_command"]),
            "metrics": dict(_last_metrics),
            "history": {
                "samples": float(len(linear_values)),
                "max_linear_speed": max(linear_values, default=0.0),
                "min_linear_speed": min(linear_values, default=0.0),
                "max_angular_speed": max(angular_values, default=0.0),
                "min_angular_speed": min(angular_values, default=0.0),
                "max_body_x": max(x_values, default=0.0),
                "min_body_x": min(x_values, default=0.0),
            },
        }

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()

    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        velocity = np.asarray(joint.velocity, dtype=float)
        linear_speed = float(metrics["linear_speed"])
        angular_speed = float(metrics["angular_speed"])

        builder.text("solver: floating-base sx world")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text("gravity: zero")
        builder.text(f"linear speed: {linear_speed:.3f} m/s")
        builder.text(f"angular speed: {angular_speed:.3f} rad/s")
        changed, spin = builder.slider("Spin command", float(velocity[5]), -6.0, 6.0)
        if changed:
            next_velocity = velocity.copy()
            next_velocity[5] = float(spin)
            joint.velocity = next_velocity
        builder.plot_lines("Linear speed", list(linear_speed_history))
        builder.plot_lines("Angular speed", list(angular_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Floating Base", build_panel)],
        info={
            "sx_world": world,
            "dofs": robot.num_dofs,
            "replay_sync": bridge.sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="floating_base",
    title="Floating Base",
    category="World Rigid Body",
    summary="A free-floating body drifts and spins under SE(3) integration.",
    build=build,
)
