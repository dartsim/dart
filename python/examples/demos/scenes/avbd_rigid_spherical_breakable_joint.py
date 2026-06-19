"""AVBD breakable spherical point-joint scene for free rigid bodies."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_BASE_HALF = np.array([0.18, 0.18, 0.18])
_PAYLOAD_HALF = np.array([0.24, 0.14, 0.14])
_GROUND_HALF = np.array([1.3, 0.55, 0.08])
_BASE_POS = np.array([0.0, 0.0, 0.88])
_CAPTURED_OFFSET = np.array([0.62, 0.0, 0.0])
_PAYLOAD_PRESTRAIN = np.array([0.22, 0.0, -0.08])
_INITIAL_YAW = 0.42
_BREAK_FORCE = 1.0e-12
_RESET_BREAK_FORCE = 1.0e12


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _transform(position: np.ndarray, yaw: float = 0.0) -> np.ndarray:
    transform = np.eye(4)
    c = float(np.cos(yaw))
    s = float(np.sin(yaw))
    transform[:3, :3] = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    transform[:3, 3] = position
    return transform


def _normalized_or(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return fallback
    return vector / norm


def _connector_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    x_axis = _normalized_or(end - start, np.array([1.0, 0.0, 0.0]))
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ x_axis)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = _normalized_or(np.cross(reference, x_axis), np.array([0.0, 1.0, 0.0]))
    z_axis = np.cross(x_axis, y_axis)

    transform = _transform(0.5 * (start + end))
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, -9.81))

    ground = world.add_rigid_body("avbd_spherical_breakable_ground")
    ground.is_static = True
    ground.transform = _transform(np.array([0.42, 0.0, -float(_GROUND_HALF[2])]))
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = 0.6

    base = world.add_rigid_body(
        "avbd_spherical_breakable_base",
        position=tuple(_BASE_POS),
    )
    base.is_static = True
    base.set_collision_shape(sx.CollisionShape.box(_BASE_HALF))

    payload_position = _BASE_POS + _CAPTURED_OFFSET
    payload = world.add_rigid_body(
        "avbd_spherical_breakable_payload",
        position=tuple(payload_position),
    )
    payload.mass = 1.0
    payload.set_collision_shape(sx.CollisionShape.box(_PAYLOAD_HALF))
    payload.friction = 0.6
    payload.linear_velocity = (0.45, 0.0, -0.25)
    payload.angular_velocity = (0.0, 0.0, 1.1)

    breakable_joint = world.add_joint(
        base,
        payload,
                sx.JointSpec(
            name="avbd_spherical_breakable_base_to_payload",
            type=sx.JointType.SPHERICAL,
        )
    )
    breakable_joint.break_force = _BREAK_FORCE
    payload.transform = _transform(payload_position + _PAYLOAD_PRESTRAIN, _INITIAL_YAW)
    world.enter_simulation_mode()

    captured_payload_transform = _transform(payload_position)
    captured_payload_rotation = captured_payload_transform[:3, :3].copy()

    bridge = WorldRenderBridge(world, name="avbd_rigid_spherical_breakable_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.55, 0.58, 0.60),
        name="avbd_spherical_breakable_ground_visual",
    )
    bridge.add_rigid_body_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_spherical_breakable_base_visual",
    )
    bridge.add_rigid_body_visual(
        payload,
        dart.BoxShape(_full(_PAYLOAD_HALF)),
        (0.24, 0.54, 0.88),
        name="avbd_spherical_breakable_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_spherical_breakable_connector_visual",
        _connector_transform(_BASE_POS, payload_position + _PAYLOAD_PRESTRAIN),
    )
    connector.set_shape(
        dart.BoxShape(np.array([np.linalg.norm(_CAPTURED_OFFSET), 0.035, 0.035]))
    )
    connector_visual = connector.create_visual_aspect()
    connector_visual.set_color([0.72, 0.78, 0.86])
    bridge.render_world.add_simple_frame(connector)

    def sync_connector() -> None:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        connector.set_transform(_connector_transform(base_pos, payload_pos))
        if breakable_joint.is_broken:
            connector_visual.set_color([0.95, 0.18, 0.13])
        else:
            connector_visual.set_color([0.72, 0.78, 0.86])

    def replay_sync() -> None:
        bridge.sync()
        sync_connector()

    anchor_error_history: deque[float] = deque(maxlen=160)
    orientation_error_history: deque[float] = deque(maxlen=160)
    speed_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)
    _last_metrics: dict[str, float | str] = {}

    def sample_metrics() -> dict[str, float | str]:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        anchor_error = float(
            np.linalg.norm((payload_pos - base_pos) - _CAPTURED_OFFSET)
        )
        orientation_drift = float(
            np.linalg.norm(
                np.asarray(payload.rotation, dtype=float) - captured_payload_rotation
            )
        )
        payload_speed = float(
            np.linalg.norm(np.asarray(payload.linear_velocity, dtype=float))
        )
        broken = 1.0 if breakable_joint.is_broken else 0.0
        return {
            "anchor_offset_error": anchor_error,
            "orientation_drift": orientation_drift,
            "payload_speed": payload_speed,
            "payload_height": float(payload_pos[2]),
            "broken": broken,
            "break_force": float(breakable_joint.break_force),
            "world_time": float(world.time),
            "status": "broken" if breakable_joint.is_broken else "intact",
        }

    def record_metrics() -> dict[str, float | str]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        anchor_error_history.append(float(_last_metrics["anchor_offset_error"]))
        orientation_error_history.append(float(_last_metrics["orientation_drift"]))
        speed_history.append(float(_last_metrics["payload_speed"]))
        broken_history.append(float(_last_metrics["broken"]))
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        anchor_values = list(anchor_error_history)
        orientation_values = list(orientation_error_history)
        speed_values = list(speed_history)
        broken_values = list(broken_history)
        return {
            "row": "avbd_rigid_spherical_breakable_joint",
            "solver": "avbd_rigid_joints",
            "executor": "World.step default",
            "constraint": "spherical_break_force_anchor_lifecycle",
            "related_source_row": "rigid_joint_breakage",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(breakable_joint.name),
            "break_force": float(breakable_joint.break_force),
            "reset_break_force": float(_RESET_BREAK_FORCE),
            "anchor_offset_error": float(_last_metrics["anchor_offset_error"]),
            "orientation_drift": float(_last_metrics["orientation_drift"]),
            "payload_speed": float(_last_metrics["payload_speed"]),
            "payload_height": float(_last_metrics["payload_height"]),
            "broken": float(_last_metrics["broken"]),
            "status": str(_last_metrics["status"]),
            "metrics": {
                "anchor_offset_error": float(_last_metrics["anchor_offset_error"]),
                "orientation_drift": float(_last_metrics["orientation_drift"]),
                "payload_speed": float(_last_metrics["payload_speed"]),
                "payload_height": float(_last_metrics["payload_height"]),
                "broken": float(_last_metrics["broken"]),
                "break_force": float(_last_metrics["break_force"]),
                "status": str(_last_metrics["status"]),
            },
            "history": {
                "samples": float(len(anchor_values)),
                "max_anchor_offset_error": max(anchor_values, default=0.0),
                "max_orientation_drift": max(orientation_values, default=0.0),
                "max_payload_speed": max(speed_values, default=0.0),
                "saw_broken": max(broken_values, default=0.0),
            },
        }

    def reset_joint(break_force: float = _RESET_BREAK_FORCE) -> None:
        payload.linear_velocity = (0.0, 0.0, 0.0)
        payload.angular_velocity = (0.0, 0.0, 0.0)
        breakable_joint.break_force = float(break_force)
        breakable_joint.reset_breakage()
        record_metrics()
        sync_connector()

    def rearm_weak_joint() -> None:
        reset_joint(_BREAK_FORCE)

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()
        sync_connector()

    bridge.sync()
    sync_connector()
    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        anchor_error = float(metrics["anchor_offset_error"])
        orientation_error = float(metrics["orientation_drift"])

        builder.text("solver: AVBD spherical point joint with break-force threshold")
        builder.text(f"joint: {breakable_joint.name}")
        builder.text(f"state: {'broken' if breakable_joint.is_broken else 'intact'}")
        builder.text(f"break force: {breakable_joint.break_force:.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"anchor-offset error: {anchor_error:.4f} m")
        builder.text(f"orientation drift: {orientation_error:.4f}")
        if builder.button("Reset joint"):
            reset_joint(_RESET_BREAK_FORCE)
        if builder.button("Re-arm weak joint"):
            rearm_weak_joint()
        builder.plot_lines("Anchor-offset error", list(anchor_error_history))
        builder.plot_lines("Orientation drift", list(orientation_error_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Spherical Breakable Joint", build_panel)],
        info={
            "sx_world": world,
            "joint": breakable_joint,
            "base": base,
            "payload": payload,
            "ground": ground,
            "connector": connector,
            "captured_offset": _CAPTURED_OFFSET.copy(),
            "captured_payload_transform": captured_payload_transform,
            "captured_payload_rotation": captured_payload_rotation,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
            "break_force": _BREAK_FORCE,
            "reset_break_force": _RESET_BREAK_FORCE,
            "reset_joint": reset_joint,
            "rearm_weak_joint": rearm_weak_joint,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_spherical_breakable_joint",
    title="AVBD Spherical Breakable Joint (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A weak spherical point joint breaks, releases, and re-engages only "
    "its captured anchor.",
    build=build,
)
