"""AVBD fixed-joint contact scene for the World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_BASE_HALF = np.array([0.18, 0.18, 0.18])
_PAYLOAD_HALF = np.array([0.24, 0.16, 0.16])
_GROUND_HALF = np.array([1.6, 0.65, 0.08])
_CAPTURED_OFFSET = np.array([0.72, 0.0, -0.34])


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
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

    transform = _translation(0.5 * (start + end))
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, -9.81))

    ground = world.add_rigid_body(
        "avbd_fixed_joint_ground",
        position=(0.35, 0.0, -float(_GROUND_HALF[2])),
    )
    ground.is_static = True
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = 0.55

    base_position = np.array([0.0, 0.0, 0.50])
    base = world.add_rigid_body(
        "avbd_fixed_joint_base", position=tuple(base_position)
    )
    base.mass = 1.5
    base.set_collision_shape(sx.CollisionShape.box(_BASE_HALF))
    base.friction = 0.55
    base.linear_velocity = (0.45, 0.0, 0.0)

    payload_position = base_position + _CAPTURED_OFFSET
    payload = world.add_rigid_body(
        "avbd_fixed_joint_payload", position=tuple(payload_position)
    )
    payload.mass = 1.0
    payload.set_collision_shape(sx.CollisionShape.box(_PAYLOAD_HALF))
    payload.friction = 0.55
    payload.linear_velocity = (0.45, 0.0, 0.0)
    payload.angular_velocity = (0.0, 0.0, 0.6)

    fixed_joint = world.add_joint(
        base,
        payload,
                sx.JointSpec(
            name="avbd_fixed_joint_base_to_payload",
            type=sx.JointType.FIXED,
        )
    )
    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_rigid_fixed_joint_contact_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.58, 0.60, 0.62),
        name="avbd_fixed_joint_ground_visual",
    )
    bridge.add_rigid_body_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_fixed_joint_base_visual",
    )
    bridge.add_rigid_body_visual(
        payload,
        dart.BoxShape(_full(_PAYLOAD_HALF)),
        (0.91, 0.45, 0.16),
        name="avbd_fixed_joint_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_fixed_joint_connector_visual",
        _connector_transform(base_position, payload_position),
    )
    connector.set_shape(
        dart.BoxShape(np.array([np.linalg.norm(_CAPTURED_OFFSET), 0.035, 0.035]))
    )
    connector.create_visual_aspect().set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(connector)

    def sync_connector() -> None:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        connector.set_transform(_connector_transform(base_pos, payload_pos))

    def replay_sync() -> None:
        bridge.sync()
        sync_connector()

    offset_history: deque[float] = deque(maxlen=120)
    clearance_history: deque[float] = deque(maxlen=120)
    speed_history: deque[float] = deque(maxlen=120)
    contact_history: deque[float] = deque(maxlen=120)
    _last_metrics: dict[str, float] = {}

    def sample_metrics() -> dict[str, float]:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        offset_error = float(
            np.linalg.norm((payload_pos - base_pos) - _CAPTURED_OFFSET)
        )
        clearance = float(payload_pos[2] - _PAYLOAD_HALF[2])
        speed = float(np.linalg.norm(np.asarray(payload.linear_velocity, dtype=float)))
        contact_count = float(len(world.collide()))
        return {
            "captured_offset_error": offset_error,
            "payload_ground_clearance": clearance,
            "payload_speed": speed,
            "contact_count": contact_count,
            "base_x": float(base_pos[0]),
            "payload_x": float(payload_pos[0]),
            "world_time": float(world.time),
        }

    def record_metrics() -> dict[str, float]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        offset_history.append(_last_metrics["captured_offset_error"])
        clearance_history.append(_last_metrics["payload_ground_clearance"])
        speed_history.append(_last_metrics["payload_speed"])
        contact_history.append(_last_metrics["contact_count"])
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        offset_values = list(offset_history)
        clearance_values = list(clearance_history)
        speed_values = list(speed_history)
        contact_values = list(contact_history)
        return {
            "row": "avbd_rigid_fixed_joint_contact",
            "solver": "avbd_rigid_joints",
            "executor": "World.step default",
            "constraint": "fixed_joint_contact_path",
            "related_source_row": "contact",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(fixed_joint.name),
            "fixed_joint_count": float(world.num_joints),
            "captured_offset_error": float(_last_metrics["captured_offset_error"]),
            "payload_ground_clearance": float(
                _last_metrics["payload_ground_clearance"]
            ),
            "payload_speed": float(_last_metrics["payload_speed"]),
            "contact_count": float(_last_metrics["contact_count"]),
            "base_x": float(_last_metrics["base_x"]),
            "payload_x": float(_last_metrics["payload_x"]),
            "metrics": dict(_last_metrics),
            "history": {
                "samples": float(len(offset_values)),
                "max_captured_offset_error": max(offset_values, default=0.0),
                "min_payload_ground_clearance": min(clearance_values, default=0.0),
                "max_payload_speed": max(speed_values, default=0.0),
                "max_contact_count": max(contact_values, default=0.0),
            },
        }

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()
        sync_connector()

    bridge.sync()
    sync_connector()
    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        metrics = _last_metrics or record_metrics()
        offset_error = float(metrics["captured_offset_error"])
        clearance = float(metrics["payload_ground_clearance"])
        speed = float(metrics["payload_speed"])

        found_joint = world.get_joint(fixed_joint.name) or fixed_joint
        builder.text("solver: rigid AVBD fixed joint + default contact")
        builder.text(f"joint: {found_joint.name}")
        builder.text(f"fixed joints: {world.num_joints}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"captured-offset error: {offset_error:.4f} m")
        builder.text(f"payload-ground clearance: {clearance:.4f} m")
        builder.text(f"payload speed: {speed:.3f} m/s")
        builder.plot_lines("Offset error", list(offset_history))
        builder.plot_lines("Ground clearance", list(clearance_history))
        builder.plot_lines("Payload speed", list(speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Fixed Joint Contact", build_panel)],
        info={
            "sx_world": world,
            "joint": fixed_joint,
            "base": base,
            "payload": payload,
            "ground": ground,
            "connector": connector,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_rigid_fixed_joint_contact",
    title="AVBD Fixed Joint Contact (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A rigid fixed joint preserves a captured offset while the payload "
    "slides against contact.",
    build=build,
)
