"""AVBD breakable fixed-joint scene for the DART 7 World facade."""

from __future__ import annotations

from collections import deque
from typing import Any

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
_PAYLOAD_PRESTRAIN = np.array([0.18, 0.0, -0.08])
_BREAK_FORCE = 1.0e-12
_RESET_BREAK_FORCE = 1.0e12
_BREAK_FORCE_LOG10_MIN = -12.0
_BREAK_FORCE_LOG10_MAX = 12.0
_BREAK_FORCE_LOG10_DEFAULT = float(np.log10(_BREAK_FORCE))


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


def _break_force_from_log10(value: float) -> float:
    exponent = float(
        np.clip(value, _BREAK_FORCE_LOG10_MIN, _BREAK_FORCE_LOG10_MAX)
    )
    return float(10.0**exponent)


def _break_force_log10(value: float) -> float:
    if not np.isfinite(value) or value <= 0.0:
        return _BREAK_FORCE_LOG10_MIN
    return float(
        np.clip(
            np.log10(value),
            _BREAK_FORCE_LOG10_MIN,
            _BREAK_FORCE_LOG10_MAX,
        )
    )


def build_breakable_joint_scene(
    panel_title: str = "AVBD Breakable Joint",
    *,
    row_id: str = "avbd_rigid_breakable_joint",
    related_source_row: str | None = "rigid_joint_breakage",
) -> SceneSetup:
    world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, -9.81))

    ground = world.add_rigid_body("avbd_breakable_joint_ground")
    ground.is_static = True
    ground.transform = _translation(np.array([0.42, 0.0, -float(_GROUND_HALF[2])]))
    ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
    ground.friction = 0.6

    base = world.add_rigid_body("avbd_breakable_joint_base", position=tuple(_BASE_POS))
    base.is_static = True
    base.set_collision_shape(sx.CollisionShape.box(_BASE_HALF))

    payload_position = _BASE_POS + _CAPTURED_OFFSET
    payload = world.add_rigid_body(
        "avbd_breakable_joint_payload",
        position=tuple(payload_position),
    )
    payload.mass = 1.0
    payload.set_collision_shape(sx.CollisionShape.box(_PAYLOAD_HALF))
    payload.friction = 0.6
    payload.linear_velocity = (0.65, 0.0, -0.35)

    breakable_joint = world.add_rigid_body_fixed_joint(
        "avbd_breakable_joint_base_to_payload", base, payload
    )
    breakable_joint.break_force = _BREAK_FORCE
    payload.transform = _translation(payload_position + _PAYLOAD_PRESTRAIN)
    world.enter_simulation_mode()
    captured_payload_transform = _translation(payload_position)
    captured_payload_rotation = captured_payload_transform[:3, :3].copy()

    bridge = WorldRenderBridge(world, name="avbd_rigid_breakable_joint_render")
    bridge.add_rigid_body_visual(
        ground,
        dart.BoxShape(_full(_GROUND_HALF)),
        (0.55, 0.58, 0.60),
        name="avbd_breakable_joint_ground_visual",
    )
    bridge.add_rigid_body_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_breakable_joint_base_visual",
    )
    bridge.add_rigid_body_visual(
        payload,
        dart.BoxShape(_full(_PAYLOAD_HALF)),
        (0.91, 0.45, 0.16),
        name="avbd_breakable_joint_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_breakable_joint_connector_visual",
        _connector_transform(_BASE_POS, payload_position + _PAYLOAD_PRESTRAIN),
    )
    connector.set_shape(
        dart.BoxShape(np.array([np.linalg.norm(_CAPTURED_OFFSET), 0.035, 0.035]))
    )
    connector_visual = connector.create_visual_aspect()
    connector_visual.set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(connector)

    def sync_connector() -> None:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        connector.set_transform(_connector_transform(base_pos, payload_pos))
        if breakable_joint.is_broken:
            connector_visual.set_color([0.95, 0.18, 0.13])
        else:
            connector_visual.set_color([0.78, 0.78, 0.70])

    def replay_sync() -> None:
        bridge.sync()
        sync_connector()

    offset_history: deque[float] = deque(maxlen=160)
    speed_history: deque[float] = deque(maxlen=160)
    release_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)
    _last_metrics: dict[str, float | str] = {}
    break_force_log10 = [_BREAK_FORCE_LOG10_DEFAULT]

    def sample_metrics() -> dict[str, float | str]:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        offset_error = float(np.linalg.norm((payload_pos - base_pos) - _CAPTURED_OFFSET))
        speed = float(np.linalg.norm(np.asarray(payload.linear_velocity, dtype=float)))
        broken = 1.0 if breakable_joint.is_broken else 0.0
        return {
            "captured_offset_error": offset_error,
            "payload_speed": speed,
            "payload_height": float(payload_pos[2]),
            "payload_release_distance": float(
                np.linalg.norm(payload_pos - (payload_position + _PAYLOAD_PRESTRAIN))
            ),
            "broken": broken,
            "break_force": float(breakable_joint.break_force),
            "break_force_log10": _break_force_log10(
                float(breakable_joint.break_force)
            ),
            "world_time": float(world.time),
            "status": "broken" if breakable_joint.is_broken else "intact",
        }

    def record_metrics() -> dict[str, float | str]:
        _last_metrics.clear()
        _last_metrics.update(sample_metrics())
        offset_history.append(float(_last_metrics["captured_offset_error"]))
        speed_history.append(float(_last_metrics["payload_speed"]))
        release_history.append(float(_last_metrics["payload_release_distance"]))
        broken_history.append(float(_last_metrics["broken"]))
        return _last_metrics

    def capture_metrics() -> dict[str, object]:
        if not _last_metrics:
            record_metrics()
        offset_values = list(offset_history)
        speed_values = list(speed_history)
        release_values = list(release_history)
        broken_values = list(broken_history)
        captured_offset = float(np.linalg.norm(_CAPTURED_OFFSET))
        captured_offset_error = float(_last_metrics["captured_offset_error"])
        payload_speed = float(_last_metrics["payload_speed"])
        payload_height = float(_last_metrics["payload_height"])
        payload_release_distance = float(_last_metrics["payload_release_distance"])
        broken = float(_last_metrics["broken"])
        break_force = float(_last_metrics["break_force"])
        active_break_force_log10 = float(_last_metrics["break_force_log10"])
        status = str(_last_metrics["status"])
        metrics_payload: dict[str, object] = {
            "row": row_id,
            "comparison_axis": "fixed_break_force_lifecycle",
            "solver": "avbd_rigid_joints",
            "constraint": "fixed_break_force_lifecycle",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(world.time),
            "joint_name": str(breakable_joint.name),
            "held_fixed": {
                "base": "static",
                "captured_offset_m": captured_offset,
                "ground_friction": float(ground.friction),
                "payload_mass": float(payload.mass),
                "solver": "AVBD rigid joints",
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "break_force": break_force,
                "break_force_log10": active_break_force_log10,
                "break_force_log10_range": [
                    _BREAK_FORCE_LOG10_MIN,
                    _BREAK_FORCE_LOG10_MAX,
                ],
                "reset_break_force": float(_RESET_BREAK_FORCE),
            },
            "breakage_payload_release_distance": payload_release_distance,
            "breakage_broken": broken,
            "breakage_captured_offset_error": captured_offset_error,
            "breakage_payload_speed": payload_speed,
            "breakage_status": status,
            "metrics": {
                "captured_offset_error": captured_offset_error,
                "payload_speed": payload_speed,
                "payload_height": payload_height,
                "payload_release_distance": payload_release_distance,
                "broken": broken,
                "break_force": break_force,
                "break_force_log10": active_break_force_log10,
                "status": status,
            },
            "history": {
                "samples": float(len(offset_values)),
                "max_captured_offset_error": max(offset_values, default=0.0),
                "max_payload_speed": max(speed_values, default=0.0),
                "max_payload_release_distance": max(release_values, default=0.0),
                "saw_broken": max(broken_values, default=0.0),
            },
        }
        if related_source_row is not None:
            metrics_payload["related_source_row"] = related_source_row
        return metrics_payload

    def set_break_force_log10(value: float) -> None:
        break_force_log10[0] = float(
            np.clip(value, _BREAK_FORCE_LOG10_MIN, _BREAK_FORCE_LOG10_MAX)
        )
        breakable_joint.break_force = _break_force_from_log10(break_force_log10[0])
        record_metrics()
        sync_connector()

    def capture_replay_state() -> dict[str, Any]:
        if not _last_metrics:
            record_metrics()
        return {
            "controls": {
                "break_force": float(breakable_joint.break_force),
                "break_force_log10": float(break_force_log10[0]),
            },
            "offset_history": list(offset_history),
            "speed_history": list(speed_history),
            "release_history": list(release_history),
            "broken_history": list(broken_history),
            "last_metrics": dict(_last_metrics),
        }

    def latest_history_value(snapshot: dict[str, Any], key: str) -> float | None:
        history = snapshot.get(key, [])
        try:
            if history:
                return float(history[-1])
        except (TypeError, ValueError, IndexError):
            return None
        return None

    def metric_value(snapshot: dict[str, Any], key: str) -> float | None:
        metrics = snapshot.get("last_metrics", {})
        if not isinstance(metrics, dict):
            return None
        try:
            return float(metrics.get(key, 0.0))
        except (TypeError, ValueError):
            return None

    def replay_timeline_signal(snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        release = latest_history_value(snapshot, "release_history")
        if release is not None:
            return release
        return metric_value(snapshot, "payload_release_distance") or 0.0

    def replay_timeline_marker(snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        broken = latest_history_value(snapshot, "broken_history")
        if broken is not None and broken >= 1.0:
            return 1.0
        release = latest_history_value(snapshot, "release_history")
        if release is not None and release >= 0.010:
            return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            if metrics.get("status") == "broken":
                return 1.0
            try:
                if float(metrics.get("broken", 0.0)) >= 1.0:
                    return 1.0
                if float(metrics.get("payload_release_distance", 0.0)) >= 0.010:
                    return 1.0
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def restore_history(history: deque[float], values: Any) -> None:
        history.clear()
        try:
            history.extend(float(value) for value in values)
        except (TypeError, ValueError):
            return

    def restore_replay_state(state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        try:
            break_force_log10[0] = _break_force_log10(
                float(controls.get("break_force", breakable_joint.break_force))
            )
            if "break_force_log10" in controls:
                break_force_log10[0] = float(
                    np.clip(
                        float(controls["break_force_log10"]),
                        _BREAK_FORCE_LOG10_MIN,
                        _BREAK_FORCE_LOG10_MAX,
                    )
                )
            breakable_joint.break_force = _break_force_from_log10(
                break_force_log10[0]
            )
        except (TypeError, ValueError, AttributeError):
            pass
        restore_history(offset_history, state.get("offset_history", []))
        restore_history(speed_history, state.get("speed_history", []))
        restore_history(release_history, state.get("release_history", []))
        restore_history(broken_history, state.get("broken_history", []))
        _last_metrics.clear()
        metrics = state.get("last_metrics", {})
        if isinstance(metrics, dict):
            for key, value in metrics.items():
                try:
                    _last_metrics[str(key)] = float(value)
                except (TypeError, ValueError):
                    _last_metrics[str(key)] = str(value)
        replay_sync()

    def reset_joint(break_force: float | None = None) -> None:
        if break_force is None:
            target_break_force = _break_force_from_log10(break_force_log10[0])
        else:
            target_break_force = float(break_force)
            break_force_log10[0] = _break_force_log10(target_break_force)
        payload.linear_velocity = (0.0, 0.0, 0.0)
        payload.angular_velocity = (0.0, 0.0, 0.0)
        breakable_joint.break_force = target_break_force
        breakable_joint.reset_breakage()
        record_metrics()
        sync_connector()

    def rearm_weak_joint() -> None:
        break_force_log10[0] = _BREAK_FORCE_LOG10_DEFAULT
        reset_joint(_BREAK_FORCE)

    def pre_step() -> None:
        bridge.pre_step()
        record_metrics()
        sync_connector()

    def reset_breakage_lifecycle() -> None:
        breakable_joint.reset_breakage()
        base.transform = _translation(_BASE_POS)
        base.linear_velocity = (0.0, 0.0, 0.0)
        base.angular_velocity = (0.0, 0.0, 0.0)
        base.clear_force()
        base.clear_torque()
        payload.transform = _translation(payload_position + _PAYLOAD_PRESTRAIN)
        payload.linear_velocity = (0.65, 0.0, -0.35)
        payload.angular_velocity = (0.0, 0.0, 0.0)
        payload.clear_force()
        payload.clear_torque()
        world.time = 0.0
        try:
            world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        offset_history.clear()
        speed_history.clear()
        release_history.clear()
        broken_history.clear()
        world.update_kinematics()
        record_metrics()
        replay_sync()

    bridge.sync()
    sync_connector()
    record_metrics()

    def build_panel(builder: object, context: object) -> None:
        if builder.button("Reset breakage lifecycle"):
            reset_breakage_lifecycle()

        metrics = _last_metrics or record_metrics()
        offset_error = float(metrics["captured_offset_error"])
        speed = float(metrics["payload_speed"])
        captured_offset = float(np.linalg.norm(_CAPTURED_OFFSET))

        changed_break_force, next_break_force_log10 = builder.slider(
            "Break force log10(N)",
            float(break_force_log10[0]),
            _BREAK_FORCE_LOG10_MIN,
            _BREAK_FORCE_LOG10_MAX,
        )
        if changed_break_force:
            set_break_force_log10(float(next_break_force_log10))
            metrics = _last_metrics or record_metrics()

        builder.text("comparison axis: fixed break-force lifecycle")
        builder.text(
            "held fixed: AVBD rigid joints | static base | payload mass "
            f"{float(payload.mass):.1f} | offset {captured_offset:.2f} m | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: AVBD fixed joint with break-force threshold")
        builder.text(f"joint: {breakable_joint.name}")
        builder.text(f"state: {'broken' if breakable_joint.is_broken else 'intact'}")
        builder.text(f"break force: {float(breakable_joint.break_force):.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"captured-offset error: {offset_error:.4f} m")
        builder.text(f"payload speed: {speed:.3f} m/s")
        if builder.button("Reset with current threshold"):
            reset_joint()
        if builder.button("Reset joint"):
            reset_joint(_RESET_BREAK_FORCE)
        if builder.button("Re-arm weak joint"):
            rearm_weak_joint()
        builder.plot_lines("Offset error", list(offset_history))
        builder.plot_lines("Payload speed", list(speed_history))
        builder.plot_lines("Release distance", list(release_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(panel_title, build_panel)],
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
            CAPTURE_METRICS_INFO_KEY: capture_metrics,
            "replay_capture_state": capture_replay_state,
            "replay_restore_state": restore_replay_state,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
            "replay_timeline": {
                "signal_label": "Payload release distance",
                "signal": replay_timeline_signal,
                "markers": replay_timeline_marker,
            },
            "reset_breakage_lifecycle": reset_breakage_lifecycle,
            "break_force": _BREAK_FORCE,
            "reset_break_force": _RESET_BREAK_FORCE,
            "reset_joint": reset_joint,
            "rearm_weak_joint": rearm_weak_joint,
            "set_break_force_log10": set_break_force_log10,
        },
    )


def build() -> SceneSetup:
    return build_breakable_joint_scene()


SCENE = PythonDemoScene(
    id="avbd_rigid_breakable_joint",
    title="AVBD Breakable Joint (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A weak fixed joint breaks, releases, and can re-engage its "
    "captured pose.",
    build=build,
)
