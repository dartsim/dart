"""Rigid collision cast-query visualizer for ray, sphere, and capsule probes."""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_HISTORY = 180
_TARGET_RADIUS = 0.18
_RAY_START_X = -1.10
_RAY_END_X = 1.10
_TARGET_Z = 0.25
_RAY_Z = _TARGET_Z
_SWEEP_Y = 0.50
_CAPSULE_SWEEP_Y = 0.68
_CAPSULE_RADIUS = 0.12
_CAPSULE_HEIGHT = 0.86
_HIDDEN_Z = -10.0


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _hidden_transform() -> np.ndarray:
    return _translation((0.0, 0.0, _HIDDEN_Z))


def _x_aligned_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    start = np.asarray(start, dtype=float).reshape(3)
    end = np.asarray(end, dtype=float).reshape(3)
    vector = end - start
    length = float(np.linalg.norm(vector))
    x_axis = vector / length if length > 1.0e-12 else np.array([1.0, 0.0, 0.0])
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(np.dot(x_axis, reference))) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(reference, x_axis)
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1.0e-12:
        y_axis = np.array([0.0, 1.0, 0.0])
    else:
        y_axis /= y_norm
    z_axis = np.cross(x_axis, y_axis)

    transform = np.eye(4)
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    transform[:3, 3] = 0.5 * (start + end)
    return transform


def _y_axis_capsule_transform(
    position: np.ndarray | tuple[float, float, float],
) -> np.ndarray:
    transform = _translation(position)
    transform[:3, 0] = np.array([1.0, 0.0, 0.0])
    transform[:3, 1] = np.array([0.0, 0.0, -1.0])
    transform[:3, 2] = np.array([0.0, 1.0, 0.0])
    return transform


def _make_frame(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float, float] | tuple[float, float, float],
    transform: np.ndarray,
) -> Any:
    frame = dart.SimpleFrame(dart.gui.world_render_frame(), name, transform)
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(color)
    render_world.add_simple_frame(frame)
    return frame


class _RigidCollisionCasts:
    def __init__(self) -> None:
        self.ray_offset_y = 0.0
        self.enable_all_ray_hits = True
        self.sweep_radius = 0.35
        self.capsule_offset_y = _CAPSULE_SWEEP_Y
        self.capsule_radius = _CAPSULE_RADIUS
        self.capsule_height = _CAPSULE_HEIGHT

        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True
        self.world.enter_simulation_mode()
        self.bridge = WorldRenderBridge(self.world, name="rigid_collision_casts")

        self.detector = dart.DartCollisionDetector()
        self.group = self.detector.create_collision_group()
        self.target_frames: list[Any] = []
        self._make_targets()
        self._make_query_visuals()

        self._ray_hit_history: deque[float] = deque(maxlen=_HISTORY)
        self._sphere_cast_toi_history: deque[float] = deque(maxlen=_HISTORY)
        self._sweep_margin_history: deque[float] = deque(maxlen=_HISTORY)
        self._capsule_cast_toi_history: deque[float] = deque(maxlen=_HISTORY)
        self._capsule_margin_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, Any] = {}
        self._record_metrics()
        self._sync()

    def _make_targets(self) -> None:
        targets = [
            ("near_sensor_target", -0.35, (0.30, 0.57, 0.90, 1.0)),
            ("far_sensor_target", 0.45, (0.86, 0.42, 0.23, 1.0)),
        ]
        for name, x, color in targets:
            frame = _make_frame(
                self.bridge.render_world,
                name,
                dart.SphereShape(_TARGET_RADIUS),
                color,
                _translation((x, 0.0, _TARGET_Z)),
            )
            self.group.add_shape_frame(frame)
            self.target_frames.append(frame)

    def _make_query_visuals(self) -> None:
        ray_length = _RAY_END_X - _RAY_START_X
        self.ray_segment = _make_frame(
            self.bridge.render_world,
            "raycast_segment",
            dart.BoxShape(np.array([ray_length, 0.014, 0.014])),
            (0.18, 0.45, 0.95, 0.55),
            _hidden_transform(),
        )
        self.sweep_segment = _make_frame(
            self.bridge.render_world,
            "sphere_cast_segment",
            dart.BoxShape(np.array([ray_length, 0.018, 0.018])),
            (0.22, 0.72, 0.42, 0.55),
            _hidden_transform(),
        )
        self.capsule_segment = _make_frame(
            self.bridge.render_world,
            "capsule_cast_segment",
            dart.BoxShape(np.array([ray_length, 0.018, 0.018])),
            (0.62, 0.38, 0.88, 0.55),
            _hidden_transform(),
        )
        self.ray_hit_marker = _make_frame(
            self.bridge.render_world,
            "raycast_first_hit_marker",
            dart.SphereShape(0.035),
            (0.98, 0.82, 0.18, 1.0),
            _hidden_transform(),
        )
        self.ray_normal_marker = _make_frame(
            self.bridge.render_world,
            "raycast_normal_marker",
            dart.BoxShape(np.array([0.16, 0.012, 0.012])),
            (0.98, 0.82, 0.18, 1.0),
            _hidden_transform(),
        )
        self.sweep_start_marker = _make_frame(
            self.bridge.render_world,
            "sphere_cast_start_probe",
            dart.SphereShape(self.sweep_radius),
            (0.22, 0.72, 0.42, 0.20),
            _hidden_transform(),
        )
        self.sweep_hit_marker = _make_frame(
            self.bridge.render_world,
            "sphere_cast_first_hit_center",
            dart.SphereShape(self.sweep_radius),
            (0.22, 0.72, 0.42, 0.32),
            _hidden_transform(),
        )
        self.sweep_contact_marker = _make_frame(
            self.bridge.render_world,
            "sphere_cast_first_contact_marker",
            dart.SphereShape(0.030),
            (0.98, 0.82, 0.18, 1.0),
            _hidden_transform(),
        )
        self.capsule_start_marker = _make_frame(
            self.bridge.render_world,
            "capsule_cast_start_probe",
            dart.CapsuleShape(self.capsule_radius, self.capsule_height),
            (0.62, 0.38, 0.88, 0.20),
            _hidden_transform(),
        )
        self.capsule_hit_marker = _make_frame(
            self.bridge.render_world,
            "capsule_cast_first_hit_probe",
            dart.CapsuleShape(self.capsule_radius, self.capsule_height),
            (0.62, 0.38, 0.88, 0.32),
            _hidden_transform(),
        )
        self.capsule_contact_marker = _make_frame(
            self.bridge.render_world,
            "capsule_cast_first_contact_marker",
            dart.SphereShape(0.030),
            (0.98, 0.82, 0.18, 1.0),
            _hidden_transform(),
        )

    def _ray_start(self) -> np.ndarray:
        return np.array([_RAY_START_X, float(self.ray_offset_y), _RAY_Z])

    def _ray_end(self) -> np.ndarray:
        return np.array([_RAY_END_X, float(self.ray_offset_y), _RAY_Z])

    def _sweep_start(self) -> np.ndarray:
        return np.array([_RAY_START_X, _SWEEP_Y, _RAY_Z])

    def _sweep_end(self) -> np.ndarray:
        return np.array([_RAY_END_X, _SWEEP_Y, _RAY_Z])

    def _capsule_start_center(self) -> np.ndarray:
        return np.array([_RAY_START_X, float(self.capsule_offset_y), _RAY_Z])

    def _capsule_end_center(self) -> np.ndarray:
        return np.array([_RAY_END_X, float(self.capsule_offset_y), _RAY_Z])

    def _capsule_start_transform(self) -> np.ndarray:
        return _y_axis_capsule_transform(self._capsule_start_center())

    def _capsule_end_transform(self) -> np.ndarray:
        return _y_axis_capsule_transform(self._capsule_end_center())

    def _target_name(self, hit: Any) -> str:
        return str(hit.m_collision_object.get_shape_frame().get_name())

    def _record_metrics(self) -> None:
        ray_option = dart.RaycastOption()
        ray_option.m_enable_all_hits = bool(self.enable_all_ray_hits)
        ray_result = self.group.raycast_result(self._ray_start(), self._ray_end(), ray_option)
        ray_hits = sorted(
            list(ray_result.m_ray_hits),
            key=lambda hit: (float(hit.m_fraction), self._target_name(hit)),
        )

        sweep_option = dart.ContinuousCollisionOption()
        sweep_option.m_enable_all_hits = True
        sweep_result = self.group.sphere_cast_result(
            self._sweep_start(),
            self._sweep_end(),
            float(self.sweep_radius),
            sweep_option,
        )
        sweep_hits = sorted(
            list(sweep_result.m_hits),
            key=lambda hit: (float(hit.m_time_of_impact), self._target_name(hit)),
        )
        capsule_result = self.group.capsule_cast_result(
            self._capsule_start_transform(),
            self._capsule_end_transform(),
            float(self.capsule_radius),
            float(self.capsule_height),
            sweep_option,
        )
        capsule_hits = sorted(
            list(capsule_result.m_hits),
            key=lambda hit: (float(hit.m_time_of_impact), self._target_name(hit)),
        )

        first_ray = ray_hits[0] if ray_hits else None
        first_sweep = sweep_hits[0] if sweep_hits else None
        first_capsule = capsule_hits[0] if capsule_hits else None
        sweep_margin = float(self.sweep_radius + _TARGET_RADIUS - abs(_SWEEP_Y))
        sweep_center = (
            self._sweep_start()
            + float(first_sweep.m_time_of_impact) * (self._sweep_end() - self._sweep_start())
            if first_sweep is not None
            else np.zeros(3)
        )
        capsule_edge_gap = max(
            0.0, abs(float(self.capsule_offset_y)) - 0.5 * float(self.capsule_height)
        )
        capsule_margin = float(self.capsule_radius + _TARGET_RADIUS - capsule_edge_gap)
        capsule_center = (
            self._capsule_start_center()
            + float(first_capsule.m_time_of_impact)
            * (self._capsule_end_center() - self._capsule_start_center())
            if first_capsule is not None
            else np.zeros(3)
        )

        self._last_metrics = {
            "ray": {
                "hit_count": len(ray_hits),
                "first_target": self._target_name(first_ray) if first_ray else "none",
                "first_fraction": float(first_ray.m_fraction) if first_ray else 0.0,
                "first_point": (
                    np.asarray(first_ray.m_point, dtype=float) if first_ray else np.zeros(3)
                ),
                "first_normal": (
                    np.asarray(first_ray.m_normal, dtype=float)
                    if first_ray
                    else np.zeros(3)
                ),
                "all_fractions": [float(hit.m_fraction) for hit in ray_hits],
                "all_targets": [self._target_name(hit) for hit in ray_hits],
                "offset_y": float(self.ray_offset_y),
            },
            "sphere_cast": {
                "hit_count": len(sweep_hits),
                "first_target": self._target_name(first_sweep) if first_sweep else "none",
                "first_toi": (
                    float(first_sweep.m_time_of_impact) if first_sweep else 0.0
                ),
                "first_point": (
                    np.asarray(first_sweep.m_point, dtype=float)
                    if first_sweep
                    else np.zeros(3)
                ),
                "first_normal": (
                    np.asarray(first_sweep.m_normal, dtype=float)
                    if first_sweep
                    else np.zeros(3)
                ),
                "first_center": sweep_center,
                "all_toi": [float(hit.m_time_of_impact) for hit in sweep_hits],
                "all_targets": [self._target_name(hit) for hit in sweep_hits],
                "radius": float(self.sweep_radius),
                "margin": sweep_margin,
            },
            "capsule_cast": {
                "hit_count": len(capsule_hits),
                "first_target": (
                    self._target_name(first_capsule) if first_capsule else "none"
                ),
                "first_toi": (
                    float(first_capsule.m_time_of_impact)
                    if first_capsule
                    else 0.0
                ),
                "first_point": (
                    np.asarray(first_capsule.m_point, dtype=float)
                    if first_capsule
                    else np.zeros(3)
                ),
                "first_normal": (
                    np.asarray(first_capsule.m_normal, dtype=float)
                    if first_capsule
                    else np.zeros(3)
                ),
                "first_center": capsule_center,
                "all_toi": [float(hit.m_time_of_impact) for hit in capsule_hits],
                "all_targets": [self._target_name(hit) for hit in capsule_hits],
                "radius": float(self.capsule_radius),
                "height": float(self.capsule_height),
                "offset_y": float(self.capsule_offset_y),
                "margin": capsule_margin,
            },
        }
        self._ray_hit_history.append(float(len(ray_hits)))
        self._sphere_cast_toi_history.append(
            float(first_sweep.m_time_of_impact) if first_sweep else 0.0
        )
        self._sweep_margin_history.append(sweep_margin)
        self._capsule_cast_toi_history.append(
            float(first_capsule.m_time_of_impact) if first_capsule else 0.0
        )
        self._capsule_margin_history.append(capsule_margin)
        self._update_query_visuals()

    def _update_query_visuals(self) -> None:
        ray_start = self._ray_start()
        ray_end = self._ray_end()
        sweep_start = self._sweep_start()
        sweep_end = self._sweep_end()
        capsule_start = self._capsule_start_center()
        capsule_end = self._capsule_end_center()
        self.ray_segment.set_transform(_x_aligned_transform(ray_start, ray_end))
        self.sweep_segment.set_transform(_x_aligned_transform(sweep_start, sweep_end))
        self.capsule_segment.set_transform(
            _x_aligned_transform(capsule_start, capsule_end)
        )
        self.sweep_start_marker.set_shape(dart.SphereShape(float(self.sweep_radius)))
        self.sweep_start_marker.set_transform(_translation(sweep_start))
        capsule_shape = dart.CapsuleShape(
            float(self.capsule_radius), float(self.capsule_height)
        )
        self.capsule_start_marker.set_shape(capsule_shape)
        self.capsule_start_marker.set_transform(
            _y_axis_capsule_transform(capsule_start)
        )

        if not self._last_metrics:
            return

        ray = self._last_metrics["ray"]
        if int(ray["hit_count"]) > 0:
            point = np.asarray(ray["first_point"], dtype=float)
            normal = np.asarray(ray["first_normal"], dtype=float)
            self.ray_hit_marker.set_transform(_translation(point))
            self.ray_normal_marker.set_transform(
                _x_aligned_transform(point, point + 0.16 * normal)
            )
        else:
            self.ray_hit_marker.set_transform(_hidden_transform())
            self.ray_normal_marker.set_transform(_hidden_transform())

        sphere_cast = self._last_metrics["sphere_cast"]
        self.sweep_hit_marker.set_shape(dart.SphereShape(float(self.sweep_radius)))
        if int(sphere_cast["hit_count"]) > 0:
            self.sweep_hit_marker.set_transform(
                _translation(np.asarray(sphere_cast["first_center"], dtype=float))
            )
            self.sweep_contact_marker.set_transform(
                _translation(np.asarray(sphere_cast["first_point"], dtype=float))
            )
        else:
            self.sweep_hit_marker.set_transform(_hidden_transform())
            self.sweep_contact_marker.set_transform(_hidden_transform())

        capsule_cast = self._last_metrics["capsule_cast"]
        self.capsule_hit_marker.set_shape(
            dart.CapsuleShape(float(self.capsule_radius), float(self.capsule_height))
        )
        if int(capsule_cast["hit_count"]) > 0:
            self.capsule_hit_marker.set_transform(
                _y_axis_capsule_transform(
                    np.asarray(capsule_cast["first_center"], dtype=float)
                )
            )
            self.capsule_contact_marker.set_transform(
                _translation(np.asarray(capsule_cast["first_point"], dtype=float))
            )
        else:
            self.capsule_hit_marker.set_transform(_hidden_transform())
            self.capsule_contact_marker.set_transform(_hidden_transform())

    def _sync(self) -> None:
        self.bridge.sync()
        self._update_query_visuals()

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "ray_offset_y": float(self.ray_offset_y),
                "enable_all_ray_hits": bool(self.enable_all_ray_hits),
                "sweep_radius": float(self.sweep_radius),
                "capsule_offset_y": float(self.capsule_offset_y),
                "capsule_radius": float(self.capsule_radius),
                "capsule_height": float(self.capsule_height),
            },
            "ray_hit_history": list(self._ray_hit_history),
            "sphere_cast_toi_history": list(self._sphere_cast_toi_history),
            "sweep_margin_history": list(self._sweep_margin_history),
            "capsule_cast_toi_history": list(self._capsule_cast_toi_history),
            "capsule_margin_history": list(self._capsule_margin_history),
            "last_metrics": self._serialize_metrics(self._last_metrics),
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.ray_offset_y = float(controls.get("ray_offset_y", self.ray_offset_y))
        self.enable_all_ray_hits = bool(
            controls.get("enable_all_ray_hits", self.enable_all_ray_hits)
        )
        self.sweep_radius = float(controls.get("sweep_radius", self.sweep_radius))
        self.capsule_offset_y = float(
            controls.get("capsule_offset_y", self.capsule_offset_y)
        )
        self.capsule_radius = float(
            controls.get("capsule_radius", self.capsule_radius)
        )
        self.capsule_height = float(
            controls.get("capsule_height", self.capsule_height)
        )
        self._ray_hit_history.clear()
        self._ray_hit_history.extend(
            float(value) for value in state.get("ray_hit_history", [])
        )
        self._sphere_cast_toi_history.clear()
        self._sphere_cast_toi_history.extend(
            float(value) for value in state.get("sphere_cast_toi_history", [])
        )
        self._sweep_margin_history.clear()
        self._sweep_margin_history.extend(
            float(value) for value in state.get("sweep_margin_history", [])
        )
        self._capsule_cast_toi_history.clear()
        self._capsule_cast_toi_history.extend(
            float(value) for value in state.get("capsule_cast_toi_history", [])
        )
        self._capsule_margin_history.clear()
        self._capsule_margin_history.extend(
            float(value) for value in state.get("capsule_margin_history", [])
        )
        self._last_metrics = self._deserialize_metrics(state.get("last_metrics", {}))
        if not self._last_metrics:
            self._record_metrics()
        self._sync()

    def _serialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        serialized: dict[str, Any] = {}
        for section, values in metrics.items():
            entry = dict(values)
            for key in ("first_point", "first_normal", "first_center"):
                if key in entry:
                    entry[key] = np.asarray(entry[key], dtype=float).tolist()
            serialized[section] = entry
        return serialized

    def _deserialize_metrics(self, metrics: dict[str, Any]) -> dict[str, Any]:
        deserialized: dict[str, Any] = {}
        for section, values in metrics.items():
            entry = dict(values)
            for key in ("first_point", "first_normal", "first_center"):
                if key in entry:
                    entry[key] = np.asarray(entry[key], dtype=float)
            deserialized[section] = entry
        return deserialized

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        metrics = self._serialize_metrics(self._last_metrics)
        ray = metrics["ray"]
        sphere_cast = metrics["sphere_cast"]
        capsule_cast = metrics["capsule_cast"]
        ray_hits = list(self._ray_hit_history)
        sphere_toi = list(self._sphere_cast_toi_history)
        sweep_margins = list(self._sweep_margin_history)
        capsule_toi = list(self._capsule_cast_toi_history)
        capsule_margins = list(self._capsule_margin_history)
        return {
            "row": "rigid_collision_casts",
            "solver": "collision_query",
            "executor": "not_applicable_collision_query",
            "query_scope": "raycast_sphere_cast_capsule_cast",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "controls": {
                "ray_offset_y": float(self.ray_offset_y),
                "enable_all_ray_hits": bool(self.enable_all_ray_hits),
                "sweep_radius": float(self.sweep_radius),
                "capsule_offset_y": float(self.capsule_offset_y),
                "capsule_radius": float(self.capsule_radius),
                "capsule_height": float(self.capsule_height),
            },
            "ray_hit_count": int(ray["hit_count"]),
            "ray_first_target": str(ray["first_target"]),
            "ray_first_fraction": float(ray["first_fraction"]),
            "ray_first_point": ray["first_point"],
            "ray_first_normal": ray["first_normal"],
            "sphere_hit_count": int(sphere_cast["hit_count"]),
            "sphere_first_target": str(sphere_cast["first_target"]),
            "sphere_first_toi": float(sphere_cast["first_toi"]),
            "sphere_first_point": sphere_cast["first_point"],
            "sphere_first_normal": sphere_cast["first_normal"],
            "sphere_first_center": sphere_cast["first_center"],
            "sphere_margin": float(sphere_cast["margin"]),
            "capsule_hit_count": int(capsule_cast["hit_count"]),
            "capsule_first_target": str(capsule_cast["first_target"]),
            "capsule_first_toi": float(capsule_cast["first_toi"]),
            "capsule_first_point": capsule_cast["first_point"],
            "capsule_first_normal": capsule_cast["first_normal"],
            "capsule_first_center": capsule_cast["first_center"],
            "capsule_margin": float(capsule_cast["margin"]),
            "metrics": metrics,
            "history": {
                "samples": float(len(ray_hits)),
                "max_ray_hit_count": max(ray_hits, default=0.0),
                "max_sphere_first_toi": max(sphere_toi, default=0.0),
                "min_sphere_margin": min(sweep_margins, default=0.0),
                "max_capsule_first_toi": max(capsule_toi, default=0.0),
                "min_capsule_margin": min(capsule_margins, default=0.0),
            },
        }

    def _reset_controls(self) -> None:
        self.ray_offset_y = 0.0
        self.enable_all_ray_hits = True
        self.sweep_radius = 0.35
        self.capsule_offset_y = _CAPSULE_SWEEP_Y
        self.capsule_radius = _CAPSULE_RADIUS
        self.capsule_height = _CAPSULE_HEIGHT
        self._record_metrics()

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_ray_offset, ray_offset = builder.slider(
            "Ray lateral offset", float(self.ray_offset_y), -0.30, 0.42
        )
        changed_all_hits, all_hits = builder.checkbox(
            "Return all ray hits", bool(self.enable_all_ray_hits)
        )
        changed_radius, sweep_radius = builder.slider(
            "Swept sphere radius", float(self.sweep_radius), 0.08, 0.45
        )
        changed_capsule_offset, capsule_offset = builder.slider(
            "Swept capsule lateral offset",
            float(self.capsule_offset_y),
            0.30,
            1.00,
        )
        changed_capsule_radius, capsule_radius = builder.slider(
            "Swept capsule radius", float(self.capsule_radius), 0.06, 0.25
        )
        changed_capsule_height, capsule_height = builder.slider(
            "Swept capsule cylinder height",
            float(self.capsule_height),
            0.20,
            1.20,
        )
        if changed_ray_offset:
            self.ray_offset_y = float(ray_offset)
        if changed_all_hits:
            self.enable_all_ray_hits = bool(all_hits)
        if changed_radius:
            self.sweep_radius = float(sweep_radius)
        if changed_capsule_offset:
            self.capsule_offset_y = float(capsule_offset)
        if changed_capsule_radius:
            self.capsule_radius = float(capsule_radius)
        if changed_capsule_height:
            self.capsule_height = float(capsule_height)
        if builder.button("Reset casts"):
            self._reset_controls()
        elif (
            changed_ray_offset
            or changed_all_hits
            or changed_radius
            or changed_capsule_offset
            or changed_capsule_radius
            or changed_capsule_height
        ):
            self._record_metrics()

        if not self._last_metrics:
            self._record_metrics()

        ray = self._last_metrics["ray"]
        sphere_cast = self._last_metrics["sphere_cast"]
        capsule_cast = self._last_metrics["capsule_cast"]
        builder.separator()
        builder.text("mode: CollisionGroup raycast + sphere + capsule cast")
        builder.text(
            f"ray hits: {int(ray['hit_count'])} | first: "
            f"{ray['first_target']} @ {float(ray['first_fraction']):.3f}"
        )
        builder.text(
            "ray fractions: "
            + ", ".join(f"{value:.3f}" for value in ray["all_fractions"][:4])
        )
        builder.text(
            f"sphere cast hits: {int(sphere_cast['hit_count'])} | first: "
            f"{sphere_cast['first_target']} @ toi {float(sphere_cast['first_toi']):.3f}"
        )
        builder.text(
            f"sweep margin: {float(sphere_cast['margin']):+.3f} m "
            f"(radius + target - offset)"
        )
        builder.text(
            f"capsule cast hits: {int(capsule_cast['hit_count'])} | first: "
            f"{capsule_cast['first_target']} @ toi "
            f"{float(capsule_cast['first_toi']):.3f}"
        )
        builder.text(
            f"capsule margin: {float(capsule_cast['margin']):+.3f} m "
            f"(radius + target - side gap)"
        )
        builder.plot_lines("Ray hit count", list(self._ray_hit_history))
        builder.plot_lines("Sphere cast TOI", list(self._sphere_cast_toi_history))
        builder.plot_lines("Sweep margin", list(self._sweep_margin_history))
        builder.plot_lines("Capsule cast TOI", list(self._capsule_cast_toi_history))
        builder.plot_lines("Capsule margin", list(self._capsule_margin_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    casts = _RigidCollisionCasts()
    return SceneSetup(
        world=casts.bridge.render_world,
        pre_step=casts.pre_step,
        force_drag=casts.bridge.force_drag,
        renderable_provider=casts.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Collision Casts", casts.build_panel)],
        info={
            "sx_world": casts.world,
            "rigid_collision_casts_controller": casts,
            "replay_capture_state": casts.capture_replay_state,
            "replay_restore_state": casts.restore_replay_state,
            "replay_sync": casts._sync,
            CAPTURE_METRICS_INFO_KEY: casts.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_collision_casts",
    title="Rigid Collision Casts",
    category="World Rigid Body",
    summary=(
        "Shows nearest/all raycast hits plus swept-sphere and swept-capsule "
        "time-of-impact queries for rigid collision debugging."
    ),
    build=build,
)
