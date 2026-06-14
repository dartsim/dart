"""Rigid-body frame hierarchy verifier for DART 7 World transforms."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_BODY_HALF = np.array([0.22, 0.12, 0.08])
_SENSOR_RADIUS = 0.045
_AXIS_LENGTH = 0.24


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _rotation_z(angle: float) -> np.ndarray:
    c = float(np.cos(angle))
    s = float(np.sin(angle))
    rotation = np.eye(3)
    rotation[0, 0] = c
    rotation[0, 1] = -s
    rotation[1, 0] = s
    rotation[1, 1] = c
    return rotation


def _transform(
    position: np.ndarray | tuple[float, float, float], yaw: float = 0.0
) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rotation_z(yaw)
    transform[:3, 3] = np.asarray(position, dtype=float).reshape(3)
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


def _axis_local_transform(axis: str, length: float = _AXIS_LENGTH) -> np.ndarray:
    if axis == "x":
        return _transform((0.5 * length, 0.0, 0.0))
    if axis == "y":
        return _transform((0.0, 0.5 * length, 0.0))
    return _transform((0.0, 0.0, 0.5 * length))


def _axis_shape(axis: str, length: float = _AXIS_LENGTH) -> dart.BoxShape:
    thickness = 0.018
    if axis == "x":
        return dart.BoxShape(np.array([length, thickness, thickness]))
    if axis == "y":
        return dart.BoxShape(np.array([thickness, length, thickness]))
    return dart.BoxShape(np.array([thickness, thickness, length]))


def _make_simple_frame(
    render_world: Any,
    name: str,
    shape: Any,
    color: tuple[float, float, float],
    transform: np.ndarray | None = None,
) -> Any:
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        name,
        np.eye(4) if transform is None else transform,
    )
    frame.set_shape(shape)
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


class _RigidFrameHierarchy:
    def __init__(self) -> None:
        self.executor_index = 0
        self.body_yaw_speed = 0.90
        self.path_radius = 0.26
        self.local_offset_x = 0.34
        self.local_offset_y = 0.18
        self.local_yaw_deg = 24.0
        self._executors = self._make_executors()

        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True
        self.body = self.world.add_rigid_body("frame_hierarchy_body")
        self.body.is_kinematic = True
        self.body.mass = 1.0
        self.body.set_collision_shape(sx.CollisionShape.box(_BODY_HALF))
        self.sensor = self.world.add_fixed_frame(
            "frame_hierarchy_sensor",
            self.body,
            offset=self._sensor_local_transform(),
        )
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_frame_hierarchy_render")
        self.bridge.force_drag_enabled = False
        self.bridge.add_rigid_body_visual(
            self.body,
            dart.BoxShape(_full(_BODY_HALF)),
            (0.28, 0.50, 0.86),
            name="frame_hierarchy_body_visual",
        )
        self._make_body_axes()
        self.sensor_marker = _make_simple_frame(
            self.bridge.render_world,
            "frame_hierarchy_sensor_marker",
            dart.SphereShape(_SENSOR_RADIUS),
            (0.95, 0.72, 0.18),
        )
        self.connector = _make_simple_frame(
            self.bridge.render_world,
            "frame_hierarchy_offset_connector",
            dart.BoxShape(np.array([0.1, 0.026, 0.026])),
            (0.76, 0.76, 0.70),
        )
        self.sensor_axes = {
            "x": _make_simple_frame(
                self.bridge.render_world,
                "frame_hierarchy_sensor_x_axis",
                _axis_shape("x"),
                (0.92, 0.20, 0.17),
            ),
            "y": _make_simple_frame(
                self.bridge.render_world,
                "frame_hierarchy_sensor_y_axis",
                _axis_shape("y"),
                (0.20, 0.74, 0.25),
            ),
            "z": _make_simple_frame(
                self.bridge.render_world,
                "frame_hierarchy_sensor_z_axis",
                _axis_shape("z"),
                (0.20, 0.38, 0.92),
            ),
        }
        self.world_origin = _make_simple_frame(
            self.bridge.render_world,
            "frame_hierarchy_world_origin",
            dart.BoxShape(np.array([0.08, 0.08, 0.08])),
            (0.82, 0.82, 0.84),
            _transform((0.0, 0.0, 0.55)),
        )

        self._world_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._relative_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._orientation_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=False)

    def _make_executors(self) -> list[tuple[str, Any]]:
        executors: list[tuple[str, Any]] = [("Sequential", sx.SequentialExecutor())]
        try:
            executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass
        return executors

    def _make_body_axes(self) -> None:
        axes = (
            ("x", (0.92, 0.20, 0.17)),
            ("y", (0.20, 0.74, 0.25)),
            ("z", (0.20, 0.38, 0.92)),
        )
        for axis, color in axes:
            self.bridge.add_rigid_body_visual(
                self.body,
                _axis_shape(axis, 0.30),
                color,
                name=f"frame_hierarchy_body_{axis}_axis",
                local_transform=_axis_local_transform(axis, 0.30),
            )

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _sensor_local_transform(self) -> np.ndarray:
        return _transform(
            (float(self.local_offset_x), float(self.local_offset_y), 0.0),
            np.deg2rad(float(self.local_yaw_deg)),
        )

    def _body_transform_at(self, time_s: float) -> np.ndarray:
        angle = 0.70 * float(time_s)
        position = np.array(
            [
                -0.28 + float(self.path_radius) * np.sin(angle),
                0.10 * np.sin(0.5 * angle),
                0.78 + 0.06 * np.cos(angle),
            ],
            dtype=float,
        )
        yaw = float(self.body_yaw_speed) * float(time_s)
        return _transform(position, yaw)

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _apply_controls(self) -> None:
        self.sensor.local_transform = self._sensor_local_transform()

    def _apply_body_pose(self) -> None:
        self.body.transform = self._body_transform_at(float(self.world.time))
        self.body.linear_velocity = (
            0.70
            * float(self.path_radius)
            * np.cos(0.70 * float(self.world.time)),
            0.035 * np.cos(0.35 * float(self.world.time)),
            -0.042 * np.sin(0.70 * float(self.world.time)),
        )
        self.body.angular_velocity = (0.0, 0.0, float(self.body_yaw_speed))
        self.world.update_kinematics()

    def _sync_visuals(self) -> None:
        body_transform = np.asarray(self.body.transform, dtype=float)
        sensor_transform = np.asarray(self.sensor.transform, dtype=float)
        body_pos = body_transform[:3, 3]
        sensor_pos = sensor_transform[:3, 3]
        connector_length = max(float(np.linalg.norm(sensor_pos - body_pos)), 0.02)
        self.connector.set_shape(
            dart.BoxShape(np.array([connector_length, 0.026, 0.026]))
        )
        self.connector.set_transform(_connector_transform(body_pos, sensor_pos))
        self.sensor_marker.set_transform(sensor_transform)
        for axis, frame in self.sensor_axes.items():
            frame.set_transform(sensor_transform @ _axis_local_transform(axis))
        self.bridge.sync()

    def _record_metrics(self) -> None:
        body_transform = np.asarray(self.body.transform, dtype=float)
        sensor_transform = np.asarray(self.sensor.transform, dtype=float)
        local_transform = np.asarray(self.sensor.local_transform, dtype=float)
        reconstructed = body_transform @ local_transform
        relative = np.asarray(self.sensor.relative_transform(self.body), dtype=float)

        world_error = float(
            np.linalg.norm(sensor_transform[:3, 3] - reconstructed[:3, 3])
        )
        relative_error = float(
            np.linalg.norm(relative[:3, 3] - local_transform[:3, 3])
        )
        orientation_error = float(
            np.linalg.norm(sensor_transform[:3, :3] - reconstructed[:3, :3])
        )
        step_ms = self._step_profile_ms()

        self._world_error_history.append(world_error)
        self._relative_error_history.append(relative_error)
        self._orientation_error_history.append(orientation_error)
        self._step_ms_history.append(step_ms)
        self._last_metrics = {
            "body_x": float(body_transform[0, 3]),
            "body_y": float(body_transform[1, 3]),
            "body_z": float(body_transform[2, 3]),
            "sensor_x": float(sensor_transform[0, 3]),
            "sensor_y": float(sensor_transform[1, 3]),
            "sensor_z": float(sensor_transform[2, 3]),
            "local_x": float(local_transform[0, 3]),
            "local_y": float(local_transform[1, 3]),
            "world_error": world_error,
            "relative_error": relative_error,
            "orientation_error": orientation_error,
            "parent": str(self.sensor.parent_frame.name),
            "step_ms": step_ms,
        }

    def reset(self, *, clear_replay: bool = True) -> None:
        self.world.time = 0.0
        self._apply_controls()
        self._apply_body_pose()
        self.body.clear_force()
        self.body.clear_torque()
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            self._world_error_history,
            self._relative_error_history,
            self._orientation_error_history,
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync_visuals()

    def pre_step(self) -> None:
        self._apply_controls()
        self.world.step(self._executor())
        self._apply_body_pose()
        self._record_metrics()
        self._sync_visuals()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "body_yaw_speed": float(self.body_yaw_speed),
                "path_radius": float(self.path_radius),
                "local_offset_x": float(self.local_offset_x),
                "local_offset_y": float(self.local_offset_y),
                "local_yaw_deg": float(self.local_yaw_deg),
            },
            "world_time": float(self.world.time),
            "world_error_history": list(self._world_error_history),
            "relative_error_history": list(self._relative_error_history),
            "orientation_error_history": list(self._orientation_error_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": dict(self._last_metrics),
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = dict(self._last_metrics)
        if not metrics:
            self._record_metrics()
            metrics = dict(self._last_metrics)
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_frame_hierarchy",
            "solver": "world_frame_hierarchy",
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "body_fixed_frame_transform_residuals",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "body_yaw_speed": float(self.body_yaw_speed),
            "path_radius": float(self.path_radius),
            "local_offset_x": float(self.local_offset_x),
            "local_offset_y": float(self.local_offset_y),
            "local_yaw_deg": float(self.local_yaw_deg),
            "parent": str(metrics["parent"]),
            "metrics": metrics,
            "controls": {
                "executor_index": float(self.executor_index),
                "body_yaw_speed": float(self.body_yaw_speed),
                "path_radius": float(self.path_radius),
                "local_offset_x": float(self.local_offset_x),
                "local_offset_y": float(self.local_offset_y),
                "local_yaw_deg": float(self.local_yaw_deg),
            },
            "body_x": float(metrics["body_x"]),
            "body_y": float(metrics["body_y"]),
            "body_z": float(metrics["body_z"]),
            "sensor_x": float(metrics["sensor_x"]),
            "sensor_y": float(metrics["sensor_y"]),
            "sensor_z": float(metrics["sensor_z"]),
            "world_error": float(metrics["world_error"]),
            "relative_error": float(metrics["relative_error"]),
            "orientation_error": float(metrics["orientation_error"]),
            "step_ms": float(metrics["step_ms"]),
            "history": {
                "samples": float(len(step_values)),
                "max_world_error": max(
                    (float(value) for value in self._world_error_history),
                    default=0.0,
                ),
                "max_relative_error": max(
                    (float(value) for value in self._relative_error_history),
                    default=0.0,
                ),
                "max_orientation_error": max(
                    (float(value) for value in self._orientation_error_history),
                    default=0.0,
                ),
                "max_step_ms": max((float(value) for value in step_values), default=0.0),
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.body_yaw_speed = float(
            controls.get("body_yaw_speed", self.body_yaw_speed)
        )
        self.path_radius = float(controls.get("path_radius", self.path_radius))
        self.local_offset_x = float(
            controls.get("local_offset_x", self.local_offset_x)
        )
        self.local_offset_y = float(
            controls.get("local_offset_y", self.local_offset_y)
        )
        self.local_yaw_deg = float(controls.get("local_yaw_deg", self.local_yaw_deg))
        self.world.time = float(state.get("world_time", self.world.time))
        self._apply_controls()
        self._apply_body_pose()
        self._restore_history(
            self._world_error_history, state.get("world_error_history", [])
        )
        self._restore_history(
            self._relative_error_history, state.get("relative_error_history", [])
        )
        self._restore_history(
            self._orientation_error_history,
            state.get("orientation_error_history", []),
        )
        self._restore_history(self._step_ms_history, state.get("step_ms_history", []))
        self._last_metrics = dict(state.get("last_metrics", {}))
        if not self._last_metrics:
            self._record_metrics()
        self._sync_visuals()

    def _restore_history(self, history: deque[float], values: list[float]) -> None:
        history.clear()
        history.extend(float(value) for value in values)

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_executor, executor_index = builder.select(
            "Executor",
            int(self.executor_index),
            [label for label, _executor in self._executors],
        )
        changed_yaw, body_yaw_speed = builder.slider(
            "Body yaw speed", float(self.body_yaw_speed), -2.4, 2.4
        )
        changed_radius, path_radius = builder.slider(
            "Path radius", float(self.path_radius), 0.0, 0.55
        )
        changed_offset_x, local_offset_x = builder.slider(
            "Local offset X", float(self.local_offset_x), -0.55, 0.65
        )
        changed_offset_y, local_offset_y = builder.slider(
            "Local offset Y", float(self.local_offset_y), -0.45, 0.45
        )
        changed_local_yaw, local_yaw_deg = builder.slider(
            "Local yaw", float(self.local_yaw_deg), -180.0, 180.0
        )

        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_yaw:
            self.body_yaw_speed = float(body_yaw_speed)
        if changed_radius:
            self.path_radius = float(path_radius)
        if changed_offset_x:
            self.local_offset_x = float(local_offset_x)
        if changed_offset_y:
            self.local_offset_y = float(local_offset_y)
        if changed_local_yaw:
            self.local_yaw_deg = float(local_yaw_deg)
        if (
            changed_executor
            or changed_yaw
            or changed_radius
            or changed_offset_x
            or changed_offset_y
            or changed_local_yaw
        ):
            self.reset(clear_replay=True)

        if builder.button("Reset frame path"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics
        builder.separator()
        builder.text("fixed frame parent: rigid body")
        builder.text(f"parent: {metrics.get('parent', self.body.name)}")
        builder.text(
            f"body xyz: {float(metrics.get('body_x', 0.0)):.3f}, "
            f"{float(metrics.get('body_y', 0.0)):.3f}, "
            f"{float(metrics.get('body_z', 0.0)):.3f}"
        )
        builder.text(
            f"sensor xyz: {float(metrics.get('sensor_x', 0.0)):.3f}, "
            f"{float(metrics.get('sensor_y', 0.0)):.3f}, "
            f"{float(metrics.get('sensor_z', 0.0)):.3f}"
        )
        builder.text(
            f"local offset: {float(metrics.get('local_x', 0.0)):.3f}, "
            f"{float(metrics.get('local_y', 0.0)):.3f}, 0.000"
        )
        builder.text(
            f"world residual {float(metrics.get('world_error', 0.0)) * 1000.0:.3f} mm | "
            f"relative residual {float(metrics.get('relative_error', 0.0)) * 1000.0:.3f} mm"
        )
        builder.text(
            f"orientation residual {float(metrics.get('orientation_error', 0.0)):.2e} | "
            f"step {float(metrics.get('step_ms', 0.0)):.3f} ms"
        )
        builder.plot_lines("World transform residual", list(self._world_error_history))
        builder.plot_lines(
            "Relative transform residual", list(self._relative_error_history)
        )
        builder.plot_lines(
            "Orientation residual", list(self._orientation_error_history)
        )
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    hierarchy = _RigidFrameHierarchy()
    return SceneSetup(
        world=hierarchy.bridge.render_world,
        pre_step=hierarchy.pre_step,
        force_drag=hierarchy.force_drag,
        renderable_provider=hierarchy.renderable_provider,
        panels=[ScenePanel("Rigid Frame Hierarchy", hierarchy.build_panel)],
        info={
            "sx_world": hierarchy.world,
            "rigid_frame_hierarchy_controller": hierarchy,
            "replay_capture_state": hierarchy.capture_replay_state,
            "replay_restore_state": hierarchy.restore_replay_state,
            "replay_sync": hierarchy._sync_visuals,
            CAPTURE_METRICS_INFO_KEY: hierarchy.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_frame_hierarchy",
    title="Rigid Frame Hierarchy",
    category="World Rigid Body",
    summary=(
        "Shows fixed sensor/tool frames attached to a moving rigid body, with "
        "local-vs-world transform residuals and executor timing."
    ),
    build=build,
)
