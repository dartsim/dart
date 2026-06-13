"""Rigid-body distance-spring verifier for DART 7 World."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import (
    CAPTURE_METRICS_INFO_KEY,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
)

_TIME_STEP = 0.004
_HISTORY = 180
_REST_LENGTH = 0.45
_SOFT_STIFFNESS = 45.0
_STIFF_STIFFNESS = 220.0
_OFFSET_STIFFNESS = 120.0
_INITIAL_STRETCH = 0.33
_ANCHOR_Z = 1.20
_ANCHOR_HALF = np.array([0.055, 0.055, 0.055])
_BODY_HALF = np.array([0.085, 0.085, 0.085])
_BAR_HALF = np.array([0.22, 0.055, 0.055])
_LANE_X = {
    "free": -1.20,
    "soft": -0.40,
    "stiff": 0.40,
    "offset": 1.20,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _z_aligned_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    start = np.asarray(start, dtype=float).reshape(3)
    end = np.asarray(end, dtype=float).reshape(3)
    vector = end - start
    length = float(np.linalg.norm(vector))
    z_axis = vector / length if length > 1.0e-12 else np.array([0.0, 0.0, 1.0])
    reference = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(z_axis, reference))) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(reference, z_axis)
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm <= 1.0e-12:
        x_axis = np.array([1.0, 0.0, 0.0])
    else:
        x_axis /= x_norm
    y_axis = np.cross(z_axis, x_axis)

    transform = np.eye(4)
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    transform[:3, 3] = 0.5 * (start + end)
    return transform


def _box_inertia(half_extents: np.ndarray, mass: float) -> tuple[tuple[float, ...], ...]:
    full = _full(half_extents)
    ix = mass * float(full[1] * full[1] + full[2] * full[2]) / 12.0
    iy = mass * float(full[0] * full[0] + full[2] * full[2]) / 12.0
    iz = mass * float(full[0] * full[0] + full[1] * full[1]) / 12.0
    return ((ix, 0.0, 0.0), (0.0, iy, 0.0), (0.0, 0.0, iz))


def _anchor_world_position(body: Any, local_anchor: np.ndarray) -> np.ndarray:
    rotation = np.asarray(body.rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(body.translation, dtype=float).reshape(3)
    return translation + rotation @ np.asarray(local_anchor, dtype=float)


def _serialized_metrics(
    metrics: dict[str, dict[str, float | str]],
) -> dict[str, dict[str, float | str]]:
    return {key: dict(value) for key, value in metrics.items()}


def _last_float(values: Any) -> float | None:
    try:
        if values:
            return float(values[-1])
    except (IndexError, TypeError, ValueError):
        return None
    return None


@dataclass
class _SpringLane:
    key: str
    label: str
    anchor: Any
    body: Any
    rest_length: float
    stiffness: float
    has_spring: bool
    spring_name: str | None
    parent_anchor: np.ndarray
    child_anchor: np.ndarray
    initial_body_position: np.ndarray
    connector: Any | None
    color: tuple[float, float, float]


class _RigidDistanceSpring:
    def __init__(self) -> None:
        self.executor_index = 0
        self.rest_length = _REST_LENGTH
        self.soft_stiffness = _SOFT_STIFFNESS
        self.stiff_stiffness = _STIFF_STIFFNESS
        self.offset_stiffness = _OFFSET_STIFFNESS
        self.initial_stretch = _INITIAL_STRETCH
        self.gravity_scale = 0.0
        self._executors = self._make_executors()

        self.world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, 0.0),
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "free",
                "No spring baseline",
                stiffness=0.0,
                has_spring=False,
                x=_LANE_X["free"],
                color=(0.46, 0.46, 0.52),
            ),
            self._make_lane(
                "soft",
                "Soft center spring",
                stiffness=self.soft_stiffness,
                has_spring=True,
                x=_LANE_X["soft"],
                color=(0.26, 0.58, 0.86),
            ),
            self._make_lane(
                "stiff",
                "Stiff center spring",
                stiffness=self.stiff_stiffness,
                has_spring=True,
                x=_LANE_X["stiff"],
                color=(0.88, 0.54, 0.22),
            ),
            self._make_lane(
                "offset",
                "Off-center anchors",
                stiffness=self.offset_stiffness,
                has_spring=True,
                x=_LANE_X["offset"],
                child_anchor=np.array([0.16, 0.0, 0.0]),
                half_extents=_BAR_HALF,
                color=(0.58, 0.46, 0.84),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_distance_spring")
        for lane in self.lanes:
            half_extents = _BAR_HALF if lane.key == "offset" else _BODY_HALF
            self.bridge.add_rigid_body_visual(
                lane.anchor,
                dart.BoxShape(_full(_ANCHOR_HALF)),
                (0.30, 0.32, 0.36),
                name=f"{lane.key}_anchor_visual",
            )
            self.bridge.add_rigid_body_visual(
                lane.body,
                dart.BoxShape(_full(half_extents)),
                lane.color,
                name=f"{lane.key}_body_visual",
            )
            if lane.has_spring:
                lane.connector = dart.SimpleFrame(
                    dart.gui.world_render_frame(),
                    f"{lane.key}_distance_spring_visual",
                    _translation((0.0, 0.0, -10.0)),
                )
                lane.connector.set_shape(dart.CylinderShape(0.010, lane.rest_length))
                lane.connector.create_visual_aspect().set_color(
                    [lane.color[0], lane.color[1], lane.color[2], 0.82]
                )
                self.bridge.render_world.add_simple_frame(lane.connector)

        self._length_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._stretch_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._angular_speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._reset()

    def _make_executors(self) -> list[tuple[str, Any]]:
        executors: list[tuple[str, Any]] = [("Sequential", sx.SequentialExecutor())]
        try:
            executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass
        return executors

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][0]

    def _make_lane(
        self,
        key: str,
        label: str,
        *,
        stiffness: float,
        has_spring: bool,
        x: float,
        color: tuple[float, float, float],
        child_anchor: np.ndarray | None = None,
        half_extents: np.ndarray = _BODY_HALF,
    ) -> _SpringLane:
        parent_anchor = np.zeros(3)
        child_anchor = (
            np.zeros(3)
            if child_anchor is None
            else np.asarray(child_anchor, dtype=float).reshape(3)
        )
        initial_body_position = self._body_position_for_lane(x)
        anchor = self.world.add_rigid_body(
            f"{key}_distance_spring_anchor",
            position=(x, 0.0, _ANCHOR_Z),
        )
        anchor.is_static = True
        anchor.set_collision_shape(sx.CollisionShape.box(_ANCHOR_HALF))

        body = self.world.add_rigid_body(
            f"{key}_distance_spring_payload",
            position=tuple(initial_body_position),
        )
        body.mass = 1.0
        body.inertia = _box_inertia(half_extents, 1.0)
        body.set_collision_shape(sx.CollisionShape.box(half_extents))
        spring_name = f"{key}_distance_spring" if has_spring else None
        if has_spring:
            self.world.add_rigid_body_distance_spring(
                spring_name,
                anchor,
                body,
                self.rest_length,
                stiffness,
                parent_anchor=tuple(parent_anchor),
                child_anchor=tuple(child_anchor),
            )
        return _SpringLane(
            key=key,
            label=label,
            anchor=anchor,
            body=body,
            rest_length=self.rest_length,
            stiffness=stiffness,
            has_spring=has_spring,
            spring_name=spring_name,
            parent_anchor=parent_anchor,
            child_anchor=child_anchor,
            initial_body_position=initial_body_position,
            connector=None,
            color=color,
        )

    def _body_position_for_lane(self, x: float) -> np.ndarray:
        length = float(self.rest_length) + max(0.05, float(self.initial_stretch))
        return np.array([float(x), 0.0, _ANCHOR_Z - length])

    def _apply_parameters(self) -> None:
        self.world.gravity = (0.0, 0.0, -4.0 * float(self.gravity_scale))

    def _stiffness_for_lane(self, key: str) -> float:
        if key == "soft":
            return float(self.soft_stiffness)
        if key == "stiff":
            return float(self.stiff_stiffness)
        if key == "offset":
            return float(self.offset_stiffness)
        return 0.0

    def _apply_spring_parameters(self) -> None:
        self.rest_length = max(0.05, float(self.rest_length))
        self.soft_stiffness = max(1.0, float(self.soft_stiffness))
        self.stiff_stiffness = max(1.0, float(self.stiff_stiffness))
        self.offset_stiffness = max(1.0, float(self.offset_stiffness))
        for lane in self.lanes:
            lane.rest_length = float(self.rest_length)
            lane.stiffness = self._stiffness_for_lane(lane.key)
            if lane.spring_name is not None:
                self.world.set_rigid_body_distance_spring_parameters(
                    lane.spring_name,
                    lane.rest_length,
                    lane.stiffness,
                )

    def _reset_lane(self, lane: _SpringLane) -> None:
        lane.initial_body_position = self._body_position_for_lane(_LANE_X[lane.key])
        lane.anchor.transform = _translation((_LANE_X[lane.key], 0.0, _ANCHOR_Z))
        lane.body.transform = _translation(lane.initial_body_position)
        lane.body.linear_velocity = (0.0, 0.0, 0.0)
        lane.body.angular_velocity = (0.0, 0.0, 0.0)
        lane.body.clear_force()
        lane.body.clear_torque()

    def _reset(self) -> None:
        self._apply_parameters()
        self._apply_spring_parameters()
        for lane in self.lanes:
            self._reset_lane(lane)
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._length_history.values(),
            *self._stretch_history.values(),
            *self._speed_history.values(),
            *self._angular_speed_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync_visuals()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _lane_length(self, lane: _SpringLane) -> float:
        return float(
            np.linalg.norm(
                _anchor_world_position(lane.anchor, lane.parent_anchor)
                - _anchor_world_position(lane.body, lane.child_anchor)
            )
        )

    def _lane_status(self, lane: _SpringLane, stretch: float) -> str:
        if not lane.has_spring:
            return "unsprung"
        if lane.key == "offset":
            return "off-center torque"
        if abs(stretch) < 0.02:
            return "near rest length"
        return "restoring"

    def _sample_lane(self, lane: _SpringLane) -> dict[str, float | str]:
        length = self._lane_length(lane)
        stretch = length - lane.rest_length
        velocity = np.asarray(lane.body.linear_velocity, dtype=float).reshape(3)
        angular_velocity = np.asarray(lane.body.angular_velocity, dtype=float).reshape(3)
        return {
            "length": length,
            "rest_length": lane.rest_length,
            "stretch": stretch,
            "abs_stretch": abs(stretch),
            "stiffness": float(lane.stiffness),
            "has_spring": float(lane.has_spring),
            "speed": float(np.linalg.norm(velocity)),
            "angular_speed": float(np.linalg.norm(angular_velocity)),
            "z": float(np.asarray(lane.body.translation, dtype=float).reshape(3)[2]),
            "status": self._lane_status(lane, stretch),
        }

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample_lane(lane)
            self._last_metrics[lane.key] = metrics
            self._length_history[lane.key].append(float(metrics["length"]))
            self._stretch_history[lane.key].append(float(metrics["stretch"]))
            self._speed_history[lane.key].append(float(metrics["speed"]))
            self._angular_speed_history[lane.key].append(
                float(metrics["angular_speed"])
            )
        self._step_ms_history.append(self._step_profile_ms())

    def _sync_visuals(self) -> None:
        self.bridge.sync()
        for lane in self.lanes:
            if lane.connector is None:
                continue
            parent = _anchor_world_position(lane.anchor, lane.parent_anchor)
            child = _anchor_world_position(lane.body, lane.child_anchor)
            length = max(1.0e-4, float(np.linalg.norm(parent - child)))
            lane.connector.set_shape(dart.CylinderShape(0.010, length))
            lane.connector.set_transform(_z_aligned_transform(parent, child))

    def pre_step(self) -> None:
        self._apply_parameters()
        self.world.step(self._executor())
        self._record_metrics()
        self._sync_visuals()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        sprung_keys = ("soft", "stiff", "offset")
        max_sprung_abs_stretch = max(
            (
                abs(float(value))
                for key in sprung_keys
                for value in self._stretch_history[key]
            ),
            default=0.0,
        )
        lane_metrics = _serialized_metrics(self._last_metrics)
        lane_abs_stretch = {
            key: abs(float(lane_metrics[key]["stretch"])) for key in _LANE_X
        }
        offset_angular_speed = float(lane_metrics["offset"]["angular_speed"])
        return {
            "row": "rigid_distance_spring",
            "comparison_axis": "distance_spring_response_family",
            "solver": "sequential_impulse_avbd_distance_spring",
            "executor": self._executor_label(),
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "gravity_z": float(self.world.gravity[2]),
            "initial_stretch": float(self.initial_stretch),
            "held_fixed": {
                "executor": self._executor_label(),
                "payload_mass": 1.0,
                "rest_length_m": float(self.rest_length),
                "solver": "Sequential impulse + AVBD distance springs",
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "gravity_scale": float(self.gravity_scale),
                "initial_stretch": float(self.initial_stretch),
                "offset_stiffness": float(self.offset_stiffness),
                "rest_length": float(self.rest_length),
                "soft_stiffness": float(self.soft_stiffness),
                "stiff_stiffness": float(self.stiff_stiffness),
            },
            "lane_order": [lane.key for lane in self.lanes],
            "spring_lanes": [lane.label for lane in self.lanes],
            "distance_spring_free_abs_stretch": lane_abs_stretch["free"],
            "distance_spring_soft_abs_stretch": lane_abs_stretch["soft"],
            "distance_spring_stiff_abs_stretch": lane_abs_stretch["stiff"],
            "distance_spring_offset_abs_stretch": lane_abs_stretch["offset"],
            "distance_spring_offset_angular_speed": offset_angular_speed,
            "distance_spring_max_sprung_abs_stretch": max_sprung_abs_stretch,
            "lanes": lane_metrics,
            "history": {
                "samples": float(len(self._step_ms_history)),
                "max_sprung_abs_stretch": max_sprung_abs_stretch,
                "max_offset_angular_speed": max(
                    self._angular_speed_history["offset"], default=0.0
                ),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "initial_stretch": float(self.initial_stretch),
                "gravity_scale": float(self.gravity_scale),
                "offset_stiffness": float(self.offset_stiffness),
                "rest_length": float(self.rest_length),
                "soft_stiffness": float(self.soft_stiffness),
                "stiff_stiffness": float(self.stiff_stiffness),
            },
            "world_state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "world_time": float(self.world.time),
            "length_history": {
                key: list(values) for key, values in self._length_history.items()
            },
            "stretch_history": {
                key: list(values) for key, values in self._stretch_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "angular_speed_history": {
                key: list(values)
                for key, values in self._angular_speed_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": _serialized_metrics(self._last_metrics),
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        stretch_histories = snapshot.get("stretch_history", {})
        if isinstance(stretch_histories, dict):
            latest_stretches = [
                abs(value)
                for key in ("soft", "stiff", "offset")
                if (value := _last_float(stretch_histories.get(key, []))) is not None
            ]
            if latest_stretches:
                return max(latest_stretches)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            latest_stretches = []
            for key in ("soft", "stiff", "offset"):
                lane_metrics = metrics.get(key, {})
                if not isinstance(lane_metrics, dict):
                    continue
                try:
                    latest_stretches.append(
                        abs(float(lane_metrics.get("stretch", 0.0)))
                    )
                except (TypeError, ValueError):
                    continue
            if latest_stretches:
                return max(latest_stretches)
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.25:
            return 1.0

        angular_histories = snapshot.get("angular_speed_history", {})
        if isinstance(angular_histories, dict):
            offset_spin = _last_float(angular_histories.get("offset", []))
            if offset_spin is not None and abs(offset_spin) >= 1.0:
                return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            offset_metrics = metrics.get("offset", {})
            if isinstance(offset_metrics, dict):
                try:
                    if abs(float(offset_metrics.get("angular_speed", 0.0))) >= 1.0:
                        return 1.0
                except (TypeError, ValueError):
                    return 0.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.initial_stretch = float(
            controls.get("initial_stretch", self.initial_stretch)
        )
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self.rest_length = float(controls.get("rest_length", self.rest_length))
        self.soft_stiffness = float(
            controls.get("soft_stiffness", self.soft_stiffness)
        )
        self.stiff_stiffness = float(
            controls.get("stiff_stiffness", self.stiff_stiffness)
        )
        self.offset_stiffness = float(
            controls.get("offset_stiffness", self.offset_stiffness)
        )
        self._apply_parameters()
        self._apply_spring_parameters()
        if "world_state" in state:
            self.world.state_vector = state["world_state"]
        self.world.time = float(state.get("world_time", self.world.time))
        self.world.update_kinematics()
        self._restore_histories(self._length_history, state.get("length_history", {}))
        self._restore_histories(self._stretch_history, state.get("stretch_history", {}))
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(
            self._angular_speed_history, state.get("angular_speed_history", {})
        )
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self._sync_visuals()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _SpringLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample_lane(lane)
        builder.text(
            f"{lane.label}: {metrics['status']} | "
            f"k {float(metrics['stiffness']):.1f} N/m | "
            f"length {float(metrics['length']):.3f} m"
        )
        builder.text(
            f"stretch {float(metrics['stretch']) * 1000.0:.1f} mm | "
            f"speed {float(metrics['speed']):.3f} m/s | "
            f"spin {float(metrics['angular_speed']):.3f} rad/s"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), [label for label, _ in self._executors]
        )
        changed_stretch, initial_stretch = builder.slider(
            "Initial stretch", float(self.initial_stretch), 0.08, 0.52
        )
        changed_rest_length, rest_length = builder.slider(
            "Rest length", float(self.rest_length), 0.25, 0.75
        )
        changed_soft, soft_stiffness = builder.slider(
            "Soft stiffness", float(self.soft_stiffness), 10.0, 100.0
        )
        changed_stiff, stiff_stiffness = builder.slider(
            "Stiff stiffness", float(self.stiff_stiffness), 80.0, 320.0
        )
        changed_offset, offset_stiffness = builder.slider(
            "Offset stiffness", float(self.offset_stiffness), 40.0, 220.0
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.0
        )
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_stretch:
            self.initial_stretch = float(initial_stretch)
        if changed_rest_length:
            self.rest_length = float(rest_length)
        if changed_soft:
            self.soft_stiffness = float(soft_stiffness)
        if changed_stiff:
            self.stiff_stiffness = float(stiff_stiffness)
        if changed_offset:
            self.offset_stiffness = float(offset_stiffness)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if (
            changed_stretch
            or changed_gravity
            or changed_rest_length
            or changed_soft
            or changed_stiff
            or changed_offset
        ):
            self._reset()
        if builder.button("Reset springs"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: distance-spring response family")
        builder.text(
            f"held fixed: executor {self._executor_label()} | sequential impulse + "
            f"AVBD springs | rest length {self.rest_length:.2f} m | payload "
            f"mass 1.0 | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: sequential impulse with AVBD distance-spring rows")
        builder.text("IPC and multibody worlds reject rigid-body distance springs")
        builder.text(
            f"spring k soft/stiff/offset {self.soft_stiffness:.1f}/"
            f"{self.stiff_stiffness:.1f}/{self.offset_stiffness:.1f} N/m"
        )
        builder.text(
            f"rest length {float(self.rest_length):.3f} m | "
            f"gravity z {float(self.world.gravity[2]):.3f} m/s^2 | "
            f"step {self._step_profile_ms():.3f} ms"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Free stretch", list(self._stretch_history["free"]))
        builder.plot_lines("Soft stretch", list(self._stretch_history["soft"]))
        builder.plot_lines("Stiff stretch", list(self._stretch_history["stiff"]))
        builder.plot_lines("Offset angular speed", list(self._angular_speed_history["offset"]))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidDistanceSpring()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Distance Spring", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_distance_spring_controller": controller,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_sync": controller._sync_visuals,
            "replay_timeline": {
                "signal_label": "Max spring stretch",
                "signal": controller.replay_timeline_signal,
                "markers": controller.replay_timeline_marker,
            },
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_distance_spring",
    title="Rigid Distance Spring",
    category="World Rigid Body",
    summary=(
        "Shows public World rigid-body distance springs reducing stretch and "
        "creating off-center anchor torque without claiming IPC support."
    ),
    build=build,
)
