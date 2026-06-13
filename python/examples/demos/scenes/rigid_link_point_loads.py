"""Rigid-body point-load verifier for DART 7 World links."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_MASS = 2.0
_INERTIA_Z = 0.16
_BODY_SIZE = np.array([0.30, 0.12, 0.08])
_MARKER_RADIUS = 0.024


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _yaw_from_rotation(rotation: np.ndarray) -> float:
    return float(np.arctan2(float(rotation[1, 0]), float(rotation[0, 0])))


def _wrap_angle(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass
class _PointLoadLane:
    key: str
    label: str
    mode: str
    link: Any
    joint: Any
    initial_position: np.ndarray
    initial_yaw: float
    color: tuple[float, float, float]


class _RigidLinkPointLoads:
    def __init__(self) -> None:
        self.executor_index = 0
        self.force_magnitude = 4.0
        self.point_offset = 0.20
        self.yaw_degrees = 90.0
        self._executors = self._make_executors()

        self.world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, 0.0),
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "center",
                "Centered force",
                "center",
                (-0.82, 0.44, 0.0),
                0.0,
                (0.22, 0.58, 0.90),
            ),
            self._make_lane(
                "offcenter",
                "Off-center force",
                "offcenter",
                (0.0, 0.44, 0.0),
                0.0,
                (0.90, 0.48, 0.24),
            ),
            self._make_lane(
                "pulse",
                "One-shot point force",
                "pulse",
                (0.82, 0.44, 0.0),
                0.0,
                (0.44, 0.72, 0.38),
            ),
            self._make_lane(
                "double",
                "Double apply before step",
                "double",
                (-0.82, -0.44, 0.0),
                0.0,
                (0.70, 0.48, 0.86),
            ),
            self._make_lane(
                "world_frame",
                "Yawed world-frame force",
                "world_frame",
                (0.0, -0.44, 0.0),
                np.deg2rad(self.yaw_degrees),
                (0.24, 0.70, 0.70),
            ),
            self._make_lane(
                "local_frame",
                "Yawed local-frame force",
                "local_frame",
                (0.82, -0.44, 0.0),
                np.deg2rad(self.yaw_degrees),
                (0.88, 0.70, 0.22),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_link_point_loads_render")
        for lane in self.lanes:
            self.bridge.add_link_visual(
                lane.link,
                dart.BoxShape(_BODY_SIZE),
                lane.color,
                name=f"{lane.key}_body_visual",
            )
        self.markers = self._make_load_markers()

        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._yaw_rate_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._yaw_accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_world_velocities: dict[str, np.ndarray] = {}
        self._last_yaw_rates: dict[str, float] = {}
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._pulse_frames_remaining = 1
        self.reset(clear_replay=True)

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

    def _make_lane(
        self,
        key: str,
        label: str,
        mode: str,
        position: tuple[float, float, float],
        yaw: float,
        color: tuple[float, float, float],
    ) -> _PointLoadLane:
        robot = self.world.add_multibody(f"{key}_point_load_robot")
        base = robot.add_link(f"{key}_base")
        link = robot.add_link(
            f"{key}_body",
            parent=base,
            joint=sx.JointSpec(
                name=f"{key}_floating",
                type=sx.JointType.FLOATING,
            ),
        )
        link.mass = _MASS
        link.inertia = (
            (0.07, 0.0, 0.0),
            (0.0, 0.10, 0.0),
            (0.0, 0.0, _INERTIA_Z),
        )
        return _PointLoadLane(
            key=key,
            label=label,
            mode=mode,
            link=link,
            joint=link.parent_joint,
            initial_position=np.asarray(position, dtype=float),
            initial_yaw=float(yaw),
            color=color,
        )

    def _make_load_markers(self) -> dict[str, Any]:
        markers: dict[str, Any] = {}
        for lane in self.lanes:
            marker = dart.SimpleFrame(
                dart.gui.world_render_frame(),
                f"{lane.key}_point_load_marker",
                np.eye(4),
            )
            marker.set_shape(dart.SphereShape(_MARKER_RADIUS))
            marker.create_visual_aspect().set_color([0.98, 0.84, 0.20])
            self.bridge.render_world.add_simple_frame(marker)
            markers[lane.key] = marker
        return markers

    def _local_point(self, lane: _PointLoadLane) -> np.ndarray:
        if lane.mode in {"offcenter", "pulse"}:
            return np.array([0.0, max(0.0, float(self.point_offset)), 0.0])
        return np.zeros(3)

    def _world_point(self, lane: _PointLoadLane, local_point: np.ndarray) -> np.ndarray:
        transform = np.asarray(lane.link.transform, dtype=float).reshape(4, 4)
        return transform[:3, :3] @ local_point + transform[:3, 3]

    def _apply_initial_state(self, lane: _PointLoadLane) -> None:
        yaw = np.deg2rad(float(self.yaw_degrees)) if lane.mode in {
            "world_frame",
            "local_frame",
        } else lane.initial_yaw
        lane.initial_yaw = float(yaw)
        lane.joint.position = [
            float(lane.initial_position[0]),
            float(lane.initial_position[1]),
            float(lane.initial_position[2]),
            0.0,
            0.0,
            float(yaw),
        ]
        lane.joint.velocity = [0.0] * int(lane.joint.num_dofs)
        lane.joint.force = [0.0] * int(lane.joint.num_dofs)

    def reset(self, *, clear_replay: bool) -> None:
        self.force_magnitude = max(0.0, float(self.force_magnitude))
        self.point_offset = float(np.clip(self.point_offset, 0.0, 0.35))
        self.yaw_degrees = float(np.clip(self.yaw_degrees, 0.0, 120.0))
        self._pulse_frames_remaining = 1
        for lane in self.lanes:
            self._apply_initial_state(lane)
        self.world.time = 0.0
        self.world.update_kinematics()
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            *self._speed_history.values(),
            *self._accel_history.values(),
            *self._yaw_rate_history.values(),
            *self._yaw_accel_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._last_world_velocities = {
            lane.key: np.zeros(3, dtype=float) for lane in self.lanes
        }
        self._last_yaw_rates = {lane.key: 0.0 for lane in self.lanes}
        self._last_metrics.clear()
        self._sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _apply_point_loads(self) -> dict[str, int]:
        force = np.array([float(self.force_magnitude), 0.0, 0.0])
        applied: dict[str, int] = {}
        for lane in self.lanes:
            point = self._local_point(lane)
            count = 0
            if lane.mode == "pulse":
                if self._pulse_frames_remaining > 0:
                    lane.link.apply_force(
                        force, self._world_point(lane, point), True, True
                    )
                    count = 1
            elif lane.mode == "double":
                lane.link.apply_force(force, point, True, False)
                lane.link.apply_force(force, point, True, False)
                count = 2
            elif lane.mode == "local_frame":
                lane.link.apply_force(force, point, False, False)
                count = 1
            elif lane.mode == "offcenter":
                lane.link.apply_force(
                    force, self._world_point(lane, point), True, True
                )
                count = 1
            else:
                lane.link.apply_force(force, point, True, False)
                count = 1
            applied[lane.key] = count
        return applied

    def _expected_world_accel(
        self, lane: _PointLoadLane, applied_count: int
    ) -> np.ndarray:
        if applied_count <= 0:
            return np.zeros(3)
        force = np.array([float(self.force_magnitude), 0.0, 0.0])
        if lane.mode == "local_frame":
            rotation = np.asarray(lane.link.rotation, dtype=float).reshape(3, 3)
            force = rotation @ force
        return (float(applied_count) / _MASS) * force

    def _expected_yaw_accel(
        self, lane: _PointLoadLane, applied_count: int
    ) -> float:
        if lane.mode not in {"offcenter", "pulse"} or applied_count <= 0:
            return 0.0
        return -float(applied_count) * float(self.force_magnitude) * float(
            self.point_offset
        ) / _INERTIA_Z

    def _sample_lane(
        self,
        lane: _PointLoadLane,
        previous_translation: np.ndarray,
        previous_yaw: float,
        applied_count: int,
    ) -> dict[str, float | str]:
        translation = np.asarray(lane.link.translation, dtype=float).reshape(3)
        yaw = _yaw_from_rotation(np.asarray(lane.link.rotation, dtype=float).reshape(3, 3))
        world_velocity = (translation - previous_translation) / _TIME_STEP
        yaw_rate = _wrap_angle(yaw - previous_yaw) / _TIME_STEP
        previous_world_velocity = self._last_world_velocities.get(
            lane.key, np.zeros(3, dtype=float)
        )
        previous_yaw_rate = float(self._last_yaw_rates.get(lane.key, 0.0))
        world_accel = (world_velocity - previous_world_velocity) / _TIME_STEP
        yaw_accel = (yaw_rate - previous_yaw_rate) / _TIME_STEP
        self._last_world_velocities[lane.key] = world_velocity
        self._last_yaw_rates[lane.key] = yaw_rate

        expected_world_accel = self._expected_world_accel(lane, applied_count)
        expected_yaw_accel = self._expected_yaw_accel(lane, applied_count)
        displacement = translation - lane.initial_position
        return {
            "status": self._lane_status(lane, applied_count),
            "speed": float(np.linalg.norm(world_velocity)),
            "world_accel_x": float(world_accel[0]),
            "world_accel_y": float(world_accel[1]),
            "expected_world_accel_x": float(expected_world_accel[0]),
            "expected_world_accel_y": float(expected_world_accel[1]),
            "yaw": float(yaw),
            "yaw_rate": float(yaw_rate),
            "yaw_accel": float(yaw_accel),
            "expected_yaw_accel": float(expected_yaw_accel),
            "displacement_x": float(displacement[0]),
            "displacement_y": float(displacement[1]),
            "applied_count": float(applied_count),
        }

    def _lane_status(self, lane: _PointLoadLane, applied_count: int) -> str:
        if lane.mode == "offcenter":
            return "world point lever arm"
        if lane.mode == "pulse":
            return "world point pulse active" if applied_count else "pulse cleared"
        if lane.mode == "double":
            return "two calls accumulated"
        if lane.mode == "world_frame":
            return "world +X force"
        if lane.mode == "local_frame":
            return "local +X force"
        return "centered translation"

    def _record_metrics(
        self,
        previous_translations: dict[str, np.ndarray],
        previous_yaws: dict[str, float],
        applied_counts: dict[str, int],
    ) -> None:
        for lane in self.lanes:
            metrics = self._sample_lane(
                lane,
                previous_translations[lane.key],
                previous_yaws[lane.key],
                applied_counts.get(lane.key, 0),
            )
            self._last_metrics[lane.key] = metrics
            self._speed_history[lane.key].append(float(metrics["speed"]))
            self._accel_history[lane.key].append(float(metrics["world_accel_x"]))
            self._yaw_rate_history[lane.key].append(float(metrics["yaw_rate"]))
            self._yaw_accel_history[lane.key].append(float(metrics["yaw_accel"]))
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        previous_translations = {
            lane.key: np.asarray(lane.link.translation, dtype=float).reshape(3).copy()
            for lane in self.lanes
        }
        previous_yaws = {
            lane.key: _yaw_from_rotation(
                np.asarray(lane.link.rotation, dtype=float).reshape(3, 3)
            )
            for lane in self.lanes
        }
        applied_counts = self._apply_point_loads()
        self.world.step(self._executor())
        if self._pulse_frames_remaining > 0:
            self._pulse_frames_remaining -= 1
        self._record_metrics(previous_translations, previous_yaws, applied_counts)
        self._sync()

    def _sync(self) -> None:
        self.bridge.sync()
        for lane in self.lanes:
            point = self._local_point(lane)
            transform = np.asarray(lane.link.transform, dtype=float).reshape(4, 4)
            self.markers[lane.key].set_transform(transform @ _translation(point))

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        self._sync()
        return self.bridge.renderable_provider()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "force_magnitude": float(self.force_magnitude),
                "point_offset": float(self.point_offset),
                "yaw_degrees": float(self.yaw_degrees),
                "pulse_frames_remaining": int(self._pulse_frames_remaining),
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "accel_history": {
                key: list(values) for key, values in self._accel_history.items()
            },
            "yaw_rate_history": {
                key: list(values) for key, values in self._yaw_rate_history.items()
            },
            "yaw_accel_history": {
                key: list(values) for key, values in self._yaw_accel_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "last_world_velocities": {
                key: values.tolist()
                for key, values in self._last_world_velocities.items()
            },
            "last_yaw_rates": dict(self._last_yaw_rates),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {key: dict(value) for key, value in self._last_metrics.items()}
        step_values = list(self._step_ms_history)
        center = metrics.get("center", {})
        offcenter = metrics.get("offcenter", {})
        pulse = metrics.get("pulse", {})
        double = metrics.get("double", {})
        local_frame = metrics.get("local_frame", {})
        return {
            "row": "rigid_link_point_loads",
            "solver": "sequential_impulse",
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "link_apply_force_point_and_frame_semantics",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "force_magnitude": float(self.force_magnitude),
            "point_offset": float(self.point_offset),
            "yaw_degrees": float(self.yaw_degrees),
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "executor_index": float(self.executor_index),
                "force_magnitude": float(self.force_magnitude),
                "point_offset": float(self.point_offset),
                "yaw_degrees": float(self.yaw_degrees),
                "pulse_frames_remaining": float(self._pulse_frames_remaining),
            },
            "center_world_accel_x": float(center.get("world_accel_x", 0.0)),
            "offcenter_yaw_accel": float(offcenter.get("yaw_accel", 0.0)),
            "pulse_applied_count": float(pulse.get("applied_count", 0.0)),
            "double_world_accel_x": float(double.get("world_accel_x", 0.0)),
            "local_frame_world_accel_y": float(local_frame.get("world_accel_y", 0.0)),
            "step_ms": float(step_values[-1]) if step_values else 0.0,
            "history": {
                "samples": float(len(step_values)),
                "max_center_speed": max(
                    (float(value) for value in self._speed_history["center"]),
                    default=0.0,
                ),
                "max_double_speed": max(
                    (float(value) for value in self._speed_history["double"]),
                    default=0.0,
                ),
                "max_offcenter_yaw_rate": max(
                    (abs(float(value)) for value in self._yaw_rate_history["offcenter"]),
                    default=0.0,
                ),
                "max_pulse_accel": max(
                    (abs(float(value)) for value in self._accel_history["pulse"]),
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
        self.force_magnitude = max(
            0.0, float(controls.get("force_magnitude", self.force_magnitude))
        )
        self.point_offset = float(
            np.clip(float(controls.get("point_offset", self.point_offset)), 0.0, 0.35)
        )
        self.yaw_degrees = float(
            np.clip(float(controls.get("yaw_degrees", self.yaw_degrees)), 0.0, 120.0)
        )
        self._pulse_frames_remaining = max(
            0, int(controls.get("pulse_frames_remaining", self._pulse_frames_remaining))
        )
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(self._accel_history, state.get("accel_history", {}))
        self._restore_histories(
            self._yaw_rate_history, state.get("yaw_rate_history", {})
        )
        self._restore_histories(
            self._yaw_accel_history, state.get("yaw_accel_history", {})
        )
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_world_velocities = {
            key: np.asarray(values, dtype=float).reshape(3)
            for key, values in state.get("last_world_velocities", {}).items()
        }
        self._last_yaw_rates = {
            str(key): float(value)
            for key, value in state.get("last_yaw_rates", {}).items()
        }
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.world.update_kinematics()
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _slider(
        self, builder: Any, label: str, value: float, minimum: float, maximum: float
    ) -> tuple[bool, float]:
        changed, next_value = builder.slider(label, float(value), minimum, maximum)
        return bool(changed), float(next_value)

    def build_panel(self, builder: Any, context: Any) -> None:
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_force, force_magnitude = self._slider(
            builder, "Force magnitude", self.force_magnitude, 0.0, 10.0
        )
        changed_offset, point_offset = self._slider(
            builder, "Point offset", self.point_offset, 0.0, 0.35
        )
        changed_yaw, yaw_degrees = self._slider(
            builder, "Yawed body angle", self.yaw_degrees, 0.0, 120.0
        )
        if changed_executor or changed_force or changed_offset or changed_yaw:
            self.force_magnitude = force_magnitude
            self.point_offset = point_offset
            self.yaw_degrees = yaw_degrees
            self.reset(clear_replay=True)

        if builder.button("Reset point loads"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("solver: sequential impulse | gravity: off | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text("Link.apply_force is one-shot; persistent lanes reapply each step")
        for key in (
            "center",
            "offcenter",
            "pulse",
            "double",
            "world_frame",
            "local_frame",
        ):
            self._lane_text(builder, key)
        builder.plot_lines("Center speed", list(self._speed_history["center"]))
        builder.plot_lines("Double speed", list(self._speed_history["double"]))
        builder.plot_lines("Off-center yaw rate", list(self._yaw_rate_history["offcenter"]))
        builder.plot_lines("Pulse acceleration", list(self._accel_history["pulse"]))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)

    def _lane_text(self, builder: Any, key: str) -> None:
        lane = next(lane for lane in self.lanes if lane.key == key)
        metrics = self._last_metrics.get(key)
        if metrics is None:
            metrics = {
                "status": self._lane_status(lane, 0),
                "speed": 0.0,
                "world_accel_x": 0.0,
                "world_accel_y": 0.0,
                "expected_world_accel_x": 0.0,
                "expected_world_accel_y": 0.0,
                "yaw_accel": 0.0,
                "expected_yaw_accel": 0.0,
                "displacement_x": 0.0,
                "displacement_y": 0.0,
            }
        builder.text(
            f"{lane.label}: {metrics['status']} | "
            f"a ({float(metrics['world_accel_x']):.2f}, "
            f"{float(metrics['world_accel_y']):.2f})/"
            f"({float(metrics['expected_world_accel_x']):.2f}, "
            f"{float(metrics['expected_world_accel_y']):.2f}) m/s^2 | "
            f"yaw a {float(metrics['yaw_accel']):.2f}/"
            f"{float(metrics['expected_yaw_accel']):.2f} rad/s^2 | "
            f"d ({float(metrics['displacement_x']):.3f}, "
            f"{float(metrics['displacement_y']):.3f}) m"
        )


def build() -> SceneSetup:
    point_loads = _RigidLinkPointLoads()
    return SceneSetup(
        world=point_loads.bridge.render_world,
        pre_step=point_loads.pre_step,
        force_drag=point_loads.force_drag,
        renderable_provider=point_loads.renderable_provider,
        panels=[ScenePanel("Rigid Link Point Loads", point_loads.build_panel)],
        info={
            "sx_world": point_loads.world,
            "rigid_link_point_loads_controller": point_loads,
            "replay_capture_state": point_loads.capture_replay_state,
            "replay_restore_state": point_loads.restore_replay_state,
            "replay_sync": point_loads._sync,
            CAPTURE_METRICS_INFO_KEY: point_loads.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_link_point_loads",
    title="Rigid Link Point Loads",
    category="World Rigid Body",
    summary=(
        "Shows one-shot Link.apply_force point loads, lever-arm torque, "
        "before-step accumulation, and world/local frame semantics."
    ),
    build=build,
)
