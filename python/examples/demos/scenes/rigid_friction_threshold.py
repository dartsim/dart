"""Rigid IPC friction-threshold ramp for visual stick/slip debugging."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_RAMP_HALF = np.array([1.35, 0.26, 0.07])
_BOX_HALF = np.array([0.13, 0.13, 0.13])
_TIME_STEP = 0.005
_HISTORY = 180
_START_DISTANCE = -0.72
_LANE_Y = {
    "below": 0.66,
    "controlled": 0.0,
    "above": -0.66,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _rotation_y(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = math.cos(angle)
    s = math.sin(angle)
    transform[0, 0] = c
    transform[0, 2] = s
    transform[2, 0] = -s
    transform[2, 2] = c
    return transform


def _oriented_transform(position: np.ndarray, angle: float) -> np.ndarray:
    transform = _rotation_y(angle)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _normal(angle: float) -> np.ndarray:
    return np.array([math.sin(angle), 0.0, math.cos(angle)], dtype=float)


def _down_slope(angle: float) -> np.ndarray:
    axis = np.array([math.cos(angle), 0.0, -math.sin(angle)], dtype=float)
    return axis / np.linalg.norm(axis)


@dataclass
class _Lane:
    key: str
    label: str
    expected: str
    color: tuple[float, float, float]
    ramp: Any
    box: Any
    start_position: np.ndarray


class _RigidFrictionThreshold:
    def __init__(self) -> None:
        self.angle_deg = 20.0
        self.controlled_mu = self._threshold()
        self.executor_index = 0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(
            time_step=_TIME_STEP, rigid_body_solver=sx.RigidBodySolver.IPC
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "below",
                "Below threshold",
                "should slide",
                (0.90, 0.30, 0.20),
            ),
            self._make_lane(
                "controlled",
                "Controlled",
                "tune and reset",
                (0.24, 0.52, 0.90),
            ),
            self._make_lane(
                "above",
                "Above threshold",
                "should stick",
                (0.22, 0.68, 0.36),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_friction_threshold")
        for lane in self.lanes:
            self.bridge.add_rigid_body_visual(
                lane.ramp,
                dart.BoxShape(_full(_RAMP_HALF)),
                (0.42, 0.44, 0.47),
                name=f"{lane.key}_threshold_ramp_visual",
            )
            self.bridge.add_rigid_body_visual(
                lane.box,
                dart.BoxShape(_full(_BOX_HALF)),
                lane.color,
                name=f"{lane.key}_threshold_box_visual",
            )

        self._distance_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._clearance_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._reset()

    def _threshold(self) -> float:
        return math.tan(math.radians(float(self.angle_deg)))

    def _lane_mu(self, key: str) -> float:
        threshold = self._threshold()
        if key == "below":
            return max(0.0, 0.75 * threshold)
        if key == "above":
            return min(1.0, 1.35 * threshold)
        return max(0.0, min(1.0, float(self.controlled_mu)))

    def _make_lane(
        self,
        key: str,
        label: str,
        expected: str,
        color: tuple[float, float, float],
    ) -> _Lane:
        origin = np.array([0.0, _LANE_Y[key], 0.0], dtype=float)
        ramp = self.world.add_rigid_body(f"{key}_threshold_ramp", position=tuple(origin))
        ramp.is_static = True
        ramp.set_collision_shape(sx.CollisionShape.box(_RAMP_HALF))

        box = self.world.add_rigid_body(f"{key}_threshold_box")
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        box.restitution = 0.0
        return _Lane(
            key=key,
            label=label,
            expected=expected,
            color=color,
            ramp=ramp,
            box=box,
            start_position=origin.copy(),
        )

    def _apply_lane_parameters(self, lane: _Lane) -> None:
        angle = math.radians(float(self.angle_deg))
        origin = np.array([0.0, _LANE_Y[lane.key], 0.0], dtype=float)
        normal = _normal(angle)
        down_slope = _down_slope(angle)
        contact_offset = _RAMP_HALF[2] + _BOX_HALF[2] + 0.004
        start_position = origin + _START_DISTANCE * down_slope + contact_offset * normal
        friction = self._lane_mu(lane.key)

        lane.ramp.transform = _oriented_transform(origin, angle)
        lane.box.transform = _oriented_transform(start_position, angle)
        lane.ramp.friction = friction
        lane.box.friction = friction
        lane.box.linear_velocity = (0.0, 0.0, 0.0)
        lane.box.angular_velocity = (0.0, 0.0, 0.0)
        lane.box.clear_force()
        lane.box.clear_torque()
        lane.start_position = start_position

    def _reset(self) -> None:
        for lane in self.lanes:
            self._apply_lane_parameters(lane)
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._distance_history.values(),
            *self._speed_history.values(),
            *self._clearance_history.values(),
        ):
            history.clear()
        self._last_metrics.clear()
        self.bridge.sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][0]

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample(self, lane: _Lane) -> dict[str, float | str]:
        angle = math.radians(float(self.angle_deg))
        down_slope = _down_slope(angle)
        normal = _normal(angle)
        origin = np.array([0.0, _LANE_Y[lane.key], 0.0], dtype=float)
        position = np.asarray(lane.box.translation, dtype=float)
        velocity = np.asarray(lane.box.linear_velocity, dtype=float)
        distance = float(np.dot(position - lane.start_position, down_slope))
        speed = float(np.dot(velocity, down_slope))
        clearance = float(
            np.dot(position - origin, normal) - (_RAMP_HALF[2] + _BOX_HALF[2])
        )
        status = "sliding" if abs(speed) > 0.03 or abs(distance) > 0.03 else "sticking"
        if lane.key == "controlled" and abs(self._lane_mu(lane.key) - self._threshold()) < 0.025:
            status = f"near threshold / {status}"
        return {
            "clearance": clearance,
            "distance": distance,
            "friction": self._lane_mu(lane.key),
            "speed": speed,
            "status": status,
        }

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._distance_history[lane.key].append(float(metrics["distance"]))
            self._speed_history[lane.key].append(float(metrics["speed"]))
            self._clearance_history[lane.key].append(float(metrics["clearance"]))

    def pre_step(self) -> None:
        self.world.step(self._executor())
        self._record_metrics()
        self.bridge.sync()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        lanes = {
            lane.key: {
                "label": lane.label,
                "expected": lane.expected,
                "metrics": dict(self._last_metrics[lane.key]),
            }
            for lane in self.lanes
        }
        distance_history = {
            lane.key: list(self._distance_history[lane.key]) for lane in self.lanes
        }
        speed_history = {
            lane.key: list(self._speed_history[lane.key]) for lane in self.lanes
        }
        clearance_history = {
            lane.key: list(self._clearance_history[lane.key]) for lane in self.lanes
        }
        executor_label = self._executor_label()
        return {
            "row": "rigid_friction_threshold",
            "comparison_axis": "friction_threshold_lane",
            "solver": "ipc",
            "solver_enum": sx.RigidBodySolver.IPC.name,
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "held_fixed": {
                "solver": "IPC",
                "executor": executor_label,
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "angle_deg": float(self.angle_deg),
                "controlled_mu": float(self.controlled_mu),
                "executor_index": int(self.executor_index),
                "threshold_mu": float(self._threshold()),
            },
            "below_distance": float(self._last_metrics["below"]["distance"]),
            "below_speed": float(self._last_metrics["below"]["speed"]),
            "below_clearance": float(self._last_metrics["below"]["clearance"]),
            "below_friction": float(self._last_metrics["below"]["friction"]),
            "controlled_distance": float(
                self._last_metrics["controlled"]["distance"]
            ),
            "controlled_speed": float(self._last_metrics["controlled"]["speed"]),
            "controlled_clearance": float(
                self._last_metrics["controlled"]["clearance"]
            ),
            "controlled_friction": float(
                self._last_metrics["controlled"]["friction"]
            ),
            "above_distance": float(self._last_metrics["above"]["distance"]),
            "above_speed": float(self._last_metrics["above"]["speed"]),
            "above_clearance": float(self._last_metrics["above"]["clearance"]),
            "above_friction": float(self._last_metrics["above"]["friction"]),
            "controlled_threshold_delta": float(
                self._last_metrics["controlled"]["friction"] - self._threshold()
            ),
            "step_ms": self._step_profile_ms(),
            "lanes": lanes,
            "history": {
                "samples": float(len(distance_history["below"])),
                "below_max_distance": max(distance_history["below"], default=0.0),
                "below_max_speed": max(speed_history["below"], default=0.0),
                "controlled_max_distance": max(
                    distance_history["controlled"], default=0.0
                ),
                "controlled_max_speed": max(
                    speed_history["controlled"], default=0.0
                ),
                "above_max_abs_distance": max(
                    (abs(value) for value in distance_history["above"]),
                    default=0.0,
                ),
                "above_max_abs_speed": max(
                    (abs(value) for value in speed_history["above"]),
                    default=0.0,
                ),
                "min_clearance": min(
                    (
                        value
                        for values in clearance_history.values()
                        for value in values
                    ),
                    default=0.0,
                ),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "angle_deg": float(self.angle_deg),
                "controlled_mu": float(self.controlled_mu),
                "executor_index": int(self.executor_index),
            },
            "distance_history": {
                key: list(values) for key, values in self._distance_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "clearance_history": {
                key: list(values) for key, values in self._clearance_history.items()
            },
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.angle_deg = float(controls.get("angle_deg", self.angle_deg))
        self.controlled_mu = float(
            controls.get("controlled_mu", self.controlled_mu)
        )
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self._restore_lane_parameters()
        self._restore_histories(
            self._distance_history, state.get("distance_history", {})
        )
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(
            self._clearance_history, state.get("clearance_history", {})
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.bridge.sync()

    def _restore_lane_parameters(self) -> None:
        angle = math.radians(float(self.angle_deg))
        normal = _normal(angle)
        down_slope = _down_slope(angle)
        contact_offset = _RAMP_HALF[2] + _BOX_HALF[2] + 0.004
        for lane in self.lanes:
            origin = np.array([0.0, _LANE_Y[lane.key], 0.0], dtype=float)
            friction = self._lane_mu(lane.key)
            lane.ramp.friction = friction
            lane.box.friction = friction
            lane.start_position = (
                origin + _START_DISTANCE * down_slope + contact_offset * normal
            )

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _Lane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        builder.text(
            f"{lane.label}: mu {float(metrics['friction']):.3f}, "
            f"{lane.expected}, {metrics['status']}"
        )
        builder.text(
            f"  drift {float(metrics['distance']):.3f} m | "
            f"speed {float(metrics['speed']):.3f} m/s | "
            f"clearance {float(metrics['clearance']):.4f} m"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_angle, angle_deg = builder.slider(
            "Ramp angle", float(self.angle_deg), 5.0, 35.0
        )
        if changed_angle:
            self.angle_deg = float(angle_deg)
            self.controlled_mu = self._threshold()

        changed_mu, controlled_mu = builder.slider(
            "Controlled friction", float(self.controlled_mu), 0.0, 1.0
        )
        if changed_mu:
            self.controlled_mu = float(controlled_mu)

        if changed_executor or changed_angle or changed_mu:
            self._reset()

        if builder.button("Reset ramp test"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: friction relative to static threshold")
        builder.text(
            f"held fixed: solver IPC | executor {self._executor_label()} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: rigid IPC")
        builder.text(f"ramp angle: {self.angle_deg:.1f} deg")
        builder.text(f"stick/slip threshold mu = tan(angle): {self._threshold():.3f}")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(f"step profile: {self._step_profile_ms():.3f} ms")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} drift", list(self._distance_history[lane.key])
            )
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} speed", list(self._speed_history[lane.key])
            )
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    threshold = _RigidFrictionThreshold()
    return SceneSetup(
        world=threshold.bridge.render_world,
        pre_step=threshold.pre_step,
        force_drag=threshold.bridge.force_drag,
        renderable_provider=threshold.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Friction Threshold", threshold.build_panel)],
        info={
            "sx_world": threshold.world,
            "rigid_body_solver": "ipc",
            "rigid_friction_threshold_controller": threshold,
            "replay_capture_state": threshold.capture_replay_state,
            "replay_restore_state": threshold.restore_replay_state,
            "replay_sync": threshold.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: threshold.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_friction_threshold",
    title="Rigid Friction Threshold",
    category="World Rigid Body",
    summary=(
        "Three IPC ramp lanes reveal stick/slip behavior below, near, and above "
        "the incline friction threshold."
    ),
    build=build,
)
