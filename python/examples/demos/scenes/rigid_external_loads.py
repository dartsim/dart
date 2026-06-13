"""Rigid-body external load response verifier for DART 7 World."""

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
_BOX_HALF = np.array([0.09, 0.09, 0.09])
_BAR_HALF = np.array([0.24, 0.045, 0.045])
_STATIC_HALF = np.array([0.10, 0.10, 0.10])
_BASE_INERTIA_Z = 0.045


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _box_inertia(scale: float = 1.0) -> tuple[tuple[float, float, float], ...]:
    ix = 0.035 * float(scale)
    iy = 0.035 * float(scale)
    iz = _BASE_INERTIA_Z * float(scale)
    return ((ix, 0.0, 0.0), (0.0, iy, 0.0), (0.0, 0.0, iz))


@dataclass
class _LoadLane:
    key: str
    label: str
    kind: str
    body: Any
    initial_position: np.ndarray
    color: tuple[float, float, float]


class _RigidExternalLoads:
    def __init__(self) -> None:
        self.executor_index = 0
        self.force_magnitude = 4.0
        self.torque_magnitude = 0.18
        self.mass_ratio = 4.0
        self.inertia_ratio = 5.0
        self._executors = self._make_executors()

        self.world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, 0.0),
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "light_force",
                "Light persistent force",
                "linear",
                (-0.96, 0.44, 0.0),
                (0.20, 0.58, 0.90),
                _BOX_HALF,
            ),
            self._make_lane(
                "heavy_force",
                "Heavy same force",
                "linear",
                (-0.32, 0.44, 0.0),
                (0.90, 0.54, 0.22),
                _BOX_HALF,
            ),
            self._make_lane(
                "pulse_force",
                "One-step pulse",
                "pulse",
                (0.32, 0.44, 0.0),
                (0.42, 0.74, 0.42),
                _BOX_HALF,
            ),
            self._make_lane(
                "static_load",
                "Static ignored load",
                "static",
                (0.96, 0.44, 0.0),
                (0.50, 0.50, 0.56),
                _STATIC_HALF,
            ),
            self._make_lane(
                "low_inertia_torque",
                "Low inertia torque",
                "angular",
                (-0.42, -0.44, 0.0),
                (0.76, 0.38, 0.82),
                _BAR_HALF,
            ),
            self._make_lane(
                "high_inertia_torque",
                "High inertia same torque",
                "angular",
                (0.42, -0.44, 0.0),
                (0.26, 0.70, 0.70),
                _BAR_HALF,
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_external_loads_render")
        for lane in self.lanes:
            half_extents = _BAR_HALF if lane.kind == "angular" else _BOX_HALF
            if lane.kind == "static":
                half_extents = _STATIC_HALF
            self.bridge.add_rigid_body_visual(
                lane.body,
                dart.BoxShape(_full(half_extents)),
                lane.color,
                name=f"{lane.key}_visual",
            )

        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._angular_speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._angular_accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._static_drift_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._pulse_frames_remaining = 1
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

    def _make_lane(
        self,
        key: str,
        label: str,
        kind: str,
        position: tuple[float, float, float],
        color: tuple[float, float, float],
        half_extents: np.ndarray,
    ) -> _LoadLane:
        body = self.world.add_rigid_body(f"{key}_body", position=position)
        body.set_collision_shape(sx.CollisionShape.box(half_extents))
        return _LoadLane(
            key=key,
            label=label,
            kind=kind,
            body=body,
            initial_position=np.asarray(position, dtype=float),
            color=color,
        )

    def _apply_parameters(self) -> None:
        mass_ratio = max(1.0, float(self.mass_ratio))
        inertia_ratio = max(1.0, float(self.inertia_ratio))
        for lane in self.lanes:
            body = lane.body
            body.is_static = lane.kind == "static"
            if lane.key == "heavy_force":
                body.mass = mass_ratio
            else:
                body.mass = 1.0
            if lane.key == "high_inertia_torque":
                body.inertia = _box_inertia(inertia_ratio)
            else:
                body.inertia = _box_inertia(1.0)

    def _apply_loads(self) -> None:
        force = np.array([float(self.force_magnitude), 0.0, 0.0])
        torque = np.array([0.0, 0.0, float(self.torque_magnitude)])
        for lane in self.lanes:
            lane.body.clear_force()
            lane.body.clear_torque()
        for key in ("light_force", "heavy_force", "static_load"):
            self._lane(key).body.apply_force(force)
        for key in ("low_inertia_torque", "high_inertia_torque", "static_load"):
            self._lane(key).body.apply_torque(torque)
        if self._pulse_frames_remaining > 0:
            self._lane("pulse_force").body.apply_force(force)

    def _lane(self, key: str) -> _LoadLane:
        return next(lane for lane in self.lanes if lane.key == key)

    def _reset_lane(self, lane: _LoadLane) -> None:
        lane.body.transform = _transform_at(lane.initial_position)
        lane.body.linear_velocity = (0.0, 0.0, 0.0)
        lane.body.angular_velocity = (0.0, 0.0, 0.0)
        lane.body.clear_force()
        lane.body.clear_torque()

    def _reset(self) -> None:
        self._apply_parameters()
        self._pulse_frames_remaining = 1
        for lane in self.lanes:
            self._reset_lane(lane)
        self._apply_loads()
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._speed_history.values(),
            *self._accel_history.values(),
            *self._angular_speed_history.values(),
            *self._angular_accel_history.values(),
            self._step_ms_history,
            self._static_drift_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self.bridge.sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample_lane(
        self,
        lane: _LoadLane,
        previous_velocity: np.ndarray,
        previous_angular_velocity: np.ndarray,
    ) -> dict[str, float | str]:
        body = lane.body
        velocity = np.asarray(body.linear_velocity, dtype=float).reshape(3)
        angular_velocity = np.asarray(body.angular_velocity, dtype=float).reshape(3)
        translation = np.asarray(body.translation, dtype=float).reshape(3)
        force = np.asarray(body.force, dtype=float).reshape(3)
        torque = np.asarray(body.torque, dtype=float).reshape(3)
        acceleration = (velocity - previous_velocity) / _TIME_STEP
        angular_acceleration = (angular_velocity - previous_angular_velocity) / _TIME_STEP
        displacement = translation - lane.initial_position
        inertia_z = float(np.asarray(body.inertia, dtype=float).reshape(3, 3)[2, 2])
        expected_linear_accel = float(force[0]) / float(body.mass)
        expected_angular_accel = float(torque[2]) / inertia_z if inertia_z > 0.0 else 0.0
        drift = float(np.linalg.norm(displacement))
        return {
            "speed": float(np.linalg.norm(velocity)),
            "linear_accel_x": float(acceleration[0]),
            "expected_linear_accel_x": expected_linear_accel,
            "angular_speed": float(np.linalg.norm(angular_velocity)),
            "angular_accel_z": float(angular_acceleration[2]),
            "expected_angular_accel_z": expected_angular_accel,
            "displacement_x": float(displacement[0]),
            "force_norm": float(np.linalg.norm(force)),
            "torque_norm": float(np.linalg.norm(torque)),
            "mass": float(body.mass),
            "inertia_z": inertia_z,
            "kinetic_energy": float(body.kinetic_energy),
            "drift": drift,
            "status": self._lane_status(lane, drift),
        }

    def _lane_status(self, lane: _LoadLane, drift: float) -> str:
        if lane.kind == "static":
            return "static ignores load"
        if lane.kind == "pulse":
            return "pulse cleared" if self._pulse_frames_remaining <= 0 else "pulse active"
        if lane.kind == "angular":
            return "torque response"
        return "persistent force"

    def _record_metrics(
        self,
        previous_velocities: dict[str, np.ndarray],
        previous_angular_velocities: dict[str, np.ndarray],
    ) -> None:
        static_drift = 0.0
        for lane in self.lanes:
            metrics = self._sample_lane(
                lane,
                previous_velocities[lane.key],
                previous_angular_velocities[lane.key],
            )
            self._last_metrics[lane.key] = metrics
            self._speed_history[lane.key].append(float(metrics["speed"]))
            self._accel_history[lane.key].append(float(metrics["linear_accel_x"]))
            self._angular_speed_history[lane.key].append(float(metrics["angular_speed"]))
            self._angular_accel_history[lane.key].append(
                float(metrics["angular_accel_z"])
            )
            if lane.kind == "static":
                static_drift = float(metrics["drift"])
        self._static_drift_history.append(static_drift)
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        previous_velocities = {
            lane.key: np.asarray(lane.body.linear_velocity, dtype=float).reshape(3).copy()
            for lane in self.lanes
        }
        previous_angular_velocities = {
            lane.key: np.asarray(lane.body.angular_velocity, dtype=float)
            .reshape(3)
            .copy()
            for lane in self.lanes
        }
        self.world.step(self._executor())
        if self._pulse_frames_remaining > 0:
            self._pulse_frames_remaining -= 1
            if self._pulse_frames_remaining <= 0:
                self._lane("pulse_force").body.clear_force()
        self._record_metrics(previous_velocities, previous_angular_velocities)
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "force_magnitude": float(self.force_magnitude),
                "torque_magnitude": float(self.torque_magnitude),
                "mass_ratio": float(self.mass_ratio),
                "inertia_ratio": float(self.inertia_ratio),
                "pulse_frames_remaining": int(self._pulse_frames_remaining),
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "accel_history": {
                key: list(values) for key, values in self._accel_history.items()
            },
            "angular_speed_history": {
                key: list(values) for key, values in self._angular_speed_history.items()
            },
            "angular_accel_history": {
                key: list(values) for key, values in self._angular_accel_history.items()
            },
            "static_drift_history": list(self._static_drift_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {key: dict(value) for key, value in self._last_metrics.items()}
        if not metrics:
            zeros = {lane.key: np.zeros(3, dtype=float) for lane in self.lanes}
            self._record_metrics(zeros, zeros)
            metrics = {key: dict(value) for key, value in self._last_metrics.items()}
        light = metrics["light_force"]
        heavy = metrics["heavy_force"]
        pulse = metrics["pulse_force"]
        static = metrics["static_load"]
        low = metrics["low_inertia_torque"]
        high = metrics["high_inertia_torque"]
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_external_loads",
            "solver": "sequential_impulse",
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "external_force_torque_accumulator_response",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "force_magnitude": float(self.force_magnitude),
            "torque_magnitude": float(self.torque_magnitude),
            "mass_ratio": float(self.mass_ratio),
            "inertia_ratio": float(self.inertia_ratio),
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "executor_index": float(self.executor_index),
                "force_magnitude": float(self.force_magnitude),
                "torque_magnitude": float(self.torque_magnitude),
                "mass_ratio": float(self.mass_ratio),
                "inertia_ratio": float(self.inertia_ratio),
            },
            "light_force_accel_x": float(light["linear_accel_x"]),
            "heavy_force_accel_x": float(heavy["linear_accel_x"]),
            "pulse_force_norm": float(pulse["force_norm"]),
            "static_drift": float(static["drift"]),
            "low_inertia_angular_accel_z": float(low["angular_accel_z"]),
            "high_inertia_angular_accel_z": float(high["angular_accel_z"]),
            "step_ms": float(step_values[-1]) if step_values else 0.0,
            "history": {
                "samples": float(len(step_values)),
                "max_light_speed": max(
                    (float(value) for value in self._speed_history["light_force"]),
                    default=0.0,
                ),
                "max_heavy_speed": max(
                    (float(value) for value in self._speed_history["heavy_force"]),
                    default=0.0,
                ),
                "max_low_inertia_angular_speed": max(
                    (
                        float(value)
                        for value in self._angular_speed_history[
                            "low_inertia_torque"
                        ]
                    ),
                    default=0.0,
                ),
                "max_high_inertia_angular_speed": max(
                    (
                        float(value)
                        for value in self._angular_speed_history[
                            "high_inertia_torque"
                        ]
                    ),
                    default=0.0,
                ),
                "max_static_drift": max(
                    (float(value) for value in self._static_drift_history),
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
        self.force_magnitude = float(
            controls.get("force_magnitude", self.force_magnitude)
        )
        self.torque_magnitude = float(
            controls.get("torque_magnitude", self.torque_magnitude)
        )
        self.mass_ratio = float(controls.get("mass_ratio", self.mass_ratio))
        self.inertia_ratio = float(controls.get("inertia_ratio", self.inertia_ratio))
        self._pulse_frames_remaining = max(
            0, int(controls.get("pulse_frames_remaining", self._pulse_frames_remaining))
        )
        self._apply_parameters()
        self._apply_loads()
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(self._accel_history, state.get("accel_history", {}))
        self._restore_histories(
            self._angular_speed_history, state.get("angular_speed_history", {})
        )
        self._restore_histories(
            self._angular_accel_history, state.get("angular_accel_history", {})
        )
        self._static_drift_history.clear()
        self._static_drift_history.extend(
            float(value) for value in state.get("static_drift_history", [])
        )
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.world.update_kinematics()
        self.bridge.sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _slider_with_reset(
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

        changed_force, force_magnitude = self._slider_with_reset(
            builder, "Force magnitude", self.force_magnitude, 0.0, 10.0
        )
        changed_torque, torque_magnitude = self._slider_with_reset(
            builder, "Torque magnitude", self.torque_magnitude, 0.0, 0.5
        )
        changed_mass, mass_ratio = self._slider_with_reset(
            builder, "Heavy mass ratio", self.mass_ratio, 1.0, 8.0
        )
        changed_inertia, inertia_ratio = self._slider_with_reset(
            builder, "High inertia ratio", self.inertia_ratio, 1.0, 10.0
        )
        if (
            changed_executor
            or changed_force
            or changed_torque
            or changed_mass
            or changed_inertia
        ):
            self.force_magnitude = force_magnitude
            self.torque_magnitude = torque_magnitude
            self.mass_ratio = mass_ratio
            self.inertia_ratio = inertia_ratio
            self._reset()

        if builder.button("Reset loads"):
            self._reset()

        builder.separator()
        builder.text("solver: sequential impulse | gravity: off | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(
            "persistent load accumulators stay active until cleared; pulse clears "
            "after one step"
        )
        self._lane_text(builder, "light_force")
        self._lane_text(builder, "heavy_force")
        self._lane_text(builder, "pulse_force")
        self._lane_text(builder, "low_inertia_torque")
        self._lane_text(builder, "high_inertia_torque")
        self._lane_text(builder, "static_load")
        builder.plot_lines("Light speed", list(self._speed_history["light_force"]))
        builder.plot_lines("Heavy speed", list(self._speed_history["heavy_force"]))
        builder.plot_lines("Pulse speed", list(self._speed_history["pulse_force"]))
        builder.plot_lines(
            "Low inertia angular speed",
            list(self._angular_speed_history["low_inertia_torque"]),
        )
        builder.plot_lines(
            "High inertia angular speed",
            list(self._angular_speed_history["high_inertia_torque"]),
        )
        builder.plot_lines("Static drift", list(self._static_drift_history))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)

    def _lane_text(self, builder: Any, key: str) -> None:
        lane = self._lane(key)
        metrics = self._last_metrics.get(key)
        if metrics is None:
            metrics = {
                "status": self._lane_status(lane, 0.0),
                "speed": 0.0,
                "linear_accel_x": 0.0,
                "expected_linear_accel_x": 0.0,
                "angular_speed": 0.0,
                "angular_accel_z": 0.0,
                "expected_angular_accel_z": 0.0,
                "force_norm": float(np.linalg.norm(np.asarray(lane.body.force))),
                "torque_norm": float(np.linalg.norm(np.asarray(lane.body.torque))),
                "drift": 0.0,
            }
        if lane.kind in {"linear", "pulse", "static"}:
            builder.text(
                f"{lane.label}: {metrics['status']} | "
                f"speed {float(metrics['speed']):.3f} m/s | "
                f"ax {float(metrics['linear_accel_x']):.3f}/"
                f"{float(metrics['expected_linear_accel_x']):.3f} m/s^2 | "
                f"|F| {float(metrics['force_norm']):.2f}"
            )
        if lane.kind in {"angular", "static"}:
            builder.text(
                f"{lane.label}: {metrics['status']} | "
                f"omega {float(metrics['angular_speed']):.3f} rad/s | "
                f"az {float(metrics['angular_accel_z']):.3f}/"
                f"{float(metrics['expected_angular_accel_z']):.3f} rad/s^2 | "
                f"|tau| {float(metrics['torque_norm']):.2f}"
            )
        if lane.kind == "static":
            builder.text(f"static drift {float(metrics['drift']):.6f} m")


def build() -> SceneSetup:
    loads = _RigidExternalLoads()
    return SceneSetup(
        world=loads.bridge.render_world,
        pre_step=loads.pre_step,
        force_drag=loads.force_drag,
        renderable_provider=loads.renderable_provider,
        panels=[ScenePanel("Rigid External Loads", loads.build_panel)],
        info={
            "sx_world": loads.world,
            "rigid_external_loads_controller": loads,
            "replay_capture_state": loads.capture_replay_state,
            "replay_restore_state": loads.restore_replay_state,
            "replay_sync": loads.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: loads.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_external_loads",
    title="Rigid External Loads",
    category="World Rigid Body",
    summary=(
        "Shows persistent force/torque accumulators, mass and inertia scaling, "
        "pulse clearing, and static-body load caveats."
    ),
    build=build,
)
