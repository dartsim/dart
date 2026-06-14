"""Rigid-body dynamic/static/kinematic mode verifier for DART 7 World."""

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
_BODY_HALF = np.array([0.10, 0.10, 0.10])
_MARKER_HALF = np.array([0.012, 0.34, 0.012])
_GRAVITY_Z = -9.81
_SOLVERS = [
    ("Sequential impulse", sx.RigidBodySolver.SEQUENTIAL_IMPULSE),
    ("IPC", sx.RigidBodySolver.IPC),
]
_LANE_Y = {
    "dynamic": 0.64,
    "static": 0.0,
    "kinematic": -0.64,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _make_marker(
    render_world: Any,
    name: str,
    position: np.ndarray,
    color: tuple[float, float, float],
    half_extents: np.ndarray = _MARKER_HALF,
) -> Any:
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        name,
        _transform_at(np.asarray(position, dtype=float)),
    )
    frame.set_shape(dart.BoxShape(_full(half_extents)))
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


@dataclass
class _ModeLane:
    key: str
    label: str
    body: Any
    initial_position: np.ndarray
    color: tuple[float, float, float]


class _RigidBodyModes:
    def __init__(self) -> None:
        self.solver_index = 0
        self.executor_index = 0
        self.gravity_scale = 0.55
        self.force_magnitude = 3.0
        self.drive_speed = 0.44
        self._executors = self._make_executors()

        self.world = sx.World(
            time_step=_TIME_STEP,
            gravity=self._gravity(),
            rigid_body_solver=self._solver(),
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "dynamic",
                "Dynamic",
                (-1.05, _LANE_Y["dynamic"], 0.86),
                (0.22, 0.56, 0.88),
            ),
            self._make_lane(
                "static",
                "Static",
                (-1.05, _LANE_Y["static"], 0.86),
                (0.55, 0.56, 0.62),
            ),
            self._make_lane(
                "kinematic",
                "Kinematic",
                (-1.05, _LANE_Y["kinematic"], 0.86),
                (0.90, 0.54, 0.20),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_body_modes_render")
        self.bridge.force_drag_enabled = False
        for lane in self.lanes:
            self.bridge.add_rigid_body_visual(
                lane.body,
                dart.BoxShape(_full(_BODY_HALF)),
                lane.color,
                name=f"{lane.key}_mode_visual",
            )

        self._make_reference_markers()
        self._height_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._x_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._static_drift_history: deque[float] = deque(maxlen=_HISTORY)
        self._kinematic_error_history: deque[float] = deque(maxlen=_HISTORY)
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

    def _make_lane(
        self,
        key: str,
        label: str,
        position: tuple[float, float, float],
        color: tuple[float, float, float],
    ) -> _ModeLane:
        body = self.world.add_rigid_body(f"{key}_mode_body", position=position)
        body.mass = 1.0
        body.set_collision_shape(sx.CollisionShape.box(_BODY_HALF))
        return _ModeLane(
            key=key,
            label=label,
            body=body,
            initial_position=np.asarray(position, dtype=float),
            color=color,
        )

    def _make_reference_markers(self) -> None:
        for lane in self.lanes:
            y = float(lane.initial_position[1])
            z = float(lane.initial_position[2] - _BODY_HALF[2] - 0.035)
            _make_marker(
                self.bridge.render_world,
                f"{lane.key}_start_marker",
                np.array([lane.initial_position[0], y, z]),
                (0.84, 0.84, 0.84),
            )
            _make_marker(
                self.bridge.render_world,
                f"{lane.key}_mode_label_marker",
                np.array([lane.initial_position[0] + 0.32, y, z]),
                lane.color,
                np.array([0.18, 0.018, 0.010]),
            )

    def _lane(self, key: str) -> _ModeLane:
        return next(lane for lane in self.lanes if lane.key == key)

    def _solver(self) -> sx.RigidBodySolver:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][1]

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _gravity(self) -> tuple[float, float, float]:
        return (0.0, 0.0, _GRAVITY_Z * max(0.0, float(self.gravity_scale)))

    def _kinematic_position_at(self, time_s: float) -> np.ndarray:
        lane = self._lane("kinematic")
        position = lane.initial_position.copy()
        position[0] += float(self.drive_speed) * float(time_s)
        position[2] += 0.08 * np.sin(2.0 * np.pi * float(time_s))
        return position

    def _kinematic_velocity_at(self, time_s: float) -> np.ndarray:
        return np.array(
            [
                float(self.drive_speed),
                0.0,
                0.08 * 2.0 * np.pi * np.cos(2.0 * np.pi * float(time_s)),
            ],
            dtype=float,
        )

    def _apply_mode_flags(self) -> None:
        dynamic = self._lane("dynamic").body
        static = self._lane("static").body
        kinematic = self._lane("kinematic").body
        dynamic.is_static = False
        dynamic.is_kinematic = False
        static.is_static = True
        kinematic.is_kinematic = True

    def _apply_parameters(self) -> None:
        self.world.rigid_body_solver = self._solver()
        self.world.gravity = self._gravity()
        self._apply_mode_flags()
        for lane in self.lanes:
            lane.body.mass = 1.0

    def _apply_forces(self) -> None:
        force = np.array([float(self.force_magnitude), 0.0, 0.0], dtype=float)
        for lane in self.lanes:
            lane.body.clear_force()
            lane.body.clear_torque()
            lane.body.apply_force(force)

    def _reset_lane(self, lane: _ModeLane) -> None:
        lane.body.transform = _transform_at(lane.initial_position)
        if lane.key == "dynamic":
            lane.body.linear_velocity = (0.16, 0.0, 0.0)
        else:
            lane.body.linear_velocity = (0.0, 0.0, 0.0)
        lane.body.angular_velocity = (0.0, 0.0, 0.0)
        lane.body.clear_force()
        lane.body.clear_torque()

    def _reset(self) -> None:
        self._apply_parameters()
        for lane in self.lanes:
            self._reset_lane(lane)
        kinematic = self._lane("kinematic").body
        kinematic.transform = _transform_at(self._kinematic_position_at(0.0))
        kinematic.linear_velocity = self._kinematic_velocity_at(0.0)
        self.world.time = 0.0
        self._apply_forces()
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._height_history.values(),
            *self._x_history.values(),
            *self._speed_history.values(),
            self._static_drift_history,
            self._kinematic_error_history,
            self._step_ms_history,
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

    def _mode_status(
        self,
        lane: _ModeLane,
        displacement: np.ndarray,
        kinematic_error: float,
    ) -> str:
        if lane.key == "dynamic":
            return "integrated"
        if lane.key == "static":
            return "fixed"
        if kinematic_error < 1.0e-8:
            return "prescribed"
        return "path error"

    def _sample_lane(self, lane: _ModeLane) -> dict[str, float | str]:
        position = np.asarray(lane.body.translation, dtype=float).reshape(3)
        velocity = np.asarray(lane.body.linear_velocity, dtype=float).reshape(3)
        force = np.asarray(lane.body.force, dtype=float).reshape(3)
        displacement = position - lane.initial_position
        expected = self._kinematic_position_at(self.world.time)
        kinematic_error = (
            float(np.linalg.norm(position - expected)) if lane.key == "kinematic" else 0.0
        )
        return {
            "x": float(position[0]),
            "height": float(position[2]),
            "speed": float(np.linalg.norm(velocity)),
            "force_norm": float(np.linalg.norm(force)),
            "displacement": float(np.linalg.norm(displacement)),
            "displacement_x": float(displacement[0]),
            "height_drop": float(lane.initial_position[2] - position[2]),
            "kinematic_error": kinematic_error,
            "is_static": float(bool(lane.body.is_static)),
            "is_kinematic": float(bool(lane.body.is_kinematic)),
            "status": self._mode_status(lane, displacement, kinematic_error),
        }

    def _record_metrics(self) -> None:
        static_drift = 0.0
        kinematic_error = 0.0
        for lane in self.lanes:
            metrics = self._sample_lane(lane)
            self._last_metrics[lane.key] = metrics
            self._height_history[lane.key].append(float(metrics["height"]))
            self._x_history[lane.key].append(float(metrics["x"]))
            self._speed_history[lane.key].append(float(metrics["speed"]))
            if lane.key == "static":
                static_drift = float(metrics["displacement"])
            if lane.key == "kinematic":
                kinematic_error = float(metrics["kinematic_error"])
        self._static_drift_history.append(static_drift)
        self._kinematic_error_history.append(kinematic_error)
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        self._apply_parameters()
        self._apply_forces()
        self.world.step(self._executor())
        kinematic = self._lane("kinematic").body
        kinematic.transform = _transform_at(self._kinematic_position_at(self.world.time))
        kinematic.linear_velocity = self._kinematic_velocity_at(self.world.time)
        kinematic.angular_velocity = (0.0, 0.0, 0.0)
        self.world.update_kinematics()
        self._record_metrics()
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "solver_index": int(self.solver_index),
                "executor_index": int(self.executor_index),
                "gravity_scale": float(self.gravity_scale),
                "force_magnitude": float(self.force_magnitude),
                "drive_speed": float(self.drive_speed),
            },
            "world_state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "world_time": float(self.world.time),
            "height_history": {
                key: list(values) for key, values in self._height_history.items()
            },
            "x_history": {
                key: list(values) for key, values in self._x_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "static_drift_history": list(self._static_drift_history),
            "kinematic_error_history": list(self._kinematic_error_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {
            lane.key: dict(self._last_metrics.get(lane.key) or self._sample_lane(lane))
            for lane in self.lanes
        }
        dynamic = metrics["dynamic"]
        static = metrics["static"]
        kinematic = metrics["kinematic"]
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_body_modes",
            "comparison_axis": "rigid_body_mode_semantics",
            "solver": _SOLVERS[int(self.solver_index)][0],
            "solver_enum": self._solver().name,
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "dynamic_static_kinematic_mode_semantics",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "gravity_z": float(self.world.gravity[2]),
            "force_magnitude": float(self.force_magnitude),
            "drive_speed": float(self.drive_speed),
            "held_fixed": {
                "solver": _SOLVERS[int(self.solver_index)][0],
                "executor": self._executors[int(self.executor_index)][0],
                "gravity_scale": float(self.gravity_scale),
                "force_magnitude": float(self.force_magnitude),
                "body_mass": 1.0,
                "time_step_ms": float(_TIME_STEP * 1000.0),
            },
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "solver_index": float(self.solver_index),
                "executor_index": float(self.executor_index),
                "gravity_scale": float(self.gravity_scale),
                "force_magnitude": float(self.force_magnitude),
                "drive_speed": float(self.drive_speed),
            },
            "dynamic_height": float(dynamic["height"]),
            "dynamic_speed": float(dynamic["speed"]),
            "dynamic_displacement_x": float(dynamic["displacement_x"]),
            "static_drift": float(static["displacement"]),
            "kinematic_x": float(kinematic["x"]),
            "kinematic_error": float(kinematic["kinematic_error"]),
            "step_ms": float(step_values[-1]) if step_values else 0.0,
            "history": {
                "samples": float(len(step_values)),
                "max_dynamic_speed": max(
                    (float(value) for value in self._speed_history["dynamic"]),
                    default=0.0,
                ),
                "min_dynamic_height": min(
                    (float(value) for value in self._height_history["dynamic"]),
                    default=float(dynamic["height"]),
                ),
                "max_static_drift": max(
                    (float(value) for value in self._static_drift_history),
                    default=0.0,
                ),
                "max_kinematic_error": max(
                    (float(value) for value in self._kinematic_error_history),
                    default=0.0,
                ),
                "max_step_ms": max((float(value) for value in step_values), default=0.0),
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.solver_index = max(
            0,
            min(int(controls.get("solver_index", self.solver_index)), len(_SOLVERS) - 1),
        )
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self.force_magnitude = float(
            controls.get("force_magnitude", self.force_magnitude)
        )
        self.drive_speed = float(controls.get("drive_speed", self.drive_speed))
        self._apply_parameters()
        if "world_state" in state:
            self.world.state_vector = state["world_state"]
        self.world.time = float(state.get("world_time", self.world.time))
        self._apply_forces()
        self.world.update_kinematics()
        self._restore_histories(self._height_history, state.get("height_history", {}))
        self._restore_histories(self._x_history, state.get("x_history", {}))
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._static_drift_history.clear()
        self._static_drift_history.extend(
            float(value) for value in state.get("static_drift_history", [])
        )
        self._kinematic_error_history.clear()
        self._kinematic_error_history.extend(
            float(value) for value in state.get("kinematic_error_history", [])
        )
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.bridge.sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _ModeLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample_lane(lane)
        builder.text(
            f"{lane.label}: {metrics['status']} | "
            f"static {bool(metrics['is_static'])} | "
            f"kinematic {bool(metrics['is_kinematic'])}"
        )
        builder.text(
            f"x {float(metrics['x']):.3f} m | "
            f"height {float(metrics['height']):.3f} m | "
            f"speed {float(metrics['speed']):.3f} m/s"
        )
        if lane.key == "kinematic":
            builder.text(
                f"path error {float(metrics['kinematic_error']) * 1000.0:.3f} mm | "
                f"drive speed {float(self.drive_speed):.3f} m/s"
            )
        else:
            builder.text(
                f"drift {float(metrics['displacement']):.4f} m | "
                f"force {float(metrics['force_norm']):.2f} N"
            )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_solver, solver_index = builder.select(
            "Solver", int(self.solver_index), [label for label, _solver in _SOLVERS]
        )
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), [label for label, _ in self._executors]
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.3
        )
        changed_force, force_magnitude = builder.slider(
            "Force magnitude", float(self.force_magnitude), 0.0, 8.0
        )
        changed_drive, drive_speed = builder.slider(
            "Kinematic speed", float(self.drive_speed), 0.05, 0.90
        )

        if changed_solver:
            self.solver_index = int(solver_index)
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if changed_force:
            self.force_magnitude = float(force_magnitude)
        if changed_drive:
            self.drive_speed = float(drive_speed)
        if (
            changed_solver
            or changed_executor
            or changed_gravity
            or changed_force
            or changed_drive
        ):
            self._reset()

        if builder.button("Reset modes"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: rigid-body mode semantics")
        builder.text(
            f"held fixed: solver {_SOLVERS[int(self.solver_index)][0]} | "
            f"executor {self._executors[int(self.executor_index)][0]} | "
            f"gravity scale {self.gravity_scale:.2f} | "
            f"force {self.force_magnitude:.2f} N | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("mode flags: dynamic, static, and kinematic")
        builder.text(
            f"world time: {self.world.time:.3f} s | "
            f"gravity z {self.world.gravity[2]:.3f} m/s^2 | "
            f"step {self._step_profile_ms():.3f} ms"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Dynamic height", list(self._height_history["dynamic"]))
        builder.plot_lines("Static drift", list(self._static_drift_history))
        builder.plot_lines("Kinematic path error", list(self._kinematic_error_history))
        builder.plot_lines("Dynamic x", list(self._x_history["dynamic"]))
        builder.plot_lines("Kinematic x", list(self._x_history["kinematic"]))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    modes = _RigidBodyModes()
    return SceneSetup(
        world=modes.bridge.render_world,
        pre_step=modes.pre_step,
        force_drag=modes.force_drag,
        renderable_provider=modes.renderable_provider,
        panels=[ScenePanel("Rigid Body Modes", modes.build_panel)],
        info={
            "sx_world": modes.world,
            "rigid_body_modes_controller": modes,
            "replay_capture_state": modes.capture_replay_state,
            "replay_restore_state": modes.restore_replay_state,
            "replay_sync": modes.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: modes.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_body_modes",
    title="Rigid Body Modes",
    category="World Rigid Body",
    summary=(
        "Compares dynamic, static, and kinematic rigid-body mode semantics "
        "with shared solver, executor, gravity, force, and drive controls."
    ),
    build=build,
)
