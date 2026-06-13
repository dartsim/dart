"""Rigid-body time-step sensitivity verifier for DART 7 World."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_BASE_TIME_STEP = 0.002
_GRAVITY_Z = -9.81
_HISTORY = 180
_BALL_RADIUS = 0.065
_GROUND_HALF = np.array([0.27, 0.24, 0.04])
_START_HEIGHT = 1.45
_LANE_X = {
    "fine": -0.78,
    "medium": 0.0,
    "coarse": 0.78,
}
_SOLVERS: tuple[tuple[str, sx.RigidBodySolver], ...] = (
    ("Sequential impulse", sx.RigidBodySolver.SEQUENTIAL_IMPULSE),
    ("IPC barrier", sx.RigidBodySolver.IPC),
)


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _TimeStepLane:
    key: str
    label: str
    multiplier: int
    substeps: int
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    ball: Any
    start_position: np.ndarray


class _RigidTimeStepSensitivity:
    def __init__(self) -> None:
        self.solver_index = 0
        self.executor_index = 0
        self.base_time_step = _BASE_TIME_STEP
        self.gravity_scale = 1.0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.lanes = [
            self._make_lane(
                "fine",
                "Fine dt",
                multiplier=1,
                substeps=4,
                color=(0.24, 0.58, 0.90),
            ),
            self._make_lane(
                "medium",
                "Medium dt",
                multiplier=2,
                substeps=2,
                color=(0.38, 0.72, 0.38),
            ),
            self._make_lane(
                "coarse",
                "Coarse dt",
                multiplier=4,
                substeps=1,
                color=(0.90, 0.48, 0.24),
            ),
        ]
        self._height_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._freefall_error_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._clearance_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._energy_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._coarse_error_ratio: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._first_contact_time: dict[str, float | None] = {
            lane.key: None for lane in self.lanes
        }
        self._reset()

    @property
    def primary_world(self) -> Any:
        return self.lanes[0].world

    @property
    def render_world(self) -> Any:
        return self.lanes[0].bridge.render_world

    def _solver(self) -> sx.RigidBodySolver:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][1]

    def _solver_label(self) -> str:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][0]

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][0]

    def _gravity(self) -> tuple[float, float, float]:
        return (0.0, 0.0, _GRAVITY_Z * max(0.0, float(self.gravity_scale)))

    def _lane_time_step(self, lane: _TimeStepLane) -> float:
        return max(0.0005, float(self.base_time_step)) * float(lane.multiplier)

    def _make_lane(
        self,
        key: str,
        label: str,
        *,
        multiplier: int,
        substeps: int,
        color: tuple[float, float, float],
    ) -> _TimeStepLane:
        x = _LANE_X[key]
        world = sx.World(
            time_step=_BASE_TIME_STEP * multiplier,
            gravity=self._gravity(),
            rigid_body_solver=self._solver(),
        )
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(f"{key}_timestep_ground")
        ground.is_static = True
        ground.transform = _transform_at(np.array([x, 0.0, -_GROUND_HALF[2]]))
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
        ground.friction = 0.4
        ground.restitution = 0.0

        start_position = np.array([x, 0.0, _START_HEIGHT], dtype=float)
        ball = world.add_rigid_body(
            f"{key}_timestep_ball", position=tuple(start_position)
        )
        ball.mass = 1.0
        ball.set_collision_shape(sx.CollisionShape.sphere(_BALL_RADIUS))
        ball.friction = 0.2
        ball.restitution = 0.0

        world.enter_simulation_mode()
        bridge = WorldRenderBridge(world, name=f"{key}_timestep_sensitivity_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.42, 0.44, 0.47),
            name=f"{key}_timestep_ground_visual",
        )
        bridge.add_rigid_body_visual(
            ball,
            dart.SphereShape(_BALL_RADIUS),
            color,
            name=f"{key}_timestep_ball_visual",
        )

        return _TimeStepLane(
            key=key,
            label=label,
            multiplier=multiplier,
            substeps=substeps,
            color=color,
            world=world,
            bridge=bridge,
            ground=ground,
            ball=ball,
            start_position=start_position,
        )

    def _apply_parameters(self, lane: _TimeStepLane) -> None:
        lane.world.time_step = self._lane_time_step(lane)
        lane.world.gravity = self._gravity()
        lane.world.rigid_body_solver = self._solver()

    def _reset_lane(self, lane: _TimeStepLane) -> None:
        self._apply_parameters(lane)
        lane.ball.transform = _transform_at(lane.start_position)
        lane.ball.linear_velocity = (0.0, 0.0, 0.0)
        lane.ball.angular_velocity = (0.0, 0.0, 0.0)
        lane.ball.clear_force()
        lane.ball.clear_torque()
        lane.world.time = 0.0
        try:
            lane.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        lane.world.update_kinematics()

    def _reset(self) -> None:
        for lane in self.lanes:
            self._reset_lane(lane)
        for history in (
            *self._height_history.values(),
            *self._freefall_error_history.values(),
            *self._clearance_history.values(),
            *self._energy_history.values(),
            *self._step_ms_history.values(),
            self._coarse_error_ratio,
        ):
            history.clear()
        self._last_metrics.clear()
        self._first_contact_time = {lane.key: None for lane in self.lanes}
        self._sync()

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _expected_freefall_z(self, lane: _TimeStepLane) -> float:
        t = float(lane.world.time)
        gravity_z = self._gravity()[2]
        return float(lane.start_position[2] + 0.5 * gravity_z * t * t)

    def _sample(self, lane: _TimeStepLane) -> dict[str, float | str]:
        position = np.asarray(lane.ball.translation, dtype=float).reshape(3)
        velocity = np.asarray(lane.ball.linear_velocity, dtype=float).reshape(3)
        height = float(position[2])
        clearance = height - _BALL_RADIUS
        contacts = len(lane.world.collide())
        if contacts > 0 and self._first_contact_time[lane.key] is None:
            self._first_contact_time[lane.key] = float(lane.world.time)
        expected_z = self._expected_freefall_z(lane)
        freefall_error = abs(height - expected_z)
        if contacts > 0:
            status = "contact"
        elif height > _BALL_RADIUS + 0.04:
            status = "free fall"
        else:
            status = "near ground"
        return {
            "clearance": clearance,
            "contact_count": float(contacts),
            "dt": float(lane.world.time_step),
            "energy": float(lane.ball.kinetic_energy + lane.ball.potential_energy),
            "expected_height": expected_z,
            "first_contact_time": (
                -1.0
                if self._first_contact_time[lane.key] is None
                else float(self._first_contact_time[lane.key])
            ),
            "freefall_error": freefall_error,
            "height": height,
            "speed": float(np.linalg.norm(velocity)),
            "status": status,
            "step_ms": self._step_profile_ms(lane.world),
            "substeps": float(lane.substeps),
            "time": float(lane.world.time),
        }

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._height_history[lane.key].append(float(metrics["height"]))
            self._freefall_error_history[lane.key].append(
                float(metrics["freefall_error"])
            )
            self._clearance_history[lane.key].append(float(metrics["clearance"]))
            self._energy_history[lane.key].append(float(metrics["energy"]))
            self._step_ms_history[lane.key].append(float(metrics["step_ms"]))
        fine = float(self._last_metrics["fine"]["freefall_error"])
        coarse = float(self._last_metrics["coarse"]["freefall_error"])
        if fine > 1.0e-12:
            self._coarse_error_ratio.append(coarse / fine)

    def _sync(self) -> None:
        for lane in self.lanes:
            lane.bridge.sync()

    def pre_step(self) -> None:
        executor = self._executor()
        for lane in self.lanes:
            for _ in range(lane.substeps):
                lane.world.step(executor)
        self._record_metrics()
        self._sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        if not event.get("active", False):
            for lane in self.lanes:
                lane.bridge.force_drag(event)
            return
        renderable_id = int(event.get("renderable_id", 0) or 0)
        for lane in self.lanes:
            if renderable_id in getattr(lane.bridge, "_by_renderable_id", {}):
                lane.bridge.force_drag(event)
                return

    def renderable_provider(self) -> list[Any]:
        renderables: list[Any] = []
        for lane in self.lanes:
            renderables.extend(lane.bridge.renderable_provider())
        return renderables

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "solver_index": int(self.solver_index),
                "executor_index": int(self.executor_index),
                "base_time_step": float(self.base_time_step),
                "gravity_scale": float(self.gravity_scale),
            },
            "world_states": {
                lane.key: np.asarray(lane.world.state_vector, dtype=float).copy()
                for lane in self.lanes
            },
            "world_times": {lane.key: float(lane.world.time) for lane in self.lanes},
            "first_contact_time": dict(self._first_contact_time),
            "height_history": {
                key: list(values) for key, values in self._height_history.items()
            },
            "freefall_error_history": {
                key: list(values)
                for key, values in self._freefall_error_history.items()
            },
            "clearance_history": {
                key: list(values) for key, values in self._clearance_history.items()
            },
            "energy_history": {
                key: list(values) for key, values in self._energy_history.items()
            },
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "coarse_error_ratio": list(self._coarse_error_ratio),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {
            lane.key: dict(self._last_metrics.get(lane.key) or self._sample(lane))
            for lane in self.lanes
        }
        fine = metrics["fine"]
        medium = metrics["medium"]
        coarse = metrics["coarse"]
        fine_step_values = list(self._step_ms_history["fine"])
        all_step_values = [
            float(value)
            for lane in self.lanes
            for value in self._step_ms_history[lane.key]
        ]
        coarse_error_ratio = (
            float(self._coarse_error_ratio[-1]) if self._coarse_error_ratio else 0.0
        )
        return {
            "row": "rigid_timestep_sensitivity",
            "comparison_axis": "time_step_multiplier",
            "solver": _SOLVERS[int(self.solver_index)][0],
            "solver_enum": self._solver().name,
            "executor": self._executors[int(self.executor_index)][0],
            "held_fixed": {
                "solver": _SOLVERS[int(self.solver_index)][0],
                "solver_enum": self._solver().name,
                "executor": self._executors[int(self.executor_index)][0],
                "display_step_ms": float(
                    max(0.0005, self.base_time_step) * 4.0 * 1000.0
                ),
                "gravity_scale": float(self.gravity_scale),
            },
            "scope": "matched_time_step_freefall_contact_sensitivity",
            "base_time_step_ms": float(max(0.0005, self.base_time_step) * 1000.0),
            "display_step_ms": float(max(0.0005, self.base_time_step) * 4.0 * 1000.0),
            "world_time": float(self.primary_world.time),
            "gravity_z": float(self._gravity()[2]),
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "solver_index": float(self.solver_index),
                "executor_index": float(self.executor_index),
                "base_time_step": float(self.base_time_step),
                "gravity_scale": float(self.gravity_scale),
            },
            "fine_freefall_error": float(fine["freefall_error"]),
            "medium_freefall_error": float(medium["freefall_error"]),
            "coarse_freefall_error": float(coarse["freefall_error"]),
            "coarse_error_ratio": coarse_error_ratio,
            "fine_clearance": float(fine["clearance"]),
            "coarse_clearance": float(coarse["clearance"]),
            "fine_step_ms": float(fine["step_ms"]),
            "coarse_step_ms": float(coarse["step_ms"]),
            "history": {
                "samples": float(len(fine_step_values)),
                "max_fine_freefall_error": max(
                    (float(value) for value in self._freefall_error_history["fine"]),
                    default=0.0,
                ),
                "max_medium_freefall_error": max(
                    (float(value) for value in self._freefall_error_history["medium"]),
                    default=0.0,
                ),
                "max_coarse_freefall_error": max(
                    (float(value) for value in self._freefall_error_history["coarse"]),
                    default=0.0,
                ),
                "max_coarse_error_ratio": max(
                    (float(value) for value in self._coarse_error_ratio),
                    default=0.0,
                ),
                "min_fine_clearance": min(
                    (float(value) for value in self._clearance_history["fine"]),
                    default=float(fine["clearance"]),
                ),
                "min_coarse_clearance": min(
                    (float(value) for value in self._clearance_history["coarse"]),
                    default=float(coarse["clearance"]),
                ),
                "max_step_ms": max(all_step_values, default=0.0),
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
        self.base_time_step = float(
            controls.get("base_time_step", self.base_time_step)
        )
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        world_states = state.get("world_states", {})
        world_times = state.get("world_times", {})
        for lane in self.lanes:
            self._apply_parameters(lane)
            if lane.key in world_states:
                lane.world.state_vector = world_states[lane.key]
            if lane.key in world_times:
                lane.world.time = float(world_times[lane.key])
            lane.world.update_kinematics()
        self._first_contact_time = {
            lane.key: (
                None
                if state.get("first_contact_time", {}).get(lane.key) is None
                else float(state.get("first_contact_time", {}).get(lane.key))
            )
            for lane in self.lanes
        }
        self._restore_histories(self._height_history, state.get("height_history", {}))
        self._restore_histories(
            self._freefall_error_history, state.get("freefall_error_history", {})
        )
        self._restore_histories(
            self._clearance_history, state.get("clearance_history", {})
        )
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
        self._restore_histories(
            self._step_ms_history, state.get("step_ms_history", {})
        )
        self._coarse_error_ratio.clear()
        self._coarse_error_ratio.extend(
            float(value) for value in state.get("coarse_error_ratio", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _TimeStepLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        contact_time = float(metrics["first_contact_time"])
        contact_text = "none" if contact_time < 0.0 else f"{contact_time:.3f}s"
        builder.text(
            f"{lane.label}: {metrics['status']} | "
            f"dt {float(metrics['dt']) * 1000.0:.1f} ms x {lane.substeps}"
        )
        builder.text(
            f"height {float(metrics['height']):.3f} m | "
            f"free-fall error {float(metrics['freefall_error']) * 1000.0:.2f} mm | "
            f"clearance {float(metrics['clearance']):.4f} m"
        )
        builder.text(
            f"first contact {contact_text} | "
            f"step profile {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        solver_choices = [label for label, _solver in _SOLVERS]
        changed_solver, solver_index = builder.select(
            "Solver", int(self.solver_index), solver_choices
        )
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_choices
        )
        changed_dt, base_time_step = builder.slider(
            "Base time step", float(self.base_time_step), 0.001, 0.006
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.25, 2.0
        )

        if changed_solver:
            self.solver_index = int(solver_index)
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_dt:
            self.base_time_step = float(base_time_step)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if changed_solver or changed_executor or changed_dt or changed_gravity:
            self._reset()

        if builder.button("Reset timestep run"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: time-step multiplier")
        builder.text(
            f"held fixed: solver {self._solver_label()} | "
            f"executor {self._executor_label()} | gravity scale {self.gravity_scale:.2f}"
        )
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(
            f"display step: {self.base_time_step * 4.0 * 1000.0:.1f} ms | "
            f"gravity z: {self._gravity()[2]:.2f} m/s^2"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        if self._coarse_error_ratio:
            builder.text(
                f"coarse/fine free-fall error ratio: "
                f"{self._coarse_error_ratio[-1]:.2f}"
            )
        builder.plot_lines(
            "Fine free-fall error", list(self._freefall_error_history["fine"])
        )
        builder.plot_lines(
            "Medium free-fall error", list(self._freefall_error_history["medium"])
        )
        builder.plot_lines(
            "Coarse free-fall error", list(self._freefall_error_history["coarse"])
        )
        builder.plot_lines("Fine clearance", list(self._clearance_history["fine"]))
        builder.plot_lines("Coarse clearance", list(self._clearance_history["coarse"]))
        builder.plot_lines("Coarse/fine error ratio", list(self._coarse_error_ratio))
        builder.plot_lines("Fine step profile ms", list(self._step_ms_history["fine"]))
        builder.plot_lines(
            "Coarse step profile ms", list(self._step_ms_history["coarse"])
        )
        builder.separator()
        self.lanes[0].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    sensitivity = _RigidTimeStepSensitivity()
    return SceneSetup(
        world=sensitivity.render_world,
        pre_step=sensitivity.pre_step,
        force_drag=sensitivity.force_drag,
        renderable_provider=sensitivity.renderable_provider,
        panels=[ScenePanel("Rigid Time Step Sensitivity", sensitivity.build_panel)],
        info={
            "sx_world": sensitivity.primary_world,
            "rigid_timestep_sensitivity_controller": sensitivity,
            "rigid_timestep_sensitivity_worlds": [
                lane.world for lane in sensitivity.lanes
            ],
            "replay_capture_state": sensitivity.capture_replay_state,
            "replay_restore_state": sensitivity.restore_replay_state,
            "replay_sync": sensitivity._sync,
            CAPTURE_METRICS_INFO_KEY: sensitivity.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_timestep_sensitivity",
    title="Rigid Time Step Sensitivity",
    category="World Rigid Body",
    summary=(
        "Shows how World time-step size changes free-fall integration error, "
        "contact timing, clearance, and step cost."
    ),
    build=build,
)
