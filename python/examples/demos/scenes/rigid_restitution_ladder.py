"""Rigid-body restitution ladder for visual bounce diagnostics."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_BALL_RADIUS = 0.08
_GROUND_HALF = np.array([0.32, 0.28, 0.04])
_TIME_STEP = 0.004
_HISTORY = 180
_LANE_X = {
    "dead": -0.84,
    "middle": 0.0,
    "high": 0.84,
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
class _RestitutionLane:
    key: str
    label: str
    preset: float
    color: tuple[float, float, float]
    ground: Any
    ball: Any
    start_position: np.ndarray


class _RigidRestitutionLadder:
    def __init__(self) -> None:
        self.solver_index = 0
        self.executor_index = 0
        self.launch_height = 0.72
        self.restitution_scale = 1.0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(
            time_step=_TIME_STEP,
            rigid_body_solver=self._solver(),
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "dead",
                "Restitution 0.00",
                0.0,
                (0.86, 0.34, 0.24),
            ),
            self._make_lane(
                "middle",
                "Restitution 0.50",
                0.50,
                (0.24, 0.54, 0.90),
            ),
            self._make_lane(
                "high",
                "Restitution 0.90",
                0.90,
                (0.24, 0.68, 0.34),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_restitution_ladder")
        for lane in self.lanes:
            self.bridge.add_rigid_body_visual(
                lane.ground,
                dart.BoxShape(_full(_GROUND_HALF)),
                (0.42, 0.44, 0.47),
                name=f"{lane.key}_restitution_ground_visual",
            )
            self.bridge.add_rigid_body_visual(
                lane.ball,
                dart.SphereShape(_BALL_RADIUS),
                lane.color,
                name=f"{lane.key}_restitution_ball_visual",
            )

        self._height_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._velocity_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._contact_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._energy_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._max_rebound_height: dict[str, float] = {}
        self._max_upward_velocity: dict[str, float] = {}
        self._had_contact: dict[str, bool] = {}
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._reset()

    def _solver(self) -> sx.RigidBodySolver:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][1]

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _lane_restitution(self, lane: _RestitutionLane) -> float:
        return max(0.0, min(1.0, float(lane.preset) * float(self.restitution_scale)))

    def _make_lane(
        self,
        key: str,
        label: str,
        preset: float,
        color: tuple[float, float, float],
    ) -> _RestitutionLane:
        x = _LANE_X[key]
        ground = self.world.add_rigid_body(
            f"{key}_restitution_ground", position=(x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        start_position = np.array([x, 0.0, float(self.launch_height)], dtype=float)
        ball = self.world.add_rigid_body(
            f"{key}_restitution_ball", position=tuple(start_position)
        )
        ball.mass = 1.0
        ball.set_collision_shape(sx.CollisionShape.sphere(_BALL_RADIUS))
        ball.friction = 0.0
        return _RestitutionLane(
            key=key,
            label=label,
            preset=preset,
            color=color,
            ground=ground,
            ball=ball,
            start_position=start_position,
        )

    def _apply_parameters(self, lane: _RestitutionLane) -> None:
        restitution = self._lane_restitution(lane)
        lane.ground.restitution = restitution
        lane.ground.friction = 0.0
        lane.ball.restitution = restitution
        lane.ball.friction = 0.0

    def _reset_lane(self, lane: _RestitutionLane) -> None:
        lane.start_position = np.array(
            [_LANE_X[lane.key], 0.0, float(self.launch_height)], dtype=float
        )
        lane.ball.transform = _transform_at(lane.start_position)
        lane.ball.linear_velocity = (0.0, 0.0, 0.0)
        lane.ball.angular_velocity = (0.0, 0.0, 0.0)
        lane.ball.clear_force()
        lane.ball.clear_torque()

    def _reset(self) -> None:
        self.world.rigid_body_solver = self._solver()
        for lane in self.lanes:
            self._apply_parameters(lane)
            self._reset_lane(lane)
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._height_history.values(),
            *self._velocity_history.values(),
            *self._contact_history.values(),
            *self._energy_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._max_rebound_height = {lane.key: 0.0 for lane in self.lanes}
        self._max_upward_velocity = {lane.key: 0.0 for lane in self.lanes}
        self._had_contact = {lane.key: False for lane in self.lanes}
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

    def _contact_counts(self) -> dict[str, int]:
        counts = {lane.key: 0 for lane in self.lanes}
        lane_names = {
            lane.key: {lane.ground.name, lane.ball.name} for lane in self.lanes
        }
        for contact in self.world.collide():
            names = {contact.body_a.name, contact.body_b.name}
            for key, expected_names in lane_names.items():
                if names == expected_names:
                    counts[key] += 1
        return counts

    def _sample(
        self, lane: _RestitutionLane, contact_count: int
    ) -> dict[str, float | str]:
        height = float(np.asarray(lane.ball.translation, dtype=float)[2])
        velocity_z = float(np.asarray(lane.ball.linear_velocity, dtype=float)[2])
        kinetic_energy = float(lane.ball.kinetic_energy)
        potential_energy = float(lane.ball.potential_energy)
        total_energy = kinetic_energy + potential_energy

        if contact_count > 0:
            self._had_contact[lane.key] = True
        if self._had_contact[lane.key]:
            self._max_rebound_height[lane.key] = max(
                self._max_rebound_height[lane.key], height
            )
            self._max_upward_velocity[lane.key] = max(
                self._max_upward_velocity[lane.key], velocity_z
            )

        if contact_count > 0:
            status = "contact"
        elif not self._had_contact[lane.key]:
            status = "falling"
        elif velocity_z > 0.03:
            status = "rebounding"
        elif abs(velocity_z) < 0.03 and height < _BALL_RADIUS + 0.02:
            status = "settled"
        else:
            status = "descending"

        return {
            "contact_count": float(contact_count),
            "height": height,
            "kinetic_energy": kinetic_energy,
            "max_rebound_height": self._max_rebound_height[lane.key],
            "max_upward_velocity": self._max_upward_velocity[lane.key],
            "potential_energy": potential_energy,
            "restitution": self._lane_restitution(lane),
            "status": status,
            "total_energy": total_energy,
            "vertical_velocity": velocity_z,
        }

    def _record_metrics(self) -> None:
        contact_counts = self._contact_counts()
        for lane in self.lanes:
            metrics = self._sample(lane, contact_counts[lane.key])
            self._last_metrics[lane.key] = metrics
            self._height_history[lane.key].append(float(metrics["height"]))
            self._velocity_history[lane.key].append(
                float(metrics["vertical_velocity"])
            )
            self._contact_history[lane.key].append(float(metrics["contact_count"]))
            self._energy_history[lane.key].append(float(metrics["total_energy"]))
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        self.world.step(self._executor())
        self._record_metrics()
        self.bridge.sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "launch_height": float(self.launch_height),
                "restitution_scale": float(self.restitution_scale),
                "solver_index": int(self.solver_index),
            },
            "height_history": {
                key: list(values) for key, values in self._height_history.items()
            },
            "velocity_history": {
                key: list(values) for key, values in self._velocity_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "energy_history": {
                key: list(values) for key, values in self._energy_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "max_rebound_height": dict(self._max_rebound_height),
            "max_upward_velocity": dict(self._max_upward_velocity),
            "had_contact": dict(self._had_contact),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {
            lane.key: dict(self._last_metrics.get(lane.key) or self._sample(lane, 0))
            for lane in self.lanes
        }
        dead = metrics["dead"]
        middle = metrics["middle"]
        high = metrics["high"]
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_restitution_ladder",
            "solver": _SOLVERS[int(self.solver_index)][0],
            "solver_enum": self._solver().name,
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "matched_restitution_bounce_ladder",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "launch_height": float(self.launch_height),
            "restitution_scale": float(self.restitution_scale),
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "solver_index": float(self.solver_index),
                "executor_index": float(self.executor_index),
                "launch_height": float(self.launch_height),
                "restitution_scale": float(self.restitution_scale),
            },
            "dead_rebound_height": float(dead["max_rebound_height"]),
            "middle_rebound_height": float(middle["max_rebound_height"]),
            "high_rebound_height": float(high["max_rebound_height"]),
            "high_upward_velocity": float(high["max_upward_velocity"]),
            "dead_contact_count": float(dead["contact_count"]),
            "middle_contact_count": float(middle["contact_count"]),
            "high_contact_count": float(high["contact_count"]),
            "step_ms": float(step_values[-1]) if step_values else 0.0,
            "history": {
                "samples": float(len(step_values)),
                "max_dead_rebound_height": max(
                    (float(value) for value in self._height_history["dead"]),
                    default=float(dead["height"]),
                ),
                "max_middle_rebound_height": float(
                    self._max_rebound_height.get("middle", 0.0)
                ),
                "max_high_rebound_height": float(
                    self._max_rebound_height.get("high", 0.0)
                ),
                "max_high_upward_velocity": float(
                    self._max_upward_velocity.get("high", 0.0)
                ),
                "max_contact_count": max(
                    (
                        float(value)
                        for history in self._contact_history.values()
                        for value in history
                    ),
                    default=0.0,
                ),
                "max_step_ms": max((float(value) for value in step_values), default=0.0),
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.solver_index = max(
            0,
            min(
                int(controls.get("solver_index", self.solver_index)),
                max(0, len(_SOLVERS) - 1),
            ),
        )
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.launch_height = float(controls.get("launch_height", self.launch_height))
        self.restitution_scale = float(
            controls.get("restitution_scale", self.restitution_scale)
        )
        self.world.rigid_body_solver = self._solver()
        for lane in self.lanes:
            self._apply_parameters(lane)
        self._restore_histories(self._height_history, state.get("height_history", {}))
        self._restore_histories(
            self._velocity_history, state.get("velocity_history", {})
        )
        self._restore_histories(
            self._contact_history, state.get("contact_history", {})
        )
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._max_rebound_height = {
            str(key): float(value)
            for key, value in state.get("max_rebound_height", {}).items()
        }
        self._max_upward_velocity = {
            str(key): float(value)
            for key, value in state.get("max_upward_velocity", {}).items()
        }
        self._had_contact = {
            str(key): bool(value) for key, value in state.get("had_contact", {}).items()
        }
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

    def _lane_text(self, builder: Any, lane: _RestitutionLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane, 0)
        builder.text(
            f"{lane.label}: e {float(metrics['restitution']):.2f}, "
            f"{metrics['status']}"
        )
        builder.text(
            f"  height {float(metrics['height']):.3f} m | "
            f"vz {float(metrics['vertical_velocity']):.3f} m/s | "
            f"rebound {float(metrics['max_rebound_height']):.3f} m"
        )
        builder.text(
            f"  total energy {float(metrics['total_energy']):.3f} J | "
            f"contacts {float(metrics['contact_count']):.0f}"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_solver, solver_index = builder.select(
            "Solver", int(self.solver_index), [label for label, _solver in _SOLVERS]
        )
        if changed_solver:
            self.solver_index = int(solver_index)

        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), [label for label, _ in self._executors]
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_height, launch_height = builder.slider(
            "Launch height", float(self.launch_height), 0.28, 1.10
        )
        if changed_height:
            self.launch_height = float(launch_height)

        changed_scale, restitution_scale = builder.slider(
            "Restitution scale", float(self.restitution_scale), 0.0, 1.0
        )
        if changed_scale:
            self.restitution_scale = float(restitution_scale)

        if changed_solver or changed_executor or changed_height or changed_scale:
            self._reset()

        if builder.button("Reset bounce"):
            self._reset()

        builder.separator()
        builder.text(f"solver: {_SOLVERS[self.solver_index][0]}")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        if self._step_ms_history:
            builder.text(f"step profile: {self._step_ms_history[-1]:.3f} ms")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} height", list(self._height_history[lane.key])
            )
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} vertical velocity",
                list(self._velocity_history[lane.key]),
            )
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} total energy", list(self._energy_history[lane.key])
            )
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    ladder = _RigidRestitutionLadder()
    return SceneSetup(
        world=ladder.bridge.render_world,
        pre_step=ladder.pre_step,
        force_drag=ladder.bridge.force_drag,
        renderable_provider=ladder.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Restitution Ladder", ladder.build_panel)],
        info={
            "sx_world": ladder.world,
            "rigid_restitution_ladder_controller": ladder,
            "replay_capture_state": ladder.capture_replay_state,
            "replay_restore_state": ladder.restore_replay_state,
            "replay_sync": ladder.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: ladder.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_restitution_ladder",
    title="Rigid Restitution Ladder",
    category="World Rigid Body",
    summary=(
        "Three matched bounce lanes show how restitution changes rebound height "
        "and energy trends."
    ),
    build=build,
)
