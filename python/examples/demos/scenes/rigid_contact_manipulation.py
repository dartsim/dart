"""Contact-rich rigid pusher manipulation verifier for DART 7 World solvers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = np.array([0.90, 0.38, 0.05])
_TARGET_HALF = np.array([0.13, 0.13, 0.13])
_PUSHER_HALF = np.array([0.10, 0.18, 0.12])
_LEFT_OFFSET = -1.05
_RIGHT_OFFSET = 1.05
_TIME_STEP = 0.004
_HISTORY = 180
_TARGET_RELATIVE_X = -0.10
_PUSHER_RELATIVE_X = -0.38
_GOAL_RELATIVE_X = 0.28


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _ManipulationCase:
    label: str
    solver: sx.RigidBodySolver
    offset_x: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    target: Any
    pusher: Any
    initial_target_position: np.ndarray
    initial_pusher_position: np.ndarray


class _RigidContactManipulation:
    def __init__(self) -> None:
        self.friction = 0.18
        self.launch_speed = 1.20
        self.pusher_mass = 10.0
        self.executor_index = 0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.cases = [
            self._make_case(
                "Sequential impulse",
                sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
                _LEFT_OFFSET,
                (0.22, 0.54, 0.86),
                "si",
            ),
            self._make_case(
                "IPC barrier",
                sx.RigidBodySolver.IPC,
                _RIGHT_OFFSET,
                (0.90, 0.44, 0.22),
                "ipc",
            ),
        ]
        self._travel_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._gap_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._contact_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._drift_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._goal_error_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._divergence_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._reset()

    @property
    def primary_world(self) -> Any:
        return self.cases[0].world

    @property
    def render_world(self) -> Any:
        return self.cases[0].bridge.render_world

    def _make_case(
        self,
        label: str,
        solver: sx.RigidBodySolver,
        offset_x: float,
        color: tuple[float, float, float],
        prefix: str,
    ) -> _ManipulationCase:
        world = sx.World(time_step=_TIME_STEP, rigid_body_solver=solver)
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(
            f"{prefix}_push_table", position=(offset_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        initial_target_position = np.array(
            [offset_x + _TARGET_RELATIVE_X, 0.0, _TARGET_HALF[2] + 0.004],
            dtype=float,
        )
        target = world.add_rigid_body(
            f"{prefix}_push_target", position=tuple(initial_target_position)
        )
        target.mass = 1.0
        target.set_collision_shape(sx.CollisionShape.box(_TARGET_HALF))

        initial_pusher_position = np.array(
            [offset_x + _PUSHER_RELATIVE_X, 0.0, _PUSHER_HALF[2] + 0.004],
            dtype=float,
        )
        pusher = world.add_rigid_body(
            f"{prefix}_push_pusher", position=tuple(initial_pusher_position)
        )
        pusher.mass = float(self.pusher_mass)
        pusher.set_collision_shape(sx.CollisionShape.box(_PUSHER_HALF))

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{prefix}_contact_manipulation_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{prefix}_push_table_visual",
        )
        bridge.add_rigid_body_visual(
            target,
            dart.BoxShape(_full(_TARGET_HALF)),
            color,
            name=f"{prefix}_push_target_visual",
        )
        bridge.add_rigid_body_visual(
            pusher,
            dart.BoxShape(_full(_PUSHER_HALF)),
            (0.22, 0.68, 0.46),
            name=f"{prefix}_push_pusher_visual",
        )

        goal = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{prefix}_push_goal_strip",
            _transform_at(
                np.array(
                    [
                        offset_x + _GOAL_RELATIVE_X,
                        0.0,
                        _GROUND_HALF[2] + 0.006,
                    ],
                    dtype=float,
                )
            ),
        )
        goal.set_shape(dart.BoxShape(np.array([0.025, 0.62, 0.012])))
        goal.create_visual_aspect().set_color([0.95, 0.82, 0.28])
        bridge.render_world.add_simple_frame(goal)

        case = _ManipulationCase(
            label=label,
            solver=solver,
            offset_x=offset_x,
            color=color,
            world=world,
            bridge=bridge,
            ground=ground,
            target=target,
            pusher=pusher,
            initial_target_position=initial_target_position,
            initial_pusher_position=initial_pusher_position,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _ManipulationCase) -> None:
        case.ground.friction = float(self.friction)
        case.ground.restitution = 0.0
        for body in (case.target, case.pusher):
            body.friction = float(self.friction)
            body.restitution = 0.01
        case.target.mass = 1.0
        case.pusher.mass = float(self.pusher_mass)

    def _reset_case(self, case: _ManipulationCase) -> None:
        case.target.transform = _transform_at(case.initial_target_position)
        case.target.linear_velocity = (0.0, 0.0, 0.0)
        case.target.angular_velocity = (0.0, 0.0, 0.0)
        case.target.clear_force()
        case.target.clear_torque()

        case.pusher.transform = _transform_at(case.initial_pusher_position)
        case.pusher.linear_velocity = (float(self.launch_speed), 0.0, 0.0)
        case.pusher.angular_velocity = (0.0, 0.0, 0.0)
        case.pusher.clear_force()
        case.pusher.clear_torque()

        case.world.time = 0.0
        try:
            case.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        case.world.update_kinematics()

    def _reset(self) -> None:
        for case in self.cases:
            self._apply_parameters(case)
            self._reset_case(case)
        for history in (
            *self._travel_history.values(),
            *self._gap_history.values(),
            *self._contact_history.values(),
            *self._speed_history.values(),
            *self._drift_history.values(),
            *self._goal_error_history.values(),
            *self._step_ms_history.values(),
            self._divergence_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][0]

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _push_contact_count(self, case: _ManipulationCase) -> int:
        pair = frozenset((case.target.name, case.pusher.name))
        return sum(
            1
            for contact in case.world.collide()
            if frozenset((contact.body_a.name, contact.body_b.name)) == pair
        )

    def _sample(self, case: _ManipulationCase) -> dict[str, float | str]:
        target_position = np.asarray(case.target.translation, dtype=float).reshape(3)
        pusher_position = np.asarray(case.pusher.translation, dtype=float).reshape(3)
        target_velocity = np.asarray(case.target.linear_velocity, dtype=float).reshape(3)
        pusher_velocity = np.asarray(case.pusher.linear_velocity, dtype=float).reshape(3)
        target_travel = float(target_position[0] - case.initial_target_position[0])
        pusher_travel = float(pusher_position[0] - case.initial_pusher_position[0])
        gap = float(
            target_position[0]
            - _TARGET_HALF[0]
            - (pusher_position[0] + _PUSHER_HALF[0])
        )
        contact_count = float(self._push_contact_count(case))
        proximity_active = contact_count > 0.0 or gap < 0.025
        lateral_drift = float(abs(target_position[1] - case.initial_target_position[1]))
        target_speed = float(np.linalg.norm(target_velocity))
        pusher_speed = float(np.linalg.norm(pusher_velocity))
        goal_error = float(_GOAL_RELATIVE_X - (target_position[0] - case.offset_x))
        if target_travel > 0.20:
            status = "object pushed"
        elif proximity_active:
            status = "push contact"
        elif pusher_speed < 0.05:
            status = "stalled"
        else:
            status = "approaching"
        return {
            "contact_count": contact_count,
            "gap": gap,
            "goal_error": goal_error,
            "lateral_drift": lateral_drift,
            "pusher_speed": pusher_speed,
            "pusher_travel": pusher_travel,
            "status": status,
            "step_ms": self._step_profile_ms(case.world),
            "target_speed": target_speed,
            "target_travel": target_travel,
        }

    def _record_metrics(self) -> None:
        travels = []
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._travel_history[case.label].append(float(metrics["target_travel"]))
            self._gap_history[case.label].append(float(metrics["gap"]))
            self._contact_history[case.label].append(float(metrics["contact_count"]))
            self._speed_history[case.label].append(float(metrics["target_speed"]))
            self._drift_history[case.label].append(float(metrics["lateral_drift"]))
            self._goal_error_history[case.label].append(float(metrics["goal_error"]))
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))
            travels.append(float(metrics["target_travel"]))
        if len(travels) == 2:
            self._divergence_history.append(abs(travels[0] - travels[1]))

    def _sync(self) -> None:
        for case in self.cases:
            case.bridge.sync()

    def pre_step(self) -> None:
        executor = self._executor()
        for case in self.cases:
            case.world.step(executor)
        self._record_metrics()
        self._sync()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        executor_label = self._executor_label()
        cases = {
            case.label: {
                "label": case.label,
                "rigid_body_solver": case.solver.name,
                "metrics": dict(self._last_metrics[case.label]),
            }
            for case in self.cases
        }
        travel_history = {
            case.label: list(self._travel_history[case.label]) for case in self.cases
        }
        gap_history = {
            case.label: list(self._gap_history[case.label]) for case in self.cases
        }
        contact_history = {
            case.label: list(self._contact_history[case.label]) for case in self.cases
        }
        speed_history = {
            case.label: list(self._speed_history[case.label]) for case in self.cases
        }
        drift_history = {
            case.label: list(self._drift_history[case.label]) for case in self.cases
        }
        goal_error_history = {
            case.label: list(self._goal_error_history[case.label])
            for case in self.cases
        }
        step_ms_history = {
            case.label: list(self._step_ms_history[case.label])
            for case in self.cases
        }
        si_label = self.cases[0].label
        ipc_label = self.cases[1].label

        def case_metric(label: str, metric_key: str) -> float:
            return float(self._last_metrics[label][metric_key])

        def min_abs(values: list[float]) -> float:
            return min((abs(value) for value in values), default=0.0)

        return {
            "row": "rigid_contact_manipulation",
            "comparison_axis": "rigid_pusher_contact_response",
            "solver": "sequential_impulse_vs_ipc",
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "held_fixed": {
                "executor": executor_label,
                "goal_relative_x": _GOAL_RELATIVE_X,
                "target_mass": 1.0,
                "time_step_ms": _TIME_STEP * 1000.0,
                "workspace": "matched table pusher lanes",
            },
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "pusher_mass": float(self.pusher_mass),
            },
            "case_pair": [case.label for case in self.cases],
            "solver_pair": [case.solver.name for case in self.cases],
            "lane_order": [case.label for case in self.cases],
            "sequential_impulse_status": str(
                self._last_metrics[si_label]["status"]
            ),
            "sequential_impulse_solver_enum": self.cases[0].solver.name,
            "sequential_impulse_target_travel": case_metric(
                si_label, "target_travel"
            ),
            "sequential_impulse_gap": case_metric(si_label, "gap"),
            "sequential_impulse_contact_count": case_metric(
                si_label, "contact_count"
            ),
            "sequential_impulse_target_speed": case_metric(
                si_label, "target_speed"
            ),
            "sequential_impulse_lateral_drift": case_metric(
                si_label, "lateral_drift"
            ),
            "sequential_impulse_goal_error": case_metric(si_label, "goal_error"),
            "sequential_impulse_step_ms": case_metric(si_label, "step_ms"),
            "sequential_impulse_max_contact_count": max(
                contact_history[si_label], default=0.0
            ),
            "sequential_impulse_min_gap": min(gap_history[si_label], default=0.0),
            "ipc_status": str(self._last_metrics[ipc_label]["status"]),
            "ipc_solver_enum": self.cases[1].solver.name,
            "ipc_target_travel": case_metric(ipc_label, "target_travel"),
            "ipc_gap": case_metric(ipc_label, "gap"),
            "ipc_contact_count": case_metric(ipc_label, "contact_count"),
            "ipc_target_speed": case_metric(ipc_label, "target_speed"),
            "ipc_lateral_drift": case_metric(ipc_label, "lateral_drift"),
            "ipc_goal_error": case_metric(ipc_label, "goal_error"),
            "ipc_step_ms": case_metric(ipc_label, "step_ms"),
            "ipc_max_contact_count": max(contact_history[ipc_label], default=0.0),
            "ipc_min_gap": min(gap_history[ipc_label], default=0.0),
            "travel_divergence": (
                float(self._divergence_history[-1])
                if self._divergence_history
                else 0.0
            ),
            "cases": cases,
            "history": {
                "samples": float(len(travel_history[si_label])),
                "max_travel_divergence": max(self._divergence_history, default=0.0),
                "sequential_impulse_max_target_travel": max(
                    travel_history[si_label], default=0.0
                ),
                "sequential_impulse_min_gap": min(gap_history[si_label], default=0.0),
                "sequential_impulse_max_contact_count": max(
                    contact_history[si_label], default=0.0
                ),
                "sequential_impulse_max_target_speed": max(
                    speed_history[si_label], default=0.0
                ),
                "sequential_impulse_max_lateral_drift": max(
                    drift_history[si_label], default=0.0
                ),
                "sequential_impulse_min_abs_goal_error": min_abs(
                    goal_error_history[si_label]
                ),
                "ipc_max_target_travel": max(travel_history[ipc_label], default=0.0),
                "ipc_min_gap": min(gap_history[ipc_label], default=0.0),
                "ipc_max_contact_count": max(contact_history[ipc_label], default=0.0),
                "ipc_max_target_speed": max(speed_history[ipc_label], default=0.0),
                "ipc_max_lateral_drift": max(drift_history[ipc_label], default=0.0),
                "ipc_min_abs_goal_error": min_abs(goal_error_history[ipc_label]),
                "max_step_ms": max(
                    (value for values in step_ms_history.values() for value in values),
                    default=0.0,
                ),
            },
        }

    def force_drag(self, event: dict[str, Any]) -> None:
        if not event.get("active", False):
            for case in self.cases:
                case.bridge.force_drag(event)
            return
        renderable_id = int(event.get("renderable_id", 0) or 0)
        for case in self.cases:
            if renderable_id in getattr(case.bridge, "_by_renderable_id", {}):
                case.bridge.force_drag(event)
                return

    def renderable_provider(self) -> list[Any]:
        renderables: list[Any] = []
        for case in self.cases:
            renderables.extend(case.bridge.renderable_provider())
        return renderables

    def capture_replay_state(self) -> dict[str, Any]:
        secondary = self.cases[1].world
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "pusher_mass": float(self.pusher_mass),
            },
            "secondary_state": np.asarray(secondary.state_vector, dtype=float).copy(),
            "secondary_time": float(secondary.time),
            "travel_history": {
                key: list(values) for key, values in self._travel_history.items()
            },
            "gap_history": {
                key: list(values) for key, values in self._gap_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "drift_history": {
                key: list(values) for key, values in self._drift_history.items()
            },
            "goal_error_history": {
                key: list(values)
                for key, values in self._goal_error_history.items()
            },
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "divergence_history": list(self._divergence_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        history = snapshot.get("divergence_history", [])
        if not history:
            return 0.0
        try:
            return float(history[-1])
        except (TypeError, ValueError):
            return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            for case_metrics in metrics.values():
                if not isinstance(case_metrics, dict):
                    continue
                if case_metrics.get("status") in {"push contact", "object pushed"}:
                    return 1.0
                try:
                    if float(case_metrics.get("contact_count", 0.0)) >= 1.0:
                        return 1.0
                    if float(case_metrics.get("gap", 1.0)) <= 0.025:
                        return 1.0
                    if float(case_metrics.get("target_travel", 0.0)) >= 0.08:
                        return 1.0
                except (TypeError, ValueError):
                    continue
        contact_histories = snapshot.get("contact_history", {})
        if isinstance(contact_histories, dict):
            for values in contact_histories.values():
                try:
                    if values and float(values[-1]) >= 1.0:
                        return 1.0
                except (TypeError, ValueError, IndexError):
                    continue
        gap_histories = snapshot.get("gap_history", {})
        if isinstance(gap_histories, dict):
            for values in gap_histories.values():
                try:
                    if values and float(values[-1]) <= 0.025:
                        return 1.0
                except (TypeError, ValueError, IndexError):
                    continue
        try:
            if float(self.replay_timeline_signal(snapshot)) >= 0.01:
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
        self.friction = float(controls.get("friction", self.friction))
        self.launch_speed = float(controls.get("launch_speed", self.launch_speed))
        self.pusher_mass = float(controls.get("pusher_mass", self.pusher_mass))
        for case in self.cases:
            self._apply_parameters(case)
        secondary = self.cases[1].world
        secondary.state_vector = state["secondary_state"]
        secondary.time = float(state["secondary_time"])
        self._restore_histories(self._travel_history, state.get("travel_history", {}))
        self._restore_histories(self._gap_history, state.get("gap_history", {}))
        self._restore_histories(self._contact_history, state.get("contact_history", {}))
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(self._drift_history, state.get("drift_history", {}))
        self._restore_histories(
            self._goal_error_history, state.get("goal_error_history", {})
        )
        self._restore_histories(
            self._step_ms_history, state.get("step_ms_history", {})
        )
        self._divergence_history.clear()
        self._divergence_history.extend(
            float(value) for value in state.get("divergence_history", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        secondary.update_kinematics()
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _case_text(self, builder: Any, case: _ManipulationCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"travel {float(metrics['target_travel']):.3f} m | "
            f"gap {float(metrics['gap']):.4f} m | "
            f"push contacts {float(metrics['contact_count']):.0f}"
        )
        builder.text(
            f"target speed {float(metrics['target_speed']):.3f} m/s | "
            f"lateral drift {float(metrics['lateral_drift']):.4f} m | "
            f"step {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_speed, launch_speed = builder.slider(
            "Pusher launch speed", float(self.launch_speed), 0.2, 2.0
        )
        if changed_speed:
            self.launch_speed = float(launch_speed)

        changed_friction, friction = builder.slider(
            "Table friction", float(self.friction), 0.05, 0.95
        )
        if changed_friction:
            self.friction = float(friction)

        changed_mass, pusher_mass = builder.slider(
            "Pusher mass", float(self.pusher_mass), 1.0, 16.0
        )
        if changed_mass:
            self.pusher_mass = float(pusher_mass)

        if changed_executor or changed_speed or changed_friction or changed_mass:
            self._reset()

        if builder.button("Reset push"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: rigid pusher contact response")
        builder.text(
            f"held fixed: executor {self._executor_label()} | shared table/goal | "
            f"target mass 1.0 | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver pair: Sequential impulse vs IPC barrier")
        builder.text("task: block pusher moves a table object toward the goal strip")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        if self._divergence_history:
            builder.text(
                f"object-travel divergence: {self._divergence_history[-1]:.4f} m"
            )
        builder.plot_lines(
            "SI target travel", list(self._travel_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC target travel", list(self._travel_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI pusher gap", list(self._gap_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC pusher gap", list(self._gap_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI push contacts", list(self._contact_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC push contacts", list(self._contact_history[self.cases[1].label])
        )
        builder.plot_lines("Travel divergence", list(self._divergence_history))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)
        builder.separator()
        self.cases[1].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    manipulation = _RigidContactManipulation()
    return SceneSetup(
        world=manipulation.render_world,
        pre_step=manipulation.pre_step,
        force_drag=manipulation.force_drag,
        renderable_provider=manipulation.renderable_provider,
        panels=[ScenePanel("Rigid Contact Manipulation", manipulation.build_panel)],
        info={
            "sx_world": manipulation.primary_world,
            "rigid_contact_manipulation_controller": manipulation,
            "rigid_contact_manipulation_worlds": [
                case.world for case in manipulation.cases
            ],
            CAPTURE_METRICS_INFO_KEY: manipulation.capture_metrics,
            "replay_capture_state": manipulation.capture_replay_state,
            "replay_restore_state": manipulation.restore_replay_state,
            "replay_sync": manipulation._sync,
            "replay_timeline": {
                "signal_label": "Travel divergence",
                "signal": manipulation.replay_timeline_signal,
                "markers": manipulation.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_contact_manipulation",
    title="Rigid Contact Manipulation",
    category="World Rigid Body",
    summary=(
        "Sequential impulse and IPC push the same table object with a moving "
        "rigid pusher."
    ),
    build=build,
)
