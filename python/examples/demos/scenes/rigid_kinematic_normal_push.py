"""Normal prescribed-motion rigid-body pusher verifier for DART 7 Worlds."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_PADDLE_HALF = np.array([0.060, 0.26, 0.20])
_TARGET_HALF = np.array([0.12, 0.12, 0.12])
_TIME_STEP = 0.004
_HISTORY = 180
_INITIAL_GAP = 0.006
_LANE_Y = {
    "ipc_normal": 0.72,
    "ipc_heavy": 0.0,
    "si_caveat": -0.72,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _NormalPushCase:
    key: str
    label: str
    solver: sx.RigidBodySolver
    target_mass_scale: float
    offset_y: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    paddle: Any
    target: Any
    initial_paddle_position: np.ndarray
    initial_target_position: np.ndarray


class _RigidKinematicNormalPush:
    def __init__(self) -> None:
        self.push_speed = 0.45
        self.target_mass = 1.0
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
                "ipc_normal",
                "IPC normal-push caveat",
                sx.RigidBodySolver.IPC,
                1.0,
                (0.22, 0.54, 0.86),
            ),
            self._make_case(
                "ipc_heavy",
                "IPC heavy-target caveat",
                sx.RigidBodySolver.IPC,
                4.0,
                (0.44, 0.70, 0.36),
            ),
            self._make_case(
                "si_caveat",
                "Sequential impulse push",
                sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
                1.0,
                (0.90, 0.44, 0.22),
            ),
        ]
        self._driver_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._target_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._gap_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._depth_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._contact_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._frame_index = 0
        self._reset()

    @property
    def primary_world(self) -> Any:
        return self.cases[0].world

    @property
    def render_world(self) -> Any:
        return self.cases[0].bridge.render_world

    def _make_case(
        self,
        key: str,
        label: str,
        solver: sx.RigidBodySolver,
        target_mass_scale: float,
        color: tuple[float, float, float],
    ) -> _NormalPushCase:
        offset_y = _LANE_Y[key]
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, 0.0),
            rigid_body_solver=solver,
        )
        world.step_profiling_enabled = True

        initial_target_position = np.array([0.0, offset_y, 0.22], dtype=float)
        initial_paddle_position = np.array(
            [
                -(_PADDLE_HALF[0] + _TARGET_HALF[0] + _INITIAL_GAP),
                offset_y,
                initial_target_position[2],
            ],
            dtype=float,
        )
        paddle = world.add_rigid_body(
            f"{key}_normal_paddle", position=tuple(initial_paddle_position)
        )
        paddle.set_collision_shape(sx.CollisionShape.box(_PADDLE_HALF))
        paddle.is_kinematic = True
        paddle.friction = 0.0
        paddle.restitution = 0.0

        target = world.add_rigid_body(
            f"{key}_normal_target", position=tuple(initial_target_position)
        )
        target.set_collision_shape(sx.CollisionShape.box(_TARGET_HALF))
        target.friction = 0.0
        target.restitution = 0.0

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_normal_push_render")
        bridge.add_rigid_body_visual(
            paddle,
            dart.BoxShape(_full(_PADDLE_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{key}_paddle_visual",
        )
        bridge.add_rigid_body_visual(
            target,
            dart.BoxShape(_full(_TARGET_HALF)),
            color,
            name=f"{key}_target_visual",
        )

        start = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_push_start_marker",
            _transform_at(
                np.array(
                    [
                        initial_target_position[0],
                        offset_y,
                        initial_target_position[2] - _TARGET_HALF[2] - 0.018,
                    ],
                    dtype=float,
                )
            ),
        )
        start.set_shape(dart.BoxShape(np.array([0.014, 0.50, 0.012])))
        start.create_visual_aspect().set_color([0.88, 0.88, 0.88])
        bridge.render_world.add_simple_frame(start)

        goal = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_push_goal_marker",
            _transform_at(
                np.array(
                    [
                        initial_target_position[0] + 0.18,
                        offset_y,
                        initial_target_position[2] - _TARGET_HALF[2] - 0.016,
                    ],
                    dtype=float,
                )
            ),
        )
        goal.set_shape(dart.BoxShape(np.array([0.014, 0.50, 0.016])))
        goal.create_visual_aspect().set_color([0.96, 0.78, 0.20])
        bridge.render_world.add_simple_frame(goal)

        case = _NormalPushCase(
            key=key,
            label=label,
            solver=solver,
            target_mass_scale=target_mass_scale,
            offset_y=offset_y,
            color=color,
            world=world,
            bridge=bridge,
            paddle=paddle,
            target=target,
            initial_paddle_position=initial_paddle_position,
            initial_target_position=initial_target_position,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _NormalPushCase) -> None:
        case.target.mass = max(
            0.10, float(self.target_mass) * float(case.target_mass_scale)
        )
        case.target.friction = 0.0
        case.target.restitution = 0.0
        case.paddle.friction = 0.0
        case.paddle.restitution = 0.0

    def _reset_case(self, case: _NormalPushCase) -> None:
        case.paddle.transform = _transform_at(case.initial_paddle_position)
        case.paddle.linear_velocity = (float(self.push_speed), 0.0, 0.0)
        case.paddle.angular_velocity = (0.0, 0.0, 0.0)
        case.paddle.clear_force()
        case.paddle.clear_torque()

        case.target.transform = _transform_at(case.initial_target_position)
        case.target.linear_velocity = (0.0, 0.0, 0.0)
        case.target.angular_velocity = (0.0, 0.0, 0.0)
        case.target.clear_force()
        case.target.clear_torque()

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
            *self._driver_history.values(),
            *self._target_history.values(),
            *self._gap_history.values(),
            *self._depth_history.values(),
            *self._contact_history.values(),
            *self._step_ms_history.values(),
        ):
            history.clear()
        self._last_metrics.clear()
        self._frame_index = 0
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

    def _contact_stats(self, case: _NormalPushCase) -> tuple[float, float]:
        pair = frozenset((case.paddle.name, case.target.name))
        count = 0
        max_depth = 0.0
        for contact in case.world.collide():
            if frozenset((contact.body_a.name, contact.body_b.name)) != pair:
                continue
            count += 1
            max_depth = max(max_depth, float(contact.depth))
        return float(count), max_depth

    def _sample(self, case: _NormalPushCase) -> dict[str, float | str]:
        paddle_position = np.asarray(case.paddle.translation, dtype=float).reshape(3)
        target_position = np.asarray(case.target.translation, dtype=float).reshape(3)
        target_velocity = np.asarray(case.target.linear_velocity, dtype=float).reshape(3)
        driver_travel = float(paddle_position[0] - case.initial_paddle_position[0])
        target_travel = float(target_position[0] - case.initial_target_position[0])
        analytic_gap = float(
            target_position[0]
            - paddle_position[0]
            - (_PADDLE_HALF[0] + _TARGET_HALF[0])
        )
        contact_count, max_depth = self._contact_stats(case)
        target_speed = float(target_velocity[0])
        denominator = max(1.0e-9, abs(float(self.push_speed)))
        speed_ratio = float(target_speed / denominator)
        if (
            case.solver == sx.RigidBodySolver.IPC
            and max_depth > 0.050
            and target_travel < 0.020
        ):
            status = "ipc penetration caveat"
        elif target_travel > 0.10 and abs(analytic_gap) < 0.045:
            status = "pushed"
        elif case.key == "si_caveat" and target_travel < 0.06:
            status = "weak normal push"
        elif analytic_gap > 0.030:
            status = "approaching"
        else:
            status = "partial push"
        return {
            "analytic_gap": analytic_gap,
            "contact_count": contact_count,
            "driver_travel": driver_travel,
            "max_depth": max_depth,
            "solver": "ipc" if case.solver == sx.RigidBodySolver.IPC else "si",
            "status": status,
            "step_ms": self._step_profile_ms(case.world),
            "target_mass": float(case.target.mass),
            "target_speed": target_speed,
            "target_travel": target_travel,
            "speed_ratio": speed_ratio,
        }

    def _record_metrics(self) -> None:
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.key] = metrics
            self._driver_history[case.key].append(float(metrics["driver_travel"]))
            self._target_history[case.key].append(float(metrics["target_travel"]))
            self._gap_history[case.key].append(float(metrics["analytic_gap"]))
            self._depth_history[case.key].append(float(metrics["max_depth"]))
            self._contact_history[case.key].append(float(metrics["contact_count"]))
            self._step_ms_history[case.key].append(float(metrics["step_ms"]))

    def _sync(self) -> None:
        for case in self.cases:
            case.bridge.sync()

    def _drive_paddle(self, case: _NormalPushCase) -> None:
        position = case.initial_paddle_position.copy()
        position[0] += float(self.push_speed) * float(self._frame_index + 1) * _TIME_STEP
        case.paddle.transform = _transform_at(position)
        case.paddle.linear_velocity = (float(self.push_speed), 0.0, 0.0)
        case.paddle.angular_velocity = (0.0, 0.0, 0.0)

    def pre_step(self) -> None:
        executor = self._executor()
        for case in self.cases:
            self._drive_paddle(case)
            case.world.step(executor)
        self._record_metrics()
        self._sync()
        self._frame_index += 1

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

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        executor_label = self._executor_label()
        lanes = {
            case.key: {
                "label": case.label,
                "rigid_body_solver": case.solver.name,
                "target_mass_scale": float(case.target_mass_scale),
                "metrics": dict(self._last_metrics.get(case.key) or self._sample(case)),
            }
            for case in self.cases
        }
        driver_history = {
            case.key: list(self._driver_history[case.key]) for case in self.cases
        }
        target_history = {
            case.key: list(self._target_history[case.key]) for case in self.cases
        }
        gap_history = {
            case.key: list(self._gap_history[case.key]) for case in self.cases
        }
        depth_history = {
            case.key: list(self._depth_history[case.key]) for case in self.cases
        }
        contact_history = {
            case.key: list(self._contact_history[case.key]) for case in self.cases
        }
        step_ms_history = {
            case.key: list(self._step_ms_history[case.key]) for case in self.cases
        }

        def lane_metric(key: str, metric_key: str) -> float:
            return float(lanes[key]["metrics"][metric_key])

        def max_abs(values: list[float]) -> float:
            return max((abs(value) for value in values), default=0.0)

        def min_abs(values: list[float]) -> float:
            return min((abs(value) for value in values), default=0.0)

        target_travel_divergence = abs(
            lane_metric("si_caveat", "target_travel")
            - lane_metric("ipc_normal", "target_travel")
        )
        history_divergence = [
            abs(float(si_value) - float(ipc_value))
            for ipc_value, si_value in zip(
                target_history["ipc_normal"], target_history["si_caveat"]
            )
        ]
        return {
            "row": "rigid_kinematic_normal_push",
            "comparison_axis": "prescribed_normal_contact_response",
            "solver": "ipc_penetration_caveat_vs_sequential_impulse_push",
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "held_fixed": {
                "contact_friction": 0.0,
                "executor": executor_label,
                "kinematic_driver": "normal paddle",
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "executor_index": int(self.executor_index),
                "push_speed": float(self.push_speed),
                "target_mass": float(self.target_mass),
            },
            "case_pair": [case.label for case in self.cases],
            "solver_pair": [case.solver.name for case in self.cases],
            "lane_order": [case.key for case in self.cases],
            "ipc_normal_solver_enum": self.cases[0].solver.name,
            "ipc_normal_target_travel": lane_metric(
                "ipc_normal", "target_travel"
            ),
            "ipc_normal_max_depth": lane_metric("ipc_normal", "max_depth"),
            "ipc_normal_contact_count": lane_metric("ipc_normal", "contact_count"),
            "ipc_normal_speed_ratio": lane_metric("ipc_normal", "speed_ratio"),
            "ipc_heavy_solver_enum": self.cases[1].solver.name,
            "ipc_heavy_target_mass": lane_metric("ipc_heavy", "target_mass"),
            "ipc_heavy_target_travel": lane_metric("ipc_heavy", "target_travel"),
            "ipc_heavy_max_depth": lane_metric("ipc_heavy", "max_depth"),
            "ipc_heavy_contact_count": lane_metric("ipc_heavy", "contact_count"),
            "si_caveat_solver_enum": self.cases[2].solver.name,
            "si_caveat_target_travel": lane_metric("si_caveat", "target_travel"),
            "si_caveat_analytic_gap": lane_metric("si_caveat", "analytic_gap"),
            "si_caveat_contact_count": lane_metric("si_caveat", "contact_count"),
            "si_caveat_speed_ratio": lane_metric("si_caveat", "speed_ratio"),
            "target_travel_divergence": target_travel_divergence,
            "lanes": lanes,
            "history": {
                "samples": float(len(target_history["ipc_normal"])),
                "ipc_normal_max_driver_travel": max(
                    driver_history["ipc_normal"], default=0.0
                ),
                "ipc_normal_max_abs_target_travel": max_abs(
                    target_history["ipc_normal"]
                ),
                "ipc_normal_max_depth": max(depth_history["ipc_normal"], default=0.0),
                "ipc_normal_max_contact_count": max(
                    contact_history["ipc_normal"], default=0.0
                ),
                "ipc_normal_min_abs_gap": min_abs(gap_history["ipc_normal"]),
                "ipc_heavy_max_depth": max(depth_history["ipc_heavy"], default=0.0),
                "ipc_heavy_max_contact_count": max(
                    contact_history["ipc_heavy"], default=0.0
                ),
                "si_caveat_max_target_travel": max(
                    target_history["si_caveat"], default=0.0
                ),
                "si_caveat_min_abs_gap": min_abs(gap_history["si_caveat"]),
                "si_caveat_max_contact_count": max(
                    contact_history["si_caveat"], default=0.0
                ),
                "max_target_travel_divergence": max(
                    history_divergence, default=0.0
                ),
                "max_step_ms": max(
                    (value for values in step_ms_history.values() for value in values),
                    default=0.0,
                ),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "frame_index": int(self._frame_index),
                "push_speed": float(self.push_speed),
                "target_mass": float(self.target_mass),
            },
            "secondary_states": [
                np.asarray(case.world.state_vector, dtype=float).copy()
                for case in self.cases[1:]
            ],
            "secondary_times": [float(case.world.time) for case in self.cases[1:]],
            "driver_history": {
                key: list(values) for key, values in self._driver_history.items()
            },
            "target_history": {
                key: list(values) for key, values in self._target_history.items()
            },
            "gap_history": {
                key: list(values) for key, values in self._gap_history.items()
            },
            "depth_history": {
                key: list(values) for key, values in self._depth_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        histories = snapshot.get("target_history", {})
        if not isinstance(histories, dict):
            return 0.0
        ipc_history = histories.get("ipc_normal", [])
        si_history = histories.get("si_caveat", [])
        if not ipc_history or not si_history:
            return 0.0
        try:
            return abs(float(si_history[-1]) - float(ipc_history[-1]))
        except (TypeError, ValueError):
            return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            ipc = metrics.get("ipc_normal", {})
            if isinstance(ipc, dict):
                try:
                    if float(ipc.get("contact_count", 0.0)) >= 1.0:
                        return 1.0
                    if float(ipc.get("max_depth", 0.0)) >= 0.05:
                        return 1.0
                    if ipc.get("status") == "ipc penetration caveat":
                        return 1.0
                except (TypeError, ValueError):
                    pass
            si = metrics.get("si_caveat", {})
            if isinstance(si, dict):
                try:
                    if float(si.get("contact_count", 0.0)) >= 1.0:
                        return 1.0
                    if float(si.get("target_travel", 0.0)) >= 0.08:
                        return 1.0
                    if si.get("status") == "pushed":
                        return 1.0
                except (TypeError, ValueError):
                    pass
        contact_histories = snapshot.get("contact_history", {})
        if isinstance(contact_histories, dict):
            try:
                for key in ("ipc_normal", "si_caveat"):
                    values = contact_histories.get(key, [])
                    if values and float(values[-1]) >= 1.0:
                        return 1.0
            except (TypeError, ValueError, IndexError):
                pass
        depth_histories = snapshot.get("depth_history", {})
        if isinstance(depth_histories, dict):
            try:
                values = depth_histories.get("ipc_normal", [])
                if values and float(values[-1]) >= 0.05:
                    return 1.0
            except (TypeError, ValueError, IndexError):
                pass
        try:
            if float(self.replay_timeline_signal(snapshot)) >= 0.06:
                return 1.0
        except (TypeError, ValueError):
            return 0.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.push_speed = float(controls.get("push_speed", self.push_speed))
        self.target_mass = float(controls.get("target_mass", self.target_mass))
        self._frame_index = max(0, int(controls.get("frame_index", self._frame_index)))
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        for case in self.cases:
            self._apply_parameters(case)

        secondary_states = list(state.get("secondary_states", []))
        secondary_times = list(state.get("secondary_times", []))
        for index, case in enumerate(self.cases[1:]):
            if index < len(secondary_states):
                case.world.state_vector = secondary_states[index]
            if index < len(secondary_times):
                case.world.time = float(secondary_times[index])
            case.world.update_kinematics()

        self._restore_histories(self._driver_history, state.get("driver_history", {}))
        self._restore_histories(self._target_history, state.get("target_history", {}))
        self._restore_histories(self._gap_history, state.get("gap_history", {}))
        self._restore_histories(self._depth_history, state.get("depth_history", {}))
        self._restore_histories(
            self._contact_history, state.get("contact_history", {})
        )
        self._restore_histories(
            self._step_ms_history, state.get("step_ms_history", {})
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

    def _case_text(self, builder: Any, case: _NormalPushCase) -> None:
        metrics = self._last_metrics.get(case.key) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"driver {float(metrics['driver_travel']):.3f} m | "
            f"target {float(metrics['target_travel']):.3f} m | "
            f"gap {float(metrics['analytic_gap']):.4f} m"
        )
        builder.text(
            f"depth {float(metrics['max_depth']):.4f} m | "
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"speed ratio {float(metrics['speed_ratio']):.2f} | "
            f"step {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_speed, push_speed = builder.slider(
            "Push speed", float(self.push_speed), 0.05, 0.80
        )
        if changed_speed:
            self.push_speed = float(push_speed)

        changed_mass, target_mass = builder.slider(
            "Target mass", float(self.target_mass), 0.20, 5.0
        )
        if changed_mass:
            self.target_mass = float(target_mass)

        if changed_executor or changed_speed or changed_mass:
            self._reset()

        if builder.button("Reset normal push"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: prescribed normal contact response")
        builder.text(
            f"held fixed: executor {self._executor_label()} | "
            f"normal kinematic paddle | zero friction | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver lanes: IPC normal, IPC heavy target, Sequential impulse")
        builder.text("mode: prescribed normal contact")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        builder.plot_lines(
            "IPC normal target travel", list(self._target_history["ipc_normal"])
        )
        builder.plot_lines(
            "IPC heavy target travel", list(self._target_history["ipc_heavy"])
        )
        builder.plot_lines(
            "SI target travel", list(self._target_history["si_caveat"])
        )
        builder.plot_lines("IPC normal gap", list(self._gap_history["ipc_normal"]))
        builder.plot_lines("IPC normal depth", list(self._depth_history["ipc_normal"]))
        builder.plot_lines("SI gap", list(self._gap_history["si_caveat"]))
        builder.separator()
        for case in self.cases:
            case.bridge.build_control_panel(builder, context)
            builder.separator()


def build() -> SceneSetup:
    controller = _RigidKinematicNormalPush()
    return SceneSetup(
        world=controller.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Kinematic Normal Push", controller.build_panel)],
        info={
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
            "sx_world": controller.primary_world,
            "rigid_kinematic_normal_push_controller": controller,
            "rigid_kinematic_normal_push_worlds": [
                case.world for case in controller.cases
            ],
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_sync": controller._sync,
            "replay_timeline": {
                "signal_label": "Target travel divergence",
                "signal": controller.replay_timeline_signal,
                "markers": controller.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_kinematic_normal_push",
    title="Rigid Kinematic Normal Push",
    category="World Rigid Body",
    summary=(
        "Compares prescribed normal contact pushes from a kinematic paddle "
        "across IPC penetration-caveat and sequential-impulse push lanes."
    ),
    build=build,
)
