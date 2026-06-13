"""Side-by-side rigid stack stability verifier for DART 7 World solvers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = np.array([0.72, 0.36, 0.04])
_BOX_HALF = np.array([0.12, 0.12, 0.10])
_TIME_STEP = 0.006
_INITIAL_GAP = 0.004
_HISTORY = 160
_LEFT_OFFSET = -0.82
_RIGHT_OFFSET = 0.82


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _StackCase:
    label: str
    solver: sx.RigidBodySolver
    offset_x: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    boxes: list[Any]
    initial_positions: list[np.ndarray]


class _RigidStackStability:
    def __init__(self) -> None:
        self.friction = 0.80
        self.top_mass_ratio = 20.0
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
                (0.24, 0.52, 0.90),
                "si",
            ),
            self._make_case(
                "IPC barrier",
                sx.RigidBodySolver.IPC,
                _RIGHT_OFFSET,
                (0.90, 0.42, 0.22),
                "ipc",
            ),
        ]
        self._speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._drift_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._clearance_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._height_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._delta_history: deque[float] = deque(maxlen=_HISTORY)
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
    ) -> _StackCase:
        world = sx.World(time_step=_TIME_STEP, rigid_body_solver=solver)
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(
            f"{prefix}_stack_ground", position=(offset_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        boxes: list[Any] = []
        initial_positions: list[np.ndarray] = []
        for index in range(2):
            lateral = (index - 0.5) * 0.008
            position = np.array(
                [
                    offset_x + lateral,
                    0.0,
                    _BOX_HALF[2] + index * (2.0 * _BOX_HALF[2] + _INITIAL_GAP),
                ],
                dtype=float,
            )
            box = world.add_rigid_body(f"{prefix}_stack_box{index}")
            box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
            boxes.append(box)
            initial_positions.append(position)

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{prefix}_stack_stability_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.42, 0.44, 0.47),
            name=f"{prefix}_stack_ground_visual",
        )
        for index, box in enumerate(boxes):
            shade = 0.10 * index
            bridge.add_rigid_body_visual(
                box,
                dart.BoxShape(_full(_BOX_HALF)),
                (
                    min(color[0] + shade, 1.0),
                    min(color[1] + shade, 1.0),
                    min(color[2] + shade, 1.0),
                ),
                name=f"{prefix}_stack_box{index}_visual",
            )

        case = _StackCase(
            label=label,
            solver=solver,
            offset_x=offset_x,
            color=color,
            world=world,
            bridge=bridge,
            ground=ground,
            boxes=boxes,
            initial_positions=initial_positions,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _StackCase) -> None:
        case.ground.friction = float(self.friction)
        case.ground.restitution = 0.0
        for index, box in enumerate(case.boxes):
            box.mass = 1.0 if index == 0 else float(self.top_mass_ratio)
            box.friction = float(self.friction)
            box.restitution = 0.0

    def _reset_case(self, case: _StackCase) -> None:
        for box, position in zip(case.boxes, case.initial_positions, strict=True):
            box.transform = _transform_at(position)
            box.linear_velocity = (0.0, 0.0, 0.0)
            box.angular_velocity = (0.0, 0.0, 0.0)
            box.clear_force()
            box.clear_torque()
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
            *self._speed_history.values(),
            *self._drift_history.values(),
            *self._clearance_history.values(),
            *self._height_history.values(),
            *self._step_ms_history.values(),
            self._delta_history,
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

    def _sample(self, case: _StackCase) -> dict[str, float | str]:
        positions = [
            np.asarray(box.translation, dtype=float) for box in case.boxes
        ]
        velocities = [
            np.asarray(box.linear_velocity, dtype=float) for box in case.boxes
        ]
        top_position = positions[-1]
        bottom_position = positions[0]
        top_initial = case.initial_positions[-1]
        max_speed = max(float(np.linalg.norm(velocity)) for velocity in velocities)
        top_drift = float(
            np.linalg.norm((top_position - top_initial)[0:2])
        )
        bottom_ground_clearance = float(bottom_position[2] - _BOX_HALF[2])
        inter_box_clearance = float(
            top_position[2] - bottom_position[2] - 2.0 * _BOX_HALF[2]
        )
        min_clearance = min(bottom_ground_clearance, inter_box_clearance)
        top_height = float(top_position[2])
        ideal_top_height = float(_BOX_HALF[2] + 2.0 * _BOX_HALF[2])
        height_error = float(top_height - ideal_top_height)
        if min_clearance < -0.004:
            status = "overlapping"
        elif top_drift > 0.12:
            status = "collapsed"
        elif max_speed < 0.015:
            status = "stable"
        else:
            status = "settling"
        return {
            "height_error": height_error,
            "max_speed": max_speed,
            "min_clearance": min_clearance,
            "step_ms": self._step_profile_ms(case.world),
            "status": status,
            "top_drift": top_drift,
            "top_height": top_height,
        }

    def _record_metrics(self) -> None:
        centered_top_x: list[float] = []
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._speed_history[case.label].append(float(metrics["max_speed"]))
            self._drift_history[case.label].append(float(metrics["top_drift"]))
            self._clearance_history[case.label].append(
                float(metrics["min_clearance"])
            )
            self._height_history[case.label].append(float(metrics["top_height"]))
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))
            centered_top_x.append(
                float(np.asarray(case.boxes[-1].translation, dtype=float)[0])
                - case.offset_x
            )
        if len(centered_top_x) == 2:
            self._delta_history.append(abs(centered_top_x[0] - centered_top_x[1]))

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
        cases = {
            case.label: {
                "rigid_body_solver": case.solver.name,
                "metrics": dict(self._last_metrics[case.label]),
            }
            for case in self.cases
        }
        speed_history = {
            case.label: list(self._speed_history[case.label]) for case in self.cases
        }
        drift_history = {
            case.label: list(self._drift_history[case.label]) for case in self.cases
        }
        clearance_history = {
            case.label: list(self._clearance_history[case.label])
            for case in self.cases
        }
        height_history = {
            case.label: list(self._height_history[case.label]) for case in self.cases
        }
        step_ms_history = {
            case.label: list(self._step_ms_history[case.label])
            for case in self.cases
        }
        si_label = self.cases[0].label
        ipc_label = self.cases[1].label
        ideal_top_height = float(_BOX_HALF[2] + 2.0 * _BOX_HALF[2])

        def case_metric(label: str, metric_key: str) -> float:
            return float(self._last_metrics[label][metric_key])

        executor_label = self._executor_label()
        return {
            "row": "rigid_stack_stability",
            "comparison_axis": "rigid_body_solver_family",
            "solver": "sequential_impulse_vs_ipc",
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "held_fixed": {
                "executor": executor_label,
                "friction": float(self.friction),
                "time_step_ms": _TIME_STEP * 1000.0,
                "top_mass_ratio": float(self.top_mass_ratio),
            },
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "top_mass_ratio": float(self.top_mass_ratio),
            },
            "case_pair": [case.label for case in self.cases],
            "solver_pair": [case.solver.name for case in self.cases],
            "sequential_impulse_solver_enum": self.cases[0].solver.name,
            "sequential_impulse_max_speed": case_metric(si_label, "max_speed"),
            "sequential_impulse_top_drift": case_metric(si_label, "top_drift"),
            "sequential_impulse_min_clearance": case_metric(
                si_label, "min_clearance"
            ),
            "sequential_impulse_height_error": case_metric(
                si_label, "height_error"
            ),
            "sequential_impulse_step_ms": case_metric(si_label, "step_ms"),
            "ipc_solver_enum": self.cases[1].solver.name,
            "ipc_max_speed": case_metric(ipc_label, "max_speed"),
            "ipc_top_drift": case_metric(ipc_label, "top_drift"),
            "ipc_min_clearance": case_metric(ipc_label, "min_clearance"),
            "ipc_height_error": case_metric(ipc_label, "height_error"),
            "ipc_step_ms": case_metric(ipc_label, "step_ms"),
            "top_x_divergence": (
                float(self._delta_history[-1]) if self._delta_history else 0.0
            ),
            "cases": cases,
            "history": {
                "samples": float(len(speed_history[si_label])),
                "max_top_x_divergence": max(self._delta_history, default=0.0),
                "sequential_impulse_max_speed": max(
                    speed_history[si_label], default=0.0
                ),
                "sequential_impulse_max_top_drift": max(
                    drift_history[si_label], default=0.0
                ),
                "sequential_impulse_min_clearance": min(
                    clearance_history[si_label], default=0.0
                ),
                "sequential_impulse_max_abs_height_error": max(
                    (
                        abs(value - ideal_top_height)
                        for value in height_history[si_label]
                    ),
                    default=0.0,
                ),
                "ipc_max_speed": max(speed_history[ipc_label], default=0.0),
                "ipc_max_top_drift": max(drift_history[ipc_label], default=0.0),
                "ipc_min_clearance": min(
                    clearance_history[ipc_label], default=0.0
                ),
                "ipc_max_abs_height_error": max(
                    (
                        abs(value - ideal_top_height)
                        for value in height_history[ipc_label]
                    ),
                    default=0.0,
                ),
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
                "top_mass_ratio": float(self.top_mass_ratio),
            },
            "secondary_state": np.asarray(secondary.state_vector, dtype=float).copy(),
            "secondary_time": float(secondary.time),
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "drift_history": {
                key: list(values) for key, values in self._drift_history.items()
            },
            "clearance_history": {
                key: list(values) for key, values in self._clearance_history.items()
            },
            "height_history": {
                key: list(values) for key, values in self._height_history.items()
            },
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "delta_history": list(self._delta_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        history = snapshot.get("delta_history", [])
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
                if case_metrics.get("status") in {"overlapping", "collapsed"}:
                    return 1.0
                try:
                    if float(case_metrics.get("min_clearance", 1.0)) <= 0.001:
                        return 1.0
                    if float(case_metrics.get("top_drift", 0.0)) >= 0.02:
                        return 1.0
                except (TypeError, ValueError):
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
        self.top_mass_ratio = float(
            controls.get("top_mass_ratio", self.top_mass_ratio)
        )
        for case in self.cases:
            self._apply_parameters(case)
        secondary = self.cases[1].world
        secondary.state_vector = state["secondary_state"]
        secondary.time = float(state["secondary_time"])
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(self._drift_history, state.get("drift_history", {}))
        self._restore_histories(
            self._clearance_history, state.get("clearance_history", {})
        )
        self._restore_histories(self._height_history, state.get("height_history", {}))
        self._restore_histories(
            self._step_ms_history, state.get("step_ms_history", {})
        )
        self._delta_history.clear()
        self._delta_history.extend(float(v) for v in state.get("delta_history", []))
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

    def _case_text(self, builder: Any, case: _StackCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"speed {float(metrics['max_speed']):.3f} m/s | "
            f"top drift {float(metrics['top_drift']):.3f} m | "
            f"clearance {float(metrics['min_clearance']):.4f} m"
        )
        builder.text(
            f"height error {float(metrics['height_error']):.4f} m | "
            f"step profile {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_ratio, top_mass_ratio = builder.slider(
            "Top mass ratio", float(self.top_mass_ratio), 1.0, 30.0
        )
        if changed_ratio:
            self.top_mass_ratio = float(top_mass_ratio)

        changed_friction, friction = builder.slider(
            "Friction", float(self.friction), 0.0, 1.0
        )
        if changed_friction:
            self.friction = float(friction)

        if changed_executor or changed_ratio or changed_friction:
            self._reset()

        if builder.button("Reset stack"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: rigid-body solver family under stack load")
        builder.text(
            f"held fixed: executor {self._executor_label()} | "
            f"top mass ratio {self.top_mass_ratio:.1f} | "
            f"friction {self.friction:.2f} | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver pair: Sequential impulse vs IPC barrier")
        builder.text("two-block mass-ratio stack")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        builder.text(f"bottom mass: 1.0 | top mass: {self.top_mass_ratio:.1f}")
        for case in self.cases:
            self._case_text(builder, case)
        if self._delta_history:
            builder.text(f"top x divergence: {self._delta_history[-1]:.4f} m")
        builder.plot_lines(
            "SI max speed", list(self._speed_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC max speed", list(self._speed_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI top drift", list(self._drift_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC top drift", list(self._drift_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI min clearance", list(self._clearance_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC min clearance", list(self._clearance_history[self.cases[1].label])
        )
        builder.plot_lines("Top x divergence", list(self._delta_history))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)
        builder.separator()
        self.cases[1].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    stability = _RigidStackStability()
    return SceneSetup(
        world=stability.render_world,
        pre_step=stability.pre_step,
        force_drag=stability.force_drag,
        renderable_provider=stability.renderable_provider,
        panels=[ScenePanel("Rigid Stack Stability", stability.build_panel)],
        info={
            "sx_world": stability.primary_world,
            "rigid_stack_stability_controller": stability,
            "rigid_stack_stability_worlds": [case.world for case in stability.cases],
            CAPTURE_METRICS_INFO_KEY: stability.capture_metrics,
            "replay_capture_state": stability.capture_replay_state,
            "replay_restore_state": stability.restore_replay_state,
            "replay_sync": stability._sync,
            "replay_timeline": {
                "signal_label": "Top x divergence",
                "signal": stability.replay_timeline_signal,
                "markers": stability.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_stack_stability",
    title="Rigid Stack Stability",
    category="World Rigid Body",
    summary=(
        "Sequential impulse and IPC solve a tiny top-heavy resting stack "
        "side by side."
    ),
    build=build,
)
