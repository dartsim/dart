"""Side-by-side rigid solver comparison for the DART 7 World facade."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = np.array([1.35, 0.55, 0.05])
_WALL_HALF = np.array([0.035, 0.55, 0.35])
_BOX_HALF = np.array([0.14, 0.14, 0.14])
_LEFT_OFFSET = -1.55
_RIGHT_OFFSET = 1.55
_TIME_STEP = 0.004
_HISTORY = 180


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _SolverCase:
    label: str
    solver: sx.RigidBodySolver
    offset_x: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    wall: Any
    box: Any
    initial_box_position: np.ndarray


class _RigidSolverComparison:
    def __init__(self) -> None:
        self.friction = 0.45
        self.restitution = 0.08
        self.launch_speed = 5.0
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
                (0.22, 0.52, 0.88),
                "si",
            ),
            self._make_case(
                "IPC barrier",
                sx.RigidBodySolver.IPC,
                _RIGHT_OFFSET,
                (0.92, 0.48, 0.22),
                "ipc",
            ),
        ]
        self._speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._clearance_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._delta_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._sync()

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
    ) -> _SolverCase:
        world = sx.World(time_step=_TIME_STEP, rigid_body_solver=solver)
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(
            f"{prefix}_ground", position=(offset_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        wall = world.add_rigid_body(
            f"{prefix}_thin_wall",
            position=(offset_x + 0.72, 0.0, _WALL_HALF[2]),
        )
        wall.is_static = True
        wall.set_collision_shape(sx.CollisionShape.box(_WALL_HALF))

        initial_box_position = np.array(
            [offset_x - 0.85, 0.0, _BOX_HALF[2] + 0.006],
            dtype=float,
        )
        box = world.add_rigid_body(
            f"{prefix}_box", position=tuple(initial_box_position)
        )
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{prefix}_solver_compare_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{prefix}_ground_visual",
        )
        bridge.add_rigid_body_visual(
            wall,
            dart.BoxShape(_full(_WALL_HALF)),
            (0.75, 0.70, 0.46),
            name=f"{prefix}_thin_wall_visual",
        )
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_full(_BOX_HALF)),
            color,
            name=f"{prefix}_box_visual",
        )

        case = _SolverCase(
            label=label,
            solver=solver,
            offset_x=offset_x,
            color=color,
            world=world,
            bridge=bridge,
            ground=ground,
            wall=wall,
            box=box,
            initial_box_position=initial_box_position,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _SolverCase) -> None:
        for body in (case.ground, case.wall, case.box):
            body.friction = float(self.friction)
            body.restitution = float(self.restitution)

    def _reset_case(self, case: _SolverCase) -> None:
        case.box.transform = _transform_at(case.initial_box_position)
        case.box.linear_velocity = (float(self.launch_speed), 0.0, 0.0)
        case.box.angular_velocity = (0.0, 0.0, 0.0)
        case.box.clear_force()
        case.box.clear_torque()
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
            *self._clearance_history.values(),
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

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample(self, case: _SolverCase) -> dict[str, float | str]:
        position = np.asarray(case.box.translation, dtype=float)
        velocity = np.asarray(case.box.linear_velocity, dtype=float)
        wall_position = np.asarray(case.wall.translation, dtype=float)
        wall_face = wall_position[0] - _WALL_HALF[0]
        box_front = position[0] + _BOX_HALF[0]
        clearance = float(wall_face - box_front)
        speed = float(np.linalg.norm(velocity))
        if box_front > wall_position[0] + _WALL_HALF[0] + _BOX_HALF[0]:
            status = "passed wall"
        elif clearance <= 0.0:
            status = "at wall"
        elif speed < 0.05:
            status = "settled"
        else:
            status = "approaching"
        return {
            "x": float(position[0]),
            "speed": speed,
            "clearance": clearance,
            "step_ms": self._step_profile_ms(case.world),
            "status": status,
        }

    def _record_metrics(self) -> None:
        positions = []
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._speed_history[case.label].append(float(metrics["speed"]))
            self._clearance_history[case.label].append(float(metrics["clearance"]))
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))
            positions.append(float(metrics["x"]) - case.offset_x)
        if len(positions) == 2:
            self._delta_history.append(abs(positions[0] - positions[1]))

    def _sync(self) -> None:
        for case in self.cases:
            case.bridge.sync()

    def pre_step(self) -> None:
        executor = self._executor()
        for case in self.cases:
            case.world.step(executor)
        self._record_metrics()
        self._sync()

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
        divergence_values = list(self._delta_history)
        return {
            "row": "rigid_solver_compare",
            "comparison_axis": "rigid_body_solver_family",
            "solver": "sequential_impulse_vs_ipc",
            "executor": self._executors[int(self.executor_index)][0],
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "case_pair": [case.label for case in self.cases],
            "solver_pair": [case.solver.name for case in self.cases],
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
            },
            "cases": {
                case.label: {
                    "rigid_body_solver": case.solver.name,
                    "metrics": dict(self._last_metrics[case.label]),
                }
                for case in self.cases
            },
            "divergence": {
                "current_x": (
                    float(divergence_values[-1]) if divergence_values else 0.0
                ),
                "max_x": max(divergence_values, default=0.0),
                "samples": float(len(divergence_values)),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        # The shared replay panel records the first world directly. Store the
        # second world, controls, and UI histories so side-by-side replay stays
        # coherent.
        secondary = self.cases[1].world
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
            },
            "secondary_state": np.asarray(secondary.state_vector, dtype=float).copy(),
            "secondary_time": float(secondary.time),
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "clearance_history": {
                key: list(values) for key, values in self._clearance_history.items()
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
        return float(history[-1])

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        histories = snapshot.get("clearance_history", {})
        if not isinstance(histories, dict):
            return 0.0
        for values in histories.values():
            try:
                if values and float(values[-1]) <= 0.05:
                    return 1.0
            except (TypeError, ValueError):
                continue
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
        self.restitution = float(controls.get("restitution", self.restitution))
        for case in self.cases:
            self._apply_parameters(case)
        secondary = self.cases[1].world
        secondary.state_vector = state["secondary_state"]
        secondary.time = float(state["secondary_time"])
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(
            self._clearance_history, state.get("clearance_history", {})
        )
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

    def _case_text(self, builder: Any, case: _SolverCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"speed {float(metrics['speed']):.3f} m/s | "
            f"wall clearance {float(metrics['clearance']):.4f} m"
        )
        builder.text(f"step profile {float(metrics['step_ms']):.3f} ms")

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed, launch_speed = builder.slider(
            "Launch speed", float(self.launch_speed), 0.0, 20.0
        )
        if changed:
            self.launch_speed = float(launch_speed)
            for case in self.cases:
                velocity = np.asarray(case.box.linear_velocity, dtype=float)
                case.box.linear_velocity = (
                    self.launch_speed,
                    float(velocity[1]),
                    float(velocity[2]),
                )

        changed, friction = builder.slider("Friction", float(self.friction), 0.0, 1.0)
        if changed:
            self.friction = float(friction)
            for case in self.cases:
                self._apply_parameters(case)

        changed, restitution = builder.slider(
            "Restitution", float(self.restitution), 0.0, 0.8
        )
        if changed:
            self.restitution = float(restitution)
            for case in self.cases:
                self._apply_parameters(case)

        if changed_executor:
            self._reset()

        if builder.button("Reset comparison"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: rigid-body solver family")
        builder.text("solver pair: Sequential impulse vs IPC barrier")
        builder.text(f"shared executor: {choices[int(self.executor_index)]}")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        if self._delta_history:
            builder.text(f"x divergence: {self._delta_history[-1]:.4f} m")
        builder.plot_lines(
            "SI speed", list(self._speed_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC speed", list(self._speed_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI wall clearance", list(self._clearance_history[self.cases[0].label])
        )
        builder.plot_lines(
            "IPC wall clearance", list(self._clearance_history[self.cases[1].label])
        )
        builder.plot_lines("Position divergence", list(self._delta_history))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)
        builder.separator()
        self.cases[1].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    comparison = _RigidSolverComparison()
    return SceneSetup(
        world=comparison.render_world,
        pre_step=comparison.pre_step,
        force_drag=comparison.force_drag,
        renderable_provider=comparison.renderable_provider,
        panels=[ScenePanel("Rigid Solver Compare", comparison.build_panel)],
        info={
            "sx_world": comparison.primary_world,
            "rigid_solver_compare": True,
            "rigid_solver_compare_controller": comparison,
            "rigid_solver_compare_worlds": [case.world for case in comparison.cases],
            CAPTURE_METRICS_INFO_KEY: comparison.capture_metrics,
            "replay_capture_state": comparison.capture_replay_state,
            "replay_restore_state": comparison.restore_replay_state,
            "replay_sync": comparison._sync,
            "replay_timeline": {
                "signal_label": "Position divergence",
                "signal": comparison.replay_timeline_signal,
                "markers": comparison.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_solver_compare",
    title="Rigid Solver Compare",
    category="World Rigid Body",
    summary=(
        "Sequential impulse and IPC solve the same sliding box/wall scene "
        "side by side."
    ),
    build=build,
)
