"""Side-by-side rigid executor equivalence verifier for DART 7 World."""

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
_BOX_HALF = np.array([0.11, 0.11, 0.11])
_LEFT_OFFSET = -1.45
_RIGHT_OFFSET = 1.45
_TIME_STEP = 0.004
_HISTORY = 180

_RELATIVE_BOX_POSITIONS = (
    np.array([-0.48, -0.12, _BOX_HALF[2] + 0.010]),
    np.array([-0.16, 0.10, _BOX_HALF[2] + 0.265]),
    np.array([0.16, -0.08, _BOX_HALF[2] + 0.520]),
    np.array([0.46, 0.12, _BOX_HALF[2] + 0.775]),
)
_BOX_COLORS = (
    (0.22, 0.54, 0.84),
    (0.88, 0.44, 0.18),
    (0.28, 0.72, 0.48),
    (0.72, 0.50, 0.86),
)
# Keep the executor-equivalence row on one realtime physics path; the dedicated
# rigid_solver_compare scene owns SI-vs-IPC visual inspection.
_SOLVERS: tuple[tuple[str, sx.RigidBodySolver], ...] = (
    ("Sequential impulse", sx.RigidBodySolver.SEQUENTIAL_IMPULSE),
)


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _ExecutorCase:
    label: str
    executor: Any
    offset_x: float
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    boxes: list[Any]
    initial_positions: list[np.ndarray]


class _RigidExecutorEquivalence:
    def __init__(self) -> None:
        self.solver_index = 0
        self.launch_speed = 0.45
        self.friction = 0.55
        self.restitution = 0.05
        self._parallel_label = "Parallel (2 workers)"
        self._parallel_executor: Any
        try:
            self._parallel_executor = sx.ParallelExecutor(2)
        except Exception:  # noqa: BLE001
            self._parallel_label = "Sequential fallback"
            self._parallel_executor = sx.SequentialExecutor()

        self.cases = [
            self._make_case(
                "Sequential executor",
                sx.SequentialExecutor(),
                _LEFT_OFFSET,
                "seq",
            ),
            self._make_case(
                self._parallel_label,
                self._parallel_executor,
                _RIGHT_OFFSET,
                "par",
            ),
        ]
        self._step_ms_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._position_divergence: deque[float] = deque(maxlen=_HISTORY)
        self._velocity_divergence: deque[float] = deque(maxlen=_HISTORY)
        self._contact_delta: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._sync()

    @property
    def primary_world(self) -> Any:
        return self.cases[0].world

    @property
    def render_world(self) -> Any:
        return self.cases[0].bridge.render_world

    def _solver_label(self) -> str:
        return _SOLVERS[self.solver_index][0]

    def _solver(self) -> sx.RigidBodySolver:
        return _SOLVERS[self.solver_index][1]

    def _make_case(
        self,
        label: str,
        executor: Any,
        offset_x: float,
        prefix: str,
    ) -> _ExecutorCase:
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            rigid_body_solver=self._solver(),
        )
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(
            f"{prefix}_executor_ground", position=(offset_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        boxes: list[Any] = []
        initial_positions: list[np.ndarray] = []
        for index, relative_position in enumerate(_RELATIVE_BOX_POSITIONS):
            position = np.array(
                [offset_x + relative_position[0], relative_position[1], relative_position[2]],
                dtype=float,
            )
            body = world.add_rigid_body(
                f"{prefix}_executor_box{index}", position=tuple(position)
            )
            body.mass = 1.0 + 0.25 * index
            body.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
            boxes.append(body)
            initial_positions.append(position)

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{prefix}_executor_equivalence_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{prefix}_executor_ground_visual",
        )
        for index, body in enumerate(boxes):
            bridge.add_rigid_body_visual(
                body,
                dart.BoxShape(_full(_BOX_HALF)),
                _BOX_COLORS[index % len(_BOX_COLORS)],
                name=f"{prefix}_executor_box{index}_visual",
            )

        case = _ExecutorCase(
            label=label,
            executor=executor,
            offset_x=offset_x,
            world=world,
            bridge=bridge,
            ground=ground,
            boxes=boxes,
            initial_positions=initial_positions,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _ExecutorCase) -> None:
        case.world.rigid_body_solver = self._solver()
        for body in (case.ground, *case.boxes):
            body.friction = float(self.friction)
            body.restitution = float(self.restitution)

    def _reset_case(self, case: _ExecutorCase) -> None:
        for index, (box, position) in enumerate(
            zip(case.boxes, case.initial_positions, strict=True)
        ):
            box.transform = _transform_at(position)
            box.linear_velocity = (
                float(self.launch_speed) * (1.0 - 0.18 * index),
                0.04 * ((index % 2) * 2.0 - 1.0),
                0.0,
            )
            box.angular_velocity = (0.0, 0.0, 0.25 * (index + 1))
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
            *self._step_ms_history.values(),
            *self._speed_history.values(),
            self._position_divergence,
            self._velocity_divergence,
            self._contact_delta,
        ):
            history.clear()
        self._last_metrics.clear()
        self._sync()

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _case_state(self, case: _ExecutorCase) -> tuple[list[np.ndarray], list[np.ndarray]]:
        positions = [
            np.asarray(box.translation, dtype=float).reshape(3)
            - np.array([case.offset_x, 0.0, 0.0])
            for box in case.boxes
        ]
        velocities = [
            np.asarray(box.linear_velocity, dtype=float).reshape(3)
            for box in case.boxes
        ]
        return positions, velocities

    def _sample(self, case: _ExecutorCase) -> dict[str, float | str]:
        _positions, velocities = self._case_state(case)
        speeds = [float(np.linalg.norm(velocity)) for velocity in velocities]
        contact_count = float(len(case.world.collide()))
        return {
            "max_speed": max(speeds) if speeds else 0.0,
            "contact_count": contact_count,
            "step_ms": self._step_profile_ms(case.world),
            "status": (
                "contacts active"
                if contact_count > 0.0
                else "falling"
                if case.world.time < 0.25
                else "separating"
            ),
        }

    def _record_metrics(self) -> None:
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._speed_history[case.label].append(float(metrics["max_speed"]))
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))

        left_positions, left_velocities = self._case_state(self.cases[0])
        right_positions, right_velocities = self._case_state(self.cases[1])
        position_divergence = max(
            float(np.linalg.norm(left - right))
            for left, right in zip(left_positions, right_positions, strict=True)
        )
        velocity_divergence = max(
            float(np.linalg.norm(left - right))
            for left, right in zip(left_velocities, right_velocities, strict=True)
        )
        contact_delta = abs(
            float(self._last_metrics[self.cases[0].label]["contact_count"])
            - float(self._last_metrics[self.cases[1].label]["contact_count"])
        )
        self._position_divergence.append(position_divergence)
        self._velocity_divergence.append(velocity_divergence)
        self._contact_delta.append(contact_delta)

    def _sync(self) -> None:
        for case in self.cases:
            case.bridge.sync()

    def pre_step(self) -> None:
        for case in self.cases:
            case.world.step(case.executor)
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
        seq_label = self.cases[0].label
        parallel_label = self.cases[1].label
        seq_metrics = dict(self._last_metrics.get(seq_label, self._sample(self.cases[0])))
        parallel_metrics = dict(
            self._last_metrics.get(parallel_label, self._sample(self.cases[1]))
        )
        position_values = list(self._position_divergence)
        velocity_values = list(self._velocity_divergence)
        contact_delta_values = list(self._contact_delta)
        sequential_step_values = list(self._step_ms_history[seq_label])
        parallel_step_values = list(self._step_ms_history[parallel_label])
        return {
            "row": "rigid_executor_equivalence",
            "comparison_axis": "executor",
            "same_solver": True,
            "solver": "same_solver_executor_equivalence",
            "solver_label": self._solver_label(),
            "solver_enum": self._solver().name,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "executor_pair": [seq_label, parallel_label],
            "controls": {
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
                "solver_index": int(self.solver_index),
            },
            "sequential_contact_count": float(seq_metrics["contact_count"]),
            "parallel_contact_count": float(parallel_metrics["contact_count"]),
            "sequential_step_ms": float(seq_metrics["step_ms"]),
            "parallel_step_ms": float(parallel_metrics["step_ms"]),
            "position_divergence": (
                float(position_values[-1]) if position_values else 0.0
            ),
            "velocity_divergence": (
                float(velocity_values[-1]) if velocity_values else 0.0
            ),
            "contact_delta": (
                float(contact_delta_values[-1]) if contact_delta_values else 0.0
            ),
            "cases": {
                seq_label: {
                    "executor": "sequential",
                    "metrics": seq_metrics,
                },
                parallel_label: {
                    "executor": "parallel"
                    if parallel_label.startswith("Parallel")
                    else "sequential_fallback",
                    "metrics": parallel_metrics,
                },
            },
            "history": {
                "samples": float(len(position_values)),
                "max_position_divergence": max(position_values, default=0.0),
                "max_velocity_divergence": max(velocity_values, default=0.0),
                "max_contact_delta": max(contact_delta_values, default=0.0),
                "max_sequential_step_ms": max(sequential_step_values, default=0.0),
                "max_parallel_step_ms": max(parallel_step_values, default=0.0),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        secondary = self.cases[1].world
        return {
            "controls": {
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
                "solver_index": int(self.solver_index),
            },
            "secondary_state": np.asarray(secondary.state_vector, dtype=float).copy(),
            "secondary_time": float(secondary.time),
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "position_divergence": list(self._position_divergence),
            "velocity_divergence": list(self._velocity_divergence),
            "contact_delta": list(self._contact_delta),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        history = snapshot.get("position_divergence", [])
        if not history:
            return 0.0
        try:
            return float(history[-1])
        except (TypeError, ValueError):
            return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        for key in ("position_divergence", "velocity_divergence"):
            history = snapshot.get(key, [])
            try:
                if history and abs(float(history[-1])) > 1.0e-8:
                    return 1.0
            except (TypeError, ValueError):
                continue
        contact_delta = snapshot.get("contact_delta", [])
        try:
            if contact_delta and abs(float(contact_delta[-1])) > 0.0:
                return 1.0
        except (TypeError, ValueError):
            return 0.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.solver_index = max(
            0,
            min(
                int(controls.get("solver_index", self.solver_index)),
                max(0, len(_SOLVERS) - 1),
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
        self._restore_histories(self._step_ms_history, state.get("step_ms_history", {}))
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._position_divergence.clear()
        self._position_divergence.extend(
            float(value) for value in state.get("position_divergence", [])
        )
        self._velocity_divergence.clear()
        self._velocity_divergence.extend(
            float(value) for value in state.get("velocity_divergence", [])
        )
        self._contact_delta.clear()
        self._contact_delta.extend(float(value) for value in state.get("contact_delta", []))
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

    def _case_text(self, builder: Any, case: _ExecutorCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"max speed {float(metrics['max_speed']):.3f} m/s | "
            f"step profile {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        solver_choices = [label for label, _solver in _SOLVERS]
        changed_solver, solver_index = builder.select(
            "Physics solver", int(self.solver_index), solver_choices
        )
        if changed_solver:
            self.solver_index = int(solver_index)

        changed_speed, launch_speed = builder.slider(
            "Launch speed", float(self.launch_speed), 0.0, 1.2
        )
        if changed_speed:
            self.launch_speed = float(launch_speed)

        changed_friction, friction = builder.slider(
            "Friction", float(self.friction), 0.0, 1.0
        )
        if changed_friction:
            self.friction = float(friction)

        changed_restitution, restitution = builder.slider(
            "Restitution", float(self.restitution), 0.0, 0.8
        )
        if changed_restitution:
            self.restitution = float(restitution)

        if changed_solver or changed_speed or changed_friction or changed_restitution:
            self._reset()

        if builder.button("Reset equivalence"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: executor only")
        builder.text(f"same physics solver: {self._solver_label()}")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        if self._position_divergence:
            builder.text(
                f"pose divergence: {self._position_divergence[-1]:.6f} m | "
                f"velocity divergence: {self._velocity_divergence[-1]:.6f} m/s"
            )
            builder.text(f"contact-count delta: {self._contact_delta[-1]:.0f}")
        builder.plot_lines(
            "Sequential step ms", list(self._step_ms_history[self.cases[0].label])
        )
        builder.plot_lines(
            "Parallel step ms", list(self._step_ms_history[self.cases[1].label])
        )
        builder.plot_lines("Pose divergence", list(self._position_divergence))
        builder.plot_lines("Velocity divergence", list(self._velocity_divergence))
        builder.plot_lines("Contact-count delta", list(self._contact_delta))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)
        builder.separator()
        self.cases[1].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    equivalence = _RigidExecutorEquivalence()
    return SceneSetup(
        world=equivalence.render_world,
        pre_step=equivalence.pre_step,
        force_drag=equivalence.force_drag,
        renderable_provider=equivalence.renderable_provider,
        panels=[ScenePanel("Rigid Executor Equivalence", equivalence.build_panel)],
        info={
            "sx_world": equivalence.primary_world,
            "rigid_executor_equivalence_controller": equivalence,
            "rigid_executor_equivalence_worlds": [
                case.world for case in equivalence.cases
            ],
            "replay_capture_state": equivalence.capture_replay_state,
            "replay_restore_state": equivalence.restore_replay_state,
            "replay_sync": equivalence._sync,
            "replay_timeline": {
                "signal_label": "Pose divergence",
                "signal": equivalence.replay_timeline_signal,
                "markers": equivalence.replay_timeline_marker,
            },
            CAPTURE_METRICS_INFO_KEY: equivalence.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_executor_equivalence",
    title="Rigid Executor Equivalence",
    category="World Rigid Body",
    summary=(
        "Sequential and parallel executors step the same rigid-body tray side "
        "by side with one shared physics solver."
    ),
    build=build,
)
