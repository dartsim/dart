"""Rigid-body World step diagnostics for profiling and memory visibility."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import (
    CAPTURE_METRICS_INFO_KEY,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
)

_TIME_STEP = 0.004
_HISTORY = 180
_GROUND_HALF = np.array([0.55, 0.34, 0.04])
_BOX_HALF = np.array([0.075, 0.075, 0.075])
_SPHERE_RADIUS = 0.065
_LANE_X = {
    "single": -1.25,
    "contact": 0.0,
    "stack": 1.25,
}
# Keep live diagnostics on the realtime SI rigid-body path; the dedicated
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


def _bytes_to_kib(value: float) -> float:
    return float(value) / 1024.0


@dataclass
class _StepDiagnosticsLane:
    key: str
    label: str
    description: str
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    bodies: list[Any]
    initial_positions: list[np.ndarray]


class _RigidStepDiagnostics:
    def __init__(self) -> None:
        self.solver_index = 0
        self.executor_index = 0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            parallel = sx.ParallelExecutor(2)
            try:
                parallel.inline_threshold = 0
            except Exception:  # noqa: BLE001
                pass
            self._executors.append(("Parallel (2 workers)", parallel))
        except Exception:  # noqa: BLE001
            pass

        self.lanes = [
            self._make_single_lane(),
            self._make_contact_lane(),
            self._make_stack_lane(),
        ]
        self._wall_ms_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._stage_ms_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._contact_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._scratch_peak_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._entity_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._last_metrics: dict[str, dict[str, float | str | bool]] = {}
        self._reset()

    @property
    def primary_world(self) -> Any:
        return self.lanes[0].world

    @property
    def render_world(self) -> Any:
        return self.lanes[0].bridge.render_world

    def _solver_label(self) -> str:
        return _SOLVERS[self.solver_index][0]

    def _solver(self) -> sx.RigidBodySolver:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][1]

    def _executor_label(self) -> str:
        return self._executors[self.executor_index][0]

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _new_world(self) -> Any:
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            rigid_body_solver=self._solver(),
        )
        try:
            world.step_profiling_enabled = True
        except Exception:  # noqa: BLE001
            pass
        return world

    def _add_ground(self, world: Any, key: str) -> Any:
        ground = world.add_rigid_body(
            f"{key}_diagnostics_ground",
            position=(_LANE_X[key], 0.0, -_GROUND_HALF[2]),
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
        ground.friction = 0.65
        ground.restitution = 0.0
        return ground

    def _make_single_lane(self) -> _StepDiagnosticsLane:
        key = "single"
        world = self._new_world()
        ground = self._add_ground(world, key)
        position = np.array([_LANE_X[key], 0.0, 0.72], dtype=float)
        body = world.add_rigid_body(f"{key}_diagnostics_sphere", position=tuple(position))
        body.mass = 1.0
        body.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))
        body.friction = 0.25
        body.restitution = 0.05
        body.linear_velocity = (0.16, 0.0, 0.0)
        body.angular_velocity = (0.0, 0.0, 0.8)
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_step_diagnostics_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.43),
            name=f"{key}_diagnostics_ground_visual",
        )
        bridge.add_rigid_body_visual(
            body,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.22, 0.56, 0.86),
            name=f"{key}_diagnostics_sphere_visual",
        )
        return _StepDiagnosticsLane(
            key=key,
            label="Single body",
            description="one free rigid body",
            world=world,
            bridge=bridge,
            ground=ground,
            bodies=[body],
            initial_positions=[position],
        )

    def _make_contact_lane(self) -> _StepDiagnosticsLane:
        key = "contact"
        world = self._new_world()
        ground = self._add_ground(world, key)
        position = np.array([_LANE_X[key], 0.0, _BOX_HALF[2] + 0.006], dtype=float)
        body = world.add_rigid_body(f"{key}_diagnostics_box", position=tuple(position))
        body.mass = 1.0
        body.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        body.friction = 0.70
        body.restitution = 0.0
        body.linear_velocity = (0.08, 0.0, 0.0)
        body.angular_velocity = (0.0, 0.0, 0.25)
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_step_diagnostics_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.43),
            name=f"{key}_diagnostics_ground_visual",
        )
        bridge.add_rigid_body_visual(
            body,
            dart.BoxShape(_full(_BOX_HALF)),
            (0.88, 0.48, 0.22),
            name=f"{key}_diagnostics_box_visual",
        )
        return _StepDiagnosticsLane(
            key=key,
            label="Contact pair",
            description="one active ground contact",
            world=world,
            bridge=bridge,
            ground=ground,
            bodies=[body],
            initial_positions=[position],
        )

    def _make_stack_lane(self) -> _StepDiagnosticsLane:
        key = "stack"
        world = self._new_world()
        ground = self._add_ground(world, key)
        bodies: list[Any] = []
        initial_positions: list[np.ndarray] = []
        colors = (
            (0.28, 0.70, 0.44),
            (0.72, 0.48, 0.84),
            (0.90, 0.62, 0.20),
            (0.42, 0.62, 0.90),
        )
        for index in range(4):
            position = np.array(
                [
                    _LANE_X[key] + 0.012 * ((index % 2) * 2.0 - 1.0),
                    0.010 * (index - 1.5),
                    _BOX_HALF[2] + 0.150 * index + 0.004,
                ],
                dtype=float,
            )
            body = world.add_rigid_body(
                f"{key}_diagnostics_box{index}",
                position=tuple(position),
            )
            body.mass = 1.0 + 0.20 * index
            body.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
            body.friction = 0.72
            body.restitution = 0.0
            body.linear_velocity = (0.04 * (1.0 - 0.15 * index), 0.0, 0.0)
            body.angular_velocity = (0.0, 0.0, 0.10 * (index + 1))
            bodies.append(body)
            initial_positions.append(position)
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_step_diagnostics_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.43),
            name=f"{key}_diagnostics_ground_visual",
        )
        for index, body in enumerate(bodies):
            bridge.add_rigid_body_visual(
                body,
                dart.BoxShape(_full(_BOX_HALF)),
                colors[index % len(colors)],
                name=f"{key}_diagnostics_box{index}_visual",
            )
        return _StepDiagnosticsLane(
            key=key,
            label="Small stack",
            description="four coupled rigid bodies",
            world=world,
            bridge=bridge,
            ground=ground,
            bodies=bodies,
            initial_positions=initial_positions,
        )

    def _apply_parameters(self, lane: _StepDiagnosticsLane) -> None:
        lane.world.rigid_body_solver = self._solver()
        try:
            lane.world.step_profiling_enabled = True
        except Exception:  # noqa: BLE001
            pass

    def _reset_lane(self, lane: _StepDiagnosticsLane) -> None:
        self._apply_parameters(lane)
        for body, position in zip(lane.bodies, lane.initial_positions, strict=True):
            body.transform = _transform_at(position)
            body.clear_force()
            body.clear_torque()
        if lane.key == "single":
            lane.bodies[0].linear_velocity = (0.16, 0.0, 0.0)
            lane.bodies[0].angular_velocity = (0.0, 0.0, 0.8)
        elif lane.key == "contact":
            lane.bodies[0].linear_velocity = (0.08, 0.0, 0.0)
            lane.bodies[0].angular_velocity = (0.0, 0.0, 0.25)
        else:
            for index, body in enumerate(lane.bodies):
                body.linear_velocity = (0.04 * (1.0 - 0.15 * index), 0.0, 0.0)
                body.angular_velocity = (0.0, 0.0, 0.10 * (index + 1))
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
            *self._wall_ms_history.values(),
            *self._stage_ms_history.values(),
            *self._contact_history.values(),
            *self._scratch_peak_history.values(),
            *self._entity_history.values(),
        ):
            history.clear()
        self._last_metrics.clear()
        self._sync()

    def _profile_metrics(self, world: Any) -> dict[str, float | str | bool]:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return {
                    "profile_empty": True,
                    "profile_status": "profiling unavailable",
                    "stage_count": 0.0,
                    "wall_ms": 0.0,
                    "stage_ms": 0.0,
                    "top_stage": "none",
                    "top_stage_domain": "none",
                    "top_stage_acceleration": "none",
                    "top_stage_accelerated_backend": False,
                    "top_stage_ms": 0.0,
                    "max_workers": 0.0,
                    "accelerated_stage_count": 0.0,
                    "accelerated_backend_active": False,
                }
            stages = list(profile.stages)
            top_stage = max(stages, key=lambda stage: float(stage.duration_ms), default=None)
            accelerated_stage_count = sum(
                1 for stage in stages if bool(stage.accelerated_backend_enabled)
            )
            return {
                "profile_empty": False,
                "profile_status": "profiled",
                "stage_count": float(len(stages)),
                "wall_ms": float(profile.wall_time_ms),
                "stage_ms": float(profile.total_stage_time_us) / 1000.0,
                "top_stage": "none" if top_stage is None else str(top_stage.name),
                "top_stage_domain": "none"
                if top_stage is None
                else str(top_stage.domain),
                "top_stage_acceleration": "none"
                if top_stage is None
                else str(top_stage.acceleration),
                "top_stage_accelerated_backend": False
                if top_stage is None
                else bool(top_stage.accelerated_backend_enabled),
                "top_stage_ms": 0.0
                if top_stage is None
                else float(top_stage.duration_ms),
                "max_workers": float(
                    max(
                        (int(stage.max_graph_worker_count) for stage in stages),
                        default=0,
                    )
                ),
                "accelerated_stage_count": float(accelerated_stage_count),
                "accelerated_backend_active": bool(accelerated_stage_count > 0),
            }
        except Exception:  # noqa: BLE001
            return {
                "profile_empty": True,
                "profile_status": "profiling unavailable",
                "stage_count": 0.0,
                "wall_ms": 0.0,
                "stage_ms": 0.0,
                "top_stage": "none",
                "top_stage_domain": "none",
                "top_stage_acceleration": "none",
                "top_stage_accelerated_backend": False,
                "top_stage_ms": 0.0,
                "max_workers": 0.0,
                "accelerated_stage_count": 0.0,
                "accelerated_backend_active": False,
            }

    def _sample(self, lane: _StepDiagnosticsLane) -> dict[str, float | str | bool]:
        contacts = len(lane.world.collide())
        speeds = [
            float(np.linalg.norm(np.asarray(body.linear_velocity, dtype=float).reshape(3)))
            for body in lane.bodies
        ]
        diagnostics = lane.world.memory_diagnostics
        ecs = diagnostics.ecs_diagnostics
        metrics = self._profile_metrics(lane.world)
        metrics.update(
            {
                "body_count": float(len(lane.bodies)),
                "contact_count": float(contacts),
                "component_count": float(ecs.component_count),
                "entity_count": float(ecs.entity_count),
                "frame_scratch_capacity_kib": _bytes_to_kib(
                    diagnostics.frame_scratch_capacity_bytes
                ),
                "frame_scratch_peak_kib": _bytes_to_kib(
                    diagnostics.frame_scratch_peak_used_bytes
                ),
                "frame_scratch_overflow_count": float(
                    diagnostics.frame_scratch_overflow_count
                ),
                "frame_scratch_reset_count": float(
                    diagnostics.frame_scratch_reset_count
                ),
                "max_speed": max(speeds) if speeds else 0.0,
                "status": "contacts active" if contacts > 0 else "free motion",
                "time": float(lane.world.time),
            }
        )
        return metrics

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._wall_ms_history[lane.key].append(float(metrics["wall_ms"]))
            self._stage_ms_history[lane.key].append(float(metrics["stage_ms"]))
            self._contact_history[lane.key].append(float(metrics["contact_count"]))
            self._scratch_peak_history[lane.key].append(
                float(metrics["frame_scratch_peak_kib"])
            )
            self._entity_history[lane.key].append(float(metrics["entity_count"]))

    def capture_metrics(self) -> dict[str, Any]:
        return {
            "comparison_axis": "workload_shape",
            "controls": {
                "executor_index": int(self.executor_index),
                "solver_index": int(self.solver_index),
            },
            "executor": self._executor_label(),
            "held_fixed": {
                "solver": self._solver_label(),
                "executor": self._executor_label(),
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "lanes": {
                lane.key: dict(self._last_metrics.get(lane.key) or self._sample(lane))
                for lane in self.lanes
            },
            "row": "rigid_step_diagnostics",
            "solver": self._solver_label(),
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
        }

    def _sync(self) -> None:
        for lane in self.lanes:
            lane.bridge.sync()

    def pre_step(self) -> None:
        executor = self._executor()
        for lane in self.lanes:
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
            },
            "world_states": {
                lane.key: np.asarray(lane.world.state_vector, dtype=float).copy()
                for lane in self.lanes
            },
            "world_times": {lane.key: float(lane.world.time) for lane in self.lanes},
            "wall_ms_history": {
                key: list(values) for key, values in self._wall_ms_history.items()
            },
            "stage_ms_history": {
                key: list(values) for key, values in self._stage_ms_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "scratch_peak_history": {
                key: list(values) for key, values in self._scratch_peak_history.items()
            },
            "entity_history": {
                key: list(values) for key, values in self._entity_history.items()
            },
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
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
        world_states = state.get("world_states", {})
        world_times = state.get("world_times", {})
        for lane in self.lanes:
            self._apply_parameters(lane)
            if lane.key in world_states:
                lane.world.state_vector = world_states[lane.key]
            if lane.key in world_times:
                lane.world.time = float(world_times[lane.key])
            lane.world.update_kinematics()
        self._restore_histories(self._wall_ms_history, state.get("wall_ms_history", {}))
        self._restore_histories(
            self._stage_ms_history, state.get("stage_ms_history", {})
        )
        self._restore_histories(self._contact_history, state.get("contact_history", {}))
        self._restore_histories(
            self._scratch_peak_history, state.get("scratch_peak_history", {})
        )
        self._restore_histories(self._entity_history, state.get("entity_history", {}))
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

    def _lane_text(self, builder: Any, lane: _StepDiagnosticsLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        builder.text(f"{lane.label}: {lane.description} | {metrics['status']}")
        builder.text(
            f"bodies {float(metrics['body_count']):.0f} | "
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"entities {float(metrics['entity_count']):.0f} | "
            f"components {float(metrics['component_count']):.0f}"
        )
        builder.text(
            f"{metrics['profile_status']}: wall {float(metrics['wall_ms']):.3f} ms | "
            f"stages {float(metrics['stage_count']):.0f} | "
            f"top {metrics['top_stage']} {float(metrics['top_stage_ms']):.3f} ms"
        )
        builder.text(
            f"top domain {metrics['top_stage_domain']} | "
            f"accel {metrics['top_stage_acceleration']} | "
            f"backend active {'yes' if metrics['accelerated_backend_active'] else 'no'} "
            f"({float(metrics['accelerated_stage_count']):.0f} stages)"
        )
        builder.text(
            f"scratch peak {float(metrics['frame_scratch_peak_kib']):.1f} KiB / "
            f"{float(metrics['frame_scratch_capacity_kib']):.1f} KiB | "
            f"overflows {float(metrics['frame_scratch_overflow_count']):.0f} | "
            f"max workers {float(metrics['max_workers']):.0f}"
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

        if changed_solver:
            self.solver_index = int(solver_index)
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_solver or changed_executor:
            self._reset()

        if builder.button("Reset diagnostics"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: workload shape")
        builder.text(
            f"held fixed: solver {self._solver_label()} | "
            f"executor {self._executor_label()} | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(
            f"solver: {self._solver_label()} | executor: {self._executor_label()} | "
            f"time step: {_TIME_STEP * 1000.0:.1f} ms"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Single wall ms", list(self._wall_ms_history["single"]))
        builder.plot_lines("Contact wall ms", list(self._wall_ms_history["contact"]))
        builder.plot_lines("Stack wall ms", list(self._wall_ms_history["stack"]))
        builder.plot_lines("Single stage ms", list(self._stage_ms_history["single"]))
        builder.plot_lines("Stack stage ms", list(self._stage_ms_history["stack"]))
        builder.plot_lines("Stack contacts", list(self._contact_history["stack"]))
        builder.plot_lines(
            "Stack scratch peak KiB", list(self._scratch_peak_history["stack"])
        )
        builder.separator()
        self.lanes[0].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    diagnostics = _RigidStepDiagnostics()
    return SceneSetup(
        world=diagnostics.render_world,
        pre_step=diagnostics.pre_step,
        force_drag=diagnostics.force_drag,
        renderable_provider=diagnostics.renderable_provider,
        panels=[ScenePanel("Rigid Step Diagnostics", diagnostics.build_panel)],
        info={
            "sx_world": diagnostics.primary_world,
            "rigid_step_diagnostics_controller": diagnostics,
            "rigid_step_diagnostics_worlds": [lane.world for lane in diagnostics.lanes],
            "replay_capture_state": diagnostics.capture_replay_state,
            "replay_restore_state": diagnostics.restore_replay_state,
            "replay_sync": diagnostics._sync,
            CAPTURE_METRICS_INFO_KEY: diagnostics.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_step_diagnostics",
    title="Rigid Step Diagnostics",
    category="World Rigid Body",
    summary=(
        "Shows World step profile stages, ECS counters, and frame-scratch memory "
        "as rigid scene complexity increases under one selected solver/executor."
    ),
    build=build,
)
