"""Rigid contact-scale budget verifier for DART 7 World solvers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import (
    CAPTURE_METRICS_INFO_KEY,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
)

_TIME_STEP = 0.004
_HISTORY = 180
_BOX_HALF = np.array([0.055, 0.055, 0.055])
_GROUND_HALF = np.array([0.58, 0.42, 0.035])
_LANE_X = {
    "single": -1.25,
    "medium": 0.0,
    "dense": 1.25,
}
# Keep live budget inspection on the realtime SI rigid-body path; the dedicated
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
class _ScaleLane:
    key: str
    label: str
    target_contacts: int
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    boxes: list[Any]
    initial_positions: list[np.ndarray]


class _RigidContactScaleBudget:
    def __init__(self) -> None:
        self.solver_index = 0
        self.executor_index = 0
        self.budget_ms = 1.25
        self.friction = 0.72
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
            self._make_lane("single", "Single box", 1),
            self._make_lane("medium", "Four boxes", 4),
            self._make_lane("dense", "Nine boxes", 9),
        ]
        self._wall_ms_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._contact_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._scratch_peak_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._per_contact_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._budget_over_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._dense_single_ratio: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str | bool]] = {}
        self.reset(clear_replay=True)

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
        world.step_profiling_enabled = True
        return world

    def _grid_offsets(self, count: int) -> list[np.ndarray]:
        if count == 1:
            return [np.array([0.0, 0.0, 0.0], dtype=float)]
        side = int(np.ceil(np.sqrt(count)))
        spacing = 2.6 * float(_BOX_HALF[0])
        offsets: list[np.ndarray] = []
        for row in range(side):
            for col in range(side):
                if len(offsets) >= count:
                    return offsets
                offsets.append(
                    np.array(
                        [
                            (col - 0.5 * (side - 1)) * spacing,
                            (row - 0.5 * (side - 1)) * spacing,
                            0.0,
                        ],
                        dtype=float,
                    )
                )
        return offsets

    def _make_lane(self, key: str, label: str, target_contacts: int) -> _ScaleLane:
        world = self._new_world()
        lane_x = _LANE_X[key]
        ground = world.add_rigid_body(
            f"{key}_scale_ground", position=(lane_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        boxes: list[Any] = []
        initial_positions: list[np.ndarray] = []
        colors = (
            (0.24, 0.56, 0.86),
            (0.30, 0.70, 0.48),
            (0.86, 0.52, 0.24),
            (0.58, 0.48, 0.84),
        )
        for index, offset in enumerate(self._grid_offsets(target_contacts)):
            position = np.array(
                [lane_x + offset[0], offset[1], _BOX_HALF[2] - 0.002],
                dtype=float,
            )
            body = world.add_rigid_body(f"{key}_scale_box{index}", position=tuple(position))
            body.mass = 1.0
            body.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
            boxes.append(body)
            initial_positions.append(position)
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_contact_scale_budget_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{key}_scale_ground_visual",
        )
        for index, body in enumerate(boxes):
            bridge.add_rigid_body_visual(
                body,
                dart.BoxShape(_full(_BOX_HALF)),
                colors[index % len(colors)],
                name=f"{key}_scale_box{index}_visual",
            )
        return _ScaleLane(
            key=key,
            label=label,
            target_contacts=target_contacts,
            world=world,
            bridge=bridge,
            ground=ground,
            boxes=boxes,
            initial_positions=initial_positions,
        )

    def _clamp_controls(self) -> None:
        self.solver_index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.executor_index = max(
            0, min(int(self.executor_index), len(self._executors) - 1)
        )
        self.budget_ms = float(np.clip(self.budget_ms, 0.10, 8.0))
        self.friction = float(np.clip(self.friction, 0.0, 1.2))

    def _apply_parameters(self, lane: _ScaleLane) -> None:
        self._clamp_controls()
        lane.world.rigid_body_solver = self._solver()
        lane.world.step_profiling_enabled = True
        lane.ground.friction = float(self.friction)
        lane.ground.restitution = 0.0
        for body in lane.boxes:
            body.mass = 1.0
            body.friction = float(self.friction)
            body.restitution = 0.0

    def _pin_contact_workload(self, lane: _ScaleLane) -> None:
        for index, (body, position) in enumerate(
            zip(lane.boxes, lane.initial_positions, strict=True)
        ):
            body.transform = _transform_at(position)
            body.linear_velocity = (0.010 * (index + 1), 0.0, -0.005)
            body.angular_velocity = (0.0, 0.0, 0.02 * (index + 1))
            body.clear_force()
            body.clear_torque()
        lane.world.update_kinematics()

    def _reset_lane(self, lane: _ScaleLane) -> None:
        self._apply_parameters(lane)
        self._pin_contact_workload(lane)
        lane.world.time = 0.0
        try:
            lane.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass

    def reset(self, *, clear_replay: bool = False) -> None:
        for lane in self.lanes:
            self._reset_lane(lane)
        if clear_replay:
            try:
                self.primary_world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            *self._wall_ms_history.values(),
            *self._contact_history.values(),
            *self._scratch_peak_history.values(),
            *self._per_contact_history.values(),
            *self._budget_over_history.values(),
            self._dense_single_ratio,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def _profile_metrics(self, world: Any) -> dict[str, float | str | bool]:
        profile = world.last_step_profile
        if profile.is_empty():
            return {
                "profile_empty": True,
                "profile_status": "profiling unavailable",
                "stage_count": 0.0,
                "wall_ms": 0.0,
                "stage_ms": 0.0,
                "top_stage": "none",
                "top_stage_ms": 0.0,
                "max_workers": 0.0,
            }
        stages = list(profile.stages)
        top_stage = max(stages, key=lambda stage: float(stage.duration_ms), default=None)
        return {
            "profile_empty": False,
            "profile_status": "profiled",
            "stage_count": float(len(stages)),
            "wall_ms": float(profile.wall_time_ms),
            "stage_ms": float(profile.total_stage_time_us) / 1000.0,
            "top_stage": "none" if top_stage is None else str(top_stage.name),
            "top_stage_ms": 0.0 if top_stage is None else float(top_stage.duration_ms),
            "max_workers": float(
                max((int(stage.max_graph_worker_count) for stage in stages), default=0)
            ),
        }

    def _sample(self, lane: _ScaleLane) -> dict[str, float | str | bool]:
        contacts = len(lane.world.collide())
        diagnostics = lane.world.memory_diagnostics
        ecs = diagnostics.ecs_diagnostics
        metrics = self._profile_metrics(lane.world)
        wall_ms = float(metrics["wall_ms"])
        divisor = max(contacts, 1)
        wall_per_contact_us = wall_ms * 1000.0 / float(divisor)
        budget_over = bool(not metrics["profile_empty"] and wall_ms > self.budget_ms)
        if bool(metrics["profile_empty"]):
            status = "profiling unavailable"
        elif budget_over:
            status = "over budget"
        else:
            status = "within budget"
        metrics.update(
            {
                "budget_ms": float(self.budget_ms),
                "budget_over": budget_over,
                "body_count": float(len(lane.boxes)),
                "component_count": float(ecs.component_count),
                "contact_count": float(contacts),
                "contacts_per_body": float(contacts) / max(float(len(lane.boxes)), 1.0),
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
                "status": status,
                "target_contacts": float(lane.target_contacts),
                "wall_per_contact_us": wall_per_contact_us,
            }
        )
        return metrics

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._wall_ms_history[lane.key].append(float(metrics["wall_ms"]))
            self._contact_history[lane.key].append(float(metrics["contact_count"]))
            self._scratch_peak_history[lane.key].append(
                float(metrics["frame_scratch_peak_kib"])
            )
            self._per_contact_history[lane.key].append(
                float(metrics["wall_per_contact_us"])
            )
            self._budget_over_history[lane.key].append(
                1.0 if bool(metrics["budget_over"]) else 0.0
            )
        single_wall = float(self._last_metrics["single"]["wall_ms"])
        dense_wall = float(self._last_metrics["dense"]["wall_ms"])
        if single_wall > 0.0:
            self._dense_single_ratio.append(dense_wall / single_wall)
        else:
            self._dense_single_ratio.append(0.0)

    def capture_metrics(self) -> dict[str, Any]:
        return {
            "budget_ms": float(self.budget_ms),
            "comparison_axis": "contact_workload_size",
            "controls": {
                "budget_ms": float(self.budget_ms),
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "solver_index": int(self.solver_index),
            },
            "dense_single_ratio": (
                float(self._dense_single_ratio[-1])
                if self._dense_single_ratio
                else 0.0
            ),
            "executor": self._executor_label(),
            "friction": float(self.friction),
            "held_fixed": {
                "solver": self._solver_label(),
                "executor": self._executor_label(),
                "friction": float(self.friction),
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "lanes": {
                lane.key: dict(self._last_metrics.get(lane.key) or self._sample(lane))
                for lane in self.lanes
            },
            "row": "rigid_contact_scale_budget",
            "solver": self._solver_label(),
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
        }

    def pre_step(self) -> None:
        executor = self._executor()
        for lane in self.lanes:
            self._apply_parameters(lane)
            self._pin_contact_workload(lane)
            lane.world.step(executor)
            self._pin_contact_workload(lane)
        self._record_metrics()
        self._sync()

    def _sync(self) -> None:
        for lane in self.lanes:
            lane.bridge.sync()

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
                "budget_ms": float(self.budget_ms),
                "friction": float(self.friction),
            },
            "world_states": {
                lane.key: np.asarray(lane.world.state_vector, dtype=float).copy()
                for lane in self.lanes
            },
            "world_times": {lane.key: float(lane.world.time) for lane in self.lanes},
            "wall_ms_history": {
                key: list(values) for key, values in self._wall_ms_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "scratch_peak_history": {
                key: list(values) for key, values in self._scratch_peak_history.items()
            },
            "per_contact_history": {
                key: list(values) for key, values in self._per_contact_history.items()
            },
            "budget_over_history": {
                key: list(values) for key, values in self._budget_over_history.items()
            },
            "dense_single_ratio": list(self._dense_single_ratio),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def restore_replay_state(self, snapshot: dict[str, Any]) -> None:
        controls = snapshot.get("controls", {})
        self.solver_index = int(controls.get("solver_index", self.solver_index))
        self.executor_index = int(controls.get("executor_index", self.executor_index))
        self.budget_ms = float(controls.get("budget_ms", self.budget_ms))
        self.friction = float(controls.get("friction", self.friction))
        self._clamp_controls()
        world_states = snapshot.get("world_states", {})
        world_times = snapshot.get("world_times", {})
        for lane in self.lanes:
            self._apply_parameters(lane)
            if lane.key in world_states:
                lane.world.state_vector = world_states[lane.key]
            if lane.key in world_times:
                lane.world.time = float(world_times[lane.key])
            lane.world.update_kinematics()
        self._restore_histories(self._wall_ms_history, snapshot.get("wall_ms_history", {}))
        self._restore_histories(self._contact_history, snapshot.get("contact_history", {}))
        self._restore_histories(
            self._scratch_peak_history, snapshot.get("scratch_peak_history", {})
        )
        self._restore_histories(
            self._per_contact_history, snapshot.get("per_contact_history", {})
        )
        self._restore_histories(
            self._budget_over_history, snapshot.get("budget_over_history", {})
        )
        self._dense_single_ratio.clear()
        self._dense_single_ratio.extend(
            float(value) for value in snapshot.get("dense_single_ratio", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in snapshot.get("last_metrics", {}).items()
        }
        self._record_metrics()
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _ScaleLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        builder.text(f"{lane.label}: {metrics['status']}")
        builder.text(
            f"bodies {float(metrics['body_count']):.0f} | "
            f"contact points {float(metrics['contact_count']):.0f} | "
            f"contacts/body {float(metrics['contacts_per_body']):.2f}"
        )
        builder.text(
            f"wall {float(metrics['wall_ms']):.3f} ms | "
            f"per contact {float(metrics['wall_per_contact_us']):.1f} us | "
            f"stages {float(metrics['stage_count']):.0f} | "
            f"top {metrics['top_stage']} {float(metrics['top_stage_ms']):.3f} ms"
        )
        builder.text(
            f"scratch peak {float(metrics['frame_scratch_peak_kib']):.1f} KiB | "
            f"entities {float(metrics['entity_count']):.0f} | "
            f"components {float(metrics['component_count']):.0f} | "
            f"workers {float(metrics['max_workers']):.0f}"
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
        changed_budget, budget_ms = builder.slider(
            "Frame budget ms", float(self.budget_ms), 0.10, 8.0
        )
        changed_friction, friction = builder.slider(
            "Contact friction", float(self.friction), 0.0, 1.2
        )

        if changed_solver:
            self.solver_index = int(solver_index)
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_budget:
            self.budget_ms = float(budget_ms)
        if changed_friction:
            self.friction = float(friction)
        if changed_solver or changed_executor or changed_friction:
            self.reset(clear_replay=True)
        else:
            self._clamp_controls()

        if builder.button("Reset budget run"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: contact workload size")
        builder.text(
            f"held fixed: solver {self._solver_label()} | "
            f"executor {self._executor_label()} | friction {self.friction:.2f}"
        )
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(
            f"solver: {self._solver_label()} | executor: {self._executor_label()} | "
            f"time step: {_TIME_STEP * 1000.0:.1f} ms | "
            f"budget {self.budget_ms:.2f} ms"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Single wall ms", list(self._wall_ms_history["single"]))
        builder.plot_lines("Four-contact wall ms", list(self._wall_ms_history["medium"]))
        builder.plot_lines("Nine-contact wall ms", list(self._wall_ms_history["dense"]))
        builder.plot_lines(
            "Nine-contact per-contact us", list(self._per_contact_history["dense"])
        )
        builder.plot_lines("Nine-contact count", list(self._contact_history["dense"]))
        builder.plot_lines(
            "Nine-contact scratch KiB", list(self._scratch_peak_history["dense"])
        )
        builder.plot_lines("Dense/single wall ratio", list(self._dense_single_ratio))
        builder.separator()
        self.lanes[0].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidContactScaleBudget()
    return SceneSetup(
        world=controller.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Contact Scale Budget", controller.build_panel)],
        info={
            "sx_world": controller.primary_world,
            "rigid_contact_scale_budget_controller": controller,
            "rigid_contact_scale_budget_worlds": [
                lane.world for lane in controller.lanes
            ],
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_sync": controller._sync,
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_contact_scale_budget",
    title="Rigid Contact Scale Budget",
    category="World Rigid Body",
    summary=(
        "Shows how rigid contact count affects frame budget, step profiling, "
        "and frame-scratch memory under one selected solver/executor."
    ),
    build=build,
)
