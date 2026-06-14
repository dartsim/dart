"""Rigid multibody integration-family verifier for DART 7 World."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_NUM_LINKS = 3
_LINK_LENGTH = 0.55
_LINK_MASS = 0.55
_TIME_STEP = 0.005
_HISTORY = 180
_INITIAL_BEND = 0.28
_BASE_HEIGHT = 1.05
_CASE_Y = {
    "semi_residual": -1.05,
    "variational_residual": 0.0,
    "variational_solved": 1.05,
}
_GRAVITY = 9.81


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _rotation_y(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = math.cos(angle)
    s = math.sin(angle)
    transform[:3, :3] = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    return transform


def _box(size: tuple[float, float, float]) -> dart.BoxShape:
    return dart.BoxShape(np.asarray(size, dtype=float))


def _joint_scalar(value: object) -> float:
    values = np.asarray(value, dtype=float).reshape(-1)
    return float(values[0]) if values.size else 0.0


def _last_float(values: Any) -> float | None:
    try:
        if values:
            return float(values[-1])
    except (IndexError, TypeError, ValueError):
        return None
    return None


@dataclass
class _SolverFamilyCase:
    key: str
    label: str
    integration_family: str
    dynamics: sx.ClosureDynamicsPolicy
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    robot: Any
    base: Any
    links: list[Any]
    closure: Any
    target_tip: np.ndarray


class _RigidMultibodySolverFamily:
    def __init__(self) -> None:
        self.executor_index = 0
        self.gravity_scale = 1.0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.cases = [
            self._make_case(
                key="semi_residual",
                label="Semi-implicit residual",
                integration_family="semi-implicit",
                dynamics=sx.ClosureDynamicsPolicy.RESIDUAL_ONLY,
                color=(0.84, 0.44, 0.20),
            ),
            self._make_case(
                key="variational_residual",
                label="Variational residual",
                integration_family="variational integrator",
                dynamics=sx.ClosureDynamicsPolicy.RESIDUAL_ONLY,
                color=(0.50, 0.42, 0.82),
            ),
            self._make_case(
                key="variational_solved",
                label="Variational solved",
                integration_family="variational integrator",
                dynamics=sx.ClosureDynamicsPolicy.SOLVE,
                color=(0.20, 0.58, 0.88),
            ),
        ]

        self._residual_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._tip_error_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._tip_height_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._joint_speed_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._solve_ratio_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str | bool]] = {}
        self.reset(clear_replay=True)

    @property
    def primary_world(self) -> Any:
        return self.cases[0].world

    @property
    def render_world(self) -> Any:
        return self.cases[0].bridge.render_world

    def _executor_label(self) -> str:
        return self._executors[self.executor_index][0]

    def _executor(self) -> Any:
        return self._executors[self.executor_index][1]

    def _make_case(
        self,
        *,
        key: str,
        label: str,
        integration_family: str,
        dynamics: sx.ClosureDynamicsPolicy,
        color: tuple[float, float, float],
    ) -> _SolverFamilyCase:
        offset_y = _CASE_Y[key]
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -_GRAVITY),
            multibody_options=sx.MultibodyOptions(
                integration_family=integration_family
            ),
        )
        world.step_profiling_enabled = True

        robot = world.add_multibody(f"{key}_chain")
        base = robot.add_link(f"{key}_base")
        links: list[Any] = []
        parent = base
        target_transform = np.eye(4)
        for index in range(_NUM_LINKS):
            offset = (
                np.array([0.0, offset_y, _BASE_HEIGHT], dtype=float)
                if index == 0
                else np.array([_LINK_LENGTH, 0.0, 0.0], dtype=float)
            )
            link = robot.add_link(
                f"{key}_link{index}",
                parent=parent,
                joint=sx.JointSpec(
                    name=f"{key}_hinge{index}",
                    type=sx.JointType.REVOLUTE,
                    axis=(0.0, 1.0, 0.0),
                    transform_from_parent=_translation(offset),
                ),
            )
            link.mass = _LINK_MASS
            ixx = 0.5 * _LINK_MASS * (0.045**2)
            itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
            link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
            links.append(link)
            parent = link
            target_transform = (
                target_transform
                @ _translation(offset)
                @ _rotation_y(self._initial_angle(index))
            )

        target_tip_transform = target_transform @ _translation((_LINK_LENGTH, 0.0, 0.0))
        closure = world.add_loop_closure(
            f"{key}_tip_closure",
            sx.LoopClosureSpec(
                frame_a=links[-1],
                frame_b=base,
                family=sx.LoopClosureFamily.POINT,
                offset_a=_translation((_LINK_LENGTH, 0.0, 0.0)),
                offset_b=target_tip_transform,
            ),
        )
        closure.dynamics = dynamics
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_multibody_solver_family")
        self._add_visuals(bridge, key, base, links, target_tip_transform[:3, 3], color)

        return _SolverFamilyCase(
            key=key,
            label=label,
            integration_family=integration_family,
            dynamics=dynamics,
            color=color,
            world=world,
            bridge=bridge,
            robot=robot,
            base=base,
            links=links,
            closure=closure,
            target_tip=np.asarray(target_tip_transform[:3, 3], dtype=float),
        )

    def _add_visuals(
        self,
        bridge: WorldRenderBridge,
        key: str,
        base: Any,
        links: list[Any],
        target_tip: np.ndarray,
        color: tuple[float, float, float],
    ) -> None:
        pivot = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_pivot_visual",
            _translation((0.0, _CASE_Y[key], _BASE_HEIGHT)),
        )
        pivot.set_shape(dart.SphereShape(0.055))
        pivot.create_visual_aspect().set_color([0.30, 0.32, 0.36])
        bridge.render_world.add_simple_frame(pivot)

        target = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_closure_target_visual",
            _translation(target_tip),
        )
        target.set_shape(dart.SphereShape(0.045))
        target.create_visual_aspect().set_color([0.96, 0.80, 0.18])
        bridge.render_world.add_simple_frame(target)

        guide = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_closure_target_guide",
            _translation((target_tip[0], target_tip[1], target_tip[2] - 0.18)),
        )
        guide.set_shape(dart.CylinderShape(0.012, 0.36))
        guide.create_visual_aspect().set_color([0.96, 0.80, 0.18])
        bridge.render_world.add_simple_frame(guide)

        base_marker = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_base_marker",
            _translation((0.0, _CASE_Y[key], _BASE_HEIGHT - 0.06)),
        )
        base_marker.set_shape(_box((0.16, 0.08, 0.04)))
        base_marker.create_visual_aspect().set_color([0.34, 0.36, 0.40])
        bridge.render_world.add_simple_frame(base_marker)

        for index, link in enumerate(links):
            shade = 0.08 * index
            bridge.add_link_visual(
                link,
                _box((_LINK_LENGTH, 0.07, 0.07)),
                (
                    min(1.0, color[0] + shade),
                    min(1.0, color[1] + shade),
                    min(1.0, color[2] + shade),
                ),
                name=f"{key}_link{index}_visual",
                local_transform=_translation((_LINK_LENGTH * 0.5, 0.0, 0.0)),
            )
        bridge.sync()

    def _initial_angle(self, index: int) -> float:
        return _INITIAL_BEND * (-1.0 if index % 2 else 1.0)

    def _apply_controls(self, case: _SolverFamilyCase) -> None:
        case.world.gravity = (0.0, 0.0, -_GRAVITY * max(0.0, float(self.gravity_scale)))
        for index, link in enumerate(case.links):
            joint = link.parent_joint
            joint.position = [self._initial_angle(index)]
            joint.velocity = [0.0]
            joint.force = [0.0]
        case.world.update_kinematics()

    def _reset_case(self, case: _SolverFamilyCase, *, clear_replay: bool) -> None:
        self._apply_controls(case)
        case.world.time = 0.0
        if clear_replay:
            try:
                case.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        case.world.update_kinematics()

    def reset(self, *, clear_replay: bool = False) -> None:
        for case in self.cases:
            self._reset_case(case, clear_replay=clear_replay)
        for history in (
            *self._residual_history.values(),
            *self._tip_error_history.values(),
            *self._tip_height_history.values(),
            *self._joint_speed_history.values(),
            *self._step_ms_history.values(),
            self._solve_ratio_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def _tip_point(self, case: _SolverFamilyCase) -> np.ndarray:
        transform = np.asarray(case.links[-1].transform, dtype=float) @ _translation(
            (_LINK_LENGTH, 0.0, 0.0)
        )
        return np.asarray(transform[:3, 3], dtype=float)

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample(self, case: _SolverFamilyCase) -> dict[str, float | str | bool]:
        residual = case.closure.compute_residual()
        tip = self._tip_point(case)
        joint_speeds = [
            abs(_joint_scalar(link.parent_joint.velocity)) for link in case.links
        ]
        return {
            "integration_family": case.integration_family,
            "dynamics": "solve"
            if case.dynamics == sx.ClosureDynamicsPolicy.SOLVE
            else "residual only",
            "dynamic_solve": case.dynamics == sx.ClosureDynamicsPolicy.SOLVE,
            "residual": float(residual.norm),
            "coordinates": float(len(np.asarray(residual.value, dtype=float))),
            "active": bool(residual.active),
            "enabled": bool(residual.enabled),
            "force_available": bool(residual.force_available),
            "tip_error": float(np.linalg.norm(tip - case.target_tip)),
            "tip_height": float(tip[2]),
            "joint_speed": max(joint_speeds) if joint_speeds else 0.0,
            "step_ms": self._step_profile_ms(case.world),
        }

    def _record_metrics(self) -> None:
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.key] = metrics
            self._residual_history[case.key].append(float(metrics["residual"]))
            self._tip_error_history[case.key].append(float(metrics["tip_error"]))
            self._tip_height_history[case.key].append(float(metrics["tip_height"]))
            self._joint_speed_history[case.key].append(float(metrics["joint_speed"]))
            self._step_ms_history[case.key].append(float(metrics["step_ms"]))

        residual_only = max(
            float(self._last_metrics["semi_residual"]["residual"]),
            float(self._last_metrics["variational_residual"]["residual"]),
        )
        solved = max(float(self._last_metrics["variational_solved"]["residual"]), 1e-12)
        self._solve_ratio_history.append(residual_only / solved)

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        executor_index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = executor_index

        def serialized_metrics(case_key: str) -> dict[str, float | str | bool]:
            serialized: dict[str, float | str | bool] = {}
            for key, value in self._last_metrics[case_key].items():
                if isinstance(value, (bool, np.bool_)):
                    serialized[key] = bool(value)
                elif isinstance(value, (int, float, np.floating)):
                    serialized[key] = float(value)
                else:
                    serialized[key] = str(value)
            return serialized

        def case_value(case_key: str, metric_key: str) -> float:
            return float(self._last_metrics[case_key][metric_key])

        def max_history(history: deque[float]) -> float:
            return max(history, default=0.0)

        cases = {
            case.key: {
                "label": case.label,
                "integration_family": case.integration_family,
                "dynamics": "solve"
                if case.dynamics == sx.ClosureDynamicsPolicy.SOLVE
                else "residual only",
                "dynamic_solve": case.dynamics == sx.ClosureDynamicsPolicy.SOLVE,
                "closure": case.closure.name,
                "target_tip": [float(value) for value in case.target_tip],
                "metrics": serialized_metrics(case.key),
            }
            for case in self.cases
        }
        residual_only = max(
            case_value("semi_residual", "residual"),
            case_value("variational_residual", "residual"),
        )
        solved_residual = max(case_value("variational_solved", "residual"), 1.0e-12)
        residual_solve_ratio = residual_only / solved_residual
        solver_family_lanes = [case.key for case in self.cases]
        max_step_ms = max(
            (max_history(history) for history in self._step_ms_history.values()),
            default=0.0,
        )
        payload: dict[str, Any] = {
            "row": "rigid_multibody_solver_family",
            "comparison_axis": "multibody_integration_solve_policy_family",
            "solver": "world_multibody_integration_family",
            "scope": "multibody_closure_solve_routing",
            "held_fixed": {
                "solver": "world_multibody_integration_family",
                "contacts": "off",
                "closure_family": "point",
                "joint_family": "three_revolute_links",
                "chain_links": _NUM_LINKS,
                "link_length": _LINK_LENGTH,
                "link_mass": _LINK_MASS,
                "initial_bend": _INITIAL_BEND,
                "gravity_scale": float(self.gravity_scale),
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "executor": self._executors[executor_index][0],
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "gravity_scale": float(self.gravity_scale),
            "controls": {
                "executor_index": float(executor_index),
                "gravity_scale": float(self.gravity_scale),
            },
            "case_order": solver_family_lanes,
            "solver_family_lanes": solver_family_lanes,
            "case_count": float(len(self.cases)),
            "cases": cases,
            "residual_only_residual": residual_only,
            "solved_residual": solved_residual,
            "residual_solve_ratio": residual_solve_ratio,
            "multibody_solver_residual_only_residual": residual_only,
            "multibody_solver_solved_residual": solved_residual,
            "multibody_solver_residual_solve_ratio": residual_solve_ratio,
            "multibody_solver_semi_residual": case_value(
                "semi_residual", "residual"
            ),
            "multibody_solver_variational_residual": case_value(
                "variational_residual", "residual"
            ),
            "multibody_solver_solved_tip_error": case_value(
                "variational_solved", "tip_error"
            ),
            "multibody_solver_max_step_ms": max_step_ms,
            "history": {
                "samples": float(len(self._solve_ratio_history)),
                "max_residual_solve_ratio": max_history(self._solve_ratio_history),
                "max_step_ms": max_step_ms,
                "cases": {
                    case.key: {
                        "samples": float(len(self._residual_history[case.key])),
                        "max_residual": max_history(
                            self._residual_history[case.key]
                        ),
                        "max_tip_error": max_history(
                            self._tip_error_history[case.key]
                        ),
                        "max_joint_speed": max_history(
                            self._joint_speed_history[case.key]
                        ),
                        "max_step_ms": max_history(self._step_ms_history[case.key]),
                    }
                    for case in self.cases
                },
            },
        }
        for case in self.cases:
            for key, value in self._last_metrics[case.key].items():
                if isinstance(value, (bool, np.bool_)):
                    continue
                if isinstance(value, (int, float, np.floating)):
                    payload[f"{case.key}_{key}"] = float(value)
        return payload

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

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "gravity_scale": float(self.gravity_scale),
            },
            "secondary_states": {
                case.key: np.asarray(case.world.state_vector, dtype=float).copy()
                for case in self.cases[1:]
            },
            "secondary_times": {
                case.key: float(case.world.time) for case in self.cases[1:]
            },
            "residual_history": {
                key: list(values) for key, values in self._residual_history.items()
            },
            "tip_error_history": {
                key: list(values) for key, values in self._tip_error_history.items()
            },
            "tip_height_history": {
                key: list(values) for key, values in self._tip_height_history.items()
            },
            "joint_speed_history": {
                key: list(values) for key, values in self._joint_speed_history.items()
            },
            "step_ms_history": {
                key: list(values) for key, values in self._step_ms_history.items()
            },
            "solve_ratio_history": list(self._solve_ratio_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        ratio = _last_float(snapshot.get("solve_ratio_history", []))
        if ratio is not None:
            return max(0.0, ratio)

        try:
            if "residual_solve_ratio" in snapshot:
                return max(0.0, float(snapshot.get("residual_solve_ratio", 0.0)))
        except (TypeError, ValueError):
            return 0.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                semi_metrics = metrics.get("semi_residual", {})
                variational_metrics = metrics.get("variational_residual", {})
                solved_metrics = metrics.get("variational_solved", {})
                if (
                    isinstance(semi_metrics, dict)
                    and isinstance(variational_metrics, dict)
                    and isinstance(solved_metrics, dict)
                ):
                    residual_only = max(
                        float(semi_metrics.get("residual", 0.0)),
                        float(variational_metrics.get("residual", 0.0)),
                    )
                    solved = max(float(solved_metrics.get("residual", 0.0)), 1.0e-12)
                    return max(0.0, residual_only / solved)
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 1.0e8:
            return 1.0

        residuals = snapshot.get("residual_history", {})
        if isinstance(residuals, dict):
            semi = _last_float(residuals.get("semi_residual", []))
            variational = _last_float(residuals.get("variational_residual", []))
            solved = _last_float(residuals.get("variational_solved", []))
            residual_only_values = [
                value for value in (semi, variational) if value is not None
            ]
            if residual_only_values and max(residual_only_values) >= 0.50:
                return 1.0
            if (
                residual_only_values
                and solved is not None
                and solved <= 1.0e-8
                and max(residual_only_values) >= 0.25
            ):
                return 1.0

        tip_errors = snapshot.get("tip_error_history", {})
        if isinstance(tip_errors, dict):
            semi = _last_float(tip_errors.get("semi_residual", []))
            variational = _last_float(tip_errors.get("variational_residual", []))
            solved = _last_float(tip_errors.get("variational_solved", []))
            residual_only_values = [
                value for value in (semi, variational) if value is not None
            ]
            if residual_only_values and max(residual_only_values) >= 0.50:
                return 1.0
            if (
                residual_only_values
                and solved is not None
                and solved <= 1.0e-8
                and max(residual_only_values) >= 0.25
            ):
                return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                semi_metrics = metrics.get("semi_residual", {})
                variational_metrics = metrics.get("variational_residual", {})
                solved_metrics = metrics.get("variational_solved", {})
                if (
                    isinstance(semi_metrics, dict)
                    and isinstance(variational_metrics, dict)
                    and isinstance(solved_metrics, dict)
                ):
                    residual_only_residual = max(
                        float(semi_metrics.get("residual", 0.0)),
                        float(variational_metrics.get("residual", 0.0)),
                    )
                    residual_only_tip_error = max(
                        float(semi_metrics.get("tip_error", 0.0)),
                        float(variational_metrics.get("tip_error", 0.0)),
                    )
                    solved_residual = float(solved_metrics.get("residual", 0.0))
                    solved_tip_error = float(solved_metrics.get("tip_error", 0.0))
                    if (
                        residual_only_residual >= 0.50
                        or residual_only_tip_error >= 0.50
                    ):
                        return 1.0
                    if (
                        solved_residual <= 1.0e-8
                        and residual_only_residual >= 0.25
                    ):
                        return 1.0
                    if (
                        solved_tip_error <= 1.0e-8
                        and residual_only_tip_error >= 0.25
                    ):
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
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        for case in self.cases:
            case.world.gravity = (
                0.0,
                0.0,
                -_GRAVITY * max(0.0, float(self.gravity_scale)),
            )

        states = state.get("secondary_states", {})
        times = state.get("secondary_times", {})
        for case in self.cases[1:]:
            if case.key in states:
                case.world.state_vector = states[case.key]
            if case.key in times:
                case.world.time = float(times[case.key])
            case.world.update_kinematics()
        self._restore_histories(self._residual_history, state.get("residual_history", {}))
        self._restore_histories(
            self._tip_error_history, state.get("tip_error_history", {})
        )
        self._restore_histories(
            self._tip_height_history, state.get("tip_height_history", {})
        )
        self._restore_histories(
            self._joint_speed_history, state.get("joint_speed_history", {})
        )
        self._restore_histories(self._step_ms_history, state.get("step_ms_history", {}))
        self._solve_ratio_history.clear()
        self._solve_ratio_history.extend(
            float(value) for value in state.get("solve_ratio_history", [])
        )
        self._last_metrics = {
            str(key): dict(value) for key, value in state.get("last_metrics", {}).items()
        }
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _case_text(self, builder: Any, case: _SolverFamilyCase) -> None:
        metrics = self._last_metrics.get(case.key) or self._sample(case)
        builder.text(f"{case.label}: {metrics['integration_family']}")
        builder.text(
            f"{metrics['dynamics']} | residual {float(metrics['residual']):.6f} | "
            f"tip error {float(metrics['tip_error']):.6f} m"
        )
        builder.text(
            f"tip height {float(metrics['tip_height']):.3f} m | "
            f"joint speed {float(metrics['joint_speed']):.3f} rad/s | "
            f"step profile {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        executor_labels = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_labels
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.8
        )
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)

        if changed_executor or changed_gravity:
            self.reset(clear_replay=True)

        if builder.button("Reset solver-family rows"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: multibody integration solve-policy family")
        builder.text(
            "held fixed: World multibody point closure | contacts off | "
            f"three revolute links | link length {_LINK_LENGTH:.2f} | "
            f"gravity scale {self.gravity_scale:.1f} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text(f"executor: {self._executor_label()}")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text("semi-implicit dynamic closure solve is intentionally not requested")
        for case in self.cases:
            self._case_text(builder, case)
        if self._solve_ratio_history:
            builder.text(f"residual solve ratio: {self._solve_ratio_history[-1]:.1e}")
        builder.plot_lines(
            "Semi-implicit residual",
            list(self._residual_history["semi_residual"]),
        )
        builder.plot_lines(
            "Variational residual",
            list(self._residual_history["variational_residual"]),
        )
        builder.plot_lines(
            "Variational solved residual",
            list(self._residual_history["variational_solved"]),
        )
        builder.plot_lines("Solve ratio", list(self._solve_ratio_history))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    verifier = _RigidMultibodySolverFamily()
    return SceneSetup(
        world=verifier.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.force_drag,
        renderable_provider=verifier.renderable_provider,
        panels=[
            ScenePanel(
                "Rigid Multibody Solver Family",
                verifier.build_panel,
            )
        ],
        info={
            "sx_world": verifier.primary_world,
            "rigid_multibody_solver_family_controller": verifier,
            "rigid_multibody_solver_family_worlds": [
                case.world for case in verifier.cases
            ],
            CAPTURE_METRICS_INFO_KEY: verifier.capture_metrics,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Residual solve ratio",
                "signal": verifier.replay_timeline_signal,
                "markers": verifier.replay_timeline_marker,
            },
            "replay_sync": verifier._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_multibody_solver_family",
    title="Rigid Multibody Solver Family",
    category="World Rigid Body",
    summary=(
        "Shows semi-implicit residual-only multibody closure diagnostics beside "
        "variational residual-only and solved closure rows."
    ),
    build=build,
)
