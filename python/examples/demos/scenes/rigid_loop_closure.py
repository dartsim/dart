"""Closed-chain loop-closure verifier for rigid multibody links."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_NUM_LINKS = 4
_LINK_LENGTH = 0.56
_LINK_MASS = 0.55
_TIME_STEP = 0.005
_HISTORY = 180
_INITIAL_BEND = 0.18
_FAMILY_BASE_Y = {
    "POINT": -1.15,
    "DISTANCE": 0.0,
    "RIGID": 1.15,
}
_POLICY_OFFSET_Y = {
    "residual": -0.22,
    "solved": 0.22,
}
_DISTANCE_ANCHOR_OFFSET = np.array([0.0, 0.0, 0.32])
_GRAVITY = 9.81


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _rotation_y(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = float(np.cos(angle))
    s = float(np.sin(angle))
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
class _LoopClosureCase:
    key: str
    label: str
    family_label: str
    policy_label: str
    family: sx.LoopClosureFamily
    dynamics: sx.ClosureDynamicsPolicy
    offset_y: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    robot: Any
    base: Any
    links: list[Any]
    closure: Any
    target_point: np.ndarray
    anchor_point: np.ndarray
    target_distance: float


class _RigidLoopClosureVerifier:
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

        self._families: list[tuple[str, sx.LoopClosureFamily, str]] = [
            ("POINT", sx.LoopClosureFamily.POINT, "endpoint point"),
            ("DISTANCE", sx.LoopClosureFamily.DISTANCE, "tether length"),
            ("RIGID", sx.LoopClosureFamily.RIGID, "full pose weld"),
        ]
        self.cases = [
            self._make_case(
                family_label=family_label,
                family=family,
                policy_label="residual",
                dynamics=sx.ClosureDynamicsPolicy.RESIDUAL_ONLY,
                color=(0.82, 0.44, 0.22),
            )
            for family_label, family, _description in self._families
        ] + [
            self._make_case(
                family_label=family_label,
                family=family,
                policy_label="solved",
                dynamics=sx.ClosureDynamicsPolicy.SOLVE,
                color=(0.20, 0.56, 0.90),
            )
            for family_label, family, _description in self._families
        ]
        self._residual_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._distance_error_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._orientation_error_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._tip_height_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._joint_speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._residual_ratio_history: dict[str, deque[float]] = {
            family_label: deque(maxlen=_HISTORY)
            for family_label, _family, _description in self._families
        }
        self._last_metrics: dict[str, dict[str, float | str | bool]] = {}
        self.reset(clear_replay=True)

    @property
    def primary_world(self) -> Any:
        return self.cases[0].world

    @property
    def render_world(self) -> Any:
        return self.cases[0].bridge.render_world

    def _make_case(
        self,
        *,
        family_label: str,
        family: sx.LoopClosureFamily,
        policy_label: str,
        dynamics: sx.ClosureDynamicsPolicy,
        color: tuple[float, float, float],
    ) -> _LoopClosureCase:
        key = f"{family_label.lower()}_{policy_label}"
        label = f"{family_label} {policy_label}"
        offset_y = _FAMILY_BASE_Y[family_label] + _POLICY_OFFSET_Y[policy_label]
        world = sx.World(time_step=_TIME_STEP)
        world.multibody_options = sx.MultibodyOptions(
            integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
        )
        world.step_profiling_enabled = True

        robot = world.add_multibody(f"{key}_closed_chain")
        base = robot.add_link(f"{key}_base")
        links: list[Any] = []
        parent = base
        target_transform = np.eye(4)
        for i in range(_NUM_LINKS):
            offset = np.array(
                [0.0, offset_y, 0.0] if i == 0 else [_LINK_LENGTH, 0.0, 0.0],
                dtype=float,
            )
            initial_position = _INITIAL_BEND * (-1.0 if i % 2 else 1.0)
            link = robot.add_link(
                f"{key}_link{i}",
                parent=parent,
                joint=sx.JointSpec(
                    name=f"{key}_hinge{i}",
                    type=sx.JointType.REVOLUTE,
                    axis=(0.0, 1.0, 0.0),
                    transform_from_parent=_translation(offset),
                ),
            )
            link.mass = _LINK_MASS
            ixx = 0.5 * _LINK_MASS * (0.045**2)
            itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
            link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
            link.parent_joint.position = [initial_position]
            links.append(link)
            parent = link
            target_transform = (
                target_transform @ _rotation_y(initial_position) @ _translation(offset)
            )

        target_tip_transform = target_transform @ _translation((_LINK_LENGTH, 0.0, 0.0))
        target_point = target_tip_transform[:3, 3]
        anchor_point = target_point.copy()
        target_distance = 0.0
        offset_b = target_tip_transform
        if family == sx.LoopClosureFamily.DISTANCE:
            anchor_point = target_point + _DISTANCE_ANCHOR_OFFSET
            target_distance = float(np.linalg.norm(target_point - anchor_point))
            offset_b = _translation(tuple(anchor_point))
        closure = world.add_loop_closure(
            f"{key}_endpoint_closure",
            sx.LoopClosureSpec(
                frame_a=links[-1],
                frame_b=base,
                family=family,
                offset_a=_translation((_LINK_LENGTH, 0.0, 0.0)),
                offset_b=offset_b,
                distance=target_distance,
            ),
        )
        closure.dynamics = dynamics
        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_rigid_loop_closure_render")
        self._add_visuals(bridge, key, base, links, anchor_point, color)

        return _LoopClosureCase(
            key=key,
            label=label,
            family_label=family_label,
            policy_label=policy_label,
            family=family,
            dynamics=dynamics,
            offset_y=offset_y,
            color=color,
            world=world,
            bridge=bridge,
            robot=robot,
            base=base,
            links=links,
            closure=closure,
            target_point=target_point,
            anchor_point=anchor_point,
            target_distance=target_distance,
        )

    def _add_visuals(
        self,
        bridge: WorldRenderBridge,
        key: str,
        base: Any,
        links: list[Any],
        marker_point: np.ndarray,
        color: tuple[float, float, float],
    ) -> None:
        bridge.add_link_visual(
            base,
            _box((0.14, 0.14, 0.14)),
            (0.34, 0.36, 0.40),
            name=f"{key}_base_visual",
        )
        link_shape = _box((_LINK_LENGTH, 0.07, 0.07))
        palette = [
            color,
            (0.28, 0.68, 0.42),
            (0.72, 0.40, 0.76),
            (0.88, 0.66, 0.22),
        ]
        for i, link in enumerate(links):
            bridge.add_link_visual(
                link,
                link_shape,
                palette[i % len(palette)],
                name=f"{key}_link{i}_visual",
                local_transform=_translation((_LINK_LENGTH * 0.5, 0.0, 0.0)),
            )
        target_frame = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_target_visual",
            _translation(tuple(marker_point)),
        )
        target_frame.set_shape(_box((0.11, 0.11, 0.11)))
        target_frame.create_visual_aspect().set_color([0.94, 0.88, 0.20])
        bridge.render_world.add_simple_frame(target_frame)
        bridge.sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _apply_controls(self, case: _LoopClosureCase) -> None:
        case.world.gravity = (0.0, 0.0, -_GRAVITY * float(self.gravity_scale))
        case.closure.dynamics = case.dynamics

    def _reset_case(self, case: _LoopClosureCase, *, clear_replay: bool) -> None:
        self._apply_controls(case)
        for i, link in enumerate(case.links):
            initial_position = _INITIAL_BEND * (-1.0 if i % 2 else 1.0)
            link.parent_joint.position = [initial_position]
            link.parent_joint.velocity = [0.0]
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
            *self._distance_error_history.values(),
            *self._orientation_error_history.values(),
            *self._tip_height_history.values(),
            *self._joint_speed_history.values(),
            *self._step_ms_history.values(),
            *self._residual_ratio_history.values(),
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _tip_point(self, case: _LoopClosureCase) -> np.ndarray:
        transform = np.asarray(case.links[-1].transform, dtype=float)
        return (transform @ np.array([_LINK_LENGTH, 0.0, 0.0, 1.0]))[:3]

    def _sample(self, case: _LoopClosureCase) -> dict[str, float | str | bool]:
        residual = case.closure.compute_residual()
        residual_value = np.asarray(residual.value, dtype=float).reshape(-1)
        residual_norm = float(residual.norm)
        tip_point = self._tip_point(case)
        tip_error = float(np.linalg.norm(tip_point - case.target_point))
        separation = float(np.linalg.norm(tip_point - case.anchor_point))
        distance_error = abs(separation - float(case.target_distance))
        orientation_error = (
            float(np.linalg.norm(residual_value[3:6]))
            if residual_value.size >= 6
            else 0.0
        )
        speeds = [
            abs(_joint_scalar(link.parent_joint.velocity)) for link in case.links
        ]
        status = "held" if residual_norm < 1.0e-5 else "drifting"
        return {
            "family": case.family_label,
            "policy": case.policy_label,
            "residual": residual_norm,
            "coordinates": float(residual_value.size),
            "tip_error": tip_error,
            "distance_error": distance_error,
            "target_distance": float(case.target_distance),
            "separation": separation,
            "orientation_error": orientation_error,
            "tip_height": float(tip_point[2]),
            "target_height": float(case.target_point[2]),
            "max_joint_speed": max(speeds) if speeds else 0.0,
            "step_ms": self._step_profile_ms(case.world),
            "active": bool(residual.active),
            "enabled": bool(residual.enabled),
            "force_available": bool(residual.force_available),
            "status": status,
        }

    def _record_metrics(self) -> None:
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._residual_history[case.label].append(float(metrics["residual"]))
            self._distance_error_history[case.label].append(
                float(metrics["distance_error"])
            )
            self._orientation_error_history[case.label].append(
                float(metrics["orientation_error"])
            )
            self._tip_height_history[case.label].append(float(metrics["tip_height"]))
            self._joint_speed_history[case.label].append(
                float(metrics["max_joint_speed"])
            )
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))
        for family_label, _family, _description in self._families:
            open_residual = float(
                self._last_metrics[f"{family_label} residual"]["residual"]
            )
            solved_residual = float(
                self._last_metrics[f"{family_label} solved"]["residual"]
            )
            self._residual_ratio_history[family_label].append(
                open_residual / max(solved_residual, 1e-12)
            )

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        executor_index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = executor_index

        def serialized_metrics(case_label: str) -> dict[str, float | str | bool]:
            serialized: dict[str, float | str | bool] = {}
            for key, value in self._last_metrics[case_label].items():
                if isinstance(value, (bool, np.bool_)):
                    serialized[key] = bool(value)
                elif isinstance(value, (int, float, np.floating)):
                    serialized[key] = float(value)
                else:
                    serialized[key] = str(value)
            return serialized

        def metric_value(case_label: str, metric_key: str) -> float:
            return float(self._last_metrics[case_label][metric_key])

        def max_history(history: deque[float]) -> float:
            return max(history, default=0.0)

        cases = {
            case.key: {
                "label": case.label,
                "family": case.family_label,
                "policy": case.policy_label,
                "dynamics": "solve"
                if case.dynamics == sx.ClosureDynamicsPolicy.SOLVE
                else "residual only",
                "dynamic_solve": case.dynamics == sx.ClosureDynamicsPolicy.SOLVE,
                "closure": case.closure.name,
                "target_point": [float(value) for value in case.target_point],
                "anchor_point": [float(value) for value in case.anchor_point],
                "target_distance": float(case.target_distance),
                "metrics": serialized_metrics(case.label),
            }
            for case in self.cases
        }
        families: dict[str, dict[str, float | str]] = {}
        for family_label, _family, description in self._families:
            residual_label = f"{family_label} residual"
            solved_label = f"{family_label} solved"
            residual = metric_value(residual_label, "residual")
            solved = max(metric_value(solved_label, "residual"), 1.0e-12)
            ratio = residual / solved
            families[family_label] = {
                "description": description,
                "residual_case": residual_label,
                "solved_case": solved_label,
                "residual": residual,
                "solved_residual": solved,
                "residual_ratio": ratio,
            }
        family_order = [
            family_label for family_label, _family, _description in self._families
        ]
        policy_order = ["residual", "solved"]
        max_step_ms = max(
            (max_history(history) for history in self._step_ms_history.values()),
            default=0.0,
        )

        payload: dict[str, Any] = {
            "row": "rigid_loop_closure",
            "comparison_axis": "loop_closure_family_policy_selection",
            "solver": "variational_rigid_multibody_loop_closure",
            "scope": "point_distance_rigid_closure_family_selection",
            "held_fixed": {
                "solver": "variational_rigid_multibody_loop_closure",
                "contacts": "off",
                "integration_family": "variational integrator",
                "joint_family": "four_revolute_links",
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
            "family_order": family_order,
            "policy_order": policy_order,
            "closure_family_lanes": family_order,
            "closure_policy_lanes": policy_order,
            "case_order": [case.key for case in self.cases],
            "case_count": float(len(self.cases)),
            "cases": cases,
            "families": families,
            "loop_closure_point_residual_ratio": float(
                families["POINT"]["residual_ratio"]
            ),
            "loop_closure_distance_residual_ratio": float(
                families["DISTANCE"]["residual_ratio"]
            ),
            "loop_closure_rigid_residual_ratio": float(
                families["RIGID"]["residual_ratio"]
            ),
            "loop_closure_distance_solved_distance_error": metric_value(
                "DISTANCE solved", "distance_error"
            ),
            "loop_closure_distance_solved_tip_error": metric_value(
                "DISTANCE solved", "tip_error"
            ),
            "loop_closure_rigid_residual_orientation_error": metric_value(
                "RIGID residual", "orientation_error"
            ),
            "loop_closure_rigid_solved_orientation_error": metric_value(
                "RIGID solved", "orientation_error"
            ),
            "loop_closure_max_step_ms": max_step_ms,
            "history": {
                "samples": float(
                    max(
                        (
                            len(history)
                            for history in self._residual_ratio_history.values()
                        ),
                        default=0,
                    )
                ),
                "max_step_ms": max_step_ms,
                "families": {
                    family_label: {
                        "max_residual_ratio": max_history(
                            self._residual_ratio_history[family_label]
                        )
                    }
                    for family_label, _family, _description in self._families
                },
                "cases": {
                    case.key: {
                        "samples": float(len(self._residual_history[case.label])),
                        "max_residual": max_history(
                            self._residual_history[case.label]
                        ),
                        "max_distance_error": max_history(
                            self._distance_error_history[case.label]
                        ),
                        "max_orientation_error": max_history(
                            self._orientation_error_history[case.label]
                        ),
                        "max_joint_speed": max_history(
                            self._joint_speed_history[case.label]
                        ),
                        "max_step_ms": max_history(self._step_ms_history[case.label]),
                    }
                    for case in self.cases
                },
            },
        }
        for family_label, family_metrics in families.items():
            prefix = family_label.lower()
            payload[f"{prefix}_residual"] = float(family_metrics["residual"])
            payload[f"{prefix}_solved_residual"] = float(
                family_metrics["solved_residual"]
            )
            payload[f"{prefix}_residual_ratio"] = float(
                family_metrics["residual_ratio"]
            )
        for case in self.cases:
            for key, value in self._last_metrics[case.label].items():
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
            self._apply_controls(case)
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
            "states": [
                np.asarray(case.world.state_vector, dtype=float).copy()
                for case in self.cases
            ],
            "times": [float(case.world.time) for case in self.cases],
            "residual_history": {
                key: list(values) for key, values in self._residual_history.items()
            },
            "distance_error_history": {
                key: list(values)
                for key, values in self._distance_error_history.items()
            },
            "orientation_error_history": {
                key: list(values)
                for key, values in self._orientation_error_history.items()
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
            "residual_ratio_history": {
                key: list(values)
                for key, values in self._residual_ratio_history.items()
            },
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        ratio_histories = snapshot.get("residual_ratio_history", {})
        if isinstance(ratio_histories, dict):
            ratios = [
                value
                for value in (
                    _last_float(ratio_histories.get(family_label, []))
                    for family_label, _family, _description in self._families
                )
                if value is not None
            ]
            if ratios:
                return max(0.0, max(ratios))

        top_level_ratios: list[float] = []
        for family_label, _family, _description in self._families:
            try:
                key = f"{family_label.lower()}_residual_ratio"
                if key in snapshot:
                    top_level_ratios.append(float(snapshot[key]))
            except (TypeError, ValueError):
                continue
        if top_level_ratios:
            return max(0.0, max(top_level_ratios))

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            ratios: list[float] = []
            for family_label, _family, _description in self._families:
                residual_metrics = metrics.get(f"{family_label} residual", {})
                solved_metrics = metrics.get(f"{family_label} solved", {})
                if isinstance(residual_metrics, dict) and isinstance(
                    solved_metrics, dict
                ):
                    try:
                        residual = float(residual_metrics.get("residual", 0.0))
                        solved = max(
                            float(solved_metrics.get("residual", 0.0)),
                            1.0e-12,
                        )
                        ratios.append(residual / solved)
                    except (TypeError, ValueError):
                        continue
            if ratios:
                return max(0.0, max(ratios))
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 1.0e8:
            return 1.0

        residuals = snapshot.get("residual_history", {})
        if isinstance(residuals, dict):
            for family_label, _family, _description in self._families:
                residual = _last_float(residuals.get(f"{family_label} residual", []))
                solved = _last_float(residuals.get(f"{family_label} solved", []))
                if (
                    residual is not None
                    and solved is not None
                    and residual >= 0.75
                    and solved <= 1.0e-8
                ):
                    return 1.0

        orientations = snapshot.get("orientation_error_history", {})
        if isinstance(orientations, dict):
            rigid_residual = _last_float(orientations.get("RIGID residual", []))
            rigid_solved = _last_float(orientations.get("RIGID solved", []))
            if rigid_residual is not None and rigid_residual >= 0.10:
                return 1.0
            if (
                rigid_residual is not None
                and rigid_solved is not None
                and rigid_residual >= 0.05
                and rigid_solved <= 1.0e-8
            ):
                return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                distance_solved = metrics.get("DISTANCE solved", {})
                if isinstance(distance_solved, dict):
                    distance_error = float(distance_solved.get("distance_error", 0.0))
                    tip_error = float(distance_solved.get("tip_error", 0.0))
                    if distance_error <= 1.0e-8 and tip_error >= 0.10:
                        return 1.0

                rigid_residual = metrics.get("RIGID residual", {})
                rigid_solved = metrics.get("RIGID solved", {})
                if isinstance(rigid_residual, dict) and isinstance(
                    rigid_solved, dict
                ):
                    residual_orientation = float(
                        rigid_residual.get("orientation_error", 0.0)
                    )
                    solved_orientation = float(
                        rigid_solved.get("orientation_error", 0.0)
                    )
                    if residual_orientation >= 0.10:
                        return 1.0
                    if (
                        residual_orientation >= 0.05
                        and solved_orientation <= 1.0e-8
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
        states = state.get("states", [])
        times = state.get("times", [])
        for i, case in enumerate(self.cases):
            self._apply_controls(case)
            if i < len(states):
                case.world.state_vector = states[i]
            if i < len(times):
                case.world.time = float(times[i])
            case.world.update_kinematics()
        self._restore_histories(self._residual_history, state.get("residual_history", {}))
        self._restore_histories(
            self._distance_error_history, state.get("distance_error_history", {})
        )
        self._restore_histories(
            self._orientation_error_history, state.get("orientation_error_history", {})
        )
        self._restore_histories(
            self._tip_height_history, state.get("tip_height_history", {})
        )
        self._restore_histories(
            self._joint_speed_history, state.get("joint_speed_history", {})
        )
        self._restore_histories(self._step_ms_history, state.get("step_ms_history", {}))
        self._restore_histories(
            self._residual_ratio_history, state.get("residual_ratio_history", {})
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self._sync()

    def _restore_histories(
        self,
        histories: dict[str, deque[float]],
        payload: dict[str, list[float]],
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _case_text(self, builder: Any, case: _LoopClosureCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        if case.family == sx.LoopClosureFamily.DISTANCE:
            detail = (
                f"distance error {float(metrics['distance_error']):.6f} m | "
                f"separation {float(metrics['separation']):.3f} / "
                f"{float(metrics['target_distance']):.3f} m"
            )
        elif case.family == sx.LoopClosureFamily.RIGID:
            detail = (
                f"tip error {float(metrics['tip_error']):.6f} m | "
                f"orientation {float(metrics['orientation_error']):.6f} rad"
            )
        else:
            detail = f"tip error {float(metrics['tip_error']):.6f} m"
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"residual {float(metrics['residual']):.6f} | {detail}"
        )
        builder.text(
            f"tip z {float(metrics['tip_height']):.3f} m | "
            f"max joint speed {float(metrics['max_joint_speed']):.3f} rad/s"
        )
        builder.text(
            f"active {bool(metrics['active'])} | "
            f"force sample {bool(metrics['force_available'])}"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.4
        )
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)

        if changed_executor or changed_gravity:
            self.reset(clear_replay=True)

        if builder.button("Reset closure verifier"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: loop-closure family and solve policy")
        builder.text(
            "held fixed: Variational rigid multibody | contacts off | "
            f"four revolute links | link length {_LINK_LENGTH:.2f} | "
            f"gravity scale {self.gravity_scale:.1f} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        executor_index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = executor_index
        builder.text(f"executor: {self._executors[executor_index][0]}")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text("families: POINT endpoint, DISTANCE tether, RIGID weld")
        builder.text("solver path: variational rigid multibody")
        for family_label, _family, description in self._families:
            builder.text(f"{family_label}: {description}")
            for case in self._cases_for_family(family_label):
                self._case_text(builder, case)
            ratio_history = self._residual_ratio_history[family_label]
            if ratio_history:
                builder.text(f"{family_label} residual ratio: {ratio_history[-1]:.1f}x")
                builder.plot_lines(
                    f"{family_label} residual ratio", list(ratio_history)
                )
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)

    def _cases_for_family(self, family_label: str) -> list[_LoopClosureCase]:
        return [case for case in self.cases if case.family_label == family_label]


def build() -> SceneSetup:
    verifier = _RigidLoopClosureVerifier()
    return SceneSetup(
        world=verifier.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.force_drag,
        renderable_provider=verifier.renderable_provider,
        panels=[ScenePanel("Rigid Loop Closure", verifier.build_panel)],
        info={
            "sx_world": verifier.primary_world,
            "rigid_loop_closure_controller": verifier,
            "rigid_loop_closure_worlds": [case.world for case in verifier.cases],
            CAPTURE_METRICS_INFO_KEY: verifier.capture_metrics,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Max closure residual ratio",
                "signal": verifier.replay_timeline_signal,
                "markers": verifier.replay_timeline_marker,
            },
            "replay_sync": verifier._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_loop_closure",
    title="Rigid Loop Closure",
    category="World Rigid Body",
    summary="Compares residual-only and solved endpoint loop closures on a closed chain.",
    build=build,
)
