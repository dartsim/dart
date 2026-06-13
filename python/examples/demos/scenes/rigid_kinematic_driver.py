"""Prescribed-motion rigid-body driver verifier for DART 7 World solvers."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_PLATFORM_HALF = np.array([0.78, 0.24, 0.05])
_BOX_HALF = np.array([0.12, 0.12, 0.12])
_TIME_STEP = 0.004
_HISTORY = 180
_INITIAL_GAP = 0.003
_LANE_Y = {
    "ipc_grip": 0.72,
    "ipc_slip": 0.0,
    "si_caveat": -0.72,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _DriverCase:
    key: str
    label: str
    solver: sx.RigidBodySolver
    friction_mode: str
    offset_y: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    platform: Any
    box: Any
    initial_platform_position: np.ndarray
    initial_box_position: np.ndarray


class _RigidKinematicDriver:
    def __init__(self) -> None:
        self.drive_speed = 0.35
        self.grip_friction = 0.80
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
                "ipc_grip",
                "IPC kinematic grip",
                sx.RigidBodySolver.IPC,
                "controlled",
                (0.22, 0.54, 0.86),
            ),
            self._make_case(
                "ipc_slip",
                "IPC low-friction slip",
                sx.RigidBodySolver.IPC,
                "zero",
                (0.56, 0.58, 0.62),
            ),
            self._make_case(
                "si_caveat",
                "Sequential impulse caveat",
                sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
                "controlled",
                (0.90, 0.44, 0.22),
            ),
        ]
        self._driver_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._box_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._slip_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._ratio_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._gap_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._contact_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._step_ms_history: dict[str, deque[float]] = {
            case.key: deque(maxlen=_HISTORY) for case in self.cases
        }
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
        key: str,
        label: str,
        solver: sx.RigidBodySolver,
        friction_mode: str,
        color: tuple[float, float, float],
    ) -> _DriverCase:
        offset_y = _LANE_Y[key]
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            rigid_body_solver=solver,
        )
        world.step_profiling_enabled = True

        initial_platform_position = np.array([0.0, offset_y, 0.0], dtype=float)
        platform = world.add_rigid_body(
            f"{key}_kinematic_platform", position=tuple(initial_platform_position)
        )
        platform.set_collision_shape(sx.CollisionShape.box(_PLATFORM_HALF))
        platform.is_kinematic = True

        initial_box_position = np.array(
            [
                0.0,
                offset_y,
                _PLATFORM_HALF[2] + _BOX_HALF[2] + _INITIAL_GAP,
            ],
            dtype=float,
        )
        box = world.add_rigid_body(f"{key}_carried_box", position=tuple(initial_box_position))
        box.mass = 1.0
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        box.restitution = 0.0

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{key}_kinematic_driver_render")
        bridge.add_rigid_body_visual(
            platform,
            dart.BoxShape(_full(_PLATFORM_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{key}_platform_visual",
        )
        bridge.add_rigid_body_visual(
            box,
            dart.BoxShape(_full(_BOX_HALF)),
            color,
            name=f"{key}_box_visual",
        )

        start = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_driver_start_marker",
            _transform_at(
                np.array(
                    [
                        initial_platform_position[0],
                        offset_y,
                        _PLATFORM_HALF[2] + 0.012,
                    ],
                    dtype=float,
                )
            ),
        )
        start.set_shape(dart.BoxShape(np.array([0.010, 0.56, 0.014])))
        start.create_visual_aspect().set_color([0.88, 0.88, 0.88])
        bridge.render_world.add_simple_frame(start)

        target = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            f"{key}_driver_target_marker",
            _transform_at(
                np.array(
                    [
                        initial_platform_position[0] + 0.16,
                        offset_y,
                        _PLATFORM_HALF[2] + 0.014,
                    ],
                    dtype=float,
                )
            ),
        )
        target.set_shape(dart.BoxShape(np.array([0.014, 0.56, 0.018])))
        target.create_visual_aspect().set_color([0.96, 0.78, 0.20])
        bridge.render_world.add_simple_frame(target)

        case = _DriverCase(
            key=key,
            label=label,
            solver=solver,
            friction_mode=friction_mode,
            offset_y=offset_y,
            color=color,
            world=world,
            bridge=bridge,
            platform=platform,
            box=box,
            initial_platform_position=initial_platform_position,
            initial_box_position=initial_box_position,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _case_friction(self, case: _DriverCase) -> float:
        if case.friction_mode == "zero":
            return 0.0
        return max(0.0, min(1.0, float(self.grip_friction)))

    def _apply_parameters(self, case: _DriverCase) -> None:
        friction = self._case_friction(case)
        case.platform.friction = friction
        case.platform.restitution = 0.0
        case.box.friction = friction
        case.box.restitution = 0.0
        case.box.mass = 1.0

    def _reset_case(self, case: _DriverCase) -> None:
        case.platform.transform = _transform_at(case.initial_platform_position)
        case.platform.linear_velocity = (float(self.drive_speed), 0.0, 0.0)
        case.platform.angular_velocity = (0.0, 0.0, 0.0)
        case.platform.clear_force()
        case.platform.clear_torque()

        case.box.transform = _transform_at(case.initial_box_position)
        case.box.linear_velocity = (0.0, 0.0, 0.0)
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
            *self._driver_history.values(),
            *self._box_history.values(),
            *self._slip_history.values(),
            *self._ratio_history.values(),
            *self._gap_history.values(),
            *self._contact_history.values(),
            *self._step_ms_history.values(),
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

    def _support_contact_count(self, case: _DriverCase) -> int:
        pair = frozenset((case.platform.name, case.box.name))
        return sum(
            1
            for contact in case.world.collide()
            if frozenset((contact.body_a.name, contact.body_b.name)) == pair
        )

    def _sample(self, case: _DriverCase) -> dict[str, float | str]:
        platform_position = np.asarray(case.platform.translation, dtype=float).reshape(3)
        box_position = np.asarray(case.box.translation, dtype=float).reshape(3)
        box_velocity = np.asarray(case.box.linear_velocity, dtype=float).reshape(3)
        driver_travel = float(platform_position[0] - case.initial_platform_position[0])
        box_travel = float(box_position[0] - case.initial_box_position[0])
        slip = float(driver_travel - box_travel)
        speed = float(box_velocity[0])
        denominator = max(1.0e-9, abs(float(self.drive_speed)))
        speed_ratio = float(speed / denominator)
        support_gap = float(
            box_position[2]
            - platform_position[2]
            - (_PLATFORM_HALF[2] + _BOX_HALF[2])
        )
        contact_count = float(self._support_contact_count(case))
        if case.key == "si_caveat" and driver_travel < 1.0e-4:
            status = "static-like caveat"
        elif case.key == "ipc_slip" and slip > 0.06:
            status = "slipping"
        elif speed_ratio > 0.60 and abs(slip) < 0.08:
            status = "carried"
        else:
            status = "partial drag"
        return {
            "box_speed": speed,
            "box_travel": box_travel,
            "contact_count": contact_count,
            "driver_travel": driver_travel,
            "friction": self._case_friction(case),
            "slip": slip,
            "speed_ratio": speed_ratio,
            "status": status,
            "step_ms": self._step_profile_ms(case.world),
            "support_gap": support_gap,
        }

    def _record_metrics(self) -> None:
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.key] = metrics
            self._driver_history[case.key].append(float(metrics["driver_travel"]))
            self._box_history[case.key].append(float(metrics["box_travel"]))
            self._slip_history[case.key].append(float(metrics["slip"]))
            self._ratio_history[case.key].append(float(metrics["speed_ratio"]))
            self._gap_history[case.key].append(float(metrics["support_gap"]))
            self._contact_history[case.key].append(float(metrics["contact_count"]))
            self._step_ms_history[case.key].append(float(metrics["step_ms"]))

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
        executor_label = self._executor_label()
        lanes = {
            case.key: {
                "label": case.label,
                "rigid_body_solver": case.solver.name,
                "friction_mode": case.friction_mode,
                "metrics": dict(self._last_metrics[case.key]),
            }
            for case in self.cases
        }
        driver_history = {
            case.key: list(self._driver_history[case.key]) for case in self.cases
        }
        box_history = {
            case.key: list(self._box_history[case.key]) for case in self.cases
        }
        slip_history = {
            case.key: list(self._slip_history[case.key]) for case in self.cases
        }
        ratio_history = {
            case.key: list(self._ratio_history[case.key]) for case in self.cases
        }
        gap_history = {
            case.key: list(self._gap_history[case.key]) for case in self.cases
        }
        contact_history = {
            case.key: list(self._contact_history[case.key]) for case in self.cases
        }
        step_ms_history = {
            case.key: list(self._step_ms_history[case.key])
            for case in self.cases
        }

        def lane_metric(key: str, metric_key: str) -> float:
            return float(self._last_metrics[key][metric_key])

        def max_abs(values: list[float]) -> float:
            return max((abs(value) for value in values), default=0.0)

        def min_abs(values: list[float]) -> float:
            return min((abs(value) for value in values), default=0.0)

        return {
            "row": "rigid_kinematic_driver",
            "comparison_axis": "prescribed_tangential_contact_response",
            "solver": "ipc_kinematic_driver_with_si_caveat",
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "held_fixed": {
                "box_mass": 1.0,
                "executor": executor_label,
                "gravity_z": -9.81,
                "kinematic_driver": "tangential support",
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "drive_speed": float(self.drive_speed),
                "executor_index": int(self.executor_index),
                "grip_friction": float(self.grip_friction),
            },
            "case_pair": [case.label for case in self.cases],
            "solver_pair": [case.solver.name for case in self.cases],
            "lane_order": [case.key for case in self.cases],
            "ipc_grip_solver_enum": self.cases[0].solver.name,
            "ipc_grip_driver_travel": lane_metric("ipc_grip", "driver_travel"),
            "ipc_grip_box_travel": lane_metric("ipc_grip", "box_travel"),
            "ipc_grip_slip": lane_metric("ipc_grip", "slip"),
            "ipc_grip_speed_ratio": lane_metric("ipc_grip", "speed_ratio"),
            "ipc_grip_support_gap": lane_metric("ipc_grip", "support_gap"),
            "ipc_grip_contact_count": lane_metric("ipc_grip", "contact_count"),
            "ipc_grip_min_abs_support_gap": min_abs(gap_history["ipc_grip"]),
            "ipc_grip_step_ms": lane_metric("ipc_grip", "step_ms"),
            "ipc_slip_solver_enum": self.cases[1].solver.name,
            "ipc_slip_driver_travel": lane_metric("ipc_slip", "driver_travel"),
            "ipc_slip_box_travel": lane_metric("ipc_slip", "box_travel"),
            "ipc_slip_slip": lane_metric("ipc_slip", "slip"),
            "ipc_slip_speed_ratio": lane_metric("ipc_slip", "speed_ratio"),
            "ipc_slip_support_gap": lane_metric("ipc_slip", "support_gap"),
            "ipc_slip_contact_count": lane_metric("ipc_slip", "contact_count"),
            "ipc_slip_min_abs_support_gap": min_abs(gap_history["ipc_slip"]),
            "ipc_slip_step_ms": lane_metric("ipc_slip", "step_ms"),
            "si_caveat_solver_enum": self.cases[2].solver.name,
            "si_caveat_driver_travel": lane_metric("si_caveat", "driver_travel"),
            "si_caveat_box_travel": lane_metric("si_caveat", "box_travel"),
            "si_caveat_slip": lane_metric("si_caveat", "slip"),
            "si_caveat_speed_ratio": lane_metric("si_caveat", "speed_ratio"),
            "si_caveat_support_gap": lane_metric("si_caveat", "support_gap"),
            "si_caveat_contact_count": lane_metric("si_caveat", "contact_count"),
            "si_caveat_step_ms": lane_metric("si_caveat", "step_ms"),
            "lanes": lanes,
            "history": {
                "samples": float(len(driver_history["ipc_grip"])),
                "ipc_grip_max_driver_travel": max(
                    driver_history["ipc_grip"], default=0.0
                ),
                "ipc_grip_max_box_travel": max(box_history["ipc_grip"], default=0.0),
                "ipc_grip_max_abs_slip": max_abs(slip_history["ipc_grip"]),
                "ipc_grip_max_speed_ratio": max(
                    ratio_history["ipc_grip"], default=0.0
                ),
                "ipc_grip_min_abs_support_gap": min_abs(gap_history["ipc_grip"]),
                "ipc_grip_max_contact_count": max(
                    contact_history["ipc_grip"], default=0.0
                ),
                "ipc_slip_max_driver_travel": max(
                    driver_history["ipc_slip"], default=0.0
                ),
                "ipc_slip_max_abs_box_travel": max_abs(box_history["ipc_slip"]),
                "ipc_slip_max_slip": max(slip_history["ipc_slip"], default=0.0),
                "ipc_slip_max_abs_speed_ratio": max_abs(ratio_history["ipc_slip"]),
                "ipc_slip_min_abs_support_gap": min_abs(gap_history["ipc_slip"]),
                "ipc_slip_max_contact_count": max(
                    contact_history["ipc_slip"], default=0.0
                ),
                "si_caveat_max_abs_driver_travel": max_abs(
                    driver_history["si_caveat"]
                ),
                "si_caveat_max_abs_box_travel": max_abs(box_history["si_caveat"]),
                "si_caveat_max_abs_slip": max_abs(slip_history["si_caveat"]),
                "si_caveat_max_abs_speed_ratio": max_abs(
                    ratio_history["si_caveat"]
                ),
                "si_caveat_min_abs_support_gap": min_abs(gap_history["si_caveat"]),
                "si_caveat_max_contact_count": max(
                    contact_history["si_caveat"], default=0.0
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
                "drive_speed": float(self.drive_speed),
                "executor_index": int(self.executor_index),
                "grip_friction": float(self.grip_friction),
            },
            "secondary_states": [
                np.asarray(case.world.state_vector, dtype=float).copy()
                for case in self.cases[1:]
            ],
            "secondary_times": [float(case.world.time) for case in self.cases[1:]],
            "driver_history": {
                key: list(values) for key, values in self._driver_history.items()
            },
            "box_history": {
                key: list(values) for key, values in self._box_history.items()
            },
            "slip_history": {
                key: list(values) for key, values in self._slip_history.items()
            },
            "ratio_history": {
                key: list(values) for key, values in self._ratio_history.items()
            },
            "gap_history": {
                key: list(values) for key, values in self._gap_history.items()
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
        histories = snapshot.get("box_history", {})
        if not isinstance(histories, dict):
            return 0.0
        history = histories.get("ipc_grip", [])
        if not history:
            return 0.0
        try:
            return float(history[-1])
        except (TypeError, ValueError):
            return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        grip_driver_travel = 0.0
        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            grip = metrics.get("ipc_grip", {})
            if isinstance(grip, dict):
                try:
                    grip_driver_travel = float(grip.get("driver_travel", 0.0))
                    if float(grip.get("contact_count", 0.0)) >= 1.0:
                        return 1.0
                    if float(grip.get("box_travel", 0.0)) >= 0.04:
                        return 1.0
                    if grip.get("status") == "carried":
                        return 1.0
                except (TypeError, ValueError):
                    pass
            slip = metrics.get("ipc_slip", {})
            if isinstance(slip, dict):
                try:
                    if float(slip.get("slip", 0.0)) >= 0.08:
                        return 1.0
                except (TypeError, ValueError):
                    pass
            caveat = metrics.get("si_caveat", {})
            if isinstance(caveat, dict) and caveat.get("status") == "static-like caveat":
                if grip_driver_travel >= 0.04:
                    return 1.0
        driver_histories = snapshot.get("driver_history", {})
        if isinstance(driver_histories, dict):
            try:
                values = driver_histories.get("ipc_grip", [])
                if values:
                    grip_driver_travel = max(grip_driver_travel, float(values[-1]))
            except (TypeError, ValueError, IndexError):
                pass
            if grip_driver_travel >= 0.04:
                caveat = metrics.get("si_caveat", {}) if isinstance(metrics, dict) else {}
                if (
                    isinstance(caveat, dict)
                    and caveat.get("status") == "static-like caveat"
                ):
                    return 1.0
        contact_histories = snapshot.get("contact_history", {})
        if isinstance(contact_histories, dict):
            try:
                values = contact_histories.get("ipc_grip", [])
                if values and float(values[-1]) >= 1.0:
                    return 1.0
            except (TypeError, ValueError, IndexError):
                pass
        slip_histories = snapshot.get("slip_history", {})
        if isinstance(slip_histories, dict):
            try:
                values = slip_histories.get("ipc_slip", [])
                if values and float(values[-1]) >= 0.08:
                    return 1.0
            except (TypeError, ValueError, IndexError):
                pass
        try:
            if float(self.replay_timeline_signal(snapshot)) >= 0.04:
                return 1.0
        except (TypeError, ValueError):
            return 0.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.drive_speed = float(controls.get("drive_speed", self.drive_speed))
        self.grip_friction = float(controls.get("grip_friction", self.grip_friction))
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
        self._restore_histories(self._box_history, state.get("box_history", {}))
        self._restore_histories(self._slip_history, state.get("slip_history", {}))
        self._restore_histories(self._ratio_history, state.get("ratio_history", {}))
        self._restore_histories(self._gap_history, state.get("gap_history", {}))
        self._restore_histories(self._contact_history, state.get("contact_history", {}))
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

    def _case_text(self, builder: Any, case: _DriverCase) -> None:
        metrics = self._last_metrics.get(case.key) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"driver {float(metrics['driver_travel']):.3f} m | "
            f"box {float(metrics['box_travel']):.3f} m | "
            f"slip {float(metrics['slip']):.3f} m"
        )
        builder.text(
            f"speed ratio {float(metrics['speed_ratio']):.2f} | "
            f"support gap {float(metrics['support_gap']):.4f} m | "
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"step {float(metrics['step_ms']):.3f} ms"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_speed, drive_speed = builder.slider(
            "Driver speed", float(self.drive_speed), 0.05, 0.80
        )
        if changed_speed:
            self.drive_speed = float(drive_speed)

        changed_friction, grip_friction = builder.slider(
            "Grip friction", float(self.grip_friction), 0.05, 0.95
        )
        if changed_friction:
            self.grip_friction = float(grip_friction)

        if changed_executor or changed_speed or changed_friction:
            self._reset()

        if builder.button("Reset driver"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: prescribed tangential contact response")
        builder.text(
            f"held fixed: executor {self._executor_label()} | tangential kinematic "
            f"support | box mass 1.0 | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text(
            "solver lanes: IPC grip, IPC low-friction slip, Sequential impulse caveat"
        )
        builder.text("mode: prescribed kinematic support")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        builder.plot_lines("IPC grip box travel", list(self._box_history["ipc_grip"]))
        builder.plot_lines("IPC slip box travel", list(self._box_history["ipc_slip"]))
        builder.plot_lines("SI caveat box travel", list(self._box_history["si_caveat"]))
        builder.plot_lines("IPC grip slip", list(self._slip_history["ipc_grip"]))
        builder.plot_lines("IPC slip amount", list(self._slip_history["ipc_slip"]))
        builder.plot_lines("SI driver travel", list(self._driver_history["si_caveat"]))
        builder.separator()
        for case in self.cases:
            case.bridge.build_control_panel(builder, context)
            builder.separator()


def build() -> SceneSetup:
    driver = _RigidKinematicDriver()
    return SceneSetup(
        world=driver.render_world,
        pre_step=driver.pre_step,
        force_drag=driver.force_drag,
        renderable_provider=driver.renderable_provider,
        panels=[ScenePanel("Rigid Kinematic Driver", driver.build_panel)],
        info={
            "sx_world": driver.primary_world,
            "rigid_kinematic_driver_controller": driver,
            "rigid_kinematic_driver_worlds": [case.world for case in driver.cases],
            CAPTURE_METRICS_INFO_KEY: driver.capture_metrics,
            "replay_capture_state": driver.capture_replay_state,
            "replay_restore_state": driver.restore_replay_state,
            "replay_sync": driver._sync,
            "replay_timeline": {
                "signal_label": "IPC grip box travel",
                "signal": driver.replay_timeline_signal,
                "markers": driver.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_kinematic_driver",
    title="Rigid Kinematic Driver",
    category="World Rigid Body",
    summary=(
        "Shows IPC prescribed-motion rigid supports carrying a box through "
        "friction, with slip and sequential-impulse caveat lanes."
    ),
    build=build,
)
