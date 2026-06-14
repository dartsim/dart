"""Side-by-side rigid contact-solver policy comparison for DART 7 World."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_GROUND_HALF = np.array([0.82, 0.45, 0.04])
_PLANK_HALF = np.array([0.28, 0.16, 0.055])
_LEFT_OFFSET = -0.92
_RIGHT_OFFSET = 0.92
_TIME_STEP = 0.004
_HISTORY = 180


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _rotation_y(angle_rad: float) -> np.ndarray:
    transform = np.eye(4)
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    transform[:3, :3] = np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )
    return transform


def _transform_at(position: np.ndarray, tilt_deg: float = 0.0) -> np.ndarray:
    transform = _rotation_y(math.radians(float(tilt_deg)))
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _plank_corner_clearance(transform: np.ndarray) -> float:
    corners = []
    for sx_sign in (-1.0, 1.0):
        for sy_sign in (-1.0, 1.0):
            for sz_sign in (-1.0, 1.0):
                corners.append(
                    np.array(
                        [
                            sx_sign * _PLANK_HALF[0],
                            sy_sign * _PLANK_HALF[1],
                            sz_sign * _PLANK_HALF[2],
                            1.0,
                        ],
                        dtype=float,
                    )
                )
    world_corners = [np.asarray(transform, dtype=float) @ corner for corner in corners]
    return float(min(corner[2] for corner in world_corners))


@dataclass
class _ContactSolverCase:
    label: str
    method: sx.ContactSolverMethod
    offset_x: float
    color: tuple[float, float, float]
    world: Any
    bridge: WorldRenderBridge
    ground: Any
    plank: Any
    initial_plank_position: np.ndarray


class _RigidContactSolverCompare:
    def __init__(self) -> None:
        self.friction = 0.70
        self.restitution = 0.05
        self.launch_speed = 0.40
        self.initial_tilt_deg = 8.5
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
                "Sequential impulse contacts",
                sx.ContactSolverMethod.SEQUENTIAL_IMPULSE,
                _LEFT_OFFSET,
                (0.22, 0.52, 0.88),
                "si_contact",
            ),
            self._make_case(
                "Boxed LCP contacts",
                sx.ContactSolverMethod.BOXED_LCP,
                _RIGHT_OFFSET,
                (0.90, 0.46, 0.22),
                "boxed_lcp",
            ),
        ]
        self._speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._angular_speed_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._contact_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._depth_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._clearance_history: dict[str, deque[float]] = {
            case.label: deque(maxlen=_HISTORY) for case in self.cases
        }
        self._energy_history: dict[str, deque[float]] = {
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
        method: sx.ContactSolverMethod,
        offset_x: float,
        color: tuple[float, float, float],
        prefix: str,
    ) -> _ContactSolverCase:
        world = sx.World(
            time_step=_TIME_STEP,
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
            contact_solver_method=method,
        )
        world.step_profiling_enabled = True

        ground = world.add_rigid_body(
            f"{prefix}_ground", position=(offset_x, 0.0, -_GROUND_HALF[2])
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))

        initial_plank_position = np.array(
            [offset_x - 0.25, 0.0, 0.35],
            dtype=float,
        )
        plank = world.add_rigid_body(f"{prefix}_tilted_plank")
        plank.mass = 1.0
        plank.set_collision_shape(sx.CollisionShape.box(_PLANK_HALF))

        world.enter_simulation_mode()

        bridge = WorldRenderBridge(world, name=f"{prefix}_contact_solver_render")
        bridge.add_rigid_body_visual(
            ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.38, 0.40, 0.44),
            name=f"{prefix}_ground_visual",
        )
        bridge.add_rigid_body_visual(
            plank,
            dart.BoxShape(_full(_PLANK_HALF)),
            color,
            name=f"{prefix}_tilted_plank_visual",
        )

        case = _ContactSolverCase(
            label=label,
            method=method,
            offset_x=offset_x,
            color=color,
            world=world,
            bridge=bridge,
            ground=ground,
            plank=plank,
            initial_plank_position=initial_plank_position,
        )
        self._apply_parameters(case)
        self._reset_case(case)
        return case

    def _apply_parameters(self, case: _ContactSolverCase) -> None:
        for body in (case.ground, case.plank):
            body.friction = float(self.friction)
            body.restitution = float(self.restitution)

    def _reset_case(self, case: _ContactSolverCase) -> None:
        case.plank.transform = _transform_at(
            case.initial_plank_position, self.initial_tilt_deg
        )
        case.plank.linear_velocity = (float(self.launch_speed), 0.0, -0.25)
        case.plank.angular_velocity = (0.0, 2.5, 0.0)
        case.plank.clear_force()
        case.plank.clear_torque()
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
            *self._angular_speed_history.values(),
            *self._contact_history.values(),
            *self._depth_history.values(),
            *self._clearance_history.values(),
            *self._energy_history.values(),
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

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _plank_contacts(self, case: _ContactSolverCase) -> list[Any]:
        return [
            contact
            for contact in case.world.collide()
            if case.plank.name in (contact.body_a.name, contact.body_b.name)
        ]

    def _sample(self, case: _ContactSolverCase) -> dict[str, float | str]:
        position = np.asarray(case.plank.translation, dtype=float).reshape(3)
        linear_velocity = np.asarray(case.plank.linear_velocity, dtype=float).reshape(3)
        angular_velocity = np.asarray(case.plank.angular_velocity, dtype=float).reshape(
            3
        )
        contacts = self._plank_contacts(case)
        depths = [float(contact.depth) for contact in contacts]
        contact_count = float(len(contacts))
        max_depth = max(depths, default=0.0)
        clearance = _plank_corner_clearance(np.asarray(case.plank.transform, dtype=float))
        speed = float(np.linalg.norm(linear_velocity))
        angular_speed = float(np.linalg.norm(angular_velocity))
        energy = float(case.plank.kinetic_energy)
        if contact_count >= 2.0 and speed < 0.05 and angular_speed < 0.10:
            status = "settled multi-contact"
        elif contact_count >= 2.0:
            status = "coupled contacts"
        elif contact_count > 0.0:
            status = "single contact"
        elif linear_velocity[2] < 0.0:
            status = "falling"
        else:
            status = "separating"
        return {
            "angular_speed": angular_speed,
            "clearance": float(clearance),
            "contact_count": contact_count,
            "energy": energy,
            "max_depth": max_depth,
            "relative_x": float(position[0] - case.offset_x),
            "speed": speed,
            "status": status,
            "step_ms": self._step_profile_ms(case.world),
        }

    def _record_metrics(self) -> None:
        positions = []
        for case in self.cases:
            metrics = self._sample(case)
            self._last_metrics[case.label] = metrics
            self._speed_history[case.label].append(float(metrics["speed"]))
            self._angular_speed_history[case.label].append(
                float(metrics["angular_speed"])
            )
            self._contact_history[case.label].append(float(metrics["contact_count"]))
            self._depth_history[case.label].append(float(metrics["max_depth"]))
            self._clearance_history[case.label].append(float(metrics["clearance"]))
            self._energy_history[case.label].append(float(metrics["energy"]))
            self._step_ms_history[case.label].append(float(metrics["step_ms"]))
            positions.append(
                np.asarray(case.plank.translation, dtype=float).reshape(3)
                - np.array([case.offset_x, 0.0, 0.0])
            )
        if len(positions) == 2:
            self._divergence_history.append(
                float(np.linalg.norm(positions[0] - positions[1]))
            )

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
        divergence_values = list(self._divergence_history)
        return {
            "row": "rigid_contact_solver_compare",
            "comparison_axis": "contact_solver_method",
            "solver": "sequential_impulse_rigid_body",
            "contact_policy": "sequential_impulse_vs_boxed_lcp",
            "executor": self._executors[int(self.executor_index)][0],
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.primary_world.time),
            "case_pair": [case.label for case in self.cases],
            "contact_policy_pair": [case.method.name for case in self.cases],
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "initial_tilt_deg": float(self.initial_tilt_deg),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
            },
            "cases": {
                case.label: {
                    "contact_solver_method": case.method.name,
                    "rigid_body_solver": case.world.rigid_body_solver.name,
                    "metrics": dict(self._last_metrics[case.label]),
                }
                for case in self.cases
            },
            "divergence": {
                "current_pose": (
                    float(divergence_values[-1]) if divergence_values else 0.0
                ),
                "max_pose": max(divergence_values, default=0.0),
                "samples": float(len(divergence_values)),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        secondary = self.cases[1].world
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "initial_tilt_deg": float(self.initial_tilt_deg),
                "launch_speed": float(self.launch_speed),
                "restitution": float(self.restitution),
            },
            "secondary_state": np.asarray(secondary.state_vector, dtype=float).copy(),
            "secondary_time": float(secondary.time),
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "angular_speed_history": {
                key: list(values)
                for key, values in self._angular_speed_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "depth_history": {
                key: list(values) for key, values in self._depth_history.items()
            },
            "clearance_history": {
                key: list(values) for key, values in self._clearance_history.items()
            },
            "energy_history": {
                key: list(values) for key, values in self._energy_history.items()
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
        contact_histories = snapshot.get("contact_history", {})
        if isinstance(contact_histories, dict):
            for values in contact_histories.values():
                try:
                    if len(values) > 0 and float(values[-1]) >= 2.0:
                        return 1.0
                except (TypeError, ValueError, IndexError):
                    continue
        depth_histories = snapshot.get("depth_history", {})
        if isinstance(depth_histories, dict):
            for values in depth_histories.values():
                try:
                    if len(values) > 0 and float(values[-1]) > 0.0:
                        return 1.0
                except (TypeError, ValueError, IndexError):
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
        self.initial_tilt_deg = float(
            controls.get("initial_tilt_deg", self.initial_tilt_deg)
        )
        self.launch_speed = float(controls.get("launch_speed", self.launch_speed))
        self.restitution = float(controls.get("restitution", self.restitution))
        for case in self.cases:
            self._apply_parameters(case)
        secondary = self.cases[1].world
        secondary.state_vector = state["secondary_state"]
        secondary.time = float(state["secondary_time"])
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(
            self._angular_speed_history, state.get("angular_speed_history", {})
        )
        self._restore_histories(self._contact_history, state.get("contact_history", {}))
        self._restore_histories(self._depth_history, state.get("depth_history", {}))
        self._restore_histories(
            self._clearance_history, state.get("clearance_history", {})
        )
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
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

    def _case_text(self, builder: Any, case: _ContactSolverCase) -> None:
        metrics = self._last_metrics.get(case.label) or self._sample(case)
        builder.text(f"{case.label}: {metrics['status']}")
        builder.text(
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"depth {float(metrics['max_depth']):.4f} m | "
            f"clearance {float(metrics['clearance']):+.4f} m"
        )
        builder.text(
            f"speed {float(metrics['speed']):.3f} m/s | "
            f"angular {float(metrics['angular_speed']):.3f} rad/s | "
            f"energy {float(metrics['energy']):.3f} J"
        )
        builder.text(f"step profile {float(metrics['step_ms']):.3f} ms")

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_speed, launch_speed = builder.slider(
            "Launch speed", float(self.launch_speed), 0.0, 1.5
        )
        if changed_speed:
            self.launch_speed = float(launch_speed)

        changed_friction, friction = builder.slider(
            "Friction", float(self.friction), 0.0, 1.0
        )
        if changed_friction:
            self.friction = float(friction)

        changed_restitution, restitution = builder.slider(
            "Restitution", float(self.restitution), 0.0, 0.5
        )
        if changed_restitution:
            self.restitution = float(restitution)

        changed_tilt, initial_tilt = builder.slider(
            "Initial tilt", float(self.initial_tilt_deg), 0.0, 18.0
        )
        if changed_tilt:
            self.initial_tilt_deg = float(initial_tilt)

        if (
            changed_executor
            or changed_speed
            or changed_friction
            or changed_restitution
            or changed_tilt
        ):
            self._reset()

        if builder.button("Reset contact policy"):
            self._reset()

        builder.separator()
        builder.text("comparison axis: contact solver method")
        builder.text("shared rigid-body solver: sequential impulse")
        builder.text("contact-policy pair: sequential impulses vs boxed LCP")
        builder.text(f"world time: {self.primary_world.time:.3f} s")
        builder.text(f"time step: {_TIME_STEP:.4f} s")
        for case in self.cases:
            self._case_text(builder, case)
        if self._divergence_history:
            builder.text(f"pose divergence: {self._divergence_history[-1]:.4f} m")
        builder.plot_lines(
            "SI contact count", list(self._contact_history[self.cases[0].label])
        )
        builder.plot_lines(
            "LCP contact count", list(self._contact_history[self.cases[1].label])
        )
        builder.plot_lines("SI speed", list(self._speed_history[self.cases[0].label]))
        builder.plot_lines(
            "LCP speed", list(self._speed_history[self.cases[1].label])
        )
        builder.plot_lines(
            "SI max depth", list(self._depth_history[self.cases[0].label])
        )
        builder.plot_lines(
            "LCP max depth", list(self._depth_history[self.cases[1].label])
        )
        builder.plot_lines("Pose divergence", list(self._divergence_history))
        builder.separator()
        self.cases[0].bridge.build_control_panel(builder, context)
        builder.separator()
        self.cases[1].bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    comparison = _RigidContactSolverCompare()
    return SceneSetup(
        world=comparison.render_world,
        pre_step=comparison.pre_step,
        force_drag=comparison.force_drag,
        renderable_provider=comparison.renderable_provider,
        panels=[ScenePanel("Rigid Contact Solver Compare", comparison.build_panel)],
        info={
            "sx_world": comparison.primary_world,
            "rigid_contact_solver_compare_controller": comparison,
            "rigid_contact_solver_compare_worlds": [
                case.world for case in comparison.cases
            ],
            CAPTURE_METRICS_INFO_KEY: comparison.capture_metrics,
            "replay_capture_state": comparison.capture_replay_state,
            "replay_restore_state": comparison.restore_replay_state,
            "replay_sync": comparison._sync,
            "replay_timeline": {
                "signal_label": "Pose divergence",
                "signal": comparison.replay_timeline_signal,
                "markers": comparison.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_contact_solver_compare",
    title="Rigid Contact Solver Compare",
    category="World Rigid Body",
    summary=(
        "Sequential-impulse and boxed-LCP contact policies settle the same "
        "tilted multi-contact plank side by side."
    ),
    build=build,
)
