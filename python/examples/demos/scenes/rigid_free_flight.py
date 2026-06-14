"""No-contact rigid free-flight verifier for DART 7 World."""

from __future__ import annotations

from collections import deque
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_SPHERE_RADIUS = 0.065
_PATH_MARKERS = 16
_PATH_MARKER_DT = 0.13
_DRIFT_START = np.array([-1.30, 0.54, 0.58])
_ARC_START = np.array([-1.30, 0.0, 0.72])
_SPIN_LOW_START = np.array([-0.46, -0.58, 0.58])
_SPIN_HIGH_START = np.array([0.50, -0.58, 0.58])
_BAR_HALF = np.array([0.28, 0.045, 0.045])
_GRAVITY_Z = -9.81
_LOW_SPIN_INERTIA_Z = 0.035


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _bar_inertia(z_scale: float = 1.0) -> tuple[tuple[float, float, float], ...]:
    return (
        (0.030, 0.0, 0.0),
        (0.0, 0.030, 0.0),
        (0.0, 0.0, _LOW_SPIN_INERTIA_Z * float(z_scale)),
    )


def _make_marker(
    render_world: Any,
    name: str,
    position: np.ndarray,
    color: tuple[float, float, float],
    radius: float = 0.022,
) -> Any:
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), name, _transform_at(position)
    )
    frame.set_shape(dart.SphereShape(radius))
    frame.create_visual_aspect().set_color(list(color))
    render_world.add_simple_frame(frame)
    return frame


class _RigidFreeFlight:
    def __init__(self) -> None:
        self.executor_index = 0
        self.launch_speed = 1.15
        self.launch_angle_deg = 32.0
        self.gravity_scale = 0.55
        self.spin_speed = 3.0
        self.inertia_ratio = 4.0
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.drift_world = self._make_world("drift", (0.0, 0.0, 0.0))
        self.arc_world = self._make_world("arc", self._arc_gravity())
        self.spin_world = self._make_world("spin", (0.0, 0.0, 0.0))

        self.drift_body = self.drift_world.add_rigid_body("free_drift_body")
        self.drift_body.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))

        self.arc_body = self.arc_world.add_rigid_body("free_arc_body")
        self.arc_body.set_collision_shape(sx.CollisionShape.sphere(_SPHERE_RADIUS))

        self.spin_low_body = self.spin_world.add_rigid_body("low_inertia_spin_body")
        self.spin_low_body.set_collision_shape(sx.CollisionShape.box(_BAR_HALF))
        self.spin_high_body = self.spin_world.add_rigid_body("high_inertia_spin_body")
        self.spin_high_body.set_collision_shape(sx.CollisionShape.box(_BAR_HALF))

        for world in self.worlds:
            world.enter_simulation_mode()

        self.drift_bridge = WorldRenderBridge(
            self.drift_world, name="rigid_free_drift_render"
        )
        self.arc_bridge = WorldRenderBridge(self.arc_world, name="rigid_free_arc_render")
        self.spin_bridge = WorldRenderBridge(
            self.spin_world, name="rigid_free_spin_render"
        )
        for bridge in self.bridges:
            bridge.force_drag_enabled = False
        self.drift_bridge.add_rigid_body_visual(
            self.drift_body,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.24, 0.58, 0.90),
            name="free_drift_visual",
        )
        self.arc_bridge.add_rigid_body_visual(
            self.arc_body,
            dart.SphereShape(_SPHERE_RADIUS),
            (0.90, 0.58, 0.24),
            name="free_arc_visual",
        )
        self.spin_bridge.add_rigid_body_visual(
            self.spin_low_body,
            dart.BoxShape(_full(_BAR_HALF)),
            (0.40, 0.72, 0.42),
            name="low_inertia_spin_visual",
        )
        self.spin_bridge.add_rigid_body_visual(
            self.spin_high_body,
            dart.BoxShape(_full(_BAR_HALF)),
            (0.74, 0.42, 0.82),
            name="high_inertia_spin_visual",
        )

        self._drift_markers = self._make_path_markers(
            self.drift_bridge.render_world,
            "drift_reference",
            (0.62, 0.78, 0.92),
        )
        self._arc_markers = self._make_path_markers(
            self.arc_bridge.render_world,
            "arc_reference",
            (0.95, 0.78, 0.32),
        )

        self._drift_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._arc_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._drift_momentum_history: deque[float] = deque(maxlen=_HISTORY)
        self._arc_momentum_residual_history: deque[float] = deque(maxlen=_HISTORY)
        self._arc_energy_drift_history: deque[float] = deque(maxlen=_HISTORY)
        self._spin_momentum_ratio_history: deque[float] = deque(maxlen=_HISTORY)
        self._spin_energy_ratio_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._initial_reference: dict[str, Any] = {}
        self._last_metrics: dict[str, float | str] = {}
        self._reset()

    @property
    def worlds(self) -> tuple[Any, Any, Any]:
        return (self.drift_world, self.arc_world, self.spin_world)

    @property
    def bridges(self) -> tuple[WorldRenderBridge, WorldRenderBridge, WorldRenderBridge]:
        return (self.drift_bridge, self.arc_bridge, self.spin_bridge)

    @property
    def primary_world(self) -> Any:
        return self.drift_world

    @property
    def render_world(self) -> Any:
        return self.drift_bridge.render_world

    def _make_world(
        self, name: str, gravity: tuple[float, float, float]
    ) -> sx.World:
        world = sx.World(
            time_step=_TIME_STEP,
            gravity=gravity,
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        world.step_profiling_enabled = True
        return world

    def _make_path_markers(
        self,
        render_world: Any,
        prefix: str,
        color: tuple[float, float, float],
    ) -> list[Any]:
        return [
            _make_marker(
                render_world,
                f"{prefix}_{index}",
                np.array([0.0, 0.0, -10.0]),
                color,
            )
            for index in range(_PATH_MARKERS)
        ]

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _arc_gravity(self) -> tuple[float, float, float]:
        scale = max(0.0, float(self.gravity_scale))
        return (0.0, 0.0, _GRAVITY_Z * scale)

    def _arc_velocity(self) -> np.ndarray:
        angle = np.deg2rad(float(self.launch_angle_deg))
        speed = float(self.launch_speed)
        return np.array([speed * np.cos(angle), 0.0, speed * np.sin(angle)])

    def _drift_velocity(self) -> np.ndarray:
        return np.array([float(self.launch_speed), 0.0, 0.0])

    def _spin_velocity(self) -> np.ndarray:
        return np.array([0.0, 0.0, float(self.spin_speed)])

    def _apply_parameters(self) -> None:
        self.arc_world.gravity = self._arc_gravity()
        self.spin_low_body.mass = 1.0
        self.spin_high_body.mass = 1.0
        self.spin_low_body.inertia = _bar_inertia(1.0)
        self.spin_high_body.inertia = _bar_inertia(max(1.0, self.inertia_ratio))

    def _reset_body_state(self) -> None:
        self.drift_body.transform = _transform_at(_DRIFT_START)
        self.drift_body.linear_velocity = self._drift_velocity()
        self.drift_body.angular_velocity = (0.0, 0.0, 0.0)
        self.drift_body.clear_force()
        self.drift_body.clear_torque()

        self.arc_body.transform = _transform_at(_ARC_START)
        self.arc_body.linear_velocity = self._arc_velocity()
        self.arc_body.angular_velocity = (0.0, 0.0, 0.0)
        self.arc_body.clear_force()
        self.arc_body.clear_torque()

        spin = self._spin_velocity()
        for body, position in (
            (self.spin_low_body, _SPIN_LOW_START),
            (self.spin_high_body, _SPIN_HIGH_START),
        ):
            body.transform = _transform_at(position)
            body.linear_velocity = (0.0, 0.0, 0.0)
            body.angular_velocity = spin
            body.clear_force()
            body.clear_torque()

        for world in self.worlds:
            world.time = 0.0
            try:
                world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
            world.update_kinematics()

    def _reset(self) -> None:
        self._apply_parameters()
        self._reset_body_state()
        self._initial_reference = self._capture_initial_reference()
        for history in (
            self._drift_error_history,
            self._arc_error_history,
            self._drift_momentum_history,
            self._arc_momentum_residual_history,
            self._arc_energy_drift_history,
            self._spin_momentum_ratio_history,
            self._spin_energy_ratio_history,
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._update_reference_markers()
        self._sync()

    def _capture_initial_reference(self) -> dict[str, Any]:
        return {
            "drift_linear_momentum": np.asarray(
                self.drift_body.linear_momentum, dtype=float
            ).copy(),
            "arc_linear_momentum": np.asarray(
                self.arc_body.linear_momentum, dtype=float
            ).copy(),
            "arc_energy": float(
                self.arc_body.kinetic_energy + self.arc_body.potential_energy
            ),
            "spin_low_angular_momentum": np.asarray(
                self.spin_low_body.angular_momentum, dtype=float
            ).copy(),
            "spin_high_angular_momentum": np.asarray(
                self.spin_high_body.angular_momentum, dtype=float
            ).copy(),
            "spin_low_energy": float(
                self.spin_low_body.kinetic_energy
                + self.spin_low_body.potential_energy
            ),
            "spin_high_energy": float(
                self.spin_high_body.kinetic_energy
                + self.spin_high_body.potential_energy
            ),
        }

    def _expected_drift_position(self, time_s: float) -> np.ndarray:
        return _DRIFT_START + self._drift_velocity() * float(time_s)

    def _expected_arc_position(self, time_s: float) -> np.ndarray:
        t = float(time_s)
        gravity = np.asarray(self._arc_gravity(), dtype=float)
        return _ARC_START + self._arc_velocity() * t + 0.5 * gravity * t * t

    def _expected_arc_momentum(self, time_s: float) -> np.ndarray:
        gravity = np.asarray(self._arc_gravity(), dtype=float)
        return self._initial_reference["arc_linear_momentum"] + (
            self.arc_body.mass * gravity * float(time_s)
        )

    def _update_reference_markers(self) -> None:
        for index, marker in enumerate(self._drift_markers):
            time_s = index * _PATH_MARKER_DT
            marker.set_transform(_transform_at(self._expected_drift_position(time_s)))
        for index, marker in enumerate(self._arc_markers):
            time_s = index * _PATH_MARKER_DT
            marker.set_transform(_transform_at(self._expected_arc_position(time_s)))

    def _step_profile_ms(self, world: Any) -> float:
        try:
            profile = world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _safe_ratio(self, numerator: float, denominator: float) -> float:
        if abs(denominator) < 1.0e-12:
            return 0.0
        return float(numerator / denominator)

    def _record_metrics(self) -> None:
        drift_position = np.asarray(self.drift_body.translation, dtype=float).reshape(3)
        drift_expected = self._expected_drift_position(float(self.drift_world.time))
        drift_momentum = np.asarray(
            self.drift_body.linear_momentum, dtype=float
        ).reshape(3)
        drift_momentum_drift = float(
            np.linalg.norm(
                drift_momentum - self._initial_reference["drift_linear_momentum"]
            )
        )
        drift_error = float(np.linalg.norm(drift_position - drift_expected))

        arc_position = np.asarray(self.arc_body.translation, dtype=float).reshape(3)
        arc_expected = self._expected_arc_position(float(self.arc_world.time))
        arc_momentum = np.asarray(self.arc_body.linear_momentum, dtype=float).reshape(3)
        arc_expected_momentum = self._expected_arc_momentum(float(self.arc_world.time))
        arc_momentum_residual = float(
            np.linalg.norm(arc_momentum - arc_expected_momentum)
        )
        arc_energy = float(self.arc_body.kinetic_energy + self.arc_body.potential_energy)
        arc_energy_drift = arc_energy - float(self._initial_reference["arc_energy"])
        arc_error = float(np.linalg.norm(arc_position - arc_expected))

        low_angular_momentum = np.asarray(
            self.spin_low_body.angular_momentum, dtype=float
        ).reshape(3)
        high_angular_momentum = np.asarray(
            self.spin_high_body.angular_momentum, dtype=float
        ).reshape(3)
        low_energy = float(
            self.spin_low_body.kinetic_energy + self.spin_low_body.potential_energy
        )
        high_energy = float(
            self.spin_high_body.kinetic_energy + self.spin_high_body.potential_energy
        )
        spin_momentum_ratio = self._safe_ratio(
            abs(float(high_angular_momentum[2])),
            abs(float(low_angular_momentum[2])),
        )
        spin_energy_ratio = self._safe_ratio(high_energy, low_energy)
        step_ms = sum(self._step_profile_ms(world) for world in self.worlds)
        contact_count = sum(len(world.collide()) for world in self.worlds)

        self._last_metrics = {
            "drift_position_error": drift_error,
            "drift_momentum_drift": drift_momentum_drift,
            "drift_speed": float(np.linalg.norm(self.drift_body.linear_velocity)),
            "arc_position_error": arc_error,
            "arc_momentum_residual": arc_momentum_residual,
            "arc_energy_drift": arc_energy_drift,
            "arc_height": float(arc_position[2]),
            "arc_expected_height": float(arc_expected[2]),
            "spin_low_angular_speed": float(
                np.linalg.norm(self.spin_low_body.angular_velocity)
            ),
            "spin_high_angular_speed": float(
                np.linalg.norm(self.spin_high_body.angular_velocity)
            ),
            "spin_momentum_ratio": spin_momentum_ratio,
            "spin_energy_ratio": spin_energy_ratio,
            "step_ms": step_ms,
            "time": float(self.primary_world.time),
            "contact_count": float(contact_count),
            "status": "no contacts" if contact_count == 0 else f"{contact_count} contacts",
        }

        self._drift_error_history.append(drift_error)
        self._arc_error_history.append(arc_error)
        self._drift_momentum_history.append(drift_momentum_drift)
        self._arc_momentum_residual_history.append(arc_momentum_residual)
        self._arc_energy_drift_history.append(abs(arc_energy_drift))
        self._spin_momentum_ratio_history.append(spin_momentum_ratio)
        self._spin_energy_ratio_history.append(spin_energy_ratio)
        self._step_ms_history.append(step_ms)

    def _sync(self) -> None:
        for bridge in self.bridges:
            bridge.sync()

    def pre_step(self) -> None:
        executor = self._executor()
        for world in self.worlds:
            world.step(executor)
        self._record_metrics()
        self._sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        if not event.get("active", False):
            for bridge in self.bridges:
                bridge.force_drag(event)
            return
        renderable_id = int(event.get("renderable_id", 0) or 0)
        for bridge in self.bridges:
            if renderable_id in getattr(bridge, "_by_renderable_id", {}):
                bridge.force_drag(event)
                return

    def renderable_provider(self) -> list[Any]:
        renderables: list[Any] = []
        for bridge in self.bridges:
            renderables.extend(bridge.renderable_provider())
        return renderables

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "launch_speed": float(self.launch_speed),
                "launch_angle_deg": float(self.launch_angle_deg),
                "gravity_scale": float(self.gravity_scale),
                "spin_speed": float(self.spin_speed),
                "inertia_ratio": float(self.inertia_ratio),
            },
            "world_states": {
                "drift": np.asarray(self.drift_world.state_vector, dtype=float).copy(),
                "arc": np.asarray(self.arc_world.state_vector, dtype=float).copy(),
                "spin": np.asarray(self.spin_world.state_vector, dtype=float).copy(),
            },
            "world_times": {
                "drift": float(self.drift_world.time),
                "arc": float(self.arc_world.time),
                "spin": float(self.spin_world.time),
            },
            "initial_reference": dict(self._initial_reference),
            "histories": {
                "drift_error": list(self._drift_error_history),
                "arc_error": list(self._arc_error_history),
                "drift_momentum": list(self._drift_momentum_history),
                "arc_momentum_residual": list(self._arc_momentum_residual_history),
                "arc_energy_drift": list(self._arc_energy_drift_history),
                "spin_momentum_ratio": list(self._spin_momentum_ratio_history),
                "spin_energy_ratio": list(self._spin_energy_ratio_history),
                "step_ms": list(self._step_ms_history),
            },
            "last_metrics": dict(self._last_metrics),
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = dict(self._last_metrics)
        if not metrics:
            metrics = {
                "drift_position_error": 0.0,
                "drift_momentum_drift": 0.0,
                "drift_speed": float(np.linalg.norm(self.drift_body.linear_velocity)),
                "arc_position_error": 0.0,
                "arc_momentum_residual": 0.0,
                "arc_energy_drift": 0.0,
                "arc_height": float(self.arc_body.translation[2]),
                "arc_expected_height": float(self.arc_body.translation[2]),
                "spin_low_angular_speed": float(
                    np.linalg.norm(self.spin_low_body.angular_velocity)
                ),
                "spin_high_angular_speed": float(
                    np.linalg.norm(self.spin_high_body.angular_velocity)
                ),
                "spin_momentum_ratio": float(self.inertia_ratio),
                "spin_energy_ratio": float(self.inertia_ratio),
                "step_ms": 0.0,
                "time": float(self.primary_world.time),
                "contact_count": 0.0,
                "status": "no contacts",
            }
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_free_flight",
            "solver": "sequential_impulse",
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "contact_free_initial_velocity_gravity_spin",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.primary_world.time),
            "launch_speed": float(self.launch_speed),
            "launch_angle_deg": float(self.launch_angle_deg),
            "gravity_z": float(self._arc_gravity()[2]),
            "spin_speed": float(self.spin_speed),
            "inertia_ratio": float(self.inertia_ratio),
            "metrics": metrics,
            "controls": {
                "executor_index": float(self.executor_index),
                "launch_speed": float(self.launch_speed),
                "launch_angle_deg": float(self.launch_angle_deg),
                "gravity_scale": float(self.gravity_scale),
                "spin_speed": float(self.spin_speed),
                "inertia_ratio": float(self.inertia_ratio),
            },
            "drift_position_error": float(metrics["drift_position_error"]),
            "drift_momentum_drift": float(metrics["drift_momentum_drift"]),
            "drift_speed": float(metrics["drift_speed"]),
            "arc_position_error": float(metrics["arc_position_error"]),
            "arc_momentum_residual": float(metrics["arc_momentum_residual"]),
            "arc_energy_drift": float(metrics["arc_energy_drift"]),
            "arc_height": float(metrics["arc_height"]),
            "spin_momentum_ratio": float(metrics["spin_momentum_ratio"]),
            "spin_energy_ratio": float(metrics["spin_energy_ratio"]),
            "contact_count": float(metrics["contact_count"]),
            "step_ms": float(metrics["step_ms"]),
            "history": {
                "samples": float(len(step_values)),
                "max_drift_position_error": max(
                    (float(value) for value in self._drift_error_history),
                    default=0.0,
                ),
                "max_arc_position_error": max(
                    (float(value) for value in self._arc_error_history),
                    default=0.0,
                ),
                "max_arc_momentum_residual": max(
                    (float(value) for value in self._arc_momentum_residual_history),
                    default=0.0,
                ),
                "max_arc_energy_drift": max(
                    (float(value) for value in self._arc_energy_drift_history),
                    default=0.0,
                ),
                "max_spin_momentum_ratio": max(
                    (float(value) for value in self._spin_momentum_ratio_history),
                    default=float(self.inertia_ratio),
                ),
                "max_step_ms": max((float(value) for value in step_values), default=0.0),
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.launch_speed = float(controls.get("launch_speed", self.launch_speed))
        self.launch_angle_deg = float(
            controls.get("launch_angle_deg", self.launch_angle_deg)
        )
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self.spin_speed = float(controls.get("spin_speed", self.spin_speed))
        self.inertia_ratio = float(controls.get("inertia_ratio", self.inertia_ratio))
        self._apply_parameters()

        world_states = state.get("world_states", {})
        world_times = state.get("world_times", {})
        for key, world in (
            ("drift", self.drift_world),
            ("arc", self.arc_world),
            ("spin", self.spin_world),
        ):
            if key in world_states:
                world.state_vector = world_states[key]
            if key in world_times:
                world.time = float(world_times[key])
            world.update_kinematics()

        self._initial_reference = dict(
            state.get("initial_reference", self._capture_initial_reference())
        )
        histories = state.get("histories", {})
        self._restore_history(self._drift_error_history, histories, "drift_error")
        self._restore_history(self._arc_error_history, histories, "arc_error")
        self._restore_history(self._drift_momentum_history, histories, "drift_momentum")
        self._restore_history(
            self._arc_momentum_residual_history, histories, "arc_momentum_residual"
        )
        self._restore_history(
            self._arc_energy_drift_history, histories, "arc_energy_drift"
        )
        self._restore_history(
            self._spin_momentum_ratio_history, histories, "spin_momentum_ratio"
        )
        self._restore_history(
            self._spin_energy_ratio_history, histories, "spin_energy_ratio"
        )
        self._restore_history(self._step_ms_history, histories, "step_ms")
        self._last_metrics = dict(state.get("last_metrics", {}))
        self._update_reference_markers()
        self._sync()

    def _restore_history(
        self,
        history: deque[float],
        histories: dict[str, list[float]],
        key: str,
    ) -> None:
        history.clear()
        history.extend(float(value) for value in histories.get(key, []))

    def build_panel(self, builder: Any, context: Any) -> None:
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_choices
        )
        changed_speed, launch_speed = builder.slider(
            "Launch speed", float(self.launch_speed), 0.2, 2.4
        )
        changed_angle, launch_angle_deg = builder.slider(
            "Launch angle", float(self.launch_angle_deg), -10.0, 65.0
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.5
        )
        changed_spin, spin_speed = builder.slider(
            "Spin speed", float(self.spin_speed), -8.0, 8.0
        )
        changed_inertia, inertia_ratio = builder.slider(
            "Spin inertia ratio", float(self.inertia_ratio), 1.0, 8.0
        )

        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_speed:
            self.launch_speed = float(launch_speed)
        if changed_angle:
            self.launch_angle_deg = float(launch_angle_deg)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if changed_spin:
            self.spin_speed = float(spin_speed)
        if changed_inertia:
            self.inertia_ratio = float(inertia_ratio)
        if (
            changed_executor
            or changed_speed
            or changed_angle
            or changed_gravity
            or changed_spin
            or changed_inertia
        ):
            self._reset()

        if builder.button("Reset free flight"):
            self._reset()

        metrics = self._last_metrics or {
            "drift_position_error": 0.0,
            "drift_momentum_drift": 0.0,
            "drift_speed": float(self.launch_speed),
            "arc_position_error": 0.0,
            "arc_momentum_residual": 0.0,
            "arc_energy_drift": 0.0,
            "arc_height": float(_ARC_START[2]),
            "arc_expected_height": float(_ARC_START[2]),
            "spin_low_angular_speed": abs(float(self.spin_speed)),
            "spin_high_angular_speed": abs(float(self.spin_speed)),
            "spin_momentum_ratio": float(self.inertia_ratio),
            "spin_energy_ratio": float(self.inertia_ratio),
            "step_ms": 0.0,
            "time": 0.0,
            "contact_count": 0.0,
            "status": "no contacts",
        }

        builder.separator()
        builder.text(
            f"world time: {float(metrics['time']):.3f} s | "
            f"{str(metrics['status'])}"
        )
        builder.text(
            f"zero-g drift error {float(metrics['drift_position_error']) * 1000.0:.3f} mm | "
            f"momentum drift {float(metrics['drift_momentum_drift']):.2e}"
        )
        builder.text(
            f"gravity arc error {float(metrics['arc_position_error']) * 1000.0:.2f} mm | "
            f"momentum residual {float(metrics['arc_momentum_residual']):.2e}"
        )
        builder.text(
            f"arc height {float(metrics['arc_height']):.3f} m "
            f"(reference {float(metrics['arc_expected_height']):.3f} m) | "
            f"energy drift {float(metrics['arc_energy_drift']):.3e} J"
        )
        builder.text(
            f"spin ratio Lz {float(metrics['spin_momentum_ratio']):.2f} | "
            f"KE ratio {float(metrics['spin_energy_ratio']):.2f}"
        )
        builder.text(f"combined step profile {float(metrics['step_ms']):.3f} ms")
        builder.plot_lines("Zero-g drift position error", list(self._drift_error_history))
        builder.plot_lines("Gravity arc position error", list(self._arc_error_history))
        builder.plot_lines(
            "Gravity arc momentum residual",
            list(self._arc_momentum_residual_history),
        )
        builder.plot_lines("Gravity arc energy drift", list(self._arc_energy_drift_history))
        builder.plot_lines("Spin Lz ratio", list(self._spin_momentum_ratio_history))
        builder.plot_lines("Spin KE ratio", list(self._spin_energy_ratio_history))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.drift_bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    flight = _RigidFreeFlight()
    return SceneSetup(
        world=flight.render_world,
        pre_step=flight.pre_step,
        force_drag=flight.force_drag,
        renderable_provider=flight.renderable_provider,
        panels=[ScenePanel("Rigid Free Flight", flight.build_panel)],
        info={
            "sx_world": flight.primary_world,
            "rigid_free_flight_controller": flight,
            "rigid_free_flight_worlds": list(flight.worlds),
            "replay_capture_state": flight.capture_replay_state,
            "replay_restore_state": flight.restore_replay_state,
            "replay_sync": flight._sync,
            CAPTURE_METRICS_INFO_KEY: flight.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_free_flight",
    title="Rigid Free Flight",
    category="World Rigid Body",
    summary=(
        "Shows no-contact initial velocity, gravity arc, spin, inertia, "
        "momentum, and energy diagnostics."
    ),
    build=build,
)
