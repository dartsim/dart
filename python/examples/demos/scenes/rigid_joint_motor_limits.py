"""World joint motor and limit verifier for rigid multibody links."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_HISTORY = 180
_MOTOR_ANCHOR = np.array([-1.10, -0.78, 0.32])
_LIMIT_ANCHOR = np.array([-1.05, 0.28, 1.25])
_LIMIT_LENGTH = 0.78
_LIMIT_LOWER = -0.25
_FORCE_ANCHOR = np.array([-1.10, 1.12, 0.32])
_MOTOR_TRAVEL_LIMIT = 0.78
_FORCE_TRAVEL_LIMIT = 1.15
_CARRIAGE_MASS = 2.0
_AXIS_X = np.array([1.0, 0.0, 0.0])


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


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


def _box(size: tuple[float, float, float]) -> dart.BoxShape:
    return dart.BoxShape(np.asarray(size, dtype=float))


class _RigidJointMotorLimitVerifier:
    def __init__(self) -> None:
        self.command_speed = 0.55
        self.velocity_limit = 0.30
        self.position_limit = 0.35
        self.force_command = 16.0
        self.effort_limit = 4.0

        self.world = sx.World(time_step=_TIME_STEP)
        self.world.step_profiling_enabled = True

        self.motor_joint, self.motor_carriage = self._add_slider_robot(
            "velocity_motor",
            _MOTOR_ANCHOR,
        )
        self.limit_joint, self.limit_bob = self._add_limit_pendulum()
        self.limited_force_joint, self.limited_force_carriage = self._add_slider_robot(
            "effort_limited",
            _FORCE_ANCHOR,
        )
        self.open_force_joint, self.open_force_carriage = self._add_slider_robot(
            "effort_reference",
            _FORCE_ANCHOR + np.array([0.0, 0.34, 0.0]),
        )

        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_joint_motor_limits")
        self._add_visuals()

        self._motor_speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._motor_position_history: deque[float] = deque(maxlen=_HISTORY)
        self._limit_angle_history: deque[float] = deque(maxlen=_HISTORY)
        self._limit_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._limited_acceleration_history: deque[float] = deque(maxlen=_HISTORY)
        self._open_acceleration_history: deque[float] = deque(maxlen=_HISTORY)
        self._force_position_gap_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=True)

    def _add_slider_robot(
        self,
        name: str,
        anchor: np.ndarray,
    ) -> tuple[Any, Any]:
        robot = self.world.add_multibody(name)
        base = robot.add_link(f"{name}_base")
        carriage = robot.add_link(
            f"{name}_carriage",
            parent=base,
            joint=sx.JointSpec(
                name=f"{name}_rail",
                type=sx.JointType.PRISMATIC,
                axis=tuple(_AXIS_X),
                transform_from_parent=_translation(anchor),
            ),
        )
        carriage.mass = _CARRIAGE_MASS
        carriage.inertia = ((0.04, 0.0, 0.0), (0.0, 0.04, 0.0), (0.0, 0.0, 0.04))
        return carriage.parent_joint, carriage

    def _add_limit_pendulum(self) -> tuple[Any, Any]:
        robot = self.world.add_multibody("position_limit")
        base = robot.add_link("position_limit_base")
        bob = robot.add_link(
            "position_limit_link",
            parent=base,
            joint=sx.JointSpec(
                name="position_limit_hinge",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, -1.0, 0.0),
                transform_from_parent=_translation(_LIMIT_ANCHOR),
            ),
        )
        bob.mass = 1.0
        bob.inertia = ((0.03, 0.0, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.03))
        return bob.parent_joint, bob

    def _add_static_visual(
        self,
        name: str,
        shape: Any,
        position: np.ndarray,
        color: tuple[float, float, float],
    ) -> Any:
        frame = dart.SimpleFrame(dart.gui.world_render_frame(), name, _translation(position))
        frame.set_shape(shape)
        frame.create_visual_aspect().set_color(list(color))
        self.bridge.render_world.add_simple_frame(frame)
        return frame

    def _add_visuals(self) -> None:
        rail_size = (1.55, 0.035, 0.035)
        self._add_static_visual(
            "velocity_motor_rail",
            _box(rail_size),
            _MOTOR_ANCHOR + np.array([_MOTOR_TRAVEL_LIMIT * 0.5, 0.0, -0.08]),
            (0.36, 0.38, 0.42),
        )
        self._add_static_visual(
            "effort_limited_rail",
            _box((1.70, 0.03, 0.03)),
            _FORCE_ANCHOR + np.array([_FORCE_TRAVEL_LIMIT * 0.5, 0.0, -0.08]),
            (0.36, 0.38, 0.42),
        )
        self._add_static_visual(
            "effort_reference_rail",
            _box((1.70, 0.03, 0.03)),
            _FORCE_ANCHOR + np.array([_FORCE_TRAVEL_LIMIT * 0.5, 0.34, -0.08]),
            (0.36, 0.38, 0.42),
        )
        self._add_static_visual(
            "position_limit_pivot",
            dart.SphereShape(0.07),
            _LIMIT_ANCHOR,
            (0.30, 0.32, 0.36),
        )
        self.limit_stop_frame = self._add_static_visual(
            "position_limit_stop",
            _box((0.05, 0.48, 0.36)),
            self._limit_stop_position(),
            (0.82, 0.30, 0.22),
        )

        self.bridge.add_link_visual(
            self.motor_carriage,
            _box((0.22, 0.14, 0.14)),
            (0.18, 0.50, 0.88),
            name="velocity_motor_carriage",
        )
        self.bridge.add_link_visual(
            self.limit_bob,
            _box((_LIMIT_LENGTH, 0.08, 0.08)),
            (0.88, 0.45, 0.18),
            name="position_limit_link",
            local_transform=_translation((_LIMIT_LENGTH * 0.5, 0.0, 0.0)),
        )
        self.bridge.add_link_visual(
            self.limited_force_carriage,
            _box((0.22, 0.12, 0.12)),
            (0.22, 0.68, 0.38),
            name="effort_limited_carriage",
        )
        self.bridge.add_link_visual(
            self.open_force_carriage,
            _box((0.22, 0.12, 0.12)),
            (0.82, 0.34, 0.76),
            name="effort_reference_carriage",
        )
        self.bridge.sync()

    def _apply_controls(self) -> None:
        command_speed = max(0.0, float(self.command_speed))
        velocity_limit = max(0.01, float(self.velocity_limit))
        position_limit = max(_LIMIT_LOWER + 0.02, float(self.position_limit))
        force_command = max(0.0, float(self.force_command))
        effort_limit = max(0.0, float(self.effort_limit))

        self.motor_joint.actuator_type = sx.ActuatorType.VELOCITY
        self.motor_joint.command_velocity = [command_speed]
        self.motor_joint.set_velocity_limits([-velocity_limit], [velocity_limit])
        self.motor_joint.set_position_limits([0.0], [_MOTOR_TRAVEL_LIMIT])

        self.limit_joint.actuator_type = sx.ActuatorType.FORCE
        self.limit_joint.set_position_limits([_LIMIT_LOWER], [position_limit])
        self.limit_joint.damping_coefficient = [0.0]
        self.limit_joint.force = [0.0]

        for joint, effort_cap in (
            (self.limited_force_joint, effort_limit),
            (self.open_force_joint, max(force_command, effort_limit)),
        ):
            joint.actuator_type = sx.ActuatorType.FORCE
            joint.force = [force_command]
            joint.set_effort_limits([-effort_cap], [effort_cap])
            joint.set_position_limits([0.0], [_FORCE_TRAVEL_LIMIT])

    def _limit_stop_position(self) -> np.ndarray:
        return _LIMIT_ANCHOR + np.array(
            [
                _LIMIT_LENGTH * np.cos(self.position_limit),
                0.0,
                -_LIMIT_LENGTH * np.sin(self.position_limit),
            ]
        )

    def reset(self, *, clear_replay: bool = False) -> None:
        self._apply_controls()
        for joint in (
            self.motor_joint,
            self.limit_joint,
            self.limited_force_joint,
            self.open_force_joint,
        ):
            joint.position = [0.0]
            joint.velocity = [0.0]
        self.world.time = 0.0
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        self.world.update_kinematics()
        for history in (
            self._motor_speed_history,
            self._motor_position_history,
            self._limit_angle_history,
            self._limit_error_history,
            self._limited_acceleration_history,
            self._open_acceleration_history,
            self._force_position_gap_history,
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample(self) -> dict[str, float | str]:
        motor_position = _joint_scalar(self.motor_joint.position)
        motor_speed = _joint_scalar(self.motor_joint.velocity)
        expected_motor_speed = min(float(self.command_speed), float(self.velocity_limit))
        motor_at_limit = motor_position >= _MOTOR_TRAVEL_LIMIT - 1.0e-5
        motor_speed_error = 0.0 if motor_at_limit else abs(motor_speed - expected_motor_speed)

        limit_angle = _joint_scalar(self.limit_joint.position)
        limit_speed = _joint_scalar(self.limit_joint.velocity)
        limit_error = max(0.0, limit_angle - float(self.position_limit))

        limited_position = _joint_scalar(self.limited_force_joint.position)
        open_position = _joint_scalar(self.open_force_joint.position)
        limited_acceleration = _joint_scalar(self.limited_force_joint.acceleration)
        open_acceleration = _joint_scalar(self.open_force_joint.acceleration)
        limited_velocity = _joint_scalar(self.limited_force_joint.velocity)
        open_velocity = _joint_scalar(self.open_force_joint.velocity)

        return {
            "motor_position": motor_position,
            "motor_speed": motor_speed,
            "motor_expected_speed": expected_motor_speed,
            "motor_speed_error": motor_speed_error,
            "motor_velocity_limit": float(self.velocity_limit),
            "motor_command_speed": float(self.command_speed),
            "position_limit_angle": limit_angle,
            "position_limit_upper": float(self.position_limit),
            "position_limit_error": limit_error,
            "position_limit_speed": limit_speed,
            "limited_force_position": limited_position,
            "open_force_position": open_position,
            "force_position_gap": open_position - limited_position,
            "force_acceleration_gap": open_acceleration - limited_acceleration,
            "limited_force_acceleration": limited_acceleration,
            "open_force_acceleration": open_acceleration,
            "limited_force_velocity": limited_velocity,
            "open_force_velocity": open_velocity,
            "requested_force": float(self.force_command),
            "effort_limit": float(self.effort_limit),
            "step_ms": self._step_profile_ms(),
            "world_time": float(self.world.time),
        }

    def _record_metrics(self) -> None:
        self._last_metrics = self._sample()
        self._motor_speed_history.append(float(self._last_metrics["motor_speed"]))
        self._motor_position_history.append(float(self._last_metrics["motor_position"]))
        self._limit_angle_history.append(
            float(self._last_metrics["position_limit_angle"])
        )
        self._limit_error_history.append(
            float(self._last_metrics["position_limit_error"])
        )
        self._limited_acceleration_history.append(
            float(self._last_metrics["limited_force_acceleration"])
        )
        self._open_acceleration_history.append(
            float(self._last_metrics["open_force_acceleration"])
        )
        self._force_position_gap_history.append(
            float(self._last_metrics["force_position_gap"])
        )
        self._step_ms_history.append(float(self._last_metrics["step_ms"]))

    def _sync(self) -> None:
        self.limit_stop_frame.set_transform(_translation(self._limit_stop_position()))
        self.bridge.sync()

    def pre_step(self) -> None:
        self._apply_controls()
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        metrics = dict(self._last_metrics)
        acceleration_gaps = [
            open_accel - limited_accel
            for limited_accel, open_accel in zip(
                self._limited_acceleration_history,
                self._open_acceleration_history,
                strict=False,
            )
        ]

        def metric_value(key: str) -> float:
            return float(metrics[key])

        def max_abs(values: deque[float]) -> float:
            return max((abs(value) for value in values), default=0.0)

        return {
            "row": "rigid_joint_motor_limits",
            "comparison_axis": "world_multibody_actuator_limit_family",
            "solver": "world_multibody_joint_actuators",
            "constraint": "velocity_motor_position_limit_effort_cap",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": metric_value("world_time"),
            "held_fixed": {
                "solver": "World multibody joint actuators",
                "joint_axes": "x-axis prismatic rails and y-axis revolute stop",
                "carriage_mass": _CARRIAGE_MASS,
                "limit_link_mass": 1.0,
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "command_speed": float(self.command_speed),
                "velocity_limit": float(self.velocity_limit),
                "position_limit": float(self.position_limit),
                "force_command": float(self.force_command),
                "effort_limit": float(self.effort_limit),
            },
            "joint_lanes": [
                "velocity_motor",
                "position_limit",
                "effort_limited",
                "effort_reference",
            ],
            "joints": {
                "velocity_motor": self.motor_joint.name,
                "position_limit": self.limit_joint.name,
                "limited_force": self.limited_force_joint.name,
                "open_force": self.open_force_joint.name,
            },
            "joint_motor_speed": metric_value("motor_speed"),
            "joint_motor_expected_speed": metric_value("motor_expected_speed"),
            "joint_motor_speed_error": metric_value("motor_speed_error"),
            "joint_motor_position_limit_angle": metric_value("position_limit_angle"),
            "joint_motor_position_limit_error": metric_value("position_limit_error"),
            "joint_motor_force_position_gap": metric_value("force_position_gap"),
            "joint_motor_force_acceleration_gap": metric_value(
                "force_acceleration_gap"
            ),
            "motor_position": metric_value("motor_position"),
            "motor_speed": metric_value("motor_speed"),
            "motor_expected_speed": metric_value("motor_expected_speed"),
            "motor_speed_error": metric_value("motor_speed_error"),
            "motor_velocity_limit": metric_value("motor_velocity_limit"),
            "motor_command_speed": metric_value("motor_command_speed"),
            "position_limit_angle": metric_value("position_limit_angle"),
            "position_limit_upper": metric_value("position_limit_upper"),
            "position_limit_error": metric_value("position_limit_error"),
            "position_limit_speed": metric_value("position_limit_speed"),
            "limited_force_position": metric_value("limited_force_position"),
            "open_force_position": metric_value("open_force_position"),
            "force_position_gap": metric_value("force_position_gap"),
            "force_acceleration_gap": metric_value("force_acceleration_gap"),
            "limited_force_acceleration": metric_value("limited_force_acceleration"),
            "open_force_acceleration": metric_value("open_force_acceleration"),
            "limited_force_velocity": metric_value("limited_force_velocity"),
            "open_force_velocity": metric_value("open_force_velocity"),
            "requested_force": metric_value("requested_force"),
            "effort_limit": metric_value("effort_limit"),
            "step_ms": metric_value("step_ms"),
            "metrics": metrics,
            "history": {
                "samples": float(len(self._motor_speed_history)),
                "max_motor_speed": max(self._motor_speed_history, default=0.0),
                "max_motor_position": max(
                    self._motor_position_history, default=0.0
                ),
                "max_abs_limit_error": max_abs(self._limit_error_history),
                "max_limit_angle": max(self._limit_angle_history, default=0.0),
                "max_limited_force_acceleration": max(
                    self._limited_acceleration_history, default=0.0
                ),
                "max_open_force_acceleration": max(
                    self._open_acceleration_history, default=0.0
                ),
                "max_force_acceleration_gap": max(acceleration_gaps, default=0.0),
                "max_force_position_gap": max(
                    self._force_position_gap_history, default=0.0
                ),
                "max_step_ms": max(self._step_ms_history, default=0.0),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "command_speed": float(self.command_speed),
                "velocity_limit": float(self.velocity_limit),
                "position_limit": float(self.position_limit),
                "force_command": float(self.force_command),
                "effort_limit": float(self.effort_limit),
            },
            "motor_speed_history": list(self._motor_speed_history),
            "motor_position_history": list(self._motor_position_history),
            "limit_angle_history": list(self._limit_angle_history),
            "limit_error_history": list(self._limit_error_history),
            "limited_acceleration_history": list(self._limited_acceleration_history),
            "open_acceleration_history": list(self._open_acceleration_history),
            "force_position_gap_history": list(self._force_position_gap_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": dict(self._last_metrics),
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        force_position_gap = _last_float(snapshot.get("force_position_gap_history", []))
        if force_position_gap is not None:
            return max(0.0, force_position_gap)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                return max(0.0, float(metrics.get("force_position_gap", 0.0)))
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.05:
            return 1.0

        metrics = snapshot.get("last_metrics", {})
        if not isinstance(metrics, dict):
            metrics = {}
        controls = snapshot.get("controls", {})
        if not isinstance(controls, dict):
            controls = {}

        try:
            command_speed = float(
                controls.get(
                    "command_speed",
                    metrics.get("motor_command_speed", self.command_speed),
                )
            )
            velocity_limit = float(
                controls.get(
                    "velocity_limit",
                    metrics.get("motor_velocity_limit", self.velocity_limit),
                )
            )
            motor_speed = _last_float(snapshot.get("motor_speed_history", []))
            if motor_speed is None:
                motor_speed = float(metrics.get("motor_speed", 0.0))
            if (
                command_speed > velocity_limit + 1.0e-6
                and abs(motor_speed) >= 0.85 * velocity_limit
            ):
                return 1.0
        except (TypeError, ValueError):
            # Malformed replay controls leave this marker inactive.
            pass

        try:
            position_limit = float(
                controls.get(
                    "position_limit",
                    metrics.get("position_limit_upper", self.position_limit),
                )
            )
            limit_angle = _last_float(snapshot.get("limit_angle_history", []))
            if limit_angle is None:
                limit_angle = float(metrics.get("position_limit_angle", 0.0))
            limit_error = _last_float(snapshot.get("limit_error_history", []))
            if limit_error is None:
                limit_error = float(metrics.get("position_limit_error", 0.0))
            if limit_angle >= position_limit - 0.02 or abs(limit_error) >= 0.005:
                return 1.0
        except (TypeError, ValueError):
            # Malformed replay limit metrics leave this marker inactive.
            pass

        limited_acceleration = _last_float(
            snapshot.get("limited_acceleration_history", [])
        )
        open_acceleration = _last_float(snapshot.get("open_acceleration_history", []))
        if limited_acceleration is not None and open_acceleration is not None:
            if abs(open_acceleration - limited_acceleration) >= 0.50:
                return 1.0
        else:
            try:
                if abs(float(metrics.get("force_acceleration_gap", 0.0))) >= 0.50:
                    return 1.0
            except (TypeError, ValueError):
                # Malformed acceleration metrics leave this marker inactive.
                pass

        force_gap = _last_float(snapshot.get("force_position_gap_history", []))
        if force_gap is None:
            try:
                force_gap = float(metrics.get("force_position_gap", 0.0))
            except (TypeError, ValueError):
                force_gap = 0.0
        if force_gap >= 0.05:
            return 1.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.command_speed = float(controls.get("command_speed", self.command_speed))
        self.velocity_limit = float(controls.get("velocity_limit", self.velocity_limit))
        self.position_limit = float(controls.get("position_limit", self.position_limit))
        self.force_command = float(controls.get("force_command", self.force_command))
        self.effort_limit = float(controls.get("effort_limit", self.effort_limit))
        self._apply_controls()
        self._restore_history(
            self._motor_speed_history,
            state.get("motor_speed_history", []),
        )
        self._restore_history(
            self._motor_position_history,
            state.get("motor_position_history", []),
        )
        self._restore_history(
            self._limit_angle_history,
            state.get("limit_angle_history", []),
        )
        self._restore_history(
            self._limit_error_history,
            state.get("limit_error_history", []),
        )
        self._restore_history(
            self._limited_acceleration_history,
            state.get("limited_acceleration_history", []),
        )
        self._restore_history(
            self._open_acceleration_history,
            state.get("open_acceleration_history", []),
        )
        self._restore_history(
            self._force_position_gap_history,
            state.get("force_position_gap_history", []),
        )
        self._restore_history(
            self._step_ms_history,
            state.get("step_ms_history", []),
        )
        self._last_metrics = {
            str(key): float(value) if isinstance(value, (int, float)) else str(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self._sync()

    def _restore_history(self, history: deque[float], values: Any) -> None:
        history.clear()
        history.extend(float(value) for value in values)

    def _slider_with_reset(
        self,
        builder: Any,
        label: str,
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        changed, updated = builder.slider(label, float(value), minimum, maximum)
        if changed:
            return float(updated)
        return float(value)

    def build_panel(self, builder: Any, context: Any) -> None:
        new_command_speed = self._slider_with_reset(
            builder, "Command speed", self.command_speed, 0.05, 0.85
        )
        new_velocity_limit = self._slider_with_reset(
            builder, "Velocity limit", self.velocity_limit, 0.05, 0.50
        )
        new_position_limit = self._slider_with_reset(
            builder, "Position upper limit", self.position_limit, 0.12, 0.65
        )
        new_force_command = self._slider_with_reset(
            builder, "Requested force", self.force_command, 2.0, 22.0
        )
        new_effort_limit = self._slider_with_reset(
            builder, "Effort cap", self.effort_limit, 1.0, 9.0
        )
        controls_changed = any(
            not np.isclose(old, new)
            for old, new in (
                (self.command_speed, new_command_speed),
                (self.velocity_limit, new_velocity_limit),
                (self.position_limit, new_position_limit),
                (self.force_command, new_force_command),
                (self.effort_limit, new_effort_limit),
            )
        )
        if controls_changed:
            self.command_speed = new_command_speed
            self.velocity_limit = new_velocity_limit
            self.position_limit = new_position_limit
            self.force_command = new_force_command
            self.effort_limit = new_effort_limit
            self.reset(clear_replay=True)
        if builder.button("Reset joint verifier"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics or {}
        builder.separator()
        builder.text("comparison axis: World multibody actuator/limit family")
        builder.text(
            "held fixed: World multibody joints | x-axis prismatic rails + "
            "y-axis revolute stop | carriage mass 2.0 | time step "
            f"{_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("velocity actuator: command is clamped by joint velocity limits")
        builder.text(
            f"speed {float(metrics.get('motor_speed', 0.0)):.3f} m/s | "
            f"target {float(metrics.get('motor_expected_speed', 0.0)):.3f} m/s"
        )
        builder.text(
            f"motor position {float(metrics.get('motor_position', 0.0)):.3f} m | "
            f"speed error {float(metrics.get('motor_speed_error', 0.0)):.6f}"
        )
        builder.text("position limit: gravity-driven hinge settles at the upper stop")
        builder.text(
            f"angle {float(metrics.get('position_limit_angle', 0.0)):.3f} rad | "
            f"upper {float(metrics.get('position_limit_upper', 0.0)):.3f} rad"
        )
        builder.text(
            f"limit overshoot {float(metrics.get('position_limit_error', 0.0)):.6f} rad | "
            f"speed {float(metrics.get('position_limit_speed', 0.0)):.3f} rad/s"
        )
        builder.text("effort cap: capped and reference sliders receive the same command")
        builder.text(
            f"limited accel {float(metrics.get('limited_force_acceleration', 0.0)):.3f} m/s^2 | "
            f"reference accel {float(metrics.get('open_force_acceleration', 0.0)):.3f} m/s^2"
        )
        builder.text(
            f"force travel gap {float(metrics.get('force_position_gap', 0.0)):.3f} m | "
            f"step {float(metrics.get('step_ms', 0.0)):.3f} ms | "
            f"time {float(metrics.get('world_time', 0.0)):.3f} s"
        )
        builder.plot_lines("Motor speed", list(self._motor_speed_history))
        builder.plot_lines("Motor position", list(self._motor_position_history))
        builder.plot_lines("Position-limit angle", list(self._limit_angle_history))
        builder.plot_lines("Position-limit error", list(self._limit_error_history))
        builder.plot_lines(
            "Limited-force acceleration", list(self._limited_acceleration_history)
        )
        builder.plot_lines(
            "Reference-force acceleration", list(self._open_acceleration_history)
        )
        builder.plot_lines("Force travel gap", list(self._force_position_gap_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    verifier = _RigidJointMotorLimitVerifier()
    return SceneSetup(
        world=verifier.bridge.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.bridge.force_drag,
        renderable_provider=verifier.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Joint Motors & Limits", verifier.build_panel)],
        info={
            "sx_world": verifier.world,
            "motor_joint": verifier.motor_joint,
            "limit_joint": verifier.limit_joint,
            "limited_force_joint": verifier.limited_force_joint,
            "open_force_joint": verifier.open_force_joint,
            "rigid_joint_motor_limit_controller": verifier,
            CAPTURE_METRICS_INFO_KEY: verifier.capture_metrics,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_sync": verifier._sync,
            "replay_timeline": {
                "signal_label": "Force travel gap",
                "signal": verifier.replay_timeline_signal,
                "markers": verifier.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_joint_motor_limits",
    title="Rigid Joint Motors & Limits",
    category="World Rigid Body",
    summary="Shows World multibody velocity motors, position stops, and effort caps.",
    build=build,
)
