"""Link-origin Jacobian verifier for DART 7 World multibodies."""

from __future__ import annotations

from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_LINK_LENGTH = 0.55
_BASE_ANCHOR = np.array([-0.42, 0.0, 0.88])
_FINITE_DIFFERENCE_EPS = 1.0e-6


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
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


def _x_aligned_transform(start: np.ndarray, vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float).reshape(3)
    length = float(np.linalg.norm(vector))
    if length < 1.0e-9:
        vector = np.array([1.0e-3, 0.0, 0.0])
        length = float(np.linalg.norm(vector))
    x_axis = vector / length
    y_axis = np.array([0.0, 1.0, 0.0])
    z_axis = np.cross(x_axis, y_axis)
    z_norm = float(np.linalg.norm(z_axis))
    if z_norm < 1.0e-9:
        z_axis = np.array([0.0, 0.0, 1.0])
    else:
        z_axis /= z_norm
    y_axis = np.cross(z_axis, x_axis)
    transform = np.eye(4)
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    transform[:3, 3] = np.asarray(start, dtype=float) + 0.5 * vector
    return transform


class _RigidLinkJacobian:
    def __init__(self) -> None:
        self.motion_speed = 0.85
        self.elbow_phase = 0.72
        self.wrench_force = 1.35
        self.wrench_angle_deg = 28.0
        self.wrench_moment = 0.12

        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True
        self.robot = self.world.add_multibody("link_jacobian_arm")
        self.base = self.robot.add_link("link_jacobian_base")
        self.links: list[Any] = []
        self.joints: list[Any] = []
        self._add_arm_links()
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_link_jacobian")
        self.link_frames: list[Any] = []
        self.base_marker = self._add_frame(
            "link_jacobian_base_marker",
            dart.SphereShape(0.050),
            _BASE_ANCHOR,
            (0.30, 0.32, 0.36),
        )
        self.origin_marker = self._add_frame(
            "link_jacobian_origin_marker",
            dart.SphereShape(0.043),
            _BASE_ANCHOR,
            (0.92, 0.80, 0.24),
        )
        self.velocity_arrow = self._add_frame(
            "link_jacobian_velocity_arrow",
            _box((0.20, 0.020, 0.020)),
            _BASE_ANCHOR,
            (0.18, 0.62, 0.88),
        )
        self.force_arrow = self._add_frame(
            "link_jacobian_force_arrow",
            _box((0.20, 0.024, 0.024)),
            _BASE_ANCHOR,
            (0.92, 0.42, 0.24),
        )
        self._add_link_visuals()

        self._speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._fd_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._power_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._tau0_history: deque[float] = deque(maxlen=_HISTORY)
        self._tau1_history: deque[float] = deque(maxlen=_HISTORY)
        self._world_body_gap_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self._last_velocity_vector = np.zeros(3)
        self._last_force_vector = np.array([1.0, 0.0, 0.0])
        self.reset(clear_replay=True)

    def _add_arm_links(self) -> None:
        parent = self.base
        for index in range(2):
            offset = _translation((_LINK_LENGTH, 0.0, 0.0))
            link = self.robot.add_link(
                f"link_jacobian_link{index}",
                parent=parent,
                joint=sx.JointSpec(
                    name=f"link_jacobian_joint{index}",
                    type=sx.JointType.REVOLUTE,
                    axis=(0.0, 1.0, 0.0),
                    transform_from_parent=offset,
                ),
            )
            link.mass = 0.85 if index == 0 else 0.60
            transverse = link.mass * (_LINK_LENGTH**2) / 12.0
            link.inertia = (
                (0.002, 0.0, 0.0),
                (0.0, transverse, 0.0),
                (0.0, 0.0, transverse),
            )
            self.links.append(link)
            self.joints.append(link.parent_joint)
            parent = link

    def _add_frame(
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

    def _add_link_visuals(self) -> None:
        colors = ((0.28, 0.56, 0.86), (0.55, 0.48, 0.82))
        for index, _link in enumerate(self.links):
            frame = self._add_frame(
                f"link_jacobian_link{index}_visual",
                _box((_LINK_LENGTH, 0.060, 0.060)),
                _BASE_ANCHOR,
                colors[index],
            )
            self.link_frames.append(frame)

    def _clamp_controls(self) -> None:
        self.motion_speed = float(np.clip(self.motion_speed, 0.10, 2.40))
        self.elbow_phase = float(np.clip(self.elbow_phase, -3.14, 3.14))
        self.wrench_force = float(np.clip(self.wrench_force, 0.0, 3.0))
        self.wrench_angle_deg = float(np.clip(self.wrench_angle_deg, -120.0, 120.0))
        self.wrench_moment = float(np.clip(self.wrench_moment, -0.60, 0.60))

    def _motion_state(self, time_s: float) -> tuple[np.ndarray, np.ndarray]:
        speed = float(self.motion_speed)
        phase = float(self.elbow_phase)
        q = np.array(
            [
                0.22 + 0.52 * np.sin(speed * time_s),
                -0.68 + 0.44 * np.sin(1.25 * speed * time_s + phase),
            ],
            dtype=float,
        )
        qdot = np.array(
            [
                0.52 * speed * np.cos(speed * time_s),
                0.44 * 1.25 * speed * np.cos(1.25 * speed * time_s + phase),
            ],
            dtype=float,
        )
        return q, qdot

    def _set_joint_state(self, q: np.ndarray, qdot: np.ndarray) -> None:
        for joint, position, velocity in zip(self.joints, q, qdot, strict=True):
            joint.position = [float(position)]
            joint.velocity = [float(velocity)]
            joint.force = [0.0]
        self.world.update_kinematics()

    def _link_origin(self) -> np.ndarray:
        return np.asarray(self.links[-1].transform, dtype=float)[:3, 3].copy()

    def _finite_difference_velocity(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        origin = self._link_origin()
        self._set_joint_state(q + qdot * _FINITE_DIFFERENCE_EPS, qdot)
        next_origin = self._link_origin()
        self._set_joint_state(q, qdot)
        return (next_origin - origin) / _FINITE_DIFFERENCE_EPS

    def _wrench(self) -> np.ndarray:
        angle = np.deg2rad(float(self.wrench_angle_deg))
        force = float(self.wrench_force) * np.array(
            [np.cos(angle), 0.0, np.sin(angle)],
            dtype=float,
        )
        moment = np.array([0.0, float(self.wrench_moment), 0.0], dtype=float)
        return np.concatenate([moment, force])

    def _record_metrics(self) -> None:
        q, qdot = self._motion_state(float(self.world.time))
        self._set_joint_state(q, qdot)
        world_jacobian = np.asarray(self.robot.get_world_jacobian(self.links[-1]), dtype=float)
        body_jacobian = np.asarray(self.robot.get_jacobian(self.links[-1]), dtype=float)
        twist = world_jacobian @ qdot
        finite_difference_velocity = self._finite_difference_velocity(q, qdot)
        wrench = self._wrench()
        tau = world_jacobian.T @ wrench
        linear_velocity = twist[3:6]
        finite_difference_error = float(
            np.linalg.norm(finite_difference_velocity - linear_velocity)
        )
        joint_power = float(np.dot(tau, qdot))
        wrench_power = float(np.dot(wrench, twist))
        power_error = abs(joint_power - wrench_power)
        world_body_gap = float(np.linalg.norm(world_jacobian - body_jacobian))
        linear_speed = float(np.linalg.norm(linear_velocity))
        angular_speed = float(np.linalg.norm(twist[:3]))
        self._last_velocity_vector = linear_velocity.copy()
        self._last_force_vector = wrench[3:6].copy()
        self._last_metrics = {
            "status": "link-origin Jacobian and wrench map",
            "dofs": float(world_jacobian.shape[1]),
            "rows": float(world_jacobian.shape[0]),
            "q0": float(q[0]),
            "q1": float(q[1]),
            "qdot0": float(qdot[0]),
            "qdot1": float(qdot[1]),
            "linear_speed": linear_speed,
            "angular_speed": angular_speed,
            "linear_vx": float(linear_velocity[0]),
            "linear_vz": float(linear_velocity[2]),
            "finite_difference_error": finite_difference_error,
            "tau0": float(tau[0]),
            "tau1": float(tau[1]),
            "joint_power": joint_power,
            "wrench_power": wrench_power,
            "power_error": power_error,
            "world_body_gap": world_body_gap,
            "force_norm": float(np.linalg.norm(wrench[3:6])),
            "moment_y": float(wrench[1]),
        }
        self._speed_history.append(linear_speed)
        self._fd_error_history.append(finite_difference_error)
        self._power_error_history.append(power_error)
        self._tau0_history.append(float(tau[0]))
        self._tau1_history.append(float(tau[1]))
        self._world_body_gap_history.append(world_body_gap)

    def reset(self, *, clear_replay: bool = False) -> None:
        self._clamp_controls()
        self.world.time = 0.0
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            self._speed_history,
            self._fd_error_history,
            self._power_error_history,
            self._tau0_history,
            self._tau1_history,
            self._world_body_gap_history,
        ):
            history.clear()
        self._record_metrics()
        self._sync()

    def pre_step(self) -> None:
        self.world.time = float(self.world.time) + _TIME_STEP
        self._record_metrics()
        self._sync()

    def _set_arrow(
        self,
        frame: Any,
        start: np.ndarray,
        vector: np.ndarray,
        *,
        min_length: float = 0.030,
    ) -> None:
        length = max(float(np.linalg.norm(vector)), min_length)
        frame.set_shape(_box((length, 0.022, 0.022)))
        frame.set_transform(_x_aligned_transform(start, vector))

    def _sync(self) -> None:
        render_offset = _translation(_BASE_ANCHOR)
        for link, frame in zip(self.links, self.link_frames, strict=True):
            transform = (
                render_offset
                @ np.asarray(link.transform, dtype=float)
                @ _translation((-0.5 * _LINK_LENGTH, 0.0, 0.0))
            )
            frame.set_transform(transform)
        origin = _BASE_ANCHOR + self._link_origin()
        self.origin_marker.set_transform(_translation(origin))
        velocity_vector = 0.42 * self._last_velocity_vector
        force_vector = 0.18 * self._last_force_vector
        self._set_arrow(self.velocity_arrow, origin, velocity_vector)
        self._set_arrow(self.force_arrow, origin + np.array([0.0, 0.0, 0.07]), force_vector)
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        self._sync()
        return self.bridge.renderable_provider()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()

        def metric_value(key: str) -> float:
            return float(self._last_metrics[key])

        def max_abs(values: deque[float]) -> float:
            return max((abs(value) for value in values), default=0.0)

        link_origin = _BASE_ANCHOR + self._link_origin()
        metrics: dict[str, float | str] = {}
        for key, value in self._last_metrics.items():
            if isinstance(value, (int, float, np.floating)):
                metrics[key] = float(value)
            else:
                metrics[key] = str(value)
        jacobian_terms = [
            "world_jacobian_twist",
            "finite_difference_velocity",
            "jacobian_transpose_wrench",
            "world_body_jacobian_gap",
        ]

        payload: dict[str, Any] = {
            "row": "rigid_link_jacobian",
            "comparison_axis": "link_origin_jacobian_mapping_family",
            "solver": "world_multibody_link_jacobian",
            "scope": "contact_free_link_origin_jacobian_wrench_map",
            "held_fixed": {
                "solver": "world_multibody_link_jacobian",
                "contacts": "off",
                "gravity": "off",
                "joint_family": "two_revolute_links",
                "link_length": _LINK_LENGTH,
                "finite_difference_eps": _FINITE_DIFFERENCE_EPS,
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "motion_speed": float(self.motion_speed),
            "elbow_phase": float(self.elbow_phase),
            "wrench_force": float(self.wrench_force),
            "wrench_angle_deg": float(self.wrench_angle_deg),
            "wrench_moment": float(self.wrench_moment),
            "controls": {
                "motion_speed": float(self.motion_speed),
                "elbow_phase": float(self.elbow_phase),
                "wrench_force": float(self.wrench_force),
                "wrench_angle_deg": float(self.wrench_angle_deg),
                "wrench_moment": float(self.wrench_moment),
            },
            "jacobian_terms": jacobian_terms,
            "joint_names": [joint.name for joint in self.joints],
            "link": self.links[-1].name,
            "metrics": metrics,
            "link_jacobian_linear_speed": metric_value("linear_speed"),
            "link_jacobian_angular_speed": metric_value("angular_speed"),
            "link_jacobian_world_body_gap": metric_value("world_body_gap"),
            "link_jacobian_finite_difference_error": metric_value(
                "finite_difference_error"
            ),
            "link_jacobian_tau0": metric_value("tau0"),
            "link_jacobian_tau1": metric_value("tau1"),
            "link_jacobian_power_error": metric_value("power_error"),
            "link_origin_world_x": float(link_origin[0]),
            "link_origin_world_y": float(link_origin[1]),
            "link_origin_world_z": float(link_origin[2]),
            "linear_velocity_x": float(self._last_velocity_vector[0]),
            "linear_velocity_y": float(self._last_velocity_vector[1]),
            "linear_velocity_z": float(self._last_velocity_vector[2]),
            "wrench_force_x": float(self._last_force_vector[0]),
            "wrench_force_y": float(self._last_force_vector[1]),
            "wrench_force_z": float(self._last_force_vector[2]),
            "rows": metric_value("rows"),
            "dofs": metric_value("dofs"),
            "q0": metric_value("q0"),
            "q1": metric_value("q1"),
            "qdot0": metric_value("qdot0"),
            "qdot1": metric_value("qdot1"),
            "linear_speed": metric_value("linear_speed"),
            "angular_speed": metric_value("angular_speed"),
            "finite_difference_error": metric_value("finite_difference_error"),
            "world_body_gap": metric_value("world_body_gap"),
            "tau0": metric_value("tau0"),
            "tau1": metric_value("tau1"),
            "joint_power": metric_value("joint_power"),
            "wrench_power": metric_value("wrench_power"),
            "power_error": metric_value("power_error"),
            "force_norm": metric_value("force_norm"),
            "moment_y": metric_value("moment_y"),
            "history": {
                "samples": float(len(self._speed_history)),
                "max_linear_speed": max(self._speed_history, default=0.0),
                "max_finite_difference_error": max(
                    self._fd_error_history, default=0.0
                ),
                "max_power_error": max(self._power_error_history, default=0.0),
                "max_abs_tau0": max_abs(self._tau0_history),
                "max_abs_tau1": max_abs(self._tau1_history),
                "max_world_body_gap": max(
                    self._world_body_gap_history, default=0.0
                ),
            },
        }
        for key, value in self._last_metrics.items():
            if isinstance(value, (int, float, np.floating)):
                payload[key] = float(value)
        return payload

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "motion_speed": float(self.motion_speed),
                "elbow_phase": float(self.elbow_phase),
                "wrench_force": float(self.wrench_force),
                "wrench_angle_deg": float(self.wrench_angle_deg),
                "wrench_moment": float(self.wrench_moment),
            },
            "state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "time": float(self.world.time),
            "speed_history": list(self._speed_history),
            "fd_error_history": list(self._fd_error_history),
            "power_error_history": list(self._power_error_history),
            "tau0_history": list(self._tau0_history),
            "tau1_history": list(self._tau1_history),
            "world_body_gap_history": list(self._world_body_gap_history),
            "last_metrics": dict(self._last_metrics),
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        speed = _last_float(snapshot.get("speed_history", []))
        if speed is not None:
            return max(0.0, speed)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                return max(0.0, float(metrics.get("linear_speed", 0.0)))
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.75:
            return 1.0

        tau0 = _last_float(snapshot.get("tau0_history", []))
        tau1 = _last_float(snapshot.get("tau1_history", []))
        if tau0 is not None or tau1 is not None:
            if max(abs(tau0 or 0.0), abs(tau1 or 0.0)) >= 0.50:
                return 1.0

        gap = _last_float(snapshot.get("world_body_gap_history", []))
        if gap is not None and gap >= 0.10:
            return 1.0

        fd_error = _last_float(snapshot.get("fd_error_history", []))
        if fd_error is not None and fd_error >= 1.0e-6:
            return 1.0
        power_error = _last_float(snapshot.get("power_error_history", []))
        if power_error is not None and power_error >= 1.0e-9:
            return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                if float(metrics.get("linear_speed", 0.0)) >= 0.75:
                    return 1.0
                if (
                    max(
                        abs(float(metrics.get("tau0", 0.0))),
                        abs(float(metrics.get("tau1", 0.0))),
                    )
                    >= 0.50
                ):
                    return 1.0
                if float(metrics.get("world_body_gap", 0.0)) >= 0.10:
                    return 1.0
                if float(metrics.get("finite_difference_error", 0.0)) >= 1.0e-6:
                    return 1.0
                if float(metrics.get("power_error", 0.0)) >= 1.0e-9:
                    return 1.0
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def restore_replay_state(self, snapshot: dict[str, Any]) -> None:
        controls = snapshot.get("controls", {})
        self.motion_speed = float(controls.get("motion_speed", self.motion_speed))
        self.elbow_phase = float(controls.get("elbow_phase", self.elbow_phase))
        self.wrench_force = float(controls.get("wrench_force", self.wrench_force))
        self.wrench_angle_deg = float(
            controls.get("wrench_angle_deg", self.wrench_angle_deg)
        )
        self.wrench_moment = float(controls.get("wrench_moment", self.wrench_moment))
        self._clamp_controls()
        try:
            self.world.state_vector = np.asarray(snapshot["state"], dtype=float)
            self.world.time = float(snapshot.get("time", self.world.time))
        except Exception:  # noqa: BLE001
            pass
        self._restore_history(self._speed_history, snapshot.get("speed_history", []))
        self._restore_history(self._fd_error_history, snapshot.get("fd_error_history", []))
        self._restore_history(
            self._power_error_history, snapshot.get("power_error_history", [])
        )
        self._restore_history(self._tau0_history, snapshot.get("tau0_history", []))
        self._restore_history(self._tau1_history, snapshot.get("tau1_history", []))
        self._restore_history(
            self._world_body_gap_history, snapshot.get("world_body_gap_history", [])
        )
        self._last_metrics = dict(snapshot.get("last_metrics", {}))
        self._record_metrics()
        self._sync()

    def _restore_history(self, history: deque[float], values: list[float]) -> None:
        history.clear()
        history.extend(float(value) for value in values)

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_speed, motion_speed = builder.slider(
            "Motion speed", float(self.motion_speed), 0.10, 2.40
        )
        changed_phase, elbow_phase = builder.slider(
            "Elbow phase", float(self.elbow_phase), -3.14, 3.14
        )
        changed_force, wrench_force = builder.slider(
            "Wrench force", float(self.wrench_force), 0.0, 3.0
        )
        changed_angle, wrench_angle = builder.slider(
            "Wrench angle", float(self.wrench_angle_deg), -120.0, 120.0
        )
        changed_moment, wrench_moment = builder.slider(
            "Wrench moment", float(self.wrench_moment), -0.60, 0.60
        )
        if changed_speed:
            self.motion_speed = float(motion_speed)
        if changed_phase:
            self.elbow_phase = float(elbow_phase)
        if changed_force:
            self.wrench_force = float(wrench_force)
        if changed_angle:
            self.wrench_angle_deg = float(wrench_angle)
        if changed_moment:
            self.wrench_moment = float(wrench_moment)
        if changed_speed or changed_phase or changed_force or changed_angle or changed_moment:
            self.reset(clear_replay=True)

        if builder.button("Reset Jacobian path"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics
        builder.separator()
        builder.text("comparison axis: link-origin Jacobian mapping family")
        builder.text(
            "held fixed: World multibody link Jacobian | contacts off | gravity off | "
            f"two revolute links | link length {_LINK_LENGTH:.2f} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("mapping: link-origin world Jacobian | contacts: off")
        builder.text("twist = J_world(q) qdot; tau = J_world(q)^T wrench")
        builder.text(
            f"q [{float(metrics['q0']):.3f}, {float(metrics['q1']):.3f}] | "
            f"qdot [{float(metrics['qdot0']):.3f}, {float(metrics['qdot1']):.3f}]"
        )
        builder.text(
            f"linear speed {float(metrics['linear_speed']):.3f} m/s | "
            f"fd error {float(metrics['finite_difference_error']):.2e} | "
            f"world/body J gap {float(metrics['world_body_gap']):.3f}"
        )
        builder.text(
            f"tau [{float(metrics['tau0']):.3f}, {float(metrics['tau1']):.3f}] | "
            f"power error {float(metrics['power_error']):.2e} | "
            f"force {float(metrics['force_norm']):.2f} N"
        )
        builder.plot_lines("Link-origin speed", list(self._speed_history))
        builder.plot_lines("Finite-difference error", list(self._fd_error_history))
        builder.plot_lines("Power error", list(self._power_error_history))
        builder.plot_lines("Shoulder torque", list(self._tau0_history))
        builder.plot_lines("Elbow torque", list(self._tau1_history))
        builder.plot_lines("World/body Jacobian gap", list(self._world_body_gap_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidLinkJacobian()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Link Jacobian", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_link_jacobian_controller": controller,
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Link-origin speed",
                "signal": controller.replay_timeline_signal,
                "markers": controller.replay_timeline_marker,
            },
            "replay_sync": controller._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_link_jacobian",
    title="Rigid Link Jacobian",
    category="World Rigid Body",
    summary=(
        "Shows link-origin world Jacobian velocity mapping and transpose wrench "
        "mapping for a contact-free two-link multibody."
    ),
    build=build,
)
