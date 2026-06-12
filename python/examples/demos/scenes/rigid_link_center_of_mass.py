"""Link center-of-mass offset verifier for DART 7 World multibodies."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.003
_HISTORY = 180
_GRAVITY = 9.81
_BASE_INERTIA_Y = 0.12
_LINK_SIZE = np.array([0.62, 0.12, 0.10])
_COM_MARKER_RADIUS = 0.035
_HINGE_MARKER_RADIUS = 0.024


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _box(size: np.ndarray | tuple[float, float, float]) -> dart.BoxShape:
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
class _ComLane:
    key: str
    label: str
    offset_multiplier: float
    high_inertia: bool
    anchor: np.ndarray
    color: tuple[float, float, float]
    robot: Any
    link: Any
    joint: Any
    body_frame: Any | None = None
    com_frame: Any | None = None
    hinge_frame: Any | None = None


class _RigidLinkCenterOfMass:
    def __init__(self) -> None:
        self.executor_index = 0
        self.com_offset = 0.18
        self.gravity_scale = 1.0
        self.link_mass = 2.0
        self.inertia_scale = 4.0
        self._executors: list[tuple[str, Any]] = [("Sequential", sx.SequentialExecutor())]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(time_step=_TIME_STEP)
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._add_lane(
                "centered",
                "Centered COM",
                0.0,
                False,
                (-1.35, 0.34, 1.0),
                (0.34, 0.62, 0.90),
            ),
            self._add_lane(
                "positive",
                "+X COM offset",
                1.0,
                False,
                (-0.45, 0.34, 1.0),
                (0.88, 0.50, 0.26),
            ),
            self._add_lane(
                "negative",
                "-X COM offset",
                -1.0,
                False,
                (0.45, 0.34, 1.0),
                (0.32, 0.72, 0.42),
            ),
            self._add_lane(
                "high_inertia",
                "+X COM, high inertia",
                1.0,
                True,
                (1.35, 0.34, 1.0),
                (0.70, 0.52, 0.86),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_link_center_of_mass")
        self._add_visuals()

        self._angle_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_error_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._torque_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._energy_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self.reset(clear_replay=True)

    def _add_lane(
        self,
        key: str,
        label: str,
        offset_multiplier: float,
        high_inertia: bool,
        anchor: tuple[float, float, float],
        color: tuple[float, float, float],
    ) -> _ComLane:
        robot = self.world.add_multibody(f"{key}_com_offset_robot")
        base = robot.add_link(f"{key}_base")
        link = robot.add_link(
            f"{key}_link",
            parent=base,
            joint=sx.JointSpec(
                name=f"{key}_hinge",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
            ),
        )
        return _ComLane(
            key=key,
            label=label,
            offset_multiplier=float(offset_multiplier),
            high_inertia=bool(high_inertia),
            anchor=np.asarray(anchor, dtype=float),
            color=color,
            robot=robot,
            link=link,
            joint=link.parent_joint,
        )

    def _add_static_visual(
        self,
        name: str,
        shape: Any,
        position: np.ndarray,
        color: tuple[float, float, float],
    ) -> Any:
        frame = dart.SimpleFrame(
            dart.gui.world_render_frame(), name, _translation(position)
        )
        frame.set_shape(shape)
        frame.create_visual_aspect().set_color(list(color))
        self.bridge.render_world.add_simple_frame(frame)
        return frame

    def _add_visuals(self) -> None:
        for lane in self.lanes:
            self._add_static_visual(
                f"{lane.key}_base_plate",
                _box((0.20, 0.18, 0.05)),
                lane.anchor + np.array([0.0, 0.0, -0.15]),
                (0.30, 0.32, 0.36),
            )
            lane.body_frame = self._add_static_visual(
                f"{lane.key}_link_visual",
                _box(_LINK_SIZE),
                lane.anchor,
                lane.color,
            )
            lane.com_frame = self._add_static_visual(
                f"{lane.key}_com_marker",
                dart.SphereShape(_COM_MARKER_RADIUS),
                lane.anchor,
                (0.98, 0.86, 0.24),
            )
            lane.hinge_frame = self._add_static_visual(
                f"{lane.key}_hinge_marker",
                dart.SphereShape(_HINGE_MARKER_RADIUS),
                lane.anchor,
                (0.12, 0.14, 0.18),
            )
        self._sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _lane_offset(self, lane: _ComLane) -> float:
        return float(lane.offset_multiplier) * float(self.com_offset)

    def _lane_inertia_y(self, lane: _ComLane) -> float:
        scale = float(self.inertia_scale) if lane.high_inertia else 1.0
        return _BASE_INERTIA_Y * scale

    def _clamp_controls(self) -> None:
        self.executor_index = max(
            0, min(int(self.executor_index), len(self._executors) - 1)
        )
        self.com_offset = float(np.clip(self.com_offset, 0.0, 0.32))
        self.gravity_scale = float(np.clip(self.gravity_scale, 0.0, 1.5))
        self.link_mass = float(np.clip(self.link_mass, 0.5, 5.0))
        self.inertia_scale = float(np.clip(self.inertia_scale, 1.0, 8.0))

    def _apply_parameters(self, *, reset_state: bool) -> None:
        self._clamp_controls()
        self.world.gravity = (0.0, 0.0, -_GRAVITY * float(self.gravity_scale))
        for lane in self.lanes:
            inertia_y = self._lane_inertia_y(lane)
            lane.link.mass = float(self.link_mass)
            lane.link.inertia = (
                (0.035, 0.0, 0.0),
                (0.0, inertia_y, 0.0),
                (0.0, 0.0, 0.035),
            )
            lane.link.center_of_mass = (self._lane_offset(lane), 0.0, 0.0)
            if reset_state:
                lane.joint.position = [0.0]
                lane.joint.velocity = [0.0]
                lane.joint.force = [0.0]
        self.world.update_kinematics()

    def reset(self, *, clear_replay: bool = False) -> None:
        self._apply_parameters(reset_state=True)
        self.world.time = 0.0
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            *self._angle_history.values(),
            *self._accel_history.values(),
            *self._accel_error_history.values(),
            *self._torque_history.values(),
            *self._energy_history.values(),
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

    def _sample(self, lane: _ComLane) -> dict[str, float | str]:
        angle = _joint_scalar(lane.joint.position)
        velocity = _joint_scalar(lane.joint.velocity)
        acceleration = _joint_scalar(lane.joint.acceleration)
        offset = self._lane_offset(lane)
        mass = float(self.link_mass)
        inertia_y = self._lane_inertia_y(lane)
        expected_mass_matrix = inertia_y + mass * offset * offset
        try:
            mass_matrix = float(np.asarray(lane.robot.mass_matrix, dtype=float)[0, 0])
        except Exception:  # noqa: BLE001
            mass_matrix = expected_mass_matrix
        try:
            gravity_force = float(
                np.asarray(lane.robot.gravity_forces, dtype=float).reshape(-1)[0]
            )
        except Exception:  # noqa: BLE001
            gravity_force = -(
                mass
                * _GRAVITY
                * float(self.gravity_scale)
                * offset
                * float(np.cos(angle))
            )
        gravity_torque = -gravity_force
        expected_acceleration = (
            gravity_torque / mass_matrix if abs(mass_matrix) > 1.0e-12 else 0.0
        )
        local_com = np.array([offset, 0.0, 0.0])
        transform = np.asarray(lane.link.transform, dtype=float).reshape(4, 4)
        world_com = lane.anchor + transform[:3, :3] @ local_com + transform[:3, 3]
        kinetic = 0.5 * mass_matrix * velocity * velocity
        potential = mass * _GRAVITY * float(self.gravity_scale) * (
            float(world_com[2]) - float(lane.anchor[2])
        )
        return {
            "status": self._lane_status(lane),
            "offset": offset,
            "inertia_y": inertia_y,
            "mass_matrix": mass_matrix,
            "expected_mass_matrix": expected_mass_matrix,
            "gravity_torque": gravity_torque,
            "gravity_force": gravity_force,
            "angle": angle,
            "velocity": velocity,
            "acceleration": acceleration,
            "expected_acceleration": expected_acceleration,
            "acceleration_error": acceleration - expected_acceleration,
            "com_world_x": float(world_com[0]),
            "com_world_z": float(world_com[2]),
            "energy": kinetic + potential,
        }

    def _lane_status(self, lane: _ComLane) -> str:
        if lane.key == "centered":
            return "COM at hinge: no gravity torque"
        if lane.key == "negative":
            return "negative COM flips torque sign"
        if lane.key == "high_inertia":
            return "same offset with larger inertia"
        return "positive COM creates gravity torque"

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._angle_history[lane.key].append(float(metrics["angle"]))
            self._accel_history[lane.key].append(float(metrics["acceleration"]))
            self._accel_error_history[lane.key].append(
                float(metrics["acceleration_error"])
            )
            self._torque_history[lane.key].append(float(metrics["gravity_torque"]))
            self._energy_history[lane.key].append(float(metrics["energy"]))
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        self.world.step(self._executor())
        self._record_metrics()
        self._sync()

    def _sync(self) -> None:
        for lane in self.lanes:
            lane_transform = _translation(lane.anchor) @ np.asarray(
                lane.link.transform, dtype=float
            )
            if lane.body_frame is not None:
                lane.body_frame.set_transform(lane_transform)
            if lane.com_frame is not None:
                lane.com_frame.set_transform(
                    lane_transform @ _translation((self._lane_offset(lane), 0.0, 0.0))
                )
            if lane.hinge_frame is not None:
                lane.hinge_frame.set_transform(lane_transform)
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        self._sync()
        return self.bridge.renderable_provider()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        executor_index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = executor_index

        def serialized_metrics(lane_key: str) -> dict[str, float | str]:
            serialized: dict[str, float | str] = {}
            for key, value in self._last_metrics[lane_key].items():
                if isinstance(value, (int, float, np.floating)):
                    serialized[key] = float(value)
                else:
                    serialized[key] = str(value)
            return serialized

        def lane_value(lane_key: str, metric_key: str) -> float:
            return float(self._last_metrics[lane_key][metric_key])

        def max_abs(values: deque[float]) -> float:
            return max((abs(value) for value in values), default=0.0)

        def max_abs_after_initial(values: deque[float]) -> float:
            samples = list(values)[1:] if len(values) > 1 else list(values)
            return max((abs(value) for value in samples), default=0.0)

        lanes = {
            lane.key: {
                "label": lane.label,
                "offset_multiplier": float(lane.offset_multiplier),
                "high_inertia": bool(lane.high_inertia),
                "link": lane.link.name,
                "joint": lane.joint.name,
                "local_center_of_mass": [float(self._lane_offset(lane)), 0.0, 0.0],
                "metrics": serialized_metrics(lane.key),
            }
            for lane in self.lanes
        }
        positive_accel = lane_value("positive", "acceleration")
        positive_expected = lane_value("positive", "expected_acceleration")
        negative_accel = lane_value("negative", "acceleration")
        high_accel = lane_value("high_inertia", "acceleration")
        positive_mass_matrix = lane_value("positive", "mass_matrix")
        high_mass_matrix = lane_value("high_inertia", "mass_matrix")
        centered_torque = lane_value("centered", "gravity_torque")
        if abs(centered_torque) < 1.0e-12:
            centered_torque = 0.0
        positive_torque = lane_value("positive", "gravity_torque")
        negative_torque = lane_value("negative", "gravity_torque")
        high_mass_matrix_ratio = high_mass_matrix / max(positive_mass_matrix, 1.0e-12)
        positive_negative_angle_sum = lane_value("positive", "angle") + lane_value(
            "negative", "angle"
        )
        high_acceleration_ratio = high_accel / max(positive_accel, 1.0e-12)
        max_abs_acceleration_error = max(
            abs(lane_value(lane.key, "acceleration_error")) for lane in self.lanes
        )
        lane_order = [lane.key for lane in self.lanes]
        payload: dict[str, Any] = {
            "row": "rigid_link_center_of_mass",
            "comparison_axis": "link_center_of_mass_offset_family",
            "solver": "world_multibody_inertial_offsets",
            "scope": "contact_free_link_center_of_mass_offsets",
            "executor": self._executors[executor_index][0],
            "held_fixed": {
                "solver": "world_multibody_inertial_offsets",
                "contacts": "off",
                "joint_type": "revolute",
                "visual_geometry": "fixed",
                "link_mass": float(self.link_mass),
                "gravity_scale": float(self.gravity_scale),
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "com_offset": float(self.com_offset),
            "gravity_scale": float(self.gravity_scale),
            "link_mass": float(self.link_mass),
            "inertia_scale": float(self.inertia_scale),
            "controls": {
                "executor_index": float(executor_index),
                "com_offset": float(self.com_offset),
                "gravity_scale": float(self.gravity_scale),
                "link_mass": float(self.link_mass),
                "inertia_scale": float(self.inertia_scale),
            },
            "com_lanes": lane_order,
            "lane_order": lane_order,
            "lane_count": float(len(self.lanes)),
            "lanes": lanes,
            "link_com_centered_gravity_torque": centered_torque,
            "link_com_positive_gravity_torque": positive_torque,
            "link_com_negative_gravity_torque": negative_torque,
            "link_com_positive_negative_angle_sum": positive_negative_angle_sum,
            "link_com_high_mass_matrix_ratio": high_mass_matrix_ratio,
            "link_com_high_acceleration_ratio": high_acceleration_ratio,
            "link_com_max_acceleration_error": max_abs_acceleration_error,
            "centered_gravity_torque": centered_torque,
            "centered_angle": lane_value("centered", "angle"),
            "centered_acceleration": lane_value("centered", "acceleration"),
            "positive_offset": lane_value("positive", "offset"),
            "negative_offset": lane_value("negative", "offset"),
            "high_inertia_offset": lane_value("high_inertia", "offset"),
            "positive_gravity_torque": positive_torque,
            "negative_gravity_torque": negative_torque,
            "high_inertia_gravity_torque": lane_value(
                "high_inertia", "gravity_torque"
            ),
            "positive_mass_matrix": positive_mass_matrix,
            "negative_mass_matrix": lane_value("negative", "mass_matrix"),
            "high_inertia_mass_matrix": high_mass_matrix,
            "high_to_positive_mass_matrix_ratio": high_mass_matrix_ratio,
            "positive_angle": lane_value("positive", "angle"),
            "negative_angle": lane_value("negative", "angle"),
            "high_inertia_angle": lane_value("high_inertia", "angle"),
            "positive_negative_angle_sum": positive_negative_angle_sum,
            "positive_acceleration": positive_accel,
            "positive_expected_acceleration": positive_expected,
            "positive_acceleration_error": lane_value(
                "positive", "acceleration_error"
            ),
            "negative_acceleration": negative_accel,
            "negative_expected_acceleration": lane_value(
                "negative", "expected_acceleration"
            ),
            "negative_acceleration_error": lane_value(
                "negative", "acceleration_error"
            ),
            "high_inertia_acceleration": high_accel,
            "high_inertia_expected_acceleration": lane_value(
                "high_inertia", "expected_acceleration"
            ),
            "high_inertia_acceleration_error": lane_value(
                "high_inertia", "acceleration_error"
            ),
            "negative_to_positive_acceleration_ratio": negative_accel
            / max(abs(positive_accel), 1.0e-12),
            "positive_negative_acceleration_sum": positive_accel + negative_accel,
            "high_to_positive_acceleration_ratio": high_acceleration_ratio,
            "positive_negative_torque_sum": positive_torque + negative_torque,
            "positive_com_world_x": lane_value("positive", "com_world_x"),
            "negative_com_world_x": lane_value("negative", "com_world_x"),
            "positive_com_world_z": lane_value("positive", "com_world_z"),
            "high_inertia_com_world_z": lane_value(
                "high_inertia", "com_world_z"
            ),
            "positive_energy": lane_value("positive", "energy"),
            "high_inertia_energy": lane_value("high_inertia", "energy"),
            "max_abs_acceleration_error": max_abs_acceleration_error,
            "step_ms": self._step_ms_history[-1] if self._step_ms_history else 0.0,
            "max_step_ms": max(self._step_ms_history, default=0.0),
            "history": {
                "samples": float(len(self._step_ms_history)),
                "max_step_ms": max(self._step_ms_history, default=0.0),
            },
        }
        for lane in self.lanes:
            lane_metrics = self._last_metrics[lane.key]
            for metric_key in (
                "offset",
                "inertia_y",
                "mass_matrix",
                "expected_mass_matrix",
                "gravity_torque",
                "gravity_force",
                "angle",
                "velocity",
                "acceleration",
                "expected_acceleration",
                "acceleration_error",
                "com_world_x",
                "com_world_z",
                "energy",
            ):
                payload[f"{lane.key}_{metric_key}"] = float(lane_metrics[metric_key])
            payload["history"][f"{lane.key}_max_abs_angle"] = max_abs(
                self._angle_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_abs_acceleration"] = max_abs(
                self._accel_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_abs_acceleration_error"] = (
                max_abs_after_initial(self._accel_error_history[lane.key])
            )
            payload["history"][f"{lane.key}_max_abs_gravity_torque"] = max_abs(
                self._torque_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_abs_energy"] = max_abs(
                self._energy_history[lane.key]
            )
        return payload

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "com_offset": float(self.com_offset),
                "gravity_scale": float(self.gravity_scale),
                "link_mass": float(self.link_mass),
                "inertia_scale": float(self.inertia_scale),
            },
            "state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "time": float(self.world.time),
            "angle_history": {
                key: list(values) for key, values in self._angle_history.items()
            },
            "accel_history": {
                key: list(values) for key, values in self._accel_history.items()
            },
            "accel_error_history": {
                key: list(values) for key, values in self._accel_error_history.items()
            },
            "torque_history": {
                key: list(values) for key, values in self._torque_history.items()
            },
            "energy_history": {
                key: list(values) for key, values in self._energy_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        angles = snapshot.get("angle_history", {})
        if isinstance(angles, dict):
            positive = _last_float(angles.get("positive", []))
            negative = _last_float(angles.get("negative", []))
            if positive is not None and negative is not None:
                return abs(positive - negative)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            positive_metrics = metrics.get("positive", {})
            negative_metrics = metrics.get("negative", {})
            if isinstance(positive_metrics, dict) and isinstance(
                negative_metrics, dict
            ):
                try:
                    return abs(
                        float(positive_metrics.get("angle", 0.0))
                        - float(negative_metrics.get("angle", 0.0))
                    )
                except (TypeError, ValueError):
                    return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.15:
            return 1.0

        angles = snapshot.get("angle_history", {})
        if isinstance(angles, dict):
            centered = _last_float(angles.get("centered", []))
            positive = _last_float(angles.get("positive", []))
            high = _last_float(angles.get("high_inertia", []))
            if centered is not None and positive is not None:
                if abs(centered) <= 0.01 and abs(positive) >= 0.08:
                    return 1.0
            if positive is not None and high is not None:
                if abs(positive) - abs(high) >= 0.05:
                    return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                centered_metrics = metrics.get("centered", {})
                positive_metrics = metrics.get("positive", {})
                negative_metrics = metrics.get("negative", {})
                high_metrics = metrics.get("high_inertia", {})
                if isinstance(positive_metrics, dict) and isinstance(
                    negative_metrics, dict
                ):
                    if (
                        abs(
                            float(positive_metrics.get("angle", 0.0))
                            - float(negative_metrics.get("angle", 0.0))
                        )
                        >= 0.15
                    ):
                        return 1.0
                if isinstance(centered_metrics, dict) and isinstance(
                    positive_metrics, dict
                ):
                    if (
                        abs(float(centered_metrics.get("angle", 0.0))) <= 0.01
                        and abs(float(positive_metrics.get("angle", 0.0))) >= 0.08
                    ):
                        return 1.0
                if isinstance(positive_metrics, dict) and isinstance(
                    high_metrics, dict
                ):
                    if (
                        abs(float(positive_metrics.get("angle", 0.0)))
                        - abs(float(high_metrics.get("angle", 0.0)))
                        >= 0.05
                    ):
                        return 1.0
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.executor_index = int(controls.get("executor_index", self.executor_index))
        self.com_offset = float(controls.get("com_offset", self.com_offset))
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self.link_mass = float(controls.get("link_mass", self.link_mass))
        self.inertia_scale = float(controls.get("inertia_scale", self.inertia_scale))
        self._apply_parameters(reset_state=False)
        try:
            self.world.state_vector = np.asarray(state["state"], dtype=float)
            self.world.time = float(state.get("time", self.world.time))
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        self._restore_histories(self._angle_history, state.get("angle_history", {}))
        self._restore_histories(self._accel_history, state.get("accel_history", {}))
        self._restore_histories(
            self._accel_error_history, state.get("accel_error_history", {})
        )
        self._restore_histories(self._torque_history, state.get("torque_history", {}))
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
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

    def _lane_text(self, builder: Any, lane: _ComLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        builder.text(f"{lane.label}: {metrics['status']}")
        builder.text(
            f"COM {float(metrics['offset']):+.3f} m | "
            f"tau {float(metrics['gravity_torque']):+.3f} N m | "
            f"qdd {float(metrics['acceleration']):+.3f}/"
            f"{float(metrics['expected_acceleration']):+.3f} rad/s^2"
        )
        builder.text(
            f"q {float(metrics['angle']):+.3f} rad | "
            f"qd {float(metrics['velocity']):+.3f} rad/s | "
            f"M {float(metrics['mass_matrix']):.3f} | "
            f"COM z {float(metrics['com_world_z']):.3f}"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_executor, executor_index = builder.select(
            "Executor",
            int(self.executor_index),
            [label for label, _executor in self._executors],
        )
        changed_offset, com_offset = builder.slider(
            "COM offset", float(self.com_offset), 0.0, 0.32
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.5
        )
        changed_mass, link_mass = builder.slider(
            "Link mass", float(self.link_mass), 0.5, 5.0
        )
        changed_inertia, inertia_scale = builder.slider(
            "High-inertia multiplier", float(self.inertia_scale), 1.0, 8.0
        )
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_offset:
            self.com_offset = float(com_offset)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if changed_mass:
            self.link_mass = float(link_mass)
        if changed_inertia:
            self.inertia_scale = float(inertia_scale)
        if (
            changed_executor
            or changed_offset
            or changed_gravity
            or changed_mass
            or changed_inertia
        ):
            self.reset(clear_replay=True)

        if builder.button("Reset COM offsets"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: link center-of-mass offset family")
        builder.text(
            "held fixed: World multibody revolute links | contacts off | "
            "visual geometry fixed | "
            f"link mass {self.link_mass:.1f} | gravity scale {self.gravity_scale:.1f} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: World multibody revolute links | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text("visual boxes stay centered; yellow markers are Link.center_of_mass")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Centered angle", list(self._angle_history["centered"]))
        builder.plot_lines("+X COM angle", list(self._angle_history["positive"]))
        builder.plot_lines("-X COM angle", list(self._angle_history["negative"]))
        builder.plot_lines(
            "High-inertia acceleration", list(self._accel_history["high_inertia"])
        )
        builder.plot_lines(
            "+X acceleration error", list(self._accel_error_history["positive"])
        )
        builder.plot_lines("+X energy", list(self._energy_history["positive"]))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidLinkCenterOfMass()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Link Center of Mass", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_link_center_of_mass_controller": controller,
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Mirrored COM angle spread",
                "signal": controller.replay_timeline_signal,
                "markers": controller.replay_timeline_marker,
            },
            "replay_sync": controller._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_link_center_of_mass",
    title="Rigid Link Center of Mass",
    category="World Rigid Body",
    summary=(
        "Shows how Link.center_of_mass offsets create gravity torque, flip "
        "hinge acceleration sign, and interact with link inertia."
    ),
    build=build,
)
