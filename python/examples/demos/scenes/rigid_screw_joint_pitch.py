"""Screw-joint pitch coupling verifier for DART 7 World multibodies."""

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
_AXIS_Z = np.array([0.0, 0.0, 1.0])
_BASE_INERTIA_XY = 0.08


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


@dataclass
class _ScrewLane:
    key: str
    label: str
    pitch_multiplier: float
    anchor: np.ndarray
    color: tuple[float, float, float]
    robot: Any
    base: Any
    nut: Any
    joint: Any
    nut_frame: Any | None = None
    tick_frame: Any | None = None


class _RigidScrewJointPitchVerifier:
    def __init__(self) -> None:
        self.executor_index = 0
        self.pitch_scale = 0.28
        self.gravity_scale = 1.0
        self.moving_mass = 2.0
        self.axial_inertia = 0.12
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(time_step=_TIME_STEP)
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._add_lane(
                "zero_pitch",
                "Zero pitch",
                0.0,
                (-1.35, 0.0, 1.05),
                (0.58, 0.58, 0.64),
            ),
            self._add_lane(
                "fine_pitch",
                "Fine pitch",
                1.0,
                (-0.45, 0.0, 1.05),
                (0.22, 0.56, 0.88),
            ),
            self._add_lane(
                "coarse_pitch",
                "Coarse pitch",
                2.0,
                (0.45, 0.0, 1.05),
                (0.26, 0.70, 0.42),
            ),
            self._add_lane(
                "reverse_pitch",
                "Reverse pitch",
                -1.0,
                (1.35, 0.0, 1.05),
                (0.86, 0.48, 0.24),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_screw_joint_pitch")
        self._add_visuals()

        self._angle_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._travel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_error_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self.reset(clear_replay=True)

    def _add_lane(
        self,
        key: str,
        label: str,
        pitch_multiplier: float,
        anchor: tuple[float, float, float],
        color: tuple[float, float, float],
    ) -> _ScrewLane:
        robot = self.world.add_multibody(f"{key}_screw")
        base = robot.add_link(f"{key}_base")
        nut = robot.add_link(
            f"{key}_nut",
            parent=base,
            joint=sx.JointSpec(
                name=f"{key}_helix",
                type=sx.JointType.SCREW,
                axis=tuple(_AXIS_Z),
            ),
        )
        return _ScrewLane(
            key=key,
            label=label,
            pitch_multiplier=float(pitch_multiplier),
            anchor=np.asarray(anchor, dtype=float),
            color=color,
            robot=robot,
            base=base,
            nut=nut,
            joint=nut.parent_joint,
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
                _box((0.34, 0.22, 0.05)),
                lane.anchor + np.array([0.0, 0.0, -0.08]),
                (0.30, 0.32, 0.36),
            )
            self._add_static_visual(
                f"{lane.key}_guide",
                _box((0.035, 0.035, 1.30)),
                lane.anchor + np.array([0.0, -0.18, -0.32]),
                (0.42, 0.44, 0.48),
            )
            self._add_static_visual(
                f"{lane.key}_pitch_marker",
                _box((0.19, 0.025, 0.025)),
                lane.anchor
                + np.array(
                    [
                        0.0,
                        0.20,
                        self._lane_pitch(lane)
                        if lane.key != "zero_pitch"
                        else 0.0,
                    ]
                ),
                (0.92, 0.82, 0.28),
            )
            lane.nut_frame = self._add_static_visual(
                f"{lane.key}_nut_visual",
                _box((0.18, 0.18, 0.12)),
                lane.anchor,
                lane.color,
            )
            lane.tick_frame = self._add_static_visual(
                f"{lane.key}_rotation_tick",
                _box((0.25, 0.035, 0.035)),
                lane.anchor + np.array([0.18, 0.0, 0.03]),
                (0.96, 0.88, 0.34),
            )
        self.bridge.sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _lane_pitch(self, lane: _ScrewLane) -> float:
        return float(lane.pitch_multiplier) * float(self.pitch_scale)

    def _effective_mass(self, lane: _ScrewLane) -> float:
        pitch = self._lane_pitch(lane)
        return float(self.axial_inertia) + float(self.moving_mass) * pitch * pitch

    def _expected_acceleration(self, lane: _ScrewLane) -> float:
        pitch = self._lane_pitch(lane)
        effective_mass = self._effective_mass(lane)
        if effective_mass <= 0.0:
            return 0.0
        return (
            -float(self.moving_mass)
            * _GRAVITY
            * float(self.gravity_scale)
            * pitch
            / effective_mass
        )

    def _apply_parameters(self, *, reset_state: bool) -> None:
        self.pitch_scale = float(np.clip(self.pitch_scale, 0.02, 0.60))
        self.gravity_scale = float(np.clip(self.gravity_scale, 0.0, 1.40))
        self.moving_mass = float(np.clip(self.moving_mass, 0.5, 5.0))
        self.axial_inertia = float(np.clip(self.axial_inertia, 0.03, 0.80))
        self.world.gravity = (0.0, 0.0, -_GRAVITY * float(self.gravity_scale))
        inertia = (
            (_BASE_INERTIA_XY, 0.0, 0.0),
            (0.0, _BASE_INERTIA_XY, 0.0),
            (0.0, 0.0, float(self.axial_inertia)),
        )
        for lane in self.lanes:
            lane.nut.mass = float(self.moving_mass)
            lane.nut.inertia = inertia
            lane.joint.pitch = self._lane_pitch(lane)
            if reset_state:
                lane.joint.position = [0.0]
                lane.joint.velocity = [0.0]
                lane.joint.force = [0.0]

    def reset(self, *, clear_replay: bool = False) -> None:
        self._apply_parameters(reset_state=True)
        self.world.time = 0.0
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        self.world.update_kinematics()
        for history in (
            *self._angle_history.values(),
            *self._travel_history.values(),
            *self._accel_error_history.values(),
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

    def _sample(self, lane: _ScrewLane) -> dict[str, float | str]:
        pitch = self._lane_pitch(lane)
        angle = _joint_scalar(lane.joint.position)
        velocity = _joint_scalar(lane.joint.velocity)
        acceleration = _joint_scalar(lane.joint.acceleration)
        delta = np.asarray(lane.nut.translation, dtype=float).reshape(3)
        axial_travel = float(np.dot(delta, _AXIS_Z))
        travel_per_radian = axial_travel / angle if abs(angle) > 1.0e-9 else pitch
        expected_accel = self._expected_acceleration(lane)
        expected_axial_accel = pitch * expected_accel
        actual_axial_accel = pitch * acceleration
        try:
            mass_matrix = float(np.asarray(lane.robot.mass_matrix, dtype=float)[0, 0])
        except Exception:  # noqa: BLE001
            mass_matrix = 0.0
        if abs(pitch) < 1.0e-12:
            status = "zero pitch: no axial coupling"
        elif pitch > 0.0:
            status = "positive pitch: gravity rotates negative"
        else:
            status = "reverse pitch: rotation sign flips"
        return {
            "pitch": pitch,
            "angle": angle,
            "velocity": velocity,
            "acceleration": acceleration,
            "expected_acceleration": expected_accel,
            "acceleration_error": acceleration - expected_accel,
            "axial_travel": axial_travel,
            "travel_per_radian": travel_per_radian,
            "actual_axial_acceleration": actual_axial_accel,
            "expected_axial_acceleration": expected_axial_accel,
            "effective_mass": self._effective_mass(lane),
            "mass_matrix": mass_matrix,
            "status": status,
        }

    def _record_metrics(self) -> None:
        for lane in self.lanes:
            metrics = self._sample(lane)
            self._last_metrics[lane.key] = metrics
            self._angle_history[lane.key].append(float(metrics["angle"]))
            self._travel_history[lane.key].append(float(metrics["axial_travel"]))
            self._accel_error_history[lane.key].append(
                float(metrics["acceleration_error"])
            )
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        self.world.step(self._executor())
        self._record_metrics()
        self._sync()

    def _sync(self) -> None:
        for lane in self.lanes:
            link_transform = np.asarray(lane.nut.transform, dtype=float)
            display_transform = _translation(lane.anchor) @ link_transform
            if lane.nut_frame is not None:
                lane.nut_frame.set_transform(display_transform)
            if lane.tick_frame is not None:
                lane.tick_frame.set_transform(
                    display_transform @ _translation((0.18, 0.0, 0.03))
                )
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
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
                "pitch_multiplier": float(lane.pitch_multiplier),
                "joint": lane.joint.name,
                "metrics": serialized_metrics(lane.key),
            }
            for lane in self.lanes
        }
        fine_pitch = lane_value("fine_pitch", "pitch")
        coarse_pitch = lane_value("coarse_pitch", "pitch")
        reverse_pitch = lane_value("reverse_pitch", "pitch")
        fine_travel = lane_value("fine_pitch", "axial_travel")
        coarse_travel = lane_value("coarse_pitch", "axial_travel")
        reverse_travel = lane_value("reverse_pitch", "axial_travel")
        coarse_fine_travel_gap = abs(coarse_travel - fine_travel)
        lane_order = [lane.key for lane in self.lanes]
        payload: dict[str, Any] = {
            "row": "rigid_screw_joint_pitch",
            "comparison_axis": "screw_pitch_coupling_family",
            "solver": "world_multibody_screw_joint_pitch",
            "scope": "contact_free_screw_pitch_lanes",
            "executor": self._executors[executor_index][0],
            "held_fixed": {
                "solver": "world_multibody_screw_joint_pitch",
                "joint_family": "screw",
                "axis": "z",
                "contacts": "off",
                "moving_mass": float(self.moving_mass),
                "axial_inertia": float(self.axial_inertia),
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "controls": {
                "executor_index": float(executor_index),
                "pitch_scale": float(self.pitch_scale),
                "gravity_scale": float(self.gravity_scale),
                "moving_mass": float(self.moving_mass),
                "axial_inertia": float(self.axial_inertia),
            },
            "joint_lanes": lane_order,
            "lane_order": lane_order,
            "lane_count": float(len(self.lanes)),
            "lanes": lanes,
            "screw_joint_zero_pitch_axial_travel": lane_value(
                "zero_pitch", "axial_travel"
            ),
            "screw_joint_fine_pitch": fine_pitch,
            "screw_joint_coarse_pitch": coarse_pitch,
            "screw_joint_reverse_pitch": reverse_pitch,
            "screw_joint_coarse_fine_travel_gap": coarse_fine_travel_gap,
            "screw_joint_reverse_angle": lane_value("reverse_pitch", "angle"),
            "screw_joint_fine_acceleration_error": lane_value(
                "fine_pitch", "acceleration_error"
            ),
            "zero_pitch": lane_value("zero_pitch", "pitch"),
            "zero_pitch_angle": lane_value("zero_pitch", "angle"),
            "zero_pitch_axial_travel": lane_value("zero_pitch", "axial_travel"),
            "fine_pitch": fine_pitch,
            "coarse_pitch": coarse_pitch,
            "reverse_pitch": reverse_pitch,
            "coarse_to_fine_pitch_ratio": coarse_pitch / max(fine_pitch, 1.0e-12),
            "reverse_to_fine_pitch_ratio": reverse_pitch / max(fine_pitch, 1.0e-12),
            "fine_angle": lane_value("fine_pitch", "angle"),
            "coarse_angle": lane_value("coarse_pitch", "angle"),
            "reverse_angle": lane_value("reverse_pitch", "angle"),
            "fine_axial_travel": fine_travel,
            "coarse_axial_travel": coarse_travel,
            "reverse_axial_travel": reverse_travel,
            "coarse_minus_fine_axial_travel": coarse_travel - fine_travel,
            "reverse_minus_fine_axial_travel": reverse_travel - fine_travel,
            "fine_travel_per_radian": lane_value(
                "fine_pitch", "travel_per_radian"
            ),
            "coarse_travel_per_radian": lane_value(
                "coarse_pitch", "travel_per_radian"
            ),
            "reverse_travel_per_radian": lane_value(
                "reverse_pitch", "travel_per_radian"
            ),
            "fine_acceleration": lane_value("fine_pitch", "acceleration"),
            "fine_expected_acceleration": lane_value(
                "fine_pitch", "expected_acceleration"
            ),
            "fine_acceleration_error": lane_value(
                "fine_pitch", "acceleration_error"
            ),
            "coarse_acceleration": lane_value("coarse_pitch", "acceleration"),
            "coarse_expected_acceleration": lane_value(
                "coarse_pitch", "expected_acceleration"
            ),
            "coarse_acceleration_error": lane_value(
                "coarse_pitch", "acceleration_error"
            ),
            "reverse_acceleration": lane_value("reverse_pitch", "acceleration"),
            "reverse_expected_acceleration": lane_value(
                "reverse_pitch", "expected_acceleration"
            ),
            "reverse_acceleration_error": lane_value(
                "reverse_pitch", "acceleration_error"
            ),
            "fine_actual_axial_acceleration": lane_value(
                "fine_pitch", "actual_axial_acceleration"
            ),
            "fine_expected_axial_acceleration": lane_value(
                "fine_pitch", "expected_axial_acceleration"
            ),
            "coarse_actual_axial_acceleration": lane_value(
                "coarse_pitch", "actual_axial_acceleration"
            ),
            "coarse_expected_axial_acceleration": lane_value(
                "coarse_pitch", "expected_axial_acceleration"
            ),
            "reverse_actual_axial_acceleration": lane_value(
                "reverse_pitch", "actual_axial_acceleration"
            ),
            "reverse_expected_axial_acceleration": lane_value(
                "reverse_pitch", "expected_axial_acceleration"
            ),
            "fine_effective_mass": lane_value("fine_pitch", "effective_mass"),
            "coarse_effective_mass": lane_value("coarse_pitch", "effective_mass"),
            "reverse_effective_mass": lane_value("reverse_pitch", "effective_mass"),
            "fine_mass_matrix": lane_value("fine_pitch", "mass_matrix"),
            "coarse_mass_matrix": lane_value("coarse_pitch", "mass_matrix"),
            "reverse_mass_matrix": lane_value("reverse_pitch", "mass_matrix"),
            "step_ms": self._step_ms_history[-1] if self._step_ms_history else 0.0,
            "history": {
                "samples": float(len(self._step_ms_history)),
                "max_step_ms": max(self._step_ms_history, default=0.0),
                "max_abs_coarse_minus_fine_axial_travel": max(
                    (
                        abs(coarse - fine)
                        for coarse, fine in zip(
                            self._travel_history["coarse_pitch"],
                            self._travel_history["fine_pitch"],
                            strict=False,
                        )
                    ),
                    default=0.0,
                ),
                "max_abs_reverse_minus_fine_axial_travel": max(
                    (
                        abs(reverse - fine)
                        for reverse, fine in zip(
                            self._travel_history["reverse_pitch"],
                            self._travel_history["fine_pitch"],
                            strict=False,
                        )
                    ),
                    default=0.0,
                ),
            },
        }
        for lane in self.lanes:
            lane_metrics = self._last_metrics[lane.key]
            for metric_key in (
                "pitch",
                "angle",
                "velocity",
                "acceleration",
                "expected_acceleration",
                "acceleration_error",
                "axial_travel",
                "travel_per_radian",
                "actual_axial_acceleration",
                "expected_axial_acceleration",
                "effective_mass",
                "mass_matrix",
            ):
                payload[f"{lane.key}_{metric_key}"] = float(
                    lane_metrics[metric_key]
                )
            payload["history"][f"{lane.key}_max_abs_angle"] = max_abs(
                self._angle_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_abs_axial_travel"] = max_abs(
                self._travel_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_abs_acceleration_error"] = (
                max_abs_after_initial(self._accel_error_history[lane.key])
            )
        return payload

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "pitch_scale": float(self.pitch_scale),
                "gravity_scale": float(self.gravity_scale),
                "moving_mass": float(self.moving_mass),
                "axial_inertia": float(self.axial_inertia),
            },
            "state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "time": float(self.world.time),
            "angle_history": {
                key: list(values) for key, values in self._angle_history.items()
            },
            "travel_history": {
                key: list(values) for key, values in self._travel_history.items()
            },
            "accel_error_history": {
                key: list(values)
                for key, values in self._accel_error_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0

        travels = snapshot.get("travel_history", {})
        if isinstance(travels, dict):
            fine = _last_float(travels.get("fine_pitch", []))
            coarse = _last_float(travels.get("coarse_pitch", []))
            if fine is not None and coarse is not None:
                return abs(coarse - fine)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            fine_metrics = metrics.get("fine_pitch", {})
            coarse_metrics = metrics.get("coarse_pitch", {})
            if isinstance(fine_metrics, dict) and isinstance(coarse_metrics, dict):
                try:
                    return abs(
                        float(coarse_metrics.get("axial_travel", 0.0))
                        - float(fine_metrics.get("axial_travel", 0.0))
                    )
                except (TypeError, ValueError):
                    return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.05:
            return 1.0

        travels = snapshot.get("travel_history", {})
        if isinstance(travels, dict):
            zero = _last_float(travels.get("zero_pitch", []))
            fine = _last_float(travels.get("fine_pitch", []))
            if zero is not None and fine is not None:
                if abs(zero) <= 0.01 and abs(fine) >= 0.08:
                    return 1.0

        angles = snapshot.get("angle_history", {})
        if isinstance(angles, dict):
            fine_angle = _last_float(angles.get("fine_pitch", []))
            reverse_angle = _last_float(angles.get("reverse_pitch", []))
            if fine_angle is not None and reverse_angle is not None:
                if fine_angle <= -0.25 and reverse_angle >= 0.25:
                    return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                zero_metrics = metrics.get("zero_pitch", {})
                fine_metrics = metrics.get("fine_pitch", {})
                reverse_metrics = metrics.get("reverse_pitch", {})
                coarse_metrics = metrics.get("coarse_pitch", {})

                if isinstance(fine_metrics, dict) and isinstance(
                    coarse_metrics, dict
                ):
                    if (
                        abs(
                            float(coarse_metrics.get("axial_travel", 0.0))
                            - float(fine_metrics.get("axial_travel", 0.0))
                        )
                        >= 0.05
                    ):
                        return 1.0

                if isinstance(zero_metrics, dict) and isinstance(fine_metrics, dict):
                    if (
                        abs(float(zero_metrics.get("axial_travel", 0.0))) <= 0.01
                        and abs(float(fine_metrics.get("axial_travel", 0.0)))
                        >= 0.08
                    ):
                        return 1.0

                if isinstance(fine_metrics, dict) and isinstance(
                    reverse_metrics, dict
                ):
                    if (
                        float(fine_metrics.get("angle", 0.0)) <= -0.25
                        and float(reverse_metrics.get("angle", 0.0)) >= 0.25
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
        self.pitch_scale = float(controls.get("pitch_scale", self.pitch_scale))
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self.moving_mass = float(controls.get("moving_mass", self.moving_mass))
        self.axial_inertia = float(controls.get("axial_inertia", self.axial_inertia))
        self._apply_parameters(reset_state=False)
        if "state" in state:
            self.world.state_vector = state["state"]
        self.world.time = float(state.get("time", self.world.time))
        self.world.update_kinematics()
        self._restore_histories(self._angle_history, state.get("angle_history", {}))
        self._restore_histories(self._travel_history, state.get("travel_history", {}))
        self._restore_histories(
            self._accel_error_history, state.get("accel_error_history", {})
        )
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

    def _lane_text(self, builder: Any, lane: _ScrewLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane)
        builder.text(f"{lane.label}: {metrics['status']}")
        builder.text(
            f"pitch {float(metrics['pitch']):.3f} m/rad | "
            f"q {float(metrics['angle']):.3f} rad | "
            f"z {float(metrics['axial_travel']):.3f} m | "
            f"z/q {float(metrics['travel_per_radian']):.3f}"
        )
        builder.text(
            f"qdd {float(metrics['acceleration']):.3f}/"
            f"{float(metrics['expected_acceleration']):.3f} rad/s^2 | "
            f"zdd {float(metrics['actual_axial_acceleration']):.3f}/"
            f"{float(metrics['expected_axial_acceleration']):.3f} m/s^2"
        )
        builder.text(
            f"effective mass {float(metrics['effective_mass']):.3f} | "
            f"mass matrix {float(metrics['mass_matrix']):.3f}"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_pitch, pitch_scale = builder.slider(
            "Pitch scale", float(self.pitch_scale), 0.02, 0.60
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.40
        )
        changed_mass, moving_mass = builder.slider(
            "Moving mass", float(self.moving_mass), 0.5, 5.0
        )
        changed_inertia, axial_inertia = builder.slider(
            "Axial inertia", float(self.axial_inertia), 0.03, 0.80
        )
        if (
            changed_executor
            or changed_pitch
            or changed_gravity
            or changed_mass
            or changed_inertia
        ):
            self.pitch_scale = float(pitch_scale)
            self.gravity_scale = float(gravity_scale)
            self.moving_mass = float(moving_mass)
            self.axial_inertia = float(axial_inertia)
            self.reset(clear_replay=True)

        if builder.button("Reset screw joints"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: screw pitch coupling family")
        builder.text(
            "held fixed: World screw joints | contacts off | z-axis screw | "
            f"moving mass {self.moving_mass:.1f} | axial inertia {self.axial_inertia:.2f} | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: World multibody screw joints | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text("pitch is translation per radian; gravity drives the screw axis")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines("Fine pitch angle", list(self._angle_history["fine_pitch"]))
        builder.plot_lines(
            "Fine pitch axial travel", list(self._travel_history["fine_pitch"])
        )
        builder.plot_lines(
            "Coarse pitch axial travel", list(self._travel_history["coarse_pitch"])
        )
        builder.plot_lines(
            "Reverse pitch angle", list(self._angle_history["reverse_pitch"])
        )
        builder.plot_lines(
            "Fine acceleration error",
            list(self._accel_error_history["fine_pitch"]),
        )
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    verifier = _RigidScrewJointPitchVerifier()
    return SceneSetup(
        world=verifier.bridge.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.force_drag,
        renderable_provider=verifier.renderable_provider,
        panels=[ScenePanel("Rigid Screw Joint Pitch", verifier.build_panel)],
        info={
            "sx_world": verifier.world,
            "rigid_screw_joint_pitch_controller": verifier,
            CAPTURE_METRICS_INFO_KEY: verifier.capture_metrics,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Coarse/fine travel gap",
                "signal": verifier.replay_timeline_signal,
                "markers": verifier.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_screw_joint_pitch",
    title="Rigid Screw Joint Pitch",
    category="World Rigid Body",
    summary=(
        "Shows how screw-joint pitch couples rotation, axial translation, "
        "gravity, mass, and inertia in World multibody dynamics."
    ),
    build=build,
)
