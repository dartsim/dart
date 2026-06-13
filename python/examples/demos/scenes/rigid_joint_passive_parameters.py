"""Rigid multibody passive-joint parameter verifier for DART 7 World."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_MASS = 2.0
_INITIAL_SPRING_POSITION = 0.52
_RAIL_LENGTH = 1.55
_RAIL_MIN = -0.60
_RAIL_MAX = 1.30


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
class _PassiveLane:
    key: str
    label: str
    kind: str
    anchor: np.ndarray
    joint: Any
    link: Any
    color: tuple[float, float, float]


class _RigidJointPassiveParameterVerifier:
    def __init__(self) -> None:
        self.executor_index = 0
        self.spring_stiffness = 16.0
        self.damping_coefficient = 3.0
        self.rest_position = 0.0
        self.coulomb_friction = 6.0
        self.hold_force = 3.0
        self.slip_force = 9.0
        self.armature_force = 6.0
        self.armature = 6.0
        self._executors = self._make_executors()

        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._add_slider_lane(
                "spring_only",
                "Spring only",
                "spring",
                (-1.15, 0.78, 0.42),
                (0.20, 0.55, 0.90),
            ),
            self._add_slider_lane(
                "spring_damper",
                "Spring + damping",
                "damped",
                (-1.15, 0.44, 0.42),
                (0.30, 0.74, 0.42),
            ),
            self._add_slider_lane(
                "stiction",
                "Coulomb stiction",
                "stiction",
                (-1.15, 0.10, 0.42),
                (0.58, 0.58, 0.64),
            ),
            self._add_slider_lane(
                "slip",
                "Coulomb slip",
                "slip",
                (-1.15, -0.24, 0.42),
                (0.90, 0.54, 0.22),
            ),
            self._add_slider_lane(
                "armature_reference",
                "No armature drive",
                "armature_reference",
                (-1.15, -0.58, 0.42),
                (0.76, 0.38, 0.82),
            ),
            self._add_slider_lane(
                "armature_heavy",
                "High armature drive",
                "armature",
                (-1.15, -0.92, 0.42),
                (0.26, 0.70, 0.70),
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(
            self.world, name="rigid_joint_passive_parameters_render"
        )
        self.rest_markers: dict[str, Any] = {}
        self._add_visuals()

        self._position_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._speed_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._accel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._energy_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self.reset(clear_replay=True)

    def _make_executors(self) -> list[tuple[str, Any]]:
        executors: list[tuple[str, Any]] = [("Sequential", sx.SequentialExecutor())]
        try:
            executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass
        return executors

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _add_slider_lane(
        self,
        key: str,
        label: str,
        kind: str,
        anchor: tuple[float, float, float],
        color: tuple[float, float, float],
    ) -> _PassiveLane:
        robot = self.world.add_multibody(f"{key}_robot")
        base = robot.add_link(f"{key}_base")
        carriage = robot.add_link(
            f"{key}_carriage",
            parent=base,
            joint=sx.JointSpec(
                name=f"{key}_rail",
                type=sx.JointType.PRISMATIC,
                axis=(1.0, 0.0, 0.0),
                transform_from_parent=_translation(anchor),
            ),
        )
        carriage.mass = _MASS
        carriage.inertia = ((0.04, 0.0, 0.0), (0.0, 0.04, 0.0), (0.0, 0.0, 0.04))
        joint = carriage.parent_joint
        joint.set_position_limits([_RAIL_MIN], [_RAIL_MAX])
        return _PassiveLane(
            key=key,
            label=label,
            kind=kind,
            anchor=np.asarray(anchor, dtype=float),
            joint=joint,
            link=carriage,
            color=color,
        )

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
        for lane in self.lanes:
            rail_center = lane.anchor + np.array([_RAIL_LENGTH * 0.5, 0.0, -0.13])
            self._add_static_visual(
                f"{lane.key}_rail",
                _box((_RAIL_LENGTH, 0.026, 0.026)),
                rail_center,
                (0.34, 0.36, 0.40),
            )
            self.rest_markers[lane.key] = self._add_static_visual(
                f"{lane.key}_rest_marker",
                _box((0.030, 0.16, 0.16)),
                self._rest_marker_position(lane),
                (0.92, 0.86, 0.32),
            )
            self.bridge.add_link_visual(
                lane.link,
                _box((0.18, 0.12, 0.12)),
                lane.color,
                name=f"{lane.key}_carriage_visual",
            )
        self.bridge.sync()

    def _rest_marker_position(self, lane: _PassiveLane) -> np.ndarray:
        if lane.kind in {"spring", "damped"}:
            q = float(self.rest_position)
        else:
            q = 0.0
        return lane.anchor + np.array([q, 0.0, -0.02])

    def _apply_parameters(self, *, reset_state: bool) -> None:
        spring_stiffness = max(0.0, float(self.spring_stiffness))
        damping = max(0.0, float(self.damping_coefficient))
        friction = max(0.0, float(self.coulomb_friction))
        hold_force = max(0.0, float(self.hold_force))
        slip_force = max(0.0, float(self.slip_force))
        armature_force = max(0.0, float(self.armature_force))
        armature = max(0.0, float(self.armature))
        rest_position = float(np.clip(self.rest_position, -0.35, 0.35))
        self.spring_stiffness = spring_stiffness
        self.damping_coefficient = damping
        self.coulomb_friction = friction
        self.hold_force = hold_force
        self.slip_force = slip_force
        self.armature_force = armature_force
        self.armature = armature
        self.rest_position = rest_position

        for lane in self.lanes:
            joint = lane.joint
            joint.spring_stiffness = [0.0]
            joint.damping_coefficient = [0.0]
            joint.rest_position = [0.0]
            joint.coulomb_friction = [0.0]
            joint.armature = [0.0]
            joint.force = [0.0]
            joint.set_position_limits([_RAIL_MIN], [_RAIL_MAX])
            if lane.kind in {"spring", "damped"}:
                joint.spring_stiffness = [spring_stiffness]
                joint.rest_position = [rest_position]
                if lane.kind == "damped":
                    joint.damping_coefficient = [damping]
            elif lane.kind == "stiction":
                joint.coulomb_friction = [friction]
                joint.force = [hold_force]
            elif lane.kind == "slip":
                joint.coulomb_friction = [friction]
                joint.force = [slip_force]
            elif lane.kind == "armature_reference":
                joint.force = [armature_force]
            elif lane.kind == "armature":
                joint.armature = [armature]
                joint.force = [armature_force]
            if reset_state:
                if lane.kind in {"spring", "damped"}:
                    joint.position = [_INITIAL_SPRING_POSITION]
                else:
                    joint.position = [0.0]
                joint.velocity = [0.0]

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
            *self._position_history.values(),
            *self._speed_history.values(),
            *self._accel_history.values(),
            *self._energy_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._sync_rest_markers()
        self._record_metrics(
            previous_positions={lane.key: _joint_scalar(lane.joint.position) for lane in self.lanes},
            previous_velocities={lane.key: _joint_scalar(lane.joint.velocity) for lane in self.lanes},
        )
        self._sync()

    def _sync_rest_markers(self) -> None:
        for lane in self.lanes:
            self.rest_markers[lane.key].set_transform(
                _translation(self._rest_marker_position(lane))
            )

    def _expected_acceleration(
        self,
        lane: _PassiveLane,
        previous_position: float,
        previous_velocity: float,
    ) -> float:
        if lane.kind in {"spring", "damped"}:
            stiffness = _joint_scalar(lane.joint.spring_stiffness)
            damping = _joint_scalar(lane.joint.damping_coefficient)
            rest = _joint_scalar(lane.joint.rest_position)
            return (-stiffness * (previous_position - rest) - damping * previous_velocity) / _MASS
        if lane.kind == "stiction":
            force = _joint_scalar(lane.joint.force)
            friction = _joint_scalar(lane.joint.coulomb_friction)
            if abs(previous_velocity) < 1.0e-12 and abs(force) <= friction:
                return 0.0
            return np.sign(force or previous_velocity) * max(abs(force) - friction, 0.0) / _MASS
        if lane.kind == "slip":
            force = _joint_scalar(lane.joint.force)
            friction = _joint_scalar(lane.joint.coulomb_friction)
            return np.sign(force) * max(abs(force) - friction, 0.0) / _MASS
        if lane.kind in {"armature_reference", "armature"}:
            effective_mass = _MASS + _joint_scalar(lane.joint.armature)
            return _joint_scalar(lane.joint.force) / effective_mass
        return 0.0

    def _lane_energy(self, lane: _PassiveLane) -> float:
        q = _joint_scalar(lane.joint.position)
        v = _joint_scalar(lane.joint.velocity)
        armature = _joint_scalar(lane.joint.armature)
        kinetic = 0.5 * (_MASS + armature) * v * v
        stiffness = _joint_scalar(lane.joint.spring_stiffness)
        rest = _joint_scalar(lane.joint.rest_position)
        spring = 0.5 * stiffness * (q - rest) * (q - rest)
        return float(kinetic + spring)

    def _lane_status(self, lane: _PassiveLane) -> str:
        if lane.kind == "spring":
            return "spring restoring"
        if lane.kind == "damped":
            return "damping dissipates"
        if lane.kind == "stiction":
            force = abs(_joint_scalar(lane.joint.force))
            friction = abs(_joint_scalar(lane.joint.coulomb_friction))
            if force > friction:
                return "force exceeds friction"
            return "held by friction"
        if lane.kind == "slip":
            return "force exceeds friction"
        if lane.kind == "armature_reference":
            return "direct drive"
        if lane.kind == "armature":
            return "armature slows drive"
        return "passive joint"

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _record_metrics(
        self,
        *,
        previous_positions: dict[str, float],
        previous_velocities: dict[str, float],
    ) -> None:
        for lane in self.lanes:
            q = _joint_scalar(lane.joint.position)
            v = _joint_scalar(lane.joint.velocity)
            acceleration = _joint_scalar(lane.joint.acceleration)
            expected = self._expected_acceleration(
                lane, previous_positions[lane.key], previous_velocities[lane.key]
            )
            energy = self._lane_energy(lane)
            metrics = {
                "position": q,
                "velocity": v,
                "speed": abs(v),
                "acceleration": acceleration,
                "expected_acceleration": expected,
                "acceleration_error": acceleration - expected,
                "energy": energy,
                "force": _joint_scalar(lane.joint.force),
                "spring_stiffness": _joint_scalar(lane.joint.spring_stiffness),
                "damping_coefficient": _joint_scalar(lane.joint.damping_coefficient),
                "coulomb_friction": _joint_scalar(lane.joint.coulomb_friction),
                "armature": _joint_scalar(lane.joint.armature),
                "status": self._lane_status(lane),
            }
            self._last_metrics[lane.key] = metrics
            self._position_history[lane.key].append(q)
            self._speed_history[lane.key].append(abs(v))
            self._accel_history[lane.key].append(acceleration)
            self._energy_history[lane.key].append(energy)
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        previous_positions = {
            lane.key: _joint_scalar(lane.joint.position) for lane in self.lanes
        }
        previous_velocities = {
            lane.key: _joint_scalar(lane.joint.velocity) for lane in self.lanes
        }
        self.world.step(self._executor())
        self._record_metrics(
            previous_positions=previous_positions,
            previous_velocities=previous_velocities,
        )
        self._sync()

    def _sync(self) -> None:
        self._sync_rest_markers()
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics(
                previous_positions={
                    lane.key: _joint_scalar(lane.joint.position)
                    for lane in self.lanes
                },
                previous_velocities={
                    lane.key: _joint_scalar(lane.joint.velocity)
                    for lane in self.lanes
                },
            )
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

        lanes = {
            lane.key: {
                "label": lane.label,
                "kind": lane.kind,
                "joint": lane.joint.name,
                "metrics": serialized_metrics(lane.key),
            }
            for lane in self.lanes
        }
        spring_energy = lane_value("spring_only", "energy")
        damped_energy = lane_value("spring_damper", "energy")
        damped_energy_ratio = damped_energy / max(spring_energy, 1.0e-12)
        reference_acceleration = lane_value("armature_reference", "acceleration")
        armature_acceleration = lane_value("armature_heavy", "acceleration")
        armature_acceleration_gap = reference_acceleration - armature_acceleration
        armature_position_gap = lane_value(
            "armature_reference", "position"
        ) - lane_value("armature_heavy", "position")
        lane_order = [lane.key for lane in self.lanes]
        payload: dict[str, Any] = {
            "row": "rigid_joint_passive_parameters",
            "comparison_axis": "passive_joint_parameter_family",
            "solver": "world_multibody_passive_joint_parameters",
            "scope": "contact_free_prismatic_lanes",
            "executor": self._executors[executor_index][0],
            "held_fixed": {
                "solver": "world_multibody_passive_joint_parameters",
                "joint_family": "prismatic",
                "gravity": "off",
                "contacts": "off",
                "link_mass": _MASS,
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "controls": {
                "executor_index": float(executor_index),
                "spring_stiffness": float(self.spring_stiffness),
                "damping_coefficient": float(self.damping_coefficient),
                "rest_position": float(self.rest_position),
                "coulomb_friction": float(self.coulomb_friction),
                "hold_force": float(self.hold_force),
                "slip_force": float(self.slip_force),
                "armature_force": float(self.armature_force),
                "armature": float(self.armature),
            },
            "joint_lanes": lane_order,
            "lane_order": lane_order,
            "lane_count": float(len(self.lanes)),
            "lanes": lanes,
            "passive_joint_spring_energy": spring_energy,
            "passive_joint_damped_energy": damped_energy,
            "passive_joint_damped_energy_ratio": damped_energy_ratio,
            "passive_joint_slip_speed": lane_value("slip", "speed"),
            "passive_joint_armature_position_gap": armature_position_gap,
            "passive_joint_armature_acceleration_gap": armature_acceleration_gap,
            "spring_energy": spring_energy,
            "damped_energy": damped_energy,
            "damped_energy_ratio": damped_energy_ratio,
            "stiction_position": lane_value("stiction", "position"),
            "stiction_speed": lane_value("stiction", "speed"),
            "slip_position": lane_value("slip", "position"),
            "slip_speed": lane_value("slip", "speed"),
            "slip_acceleration": lane_value("slip", "acceleration"),
            "slip_expected_acceleration": lane_value(
                "slip", "expected_acceleration"
            ),
            "slip_acceleration_error": lane_value("slip", "acceleration_error"),
            "armature_reference_acceleration": reference_acceleration,
            "armature_heavy_acceleration": armature_acceleration,
            "armature_acceleration_gap": armature_acceleration_gap,
            "armature_position_gap": armature_position_gap,
            "step_ms": self._step_ms_history[-1] if self._step_ms_history else 0.0,
            "history": {
                "samples": float(len(self._step_ms_history)),
                "max_step_ms": max(self._step_ms_history, default=0.0),
                "max_spring_energy": max(
                    self._energy_history["spring_only"], default=0.0
                ),
                "max_damped_energy": max(
                    self._energy_history["spring_damper"], default=0.0
                ),
                "max_slip_position": max(
                    self._position_history["slip"], default=0.0
                ),
                "max_armature_position_gap": max(
                    (
                        reference - heavy
                        for reference, heavy in zip(
                            self._position_history["armature_reference"],
                            self._position_history["armature_heavy"],
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
                "position",
                "velocity",
                "speed",
                "acceleration",
                "expected_acceleration",
                "acceleration_error",
                "energy",
            ):
                payload[f"{lane.key}_{metric_key}"] = float(
                    lane_metrics[metric_key]
                )
            payload["history"][f"{lane.key}_max_abs_position"] = max_abs(
                self._position_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_speed"] = max(
                self._speed_history[lane.key], default=0.0
            )
            payload["history"][f"{lane.key}_max_abs_acceleration"] = max_abs(
                self._accel_history[lane.key]
            )
            payload["history"][f"{lane.key}_max_energy"] = max(
                self._energy_history[lane.key], default=0.0
            )
        return payload

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "spring_stiffness": float(self.spring_stiffness),
                "damping_coefficient": float(self.damping_coefficient),
                "rest_position": float(self.rest_position),
                "coulomb_friction": float(self.coulomb_friction),
                "hold_force": float(self.hold_force),
                "slip_force": float(self.slip_force),
                "armature_force": float(self.armature_force),
                "armature": float(self.armature),
            },
            "position_history": {
                key: list(values) for key, values in self._position_history.items()
            },
            "speed_history": {
                key: list(values) for key, values in self._speed_history.items()
            },
            "accel_history": {
                key: list(values) for key, values in self._accel_history.items()
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

        positions = snapshot.get("position_history", {})
        if isinstance(positions, dict):
            reference = _last_float(positions.get("armature_reference", []))
            heavy = _last_float(positions.get("armature_heavy", []))
            if reference is not None and heavy is not None:
                return max(0.0, reference - heavy)

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            reference_metrics = metrics.get("armature_reference", {})
            heavy_metrics = metrics.get("armature_heavy", {})
            if isinstance(reference_metrics, dict) and isinstance(
                heavy_metrics, dict
            ):
                try:
                    return max(
                        0.0,
                        float(reference_metrics.get("position", 0.0))
                        - float(heavy_metrics.get("position", 0.0)),
                    )
                except (TypeError, ValueError):
                    return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        if self.replay_timeline_signal(snapshot) >= 0.05:
            return 1.0

        energies = snapshot.get("energy_history", {})
        if isinstance(energies, dict):
            spring_energy = _last_float(energies.get("spring_only", []))
            damped_energy = _last_float(energies.get("spring_damper", []))
            if (
                spring_energy is not None
                and damped_energy is not None
                and spring_energy - damped_energy >= 0.25
            ):
                return 1.0

        positions = snapshot.get("position_history", {})
        if isinstance(positions, dict):
            slip_position = _last_float(positions.get("slip", []))
            stiction_position = _last_float(positions.get("stiction", []))
            if slip_position is not None:
                stiction_position = stiction_position or 0.0
                if slip_position - stiction_position >= 0.05:
                    return 1.0

        accelerations = snapshot.get("accel_history", {})
        if isinstance(accelerations, dict):
            reference_acceleration = _last_float(
                accelerations.get("armature_reference", [])
            )
            heavy_acceleration = _last_float(accelerations.get("armature_heavy", []))
            if (
                reference_acceleration is not None
                and heavy_acceleration is not None
                and abs(reference_acceleration - heavy_acceleration) >= 0.50
            ):
                return 1.0

        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                spring_metrics = metrics.get("spring_only", {})
                damped_metrics = metrics.get("spring_damper", {})
                if isinstance(spring_metrics, dict) and isinstance(
                    damped_metrics, dict
                ):
                    if (
                        float(spring_metrics.get("energy", 0.0))
                        - float(damped_metrics.get("energy", 0.0))
                        >= 0.25
                    ):
                        return 1.0

                stiction_metrics = metrics.get("stiction", {})
                slip_metrics = metrics.get("slip", {})
                if isinstance(stiction_metrics, dict) and isinstance(
                    slip_metrics, dict
                ):
                    if (
                        float(slip_metrics.get("position", 0.0))
                        - float(stiction_metrics.get("position", 0.0))
                        >= 0.05
                    ):
                        return 1.0

                reference_metrics = metrics.get("armature_reference", {})
                heavy_metrics = metrics.get("armature_heavy", {})
                if isinstance(reference_metrics, dict) and isinstance(
                    heavy_metrics, dict
                ):
                    if (
                        abs(
                            float(reference_metrics.get("acceleration", 0.0))
                            - float(heavy_metrics.get("acceleration", 0.0))
                        )
                        >= 0.50
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
        self.spring_stiffness = float(
            controls.get("spring_stiffness", self.spring_stiffness)
        )
        self.damping_coefficient = float(
            controls.get("damping_coefficient", self.damping_coefficient)
        )
        self.rest_position = float(controls.get("rest_position", self.rest_position))
        self.coulomb_friction = float(
            controls.get("coulomb_friction", self.coulomb_friction)
        )
        self.hold_force = float(controls.get("hold_force", self.hold_force))
        self.slip_force = float(controls.get("slip_force", self.slip_force))
        self.armature_force = float(
            controls.get("armature_force", self.armature_force)
        )
        self.armature = float(controls.get("armature", self.armature))
        self._apply_parameters(reset_state=False)
        self._restore_histories(self._position_history, state.get("position_history", {}))
        self._restore_histories(self._speed_history, state.get("speed_history", {}))
        self._restore_histories(self._accel_history, state.get("accel_history", {}))
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.world.update_kinematics()
        self._sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def build_panel(self, builder: Any, context: Any) -> None:
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), executor_choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_spring, spring_stiffness = builder.slider(
            "Spring stiffness", float(self.spring_stiffness), 0.0, 30.0
        )
        changed_damping, damping_coefficient = builder.slider(
            "Damping", float(self.damping_coefficient), 0.0, 8.0
        )
        changed_rest, rest_position = builder.slider(
            "Rest position", float(self.rest_position), -0.30, 0.30
        )
        changed_friction, coulomb_friction = builder.slider(
            "Coulomb friction", float(self.coulomb_friction), 0.0, 12.0
        )
        changed_hold, hold_force = builder.slider(
            "Hold force", float(self.hold_force), 0.0, 12.0
        )
        changed_slip, slip_force = builder.slider(
            "Slip force", float(self.slip_force), 0.0, 18.0
        )
        changed_armature_force, armature_force = builder.slider(
            "Armature drive force", float(self.armature_force), 0.0, 18.0
        )
        changed_armature, armature = builder.slider(
            "Armature", float(self.armature), 0.0, 12.0
        )
        if (
            changed_spring
            or changed_damping
            or changed_rest
            or changed_friction
            or changed_hold
            or changed_slip
            or changed_armature_force
            or changed_armature
        ):
            self.spring_stiffness = float(spring_stiffness)
            self.damping_coefficient = float(damping_coefficient)
            self.rest_position = float(rest_position)
            self.coulomb_friction = float(coulomb_friction)
            self.hold_force = float(hold_force)
            self.slip_force = float(slip_force)
            self.armature_force = float(armature_force)
            self.armature = float(armature)
            self.reset(clear_replay=True)

        if builder.button("Reset passive joints"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("comparison axis: passive joint parameter family")
        builder.text(
            "held fixed: World prismatic joints | gravity off | contacts off | "
            f"link mass {_MASS:.1f} | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: World multibody joints | gravity: off | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(
            "spring/rest/damping, Coulomb friction, drive forces, and armature "
            "are passive joint parameters"
        )
        for key in (
            "spring_only",
            "spring_damper",
            "stiction",
            "slip",
            "armature_reference",
            "armature_heavy",
        ):
            self._lane_text(builder, key)
        builder.plot_lines("Spring position", list(self._position_history["spring_only"]))
        builder.plot_lines("Damped position", list(self._position_history["spring_damper"]))
        builder.plot_lines("Spring energy", list(self._energy_history["spring_only"]))
        builder.plot_lines("Damped energy", list(self._energy_history["spring_damper"]))
        builder.plot_lines("Stiction position", list(self._position_history["stiction"]))
        builder.plot_lines("Slip position", list(self._position_history["slip"]))
        builder.plot_lines(
            "Armature reference",
            list(self._position_history["armature_reference"]),
        )
        builder.plot_lines("High armature", list(self._position_history["armature_heavy"]))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)

    def _lane_text(self, builder: Any, key: str) -> None:
        lane = next(lane for lane in self.lanes if lane.key == key)
        metrics = self._last_metrics.get(key)
        if metrics is None:
            metrics = {
                "position": _joint_scalar(lane.joint.position),
                "velocity": _joint_scalar(lane.joint.velocity),
                "acceleration": _joint_scalar(lane.joint.acceleration),
                "expected_acceleration": 0.0,
                "energy": self._lane_energy(lane),
                "status": self._lane_status(lane),
            }
        builder.text(
            f"{lane.label}: {metrics['status']} | "
            f"q {float(metrics['position']):.3f} m | "
            f"v {float(metrics['velocity']):.3f} m/s | "
            f"a {float(metrics['acceleration']):.3f}/"
            f"{float(metrics['expected_acceleration']):.3f} m/s^2 | "
            f"E {float(metrics['energy']):.3f}"
        )


def build() -> SceneSetup:
    controller = _RigidJointPassiveParameterVerifier()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Joint Passive Parameters", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_joint_passive_parameters_controller": controller,
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_timeline": {
                "signal_label": "Armature position gap",
                "signal": controller.replay_timeline_signal,
                "markers": controller.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_joint_passive_parameters",
    title="Rigid Joint Passive Parameters",
    category="World Rigid Body",
    summary=(
        "A contact-free multibody verifier for spring/rest, damping, Coulomb "
        "friction, and armature joint parameters."
    ),
    build=build,
)
