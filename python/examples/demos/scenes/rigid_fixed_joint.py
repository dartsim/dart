"""Rigid-body fixed-joint verifier for the DART 7 World facade."""

from __future__ import annotations

import math
from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_HISTORY = 180
_BASE_POSITION = np.array([0.0, 0.0, 1.0])
_PAYLOAD_OFFSET = np.array([0.85, 0.0, 0.0])
_BASE_HALF = np.array([0.16, 0.16, 0.16])
_PAYLOAD_HALF = np.array([0.22, 0.14, 0.14])


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _rotation_z(angle: float) -> np.ndarray:
    transform = np.eye(4)
    c = math.cos(angle)
    s = math.sin(angle)
    transform[0, 0] = c
    transform[0, 1] = -s
    transform[1, 0] = s
    transform[1, 1] = c
    return transform


def _angle_between(reference: np.ndarray, current: np.ndarray) -> float:
    delta = np.asarray(reference, dtype=float).T @ np.asarray(current, dtype=float)
    cosine = float(np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0))
    return float(math.acos(cosine))


class _RigidFixedJointVerifier:
    def __init__(self) -> None:
        self.perturbation = 0.18
        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, -9.81))

        self.base = self.world.add_rigid_body(
            "fixed_joint_base", position=tuple(_BASE_POSITION)
        )
        self.base.is_static = True

        self.payload = self.world.add_rigid_body(
            "fixed_joint_payload", position=tuple(_BASE_POSITION + _PAYLOAD_OFFSET)
        )
        self.payload.mass = 1.0
        self.payload.angular_velocity = (0.0, 0.0, 1.2)

        self.fixed_joint = self.world.add_joint(
            self.base,
            self.payload,
                        sx.JointSpec(
                name="fixed_joint_base_to_payload",
                type=sx.JointType.FIXED,
            )
        )
        self._captured_relative_transform = self._relative_transform()
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_fixed_joint_render")
        self.bridge.add_rigid_body_visual(
            self.base,
            dart.BoxShape(2.0 * _BASE_HALF),
            (0.32, 0.34, 0.38),
            name="fixed_joint_base_visual",
        )
        self.bridge.add_rigid_body_visual(
            self.payload,
            dart.BoxShape(2.0 * _PAYLOAD_HALF),
            (0.88, 0.42, 0.18),
            name="fixed_joint_payload_visual",
        )

        connector = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            "fixed_joint_connector_visual",
            _translation(_BASE_POSITION + 0.5 * _PAYLOAD_OFFSET),
        )
        connector.set_shape(dart.BoxShape(np.array([_PAYLOAD_OFFSET[0], 0.035, 0.035])))
        connector.create_visual_aspect().set_color([0.78, 0.78, 0.72])
        self.bridge.render_world.add_simple_frame(connector)

        self._translation_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._orientation_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._angular_speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=True)

    def _relative_transform(self) -> np.ndarray:
        base_transform = np.asarray(self.base.transform, dtype=float)
        payload_transform = np.asarray(self.payload.transform, dtype=float)
        return np.linalg.inv(base_transform) @ payload_transform

    def _target_payload_transform(self) -> np.ndarray:
        return np.asarray(self.base.transform, dtype=float) @ self._captured_relative_transform

    def _reset_body_state(self) -> None:
        self.base.transform = _translation(_BASE_POSITION)
        self.payload.transform = self._target_payload_transform()
        self.base.linear_velocity = (0.0, 0.0, 0.0)
        self.base.angular_velocity = (0.0, 0.0, 0.0)
        self.payload.linear_velocity = (0.0, 0.0, 0.0)
        self.payload.angular_velocity = (0.0, 0.0, 1.2)
        self.base.clear_force()
        self.base.clear_torque()
        self.payload.clear_force()
        self.payload.clear_torque()

    def reset(self, *, clear_replay: bool = False) -> None:
        self._reset_body_state()
        self.world.time = 0.0
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        self.world.update_kinematics()
        for history in (
            self._translation_error_history,
            self._orientation_error_history,
            self._speed_history,
            self._angular_speed_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def perturb(self) -> None:
        scale = max(0.0, min(0.35, float(self.perturbation)))
        target = self._target_payload_transform()
        perturbed = _rotation_z(2.0 * scale) @ target
        perturbed[:3, 3] = target[:3, 3] + np.array(
            [scale, -0.65 * scale, 0.45 * scale]
        )
        self.payload.transform = perturbed
        self.payload.linear_velocity = (1.0 * scale, -0.6 * scale, 0.4 * scale)
        self.payload.angular_velocity = (0.2 * scale, -0.1 * scale, 2.4 * scale)
        self.payload.clear_force()
        self.payload.clear_torque()
        self.world.update_kinematics()
        self._record_metrics()
        self._sync()

    def _record_metrics(self) -> None:
        relative = self._relative_transform()
        reference = self._captured_relative_transform
        translation_error = float(
            np.linalg.norm(relative[:3, 3] - reference[:3, 3])
        )
        orientation_error = _angle_between(reference[:3, :3], relative[:3, :3])
        speed = float(np.linalg.norm(np.asarray(self.payload.linear_velocity, dtype=float)))
        angular_speed = float(
            np.linalg.norm(np.asarray(self.payload.angular_velocity, dtype=float))
        )
        self._last_metrics = {
            "translation_error": translation_error,
            "orientation_error": orientation_error,
            "payload_speed": speed,
            "payload_angular_speed": angular_speed,
            "world_time": float(self.world.time),
            "joint_name": str(self.fixed_joint.name),
        }
        self._translation_error_history.append(translation_error)
        self._orientation_error_history.append(orientation_error)
        self._speed_history.append(speed)
        self._angular_speed_history.append(angular_speed)

    def _sync(self) -> None:
        self.bridge.sync()

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        translation_history = list(self._translation_error_history)
        orientation_history = list(self._orientation_error_history)
        translation_error = float(self._last_metrics["translation_error"])
        orientation_error = float(self._last_metrics["orientation_error"])
        payload_speed = float(self._last_metrics["payload_speed"])
        payload_angular_speed = float(self._last_metrics["payload_angular_speed"])
        captured_offset = float(
            np.linalg.norm(self._captured_relative_transform[:3, 3])
        )
        return {
            "row": "rigid_fixed_joint",
            "comparison_axis": "fixed_relative_transform_recovery",
            "solver": "sequential_rigid_joints",
            "constraint": "fixed_relative_transform",
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "joint_name": str(self.fixed_joint.name),
            "fixed_joint_count": float(self.world.num_joints),
            "held_fixed": {
                "base": "static",
                "captured_offset_m": captured_offset,
                "gravity_z": -9.81,
                "payload_mass": float(self.payload.mass),
                "solver": "Sequential rigid joints",
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "perturbation": float(self.perturbation),
            },
            "fixed_joint_translation_error": translation_error,
            "fixed_joint_orientation_error": orientation_error,
            "fixed_joint_payload_speed": payload_speed,
            "fixed_joint_payload_angular_speed": payload_angular_speed,
            "metrics": {
                "translation_error": translation_error,
                "orientation_error": orientation_error,
                "payload_speed": payload_speed,
                "payload_angular_speed": payload_angular_speed,
            },
            "history": {
                "samples": float(len(translation_history)),
                "max_translation_error": max(translation_history, default=0.0),
                "max_orientation_error": max(orientation_history, default=0.0),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "perturbation": float(self.perturbation),
            },
            "translation_error_history": list(self._translation_error_history),
            "orientation_error_history": list(self._orientation_error_history),
            "speed_history": list(self._speed_history),
            "angular_speed_history": list(self._angular_speed_history),
            "last_metrics": dict(self._last_metrics),
        }

    def replay_timeline_signal(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        history = snapshot.get("translation_error_history", [])
        if history:
            try:
                return float(history[-1])
            except (TypeError, ValueError):
                return 0.0
        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            try:
                return float(metrics.get("translation_error", 0.0))
            except (TypeError, ValueError):
                return 0.0
        return 0.0

    def replay_timeline_marker(self, snapshot: dict[str, Any] | None) -> float:
        if not isinstance(snapshot, dict):
            return 0.0
        history_thresholds = (
            ("translation_error_history", 0.010),
            ("orientation_error_history", 0.020),
            ("speed_history", 0.050),
            ("angular_speed_history", 0.050),
        )
        for history_key, threshold in history_thresholds:
            history = snapshot.get(history_key, [])
            try:
                if history and abs(float(history[-1])) >= threshold:
                    return 1.0
            except (TypeError, ValueError, IndexError):
                continue
        metrics = snapshot.get("last_metrics", {})
        if isinstance(metrics, dict):
            metric_thresholds = (
                ("translation_error", 0.010),
                ("orientation_error", 0.020),
                ("payload_speed", 0.050),
                ("payload_angular_speed", 0.050),
            )
            for metric_key, threshold in metric_thresholds:
                try:
                    if abs(float(metrics.get(metric_key, 0.0))) >= threshold:
                        return 1.0
                except (TypeError, ValueError):
                    continue
        return 0.0

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.perturbation = float(controls.get("perturbation", self.perturbation))
        self._restore_history(
            self._translation_error_history,
            state.get("translation_error_history", []),
        )
        self._restore_history(
            self._orientation_error_history,
            state.get("orientation_error_history", []),
        )
        self._restore_history(self._speed_history, state.get("speed_history", []))
        self._restore_history(
            self._angular_speed_history,
            state.get("angular_speed_history", []),
        )
        self._last_metrics = {
            str(key): float(value) if isinstance(value, (int, float)) else str(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self._sync()

    def _restore_history(self, history: deque[float], values: Any) -> None:
        history.clear()
        history.extend(float(value) for value in values)

    def build_panel(self, builder: Any, context: Any) -> None:
        changed, perturbation = builder.slider(
            "Perturbation", float(self.perturbation), 0.0, 0.35
        )
        if changed:
            self.perturbation = float(perturbation)
        if builder.button("Perturb payload"):
            self.perturb()
        builder.same_line()
        if builder.button("Reset fixed joint"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics or {}
        found_joint = self.world.get_joint(
            self.fixed_joint.name
        ) or self.fixed_joint
        captured_offset = float(
            np.linalg.norm(self._captured_relative_transform[:3, 3])
        )
        builder.separator()
        builder.text("comparison axis: fixed relative transform recovery")
        builder.text(
            "held fixed: sequential rigid joints | static base | payload mass "
            f"{float(self.payload.mass):.1f} | offset {captured_offset:.2f} m | "
            f"time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("constraint: fixed relative transform")
        builder.text(f"name: {found_joint.name}")
        builder.text(f"fixed joints: {self.world.num_joints}")
        builder.text(f"parent: {found_joint.parent_rigid_body.name}")
        builder.text(f"child: {found_joint.child_rigid_body.name}")
        builder.text(f"world time: {float(metrics.get('world_time', 0.0)):.3f} s")
        builder.text(
            "relative offset error: "
            f"{float(metrics.get('translation_error', 0.0)):.6f} m"
        )
        builder.text(
            "relative orientation error: "
            f"{float(metrics.get('orientation_error', 0.0)):.6f} rad"
        )
        builder.text(
            f"payload speed: {float(metrics.get('payload_speed', 0.0)):.4f} m/s"
        )
        builder.text(
            "payload angular speed: "
            f"{float(metrics.get('payload_angular_speed', 0.0)):.4f} rad/s"
        )
        builder.plot_lines("Offset error", list(self._translation_error_history))
        builder.plot_lines("Orientation error", list(self._orientation_error_history))
        builder.plot_lines("Payload speed", list(self._speed_history))
        builder.plot_lines("Payload angular speed", list(self._angular_speed_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    verifier = _RigidFixedJointVerifier()
    return SceneSetup(
        world=verifier.bridge.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.bridge.force_drag,
        renderable_provider=verifier.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Fixed Joint", verifier.build_panel)],
        info={
            "sx_world": verifier.world,
            "joint": verifier.fixed_joint,
            "base": verifier.base,
            "payload": verifier.payload,
            "rigid_fixed_joint_controller": verifier,
            CAPTURE_METRICS_INFO_KEY: verifier.capture_metrics,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_sync": verifier._sync,
            "replay_timeline": {
                "signal_label": "Fixed-joint offset error",
                "signal": verifier.replay_timeline_signal,
                "markers": verifier.replay_timeline_marker,
            },
        },
    )


SCENE = PythonDemoScene(
    id="rigid_fixed_joint",
    title="Rigid Fixed Joint",
    category="World Rigid Body",
    summary="Verifies that a fixed joint preserves the captured relative transform.",
    build=build,
)
