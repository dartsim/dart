"""Rigid-body one-DOF joint verifier for the DART 7 World facade."""

from __future__ import annotations

import math
from collections import deque
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.005
_HISTORY = 180
_BASE_HALF = np.array([0.16, 0.16, 0.16])
_HINGE_HALF = np.array([0.28, 0.12, 0.12])
_SLIDER_HALF = np.array([0.18, 0.18, 0.22])
_HINGE_ANCHOR = np.array([-0.85, 0.0, 1.0])
_SLIDER_ANCHOR = np.array([0.85, 0.0, 1.0])
_HINGE_RADIUS = 0.42
_SLIDER_INITIAL_TRAVEL = 0.55
_AXIS_Z = np.array([0.0, 0.0, 1.0])


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


def _transform(position: np.ndarray, yaw: float = 0.0) -> np.ndarray:
    transform = _rotation_z(yaw)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


class _RigidOneDofJointVerifier:
    def __init__(self) -> None:
        self.perturbation = 0.18
        self.world = sx.World(time_step=_TIME_STEP, gravity=(0.0, 0.0, 0.0))

        self.hinge_base = self.world.add_rigid_body(
            "hinge_base", position=tuple(_HINGE_ANCHOR)
        )
        self.hinge_base.is_static = True
        self.hinge_payload = self.world.add_rigid_body(
            "hinge_payload",
            position=tuple(_HINGE_ANCHOR + np.array([_HINGE_RADIUS, 0.0, 0.0])),
        )
        self.hinge_payload.mass = 1.0
        self.hinge_payload.angular_velocity = (0.0, 0.0, 1.4)
        self.hinge_joint = self.world.add_rigid_body_revolute_joint(
            "hinge_base_to_payload",
            self.hinge_base,
            self.hinge_payload,
            axis=tuple(_AXIS_Z),
        )

        self.slider_base = self.world.add_rigid_body(
            "slider_base", position=tuple(_SLIDER_ANCHOR)
        )
        self.slider_base.is_static = True
        self.slider_payload = self.world.add_rigid_body(
            "slider_payload",
            position=tuple(_SLIDER_ANCHOR + _SLIDER_INITIAL_TRAVEL * _AXIS_Z),
        )
        self.slider_payload.mass = 1.0
        self.slider_payload.linear_velocity = (0.0, 0.0, 0.45)
        self.slider_joint = self.world.add_rigid_body_prismatic_joint(
            "slider_base_to_payload",
            self.slider_base,
            self.slider_payload,
            axis=tuple(_AXIS_Z),
        )

        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_limited_joints_render")
        self._add_visuals()

        self._hinge_radius_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._hinge_z_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._hinge_yaw_history: deque[float] = deque(maxlen=_HISTORY)
        self._slider_orthogonal_error_history: deque[float] = deque(maxlen=_HISTORY)
        self._slider_axis_travel_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=True)

    def _add_visuals(self) -> None:
        self.bridge.add_rigid_body_visual(
            self.hinge_base,
            dart.BoxShape(_full(_BASE_HALF)),
            (0.30, 0.33, 0.38),
            name="hinge_base_visual",
        )
        self.bridge.add_rigid_body_visual(
            self.hinge_payload,
            dart.BoxShape(_full(_HINGE_HALF)),
            (0.88, 0.34, 0.18),
            name="hinge_payload_visual",
        )
        self.bridge.add_rigid_body_visual(
            self.slider_base,
            dart.BoxShape(_full(_BASE_HALF)),
            (0.30, 0.33, 0.38),
            name="slider_base_visual",
        )
        self.bridge.add_rigid_body_visual(
            self.slider_payload,
            dart.BoxShape(_full(_SLIDER_HALF)),
            (0.22, 0.54, 0.86),
            name="slider_payload_visual",
        )

        hinge_axis = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            "hinge_axis_visual",
            _translation(_HINGE_ANCHOR + np.array([0.0, 0.0, 0.35])),
        )
        hinge_axis.set_shape(dart.CylinderShape(0.025, 0.7))
        hinge_axis.create_visual_aspect().set_color([0.96, 0.78, 0.22])
        self.bridge.render_world.add_simple_frame(hinge_axis)

        slider_axis = dart.SimpleFrame(
            dart.gui.world_render_frame(),
            "slider_axis_visual",
            _translation(_SLIDER_ANCHOR + np.array([0.0, 0.0, 0.55])),
        )
        slider_axis.set_shape(dart.CylinderShape(0.02, 1.1))
        slider_axis.create_visual_aspect().set_color([0.22, 0.78, 0.58])
        self.bridge.render_world.add_simple_frame(slider_axis)

    def _reset_body_state(self) -> None:
        self.hinge_base.transform = _translation(_HINGE_ANCHOR)
        self.hinge_payload.transform = _translation(
            _HINGE_ANCHOR + np.array([_HINGE_RADIUS, 0.0, 0.0])
        )
        self.hinge_payload.linear_velocity = (0.0, 0.0, 0.0)
        self.hinge_payload.angular_velocity = (0.0, 0.0, 1.4)

        self.slider_base.transform = _translation(_SLIDER_ANCHOR)
        self.slider_payload.transform = _translation(
            _SLIDER_ANCHOR + _SLIDER_INITIAL_TRAVEL * _AXIS_Z
        )
        self.slider_payload.linear_velocity = (0.0, 0.0, 0.45)
        self.slider_payload.angular_velocity = (0.0, 0.0, 0.0)

        for body in (
            self.hinge_base,
            self.hinge_payload,
            self.slider_base,
            self.slider_payload,
        ):
            body.clear_force()
            body.clear_torque()

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
            self._hinge_radius_error_history,
            self._hinge_z_error_history,
            self._hinge_yaw_history,
            self._slider_orthogonal_error_history,
            self._slider_axis_travel_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self._sync()

    def perturb(self) -> None:
        scale = max(0.0, min(0.35, float(self.perturbation)))
        hinge_position = _HINGE_ANCHOR + np.array(
            [_HINGE_RADIUS + scale, -0.55 * scale, 0.70 * scale]
        )
        self.hinge_payload.transform = _transform(hinge_position, yaw=1.8 * scale)
        self.hinge_payload.linear_velocity = (0.35 * scale, -0.4 * scale, 0.2 * scale)
        self.hinge_payload.angular_velocity = (0.0, 0.0, 1.4)

        slider_position = _SLIDER_ANCHOR + np.array(
            [0.80 * scale, -0.65 * scale, _SLIDER_INITIAL_TRAVEL + 0.25 * scale]
        )
        self.slider_payload.transform = _translation(slider_position)
        self.slider_payload.linear_velocity = (0.35 * scale, -0.2 * scale, 0.45)
        self.slider_payload.angular_velocity = (0.0, 0.0, 0.0)

        for body in (self.hinge_payload, self.slider_payload):
            body.clear_force()
            body.clear_torque()
        self.world.update_kinematics()
        self._record_metrics()
        self._sync()

    def _sample(self) -> dict[str, float | str]:
        hinge_position = np.asarray(self.hinge_payload.translation, dtype=float).reshape(3)
        hinge_delta = hinge_position - _HINGE_ANCHOR
        hinge_radius = float(np.linalg.norm(hinge_delta[:2]))
        hinge_z_error = abs(float(hinge_delta[2]))
        hinge_transform = np.asarray(self.hinge_payload.transform, dtype=float)
        hinge_yaw = float(math.atan2(hinge_transform[1, 0], hinge_transform[0, 0]))

        slider_position = np.asarray(self.slider_payload.translation, dtype=float).reshape(3)
        slider_delta = slider_position - _SLIDER_ANCHOR
        slider_axis_travel = float(np.dot(slider_delta, _AXIS_Z))
        slider_orthogonal = slider_delta - slider_axis_travel * _AXIS_Z
        slider_axis_speed = float(
            np.dot(np.asarray(self.slider_payload.linear_velocity, dtype=float), _AXIS_Z)
        )

        return {
            "hinge_radius_error": abs(hinge_radius - _HINGE_RADIUS),
            "hinge_z_error": hinge_z_error,
            "hinge_yaw": hinge_yaw,
            "hinge_angular_speed": float(
                np.dot(np.asarray(self.hinge_payload.angular_velocity, dtype=float), _AXIS_Z)
            ),
            "slider_orthogonal_error": float(np.linalg.norm(slider_orthogonal)),
            "slider_axis_travel": slider_axis_travel,
            "slider_axis_speed": slider_axis_speed,
            "world_time": float(self.world.time),
        }

    def _record_metrics(self) -> None:
        self._last_metrics = self._sample()
        self._hinge_radius_error_history.append(
            float(self._last_metrics["hinge_radius_error"])
        )
        self._hinge_z_error_history.append(float(self._last_metrics["hinge_z_error"]))
        self._hinge_yaw_history.append(float(self._last_metrics["hinge_yaw"]))
        self._slider_orthogonal_error_history.append(
            float(self._last_metrics["slider_orthogonal_error"])
        )
        self._slider_axis_travel_history.append(
            float(self._last_metrics["slider_axis_travel"])
        )

    def _sync(self) -> None:
        self.bridge.sync()

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self._sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "perturbation": float(self.perturbation),
            },
            "hinge_radius_error_history": list(self._hinge_radius_error_history),
            "hinge_z_error_history": list(self._hinge_z_error_history),
            "hinge_yaw_history": list(self._hinge_yaw_history),
            "slider_orthogonal_error_history": list(
                self._slider_orthogonal_error_history
            ),
            "slider_axis_travel_history": list(self._slider_axis_travel_history),
            "last_metrics": dict(self._last_metrics),
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.perturbation = float(controls.get("perturbation", self.perturbation))
        self._restore_history(
            self._hinge_radius_error_history,
            state.get("hinge_radius_error_history", []),
        )
        self._restore_history(
            self._hinge_z_error_history,
            state.get("hinge_z_error_history", []),
        )
        self._restore_history(
            self._hinge_yaw_history,
            state.get("hinge_yaw_history", []),
        )
        self._restore_history(
            self._slider_orthogonal_error_history,
            state.get("slider_orthogonal_error_history", []),
        )
        self._restore_history(
            self._slider_axis_travel_history,
            state.get("slider_axis_travel_history", []),
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
        if builder.button("Perturb joints"):
            self.perturb()
        builder.same_line()
        if builder.button("Reset one-DOF joints"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics or {}
        builder.separator()
        builder.text("revolute: locked anchor, free z-axis spin")
        builder.text(f"hinge joint: {self.hinge_joint.name}")
        builder.text(
            "hinge radius error: "
            f"{float(metrics.get('hinge_radius_error', 0.0)):.6f} m"
        )
        builder.text(
            f"hinge z error: {float(metrics.get('hinge_z_error', 0.0)):.6f} m"
        )
        builder.text(
            f"hinge yaw: {float(metrics.get('hinge_yaw', 0.0)):.3f} rad | "
            f"axis speed {float(metrics.get('hinge_angular_speed', 0.0)):.3f} rad/s"
        )
        builder.text("prismatic: locked lateral offset, free z-axis slide")
        builder.text(f"slider joint: {self.slider_joint.name}")
        builder.text(
            "slider orthogonal error: "
            f"{float(metrics.get('slider_orthogonal_error', 0.0)):.6f} m"
        )
        builder.text(
            f"slider axis travel: {float(metrics.get('slider_axis_travel', 0.0)):.3f} m | "
            f"axis speed {float(metrics.get('slider_axis_speed', 0.0)):.3f} m/s"
        )
        builder.text(f"rigid-body joints: {self.world.num_rigid_body_joints}")
        builder.text(f"world time: {float(metrics.get('world_time', 0.0)):.3f} s")
        builder.plot_lines("Hinge radius error", list(self._hinge_radius_error_history))
        builder.plot_lines("Hinge z error", list(self._hinge_z_error_history))
        builder.plot_lines("Hinge yaw", list(self._hinge_yaw_history))
        builder.plot_lines(
            "Slider xy error", list(self._slider_orthogonal_error_history)
        )
        builder.plot_lines("Slider travel", list(self._slider_axis_travel_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    verifier = _RigidOneDofJointVerifier()
    return SceneSetup(
        world=verifier.bridge.render_world,
        pre_step=verifier.pre_step,
        force_drag=verifier.bridge.force_drag,
        renderable_provider=verifier.bridge.renderable_provider,
        panels=[ScenePanel("Rigid One-DOF Joints", verifier.build_panel)],
        info={
            "sx_world": verifier.world,
            "hinge_joint": verifier.hinge_joint,
            "slider_joint": verifier.slider_joint,
            "hinge_base": verifier.hinge_base,
            "hinge_payload": verifier.hinge_payload,
            "slider_base": verifier.slider_base,
            "slider_payload": verifier.slider_payload,
            "rigid_one_dof_joint_controller": verifier,
            "replay_capture_state": verifier.capture_replay_state,
            "replay_restore_state": verifier.restore_replay_state,
            "replay_sync": verifier._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_limited_joints",
    title="Rigid One-DOF Joints",
    category="World Rigid Body",
    summary="Verifies revolute and prismatic rigid-body joint constraint rows.",
    build=build,
)
