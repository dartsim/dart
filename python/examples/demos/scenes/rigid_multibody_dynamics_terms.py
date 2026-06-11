"""Generalized dynamics-term verifier for DART 7 World multibodies."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.003
_HISTORY = 180
_LINK_LENGTH = 0.55
_BASE_MASS = 0.85
_DISTAL_MASS = 0.65
_INITIAL_Q = (0.42, -0.62)
_INITIAL_QDOT = (0.45, -0.32)
_GRAVITY = 9.81


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _box(size: tuple[float, float, float]) -> dart.BoxShape:
    return dart.BoxShape(np.asarray(size, dtype=float))


def _joint_scalar(value: object) -> float:
    values = np.asarray(value, dtype=float).reshape(-1)
    return float(values[0]) if values.size else 0.0


def _bar_inertia(mass: float) -> tuple[tuple[float, float, float], ...]:
    transverse = float(mass) * (_LINK_LENGTH**2) / 12.0
    axial = 0.5 * float(mass) * (0.035**2)
    return ((axial, 0.0, 0.0), (0.0, transverse, 0.0), (0.0, 0.0, transverse))


@dataclass
class _DynamicsLane:
    key: str
    label: str
    anchor: np.ndarray
    robot: Any
    links: list[Any]
    joints: list[Any]
    target_pattern: np.ndarray
    impulse_pattern: np.ndarray
    color: tuple[float, float, float]
    distal_mass_scale: float = 1.0
    link_frames: list[Any] = field(default_factory=list)


class _RigidMultibodyDynamicsTerms:
    def __init__(self) -> None:
        self.executor_index = 0
        self.target_acceleration = 2.2
        self.joint_impulse = 3.0
        self.heavy_distal_mass_scale = 4.0
        self.gravity_scale = 1.0
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
                key="single_hinge",
                label="Single hinge",
                dofs=1,
                anchor=(-1.25, 0.42, 0.86),
                target_pattern=(1.0,),
                impulse_pattern=(1.0,),
                color=(0.24, 0.58, 0.88),
            ),
            self._add_lane(
                key="coupled_two_link",
                label="Coupled two-link",
                dofs=2,
                anchor=(-0.30, -0.42, 0.86),
                target_pattern=(1.0, -0.45),
                impulse_pattern=(1.0, 0.0),
                color=(0.48, 0.45, 0.82),
            ),
            self._add_lane(
                key="heavy_distal",
                label="Heavy distal",
                dofs=2,
                anchor=(0.95, -0.42, 0.86),
                target_pattern=(1.0, -0.45),
                impulse_pattern=(1.0, 0.0),
                color=(0.84, 0.48, 0.20),
                distal_mass_scale=self.heavy_distal_mass_scale,
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(
            self.world, name="rigid_multibody_dynamics_terms"
        )
        self._add_visuals()

        self._residual_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._response_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._tau_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._coupling_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self.reset(clear_replay=True)

    def _add_lane(
        self,
        *,
        key: str,
        label: str,
        dofs: int,
        anchor: tuple[float, float, float],
        target_pattern: tuple[float, ...],
        impulse_pattern: tuple[float, ...],
        color: tuple[float, float, float],
        distal_mass_scale: float = 1.0,
    ) -> _DynamicsLane:
        robot = self.world.add_multibody(f"{key}_robot")
        base = robot.add_link(f"{key}_base")
        parent = base
        links: list[Any] = []
        joints: list[Any] = []
        for index in range(dofs):
            link = robot.add_link(
                f"{key}_link{index}",
                parent=parent,
                joint=sx.JointSpec(
                    name=f"{key}_joint{index}",
                    type=sx.JointType.REVOLUTE,
                    axis=(0.0, 1.0, 0.0),
                    transform_from_parent=_translation((_LINK_LENGTH, 0.0, 0.0)),
                ),
            )
            mass = _BASE_MASS
            if index == 1:
                mass = _DISTAL_MASS * float(distal_mass_scale)
            link.mass = mass
            link.inertia = _bar_inertia(mass)
            links.append(link)
            joints.append(link.parent_joint)
            parent = link
        return _DynamicsLane(
            key=key,
            label=label,
            anchor=np.asarray(anchor, dtype=float),
            robot=robot,
            links=links,
            joints=joints,
            target_pattern=np.asarray(target_pattern, dtype=float),
            impulse_pattern=np.asarray(impulse_pattern, dtype=float),
            color=color,
            distal_mass_scale=float(distal_mass_scale),
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
            self._add_static_visual(
                f"{lane.key}_base_marker",
                dart.SphereShape(0.055),
                lane.anchor,
                (0.30, 0.32, 0.36),
            )
            self._add_static_visual(
                f"{lane.key}_impulse_axis",
                _box((0.46, 0.020, 0.020)),
                lane.anchor + np.array([0.26, 0.0, -0.20]),
                (0.94, 0.82, 0.25),
            )
            lane.link_frames.clear()
            for index, _link in enumerate(lane.links):
                shade = 0.10 * index
                frame = self._add_static_visual(
                    f"{lane.key}_link{index}_visual",
                    _box((_LINK_LENGTH, 0.065, 0.065)),
                    lane.anchor,
                    (
                        min(1.0, lane.color[0] + shade),
                        min(1.0, lane.color[1] + shade),
                        min(1.0, lane.color[2] + shade),
                    ),
                )
                lane.link_frames.append(frame)
        self._sync()

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _clamp_controls(self) -> None:
        self.executor_index = max(
            0, min(int(self.executor_index), len(self._executors) - 1)
        )
        self.target_acceleration = float(
            np.clip(self.target_acceleration, 0.2, 5.0)
        )
        self.joint_impulse = float(np.clip(self.joint_impulse, 0.2, 8.0))
        self.heavy_distal_mass_scale = float(
            np.clip(self.heavy_distal_mass_scale, 1.2, 8.0)
        )
        self.gravity_scale = float(np.clip(self.gravity_scale, 0.0, 1.5))

    def _set_lane_masses(self) -> None:
        for lane in self.lanes:
            for index, link in enumerate(lane.links):
                mass = _BASE_MASS
                if index == 1:
                    scale = (
                        self.heavy_distal_mass_scale
                        if lane.key == "heavy_distal"
                        else lane.distal_mass_scale
                    )
                    mass = _DISTAL_MASS * float(scale)
                link.mass = mass
                link.inertia = _bar_inertia(mass)

    def _reset_lane_state(self, lane: _DynamicsLane) -> None:
        for index, joint in enumerate(lane.joints):
            joint.position = [_INITIAL_Q[index]]
            joint.velocity = [_INITIAL_QDOT[index]]
            joint.force = [0.0]

    def _target(self, lane: _DynamicsLane) -> np.ndarray:
        return lane.target_pattern * float(self.target_acceleration)

    def _impulse(self, lane: _DynamicsLane) -> np.ndarray:
        return lane.impulse_pattern * float(self.joint_impulse)

    def _apply_parameters(self, *, reset_state: bool) -> None:
        self._clamp_controls()
        self.world.gravity = (0.0, 0.0, -_GRAVITY * float(self.gravity_scale))
        self._set_lane_masses()
        if reset_state:
            for lane in self.lanes:
                self._reset_lane_state(lane)
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
            *self._residual_history.values(),
            *self._response_history.values(),
            *self._tau_history.values(),
            self._coupling_history,
            self._step_ms_history,
        ):
            history.clear()
        terms = self._prepare_terms_and_forces()
        self._record_metrics(terms, use_joint_acceleration=False)
        self._sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _prepare_terms_and_forces(self) -> dict[str, dict[str, np.ndarray]]:
        terms: dict[str, dict[str, np.ndarray]] = {}
        for lane in self.lanes:
            target = self._target(lane)
            impulse = self._impulse(lane)
            mass_matrix = np.asarray(lane.robot.mass_matrix, dtype=float)
            inverse_mass = np.asarray(lane.robot.inverse_mass_matrix, dtype=float)
            coriolis = np.asarray(lane.robot.coriolis_forces, dtype=float)
            gravity = np.asarray(lane.robot.gravity_forces, dtype=float)
            bias = np.asarray(lane.robot.coriolis_and_gravity_forces, dtype=float)
            tau = np.asarray(lane.robot.compute_inverse_dynamics(target), dtype=float)
            response = np.asarray(
                lane.robot.compute_impulse_response(impulse), dtype=float
            )
            for joint, value in zip(lane.joints, tau):
                joint.force = [float(value)]
            terms[lane.key] = {
                "target": target,
                "impulse": impulse,
                "mass_matrix": mass_matrix,
                "inverse_mass": inverse_mass,
                "coriolis": coriolis,
                "gravity": gravity,
                "bias": bias,
                "tau": tau,
                "response": response,
            }
        return terms

    def pre_step(self) -> None:
        terms = self._prepare_terms_and_forces()
        self.world.step(self._executor())
        self._record_metrics(terms, use_joint_acceleration=True)
        self._sync()

    def _lane_acceleration(
        self, lane: _DynamicsLane, terms: dict[str, np.ndarray], *, use_joint_acceleration: bool
    ) -> np.ndarray:
        if not use_joint_acceleration:
            return np.asarray(terms["target"], dtype=float)
        return np.array([_joint_scalar(joint.acceleration) for joint in lane.joints])

    def _record_metrics(
        self,
        terms: dict[str, dict[str, np.ndarray]],
        *,
        use_joint_acceleration: bool,
    ) -> None:
        metrics_by_key: dict[str, dict[str, float | str]] = {}
        for lane in self.lanes:
            lane_terms = terms[lane.key]
            mass_matrix = lane_terms["mass_matrix"]
            inverse_mass = lane_terms["inverse_mass"]
            target = lane_terms["target"]
            impulse = lane_terms["impulse"]
            response = lane_terms["response"]
            tau = lane_terms["tau"]
            bias = lane_terms["bias"]
            acceleration = self._lane_acceleration(
                lane, lane_terms, use_joint_acceleration=use_joint_acceleration
            )
            dofs = int(target.size)
            identity_error = 0.0
            condition = 0.0
            coupling = 0.0
            if dofs:
                identity_error = float(
                    np.linalg.norm(mass_matrix @ inverse_mass - np.eye(dofs))
                )
                condition = float(np.linalg.cond(mass_matrix))
            if dofs > 1:
                coupling = float(mass_matrix[0, 1])
            dynamics_residual = float(
                np.linalg.norm(mass_matrix @ acceleration + bias - tau)
            )
            inverse_dynamics_residual = float(
                np.linalg.norm(mass_matrix @ target + bias - tau)
            )
            impulse_residual = float(np.linalg.norm(mass_matrix @ response - impulse))
            acceleration_error = float(np.linalg.norm(acceleration - target))
            response_norm = float(np.linalg.norm(response))
            tau_norm = float(np.linalg.norm(tau))
            metrics = {
                "status": self._lane_status(lane),
                "dofs": float(dofs),
                "mass_diag0": float(mass_matrix[0, 0]) if dofs else 0.0,
                "mass_diag1": float(mass_matrix[1, 1]) if dofs > 1 else 0.0,
                "inverse_diag0": float(inverse_mass[0, 0]) if dofs else 0.0,
                "coupling": coupling,
                "condition": condition,
                "identity_error": identity_error,
                "coriolis_norm": float(np.linalg.norm(lane_terms["coriolis"])),
                "gravity_norm": float(np.linalg.norm(lane_terms["gravity"])),
                "tau_norm": tau_norm,
                "response_norm": response_norm,
                "response0": float(response[0]) if dofs else 0.0,
                "response1": float(response[1]) if dofs > 1 else 0.0,
                "dynamics_residual": dynamics_residual,
                "inverse_dynamics_residual": inverse_dynamics_residual,
                "impulse_residual": impulse_residual,
                "acceleration_error": acceleration_error,
                "q0": _joint_scalar(lane.joints[0].position),
                "qdot0": _joint_scalar(lane.joints[0].velocity),
            }
            metrics_by_key[lane.key] = metrics
            self._residual_history[lane.key].append(dynamics_residual)
            self._response_history[lane.key].append(response_norm)
            self._tau_history[lane.key].append(tau_norm)
        self._coupling_history.append(
            abs(float(metrics_by_key["coupled_two_link"]["coupling"]))
        )
        self._step_ms_history.append(self._step_profile_ms())
        self._last_metrics = metrics_by_key

    def _lane_status(self, lane: _DynamicsLane) -> str:
        if lane.key == "single_hinge":
            return "scalar M, inverse dynamics, impulse"
        if lane.key == "heavy_distal":
            return "same command with larger distal mass"
        return "two-DOF coupling and off-diagonal M"

    def _sync(self) -> None:
        for lane in self.lanes:
            visual_offset = _translation(lane.anchor)
            for link, frame in zip(lane.links, lane.link_frames):
                transform = (
                    visual_offset
                    @ np.asarray(link.transform, dtype=float)
                    @ _translation((-0.5 * _LINK_LENGTH, 0.0, 0.0))
                )
                frame.set_transform(transform)
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        self._sync()
        return self.bridge.renderable_provider()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "target_acceleration": float(self.target_acceleration),
                "joint_impulse": float(self.joint_impulse),
                "heavy_distal_mass_scale": float(self.heavy_distal_mass_scale),
                "gravity_scale": float(self.gravity_scale),
            },
            "state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "time": float(self.world.time),
            "residual_history": {
                key: list(values) for key, values in self._residual_history.items()
            },
            "response_history": {
                key: list(values) for key, values in self._response_history.items()
            },
            "tau_history": {
                key: list(values) for key, values in self._tau_history.items()
            },
            "coupling_history": list(self._coupling_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def restore_replay_state(self, snapshot: dict[str, Any]) -> None:
        controls = snapshot.get("controls", {})
        self.executor_index = int(controls.get("executor_index", self.executor_index))
        self.target_acceleration = float(
            controls.get("target_acceleration", self.target_acceleration)
        )
        self.joint_impulse = float(controls.get("joint_impulse", self.joint_impulse))
        self.heavy_distal_mass_scale = float(
            controls.get("heavy_distal_mass_scale", self.heavy_distal_mass_scale)
        )
        self.gravity_scale = float(controls.get("gravity_scale", self.gravity_scale))
        self._apply_parameters(reset_state=False)
        try:
            self.world.state_vector = np.asarray(snapshot["state"], dtype=float)
            self.world.time = float(snapshot.get("time", self.world.time))
        except Exception:  # noqa: BLE001
            pass

        def _restore_dict(
            key: str, target: dict[str, deque[float]]
        ) -> None:
            values_by_lane = snapshot.get(key, {})
            for lane_key, values in target.items():
                values.clear()
                for value in values_by_lane.get(lane_key, []):
                    values.append(float(value))

        _restore_dict("residual_history", self._residual_history)
        _restore_dict("response_history", self._response_history)
        _restore_dict("tau_history", self._tau_history)
        self._coupling_history.clear()
        for value in snapshot.get("coupling_history", []):
            self._coupling_history.append(float(value))
        self._step_ms_history.clear()
        for value in snapshot.get("step_ms_history", []):
            self._step_ms_history.append(float(value))
        self._last_metrics = {
            key: dict(value)
            for key, value in snapshot.get("last_metrics", {}).items()
        }
        self.world.update_kinematics()
        self._sync()

    def _lane_text(self, builder: Any, lane: _DynamicsLane) -> None:
        metrics = self._last_metrics.get(lane.key)
        if metrics is None:
            return
        builder.text(
            f"{lane.label}: {metrics['status']} | dofs {int(metrics['dofs'])} | "
            f"M00 {float(metrics['mass_diag0']):.3f} | "
            f"M01 {float(metrics['coupling']):.3f} | "
            f"cond {float(metrics['condition']):.2f}"
        )
        builder.text(
            f"tau ||{float(metrics['tau_norm']):.3f}|| | "
            f"dqdot ||{float(metrics['response_norm']):.3f}|| | "
            f"res {float(metrics['dynamics_residual']):.2e} | "
            f"imp res {float(metrics['impulse_residual']):.2e}"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_executor, executor_index = builder.select(
            "Executor",
            int(self.executor_index),
            [label for label, _executor in self._executors],
        )
        changed_accel, target_acceleration = builder.slider(
            "Target acceleration", float(self.target_acceleration), 0.2, 5.0
        )
        changed_impulse, joint_impulse = builder.slider(
            "Joint impulse", float(self.joint_impulse), 0.2, 8.0
        )
        changed_mass, heavy_mass = builder.slider(
            "Heavy distal mass scale",
            float(self.heavy_distal_mass_scale),
            1.2,
            8.0,
        )
        changed_gravity, gravity_scale = builder.slider(
            "Gravity scale", float(self.gravity_scale), 0.0, 1.5
        )
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_accel:
            self.target_acceleration = float(target_acceleration)
        if changed_impulse:
            self.joint_impulse = float(joint_impulse)
        if changed_mass:
            self.heavy_distal_mass_scale = float(heavy_mass)
        if changed_gravity:
            self.gravity_scale = float(gravity_scale)
        if (
            changed_executor
            or changed_accel
            or changed_impulse
            or changed_mass
            or changed_gravity
        ):
            self.reset(clear_replay=True)

        if builder.button("Reset dynamics terms"):
            self.reset(clear_replay=True)

        builder.separator()
        builder.text("solver: World multibody dynamics | contacts: off")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text("tau = M qddot + C + g; impulse response is joint-space M^-1 f")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        builder.plot_lines(
            "Single hinge residual", list(self._residual_history["single_hinge"])
        )
        builder.plot_lines(
            "Coupled residual", list(self._residual_history["coupled_two_link"])
        )
        builder.plot_lines(
            "Heavy response", list(self._response_history["heavy_distal"])
        )
        builder.plot_lines("Off-diagonal coupling", list(self._coupling_history))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidMultibodyDynamicsTerms()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        renderable_provider=controller.renderable_provider,
        panels=[ScenePanel("Rigid Multibody Dynamics Terms", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_multibody_dynamics_terms_controller": controller,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_sync": controller._sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_multibody_dynamics_terms",
    title="Rigid Multibody Dynamics Terms",
    category="World Rigid Body",
    summary=(
        "Shows mass-matrix, inverse-dynamics, and joint-space impulse-response "
        "diagnostics for contact-free fixed-base multibodies."
    ),
    build=build,
)
