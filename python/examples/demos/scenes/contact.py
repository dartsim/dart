"""Rigid multibody-link contact verifier for the DART 7 World."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.002
_HISTORY = 180
_FOOT_RADIUS = 0.20
_PUSH_RADIUS = 0.16
_GROUND_HALF = np.array([2.60, 1.55, 0.25])
_GROUND_TOP = -0.50
_REST_Z = _GROUND_TOP + _FOOT_RADIUS
_DROP_ANCHOR = np.array([-1.10, -0.78, 0.0])
_SLIDE_ANCHOR = np.array([-0.95, 0.18, 0.0])
_PUSH_ANCHOR = np.array([0.86, 0.78, _GROUND_TOP + _PUSH_RADIUS])
_SLIDE_START_X = -0.34
_PUSH_START_X = -0.34
_TARGET_START_X = 0.02
_STRIKER_MASS = 2.0
_TARGET_MASS = 1.0


def _translation(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _shape_box(half_extents: np.ndarray) -> dart.BoxShape:
    return dart.BoxShape(2.0 * np.asarray(half_extents, dtype=float))


def _joint_scalar(value: object) -> float:
    values = np.asarray(value, dtype=float).reshape(-1)
    return float(values[0]) if values.size else 0.0


def _set_joint_scalar(joint: object, attr: str, value: float) -> None:
    current = np.asarray(getattr(joint, attr), dtype=float)
    setattr(joint, attr, np.full(current.shape, float(value)))


@dataclass
class _LinkContactState:
    drop_joint: Any
    drop_link: Any
    slide_vertical_joint: Any
    slide_horizontal_joint: Any
    slide_link: Any
    push_joint: Any
    push_link: Any
    target: Any


class _RigidLinkContact:
    def __init__(self) -> None:
        self.executor_index = 0
        self.ground_friction = 0.78
        self.ground_restitution = 0.55
        self.drop_height = 0.22
        self.slide_speed = 1.0
        self.push_speed = 0.82
        self._executors: list[tuple[str, Any]] = [("Sequential", sx.SequentialExecutor())]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(
            time_step=_TIME_STEP,
            gravity=(0.0, 0.0, -9.81),
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        self.world.step_profiling_enabled = True
        self.ground = self._add_ground()
        self.state = self._add_contact_lanes()
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_link_contact_render")
        self._add_visuals()

        self._drop_height_history: deque[float] = deque(maxlen=_HISTORY)
        self._drop_speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._drop_contact_history: deque[float] = deque(maxlen=_HISTORY)
        self._slide_speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._slide_travel_history: deque[float] = deque(maxlen=_HISTORY)
        self._slide_contact_history: deque[float] = deque(maxlen=_HISTORY)
        self._push_target_travel_history: deque[float] = deque(maxlen=_HISTORY)
        self._push_target_speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._push_contact_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._drop_max_upward_velocity = 0.0
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=True)

    def _add_ground(self) -> Any:
        ground = self.world.add_rigid_body(
            "link_contact_ground",
            position=(0.0, 0.0, _GROUND_TOP - float(_GROUND_HALF[2])),
        )
        ground.is_static = True
        ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
        return ground

    def _add_contact_lanes(self) -> _LinkContactState:
        drop_robot = self.world.add_multibody("drop_link_robot")
        drop_base = drop_robot.add_link("drop_base")
        drop_link = drop_robot.add_link(
            "drop_link",
            parent=drop_base,
            joint=sx.JointSpec(
                name="drop_slider",
                type=sx.JointType.PRISMATIC,
                axis=(0.0, 0.0, 1.0),
                transform_from_parent=_translation(_DROP_ANCHOR),
            ),
        )
        drop_link.mass = 1.0
        drop_link.set_collision_shape(sx.CollisionShape.sphere(_FOOT_RADIUS))

        slide_robot = self.world.add_multibody("slide_link_robot")
        slide_base = slide_robot.add_link("slide_base")
        carrier = slide_robot.add_link(
            "slide_carrier",
            parent=slide_base,
            joint=sx.JointSpec(
                name="vertical_support",
                type=sx.JointType.PRISMATIC,
                axis=(0.0, 0.0, 1.0),
                transform_from_parent=_translation(_SLIDE_ANCHOR),
            ),
        )
        carrier.mass = 0.1
        slide_link = slide_robot.add_link(
            "slide_link",
            parent=carrier,
            joint=sx.JointSpec(
                name="horizontal_slide",
                type=sx.JointType.PRISMATIC,
                axis=(1.0, 0.0, 0.0),
            ),
        )
        slide_link.mass = 1.0
        slide_link.set_collision_shape(sx.CollisionShape.sphere(_FOOT_RADIUS))

        push_robot = self.world.add_multibody("push_link_robot")
        push_base = push_robot.add_link("push_base")
        push_link = push_robot.add_link(
            "push_link",
            parent=push_base,
            joint=sx.JointSpec(
                name="push_rail",
                type=sx.JointType.PRISMATIC,
                axis=(1.0, 0.0, 0.0),
                transform_from_parent=_translation(_PUSH_ANCHOR),
            ),
        )
        push_link.mass = _STRIKER_MASS
        push_link.set_collision_shape(sx.CollisionShape.sphere(_PUSH_RADIUS))

        target_position = _PUSH_ANCHOR + np.array([_TARGET_START_X, 0.0, 0.0])
        target = self.world.add_rigid_body(
            "push_target",
            mass=_TARGET_MASS,
            position=tuple(target_position),
        )
        target.set_collision_shape(sx.CollisionShape.sphere(_PUSH_RADIUS))
        target.friction = 0.24

        return _LinkContactState(
            drop_joint=drop_link.parent_joint,
            drop_link=drop_link,
            slide_vertical_joint=carrier.parent_joint,
            slide_horizontal_joint=slide_link.parent_joint,
            slide_link=slide_link,
            push_joint=push_link.parent_joint,
            push_link=push_link,
            target=target,
        )

    def _add_visuals(self) -> None:
        self.bridge.add_rigid_body_visual(
            self.ground,
            _shape_box(_GROUND_HALF),
            (0.60, 0.62, 0.64),
            name="link_contact_ground_visual",
        )
        self.bridge.add_link_visual(
            self.state.drop_link,
            dart.SphereShape(_FOOT_RADIUS),
            (0.30, 0.58, 0.90),
            name="drop_link_visual",
        )
        self.bridge.add_link_visual(
            self.state.slide_link,
            dart.SphereShape(_FOOT_RADIUS),
            (0.32, 0.72, 0.44),
            name="slide_link_visual",
        )
        self.bridge.add_link_visual(
            self.state.push_link,
            dart.SphereShape(_PUSH_RADIUS),
            (0.88, 0.50, 0.26),
            name="push_link_visual",
        )
        self.bridge.add_rigid_body_visual(
            self.state.target,
            dart.SphereShape(_PUSH_RADIUS),
            (0.72, 0.48, 0.86),
            name="push_target_visual",
        )

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        return self._executors[self.executor_index][0]

    def _step_profile_ms(self) -> float:
        profile = getattr(self.world, "last_step_profile", None)
        if profile is None:
            return 0.0
        total = float(getattr(profile, "total_wall_time_ns", 0.0) or 0.0)
        return total / 1.0e6

    def _apply_parameters(self) -> None:
        self.ground.friction = float(self.ground_friction)
        self.ground.restitution = float(self.ground_restitution)
        self.state.target.friction = min(0.35, float(self.ground_friction))

    def _reset_state(self) -> None:
        drop_start_z = _REST_Z + max(0.02, float(self.drop_height))
        _set_joint_scalar(self.state.drop_joint, "position", drop_start_z)
        _set_joint_scalar(self.state.drop_joint, "velocity", 0.0)
        _set_joint_scalar(self.state.drop_joint, "force", 0.0)

        _set_joint_scalar(self.state.slide_vertical_joint, "position", _REST_Z)
        _set_joint_scalar(self.state.slide_vertical_joint, "velocity", 0.0)
        _set_joint_scalar(self.state.slide_vertical_joint, "force", 0.0)
        _set_joint_scalar(self.state.slide_horizontal_joint, "position", _SLIDE_START_X)
        _set_joint_scalar(
            self.state.slide_horizontal_joint,
            "velocity",
            max(0.0, float(self.slide_speed)),
        )
        _set_joint_scalar(self.state.slide_horizontal_joint, "force", 0.0)

        _set_joint_scalar(self.state.push_joint, "position", _PUSH_START_X)
        _set_joint_scalar(
            self.state.push_joint,
            "velocity",
            max(0.0, float(self.push_speed)),
        )
        _set_joint_scalar(self.state.push_joint, "force", 0.0)
        self.state.target.transform = _translation(
            _PUSH_ANCHOR + np.array([_TARGET_START_X, 0.0, 0.0])
        )
        self.state.target.linear_velocity = (0.0, 0.0, 0.0)
        self.state.target.angular_velocity = (0.0, 0.0, 0.0)
        if hasattr(self.state.target, "clear_force"):
            self.state.target.clear_force()
        if hasattr(self.state.target, "clear_torque"):
            self.state.target.clear_torque()

    def reset(self, *, clear_replay: bool = False) -> None:
        self.world.time = 0.0
        self._apply_parameters()
        self._reset_state()
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            self._drop_height_history,
            self._drop_speed_history,
            self._drop_contact_history,
            self._slide_speed_history,
            self._slide_travel_history,
            self._slide_contact_history,
            self._push_target_travel_history,
            self._push_target_speed_history,
            self._push_contact_history,
            self._step_ms_history,
        ):
            history.clear()
        self._drop_max_upward_velocity = 0.0
        self._last_metrics.clear()
        self.world.update_kinematics()
        self.bridge.sync()
        self._record_metrics()

    def _contact_count(self, required_names: set[str]) -> int:
        count = 0
        for contact in self.world.collide():
            names = {contact.body_a.name, contact.body_b.name}
            if required_names <= names:
                count += 1
        return count

    def _any_link_contact(self) -> tuple[str, str]:
        for contact in self.world.collide():
            bodies = (contact.body_a, contact.body_b)
            if not any(body.is_link for body in bodies):
                continue
            kinds = []
            for body in bodies:
                if body.is_link:
                    kinds.append("link")
                elif body.is_rigid_body:
                    rigid = body.as_rigid_body()
                    kinds.append("static rigid" if rigid is not None and rigid.is_static else "rigid")
                else:
                    kinds.append("unknown")
            return kinds[0], kinds[1]
        return "none", "none"

    def _record_metrics(self) -> None:
        drop_z = float(np.asarray(self.state.drop_link.translation, dtype=float)[2])
        drop_velocity = _joint_scalar(self.state.drop_joint.velocity)
        slide_velocity = _joint_scalar(self.state.slide_horizontal_joint.velocity)
        slide_position = _joint_scalar(self.state.slide_horizontal_joint.position)
        slide_z = float(np.asarray(self.state.slide_link.translation, dtype=float)[2])
        push_velocity = _joint_scalar(self.state.push_joint.velocity)
        target_x = float(np.asarray(self.state.target.translation, dtype=float)[0])
        target_velocity = float(np.asarray(self.state.target.linear_velocity, dtype=float)[0])
        target_start_x = float(_PUSH_ANCHOR[0] + _TARGET_START_X)
        push_contact_count = self._contact_count({"push_link", "push_target"})
        drop_contact_count = self._contact_count({"drop_link", "link_contact_ground"})
        slide_contact_count = self._contact_count({"slide_link", "link_contact_ground"})
        self._drop_max_upward_velocity = max(
            self._drop_max_upward_velocity,
            drop_velocity,
        )
        body_kind_a, body_kind_b = self._any_link_contact()
        step_ms = self._step_profile_ms()

        self._drop_height_history.append(drop_z)
        self._drop_speed_history.append(abs(drop_velocity))
        self._drop_contact_history.append(float(drop_contact_count))
        self._slide_speed_history.append(abs(slide_velocity))
        self._slide_travel_history.append(slide_position - _SLIDE_START_X)
        self._slide_contact_history.append(float(slide_contact_count))
        self._push_target_travel_history.append(target_x - target_start_x)
        self._push_target_speed_history.append(abs(target_velocity))
        self._push_contact_history.append(float(push_contact_count))
        self._step_ms_history.append(step_ms)
        self._last_metrics = {
            "drop_height": drop_z,
            "drop_velocity": drop_velocity,
            "drop_contact_count": float(drop_contact_count),
            "drop_max_upward_velocity": self._drop_max_upward_velocity,
            "slide_speed": abs(slide_velocity),
            "slide_travel": slide_position - _SLIDE_START_X,
            "slide_height_error": slide_z - _REST_Z,
            "slide_contact_count": float(slide_contact_count),
            "push_striker_speed": push_velocity,
            "push_target_speed": target_velocity,
            "push_target_travel": target_x - target_start_x,
            "push_contact_count": float(push_contact_count),
            "step_ms": step_ms,
            "executor": self._executor_label(),
            "contact_body_kinds": f"{body_kind_a}/{body_kind_b}",
        }

    def pre_step(self) -> None:
        self._apply_parameters()
        self.world.step(self._executor())
        self._record_metrics()
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        drop_contacts = list(self._drop_contact_history)
        drop_speeds = list(self._drop_speed_history)
        drop_heights = list(self._drop_height_history)
        slide_contacts = list(self._slide_contact_history)
        slide_speeds = list(self._slide_speed_history)
        slide_travel = list(self._slide_travel_history)
        push_contacts = list(self._push_contact_history)
        push_travel = list(self._push_target_travel_history)
        push_speeds = list(self._push_target_speed_history)
        step_values = list(self._step_ms_history)
        metrics = dict(self._last_metrics)
        return {
            "row": "contact",
            "solver": "sequential_impulse_rigid_body",
            "contact_scope": "multibody_link_contact",
            "rigid_body_solver": self.world.rigid_body_solver.name,
            "executor": self._executor_label(),
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "controls": {
                "executor_index": float(self.executor_index),
                "ground_friction": float(self.ground_friction),
                "ground_restitution": float(self.ground_restitution),
                "drop_height": float(self.drop_height),
                "slide_speed": float(self.slide_speed),
                "push_speed": float(self.push_speed),
            },
            "lanes": {
                "drop": {
                    "height": float(metrics.get("drop_height", 0.0)),
                    "velocity": float(metrics.get("drop_velocity", 0.0)),
                    "contact_count": float(metrics.get("drop_contact_count", 0.0)),
                    "max_upward_velocity": float(
                        metrics.get("drop_max_upward_velocity", 0.0)
                    ),
                },
                "slide": {
                    "speed": float(metrics.get("slide_speed", 0.0)),
                    "travel": float(metrics.get("slide_travel", 0.0)),
                    "height_error": float(metrics.get("slide_height_error", 0.0)),
                    "contact_count": float(metrics.get("slide_contact_count", 0.0)),
                },
                "push": {
                    "striker_speed": float(metrics.get("push_striker_speed", 0.0)),
                    "target_speed": float(metrics.get("push_target_speed", 0.0)),
                    "target_travel": float(metrics.get("push_target_travel", 0.0)),
                    "contact_count": float(metrics.get("push_contact_count", 0.0)),
                },
            },
            "metrics": metrics,
            "history": {
                "samples": float(len(step_values)),
                "drop_min_height": min(drop_heights, default=0.0),
                "drop_max_speed": max(drop_speeds, default=0.0),
                "drop_max_contacts": max(drop_contacts, default=0.0),
                "slide_min_speed": min(slide_speeds, default=0.0),
                "slide_max_travel": max(slide_travel, default=0.0),
                "slide_max_contacts": max(slide_contacts, default=0.0),
                "push_max_target_travel": max(push_travel, default=0.0),
                "push_max_target_speed": max(push_speeds, default=0.0),
                "push_max_contacts": max(push_contacts, default=0.0),
                "max_step_ms": max(step_values, default=0.0),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "ground_friction": float(self.ground_friction),
                "ground_restitution": float(self.ground_restitution),
                "drop_height": float(self.drop_height),
                "slide_speed": float(self.slide_speed),
                "push_speed": float(self.push_speed),
            },
            "state": {
                "drop_position": _joint_scalar(self.state.drop_joint.position),
                "drop_velocity": _joint_scalar(self.state.drop_joint.velocity),
                "slide_vertical_position": _joint_scalar(
                    self.state.slide_vertical_joint.position
                ),
                "slide_horizontal_position": _joint_scalar(
                    self.state.slide_horizontal_joint.position
                ),
                "slide_horizontal_velocity": _joint_scalar(
                    self.state.slide_horizontal_joint.velocity
                ),
                "push_position": _joint_scalar(self.state.push_joint.position),
                "push_velocity": _joint_scalar(self.state.push_joint.velocity),
                "target_transform": np.asarray(self.state.target.transform).tolist(),
                "target_linear_velocity": np.asarray(
                    self.state.target.linear_velocity
                ).tolist(),
                "target_angular_velocity": np.asarray(
                    self.state.target.angular_velocity
                ).tolist(),
                "time": float(self.world.time),
                "drop_max_upward_velocity": float(self._drop_max_upward_velocity),
            },
            "history": {
                "drop_height": list(self._drop_height_history),
                "drop_speed": list(self._drop_speed_history),
                "drop_contact": list(self._drop_contact_history),
                "slide_speed": list(self._slide_speed_history),
                "slide_travel": list(self._slide_travel_history),
                "slide_contact": list(self._slide_contact_history),
                "push_target_travel": list(self._push_target_travel_history),
                "push_target_speed": list(self._push_target_speed_history),
                "push_contact": list(self._push_contact_history),
                "step_ms": list(self._step_ms_history),
            },
            "metrics": dict(self._last_metrics),
        }

    def restore_replay_state(self, snapshot: dict[str, Any]) -> None:
        controls = snapshot.get("controls", {})
        self.executor_index = int(controls.get("executor_index", self.executor_index))
        self.ground_friction = float(
            controls.get("ground_friction", self.ground_friction)
        )
        self.ground_restitution = float(
            controls.get("ground_restitution", self.ground_restitution)
        )
        self.drop_height = float(controls.get("drop_height", self.drop_height))
        self.slide_speed = float(controls.get("slide_speed", self.slide_speed))
        self.push_speed = float(controls.get("push_speed", self.push_speed))
        self._apply_parameters()

        state = snapshot.get("state", {})
        _set_joint_scalar(
            self.state.drop_joint,
            "position",
            float(state.get("drop_position", _joint_scalar(self.state.drop_joint.position))),
        )
        _set_joint_scalar(
            self.state.drop_joint,
            "velocity",
            float(state.get("drop_velocity", _joint_scalar(self.state.drop_joint.velocity))),
        )
        _set_joint_scalar(
            self.state.slide_vertical_joint,
            "position",
            float(
                state.get(
                    "slide_vertical_position",
                    _joint_scalar(self.state.slide_vertical_joint.position),
                )
            ),
        )
        _set_joint_scalar(
            self.state.slide_horizontal_joint,
            "position",
            float(
                state.get(
                    "slide_horizontal_position",
                    _joint_scalar(self.state.slide_horizontal_joint.position),
                )
            ),
        )
        _set_joint_scalar(
            self.state.slide_horizontal_joint,
            "velocity",
            float(
                state.get(
                    "slide_horizontal_velocity",
                    _joint_scalar(self.state.slide_horizontal_joint.velocity),
                )
            ),
        )
        _set_joint_scalar(
            self.state.push_joint,
            "position",
            float(state.get("push_position", _joint_scalar(self.state.push_joint.position))),
        )
        _set_joint_scalar(
            self.state.push_joint,
            "velocity",
            float(state.get("push_velocity", _joint_scalar(self.state.push_joint.velocity))),
        )
        self.state.target.transform = state.get("target_transform", self.state.target.transform)
        self.state.target.linear_velocity = state.get(
            "target_linear_velocity",
            self.state.target.linear_velocity,
        )
        self.state.target.angular_velocity = state.get(
            "target_angular_velocity",
            self.state.target.angular_velocity,
        )
        self.world.time = float(state.get("time", self.world.time))
        self._drop_max_upward_velocity = float(
            state.get("drop_max_upward_velocity", self._drop_max_upward_velocity)
        )

        history = snapshot.get("history", {})
        self._restore_history(self._drop_height_history, history.get("drop_height", []))
        self._restore_history(self._drop_speed_history, history.get("drop_speed", []))
        self._restore_history(self._drop_contact_history, history.get("drop_contact", []))
        self._restore_history(self._slide_speed_history, history.get("slide_speed", []))
        self._restore_history(self._slide_travel_history, history.get("slide_travel", []))
        self._restore_history(
            self._slide_contact_history,
            history.get("slide_contact", []),
        )
        self._restore_history(
            self._push_target_travel_history,
            history.get("push_target_travel", []),
        )
        self._restore_history(
            self._push_target_speed_history,
            history.get("push_target_speed", []),
        )
        self._restore_history(self._push_contact_history, history.get("push_contact", []))
        self._restore_history(self._step_ms_history, history.get("step_ms", []))
        self._last_metrics = dict(snapshot.get("metrics", self._last_metrics))
        self.world.update_kinematics()
        self.bridge.sync()

    @staticmethod
    def _restore_history(history: deque[float], values: object) -> None:
        history.clear()
        if isinstance(values, list):
            history.extend(float(value) for value in values)

    def build_panel(self, builder: Any, context: Any) -> None:
        metrics = self._last_metrics
        executor_choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor",
            int(self.executor_index),
            executor_choices,
        )
        changed_friction, friction = builder.slider(
            "Ground friction",
            float(self.ground_friction),
            0.0,
            1.5,
        )
        changed_restitution, restitution = builder.slider(
            "Ground restitution",
            float(self.ground_restitution),
            0.0,
            0.95,
        )
        changed_drop, drop_height = builder.slider(
            "Drop height",
            float(self.drop_height),
            0.04,
            0.60,
        )
        changed_slide, slide_speed = builder.slider(
            "Slide speed",
            float(self.slide_speed),
            0.05,
            2.0,
        )
        changed_push, push_speed = builder.slider(
            "Push speed",
            float(self.push_speed),
            0.05,
            2.0,
        )
        if changed_executor:
            self.executor_index = int(executor_index)
        if changed_friction:
            self.ground_friction = float(friction)
        if changed_restitution:
            self.ground_restitution = float(restitution)
        if changed_drop:
            self.drop_height = float(drop_height)
        if changed_slide:
            self.slide_speed = float(slide_speed)
        if changed_push:
            self.push_speed = float(push_speed)
        if (
            builder.button("Reset link contact")
            or changed_executor
            or changed_friction
            or changed_restitution
            or changed_drop
            or changed_slide
            or changed_push
        ):
            self.reset(clear_replay=True)
            metrics = self._last_metrics

        builder.text("solver: World multibody link contact | SI contact path")
        builder.text(f"executor: {metrics.get('executor', self._executor_label())}")
        builder.text(
            f"drop height {float(metrics.get('drop_height', 0.0)):.3f} m | "
            f"rebound {float(metrics.get('drop_max_upward_velocity', 0.0)):.3f} m/s"
        )
        builder.text(
            f"slide speed {float(metrics.get('slide_speed', 0.0)):.3f} m/s | "
            f"travel {float(metrics.get('slide_travel', 0.0)):.3f} m"
        )
        builder.text(
            f"target travel {float(metrics.get('push_target_travel', 0.0)):.3f} m | "
            f"target speed {float(metrics.get('push_target_speed', 0.0)):.3f} m/s"
        )
        builder.text(
            f"contacts drop/slide/push "
            f"{float(metrics.get('drop_contact_count', 0.0)):.0f}/"
            f"{float(metrics.get('slide_contact_count', 0.0)):.0f}/"
            f"{float(metrics.get('push_contact_count', 0.0)):.0f}"
        )
        builder.text(f"first link contact bodies: {metrics.get('contact_body_kinds', 'none')}")
        builder.text(f"step profile {float(metrics.get('step_ms', 0.0)):.3f} ms")
        builder.plot_lines("Drop height", list(self._drop_height_history))
        builder.plot_lines("Drop contact count", list(self._drop_contact_history))
        builder.plot_lines("Slide speed", list(self._slide_speed_history))
        builder.plot_lines("Slide travel", list(self._slide_travel_history))
        builder.plot_lines("Push target travel", list(self._push_target_travel_history))
        builder.plot_lines("Push contact count", list(self._push_contact_history))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    controller = _RigidLinkContact()
    return SceneSetup(
        world=controller.bridge.render_world,
        pre_step=controller.pre_step,
        force_drag=controller.force_drag,
        panels=[ScenePanel("Rigid Link Contact", controller.build_panel)],
        info={
            "sx_world": controller.world,
            "rigid_link_contact_controller": controller,
            "replay_capture_state": controller.capture_replay_state,
            "replay_restore_state": controller.restore_replay_state,
            "replay_sync": controller.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: controller.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="contact",
    title="Rigid Link Contact",
    category="World Rigid Body",
    summary=(
        "Shows multibody links contacting ground and rigid bodies through drop, "
        "friction-slide, and pusher lanes."
    ),
    build=build,
)
