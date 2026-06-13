"""Rigid contact material mixing for disagreeing body/surface pairs."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_BALL_RADIUS = 0.065
_BOX_HALF = np.array([0.07, 0.07, 0.07])
_BOUNCE_PAD_HALF = np.array([0.25, 0.18, 0.035])
_SLIDE_PAD_HALF = np.array([0.58, 0.18, 0.035])
_TIME_STEP = 0.004
_HISTORY = 180
_BOUNCE_Y = 0.46
_SLIDE_Y = -0.54
_BOUNCE_START_CLEARANCE = 0.16
_SLIDE_START_OFFSET = -0.34
_LANE_X = {
    "bounce_body_high": -0.48,
    "bounce_surface_high": 0.48,
    "slide_body_high": -0.48,
    "slide_surface_high": 0.48,
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


@dataclass
class _MaterialLane:
    key: str
    label: str
    mode: str
    color: tuple[float, float, float]
    body_is_high: bool
    surface_is_high: bool
    pad: Any
    mover: Any
    start_position: np.ndarray


class _RigidMaterialMixing:
    def __init__(self) -> None:
        self.executor_index = 0
        self.impact_speed = 1.20
        self.tangential_speed = 1.10
        self.low_restitution = 0.05
        self.high_restitution = 0.82
        self.low_friction = 0.04
        self.high_friction = 0.90
        self._executors: list[tuple[str, Any]] = [
            ("Sequential", sx.SequentialExecutor()),
        ]
        try:
            self._executors.append(("Parallel (2 workers)", sx.ParallelExecutor(2)))
        except Exception:  # noqa: BLE001
            pass

        self.world = sx.World(
            time_step=_TIME_STEP,
            rigid_body_solver=sx.RigidBodySolver.SEQUENTIAL_IMPULSE,
        )
        self.world.step_profiling_enabled = True
        self.lanes = [
            self._make_lane(
                "bounce_body_high",
                "Bounce: high body / low surface",
                "bounce",
                (0.28, 0.55, 0.90),
                body_is_high=True,
                surface_is_high=False,
            ),
            self._make_lane(
                "bounce_surface_high",
                "Bounce: low body / high surface",
                "bounce",
                (0.93, 0.66, 0.24),
                body_is_high=False,
                surface_is_high=True,
            ),
            self._make_lane(
                "slide_body_high",
                "Slide: high body / low surface",
                "slide",
                (0.28, 0.72, 0.44),
                body_is_high=True,
                surface_is_high=False,
            ),
            self._make_lane(
                "slide_surface_high",
                "Slide: low body / high surface",
                "slide",
                (0.84, 0.34, 0.24),
                body_is_high=False,
                surface_is_high=True,
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_material_mixing")
        for lane in self.lanes:
            pad_half = _BOUNCE_PAD_HALF if lane.mode == "bounce" else _SLIDE_PAD_HALF
            mover_shape = (
                dart.SphereShape(_BALL_RADIUS)
                if lane.mode == "bounce"
                else dart.BoxShape(_full(_BOX_HALF))
            )
            self.bridge.add_rigid_body_visual(
                lane.pad,
                dart.BoxShape(_full(pad_half)),
                (0.43, 0.44, 0.47),
                name=f"{lane.key}_material_pad_visual",
            )
            self.bridge.add_rigid_body_visual(
                lane.mover,
                mover_shape,
                lane.color,
                name=f"{lane.key}_material_mover_visual",
            )

        self._height_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._rebound_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._speed_loss_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._contact_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._max_rebound_height: dict[str, float] = {}
        self._max_upward_velocity: dict[str, float] = {}
        self._had_contact: dict[str, bool] = {}
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._reset()

    def _make_lane(
        self,
        key: str,
        label: str,
        mode: str,
        color: tuple[float, float, float],
        *,
        body_is_high: bool,
        surface_is_high: bool,
    ) -> _MaterialLane:
        x = _LANE_X[key]
        y = _BOUNCE_Y if mode == "bounce" else _SLIDE_Y
        pad_half = _BOUNCE_PAD_HALF if mode == "bounce" else _SLIDE_PAD_HALF
        pad = self.world.add_rigid_body(
            f"{key}_material_surface", position=(x, y, -pad_half[2])
        )
        pad.is_static = True
        pad.set_collision_shape(sx.CollisionShape.box(pad_half))

        if mode == "bounce":
            start_position = np.array(
                [x, y, _BALL_RADIUS + _BOUNCE_START_CLEARANCE], dtype=float
            )
            mover = self.world.add_rigid_body(
                f"{key}_material_body", position=tuple(start_position)
            )
            mover.set_collision_shape(sx.CollisionShape.sphere(_BALL_RADIUS))
        else:
            start_position = np.array(
                [x + _SLIDE_START_OFFSET, y, _BOX_HALF[2] + 0.004], dtype=float
            )
            mover = self.world.add_rigid_body(
                f"{key}_material_body", position=tuple(start_position)
            )
            mover.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        mover.mass = 1.0
        return _MaterialLane(
            key=key,
            label=label,
            mode=mode,
            color=color,
            body_is_high=body_is_high,
            surface_is_high=surface_is_high,
            pad=pad,
            mover=mover,
            start_position=start_position,
        )

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _body_restitution(self, lane: _MaterialLane) -> float:
        return float(self.high_restitution if lane.body_is_high else self.low_restitution)

    def _surface_restitution(self, lane: _MaterialLane) -> float:
        return float(
            self.high_restitution if lane.surface_is_high else self.low_restitution
        )

    def _body_friction(self, lane: _MaterialLane) -> float:
        return float(self.high_friction if lane.body_is_high else self.low_friction)

    def _surface_friction(self, lane: _MaterialLane) -> float:
        return float(self.high_friction if lane.surface_is_high else self.low_friction)

    def _effective_restitution(self, lane: _MaterialLane) -> float:
        return max(self._body_restitution(lane), self._surface_restitution(lane))

    def _effective_friction(self, lane: _MaterialLane) -> float:
        return math.sqrt(max(0.0, self._body_friction(lane) * self._surface_friction(lane)))

    def _apply_lane_parameters(self, lane: _MaterialLane) -> None:
        lane.mover.restitution = self._body_restitution(lane)
        lane.pad.restitution = self._surface_restitution(lane)
        lane.mover.friction = self._body_friction(lane)
        lane.pad.friction = self._surface_friction(lane)

    def _reset_lane(self, lane: _MaterialLane) -> None:
        self._apply_lane_parameters(lane)
        if lane.mode == "bounce":
            lane.start_position = np.array(
                [
                    _LANE_X[lane.key],
                    _BOUNCE_Y,
                    _BALL_RADIUS + _BOUNCE_START_CLEARANCE,
                ],
                dtype=float,
            )
            lane.mover.linear_velocity = (0.0, 0.0, -abs(float(self.impact_speed)))
        else:
            lane.start_position = np.array(
                [
                    _LANE_X[lane.key] + _SLIDE_START_OFFSET,
                    _SLIDE_Y,
                    _BOX_HALF[2] + 0.004,
                ],
                dtype=float,
            )
            lane.mover.linear_velocity = (abs(float(self.tangential_speed)), 0.0, 0.0)
        lane.mover.transform = _transform_at(lane.start_position)
        lane.mover.angular_velocity = (0.0, 0.0, 0.0)
        lane.mover.clear_force()
        lane.mover.clear_torque()

    def _reset(self) -> None:
        self.impact_speed = max(0.2, min(float(self.impact_speed), 3.0))
        self.tangential_speed = max(0.2, min(float(self.tangential_speed), 3.0))
        self.low_restitution = max(0.0, min(float(self.low_restitution), 0.40))
        self.high_restitution = max(
            self.low_restitution, min(float(self.high_restitution), 1.0)
        )
        self.low_friction = max(0.0, min(float(self.low_friction), 0.40))
        self.high_friction = max(self.low_friction, min(float(self.high_friction), 1.2))
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        for lane in self.lanes:
            self._reset_lane(lane)
        self.world.update_kinematics()
        for history in (
            *self._height_history.values(),
            *self._rebound_history.values(),
            *self._speed_loss_history.values(),
            *self._contact_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._max_rebound_height = {lane.key: 0.0 for lane in self.lanes}
        self._max_upward_velocity = {lane.key: 0.0 for lane in self.lanes}
        self._had_contact = {lane.key: False for lane in self.lanes}
        self._last_metrics.clear()
        self.bridge.sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _contact_counts(self) -> dict[str, int]:
        counts = {lane.key: 0 for lane in self.lanes}
        expected = {
            lane.key: {lane.pad.name, lane.mover.name} for lane in self.lanes
        }
        for contact in self.world.collide():
            names = {contact.body_a.name, contact.body_b.name}
            for key, pair_names in expected.items():
                if names == pair_names:
                    counts[key] += 1
        return counts

    def _sample(
        self, lane: _MaterialLane, contact_count: int
    ) -> dict[str, float | str]:
        position = np.asarray(lane.mover.translation, dtype=float)
        velocity = np.asarray(lane.mover.linear_velocity, dtype=float)
        if contact_count > 0:
            self._had_contact[lane.key] = True
        height = float(position[2])
        vertical_velocity = float(velocity[2])
        tangential_speed = float(velocity[0])
        if self._had_contact[lane.key] and lane.mode == "bounce":
            self._max_rebound_height[lane.key] = max(
                self._max_rebound_height[lane.key], height
            )
            self._max_upward_velocity[lane.key] = max(
                self._max_upward_velocity[lane.key], vertical_velocity
            )
        speed_loss = (
            max(0.0, float(self.tangential_speed) - tangential_speed)
            if lane.mode == "slide"
            else 0.0
        )
        if contact_count > 0:
            status = "contact"
        elif not self._had_contact[lane.key]:
            status = "approaching"
        elif lane.mode == "bounce" and vertical_velocity > 0.03:
            status = "rebounding"
        elif lane.mode == "slide" and tangential_speed > 0.03:
            status = "sliding"
        else:
            status = "settling"
        return {
            "body_friction": self._body_friction(lane),
            "body_restitution": self._body_restitution(lane),
            "contact_count": float(contact_count),
            "effective_friction": self._effective_friction(lane),
            "effective_restitution": self._effective_restitution(lane),
            "height": height,
            "max_rebound_height": self._max_rebound_height[lane.key],
            "max_upward_velocity": self._max_upward_velocity[lane.key],
            "mode": lane.mode,
            "speed_loss": speed_loss,
            "status": status,
            "surface_friction": self._surface_friction(lane),
            "surface_restitution": self._surface_restitution(lane),
            "tangential_speed": tangential_speed,
            "vertical_velocity": vertical_velocity,
        }

    def _record_metrics(self) -> None:
        contact_counts = self._contact_counts()
        for lane in self.lanes:
            metrics = self._sample(lane, contact_counts[lane.key])
            self._last_metrics[lane.key] = metrics
            self._height_history[lane.key].append(float(metrics["height"]))
            self._rebound_history[lane.key].append(
                float(metrics["max_rebound_height"])
            )
            self._speed_loss_history[lane.key].append(float(metrics["speed_loss"]))
            self._contact_history[lane.key].append(float(metrics["contact_count"]))
        self._step_ms_history.append(self._step_profile_ms())

    def pre_step(self) -> None:
        self.world.step(self._executor())
        self._record_metrics()
        self.bridge.sync()

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "executor_index": int(self.executor_index),
                "high_friction": float(self.high_friction),
                "high_restitution": float(self.high_restitution),
                "impact_speed": float(self.impact_speed),
                "low_friction": float(self.low_friction),
                "low_restitution": float(self.low_restitution),
                "tangential_speed": float(self.tangential_speed),
            },
            "world_state": np.asarray(self.world.state_vector, dtype=float).copy(),
            "world_time": float(self.world.time),
            "height_history": {
                key: list(values) for key, values in self._height_history.items()
            },
            "rebound_history": {
                key: list(values) for key, values in self._rebound_history.items()
            },
            "speed_loss_history": {
                key: list(values) for key, values in self._speed_loss_history.items()
            },
            "contact_history": {
                key: list(values) for key, values in self._contact_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "max_rebound_height": dict(self._max_rebound_height),
            "max_upward_velocity": dict(self._max_upward_velocity),
            "had_contact": dict(self._had_contact),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
        }

    def capture_metrics(self) -> dict[str, Any]:
        metrics = {
            lane.key: dict(self._last_metrics.get(lane.key) or self._sample(lane, 0))
            for lane in self.lanes
        }
        bounce_body = metrics["bounce_body_high"]
        bounce_surface = metrics["bounce_surface_high"]
        slide_body = metrics["slide_body_high"]
        slide_surface = metrics["slide_surface_high"]
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_material_mixing",
            "solver": "sequential_impulse",
            "executor": self._executors[int(self.executor_index)][0],
            "scope": "body_surface_pair_material_mixing_rules",
            "time_step_ms": float(_TIME_STEP * 1000.0),
            "world_time": float(self.world.time),
            "impact_speed": float(self.impact_speed),
            "tangential_speed": float(self.tangential_speed),
            "low_restitution": float(self.low_restitution),
            "high_restitution": float(self.high_restitution),
            "low_friction": float(self.low_friction),
            "high_friction": float(self.high_friction),
            "expected_restitution_rule": "max",
            "expected_friction_rule": "sqrt_product",
            "lane_order": [lane.key for lane in self.lanes],
            "lanes": metrics,
            "controls": {
                "executor_index": float(self.executor_index),
                "impact_speed": float(self.impact_speed),
                "tangential_speed": float(self.tangential_speed),
                "low_restitution": float(self.low_restitution),
                "high_restitution": float(self.high_restitution),
                "low_friction": float(self.low_friction),
                "high_friction": float(self.high_friction),
            },
            "bounce_body_high_rebound": float(bounce_body["max_rebound_height"]),
            "bounce_surface_high_rebound": float(
                bounce_surface["max_rebound_height"]
            ),
            "slide_body_high_speed_loss": float(slide_body["speed_loss"]),
            "slide_surface_high_speed_loss": float(slide_surface["speed_loss"]),
            "effective_restitution": float(bounce_body["effective_restitution"]),
            "effective_friction": float(slide_body["effective_friction"]),
            "step_ms": float(step_values[-1]) if step_values else 0.0,
            "history": {
                "samples": float(len(step_values)),
                "max_bounce_body_high_rebound": float(
                    self._max_rebound_height.get("bounce_body_high", 0.0)
                ),
                "max_bounce_surface_high_rebound": float(
                    self._max_rebound_height.get("bounce_surface_high", 0.0)
                ),
                "max_slide_body_high_speed_loss": max(
                    (float(value) for value in self._speed_loss_history["slide_body_high"]),
                    default=0.0,
                ),
                "max_slide_surface_high_speed_loss": max(
                    (
                        float(value)
                        for value in self._speed_loss_history["slide_surface_high"]
                    ),
                    default=0.0,
                ),
                "max_contact_count": max(
                    (
                        float(value)
                        for history in self._contact_history.values()
                        for value in history
                    ),
                    default=0.0,
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
        self.impact_speed = float(controls.get("impact_speed", self.impact_speed))
        self.tangential_speed = float(
            controls.get("tangential_speed", self.tangential_speed)
        )
        self.low_restitution = float(
            controls.get("low_restitution", self.low_restitution)
        )
        self.high_restitution = float(
            controls.get("high_restitution", self.high_restitution)
        )
        self.low_friction = float(controls.get("low_friction", self.low_friction))
        self.high_friction = float(controls.get("high_friction", self.high_friction))
        self.impact_speed = max(0.2, min(self.impact_speed, 3.0))
        self.tangential_speed = max(0.2, min(self.tangential_speed, 3.0))
        self.low_restitution = max(0.0, min(self.low_restitution, 0.40))
        self.high_restitution = max(self.low_restitution, min(self.high_restitution, 1.0))
        self.low_friction = max(0.0, min(self.low_friction, 0.40))
        self.high_friction = max(self.low_friction, min(self.high_friction, 1.2))
        for lane in self.lanes:
            self._apply_lane_parameters(lane)
        if "world_state" in state:
            self.world.state_vector = state["world_state"]
        if "world_time" in state:
            self.world.time = float(state["world_time"])
        self.world.update_kinematics()
        self._restore_histories(self._height_history, state.get("height_history", {}))
        self._restore_histories(self._rebound_history, state.get("rebound_history", {}))
        self._restore_histories(
            self._speed_loss_history, state.get("speed_loss_history", {})
        )
        self._restore_histories(self._contact_history, state.get("contact_history", {}))
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._max_rebound_height = {
            str(key): float(value)
            for key, value in state.get("max_rebound_height", {}).items()
        }
        self._max_upward_velocity = {
            str(key): float(value)
            for key, value in state.get("max_upward_velocity", {}).items()
        }
        self._had_contact = {
            str(key): bool(value) for key, value in state.get("had_contact", {}).items()
        }
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        self.bridge.sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _MaterialLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample(lane, 0)
        builder.text(
            f"{lane.label}: e max {float(metrics['effective_restitution']):.2f} | "
            f"mu sqrt {float(metrics['effective_friction']):.3f}"
        )
        if lane.mode == "bounce":
            builder.text(
                f"  {metrics['status']} | rebound "
                f"{float(metrics['max_rebound_height']):.3f} m | "
                f"vz {float(metrics['vertical_velocity']):.3f} m/s"
            )
        else:
            builder.text(
                f"  {metrics['status']} | speed loss "
                f"{float(metrics['speed_loss']):.3f} m/s | "
                f"vx {float(metrics['tangential_speed']):.3f} m/s"
            )

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), [label for label, _ in self._executors]
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_impact, impact_speed = builder.slider(
            "Impact speed", float(self.impact_speed), 0.2, 3.0
        )
        changed_tangent, tangential_speed = builder.slider(
            "Tangential speed", float(self.tangential_speed), 0.2, 3.0
        )
        changed_low_e, low_restitution = builder.slider(
            "Low restitution", float(self.low_restitution), 0.0, 0.40
        )
        changed_high_e, high_restitution = builder.slider(
            "High restitution", float(self.high_restitution), 0.40, 1.0
        )
        changed_low_mu, low_friction = builder.slider(
            "Low friction", float(self.low_friction), 0.0, 0.40
        )
        changed_high_mu, high_friction = builder.slider(
            "High friction", float(self.high_friction), 0.40, 1.20
        )
        if changed_impact:
            self.impact_speed = float(impact_speed)
        if changed_tangent:
            self.tangential_speed = float(tangential_speed)
        if changed_low_e:
            self.low_restitution = float(low_restitution)
        if changed_high_e:
            self.high_restitution = float(high_restitution)
        if changed_low_mu:
            self.low_friction = float(low_friction)
        if changed_high_mu:
            self.high_friction = float(high_friction)
        if (
            changed_executor
            or changed_impact
            or changed_tangent
            or changed_low_e
            or changed_high_e
            or changed_low_mu
            or changed_high_mu
        ):
            self._reset()

        if builder.button("Reset material mix"):
            self._reset()

        builder.separator()
        builder.text("solver: sequential impulse")
        builder.text(
            f"low/high e: {self.low_restitution:.2f}/{self.high_restitution:.2f} | "
            f"low/high mu: {self.low_friction:.3f}/{self.high_friction:.3f}"
        )
        builder.text(f"world time: {self.world.time:.3f} s")
        if self._step_ms_history:
            builder.text(f"step profile: {self._step_ms_history[-1]:.3f} ms")
        for lane in self.lanes:
            self._lane_text(builder, lane)
        for lane in self.lanes:
            if lane.mode == "bounce":
                builder.plot_lines(
                    f"{lane.label} rebound", list(self._rebound_history[lane.key])
                )
            else:
                builder.plot_lines(
                    f"{lane.label} speed loss",
                    list(self._speed_loss_history[lane.key]),
                )
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    mixing = _RigidMaterialMixing()
    return SceneSetup(
        world=mixing.bridge.render_world,
        pre_step=mixing.pre_step,
        force_drag=mixing.bridge.force_drag,
        renderable_provider=mixing.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Material Mixing", mixing.build_panel)],
        info={
            "sx_world": mixing.world,
            "rigid_material_mixing_controller": mixing,
            "replay_capture_state": mixing.capture_replay_state,
            "replay_restore_state": mixing.restore_replay_state,
            "replay_sync": mixing.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: mixing.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_material_mixing",
    title="Rigid Material Mixing",
    category="World Rigid Body",
    summary=(
        "Swapped body/surface pairs show that rigid contact restitution uses max "
        "and friction uses the geometric mean."
    ),
    build=build,
)
