"""Rigid spin/roll coupling under contact friction."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 0.004
_HISTORY = 180
_RADIUS = 0.08
_SPHERE_INERTIA = 0.4 * _RADIUS * _RADIUS
_GROUND_HALF = np.array([1.45, 0.86, 0.04])
_START_X = -0.88
_LANE_Y = {
    "matched_roll": 0.54,
    "slide_to_roll": 0.18,
    "backspin_scrub": -0.18,
    "low_friction_slip": -0.54,
}
_LANE_COLORS = {
    "matched_roll": (0.25, 0.58, 0.90),
    "slide_to_roll": (0.24, 0.70, 0.44),
    "backspin_scrub": (0.90, 0.48, 0.22),
    "low_friction_slip": (0.62, 0.48, 0.82),
}


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray | tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


def _stripe_transform() -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.array([0.0, 0.0, _RADIUS + 0.006])
    return transform


def _sphere_inertia() -> np.ndarray:
    return np.diag([_SPHERE_INERTIA, _SPHERE_INERTIA, _SPHERE_INERTIA])


@dataclass
class _SpinRollLane:
    key: str
    label: str
    summary: str
    body: Any
    start_position: np.ndarray
    initial_spin_ratio: float
    friction_scale: float


class _RigidSpinRollCoupling:
    def __init__(self) -> None:
        self.executor_index = 0
        self.friction = 0.35
        self.launch_speed = 1.0
        self.backspin_ratio = -0.6
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
        self.ground = self.world.add_rigid_body("spin_roll_ground")
        self.ground.is_static = True
        self.ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
        self.ground.transform = _transform_at((0.0, 0.0, -_GROUND_HALF[2]))

        self.lanes = [
            self._make_lane(
                "matched_roll",
                "Matched rolling",
                "low slip reference",
                spin_ratio=1.0,
                friction_scale=1.0,
            ),
            self._make_lane(
                "slide_to_roll",
                "Sliding, no spin",
                "friction spins it up",
                spin_ratio=0.0,
                friction_scale=1.0,
            ),
            self._make_lane(
                "backspin_scrub",
                "Backspin scrub",
                "friction fights mismatch",
                spin_ratio=self.backspin_ratio,
                friction_scale=1.0,
            ),
            self._make_lane(
                "low_friction_slip",
                "Low-friction slip",
                "slip baseline",
                spin_ratio=0.0,
                friction_scale=0.0,
            ),
        ]
        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(self.world, name="rigid_spin_roll_coupling")
        self.bridge.add_rigid_body_visual(
            self.ground,
            dart.BoxShape(_full(_GROUND_HALF)),
            (0.42, 0.44, 0.47),
            name="spin_roll_ground_visual",
        )
        for lane in self.lanes:
            self.bridge.add_rigid_body_visual(
                lane.body,
                dart.SphereShape(_RADIUS),
                _LANE_COLORS[lane.key],
                name=f"{lane.key}_sphere_visual",
            )
            self.bridge.add_rigid_body_visual(
                lane.body,
                dart.BoxShape(np.array([0.032, 0.010, 0.010])),
                (0.98, 0.96, 0.82),
                name=f"{lane.key}_spin_stripe_visual",
                local_transform=_stripe_transform(),
            )

        self._slip_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._roll_ratio_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._travel_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._energy_history: dict[str, deque[float]] = {
            lane.key: deque(maxlen=_HISTORY) for lane in self.lanes
        }
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, dict[str, float | str]] = {}
        self._last_contact_count = 0
        self.reset()

    def _make_lane(
        self,
        key: str,
        label: str,
        summary: str,
        *,
        spin_ratio: float,
        friction_scale: float,
    ) -> _SpinRollLane:
        start_position = np.array([_START_X, _LANE_Y[key], _RADIUS + 0.004])
        body = self.world.add_rigid_body(f"{key}_spin_roll_body")
        body.mass = 1.0
        body.set_collision_shape(sx.CollisionShape.sphere(_RADIUS))
        body.restitution = 0.0
        return _SpinRollLane(
            key=key,
            label=label,
            summary=summary,
            body=body,
            start_position=start_position,
            initial_spin_ratio=float(spin_ratio),
            friction_scale=float(friction_scale),
        )

    def _executor(self) -> Any:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][1]

    def _executor_label(self) -> str:
        index = max(0, min(int(self.executor_index), len(self._executors) - 1))
        self.executor_index = index
        return self._executors[index][0]

    def _lane_initial_spin_ratio(self, lane: _SpinRollLane) -> float:
        if lane.key == "backspin_scrub":
            return float(self.backspin_ratio)
        return float(lane.initial_spin_ratio)

    def _lane_friction(self, lane: _SpinRollLane) -> float:
        return max(0.0, float(self.friction) * float(lane.friction_scale))

    def _apply_lane_state(self, lane: _SpinRollLane) -> None:
        body = lane.body
        body.mass = 1.0
        body.inertia = _sphere_inertia()
        body.transform = _transform_at(lane.start_position)
        body.linear_velocity = (float(self.launch_speed), 0.0, 0.0)
        spin_y = self._lane_initial_spin_ratio(lane) * float(self.launch_speed) / _RADIUS
        body.angular_velocity = (0.0, spin_y, 0.0)
        body.friction = self._lane_friction(lane)
        body.restitution = 0.0
        body.clear_force()
        body.clear_torque()

    def _apply_materials(self) -> None:
        self.ground.friction = max(0.0, float(self.friction))
        for lane in self.lanes:
            lane.body.friction = self._lane_friction(lane)
            lane.body.restitution = 0.0

    def reset(self) -> None:
        self._apply_materials()
        for lane in self.lanes:
            self._apply_lane_state(lane)
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            *self._slip_history.values(),
            *self._roll_ratio_history.values(),
            *self._travel_history.values(),
            *self._energy_history.values(),
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self._record_metrics()
        self.bridge.sync()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _contact_count(self) -> int:
        try:
            return len(self.world.collide())
        except Exception:  # noqa: BLE001
            return 0

    def _sample_lane(self, lane: _SpinRollLane) -> dict[str, float | str]:
        body = lane.body
        velocity = np.asarray(body.linear_velocity, dtype=float).reshape(3)
        angular = np.asarray(body.angular_velocity, dtype=float).reshape(3)
        position = np.asarray(body.translation, dtype=float).reshape(3)
        linear_x = float(velocity[0])
        spin_y = float(angular[1])
        surface_speed = _RADIUS * spin_y
        slip = linear_x - surface_speed
        denom = abs(linear_x) + abs(surface_speed) + 1.0e-9
        slip_ratio = abs(slip) / denom
        roll_ratio = linear_x / surface_speed if abs(surface_speed) > 1.0e-9 else 0.0
        translational_energy = 0.5 * float(body.mass) * float(np.dot(velocity, velocity))
        total_energy = float(body.kinetic_energy)
        rotational_energy = max(0.0, total_energy - translational_energy)
        initial_spin_y = self._lane_initial_spin_ratio(lane) * float(self.launch_speed) / _RADIUS

        if slip_ratio < 0.05:
            status = "rolling match"
        elif lane.key == "low_friction_slip":
            status = "slipping"
        elif linear_x < 0.0:
            status = "scrubbed / reversed"
        else:
            status = "slip coupling"

        return {
            "angular_speed": abs(spin_y),
            "contact_slip": slip,
            "friction": self._lane_friction(lane),
            "initial_contact_slip": float(self.launch_speed)
            - _RADIUS * initial_spin_y,
            "linear_speed": float(np.linalg.norm(velocity)),
            "roll_ratio": roll_ratio,
            "rotational_energy": rotational_energy,
            "slip_ratio": slip_ratio,
            "spin_delta": spin_y - initial_spin_y,
            "status": status,
            "surface_speed": surface_speed,
            "total_energy": total_energy,
            "translational_energy": translational_energy,
            "travel": float(position[0] - lane.start_position[0]),
        }

    def _record_metrics(self) -> None:
        self._last_contact_count = self._contact_count()
        step_ms = self._step_profile_ms()
        self._step_ms_history.append(step_ms)
        for lane in self.lanes:
            metrics = self._sample_lane(lane)
            self._last_metrics[lane.key] = metrics
            self._slip_history[lane.key].append(abs(float(metrics["contact_slip"])))
            self._roll_ratio_history[lane.key].append(float(metrics["roll_ratio"]))
            self._travel_history[lane.key].append(float(metrics["travel"]))
            self._energy_history[lane.key].append(float(metrics["total_energy"]))

    def pre_step(self) -> None:
        self._apply_materials()
        self.world.step(self._executor())
        self._record_metrics()
        self.bridge.sync()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        lanes = {
            lane.key: {
                "label": lane.label,
                "summary": lane.summary,
                "metrics": dict(self._last_metrics[lane.key]),
            }
            for lane in self.lanes
        }
        slip_history = {
            lane.key: list(self._slip_history[lane.key]) for lane in self.lanes
        }
        roll_history = {
            lane.key: list(self._roll_ratio_history[lane.key]) for lane in self.lanes
        }
        travel_history = {
            lane.key: list(self._travel_history[lane.key]) for lane in self.lanes
        }
        energy_history = {
            lane.key: list(self._energy_history[lane.key]) for lane in self.lanes
        }

        def lane_metric(lane_key: str, metric_key: str) -> float:
            return float(self._last_metrics[lane_key][metric_key])

        executor_label = self._executor_label()
        return {
            "row": "rigid_spin_roll_coupling",
            "comparison_axis": "spin_roll_initial_condition",
            "solver": "sequential_impulse",
            "solver_enum": sx.RigidBodySolver.SEQUENTIAL_IMPULSE.name,
            "executor": executor_label,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "held_fixed": {
                "solver": "Sequential impulse",
                "executor": executor_label,
                "time_step_ms": _TIME_STEP * 1000.0,
            },
            "controls": {
                "backspin_ratio": float(self.backspin_ratio),
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
            },
            "contact_count": float(self._last_contact_count),
            "step_ms": self._step_profile_ms(),
            "matched_roll_contact_slip": lane_metric(
                "matched_roll", "contact_slip"
            ),
            "matched_roll_roll_ratio": lane_metric("matched_roll", "roll_ratio"),
            "matched_roll_travel": lane_metric("matched_roll", "travel"),
            "matched_roll_total_energy": lane_metric("matched_roll", "total_energy"),
            "slide_to_roll_contact_slip": lane_metric(
                "slide_to_roll", "contact_slip"
            ),
            "slide_to_roll_roll_ratio": lane_metric("slide_to_roll", "roll_ratio"),
            "slide_to_roll_spin_delta": lane_metric("slide_to_roll", "spin_delta"),
            "slide_to_roll_travel": lane_metric("slide_to_roll", "travel"),
            "backspin_scrub_contact_slip": lane_metric(
                "backspin_scrub", "contact_slip"
            ),
            "backspin_scrub_roll_ratio": lane_metric(
                "backspin_scrub", "roll_ratio"
            ),
            "backspin_scrub_spin_delta": lane_metric(
                "backspin_scrub", "spin_delta"
            ),
            "backspin_scrub_travel": lane_metric("backspin_scrub", "travel"),
            "low_friction_slip_contact_slip": lane_metric(
                "low_friction_slip", "contact_slip"
            ),
            "low_friction_slip_friction": lane_metric(
                "low_friction_slip", "friction"
            ),
            "low_friction_slip_spin_delta": lane_metric(
                "low_friction_slip", "spin_delta"
            ),
            "low_friction_slip_travel": lane_metric(
                "low_friction_slip", "travel"
            ),
            "lanes": lanes,
            "history": {
                "samples": float(len(self._step_ms_history)),
                "max_step_ms": max(self._step_ms_history, default=0.0),
                "matched_roll_max_slip": max(
                    slip_history["matched_roll"], default=0.0
                ),
                "slide_to_roll_max_slip": max(
                    slip_history["slide_to_roll"], default=0.0
                ),
                "backspin_scrub_max_slip": max(
                    slip_history["backspin_scrub"], default=0.0
                ),
                "low_friction_slip_max_slip": max(
                    slip_history["low_friction_slip"], default=0.0
                ),
                "matched_roll_max_travel": max(
                    travel_history["matched_roll"], default=0.0
                ),
                "low_friction_slip_max_travel": max(
                    travel_history["low_friction_slip"], default=0.0
                ),
                "max_abs_roll_ratio": max(
                    (abs(value) for values in roll_history.values() for value in values),
                    default=0.0,
                ),
                "min_total_energy": min(
                    (value for values in energy_history.values() for value in values),
                    default=0.0,
                ),
                "max_total_energy": max(
                    (value for values in energy_history.values() for value in values),
                    default=0.0,
                ),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "backspin_ratio": float(self.backspin_ratio),
                "executor_index": int(self.executor_index),
                "friction": float(self.friction),
                "launch_speed": float(self.launch_speed),
            },
            "energy_history": {
                key: list(values) for key, values in self._energy_history.items()
            },
            "last_contact_count": int(self._last_contact_count),
            "last_metrics": {
                key: dict(value) for key, value in self._last_metrics.items()
            },
            "roll_ratio_history": {
                key: list(values) for key, values in self._roll_ratio_history.items()
            },
            "slip_history": {
                key: list(values) for key, values in self._slip_history.items()
            },
            "step_ms_history": list(self._step_ms_history),
            "travel_history": {
                key: list(values) for key, values in self._travel_history.items()
            },
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.backspin_ratio = float(
            controls.get("backspin_ratio", self.backspin_ratio)
        )
        self.executor_index = max(
            0,
            min(
                int(controls.get("executor_index", self.executor_index)),
                max(0, len(self._executors) - 1),
            ),
        )
        self.friction = float(controls.get("friction", self.friction))
        self.launch_speed = float(controls.get("launch_speed", self.launch_speed))
        self._apply_materials()
        self._restore_histories(self._slip_history, state.get("slip_history", {}))
        self._restore_histories(
            self._roll_ratio_history, state.get("roll_ratio_history", {})
        )
        self._restore_histories(self._travel_history, state.get("travel_history", {}))
        self._restore_histories(self._energy_history, state.get("energy_history", {}))
        self._step_ms_history.clear()
        self._step_ms_history.extend(
            float(value) for value in state.get("step_ms_history", [])
        )
        self._last_contact_count = int(state.get("last_contact_count", 0))
        self._last_metrics = {
            str(key): dict(value)
            for key, value in state.get("last_metrics", {}).items()
        }
        if not self._last_metrics:
            self._record_metrics()
        self.bridge.sync()

    def _restore_histories(
        self, histories: dict[str, deque[float]], payload: dict[str, list[float]]
    ) -> None:
        for key, history in histories.items():
            history.clear()
            history.extend(float(value) for value in payload.get(key, []))

    def _lane_text(self, builder: Any, lane: _SpinRollLane) -> None:
        metrics = self._last_metrics.get(lane.key) or self._sample_lane(lane)
        builder.text(
            f"{lane.label}: {lane.summary}, mu {float(metrics['friction']):.2f}, "
            f"{metrics['status']}"
        )
        builder.text(
            f"  slip {float(metrics['contact_slip']):.3f} m/s | "
            f"roll ratio {float(metrics['roll_ratio']):.2f} | "
            f"travel {float(metrics['travel']):.3f} m"
        )
        builder.text(
            f"  speed {float(metrics['linear_speed']):.3f} m/s | "
            f"spin {float(metrics['angular_speed']):.3f} rad/s | "
            f"energy {float(metrics['total_energy']):.3f} J"
        )

    def build_panel(self, builder: Any, context: Any) -> None:
        choices = [label for label, _executor in self._executors]
        changed_executor, executor_index = builder.select(
            "Executor", int(self.executor_index), choices
        )
        if changed_executor:
            self.executor_index = int(executor_index)

        changed_friction, friction = builder.slider(
            "Contact friction", float(self.friction), 0.0, 1.0
        )
        changed_speed, launch_speed = builder.slider(
            "Launch speed", float(self.launch_speed), 0.2, 1.8
        )
        changed_backspin, backspin_ratio = builder.slider(
            "Backspin ratio", float(self.backspin_ratio), -1.5, 0.5
        )
        if changed_friction:
            self.friction = float(friction)
        if changed_speed:
            self.launch_speed = float(launch_speed)
        if changed_backspin:
            self.backspin_ratio = float(backspin_ratio)
        if changed_executor or changed_friction or changed_speed or changed_backspin:
            self.reset()

        if builder.button("Reset spin/roll test"):
            self.reset()

        builder.separator()
        builder.text("comparison axis: spin/roll initial condition")
        builder.text(
            f"held fixed: solver Sequential impulse | "
            f"executor {self._executor_label()} | time step {_TIME_STEP * 1000.0:.1f} ms"
        )
        builder.text("solver: sequential impulse")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(
            f"contacts: {self._last_contact_count} | "
            f"step {self._step_profile_ms():.3f} ms"
        )
        for lane in self.lanes:
            self._lane_text(builder, lane)
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} slip", list(self._slip_history[lane.key])
            )
        for lane in self.lanes:
            builder.plot_lines(
                f"{lane.label} roll ratio",
                list(self._roll_ratio_history[lane.key]),
            )
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    coupling = _RigidSpinRollCoupling()
    return SceneSetup(
        world=coupling.bridge.render_world,
        pre_step=coupling.pre_step,
        force_drag=coupling.bridge.force_drag,
        renderable_provider=coupling.bridge.renderable_provider,
        panels=[ScenePanel("Rigid Spin/Roll Coupling", coupling.build_panel)],
        info={
            "sx_world": coupling.world,
            "rigid_body_solver": "sequential_impulse",
            "rigid_spin_roll_coupling_controller": coupling,
            "replay_capture_state": coupling.capture_replay_state,
            "replay_restore_state": coupling.restore_replay_state,
            "replay_sync": coupling.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: coupling.capture_metrics,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_spin_roll_coupling",
    title="Rigid Spin/Roll Coupling",
    category="World Rigid Body",
    summary=(
        "Rolling, sliding, backspin, and low-friction lanes show how contact "
        "friction couples linear and angular rigid-body motion."
    ),
    build=build,
)
