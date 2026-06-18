"""World rigid-body scene: four spheres and a box dropping onto a ground plate.

Mirrors examples/demos/scenes/rigid_body.cpp via the
WorldRenderBridge: a World owns the physics (RigidBody + collision
shape), and a parallel render World owns the SimpleFrame
visuals the C++ viewer renders. The bridge advances physics in
``pre_step`` and copies each body's world transform onto its render
frame.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import CAPTURE_METRICS_INFO_KEY, PythonDemoScene, ScenePanel, SceneSetup

_TIME_STEP = 1.0 / 120.0
_HISTORY = 160
_GROUND_HALF = np.array([1.5, 1.5, 0.04])
_BOX_HALF = np.array([0.1, 0.12, 0.08])
_DEFAULT_FRICTION = 0.85
_DEFAULT_RESTITUTION = 0.15
# Keep this baseline on the realtime SI rigid-body path; the dedicated
# rigid_solver_compare scene owns SI-vs-IPC visual inspection.
_SOLVERS: tuple[tuple[str, sx.RigidBodySolver], ...] = (
    ("Sequential impulse", sx.RigidBodySolver.SEQUENTIAL_IMPULSE),
)
_CONTACT_METHODS: tuple[tuple[str, sx.ContactSolverMethod], ...] = (
    ("Sequential impulse", sx.ContactSolverMethod.SEQUENTIAL_IMPULSE),
    ("Boxed LCP", sx.ContactSolverMethod.BOXED_LCP),
)


def _visual_for(
    shape_kind: str, *, radius: float = 0.0, half_extents=None
) -> "dart.Shape":
    if shape_kind == "sphere":
        return dart.SphereShape(radius)
    if shape_kind == "box":
        return dart.BoxShape(2.0 * np.asarray(half_extents, dtype=float))
    raise ValueError(f"unknown shape kind: {shape_kind}")


@dataclass
class _InitialBodyState:
    body: Any
    transform: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray


class _RigidBodyBaseline:
    def __init__(self) -> None:
        self.solver_index = 0
        self.contact_method_index = 0
        self.friction = _DEFAULT_FRICTION
        self.restitution = _DEFAULT_RESTITUTION

        self.world = sx.World(
            time_step=_TIME_STEP,
            rigid_body_solver=self._solver(),
            contact_solver_method=self._contact_method(),
        )
        self.world.step_profiling_enabled = True
        self.bridge = WorldRenderBridge(self.world, name="world_rigid_body_render")
        self.bridge.render_world.set_time_step(_TIME_STEP)

        ground_opts = sx.RigidBodyOptions()
        ground_opts.position = np.array([0.0, 0.0, -0.08])
        ground_opts.is_static = True
        self.ground = self.world.add_rigid_body("ground", ground_opts)
        self.ground.set_collision_shape(sx.CollisionShape.box(_GROUND_HALF))
        self.bridge.add_rigid_body_visual(
            self.ground,
            _visual_for("box", half_extents=_GROUND_HALF),
            (0.42, 0.45, 0.48),
            name="ground_visual",
        )

        self.dynamic_bodies: list[Any] = []
        for i in range(4):
            opts = sx.RigidBodyOptions()
            opts.mass = 1.0 + 0.25 * i
            opts.position = np.array(
                [-0.75 + 0.5 * i, 0.18 * (i % 2), 1.2 + 0.45 * i]
            )
            opts.linear_velocity = np.array([0.45 - 0.25 * i, 0.0, 0.0])
            opts.angular_velocity = np.array([0.0, 0.4 + 0.2 * i, 0.25])

            radius = 0.18 + 0.03 * i
            body = self.world.add_rigid_body(f"falling_sphere_{i}", opts)
            body.set_collision_shape(sx.CollisionShape.sphere(radius))
            self.dynamic_bodies.append(body)
            self.bridge.add_rigid_body_visual(
                body,
                _visual_for("sphere", radius=radius),
                (0.16 + 0.12 * i, 0.42, 0.92 - 0.1 * i),
                name=f"sphere_{i}_visual",
            )

        box_opts = sx.RigidBodyOptions()
        box_opts.mass = 2.0
        box_opts.position = np.array([0.85, -0.35, 2.2])
        box_opts.angular_velocity = np.array([0.2, -0.35, 0.1])
        box = self.world.add_rigid_body("falling_box", box_opts)
        box.set_collision_shape(sx.CollisionShape.box(_BOX_HALF))
        self.dynamic_bodies.append(box)
        self.bridge.add_rigid_body_visual(
            box,
            _visual_for("box", half_extents=_BOX_HALF),
            (0.93, 0.56, 0.18),
            name="box_visual",
        )

        self._initial_states = self._capture_initial_states()
        self.world.enter_simulation_mode()

        self._speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._min_height_history: deque[float] = deque(maxlen=_HISTORY)
        self._energy_history: deque[float] = deque(maxlen=_HISTORY)
        self._contact_history: deque[float] = deque(maxlen=_HISTORY)
        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self.reset(clear_replay=False)

    def _solver(self) -> sx.RigidBodySolver:
        index = max(0, min(int(self.solver_index), len(_SOLVERS) - 1))
        self.solver_index = index
        return _SOLVERS[index][1]

    def _solver_label(self) -> str:
        return _SOLVERS[max(0, min(int(self.solver_index), len(_SOLVERS) - 1))][0]

    def _contact_method(self) -> sx.ContactSolverMethod:
        index = max(0, min(int(self.contact_method_index), len(_CONTACT_METHODS) - 1))
        self.contact_method_index = index
        return _CONTACT_METHODS[index][1]

    def _contact_method_label(self) -> str:
        return _CONTACT_METHODS[
            max(0, min(int(self.contact_method_index), len(_CONTACT_METHODS) - 1))
        ][0]

    def _capture_initial_states(self) -> list[_InitialBodyState]:
        return [
            _InitialBodyState(
                body=body,
                transform=np.asarray(body.transform, dtype=float).copy(),
                linear_velocity=np.asarray(body.linear_velocity, dtype=float)
                .reshape(3)
                .copy(),
                angular_velocity=np.asarray(body.angular_velocity, dtype=float)
                .reshape(3)
                .copy(),
            )
            for body in [self.ground, *self.dynamic_bodies]
        ]

    def _apply_materials(self) -> None:
        self.ground.friction = float(self.friction)
        for body in self.dynamic_bodies:
            body.friction = float(self.friction)
            body.restitution = float(self.restitution)

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

    def _record_metrics(self) -> None:
        speeds = [
            float(np.linalg.norm(np.asarray(body.linear_velocity, dtype=float)))
            for body in self.dynamic_bodies
        ]
        heights = [
            float(np.asarray(body.translation, dtype=float)[2])
            for body in self.dynamic_bodies
        ]
        energy = sum(float(body.kinetic_energy) for body in self.dynamic_bodies)
        max_speed = max(speeds) if speeds else 0.0
        min_height = min(heights) if heights else 0.0
        contact_count = float(self._contact_count())
        step_ms = self._step_profile_ms()

        self._speed_history.append(max_speed)
        self._min_height_history.append(min_height)
        self._energy_history.append(energy)
        self._contact_history.append(contact_count)
        self._step_ms_history.append(step_ms)
        self._last_metrics = {
            "max_speed": max_speed,
            "min_height": min_height,
            "energy": energy,
            "contact_count": contact_count,
            "step_ms": step_ms,
            "solver": self._solver_label(),
            "contact_solver": self._contact_method_label(),
            "executor": "World.step default",
        }

    def reset(self, *, clear_replay: bool = True) -> None:
        self.world.time = 0.0
        self.world.rigid_body_solver = self._solver()
        self.world.contact_solver_method = self._contact_method()
        self._apply_materials()
        for state in self._initial_states:
            state.body.transform = state.transform
            state.body.linear_velocity = state.linear_velocity
            state.body.angular_velocity = state.angular_velocity
            if hasattr(state.body, "clear_force"):
                state.body.clear_force()
            if hasattr(state.body, "clear_torque"):
                state.body.clear_torque()
        if clear_replay:
            try:
                self.world.clear_replay_recording()
            except Exception:  # noqa: BLE001
                pass
        for history in (
            self._speed_history,
            self._min_height_history,
            self._energy_history,
            self._contact_history,
            self._step_ms_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self.world.update_kinematics()
        self.bridge.sync()
        self._record_metrics()

    def pre_step(self) -> None:
        self.world.rigid_body_solver = self._solver()
        self.world.contact_solver_method = self._contact_method()
        self._apply_materials()
        self.bridge.pre_step()
        self._record_metrics()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def capture_metrics(self) -> dict[str, Any]:
        if not self._last_metrics:
            self._record_metrics()
        speed_values = list(self._speed_history)
        height_values = list(self._min_height_history)
        energy_values = list(self._energy_history)
        contact_values = list(self._contact_history)
        step_values = list(self._step_ms_history)
        return {
            "row": "rigid_body",
            "solver": self._solver_label(),
            "contact_solver": self._contact_method_label(),
            "executor": "World.step default",
            "solver_enum": self._solver().name,
            "contact_solver_method": self._contact_method().name,
            "time_step_ms": _TIME_STEP * 1000.0,
            "world_time": float(self.world.time),
            "dynamic_body_count": float(len(self.dynamic_bodies)),
            "baseline_max_speed": float(self._last_metrics["max_speed"]),
            "baseline_min_height": float(self._last_metrics["min_height"]),
            "baseline_energy": float(self._last_metrics["energy"]),
            "baseline_scene_contact_count": float(
                self._last_metrics["contact_count"]
            ),
            "baseline_step_ms": float(self._last_metrics["step_ms"]),
            "controls": {
                "friction": float(self.friction),
                "restitution": float(self.restitution),
                "solver_index": float(self.solver_index),
                "contact_method_index": float(self.contact_method_index),
            },
            "metrics": dict(self._last_metrics),
            "history": {
                "samples": float(len(speed_values)),
                "max_speed": max(speed_values, default=0.0),
                "min_height": min(height_values, default=0.0),
                "max_energy": max(energy_values, default=0.0),
                "max_contacts": max(contact_values, default=0.0),
                "max_step_ms": max(step_values, default=0.0),
            },
        }

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "solver_index": int(self.solver_index),
                "contact_method_index": int(self.contact_method_index),
                "friction": float(self.friction),
                "restitution": float(self.restitution),
            },
            "speed_history": list(self._speed_history),
            "min_height_history": list(self._min_height_history),
            "energy_history": list(self._energy_history),
            "contact_history": list(self._contact_history),
            "step_ms_history": list(self._step_ms_history),
            "last_metrics": dict(self._last_metrics),
        }

    def restore_replay_state(self, state: dict[str, Any]) -> None:
        controls = state.get("controls", {})
        self.solver_index = max(
            0,
            min(
                int(controls.get("solver_index", self.solver_index)),
                len(_SOLVERS) - 1,
            ),
        )
        self.contact_method_index = max(
            0,
            min(
                int(controls.get("contact_method_index", self.contact_method_index)),
                len(_CONTACT_METHODS) - 1,
            ),
        )
        self.friction = float(controls.get("friction", self.friction))
        self.restitution = float(controls.get("restitution", self.restitution))
        self.world.rigid_body_solver = self._solver()
        self.world.contact_solver_method = self._contact_method()
        self._apply_materials()
        self._restore_history(self._speed_history, state.get("speed_history", []))
        self._restore_history(
            self._min_height_history, state.get("min_height_history", [])
        )
        self._restore_history(self._energy_history, state.get("energy_history", []))
        self._restore_history(self._contact_history, state.get("contact_history", []))
        self._restore_history(self._step_ms_history, state.get("step_ms_history", []))
        self._last_metrics = dict(state.get("last_metrics", {}))
        if not self._last_metrics:
            self._record_metrics()
        self.bridge.sync()

    def _restore_history(self, history: deque[float], values: list[float]) -> None:
        history.clear()
        history.extend(float(value) for value in values)

    def build_panel(self, builder: object, context: object) -> None:
        changed_solver, solver_index = builder.select(
            "Solver", int(self.solver_index), [label for label, _solver in _SOLVERS]
        )
        changed_contact_method, contact_method_index = builder.select(
            "Contact solver",
            int(self.contact_method_index),
            [label for label, _method in _CONTACT_METHODS],
        )
        changed_friction, friction = builder.slider(
            "Friction", float(self.friction), 0.0, 1.0
        )
        changed_restitution, restitution = builder.slider(
            "Restitution", float(self.restitution), 0.0, 0.8
        )

        if changed_solver:
            self.solver_index = int(solver_index)
            self.reset(clear_replay=True)
        if changed_contact_method:
            self.contact_method_index = int(contact_method_index)
            self.reset(clear_replay=True)
        if changed_friction:
            self.friction = float(friction)
            self._apply_materials()
        if changed_restitution:
            self.restitution = float(restitution)
            self._apply_materials()
        if builder.button("Reset baseline scene"):
            self.reset(clear_replay=True)

        metrics = self._last_metrics
        builder.text(f"solver: {metrics.get('solver', self._solver_label())}")
        builder.text(
            f"contact solver: "
            f"{metrics.get('contact_solver', self._contact_method_label())}"
        )
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(f"dynamic bodies: {len(self.dynamic_bodies)}")
        builder.text(f"max speed: {float(metrics.get('max_speed', 0.0)):.3f} m/s")
        builder.text(f"min height: {float(metrics.get('min_height', 0.0)):.3f} m")
        builder.text(f"kinetic energy: {float(metrics.get('energy', 0.0)):.3f} J")
        builder.text(
            f"contacts: {float(metrics.get('contact_count', 0.0)):.0f} | "
            f"step {float(metrics.get('step_ms', 0.0)):.3f} ms"
        )
        builder.plot_lines("Max speed", list(self._speed_history))
        builder.plot_lines("Min height", list(self._min_height_history))
        builder.plot_lines("Energy", list(self._energy_history))
        builder.plot_lines("Contacts", list(self._contact_history))
        builder.plot_lines("Step profile ms", list(self._step_ms_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def build() -> SceneSetup:
    baseline = _RigidBodyBaseline()
    return SceneSetup(
        world=baseline.bridge.render_world,
        pre_step=baseline.pre_step,
        force_drag=baseline.force_drag,
        renderable_provider=baseline.renderable_provider,
        panels=[ScenePanel("Rigid Bodies", baseline.build_panel)],
        info={
            "sx_world": baseline.world,
            "rigid_body_controller": baseline,
            CAPTURE_METRICS_INFO_KEY: baseline.capture_metrics,
            "replay_capture_state": baseline.capture_replay_state,
            "replay_restore_state": baseline.restore_replay_state,
            "replay_sync": baseline.bridge.sync,
        },
    )


SCENE = PythonDemoScene(
    id="rigid_body",
    title="World Rigid Body",
    category="World Rigid Body",
    summary="Four spheres + a box dropping under the World.",
    build=build,
)
