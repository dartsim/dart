"""Capture-first rigid IPC stack stress packet.

This scene is intentionally kept in the non-numbered Rigid IPC shelf. It gives
users a GUI/capture view of the heavier resting-stack case that is too slow to
promote into the default World Rigid Body workflow until the rigid IPC runtime
cost improves.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import dartpy as dart
import dartpy as sx
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import (
    CAPTURE_METRICS_INFO_KEY,
    PythonDemoScene,
    ScenePanel,
    SceneSetup,
)

_BOX_HALF = np.array([0.16, 0.16, 0.12])
_GROUND_HALF = np.array([0.55, 0.36, 0.04])
_GAP = 0.004
_TIME_STEP = 1.0 / 240.0
_HISTORY = 120
_DEFAULT_FRAME_BUDGET_MS = 33.3


@dataclass(frozen=True)
class _StackPacketSpec:
    scene_id: str
    title: str
    panel_title: str
    panel_heading: str
    summary: str
    box_count: int
    box_half: np.ndarray
    ground_half: np.ndarray
    lateral_step: float
    mass_start: float
    mass_step: float
    frame_budget_ms: float


_STACK_PACKET_SPEC = _StackPacketSpec(
    scene_id="rigid_ipc_stack_packet",
    title="Rigid IPC Stack Packet",
    panel_title="Rigid IPC Stack Packet",
    panel_heading="capture-first rigid IPC stack",
    summary=(
        "A four-box rigid IPC resting stack with wall-time and clearance "
        "diagnostics for capture-first performance evidence."
    ),
    box_count=4,
    box_half=_BOX_HALF,
    ground_half=_GROUND_HALF,
    lateral_step=0.012,
    mass_start=1.0,
    mass_step=0.35,
    frame_budget_ms=_DEFAULT_FRAME_BUDGET_MS,
)

_HEAVY_STACK_PACKET_SPEC = _StackPacketSpec(
    scene_id="rigid_ipc_heavy_stack_packet",
    title="Rigid IPC Heavy Stack Packet",
    panel_title="Rigid IPC Heavy Stack Packet",
    panel_heading="capture-first heavy rigid IPC stack",
    summary=(
        "A six-box rigid IPC stack with a mass gradient, wall-time, clearance, "
        "and drift diagnostics for capture-first stress evidence."
    ),
    box_count=6,
    box_half=_BOX_HALF,
    ground_half=np.array([0.62, 0.40, 0.04]),
    lateral_step=0.018,
    mass_start=1.0,
    mass_step=0.65,
    frame_budget_ms=_DEFAULT_FRAME_BUDGET_MS,
)


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * np.asarray(half_extents, dtype=float)


def _transform_at(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(position, dtype=float)
    return transform


class _RigidIpcStackPacket:
    def __init__(self, spec: _StackPacketSpec) -> None:
        self.spec = spec
        self.friction = 0.80
        self.frame_budget_ms = float(spec.frame_budget_ms)
        self.world = sx.World(
            time_step=_TIME_STEP,
            rigid_body_solver=sx.RigidBodySolver.IPC,
        )
        self.world.step_profiling_enabled = True

        self.ground = self.world.add_rigid_body(
            f"{spec.scene_id}_ground",
            position=(0.0, 0.0, -spec.ground_half[2]),
        )
        self.ground.is_static = True
        self.ground.set_collision_shape(sx.CollisionShape.box(spec.ground_half))

        self.boxes: list[Any] = []
        self.initial_positions: list[np.ndarray] = []
        for index in range(spec.box_count):
            lateral = spec.lateral_step * ((index % 2) - 0.5)
            position = np.array(
                [
                    lateral,
                    0.0,
                    spec.box_half[2]
                    + _GAP
                    + index * (2.0 * spec.box_half[2] + _GAP),
                ],
                dtype=float,
            )
            box = self.world.add_rigid_body(f"{spec.scene_id}_box{index}")
            box.set_collision_shape(sx.CollisionShape.box(spec.box_half))
            self.boxes.append(box)
            self.initial_positions.append(position)

        self.world.enter_simulation_mode()

        self.bridge = WorldRenderBridge(
            self.world,
            name=f"{spec.scene_id}_render",
        )
        self.bridge.add_rigid_body_visual(
            self.ground,
            dart.BoxShape(_full(spec.ground_half)),
            (0.40, 0.42, 0.45),
            name=f"{spec.scene_id}_ground_visual",
        )
        for index, box in enumerate(self.boxes):
            shade = 0.10 * index
            self.bridge.add_rigid_body_visual(
                box,
                dart.BoxShape(_full(spec.box_half)),
                (0.90, min(0.30 + shade, 0.75), 0.20),
                name=f"{spec.scene_id}_box{index}_visual",
            )

        self._step_ms_history: deque[float] = deque(maxlen=_HISTORY)
        self._clearance_history: deque[float] = deque(maxlen=_HISTORY)
        self._speed_history: deque[float] = deque(maxlen=_HISTORY)
        self._drift_history: deque[float] = deque(maxlen=_HISTORY)
        self._contact_history: deque[float] = deque(maxlen=_HISTORY)
        self._last_metrics: dict[str, float | str] = {}
        self._reset()

    def _apply_parameters(self) -> None:
        self.ground.friction = float(self.friction)
        self.ground.restitution = 0.0
        for index, box in enumerate(self.boxes):
            box.mass = self.spec.mass_start + self.spec.mass_step * index
            box.friction = float(self.friction)
            box.restitution = 0.0

    def _reset(self) -> None:
        self._apply_parameters()
        for box, position in zip(self.boxes, self.initial_positions, strict=True):
            box.transform = _transform_at(position)
            box.linear_velocity = (0.0, 0.0, 0.0)
            box.angular_velocity = (0.0, 0.0, 0.0)
            box.clear_force()
            box.clear_torque()
        self.world.time = 0.0
        try:
            self.world.clear_replay_recording()
        except Exception:  # noqa: BLE001
            pass
        self.world.update_kinematics()
        for history in (
            self._step_ms_history,
            self._clearance_history,
            self._speed_history,
            self._drift_history,
            self._contact_history,
        ):
            history.clear()
        self._last_metrics.clear()
        self.bridge.sync()
        self._record_metrics()

    def _step_profile_ms(self) -> float:
        try:
            profile = self.world.last_step_profile
            if profile.is_empty():
                return 0.0
            return float(profile.wall_time_ms)
        except Exception:  # noqa: BLE001
            return 0.0

    def _sample(self) -> dict[str, float | str]:
        positions = [
            np.asarray(box.translation, dtype=float) for box in self.boxes
        ]
        velocities = [
            np.asarray(box.linear_velocity, dtype=float) for box in self.boxes
        ]
        box_half = self.spec.box_half
        bottom_clearance = float(positions[0][2] - box_half[2])
        inter_box_clearances = [
            float(positions[index][2] - positions[index - 1][2] - 2.0 * box_half[2])
            for index in range(1, len(positions))
        ]
        min_clearance = min([bottom_clearance, *inter_box_clearances])
        max_speed = max(float(np.linalg.norm(velocity)) for velocity in velocities)
        top_drift = float(
            np.linalg.norm((positions[-1] - self.initial_positions[-1])[0:2])
        )
        ideal_top_height = float(
            box_half[2] + _GAP + (len(self.boxes) - 1) * (2.0 * box_half[2] + _GAP)
        )
        height_error = float(positions[-1][2] - ideal_top_height)
        contact_count = float(len(self.world.collide()))
        step_ms = self._step_profile_ms()
        if min_clearance < -0.004:
            status = "overlap"
        elif step_ms > float(self.frame_budget_ms):
            status = "capture-first"
        elif max_speed < 0.015:
            status = "standing"
        else:
            status = "settling"
        return {
            "box_count": float(len(self.boxes)),
            "contact_count": contact_count,
            "height_error": height_error,
            "max_speed": max_speed,
            "min_clearance": min_clearance,
            "status": status,
            "step_ms": step_ms,
            "top_drift": top_drift,
        }

    def _record_metrics(self) -> None:
        metrics = self._sample()
        self._last_metrics = metrics
        self._step_ms_history.append(float(metrics["step_ms"]))
        self._clearance_history.append(float(metrics["min_clearance"]))
        self._speed_history.append(float(metrics["max_speed"]))
        self._drift_history.append(float(metrics["top_drift"]))
        self._contact_history.append(float(metrics["contact_count"]))

    def capture_metrics(self) -> dict[str, Any]:
        metrics = dict(self._last_metrics or self._sample())
        metrics.update(
            {
                "benchmark": "bm_rigid_ipc_solver",
                "capture_first": True,
                "controls": {
                    "friction": float(self.friction),
                    "frame_budget_ms": float(self.frame_budget_ms),
                },
                "frame_budget_ms": float(self.frame_budget_ms),
                "friction": float(self.friction),
                "row": self.spec.scene_id,
                "solver": "ipc",
                "executor": "World.step default",
                "top_mass": float(self.boxes[-1].mass),
                "world_time": float(self.world.time),
            }
        )
        return metrics

    def capture_replay_state(self) -> dict[str, Any]:
        return {
            "controls": {
                "friction": float(self.friction),
                "frame_budget_ms": float(self.frame_budget_ms),
            },
            "step_ms_history": list(self._step_ms_history),
            "clearance_history": list(self._clearance_history),
            "speed_history": list(self._speed_history),
            "drift_history": list(self._drift_history),
            "contact_history": list(self._contact_history),
            "last_metrics": dict(self._last_metrics),
        }

    def restore_replay_state(self, snapshot: dict[str, Any]) -> None:
        controls = snapshot.get("controls", {})
        self.friction = float(controls.get("friction", self.friction))
        self.frame_budget_ms = float(
            controls.get("frame_budget_ms", self.frame_budget_ms)
        )
        self._apply_parameters()
        self._restore_history(
            self._step_ms_history, snapshot.get("step_ms_history", [])
        )
        self._restore_history(
            self._clearance_history, snapshot.get("clearance_history", [])
        )
        self._restore_history(self._speed_history, snapshot.get("speed_history", []))
        self._restore_history(self._drift_history, snapshot.get("drift_history", []))
        self._restore_history(
            self._contact_history, snapshot.get("contact_history", [])
        )
        self._last_metrics = dict(snapshot.get("last_metrics", self._last_metrics))
        self.world.update_kinematics()
        self.bridge.sync()

    @staticmethod
    def _restore_history(history: deque[float], values: object) -> None:
        history.clear()
        if isinstance(values, list):
            history.extend(float(value) for value in values)

    def pre_step(self) -> None:
        self.world.step()
        self._record_metrics()
        self.bridge.sync()

    def force_drag(self, event: dict[str, Any]) -> None:
        self.bridge.force_drag(event)

    def renderable_provider(self) -> list[Any]:
        return self.bridge.renderable_provider()

    def build_panel(self, builder: Any, context: Any) -> None:
        changed_friction, friction = builder.slider(
            "Friction",
            float(self.friction),
            0.0,
            1.0,
        )
        if changed_friction:
            self.friction = float(friction)

        changed_budget, frame_budget = builder.slider(
            "Frame budget ms",
            float(self.frame_budget_ms),
            8.0,
            120.0,
        )
        if changed_budget:
            self.frame_budget_ms = float(frame_budget)

        if changed_friction:
            self._reset()
        if builder.button("Reset stack packet"):
            self._reset()

        metrics = self._last_metrics or self._sample()
        builder.separator()
        builder.text(self.spec.panel_heading)
        builder.text("not a numbered World Rigid Body workflow row")
        builder.text("benchmark owner: bm_rigid_ipc_solver")
        builder.text(f"solver: rigid IPC | boxes: {self.spec.box_count}")
        builder.text(f"top mass: {self.boxes[-1].mass:.2f} kg")
        builder.text(f"world time: {self.world.time:.3f} s")
        builder.text(
            f"status {metrics['status']} | step {float(metrics['step_ms']):.3f} ms "
            f"/ {self.frame_budget_ms:.1f} ms"
        )
        builder.text(
            f"clearance {float(metrics['min_clearance']):.4f} m | "
            f"contacts {float(metrics['contact_count']):.0f} | "
            f"top drift {float(metrics['top_drift']):.4f} m"
        )
        builder.text(f"height error {float(metrics['height_error']):.4f} m")
        builder.plot_lines("Step wall ms", list(self._step_ms_history))
        builder.plot_lines("Min clearance", list(self._clearance_history))
        builder.plot_lines("Max speed", list(self._speed_history))
        builder.plot_lines("Top drift", list(self._drift_history))
        builder.plot_lines("Contact count", list(self._contact_history))
        builder.separator()
        self.bridge.build_control_panel(builder, context)


def _build_from_spec(spec: _StackPacketSpec) -> SceneSetup:
    packet = _RigidIpcStackPacket(spec)
    return SceneSetup(
        world=packet.bridge.render_world,
        pre_step=packet.pre_step,
        force_drag=packet.force_drag,
        renderable_provider=packet.renderable_provider,
        panels=[ScenePanel(spec.panel_title, packet.build_panel)],
        info={
            "sx_world": packet.world,
            "rigid_body_solver": "ipc",
            f"{spec.scene_id}": True,
            f"{spec.scene_id}_benchmark": "bm_rigid_ipc_solver",
            f"{spec.scene_id}_capture_first": True,
            f"{spec.scene_id}_controller": packet,
            "rigid_ipc_stack_packet_variant": spec.scene_id,
            "replay_capture_state": packet.capture_replay_state,
            "replay_restore_state": packet.restore_replay_state,
            "replay_sync": packet.bridge.sync,
            CAPTURE_METRICS_INFO_KEY: packet.capture_metrics,
        },
    )


def build() -> SceneSetup:
    return _build_from_spec(_STACK_PACKET_SPEC)


def build_heavy() -> SceneSetup:
    return _build_from_spec(_HEAVY_STACK_PACKET_SPEC)


SCENE = PythonDemoScene(
    id=_STACK_PACKET_SPEC.scene_id,
    title=_STACK_PACKET_SPEC.title,
    category="Rigid IPC",
    summary=_STACK_PACKET_SPEC.summary,
    build=build,
)

HEAVY_SCENE = PythonDemoScene(
    id=_HEAVY_STACK_PACKET_SPEC.scene_id,
    title=_HEAVY_STACK_PACKET_SPEC.title,
    category="Rigid IPC",
    summary=_HEAVY_STACK_PACKET_SPEC.summary,
    build=build_heavy,
)
