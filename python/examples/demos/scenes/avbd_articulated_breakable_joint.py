"""AVBD articulated breakable-joint scene for the experimental World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_BASE_HALF = np.array([0.16, 0.16, 0.16])
_PAYLOAD_HALF = np.array([0.42, 0.11, 0.11])
_CAPTURED_POSITION = np.array([0.68, 0.0, 0.0])
_CAPTURED_YAW = 0.22
_PULL_FORCE = np.array([0.0, 5.0, 0.0])
_BREAK_FORCE = 1.0e-18
_RESET_BREAK_FORCE = 1.0e6


def _full(half_extents: np.ndarray) -> np.ndarray:
    return 2.0 * half_extents


def _translation(position: np.ndarray) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = position
    return transform


def _normalized_or(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1.0e-12:
        return fallback
    return vector / norm


def _connector_transform(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    x_axis = _normalized_or(end - start, np.array([1.0, 0.0, 0.0]))
    reference = np.array([0.0, 0.0, 1.0])
    if abs(float(reference @ x_axis)) > 0.95:
        reference = np.array([0.0, 1.0, 0.0])
    y_axis = _normalized_or(np.cross(reference, x_axis), np.array([0.0, 1.0, 0.0]))
    z_axis = np.cross(x_axis, y_axis)

    transform = _translation(0.5 * (start + end))
    transform[:3, 0] = x_axis
    transform[:3, 1] = y_axis
    transform[:3, 2] = z_axis
    return transform


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    arm = world.add_multibody("avbd_articulated_breakable_arm")
    base = arm.add_link("avbd_articulated_breakable_base")
    payload = arm.add_link(
        "avbd_articulated_breakable_payload",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_breakable_floating_payload",
            type=sx.JointType.FLOATING,
        ),
    )
    payload.mass = 2.0
    payload.inertia = ((0.2, 0.0, 0.0), (0.0, 0.2, 0.0), (0.0, 0.0, 0.3))
    payload.parent_joint.position = [
        *_CAPTURED_POSITION,
        0.0,
        0.0,
        _CAPTURED_YAW,
    ]

    breakable_joint = world.add_joint(
        payload,
                sx.JointSpec(
            name="avbd_articulated_breakable_hold",
            type=sx.JointType.FIXED,
        )
    )
    breakable_joint.break_force = _BREAK_FORCE
    world.enter_simulation_mode()

    captured_transform = np.asarray(payload.transform, dtype=float).copy()
    captured_position = captured_transform[:3, 3].copy()

    bridge = WorldRenderBridge(world, name="avbd_articulated_breakable_joint_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_breakable_base_visual",
    )
    bridge.add_link_visual(
        payload,
        dart.BoxShape(_full(_PAYLOAD_HALF)),
        (0.88, 0.48, 0.18),
        name="avbd_articulated_breakable_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_breakable_connector_visual",
        _connector_transform(np.zeros(3), captured_position),
    )
    connector.set_shape(
        dart.BoxShape(np.array([np.linalg.norm(captured_position), 0.035, 0.035]))
    )
    connector_visual = connector.create_visual_aspect()
    connector_visual.set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(connector)

    force_state = {"enabled": True}

    def sync_connector() -> None:
        base_pos = np.asarray(base.translation, dtype=float).reshape(3)
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        connector.set_transform(_connector_transform(base_pos, payload_pos))
        if breakable_joint.is_broken:
            connector_visual.set_color([0.95, 0.18, 0.13])
        else:
            connector_visual.set_color([0.78, 0.78, 0.70])

    def reset_joint(break_force: float = _RESET_BREAK_FORCE) -> None:
        payload.parent_joint.velocity = [0.0] * 6
        breakable_joint.break_force = float(break_force)
        breakable_joint.reset_breakage()
        sync_connector()

    def rearm_weak_joint() -> None:
        reset_joint(_BREAK_FORCE)

    def replay_sync() -> None:
        bridge.sync()
        sync_connector()

    def pre_step() -> None:
        if force_state["enabled"]:
            payload.apply_force(_PULL_FORCE)
        bridge.pre_step()
        sync_connector()

    bridge.sync()
    sync_connector()

    offset_history: deque[float] = deque(maxlen=160)
    lateral_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        payload_pos = np.asarray(payload.translation, dtype=float).reshape(3)
        translation_error = float(np.linalg.norm(payload_pos - captured_position))
        lateral_drift = float(payload_pos[1] - captured_position[1])
        broken = 1.0 if breakable_joint.is_broken else 0.0
        offset_history.append(translation_error)
        lateral_history.append(lateral_drift)
        broken_history.append(broken)

        builder.text("solver: AVBD world-anchored articulated fixed point joint")
        builder.text(f"joint: {breakable_joint.name}")
        builder.text(f"state: {'broken' if breakable_joint.is_broken else 'intact'}")
        builder.text(f"break force: {breakable_joint.break_force:.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"captured-pose drift: {translation_error:.4f} m")
        builder.text(f"lateral release: {lateral_drift:.4f} m")
        if builder.button("Reset joint"):
            reset_joint(_RESET_BREAK_FORCE)
        if builder.button("Re-arm weak joint"):
            rearm_weak_joint()
        changed, enabled = builder.checkbox(
            "Apply pull force",
            bool(force_state["enabled"]),
        )
        if changed:
            force_state["enabled"] = bool(enabled)
        builder.plot_lines("Captured-pose drift", list(offset_history))
        builder.plot_lines("Lateral release", list(lateral_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Breakable Joint", build_panel)],
        info={
            "sx_world": world,
            "joint": breakable_joint,
            "base": base,
            "payload": payload,
            "connector": connector,
            "world_anchor": np.zeros(3),
            "captured_transform": captured_transform,
            "captured_position": captured_position,
            "break_force": _BREAK_FORCE,
            "reset_break_force": _RESET_BREAK_FORCE,
            "pull_force": _PULL_FORCE.copy(),
            "reset_joint": reset_joint,
            "rearm_weak_joint": rearm_weak_joint,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_breakable_joint",
    title="AVBD Articulated Breakable Joint (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public world-anchored articulated fixed point joint breaks, "
    "releases, and can re-engage through the AVBD variational bridge.",
    build=build,
)
