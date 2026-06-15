"""AVBD same-multibody articulated spherical breakable-joint scene."""

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
_CHILD_ANCHOR = np.array([-0.34, 0.0, 0.0])
_FORCE_POINT = _CHILD_ANCHOR + np.array([0.55, 0.0, 0.0])
_PULL_FORCE = np.array([0.0, 5.0, 0.0])
_BREAK_FORCE = 1.0e-18
_RESET_BREAK_FORCE = 1.0e12


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


def _link_anchor(link: sx.Link, local_anchor: np.ndarray) -> np.ndarray:
    return np.asarray(link.translation, dtype=float).reshape(3) + (
        np.asarray(link.rotation, dtype=float) @ local_anchor
    )


def _local_anchor_for_world(link: sx.Link, world_point: np.ndarray) -> np.ndarray:
    rotation = np.asarray(link.rotation, dtype=float).reshape(3, 3)
    translation = np.asarray(link.translation, dtype=float).reshape(3)
    return rotation.T @ (world_point - translation)


def build() -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, 0.0))
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )

    arm = world.add_multibody("avbd_articulated_spherical_pair_breakable_arm")
    base = arm.add_link("avbd_articulated_spherical_pair_breakable_base")
    payload = arm.add_link(
        "avbd_articulated_spherical_pair_breakable_payload",
        parent=base,
        joint=sx.JointSpec(
            name="avbd_articulated_spherical_pair_breakable_floating_payload",
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

    captured_child_anchor = _link_anchor(payload, _CHILD_ANCHOR)
    parent_anchor = _local_anchor_for_world(base, captured_child_anchor)
    breakable_joint = world.add_articulated_spherical_joint(
        "avbd_articulated_spherical_pair_breakable_socket",
        base,
        payload,
        parent_anchor=parent_anchor.tolist(),
        child_anchor=_CHILD_ANCHOR.tolist(),
    )
    breakable_joint.break_force = _BREAK_FORCE
    world.enter_simulation_mode()

    captured_rotation = np.asarray(payload.rotation, dtype=float).copy()

    bridge = WorldRenderBridge(
        world, name="avbd_articulated_spherical_pair_breakable_joint_render"
    )
    bridge.add_link_visual(
        base,
        dart.BoxShape(_full(_BASE_HALF)),
        (0.26, 0.29, 0.34),
        name="avbd_articulated_spherical_pair_breakable_base_visual",
    )
    bridge.add_link_visual(
        payload,
        dart.BoxShape(_full(_PAYLOAD_HALF)),
        (0.88, 0.48, 0.18),
        name="avbd_articulated_spherical_pair_breakable_payload_visual",
    )

    connector = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        "avbd_articulated_spherical_pair_breakable_connector_visual",
        _connector_transform(
            _link_anchor(base, parent_anchor), _link_anchor(payload, _CHILD_ANCHOR)
        ),
    )
    connector.set_shape(dart.BoxShape(np.array([0.08, 0.035, 0.035])))
    connector_visual = connector.create_visual_aspect()
    connector_visual.set_color([0.78, 0.78, 0.70])
    bridge.render_world.add_simple_frame(connector)

    force_state = {"enabled": True}

    def parent_anchor_position() -> np.ndarray:
        return _link_anchor(base, parent_anchor)

    def payload_anchor() -> np.ndarray:
        return _link_anchor(payload, _CHILD_ANCHOR)

    def sync_connector() -> None:
        connector.set_transform(
            _connector_transform(parent_anchor_position(), payload_anchor())
        )
        if breakable_joint.is_broken:
            connector_visual.set_color([0.95, 0.18, 0.13])
        else:
            connector_visual.set_color([0.78, 0.78, 0.70])

    def reset_joint(force: float) -> None:
        payload.parent_joint.velocity = [0.0] * 6
        breakable_joint.break_force = force
        breakable_joint.reset_breakage()

    def rearm_weak_joint() -> None:
        reset_joint(_BREAK_FORCE)

    def replay_sync() -> None:
        bridge.sync()
        sync_connector()

    def pre_step() -> None:
        if force_state["enabled"]:
            payload.apply_force(_PULL_FORCE, _FORCE_POINT)
        bridge.pre_step()
        sync_connector()

    bridge.sync()
    sync_connector()

    anchor_history: deque[float] = deque(maxlen=160)
    rotation_history: deque[float] = deque(maxlen=160)
    broken_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        anchor_residual = float(np.linalg.norm(payload_anchor() - parent_anchor_position()))
        rotation_error = float(
            np.linalg.norm(np.asarray(payload.rotation, dtype=float) - captured_rotation)
        )
        broken = 1.0 if breakable_joint.is_broken else 0.0
        anchor_history.append(anchor_residual)
        rotation_history.append(rotation_error)
        broken_history.append(broken)

        builder.text("solver: AVBD same-multibody articulated spherical point joint")
        builder.text(f"joint: {breakable_joint.name}")
        builder.text(f"state: {'broken' if breakable_joint.is_broken else 'intact'}")
        builder.text(f"break force: {breakable_joint.break_force:.1e} N")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"anchor residual: {anchor_residual:.4f} m")
        builder.text(f"free orientation: {rotation_error:.4f}")
        if builder.button("Reset socket"):
            reset_joint(_RESET_BREAK_FORCE)
        if builder.button("Re-arm weak socket"):
            rearm_weak_joint()
        changed, enabled = builder.checkbox(
            "Apply off-center pull",
            bool(force_state["enabled"]),
        )
        if changed:
            force_state["enabled"] = bool(enabled)
        builder.plot_lines("Anchor residual", list(anchor_history))
        builder.plot_lines("Free orientation", list(rotation_history))
        builder.plot_lines("Broken", list(broken_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[
            ScenePanel(
                "AVBD Articulated Pair Spherical Breakable Joint", build_panel
            )
        ],
        info={
            "sx_world": world,
            "joint": breakable_joint,
            "base": base,
            "payload": payload,
            "connector": connector,
            "parent_anchor": parent_anchor.copy(),
            "child_anchor": _CHILD_ANCHOR.copy(),
            "parent_anchor_position": parent_anchor_position,
            "payload_anchor": payload_anchor,
            "captured_rotation": captured_rotation,
            "break_force": _BREAK_FORCE,
            "reset_break_force": _RESET_BREAK_FORCE,
            "pull_force": _PULL_FORCE.copy(),
            "force_point": _FORCE_POINT.copy(),
            "reset_joint": reset_joint,
            "rearm_weak_joint": rearm_weak_joint,
            "replay_sync": replay_sync,
            "replay_live_step_is_stateless": True,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_spherical_pair_breakable_joint",
    title="AVBD Articulated Pair Spherical Breakable Joint (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A public same-multibody articulated spherical point joint breaks, "
    "releases its anchor, and re-engages while orientation stays free.",
    build=build,
)
