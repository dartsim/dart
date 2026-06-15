"""AVBD articulated high mass-ratio chain smoke scene."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_NARROW_SCENE_ID = "avbd_articulated_high_ratio_chain"
_PAPER_SCALE_SCENE_ID = "avbd_paper_scale_high_ratio_chain"
_NARROW_NUM_LINKS = 5
_PAPER_SCALE_NUM_LINKS = 50
_LINK_LENGTH = 0.45
_LINK_WIDTH = 0.08
_LIGHT_LINK_MASS = 1.0
_NARROW_TIP_LINK_MASS = 200.0
_PAPER_SCALE_TIP_LINK_MASS = 50000.0
_NARROW_AUTO_RESET_SECONDS = 1.0
_PAPER_SCALE_AUTO_RESET_SECONDS = 0.16
_PAPER_SCALE_MAX_ITERATIONS = 200
_PAPER_SCALE_TOLERANCE = 1.0e-9


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _link_tip(link: sx.Link, link_length: float) -> np.ndarray:
    return np.asarray(link.translation, dtype=float).reshape(3) + (
        np.asarray(link.rotation, dtype=float).reshape(3, 3)
        @ np.array([link_length, 0.0, 0.0])
    )


def _joint_angle(joint: sx.Joint) -> float:
    return float(np.asarray(joint.position, dtype=float).reshape(-1)[0])


def _build_high_ratio_chain(
    *,
    scene_id: str,
    panel_title: str,
    num_links: int,
    tip_link_mass: float,
    auto_reset_seconds: float,
    variational_max_iterations: int | None = None,
    variational_tolerance: float | None = None,
) -> SceneSetup:
    world = sx.World(time_step=0.005, gravity=(0.0, 0.0, -9.81))
    multibody_options: dict[str, object] = {
        "integration_family": sx.MultibodyIntegrationFamily.VARIATIONAL
    }
    if variational_max_iterations is not None:
        multibody_options["variational_max_iterations"] = variational_max_iterations
    if variational_tolerance is not None:
        multibody_options["variational_tolerance"] = variational_tolerance
    world.multibody_options = sx.MultibodyOptions(**multibody_options)

    chain = world.add_multibody(scene_id)
    base = chain.add_link(f"{scene_id}_base")
    parent = base
    links: list[sx.Link] = []
    joints: list[sx.Joint] = []

    for index in range(num_links):
        transform_from_parent = _translation(
            0.0 if index == 0 else _LINK_LENGTH,
            0.0,
            0.0,
        )
        link = chain.add_link(
            f"{scene_id}_link_{index}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"{scene_id}_joint_{index}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=transform_from_parent,
            ),
        )
        mass = tip_link_mass if index == num_links - 1 else _LIGHT_LINK_MASS
        link.mass = mass
        transverse_inertia = mass * (_LINK_LENGTH**2) / 12.0
        axis_inertia = mass * (_LINK_WIDTH**2) / 6.0
        link.inertia = (
            (axis_inertia, 0.0, 0.0),
            (0.0, transverse_inertia, 0.0),
            (0.0, 0.0, transverse_inertia),
        )
        links.append(link)
        joints.append(link.parent_joint)
        parent = link

    world.enter_simulation_mode()
    initial_tip = _link_tip(links[-1], _LINK_LENGTH).copy()

    bridge = WorldRenderBridge(world, name=f"{scene_id}_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(np.array([0.12, 0.18, 0.18])),
        (0.26, 0.29, 0.34),
        name=f"{scene_id}_base_visual",
    )
    palette = [
        (0.20, 0.55, 0.90),
        (0.30, 0.69, 0.31),
        (0.95, 0.61, 0.16),
        (0.61, 0.35, 0.71),
        (0.88, 0.24, 0.18),
    ]
    visual_offset = _translation(0.5 * _LINK_LENGTH, 0.0, 0.0)
    for index, link in enumerate(links):
        width = 1.35 * _LINK_WIDTH if index == num_links - 1 else _LINK_WIDTH
        bridge.add_link_visual(
            link,
            dart.BoxShape(np.array([_LINK_LENGTH, width, width])),
            palette[index % len(palette)],
            name=f"{scene_id}_link_{index}_visual",
            local_transform=visual_offset,
        )

    replay_state = {"enabled": True, "last_reset": 0.0, "cycles": 0}

    def tip_position() -> np.ndarray:
        return _link_tip(links[-1], _LINK_LENGTH)

    def max_abs_joint_angle() -> float:
        return max(abs(_joint_angle(joint)) for joint in joints)

    def reset_chain() -> None:
        for joint in joints:
            joint.position = [0.0]
            joint.velocity = [0.0]
        replay_state["last_reset"] = float(world.time)
        replay_state["cycles"] = int(replay_state["cycles"]) + 1
        bridge.sync()

    def replay_capture_state() -> dict[str, object]:
        return dict(replay_state)

    def replay_restore_state(state: dict[str, object]) -> None:
        replay_state["enabled"] = bool(state.get("enabled", True))
        replay_state["last_reset"] = float(state.get("last_reset", 0.0))
        replay_state["cycles"] = int(state.get("cycles", 0))
        bridge.sync()

    def pre_step() -> None:
        if (
            bool(replay_state["enabled"])
            and world.time - float(replay_state["last_reset"]) >= auto_reset_seconds
        ):
            reset_chain()
        bridge.pre_step()

    bridge.sync()

    tip_drop_history: deque[float] = deque(maxlen=160)
    tip_height_history: deque[float] = deque(maxlen=160)
    angle_history: deque[float] = deque(maxlen=160)

    def build_panel(builder: object, context: object) -> None:
        tip = tip_position()
        tip_drop = float(initial_tip[2] - tip[2])
        max_angle = max_abs_joint_angle()
        tip_drop_history.append(tip_drop)
        tip_height_history.append(float(tip[2]))
        angle_history.append(max_angle)

        builder.text("solver: AVBD articulated variational chain")
        builder.text(f"links: {num_links}")
        builder.text(f"mass ratio: {tip_link_mass / _LIGHT_LINK_MASS:.0f}:1")
        if variational_max_iterations is not None:
            builder.text(f"max iterations: {variational_max_iterations}")
        if variational_tolerance is not None:
            builder.text(f"tolerance: {variational_tolerance:.1e}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"tip drop: {tip_drop:.3f} m")
        builder.text(f"max joint angle: {max_angle:.3f} rad")
        builder.text(f"replay cycles: {int(replay_state['cycles'])}")
        if builder.button("Reset chain"):
            reset_chain()
        changed, enabled = builder.checkbox(
            "Loop replay",
            bool(replay_state["enabled"]),
        )
        if changed:
            replay_state["enabled"] = bool(enabled)
            replay_state["last_reset"] = float(world.time)
        builder.plot_lines("Tip drop", list(tip_drop_history))
        builder.plot_lines("Tip height", list(tip_height_history))
        builder.plot_lines("Max joint angle", list(angle_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(panel_title, build_panel)],
        info={
            "sx_world": world,
            "multibody": chain,
            "base": base,
            "links": links,
            "joints": joints,
            "num_links": num_links,
            "light_link_mass": _LIGHT_LINK_MASS,
            "tip_link_mass": tip_link_mass,
            "mass_ratio": tip_link_mass / _LIGHT_LINK_MASS,
            "auto_reset_seconds": auto_reset_seconds,
            "variational_max_iterations": variational_max_iterations,
            "variational_tolerance": variational_tolerance,
            "replay_state": replay_state,
            "initial_tip": initial_tip,
            "tip_position": tip_position,
            "max_abs_joint_angle": max_abs_joint_angle,
            "reset_chain": reset_chain,
            "replay_capture_state": replay_capture_state,
            "replay_restore_state": replay_restore_state,
        },
    )


def build() -> SceneSetup:
    return _build_high_ratio_chain(
        scene_id=_NARROW_SCENE_ID,
        panel_title="AVBD Articulated High-Ratio Chain",
        num_links=_NARROW_NUM_LINKS,
        tip_link_mass=_NARROW_TIP_LINK_MASS,
        auto_reset_seconds=_NARROW_AUTO_RESET_SECONDS,
    )


def build_paper_scale() -> SceneSetup:
    return _build_high_ratio_chain(
        scene_id=_PAPER_SCALE_SCENE_ID,
        panel_title="AVBD Paper-Scale High-Ratio Chain",
        num_links=_PAPER_SCALE_NUM_LINKS,
        tip_link_mass=_PAPER_SCALE_TIP_LINK_MASS,
        auto_reset_seconds=_PAPER_SCALE_AUTO_RESET_SECONDS,
        variational_max_iterations=_PAPER_SCALE_MAX_ITERATIONS,
        variational_tolerance=_PAPER_SCALE_TOLERANCE,
    )


SCENE = PythonDemoScene(
    id=_NARROW_SCENE_ID,
    title="AVBD Articulated High-Ratio Chain (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A five-link AVBD variational chain swings with a 200:1 heavy tip "
    "as a narrow high mass-ratio articulated smoke case.",
    build=build,
)

PAPER_SCALE_SCENE = PythonDemoScene(
    id=_PAPER_SCALE_SCENE_ID,
    title="AVBD Paper-Scale High-Ratio Chain (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A 50-link AVBD variational chain with a 50,000:1 heavy tip "
    "for paper-scale high mass-ratio visual evidence.",
    build=build_paper_scale,
)
