"""World-loaded humanoid puppet scenes.

These scenes bring back the DART 6 humanoid puppet seeds as DART 7 World
model-loading demos. They intentionally stop short of the old whole-body IK and
support-polygon tooling; the current value is proving that bundled humanoids
load, expose named joints, and can be posed through the World surface.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable, Mapping
from typing import Any

import dartpy as dart
import numpy as np

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup
from ._robot_model_world import (
    add_ground_visual,
    add_robot_link_visuals,
    apply_joint_targets,
    load_atlas_world,
    load_hubo_world,
)

_ATLAS_NEUTRAL = {
    "back_bky": 0.08,
    "l_leg_hpy": -0.18,
    "r_leg_hpy": -0.18,
    "l_leg_kny": 0.34,
    "r_leg_kny": 0.34,
    "l_leg_aky": -0.16,
    "r_leg_aky": -0.16,
    "l_arm_shx": -0.55,
    "r_arm_shx": 0.55,
    "l_arm_ely": 1.15,
    "r_arm_ely": 1.15,
}

_ATLAS_REACH = {
    **_ATLAS_NEUTRAL,
    "l_arm_shx": -1.15,
    "r_arm_shx": 1.15,
    "l_arm_ely": 0.35,
    "r_arm_ely": 0.35,
}

_ATLAS_CROUCH = {
    **_ATLAS_NEUTRAL,
    "l_leg_hpy": -0.55,
    "r_leg_hpy": -0.55,
    "l_leg_kny": 0.95,
    "r_leg_kny": 0.95,
    "l_leg_aky": -0.36,
    "r_leg_aky": -0.36,
}

_HUBO_NEUTRAL = {
    "LHP": -0.15,
    "RHP": -0.15,
    "LKP": 0.30,
    "RKP": 0.30,
    "LAP": -0.12,
    "RAP": -0.12,
    "LSP": 0.35,
    "RSP": -0.35,
    "LEP": -0.70,
    "REP": 0.70,
}

_HUBO_REACH = {
    **_HUBO_NEUTRAL,
    "LSP": 0.85,
    "RSP": -0.85,
    "LEP": -0.25,
    "REP": 0.25,
}

_HUBO_CROUCH = {
    **_HUBO_NEUTRAL,
    "LHP": -0.48,
    "RHP": -0.48,
    "LKP": 0.82,
    "RKP": 0.82,
    "LAP": -0.30,
    "RAP": -0.30,
}


def _blend_targets(
    start: Mapping[str, float],
    end: Mapping[str, float],
    amount: float,
) -> dict[str, float]:
    blend = float(np.clip(amount, 0.0, 1.0))
    names = set(start) | set(end)
    return {
        name: (1.0 - blend) * float(start.get(name, 0.0))
        + blend * float(end.get(name, 0.0))
        for name in names
    }


def _build_robot_puppet(
    *,
    scene_id: str,
    title: str,
    model_label: str,
    loader: Callable[[Mapping[str, float]], tuple[Any, Any]],
    neutral: Mapping[str, float],
    reach: Mapping[str, float],
    crouch: Mapping[str, float],
) -> SceneSetup:
    world, robot = loader(neutral)
    bridge = WorldRenderBridge(world, name=f"{scene_id}_render")
    visual_count = add_robot_link_visuals(bridge, robot, prefix=scene_id)
    add_ground_visual(bridge, name=f"{scene_id}_ground_visual")
    bridge.sync()

    pose_blend = {"value": 0.0, "target": "reach"}
    height_history: deque[float] = deque(maxlen=120)

    def _apply_current_pose() -> None:
        target = reach if pose_blend["target"] == "reach" else crouch
        apply_joint_targets(
            robot, _blend_targets(neutral, target, float(pose_blend["value"]))
        )
        world.update_kinematics()
        bridge.sync()

    def build_panel(builder: object, context: object) -> None:
        pelvis = robot.links[1] if robot.num_links > 1 else robot.links[0]
        height = float(np.asarray(pelvis.translation, dtype=float)[2])
        height_history.append(height)

        builder.text(f"model: {model_label}")
        builder.text(f"links: {robot.num_links}")
        builder.text(f"dofs: {robot.num_dofs}")
        changed, blend = builder.slider(
            "Pose blend", float(pose_blend["value"]), 0.0, 1.0
        )
        if changed:
            pose_blend["value"] = float(blend)
            _apply_current_pose()
        if builder.button("Reach pose"):
            pose_blend["target"] = "reach"
            pose_blend["value"] = 1.0
            _apply_current_pose()
        if builder.button("Crouch pose"):
            pose_blend["target"] = "crouch"
            pose_blend["value"] = 1.0
            _apply_current_pose()
        if builder.button("Neutral pose"):
            pose_blend["value"] = 0.0
            _apply_current_pose()
        builder.plot_lines("Root height", list(height_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel(title, build_panel)],
        info={
            "sx_world": world,
            "robot": scene_id,
            "model": model_label,
            "dofs": robot.num_dofs,
            "links": robot.num_links,
            "visual_links": visual_count,
        },
    )


def build_atlas() -> SceneSetup:
    return _build_robot_puppet(
        scene_id="atlas_puppet",
        title="Atlas Puppet",
        model_label="Atlas v5 no-head",
        loader=load_atlas_world,
        neutral=_ATLAS_NEUTRAL,
        reach=_ATLAS_REACH,
        crouch=_ATLAS_CROUCH,
    )


def build_hubo() -> SceneSetup:
    return _build_robot_puppet(
        scene_id="hubo_puppet",
        title="Hubo Puppet",
        model_label="DRC-Hubo",
        loader=load_hubo_world,
        neutral=_HUBO_NEUTRAL,
        reach=_HUBO_REACH,
        crouch=_HUBO_CROUCH,
    )


def build_g1_placeholder() -> SceneSetup:
    current_route = "Atlas Puppet and Hubo Puppet for current loaded robot models"
    target = "tracked Unitree G1 asset and World model loader"
    unblocker = "tracked Unitree G1 model asset and loader mapping"
    retire_when = "the G1 puppet loads as a World scene with pose controls"
    world = dart.gui.DescriptorRenderScene(dart.World(), "g1_puppet_planned")
    world.set_time_step(1.0 / 60.0)
    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), "g1_puppet_marker", np.eye(4)
    )
    frame.set_shape(dart.BoxShape(np.array([0.42, 0.42, 0.14])))
    frame.create_visual_aspect().set_color([0.35, 0.56, 0.86])
    world.add_simple_frame(frame)

    def build_panel(builder: object, context: object) -> None:
        builder.text("status: planned World demo")
        builder.text(f"try now: {current_route}")
        builder.text("legacy seed: g1_puppet")
        builder.separator()
        builder.text(f"target: {target}")
        builder.text(f"blocked on: {unblocker}")
        builder.text(f"replace when: {retire_when}")

    return SceneSetup(
        world=world,
        panels=[ScenePanel("G1 Puppet", build_panel)],
        info={
            "planned_world_port": "g1_puppet",
            "planned_status": "planned World demo",
            "legacy_seeds": ("g1_puppet",),
            "asset_policy": "missing tracked Unitree G1 model",
            "current_route": current_route,
            "target": target,
            "unblocker": unblocker,
            "retire_when": retire_when,
        },
    )


ATLAS_PUPPET = PythonDemoScene(
    id="atlas_puppet",
    title="Atlas Puppet",
    category="Robot Models",
    summary="Atlas v5 loaded into the DART 7 World with pose controls.",
    build=build_atlas,
)

HUBO_PUPPET = PythonDemoScene(
    id="hubo_puppet",
    title="Hubo Puppet",
    category="Robot Models",
    summary="DRC-Hubo loaded into the DART 7 World with pose controls.",
    build=build_hubo,
)

G1_PUPPET = PythonDemoScene(
    id="g1_puppet",
    title="G1 Puppet",
    category="Planned World Ports",
    summary="Asset-gated placeholder for a Unitree G1 World puppet.",
    build=build_g1_placeholder,
)
