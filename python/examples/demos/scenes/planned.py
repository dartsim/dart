"""Lightweight roadmap placeholders for high-value World demo ports."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

import dartpy as dart

from ..runner import PythonDemoScene, ScenePanel, SceneSetup


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _build_placeholder(
    scene_id: str,
    title: str,
    legacy_seeds: Iterable[str],
    target: str,
) -> SceneSetup:
    world = dart.gui.RenderWorld(f"planned_{scene_id}")
    world.set_time_step(1.0 / 60.0)

    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), f"{scene_id}_marker", _translation(0.0, 0.0, 0.08)
    )
    frame.set_shape(dart.BoxShape(np.array([0.56, 0.56, 0.16])))
    frame.create_visual_aspect().set_color([0.35, 0.56, 0.86])
    world.add_simple_frame(frame)

    seeds = ", ".join(legacy_seeds)

    def build_panel(builder: object, context: object) -> None:
        builder.text("status: planned World demo")
        builder.text(f"legacy seeds: {seeds}")
        builder.separator()
        builder.text(f"target: {target}")

    return SceneSetup(
        world=world,
        panels=[ScenePanel(title, build_panel)],
        info={"planned_world_port": scene_id, "legacy_seeds": tuple(legacy_seeds)},
    )


def _planned_scene(
    *,
    scene_id: str,
    title: str,
    summary: str,
    legacy_seeds: tuple[str, ...],
    target: str,
) -> PythonDemoScene:
    return PythonDemoScene(
        id=scene_id,
        title=title,
        category="Planned World Ports",
        summary=summary,
        build=lambda: _build_placeholder(scene_id, title, legacy_seeds, target),
    )


INVERSE_KINEMATICS = _planned_scene(
    scene_id="planned_inverse_kinematics",
    title="Inverse Kinematics",
    summary="Placeholder for World-native IK targets and solver handles.",
    legacy_seeds=("wam_ikfast", "kr5_arm", "atlas_ik"),
    target="interactive end-effector targets on World multibodies",
)

SIMBICON_WALKING = _planned_scene(
    scene_id="planned_simbicon_walking",
    title="SIMBICON Walking",
    summary="Placeholder for World-native biped gait control.",
    legacy_seeds=("atlas_simbicon", "g1_simbicon", "biped_stand"),
    target="closed-loop walking controller on a World humanoid",
)

OPERATIONAL_SPACE_CONTROL = _planned_scene(
    scene_id="planned_operational_space_control",
    title="Operational Space Control",
    summary="Placeholder for task-space control on World articulated bodies.",
    legacy_seeds=("operational_space_control",),
    target="task-space Jacobian control with World dynamics diagnostics",
)

ROBOT_PUPPETS = _planned_scene(
    scene_id="planned_robot_puppets",
    title="Robot Model Puppets",
    summary="Placeholder for maintained robot model loading and pose demos.",
    legacy_seeds=("atlas_puppet", "g1_puppet", "hubo_puppet"),
    target="asset-loaded humanoid puppets with World-friendly pose controls",
)

COLLISION_SANDBOX = _planned_scene(
    scene_id="planned_collision_sandbox",
    title="Collision Sandbox",
    summary="Placeholder for collision-pair inspection on the World demos path.",
    legacy_seeds=("collision_sandbox", "point_cloud", "polyhedron_visual"),
    target="interactive collision/debug visualization for World bodies",
)

MOBILE_MANIPULATION = _planned_scene(
    scene_id="planned_mobile_manipulation",
    title="Mobile Manipulation",
    summary="Placeholder for robot and vehicle workflows from the legacy demos.",
    legacy_seeds=("fetch", "vehicle"),
    target="mobile base and manipulator workflows using the World API",
)
