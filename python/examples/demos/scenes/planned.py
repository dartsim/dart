"""Lightweight roadmap placeholders for high-value World demo ports."""

from __future__ import annotations

from dataclasses import dataclass

import dartpy as dart
import numpy as np

from ..runner import PythonDemoScene, ScenePanel, SceneSetup


@dataclass(frozen=True)
class PlannedWorldPort:
    scene_id: str
    title: str
    summary: str
    legacy_seeds: tuple[str, ...]
    target: str
    current_route: str
    unblocker: str
    retire_when: str


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _build_placeholder(
    port: PlannedWorldPort,
) -> SceneSetup:
    scene_id = port.scene_id
    world = dart.gui.DescriptorRenderScene(dart.World(), f"planned_{scene_id}")
    world.set_time_step(1.0 / 60.0)

    frame = dart.SimpleFrame(
        dart.gui.world_render_frame(),
        f"{scene_id}_marker",
        _translation(0.0, 0.0, 0.08),
    )
    frame.set_shape(dart.BoxShape(np.array([0.56, 0.56, 0.16])))
    frame.create_visual_aspect().set_color([0.35, 0.56, 0.86])
    world.add_simple_frame(frame)

    seeds = ", ".join(port.legacy_seeds)

    def build_panel(builder: object, context: object) -> None:
        builder.text("status: planned World demo")
        builder.text(f"try now: {port.current_route}")
        builder.text(f"legacy seeds: {seeds}")
        builder.separator()
        builder.text(f"target: {port.target}")
        builder.text(f"blocked on: {port.unblocker}")
        builder.text(f"replace when: {port.retire_when}")

    return SceneSetup(
        world=world,
        panels=[ScenePanel(port.title, build_panel)],
        info={
            "planned_world_port": scene_id,
            "planned_status": "planned World demo",
            "legacy_seeds": port.legacy_seeds,
            "current_route": port.current_route,
            "target": port.target,
            "unblocker": port.unblocker,
            "retire_when": port.retire_when,
        },
    )


def _planned_scene(
    *,
    scene_id: str,
    title: str,
    summary: str,
    legacy_seeds: tuple[str, ...],
    target: str,
    current_route: str,
    unblocker: str,
    retire_when: str,
) -> PythonDemoScene:
    port = PlannedWorldPort(
        scene_id=scene_id,
        title=title,
        summary=summary,
        legacy_seeds=legacy_seeds,
        target=target,
        current_route=current_route,
        unblocker=unblocker,
        retire_when=retire_when,
    )
    return PythonDemoScene(
        id=port.scene_id,
        title=port.title,
        category="Planned World Ports",
        summary=port.summary,
        build=lambda: _build_placeholder(port),
    )


INVERSE_KINEMATICS = _planned_scene(
    scene_id="planned_inverse_kinematics",
    title="Inverse Kinematics",
    summary="Placeholder for World-native IK targets and solver handles.",
    legacy_seeds=("wam_ikfast", "kr5_arm", "atlas_ik"),
    target="interactive end-effector targets on World multibodies",
    current_route=(
        "rigid_link_jacobian for Jacobian/wrench maps; "
        "Atlas/Hubo Puppet for loaded robot pose controls"
    ),
    unblocker="World-native IK solver handles and target widgets",
    retire_when="an IK target scene has solver diagnostics and capture evidence",
)

SIMBICON_WALKING = _planned_scene(
    scene_id="planned_simbicon_walking",
    title="SIMBICON Walking",
    summary="Placeholder for World-native biped gait control.",
    legacy_seeds=("atlas_simbicon", "g1_simbicon", "biped_stand"),
    target="closed-loop walking controller on a World humanoid",
    current_route="atlas_simbicon for the current gait-target preview",
    unblocker="lateral balance and robust dynamic walking controller evidence",
    retire_when="a closed-loop walking row replaces the target-preview route",
)

OPERATIONAL_SPACE_CONTROL = _planned_scene(
    scene_id="planned_operational_space_control",
    title="Operational Space Control",
    summary="Placeholder for task-space control on World articulated bodies.",
    legacy_seeds=("operational_space_control",),
    target="task-space Jacobian control with World dynamics diagnostics",
    current_route=(
        "rigid_link_jacobian and rigid_multibody_dynamics_terms "
        "for J, J.T wrench, and dynamics-term diagnostics"
    ),
    unblocker="World task-space controller facade and stable tracking example",
    retire_when="an OSC scene owns task-space tracking tests and capture metrics",
)

ROBOT_PUPPETS = _planned_scene(
    scene_id="planned_robot_puppets",
    title="Robot Model Puppets",
    summary="Placeholder for maintained robot model loading and pose demos.",
    legacy_seeds=("atlas_puppet", "g1_puppet", "hubo_puppet"),
    target="asset-loaded humanoid puppets with World-friendly pose controls",
    current_route="Atlas Puppet and Hubo Puppet for current loaded robot models",
    unblocker="remaining tracked robot assets and pose-control coverage",
    retire_when="all intended robot puppet rows are usable World-native scenes",
)

MOBILE_MANIPULATION = _planned_scene(
    scene_id="planned_mobile_manipulation",
    title="Mobile Manipulation",
    summary="Placeholder for robot and vehicle workflows from the legacy demos.",
    legacy_seeds=("fetch", "vehicle"),
    target="mobile base and manipulator workflows using the World API",
    current_route=(
        "Atlas/Hubo Puppet for model loading; rigid_contact_manipulation "
        "for contact-rich manipulation signals"
    ),
    unblocker="mobile-base model loading, controls, and manipulation task setup",
    retire_when="a World mobile-manipulation row has controls and visual evidence",
)
