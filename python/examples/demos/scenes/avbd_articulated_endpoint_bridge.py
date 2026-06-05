"""AVBD articulated endpoint bridge preview for the experimental World facade."""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy.simulation_experimental as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_NUM_LINKS = 4
_LINK_LENGTH = 0.56
_LINK_MASS = 0.55


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def build() -> SceneSetup:
    world = sx.World()
    world.multibody_options = sx.MultibodyOptions(
        integration_family="variational integrator"
    )
    world.gravity = (0.0, 0.0, -9.81)
    world.time_step = 0.005

    robot = world.add_multibody("avbd_endpoint_chain")
    base = robot.add_link("base")

    links = []
    parent = base
    for i in range(_NUM_LINKS):
        offset = 0.0 if i == 0 else _LINK_LENGTH
        link = robot.add_link(
            f"link{i}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"hinge{i}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=_translation(offset, 0.0, 0.0),
            ),
        )
        link.mass = _LINK_MASS
        ixx = 0.5 * _LINK_MASS * (0.05**2)
        itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        link.parent_joint.position = [0.18 * (-1.0 if i % 2 else 1.0)]
        links.append(link)
        parent = link

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="avbd_articulated_endpoint_bridge_render")
    bridge.add_link_visual(
        base,
        dart.BoxShape(np.array([0.18, 0.18, 0.18])),
        (0.38, 0.40, 0.44),
        name="avbd_endpoint_base_visual",
    )
    palette = [
        (0.20, 0.55, 0.90),
        (0.30, 0.72, 0.48),
        (0.92, 0.56, 0.20),
        (0.67, 0.40, 0.78),
    ]
    link_shape = dart.BoxShape(np.array([_LINK_LENGTH, 0.075, 0.075]))
    for i, link in enumerate(links):
        bridge.add_link_visual(
            link,
            link_shape,
            palette[i % len(palette)],
            name=f"avbd_endpoint_link{i}_visual",
        )
    bridge.sync()

    tip_height_history: deque[float] = deque(maxlen=140)
    joint_speed_history: deque[float] = deque(maxlen=140)

    def build_panel(builder: object, context: object) -> None:
        speeds = [
            float(np.linalg.norm(np.asarray(link.parent_joint.velocity, dtype=float)))
            for link in links
        ]
        tip_height = float(np.asarray(links[-1].translation, dtype=float).reshape(3)[2])
        tip_height_history.append(tip_height)
        joint_speed_history.append(max(speeds) if speeds else 0.0)

        builder.text("solver: articulated sx world")
        builder.text("AVBD endpoint class: multibody link")
        builder.text("AVBD row path: conservative fallback")
        builder.text(f"links: {robot.num_links}")
        builder.text(f"dofs: {robot.num_dofs}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"tip height: {tip_height:.3f} m")
        builder.plot_lines("Tip height", list(tip_height_history))
        builder.plot_lines("Max joint speed", list(joint_speed_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("AVBD Articulated Endpoint", build_panel)],
        info={
            "sx_world": world,
            "avbd_endpoint_kind": "multibody_link",
            "avbd_row_path": "fallback",
            "dofs": robot.num_dofs,
        },
    )


SCENE = PythonDemoScene(
    id="avbd_articulated_endpoint_bridge",
    title="AVBD Articulated Endpoint Bridge (sx)",
    category="AVBD Rigid Constraints (sx)",
    summary="A multibody-link endpoint preview for the articulated AVBD bridge.",
    build=build,
)
