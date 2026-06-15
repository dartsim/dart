"""Variational-integrator scene: a passive chain that swings without damping.

A 5-link revolute chain is released from horizontal under gravity on the
World with the **variational integrator** selected
(``MultibodyOptions(integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL)``). Because the
integrator is symplectic, the chain keeps swinging with no secular energy
loss — visually it does not slowly wind down the way a dissipative
(semi-implicit Euler) step would. This is the headline visual check for the VI.

World owns the physics; WorldRenderBridge mirrors each link's world transform
onto a SimpleFrame box visual so the C++ viewer renders the motion.
"""

from __future__ import annotations

from collections import deque

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, ScenePanel, SceneSetup

_NUM_LINKS = 5
_LINK_LENGTH = 0.5
_LINK_MASS = 0.5


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def build() -> SceneSetup:
    world = sx.World()
    # Select the variational integrator before entering simulation mode; the
    # default path stays semi-implicit Euler when this is left unset.
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    world.gravity = (0.0, 0.0, -9.81)
    world.time_step = 0.005

    robot = world.add_multibody("chain")
    base = robot.add_link("base")

    parent = base
    links = []
    # Build the chain extending along +X with revolute joints about +Y, so it
    # starts horizontal and swings in the X-Z plane. Joints at 0 ⇒ released
    # from horizontal; gravity does the rest.
    for i in range(_NUM_LINKS):
        # The first joint sits at the base origin; later joints step one link
        # length along the parent's body frame.
        offset = 0.0 if i == 0 else _LINK_LENGTH
        link = robot.add_link(
            f"link{i}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"joint{i}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 1.0, 0.0),
                transform_from_parent=_translation(offset, 0.0, 0.0),
            ),
        )
        link.mass = _LINK_MASS
        # Slender-rod inertia about the transverse axes; small about the axis.
        ixx = 0.5 * _LINK_MASS * (0.05**2)
        itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        links.append(link)
        parent = link

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="variational_chain_render")
    palette = [
        (0.90, 0.30, 0.24),
        (0.95, 0.61, 0.16),
        (0.30, 0.69, 0.31),
        (0.20, 0.55, 0.90),
        (0.61, 0.35, 0.71),
    ]
    for i, link in enumerate(links):
        bridge.add_link_visual(
            link,
            dart.BoxShape(np.array([_LINK_LENGTH, 0.08, 0.08])),
            palette[i % len(palette)],
            name=f"link{i}_visual",
    )
    bridge.sync()

    tip_height_history: deque[float] = deque(maxlen=120)

    def build_panel(builder: object, context: object) -> None:
        tip_height = float(np.asarray(links[-1].translation, dtype=float).reshape(3)[2])
        tip_height_history.append(tip_height)
        builder.text("integrator: variational")
        builder.text(f"links: {_NUM_LINKS}")
        builder.text(f"dofs: {robot.num_dofs}")
        builder.text(f"world time: {world.time:.3f} s")
        builder.text(f"time step: {world.time_step:.4f} s")
        builder.text(f"tip height: {tip_height:.3f} m")
        builder.plot_lines("Tip height", list(tip_height_history))
        builder.separator()
        bridge.build_control_panel(builder, context)

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        panels=[ScenePanel("Variational Chain", build_panel)],
        info={"sx_world": world, "dofs": robot.num_dofs},
    )


SCENE = PythonDemoScene(
    id="variational_chain",
    title="Variational Chain",
    category="Variational Integrators",
    summary="A passive chain swings without numerical damping (symplectic VI).",
    build=build,
)
