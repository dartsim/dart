"""Compliant ground contact under the variational integrator.

A two-link revolute pendulum (pivot at the origin, links along ``+X``, swinging
in the X-Z plane about ``+Y``) is released horizontal and falls under gravity
until its tip strikes a ground plane below, where **compliant penalty contact**
-- with Kelvin-Voigt damping and lagged Coulomb friction -- catches it: the tip
settles on the floor instead of swinging through. Contact is configured entirely
through the **World surface** (``Multibody.set_ground_contact`` /
``add_ground_contact_point``), and the World's variational
integrator folds it into the forced discrete-Euler-Lagrange root-find each step
(PLAN-084 Phase C, rungs C1/C2 -- compliant contact + lagged friction).

World owns the physics; WorldRenderBridge mirrors each link's world transform
onto a SimpleFrame box visual so the C++ viewer renders the motion, and a thin
slab marks the ground plane.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_NUM_LINKS = 2
_LINK_LENGTH = 0.3
_LINK_MASS = 0.5
_GROUND_Z = -0.2  # plane below the pivot, within the tip's downswing arc.
# Within the gate-2 k <= 1e4*mg envelope; damping ~critical to settle the impact.
_CONTACT_STIFFNESS = 2.0e3
_CONTACT_DAMPING = 30.0
_CONTACT_FRICTION = 0.6


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
    world.time_step = 0.002

    robot = world.add_multibody("contact_arm")
    base = robot.add_link("base")

    parent = base
    links = []
    for i in range(_NUM_LINKS):
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
        ixx = 0.5 * _LINK_MASS * (0.04**2)
        itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        links.append(link)
        parent = link

    tip = links[-1]

    world.enter_simulation_mode()

    # Configure compliant ground contact through the World surface (the contact
    # reaches the integrator via the variational-integration stage). The contact
    # point is the far end of the last link.
    robot.set_ground_contact(
        plane_normal=np.array([0.0, 0.0, 1.0]),
        plane_point=np.array([0.0, 0.0, _GROUND_Z]),
        stiffness=_CONTACT_STIFFNESS,
        friction_coefficient=_CONTACT_FRICTION,
        damping_coefficient=_CONTACT_DAMPING,
    )
    robot.add_ground_contact_point(tip, np.array([_LINK_LENGTH, 0.0, 0.0]))

    bridge = WorldRenderBridge(world, name="variational_contact_render")
    palette = [(0.90, 0.30, 0.24), (0.20, 0.55, 0.90)]
    for i, link in enumerate(links):
        bridge.add_link_visual(
            link,
            dart.BoxShape(np.array([_LINK_LENGTH, 0.08, 0.08])),
            palette[i % len(palette)],
            name=f"link{i}_visual",
        )
    # A thin static ground slab at the contact plane (a standalone render frame
    # so the bridge's per-step sync never moves it).
    ground = dart.SimpleFrame(
        dart.gui.world_render_frame(), "ground_visual", _translation(0.4, 0.0, _GROUND_Z)
    )
    ground.set_shape(dart.BoxShape(np.array([1.6, 0.8, 0.02])))
    ground.create_visual_aspect().set_color([0.55, 0.55, 0.58])
    bridge.render_world.add_simple_frame(ground)
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        info={
            "sx_world": world,
            "dofs": robot.num_dofs,
            "ground_z": _GROUND_Z,
            "contact_stiffness": _CONTACT_STIFFNESS,
            "tip": tip,
            "tip_local": (_LINK_LENGTH, 0.0, 0.0),
        },
    )


SCENE = PythonDemoScene(
    id="variational_contact",
    title="Variational Contact",
    category="Variational Integrators",
    summary="A VI pendulum tip caught by compliant ground contact + friction.",
    build=build,
)
