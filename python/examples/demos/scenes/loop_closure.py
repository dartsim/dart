"""Loop-closure scene: a holonomic closure held under the variational integrator.

A planar 4-link revolute arm (joints about +Z, links along +X) swings under
gravity on the World with the **variational integrator** selected
(``MultibodyOptions(integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL)``). A holonomic
``DISTANCE`` loop closure ties the end of the last link to a fixed world anchor
(a point carried on the root/base link, which never moves) at the exact
tip-to-anchor separation of the rest pose. With the closure's dynamics policy set
to ``SOLVE``, the VI Newton-projects each step onto the constraint manifold, so
the open chain behaves as a *closed* linkage: the tip is kept on a sphere of the
target radius about the anchor while the mechanism keeps moving. This is the
Phase B2 showcase — a holonomic loop that holds while the body swings.

The ``DISTANCE`` family is a single robust row (separation minus target). A POINT
or RIGID closure on a planar arm is rank-deficient/singular, so DISTANCE is used.

World owns the physics; WorldRenderBridge mirrors each link's world transform
onto a SimpleFrame box visual so the C++ viewer renders the motion, and a thin
marker box shows the fixed anchor.
"""

from __future__ import annotations

import numpy as np

import dartpy as dart
import dartpy as sx

from .._world_bridge import WorldRenderBridge
from ..runner import PythonDemoScene, SceneSetup

_NUM_LINKS = 4
_LINK_LENGTH = 0.5
_LINK_MASS = 0.5
# Fixed world anchor, expressed relative to the (immobile) base link origin.
# Offset in +X and lifted in +Z so the arm hangs and swings as a closed loop.
_ANCHOR = np.array([1.0, 0.0, 1.0])


def _translation(x: float, y: float, z: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, 3] = (x, y, z)
    return transform


def _rest_tip_position() -> np.ndarray:
    """Tip-end world position at the zero joint configuration.

    Joints are all zero at build time, so the kinematics are a pure translation
    along +X: link ``i`` origin sits at ``i * _LINK_LENGTH`` (the first joint is
    at the base origin), and the tip marker is one more link length past the
    last link's origin. Computing this analytically avoids querying post-sim
    state to seed the closure distance.
    """

    return np.array([_NUM_LINKS * _LINK_LENGTH, 0.0, 0.0])


def build() -> SceneSetup:
    world = sx.World()
    # Select the variational integrator before entering simulation mode; the VI
    # is the integrator that Newton-projects onto the loop-closure manifold.
    world.multibody_options = sx.MultibodyOptions(
        integration_family=sx.MultibodyIntegrationFamily.VARIATIONAL
    )
    world.gravity = (0.0, 0.0, -9.81)
    world.time_step = 0.005

    robot = world.add_multibody("loop_arm")
    base = robot.add_link("base")

    parent = base
    links = []
    # Planar arm: revolute joints about +Z, links stepping along +X. Joints at
    # 0 ⇒ released straight along +X; gravity swings it in the X-Z plane.
    for i in range(_NUM_LINKS):
        offset = 0.0 if i == 0 else _LINK_LENGTH
        link = robot.add_link(
            f"link{i}",
            parent=parent,
            joint=sx.JointSpec(
                name=f"joint{i}",
                type=sx.JointType.REVOLUTE,
                axis=(0.0, 0.0, 1.0),
                transform_from_parent=_translation(offset, 0.0, 0.0),
            ),
        )
        link.mass = _LINK_MASS
        # Slender-rod inertia: small about the long (x) axis, larger transverse.
        ixx = 0.5 * _LINK_MASS * (0.05**2)
        itrans = _LINK_MASS * (_LINK_LENGTH**2) / 12.0
        link.inertia = ((ixx, 0.0, 0.0), (0.0, itrans, 0.0), (0.0, 0.0, itrans))
        links.append(link)
        parent = link

    tip = links[-1]

    # DISTANCE closure: tie the tip-end (one link length past the last link's
    # origin) to the fixed anchor carried on the immobile base. The target is
    # the actual rest-pose separation, so the closure residual is ~0 at t=0.
    rest_distance = float(np.linalg.norm(_rest_tip_position() - _ANCHOR))
    closure = world.add_loop_closure(
        "tip_anchor",
        sx.LoopClosureSpec(
            frame_a=tip,
            frame_b=base,
            family=sx.LoopClosureFamily.DISTANCE,
            offset_a=_translation(_LINK_LENGTH, 0.0, 0.0),
            offset_b=_translation(*_ANCHOR),
            distance=rest_distance,
        ),
    )
    # RESIDUAL_ONLY (the default) only reports the violation; SOLVE makes the VI
    # actually enforce it each step, so the loop holds while the arm swings.
    closure.dynamics = sx.ClosureDynamicsPolicy.SOLVE

    world.enter_simulation_mode()

    bridge = WorldRenderBridge(world, name="loop_closure_render")
    palette = [
        (0.90, 0.30, 0.24),
        (0.95, 0.61, 0.16),
        (0.30, 0.69, 0.31),
        (0.20, 0.55, 0.90),
    ]
    for i, link in enumerate(links):
        bridge.add_link_visual(
            link,
            dart.BoxShape(np.array([_LINK_LENGTH, 0.08, 0.08])),
            palette[i % len(palette)],
            name=f"link{i}_visual",
        )
    # A small static marker box at the fixed world anchor. It is a standalone
    # render frame (not bridged to any body) so the bridge's per-step sync
    # never moves it off the anchor.
    anchor_frame = dart.SimpleFrame(
        dart.gui.world_render_frame(), "anchor_visual", _translation(*_ANCHOR)
    )
    anchor_frame.set_shape(dart.BoxShape(np.array([0.12, 0.12, 0.12])))
    anchor_frame.create_visual_aspect().set_color([0.85, 0.85, 0.20])
    bridge.render_world.add_simple_frame(anchor_frame)
    bridge.sync()

    return SceneSetup(
        world=bridge.render_world,
        pre_step=bridge.pre_step,
        force_drag=bridge.force_drag,
        info={
            "sx_world": world,
            "dofs": robot.num_dofs,
            "closure": closure,
            "closure_distance": rest_distance,
        },
    )


SCENE = PythonDemoScene(
    id="loop_closure",
    title="Loop Closure",
    category="Variational Integrators",
    summary="A holonomic distance loop holds under the VI while the arm swings.",
    build=build,
)
