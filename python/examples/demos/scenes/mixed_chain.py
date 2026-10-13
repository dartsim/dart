"""Apply timed impulses to the fourth soft body in a mixed chain."""

import dartpy as dart
import numpy as np

from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    uri = "dart://sample/skel/test/test_articulated_bodies_10bodies.skel"
    world = dart.utils.SkelParser.readWorld(uri)
    if world is None:
        raise RuntimeError(f"failed to load {uri}")
    reorient_z_up(world)
    chain = world.getSkeleton(1)
    if chain is not None:
        pose = chain.getPositions().copy()
        for i in range(min(3, chain.getNumDofs())):
            pose[i] = dart.math.Random.uniform(-0.5, 0.5)
        chain.setPositions(pose)

    force = np.zeros(3)
    impulse_duration = 0
    panel = dart.viewer.Panel()
    soft_body_text = panel.add_text("")
    impulse_text = panel.add_text("Impulse remaining: 0 steps")
    panel.add_text(
        "q/w apply -X/+X, e/r apply -Z/+Z, t/y apply +Y/-Y: a 500 N "
        "impulse on the chain's 4th soft body node, applied for 100 steps.",
        wrap=True,
    )

    def apply_impulse(axis, value):
        nonlocal impulse_duration
        force[axis] = value
        impulse_duration = 100

    def pre_step():
        nonlocal impulse_duration
        if impulse_duration <= 0:
            return
        chain = world.getSkeleton(1)
        if chain is not None and chain.getNumSoftBodyNodes() > 3:
            soft_bodies = [
                body
                for body in chain.getBodyNodes()
                if isinstance(body, dart.dynamics.SoftBodyNode)
            ]
            soft_bodies[3].addExtForce(force)
        impulse_duration -= 1
        if impulse_duration <= 0:
            impulse_duration = 0
            force[:] = 0.0

    def pre_refresh():
        chain = world.getSkeleton(1)
        count = chain.getNumSoftBodyNodes() if chain is not None else 0
        soft_body_text.label = f"Soft body nodes: {count}"
        impulse_text.label = f"Impulse remaining: {impulse_duration} steps"

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_step=pre_step,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction(
                "q", "Force -X on soft link", lambda: apply_impulse(0, -500.0)
            ),
            dart.viewer.KeyAction(
                "w", "Force +X on soft link", lambda: apply_impulse(0, 500.0)
            ),
            dart.viewer.KeyAction(
                "e", "Force -Z on soft link", lambda: apply_impulse(2, -500.0)
            ),
            dart.viewer.KeyAction(
                "r", "Force +Z on soft link", lambda: apply_impulse(2, 500.0)
            ),
            dart.viewer.KeyAction(
                "t", "Force +Y on soft link", lambda: apply_impulse(1, 500.0)
            ),
            dart.viewer.KeyAction(
                "y", "Force -Y on soft link", lambda: apply_impulse(1, -500.0)
            ),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[7.5, -7.5, 3.2], center=[0.0, 0.0, 2.2], up=[0.0, 0.0, 1.0]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "mixed_chain",
            title="Mixed Chain",
            category="Soft Bodies",
            summary="Apply impulses to a soft link in a mixed rigid/soft chain.",
            factory=build,
        )
    ]
