"""Rigid Body > boxes: a 5x5x5 grid of colored falling boxes."""

from __future__ import annotations

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    index = 0
    for i in range(5):
        for j in range(5):
            for k in range(5):
                box = dart.dynamics.Skeleton(f"box{index}")
                index += 1
                joint, body = box.createFreeJointAndBodyNodePair()
                node = body.createShapeNode(dart.dynamics.BoxShape([0.9, 0.9, 0.9]))
                node.createVisualAspect().setColor([i / 5, j / 5, k / 5])
                node.createCollisionAspect()
                node.createDynamicsAspect().setRestitutionCoeff(0.9)
                transform = dart.math.Isometry3()
                transform.set_translation([i - 2, j - 2, k + 5])
                joint.setTransformFromParentBodyNode(transform)
                world.addSkeleton(box)

    ground = dart.dynamics.Skeleton("ground")
    body = ground.createWeldJointAndBodyNodePair()[1]
    node = body.createShapeNode(dart.dynamics.BoxShape([25, 25, 0.1]))
    node.createVisualAspect().setColor([0.9, 0.9, 0.9])
    node.createCollisionAspect()
    node.createDynamicsAspect().setRestitutionCoeff(0.9)
    world.addSkeleton(ground)

    panel = dart.viewer.Panel()
    count = panel.add_text("")
    panel.add_text(
        "A 5x5x5 grid of boxes free-falls onto the ground plane under "
        "gravity; each box has a restitution coefficient of 0.9.",
        wrap=True,
    )

    def pre_refresh():
        count.label = f"Boxes: {max(world.getNumSkeletons() - 1, 0)}"

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[20, 20, 15], center=[0, 0, 3], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "boxes",
            title="Boxes",
            category="Rigid Body",
            summary="A 5x5x5 grid of colored boxes dropped onto the ground.",
            factory=build,
        )
    ]
