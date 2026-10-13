"""Getting Started > hello_world: a free box falling onto a ground body."""

from __future__ import annotations

import dartpy as dart


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    box = dart.dynamics.Skeleton("box")
    joint, body = box.createFreeJointAndBodyNodePair()
    transform = dart.math.Isometry3()
    transform.set_translation([0, 0, 1])
    transform.set_rotation(
        dart.math.expMapRot([dart.math.Random.uniform(-1, 1) for _ in range(3)])
    )
    joint.setTransformFromParentBodyNode(transform)

    shape = dart.dynamics.BoxShape([0.3, 0.3, 0.3])
    node = body.createShapeNode(shape)
    node.createVisualAspect().setColor([0.1, 0.1, 0.9])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    body.setInertia(dart.dynamics.Inertia(1.0, [0, 0, 0], shape.computeInertia(1.0)))
    world.addSkeleton(box)

    ground = dart.dynamics.Skeleton("ground")
    ground_body = ground.createWeldJointAndBodyNodePair()[1]
    node = ground_body.createShapeNode(dart.dynamics.BoxShape([10, 10, 0.1]))
    node.createVisualAspect().setColor([0.9, 0.9, 0.9])
    node.createCollisionAspect()
    node.createDynamicsAspect()
    world.addSkeleton(ground)

    panel = dart.viewer.Panel()
    height = panel.add_text("")
    velocity = panel.add_text("")

    def pre_refresh():
        height.label = f"Box height: {body.getWorldTransform().translation()[2]:.2f} m"
        velocity.label = f"Box velocity: {body.getSpatialVelocity()[5]:.2f} m/s"

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2.57, 3.14, 1.64], center=[0, 0, 0.5], up=[-0.24, -0.25, 0.94]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "hello_world",
            title="Hello World",
            category="Getting Started",
            summary="A single free box falling onto a static ground body.",
            factory=build,
        )
    ]
