"""Spawn and delete random dynamic cubes on a ground plane."""

from __future__ import annotations

import dartpy as dart

from .common import reorient_z_up


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/ground.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/ground.skel")
    world.setGravity([0, -9.81, 0])
    reorient_z_up(world)
    bullet = getattr(dart.collision, "BulletCollisionDetector", None)
    if bullet is not None:
        world.getConstraintSolver().setCollisionDetector(bullet())
    spawn_count = 0

    def spawn_cube():
        nonlocal spawn_count
        position = [
            dart.math.Random.uniform(-1.0, 1.0),
            dart.math.Random.uniform(-1.0, 1.0),
            dart.math.Random.uniform(0.5, 1.0),
        ]
        size = [dart.math.Random.uniform(0.1, 0.5) for _ in range(3)]
        shape = dart.dynamics.BoxShape(size)
        skeleton = dart.dynamics.Skeleton(f"cube{spawn_count}")
        spawn_count += 1
        joint, body = skeleton.createFreeJointAndBodyNodePair()
        joint.setName("cube_joint")
        transform = dart.math.Isometry3()
        transform.set_translation(position)
        joint.setTransformFromParentBodyNode(transform)
        body.setName("cube_link")
        inertia = dart.dynamics.Inertia()
        inertia.setMass(0.1)
        inertia.setMoment(shape.computeInertia(0.1))
        body.setInertia(inertia)
        node = body.createShapeNode(shape)
        node.createVisualAspect()
        node.createCollisionAspect()
        node.createDynamicsAspect()
        node.getVisualAspect().setColor(
            [dart.math.Random.uniform(0.0, 1.0) for _ in range(3)]
        )
        world.addSkeleton(skeleton)

    def delete_last():
        if world.getNumSkeletons() > 1:
            world.removeSkeleton(world.getSkeleton(world.getNumSkeletons() - 1))

    panel = dart.viewer.Panel()
    cubes = panel.add_text("Spawned cubes: 0")
    panel.add_text(
        "'q' spawns a random cube above the ground; 'w' deletes the most "
        "recently added skeleton.",
        wrap=True,
    )

    def pre_refresh():
        cubes.label = f"Spawned cubes: {max(0, world.getNumSkeletons() - 1)}"

    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("q", "Spawn cube", spawn_cube),
            dart.viewer.KeyAction("w", "Delete last skeleton", delete_last),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[7, 7, 8], center=[0, 0, 1], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "add_delete_skels",
            title="Add / Delete Skeletons",
            category="Rigid Body",
            summary="Spawn and delete dynamic cubes at runtime.",
            factory=build,
        )
    ]
