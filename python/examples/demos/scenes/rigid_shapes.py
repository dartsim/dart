"""Spawn boxes, ellipsoids and cylinders on a loaded ground plane."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from .common import reorient_z_up


def _random_vector(minimum, maximum):
    return np.array([dart.math.Random.uniform(minimum, maximum) for _ in range(3)])


def _spawn(world, shape, name):
    skeleton = dart.dynamics.Skeleton()
    joint, body = skeleton.createFreeJointAndBodyNodePair()
    joint.setName(f"{name}_joint")
    transform = dart.math.Isometry3()
    rotation = _random_vector(-np.pi, np.pi)
    position = [
        dart.math.Random.uniform(-1.0, 1.0),
        dart.math.Random.uniform(-1.0, 1.0),
        dart.math.Random.uniform(0.5, 1.0),
    ]
    transform.set_translation(position)
    transform.set_rotation(dart.math.expMapRot(rotation))
    joint.setTransformFromParentBodyNode(transform)
    body.setName(f"{name}_link")
    body.setMass(10.0)
    node = body.createShapeNode(shape)
    node.createVisualAspect()
    node.createCollisionAspect()
    node.createDynamicsAspect()
    node.getVisualAspect().setColor(_random_vector(0.0, 1.0))
    world.addSkeleton(skeleton)


def build() -> dart.viewer.SceneSetup:
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/shapes.skel")
    if world is None:
        raise RuntimeError("failed to load dart://sample/skel/shapes.skel")
    reorient_z_up(world)
    bullet = getattr(dart.collision, "BulletCollisionDetector", None)
    if bullet is not None:
        world.getConstraintSolver().setCollisionDetector(bullet())

    def spawn_box():
        _spawn(world, dart.dynamics.BoxShape(_random_vector(0.05, 0.25)), "box")

    def spawn_ellipsoid():
        _spawn(
            world,
            dart.dynamics.EllipsoidShape(_random_vector(0.025, 0.125) * 2.0),
            "ellipsoid",
        )

    def spawn_cylinder():
        radius = dart.math.Random.uniform(0.05, 0.25)
        height = dart.math.Random.uniform(0.1, 0.5)
        _spawn(world, dart.dynamics.CylinderShape(radius, height), "cylinder")

    def delete_last():
        if world.getNumSkeletons() > 1:
            world.removeSkeleton(world.getSkeleton(world.getNumSkeletons() - 1))

    panel = dart.viewer.Panel()
    skeletons = panel.add_text(f"Skeletons: {world.getNumSkeletons()}")
    panel.add_text(
        "'q'/'w'/'e' spawn a random box/ellipsoid/cylinder above the "
        "ground; 'a' deletes the most recently spawned one.",
        wrap=True,
    )

    def pre_refresh():
        skeletons.label = f"Skeletons: {world.getNumSkeletons()}"

    return dart.viewer.SceneSetup(
        world,
        pre_refresh=pre_refresh,
        key_actions=[
            dart.viewer.KeyAction("q", "Spawn box", spawn_box),
            dart.viewer.KeyAction("w", "Spawn ellipsoid", spawn_ellipsoid),
            dart.viewer.KeyAction("e", "Spawn cylinder", spawn_cylinder),
            dart.viewer.KeyAction("a", "Delete last skeleton", delete_last),
        ],
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2, -2, 2], center=[0, 0, 0], up=[0, 0, 1]
        ),
    )


def scenes():
    return [
        dart.viewer.Scene(
            "rigid_shapes",
            title="Rigid Shapes",
            category="Rigid Body",
            summary=(
                "Spawn assorted rigid shapes (boxes, ellipsoids, cylinders) onto a "
                "ground plane."
            ),
            factory=build,
        )
    ]
