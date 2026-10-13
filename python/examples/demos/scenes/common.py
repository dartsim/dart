"""Shared helpers matching the native demo scenes."""

import dartpy as dart


def reorient_z_up(world):
    """Rotate a Y-up world and its gravity by +90 degrees around X."""
    rotation = dart.math.Isometry3()
    rotation.set_rotation([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    for si in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(si)
        for ji in range(skeleton.getNumJoints()):
            joint = skeleton.getJoint(ji)
            if joint.getParentBodyNode() is None:
                joint.setTransformFromParentBodyNode(
                    rotation.multiply(joint.getTransformFromParentBodyNode())
                )
    world.setGravity(rotation.rotation() @ world.getGravity())
