import platform

import dartpy as dart
import numpy as np
import pytest


def test_basic():
    skel = dart.dynamics.Skeleton()

    joint_prop = dart.dynamics.FreeJointProperties()
    joint_prop.mName = "joint0"
    assert joint_prop.mName == "joint0"

    [joint1, body1] = skel.createFreeJointAndBodyNodePair(None, joint_prop)
    assert joint1.getType() == "FreeJoint"
    assert joint1.getName() == "joint0"

    assert skel.getBodyNodes()[0].getName() == body1.getName()


def test_resting_and_island_index():
    world = dart.simulation.World()
    options = world.getDeactivationOptions()
    options.mEnabled = True
    world.setDeactivationOptions(options)
    world.setGravity([0, 0, 0])

    skel = dart.dynamics.Skeleton()
    skel.createFreeJointAndBodyNodePair()
    world.addSkeleton(skel)

    assert skel.isResting() is False
    assert skel.getIslandIndex() == -1
    assert skel.setResting(resting=True) is None
    assert skel.isResting() is True
    assert skel.setResting(resting=False) is None
    assert skel.isResting() is False

    world.step()
    index = skel.getIslandIndex()
    assert isinstance(index, int)
    assert index == -1 or 0 <= index < world.getNumSkeletons()


if __name__ == "__main__":
    pytest.main()
