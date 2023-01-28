import platform

import dartpy as dart
import numpy as np
import pytest


def collision_groups_tester(cd):
    size = [1, 1, 1]
    pos1 = [0, 0, 0]
    pos2 = [0.5, 0, 0]

    simple_frame1 = dart.dynamics.SimpleFrame()
    simple_frame2 = dart.dynamics.SimpleFrame()

    sphere1 = dart.dynamics.SphereShape(1)
    sphere2 = dart.dynamics.SphereShape(1)

    simple_frame1.setShape(sphere1)
    simple_frame2.setShape(sphere2)

    group = cd.createCollisionGroup()
    group.addShapeFrame(simple_frame1)
    group.addShapeFrame(simple_frame2)
    assert group.getNumShapeFrames() is 2

    #
    #    ( s1,s2 )              collision!
    # ---+---|---+---+---+---+--->
    #   -1   0  +1  +2  +3  +4
    #
    assert group.collide()

    #
    #    (  s1   )   (  s2   )  no collision
    # ---+---|---+---+---+---+--->
    #   -1   0  +1  +2  +3  +4
    #
    simple_frame2.setTranslation([3, 0, 0])
    assert not group.collide()

    option = dart.collision.CollisionOption()
    result = dart.collision.CollisionResult()

    group.collide(option, result)
    assert not result.isCollision()
    assert result.getNumContacts() is 0

    option.enableContact = True
    simple_frame2.setTranslation([1.99, 0, 0])

    group.collide(option, result)
    assert result.isCollision()
    assert result.getNumContacts() is not 0

    # Repeat the same test with BodyNodes instead of SimpleFrames

    group.removeAllShapeFrames()
    assert group.getNumShapeFrames() is 0

    skel1 = dart.dynamics.Skeleton()
    skel2 = dart.dynamics.Skeleton()

    [joint1, body1] = skel1.createFreeJointAndBodyNodePair(None)
    [joint2, body2] = skel2.createFreeJointAndBodyNodePair(None)

    shape_node1 = body1.createShapeNode(sphere1)
    shape_node1.createVisualAspect()
    shape_node1.createCollisionAspect()

    shape_node2 = body2.createShapeNode(sphere2)
    shape_node2.createVisualAspect()
    shape_node2.createCollisionAspect()

    group.addShapeFramesOf(body1)
    group.addShapeFramesOf(body2)

    assert group.getNumShapeFrames() is 2

    assert group.collide()

    joint2.setPosition(3, 3)
    assert not group.collide()

    # Repeat the same test with BodyNodes and two groups

    joint2.setPosition(3, 0)

    group.removeAllShapeFrames()
    assert group.getNumShapeFrames() is 0
    group2 = cd.createCollisionGroup()

    group.addShapeFramesOf(body1)
    group2.addShapeFramesOf(body2)

    assert group.getNumShapeFrames() is 1
    assert group2.getNumShapeFrames() is 1

    assert group.collide(group2)

    joint2.setPosition(3, 3)
    assert not group.collide(group2)


def test_collision_groups():
    cd = dart.collision.FCLCollisionDetector()
    collision_groups_tester(cd)

    cd = dart.collision.DARTCollisionDetector()
    collision_groups_tester(cd)

    if hasattr(dart.collision, "BulletCollisionDetector"):
        cd = dart.collision.BulletCollisionDetector()
        collision_groups_tester(cd)

    if hasattr(dart.collision, "OdeCollisionDetector"):
        cd = dart.collision.OdeCollisionDetector()
        collision_groups_tester(cd)


# TODO: Add more collision detectors
@pytest.mark.parametrize("cd", [dart.collision.FCLCollisionDetector()])
def test_filter(cd):
    # Create two bodies skeleton. The two bodies are placed at the same position
    # with the same size shape so that they collide by default.
    skel = dart.dynamics.Skeleton()

    shape = dart.dynamics.BoxShape(np.ones(3))

    _, body0 = skel.createRevoluteJointAndBodyNodePair()
    shape_node0 = body0.createShapeNode(shape)
    shape_node0.createVisualAspect()
    shape_node0.createCollisionAspect()

    _, body1 = skel.createRevoluteJointAndBodyNodePair(body0)
    shape_node1 = body1.createShapeNode(shape)
    shape_node1.createVisualAspect()
    shape_node1.createCollisionAspect()

    # Create a world and add the created skeleton
    world = dart.simulation.World()
    world.addSkeleton(skel)

    # Set a new collision detector
    constraint_solver = world.getConstraintSolver()
    constraint_solver.setCollisionDetector(cd)

    # Get the collision group from the constraint solver
    group = constraint_solver.getCollisionGroup()
    assert group.getNumShapeFrames() == 2

    # Create BodyNodeCollisionFilter
    option = constraint_solver.getCollisionOption()
    body_node_filter = dart.collision.BodyNodeCollisionFilter()
    option.collisionFilter = body_node_filter

    skel.enableSelfCollisionCheck()
    skel.enableAdjacentBodyCheck()
    assert skel.isEnabledSelfCollisionCheck()
    assert skel.isEnabledAdjacentBodyCheck()
    assert group.collide()
    assert group.collide(option)

    skel.enableSelfCollisionCheck()
    skel.disableAdjacentBodyCheck()
    assert skel.isEnabledSelfCollisionCheck()
    assert not skel.isEnabledAdjacentBodyCheck()
    assert group.collide()
    assert not group.collide(option)

    skel.disableSelfCollisionCheck()
    skel.enableAdjacentBodyCheck()
    assert not skel.isEnabledSelfCollisionCheck()
    assert skel.isEnabledAdjacentBodyCheck()
    assert group.collide()
    assert not group.collide(option)

    skel.disableSelfCollisionCheck()
    skel.disableAdjacentBodyCheck()
    assert not skel.isEnabledSelfCollisionCheck()
    assert not skel.isEnabledAdjacentBodyCheck()
    assert group.collide()
    assert not group.collide(option)

    # Test collision body filtering
    skel.enableSelfCollisionCheck()
    skel.enableAdjacentBodyCheck()
    body_node_filter.addBodyNodePairToBlackList(body0, body1)
    assert not group.collide(option)
    body_node_filter.removeBodyNodePairFromBlackList(body0, body1)
    assert group.collide(option)
    body_node_filter.addBodyNodePairToBlackList(body0, body1)
    assert not group.collide(option)
    body_node_filter.removeAllBodyNodePairsFromBlackList()
    assert group.collide(option)


def test_raycast():
    cd = dart.collision.BulletCollisionDetector()

    simple_frame = dart.dynamics.SimpleFrame()
    sphere = dart.dynamics.SphereShape(1)
    simple_frame.setShape(sphere)

    group = cd.createCollisionGroup()
    group.addShapeFrame(simple_frame)
    assert group.getNumShapeFrames() == 1

    option = dart.collision.RaycastOption()
    option.mEnableAllHits = False

    result = dart.collision.RaycastResult()
    assert not result.hasHit()

    ray_hit = dart.collision.RayHit()

    result.clear()
    simple_frame.setTranslation(np.zeros(3))
    assert group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert result.hasHit()
    assert len(result.mRayHits) == 1
    ray_hit = result.mRayHits[0]
    assert np.isclose(ray_hit.mPoint, [-1, 0, 0]).all()
    assert np.isclose(ray_hit.mNormal, [-1, 0, 0]).all()
    assert ray_hit.mFraction == pytest.approx(0.25)

    result.clear()
    simple_frame.setTranslation(np.zeros(3))
    assert group.raycast([2, 0, 0], [-2, 0, 0], option, result)
    assert result.hasHit()
    assert len(result.mRayHits) == 1
    ray_hit = result.mRayHits[0]
    assert np.isclose(ray_hit.mPoint, [1, 0, 0]).all()
    assert np.isclose(ray_hit.mNormal, [1, 0, 0]).all()
    assert ray_hit.mFraction == pytest.approx(0.25)

    result.clear()
    simple_frame.setTranslation([1, 0, 0])
    assert group.raycast([-2, 0, 0], [2, 0, 0], option, result)
    assert result.hasHit()
    assert len(result.mRayHits) == 1
    ray_hit = result.mRayHits[0]
    assert np.isclose(ray_hit.mPoint, [0, 0, 0]).all()
    assert np.isclose(ray_hit.mNormal, [-1, 0, 0]).all()
    assert ray_hit.mFraction == pytest.approx(0.5)


if __name__ == "__main__":
    pytest.main()
