import platform
import subprocess
import sys
import textwrap

import dartpy as dart
import numpy as np
import pytest


def test_collision_option_allows_negative_penetration_depth_opt_in():
    option = dart.collision.CollisionOption()
    assert not option.allowNegativePenetrationDepthContacts

    option.allowNegativePenetrationDepthContacts = True
    assert option.allowNegativePenetrationDepthContacts

    option = dart.collision.CollisionOption(True, 10, None, True)
    assert option.allowNegativePenetrationDepthContacts


def test_collision_group_add_shape_frames_list():
    detector = dart.collision.DARTCollisionDetector()
    group = detector.createCollisionGroup()
    group.addShapeFrames([])
    assert group.getNumShapeFrames() == 0

    frames = [dart.dynamics.SimpleFrame(), dart.dynamics.SimpleFrame()]
    for frame in frames:
        frame.setShape(dart.dynamics.SphereShape(1))
    group.addShapeFrames(frames)
    assert group.getNumShapeFrames() == len(frames)
    assert all(group.hasShapeFrame(frame) for frame in frames)


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
    assert group.getNumShapeFrames() == 2

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
    assert result.getNumContacts() == 0

    option.enableContact = True
    simple_frame2.setTranslation([1.99, 0, 0])

    group.collide(option, result)
    assert result.isCollision()
    assert result.getNumContacts() != 0

    # Repeat the same test with BodyNodes instead of SimpleFrames

    group.removeAllShapeFrames()
    assert group.getNumShapeFrames() == 0

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

    assert group.getNumShapeFrames() == 2

    assert group.collide()

    joint2.setPosition(3, 3)
    assert not group.collide()

    # Repeat the same test with BodyNodes and two groups

    joint2.setPosition(3, 0)

    group.removeAllShapeFrames()
    assert group.getNumShapeFrames() == 0
    group2 = cd.createCollisionGroup()

    group.addShapeFramesOf(body1)
    group2.addShapeFramesOf(body2)

    assert group.getNumShapeFrames() == 1
    assert group2.getNumShapeFrames() == 1

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


def run_isolated(script):
    # A use-after-free crashes the interpreter, so run each case in a
    # subprocess and require a clean exit.
    result = subprocess.run(
        [sys.executable, "-c", textwrap.dedent(script)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"subprocess exited with {result.returncode}\n"
        f"stdout: {result.stdout[-2000:]}\nstderr: {result.stderr[-2000:]}"
    )
    assert "done" in result.stdout


# Keep the scene owners alive while testing the CollisionResult's storage.
COLLISION_SCENE = """
import gc
import dartpy as dart
import numpy as np

skels = [dart.dynamics.Skeleton("a"), dart.dynamics.Skeleton("b")]
for skel in skels:
    body = skel.createFreeJointAndBodyNodePair()[1]
    shape = body.createShapeNode(dart.dynamics.BoxShape([1, 1, 1]))
    shape.createCollisionAspect()
    del shape
skels[1].getJoint(0).setPosition(3, 0.5)
detector = dart.collision.FCLCollisionDetector()
group = detector.createCollisionGroup()
for skel in skels:
    group.addShapeFramesOf(skel)
option = dart.collision.CollisionOption(True, 100, None)
"""


@pytest.mark.parametrize("accessor", ["getContact", "getContacts"])
@pytest.mark.parametrize("source", ["result", "world"])
def test_contacts_keep_result_alive(accessor, source):
    run_isolated(
        COLLISION_SCENE
        + textwrap.dedent(
            f"""
            if "{source}" == "world":
                world = dart.simulation.World()
                world.setCollisionDetector(detector)
                for skel in skels:
                    world.addSkeleton(skel)
                world.step()
                result = world.getLastCollisionResult()
            else:
                result = dart.collision.CollisionResult()
                assert group.collide(option, result)
            assert result.getNumContacts() > 0
            before = result.getContact(0).point.copy()
            if "{accessor}" == "getContacts":
                contact = result.getContacts()[0]
            else:
                contact = result.getContact(0)
            del result
            gc.collect()
            replacements = []
            for _ in range(200):
                other = dart.collision.CollisionResult()
                assert group.collide(option, other)
                for i in range(other.getNumContacts()):
                    other.getContact(i).point = [91, 92, 93]
                replacements.append(other)
            np.testing.assert_array_equal(contact.point, before)
            print("done")
            """
        )
    )


def test_colliding_shape_frames_survive_exit():
    run_isolated(
        COLLISION_SCENE
        + """
result = dart.collision.CollisionResult()
assert group.collide(option, result)
frames = result.getCollidingShapeFrames()
assert len(frames) == 2
assert {frame.getName() for frame in frames} == {
    skel.getBodyNode(0).getShapeNode(0).getName() for skel in skels
}
gc.collect()
for skel in skels:
    assert skel.getBodyNode(0).getShapeNode(0).getName()
print("done")
"""
    )


@pytest.mark.parametrize("base", ["CompositeCollisionFilter", "BodyNodeCollisionFilter"])
def test_collision_filter_python_override(base):
    run_isolated(
        COLLISION_SCENE
        + textwrap.dedent(
            f"""
            class Filter(dart.collision.{base}):
                def ignoresCollision(self, object1, object2):
                    self.calls += 1
                    assert object1.getShapeFrame().getName()
                    assert object2.getShapeFrame().getName()
                    return True

            collision_filter = Filter()
            collision_filter.calls = 0
            option.collisionFilter = collision_filter
            assert not group.collide(option)
            assert collision_filter.calls > 0
            print("done")
            """
        )
    )


@pytest.mark.parametrize("base", ["CompositeCollisionFilter", "BodyNodeCollisionFilter"])
def test_collision_filter_cpp_fallback(base):
    run_isolated(
        COLLISION_SCENE
        + textwrap.dedent(
            f"""
            class Filter(dart.collision.{base}):
                pass

            collision_filter = Filter()
            option.collisionFilter = collision_filter
            assert group.collide(option)
            if "{base}" == "BodyNodeCollisionFilter":
                collision_filter.addBodyNodePairToBlackList(
                    skels[0].getBodyNode(0), skels[1].getBodyNode(0)
                )
                assert not group.collide(option)
            print("done")
            """
        )
    )


if __name__ == "__main__":
    pytest.main()
