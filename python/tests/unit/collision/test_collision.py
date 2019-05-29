import platform
import pytest
import dartpy as dart
import numpy as np


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
