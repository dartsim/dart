import platform
import pytest
import dartpy as dart


def test_create_collision_groups():
    size = [1, 1, 1]
    pos1 = [0, 0, 0]
    pos2 = [0.5, 0, 0]

    simple_frame1 = dart.dynamics.SimpleFrame()
    simple_frame2 = dart.dynamics.SimpleFrame()

    sphere1 = dart.dynamics.SphereShape(1)
    sphere2 = dart.dynamics.SphereShape(1)

    simple_frame1.setShape(sphere1)
    simple_frame2.setShape(sphere2)

    cd = dart.collision.FCLCollisionDetector.create()
    group = cd.createCollisionGroupAsSharedPtr()
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


if __name__ == "__main__":
    pytest.main()
