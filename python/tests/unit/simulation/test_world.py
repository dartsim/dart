import platform

import dartpy as dart
import pytest


def test_empty_world():
    world = dart.simulation.World("my world")
    assert world.getNumSkeletons() is 0
    assert world.getNumSimpleFrames() is 0


def test_collision_detector_change():
    world = dart.simulation.World("world")
    solver = world.getConstraintSolver()
    assert solver is not None

    assert (
        solver.getCollisionDetector().getType()
        == dart.collision.FCLCollisionDetector().getStaticType()
    )

    solver.setCollisionDetector(dart.collision.DARTCollisionDetector())
    assert (
        solver.getCollisionDetector().getType()
        == dart.collision.DARTCollisionDetector().getStaticType()
    )

    if hasattr(dart.collision, "BulletCollisionDetector"):
        solver.setCollisionDetector(dart.collision.BulletCollisionDetector())
        assert (
            solver.getCollisionDetector().getType()
            == dart.collision.BulletCollisionDetector().getStaticType()
        )

    if hasattr(dart.collision, "OdeCollisionDetector"):
        solver.setCollisionDetector(dart.collision.OdeCollisionDetector())
        assert (
            solver.getCollisionDetector().getType()
            == dart.collision.OdeCollisionDetector().getStaticType()
        )


if __name__ == "__main__":
    pytest.main()
