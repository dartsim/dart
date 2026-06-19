import platform

import dartpy as dart
import pytest


def test_empty_world():
    world = dart.simulation.World("my world")
    assert world.getNumSkeletons() == 0
    assert world.getNumSimpleFrames() == 0


def test_num_threads_binding():
    world = dart.simulation.World("threaded world")
    solver = world.getConstraintSolver()

    assert world.getNumThreads() == 1
    assert solver.getNumThreads() == 1

    world.setNumThreads(4)
    assert world.getNumThreads() == 4
    assert solver.getNumThreads() == 4

    world.setNumThreads(0)
    assert world.getNumThreads() == 1
    assert solver.getNumThreads() == 1

    world.setNumThreads(3)
    clone = world.clone()
    assert clone.getNumThreads() == 3
    assert clone.getConstraintSolver().getNumThreads() == 3


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
