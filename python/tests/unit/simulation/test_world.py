import platform

import dartpy as dart
import pytest


def test_empty_world():
    world = dart.simulation.World("my world")
    assert world.getNumSkeletons() == 0
    assert world.getNumSimpleFrames() == 0


def test_num_simulation_threads_binding():
    world = dart.simulation.World("threaded world")
    solver = world.getConstraintSolver()

    assert world.getNumSimulationThreads() == 1
    assert solver.getNumSimulationThreads() == 1

    world.setNumSimulationThreads(4)
    assert world.getNumSimulationThreads() == 4
    assert solver.getNumSimulationThreads() == 4

    world.setNumSimulationThreads(0)
    assert world.getNumSimulationThreads() >= 1
    assert solver.getNumSimulationThreads() == world.getNumSimulationThreads()

    world.setNumSimulationThreads(3)
    clone = world.clone()
    assert clone.getNumSimulationThreads() == 3
    assert clone.getConstraintSolver().getNumSimulationThreads() == 3


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
