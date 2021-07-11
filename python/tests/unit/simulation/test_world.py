import platform
import pytest
import dartpy as dart


def test_empty_world():
    world = dart.simulation.World('my world')
    assert world.getNumSkeletons() is 0
    assert world.getNumSimpleFrames() is 0


def test_collision_detector_change():
    world = dart.simulation.World('world')
    solver = world.getConstraintSolver()
    assert solver is not None

    assert solver.getCollisionDetector().getType() == dart.dynamics.FCLCollisionDetector().getStaticType()

    solver.setCollisionDetector(dart.dynamics.DARTCollisionDetector())
    assert solver.getCollisionDetector().getType() == dart.dynamics.DARTCollisionDetector().getStaticType()

    if hasattr(dart.collision, 'BulletCollisionDetector'):
        solver.setCollisionDetector(dart.dynamics.BulletCollisionDetector())
        assert solver.getCollisionDetector().getType() == dart.dynamics.BulletCollisionDetector().getStaticType()

    if hasattr(dart.collision, 'OdeCollisionDetector'):
        solver.setCollisionDetector(dart.dynamics.OdeCollisionDetector())
        assert solver.getCollisionDetector().getType() == dart.dynamics.OdeCollisionDetector().getStaticType()


if __name__ == "__main__":
    pytest.main()
