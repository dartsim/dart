import platform

import dartpy as dart
import pytest


def test_empty_world():
    world = dart.simulation.World("my world")
    assert world.get_num_skeletons() == 0
    assert world.get_num_simple_frames() == 0


def test_collision_detector_change():
    world = dart.simulation.World("world")
    solver = world.get_constraint_solver()
    assert solver is not None

    assert (
        solver.get_collision_detector().get_type()
        == dart.collision.FCLCollisionDetector().get_static_type()
    )

    solver.set_collision_detector(dart.collision.DARTCollisionDetector())
    assert (
        solver.get_collision_detector().get_type()
        == dart.collision.DARTCollisionDetector().get_static_type()
    )

    if hasattr(dart.collision, "BulletCollisionDetector"):
        solver.set_collision_detector(dart.collision.BulletCollisionDetector())
        assert (
            solver.get_collision_detector().get_type()
            == dart.collision.BulletCollisionDetector().get_static_type()
        )

    if hasattr(dart.collision, "OdeCollisionDetector"):
        solver.set_collision_detector(dart.collision.OdeCollisionDetector())
        assert (
            solver.get_collision_detector().get_type()
            == dart.collision.OdeCollisionDetector().get_static_type()
        )


if __name__ == "__main__":
    pytest.main()
