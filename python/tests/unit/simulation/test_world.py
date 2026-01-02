import platform

import dartpy as dart
import pytest


def test_empty_world():
    world = dart.World("my world")
    assert world.get_num_skeletons() == 0
    assert world.get_num_simple_frames() == 0


def test_collision_detector_change():
    world = dart.World("world")
    solver = world.get_constraint_solver()
    assert solver is not None

    assert (
        solver.get_collision_detector().get_type()
        == dart.DARTCollisionDetector().get_static_type()
    )

    solver.set_collision_detector(dart.DARTCollisionDetector())
    assert (
        solver.get_collision_detector().get_type()
        == dart.DARTCollisionDetector().get_static_type()
    )

    solver.set_collision_detector(dart.DARTCollisionDetector())
    assert (
        solver.get_collision_detector().get_type()
        == dart.DARTCollisionDetector().get_static_type()
    )


if __name__ == "__main__":
    pytest.main()
