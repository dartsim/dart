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
        == dart.FCLCollisionDetector().get_static_type()
    )

    solver.set_collision_detector(dart.DARTCollisionDetector())
    assert (
        solver.get_collision_detector().get_type()
        == dart.DARTCollisionDetector().get_static_type()
    )

    if hasattr(dart, "BulletCollisionDetector"):
        solver.set_collision_detector(dart.BulletCollisionDetector())
        assert (
            solver.get_collision_detector().get_type()
            == dart.BulletCollisionDetector().get_static_type()
        )

    if hasattr(dart, "OdeCollisionDetector"):
        solver.set_collision_detector(dart.OdeCollisionDetector())
        assert (
            solver.get_collision_detector().get_type()
            == dart.OdeCollisionDetector().get_static_type()
        )


def test_contacts_used_for_constraints():
    world = dart.World("world")
    contacts = world.get_contacts_used_for_constraints()
    assert isinstance(contacts, list)


if __name__ == "__main__":
    pytest.main()
