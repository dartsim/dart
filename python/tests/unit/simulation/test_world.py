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


def test_lcp_solver_type_enum():
    assert hasattr(dart, "LcpSolverType")
    assert dart.LcpSolverType.Dantzig is not None
    assert dart.LcpSolverType.Pgs is not None
    assert dart.LcpSolverType.Lemke is not None


def test_collision_detector_type_enum():
    assert hasattr(dart, "CollisionDetectorType")
    assert dart.CollisionDetectorType.Fcl is not None
    assert dart.CollisionDetectorType.Dart is not None
    assert dart.CollisionDetectorType.Bullet is not None
    assert dart.CollisionDetectorType.Ode is not None


def test_world_config():
    config = dart.WorldConfig()
    assert config.name == "world"
    assert config.collision_detector == dart.CollisionDetectorType.Fcl
    assert config.primary_lcp_solver == dart.LcpSolverType.Dantzig
    assert config.secondary_lcp_solver == dart.LcpSolverType.Pgs

    config.name = "my_world"
    config.primary_lcp_solver = dart.LcpSolverType.Pgs
    config.clear_secondary_lcp_solver()

    assert config.name == "my_world"
    assert config.primary_lcp_solver == dart.LcpSolverType.Pgs
    assert config.secondary_lcp_solver is None

    config.set_secondary_lcp_solver(dart.LcpSolverType.Lemke)
    assert config.secondary_lcp_solver == dart.LcpSolverType.Lemke


def test_world_create_with_config():
    config = dart.WorldConfig("config_world")
    config.primary_lcp_solver = dart.LcpSolverType.Pgs

    world = dart.World(config)
    assert world.get_name() == "config_world"


def test_world_create_factory():
    world = dart.World.create("factory_world")
    assert world.get_name() == "factory_world"

    config = dart.WorldConfig("config_factory")
    world2 = dart.World.create(config)
    assert world2.get_name() == "config_factory"


if __name__ == "__main__":
    pytest.main()
