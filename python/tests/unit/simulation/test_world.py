import platform

import dartpy as dart
import numpy as np
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


def test_world_step_substeps():
    world = dart.World()
    world.set_time_step(0.01)

    world.step_substeps(4)

    assert world.get_sim_frames() == 4
    assert world.get_time() == pytest.approx(0.01)
    assert world.get_time_step() == pytest.approx(0.01)


def test_world_step_unconstrained_runge_kutta4():
    world = dart.World()
    world.set_time_step(0.01)

    world.step_unconstrained_runge_kutta4()

    assert world.get_sim_frames() == 1
    assert world.get_time() == pytest.approx(0.01)
    assert world.get_time_step() == pytest.approx(0.01)


def _create_substep_free_body_world(time_step):
    world = dart.World()
    world.set_gravity(np.zeros(3))
    world.set_time_step(time_step)

    skeleton = dart.Skeleton()
    joint, body = skeleton.create_free_joint_and_body_node_pair()
    assert body is not None

    positions = np.array([0.05, -0.02, 0.03, 0.1, -0.2, 0.3])
    velocities = np.array([0.2, -0.1, 0.15, 0.7, -0.3, 0.4])
    joint.set_positions(positions)
    joint.set_velocities(velocities)

    world.add_skeleton(skeleton)
    return world, skeleton, joint


def test_world_step_substeps_matches_manual_smaller_timestep():
    substeps = 5
    outer_time_step = 0.01
    internal_time_step = outer_time_step / substeps

    substep_world, substep_skeleton, substep_joint = _create_substep_free_body_world(
        outer_time_step
    )
    manual_world, manual_skeleton, manual_joint = _create_substep_free_body_world(
        internal_time_step
    )

    substep_world.step_substeps(substeps)
    for i in range(substeps):
        manual_world.step(i + 1 == substeps)

    assert substep_world.get_sim_frames() == manual_world.get_sim_frames()
    assert substep_world.get_time() == pytest.approx(manual_world.get_time())
    assert substep_world.get_time_step() == pytest.approx(outer_time_step)
    assert np.allclose(
        substep_skeleton.get_positions(), manual_skeleton.get_positions()
    )
    assert np.allclose(substep_joint.get_velocities(), manual_joint.get_velocities())


def test_world_step_substeps_rejects_negative_count():
    world = dart.World()
    world.set_time_step(0.01)

    with pytest.raises(TypeError):
        world.step_substeps(-1)

    assert world.get_sim_frames() == 0
    assert world.get_time() == pytest.approx(0.0)
    assert world.get_time_step() == pytest.approx(0.01)


if __name__ == "__main__":
    pytest.main()
