import platform

import dartpy as dart
import numpy as np
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



def _state_world(n=1):
    world = dart.simulation.World()
    for i in range(n):
        skel = dart.dynamics.Skeleton(f"s{i}")
        skel.createFreeJointAndBodyNodePair()
        world.addSkeleton(skel)
    return world


def test_capture_step_restore_round_trip():
    world = _state_world()
    world.getSkeleton(0).setVelocities(np.arange(6, dtype=float) / 10)
    state = world.capture_state()
    for _ in range(5):
        world.step()
    assert not np.array_equal(world.getSkeleton(0).getPositions(), state.positions)
    world.restore_state(state)
    skel = world.getSkeleton(0)
    np.testing.assert_array_equal(skel.getPositions(), state.positions)
    np.testing.assert_array_equal(skel.getVelocities(), state.velocities)
    assert world.getTime() == state.time == 0.0


def test_capture_state_out_returns_same_object():
    world = _state_world()
    state = world.capture_state()
    world.step()
    assert world.capture_state(out=state) is state
    assert state.time == world.getTime() > 0.0


def test_capture_state_out_rejects_other_types():
    world = dart.simulation.World()
    with pytest.raises(TypeError):
        world.capture_state(out=np.zeros(3))


def test_world_state_arrays_are_read_only_copies():
    world = _state_world(2)
    state = world.capture_state()
    assert state.body_external_forces.shape == (2, 6)
    for name in (
        "positions",
        "velocities",
        "accelerations",
        "forces",
        "commands",
        "body_external_forces",
    ):
        arr = getattr(state, name)
        assert not arr.flags.writeable
        with pytest.raises(ValueError):
            arr[...] = 1.0
    assert state.positions.shape == (12,)


def test_restore_state_rejects_different_structure():
    state = _state_world(1).capture_state()
    with pytest.raises(ValueError):
        _state_world(2).restore_state(state)


def test_world_state_repr():
    state = _state_world().capture_state()
    assert repr(state) == "WorldState(time=0.0, num_dofs=6)"


def test_world_state_body_external_forces_rows():
    world = _state_world(2)
    forces = [np.array([1.0, 2.0, 3.0]), np.array([4.0, 5.0, 6.0])]
    for i, force in enumerate(forces):
        body = world.getSkeleton(i).getBodyNode(0)
        body.addExtForce(force, np.zeros(3), True)
    state = world.capture_state()
    expected = np.array([np.r_[np.zeros(3), force] for force in forces])
    np.testing.assert_array_equal(state.body_external_forces, expected)
    world.step()
    assert not world.capture_state().body_external_forces.any()
    world.restore_state(state)
    np.testing.assert_array_equal(
        world.capture_state().body_external_forces, expected
    )


def test_capture_state_out_from_another_structure_is_unchanged():
    state = _state_world(1).capture_state()
    positions = state.positions
    with pytest.raises(ValueError):
        _state_world(2).capture_state(out=state)
    np.testing.assert_array_equal(state.positions, positions)
    assert state.positions.shape == (6,)


def test_world_state_arrays_do_not_follow_refills():
    world = _state_world()
    state = world.capture_state()
    before = state.positions
    world.getSkeleton(0).setPositions(np.ones(6))
    world.capture_state(out=state)
    assert not before.any()
    np.testing.assert_array_equal(state.positions, np.ones(6))


if __name__ == "__main__":
    pytest.main()
