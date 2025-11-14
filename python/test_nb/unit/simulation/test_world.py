import platform

import dartpy_nb as dart
import numpy as np
import pytest


def test_empty_world():
    world = dart.simulation.World("my world")
    assert world.getNumSkeletons() == 0
    assert world.getNumSimpleFrames() == 0


def test_world_name_clone_and_gravity():
    world = dart.simulation.World()
    assert world.getName() == "world"

    world.setName("renamed_world")
    assert world.getName() == "renamed_world"

    clone = world.clone()
    assert clone is not world
    assert clone.getName() == "renamed_world"

    world.setGravity(np.array([0.0, 0.0, -9.81]))
    np.testing.assert_allclose(world.getGravity(), np.array([0.0, 0.0, -9.81]))

    world.setGravity(1.0, 2.0, 3.0)
    np.testing.assert_allclose(world.getGravity(), np.array([1.0, 2.0, 3.0]))


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


def test_skeleton_and_simple_frame_management():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton()
    skeleton.setName("simple_skeleton")

    added_name = world.addSkeleton(skeleton)
    assert added_name == "simple_skeleton"
    assert world.getNumSkeletons() == 1
    assert world.hasSkeleton(skeleton)
    assert world.hasSkeleton("simple_skeleton")
    assert world.getSkeleton(0) is skeleton
    assert world.getSkeleton("simple_skeleton") is skeleton

    frame = dart.dynamics.SimpleFrame()
    frame.setName("frame")
    frame_result = world.addSimpleFrame(frame)
    assert frame_result == "frame"
    assert world.getNumSimpleFrames() == 1
    assert world.getSimpleFrame(0) is frame
    assert world.getSimpleFrame("frame") is frame

    removed_skeletons = world.removeAllSkeletons()
    assert len(removed_skeletons) == 1
    assert skeleton in removed_skeletons
    assert world.getNumSkeletons() == 0

    removed_frames = world.removeAllSimpleFrames()
    assert len(removed_frames) == 1
    assert frame in removed_frames
    assert world.getNumSimpleFrames() == 0


def test_time_and_collision_apis():
    world = dart.simulation.World()
    world.setTimeStep(0.001)
    assert world.getTimeStep() == pytest.approx(0.001)

    world.setTime(1.0)
    assert world.getTime() == pytest.approx(1.0)
    assert world.getSimFrames() == 0

    world.step()
    assert world.getTime() == pytest.approx(1.001)
    assert world.getSimFrames() == 1

    world.step(resetCommand=False)
    assert world.getTime() == pytest.approx(1.002)
    assert world.getSimFrames() == 2

    world.reset()
    assert world.getTime() == pytest.approx(0.0)
    assert world.getSimFrames() == 0

    world.bake()

    result = dart.collision.CollisionResult()
    option = dart.collision.CollisionOption()

    assert isinstance(world.checkCollision(), bool)
    assert isinstance(world.checkCollision(option), bool)
    assert isinstance(world.checkCollision(option, result), bool)

    last_result = world.getLastCollisionResult()
    assert hasattr(last_result, "isCollision")
    assert last_result.isCollision() in (True, False)


if __name__ == "__main__":
    pytest.main()
