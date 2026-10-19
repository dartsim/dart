"""The moved SPD controller preserves gains, ankle state, and owned arrays."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.joint_constraints import JointConstraintsController
from examples.demos.scenes.common import reorient_z_up


def _hinges(count=28):
    skeleton = dart.dynamics.Skeleton("hinges")
    for _ in range(count):
        joint, body = skeleton.createRevoluteJointAndBodyNodePair()
        joint.setAxis([0, 1, 0])
        body.setMomentOfInertia(1, 1, 1)
    skeleton.setGravity([0, 0, 0])
    return skeleton


def test_joint_constraints_original_gains_and_implicit_damping():
    skeleton = _hinges()
    controller = JointConstraintsController(skeleton, 0.001)
    skeleton.setPositions(np.full(28, 0.3))
    skeleton.setVelocities(np.full(28, 0.2))
    torques = []
    for timestep in (0.001, 0.002):
        controller.timestep = timestep
        assert controller.timestep == timestep
        result = controller.compute_torques(skeleton)
        np.testing.assert_array_equal(result[:6], np.zeros(6))
        kp = np.r_[np.full(16, 200), np.full(6, 20)]
        kd = np.r_[np.full(16, 100), np.full(6, 10)]
        expected = (-kp * (0.3 + 0.2 * timestep) - kd * 0.2) / (1 + kd * timestep)
        np.testing.assert_allclose(result[6:], expected, atol=1e-12, rtol=0)
        torques.append(result)
    assert not np.array_equal(*torques)
    np.testing.assert_array_equal(skeleton.getForces(), np.zeros(28))
    saved = torques[-1].copy()
    del skeleton, controller
    gc.collect()
    np.testing.assert_array_equal(torques[-1], saved)


def test_joint_constraints_ankle_strategy_retains_previous_offset():
    skeleton = _hinges()
    skeleton.getBodyNode(0).setName("h_heel_left")
    controller = JointConstraintsController(skeleton, 0.001)
    first = controller.compute_torques(skeleton)
    second = controller.compute_torques(skeleton)
    expected = np.zeros(28)
    expected[[17, 19]] = 6
    expected[[25, 26]] = 5.5
    np.testing.assert_allclose(first, expected, atol=1e-12, rtol=0)
    expected[[17, 19]] = 1
    expected[[25, 26]] = 0.5
    np.testing.assert_allclose(second, expected, atol=1e-12, rtol=0)


@pytest.mark.parametrize("timestep", [0, -0.001, np.nan, np.inf, -np.inf])
def test_joint_constraints_rejects_invalid_timesteps(timestep):
    skeleton = _hinges(5)
    with pytest.raises(ValueError):
        JointConstraintsController(skeleton, timestep)
    controller = JointConstraintsController(skeleton, 0.001)
    with pytest.raises(ValueError):
        controller.timestep = timestep
    assert controller.timestep == 0.001


def test_joint_constraints_rejects_invalid_skeletons():
    for invalid in (None, _hinges(0)):
        with pytest.raises(ValueError):
            JointConstraintsController(invalid, 0.001)
    skeleton = _hinges(5)
    skeleton.getBodyNode(0).setName("h_heel_left")
    controller = JointConstraintsController(skeleton, 0.001)
    np.testing.assert_array_equal(controller.compute_torques(skeleton), np.zeros(5))
    for invalid in (None, _hinges(6)):
        with pytest.raises(ValueError):
            controller.compute_torques(invalid)


def test_joint_constraints_snapshot_does_not_retain_skeleton_wrapper():
    skeleton = _hinges()
    reference = weakref.ref(skeleton)
    controller = JointConstraintsController(skeleton, 0.001)
    del skeleton
    gc.collect()
    assert reference() is None
    assert controller.timestep == 0.001
    np.testing.assert_array_equal(controller.compute_torques(_hinges()), np.zeros(28))


def test_joint_constraints_rejects_wrong_skeleton_types():
    with pytest.raises(TypeError, match="must be a Skeleton"):
        JointConstraintsController(object(), 0.001)
    controller = JointConstraintsController(_hinges(), 0.001)
    with pytest.raises(TypeError, match="must be a Skeleton"):
        controller.compute_torques(object())


def test_joint_constraints_repeats_finite_hinge_world_steps():
    states = []
    for _ in range(2):
        skeleton = _hinges()
        world = dart.simulation.World()
        world.setNumSimulationThreads(1)
        world.setGravity([0, 0, 0])
        world.addSkeleton(skeleton)
        controller = JointConstraintsController(skeleton, world.getTimeStep())
        skeleton.setPositions(np.full(28, 0.3))
        for _ in range(200):
            torques = controller.compute_torques(skeleton)
            assert np.isfinite(torques).all()
            np.testing.assert_array_equal(torques[:6], np.zeros(6))
            skeleton.setForces(torques)
            world.step()
        states.append(np.r_[skeleton.getPositions(), skeleton.getVelocities()])
        assert np.isfinite(states[-1]).all()
        assert skeleton.getPositions()[6] < 0.3
    np.testing.assert_array_equal(*states)


def test_joint_constraints_repeats_finite_fullbody_world_steps():
    states = []
    for _ in range(2):
        world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
        world.setNumSimulationThreads(1)
        reorient_z_up(world)
        biped = world.getSkeleton(1)
        biped.setPositions(
            [1, 4, 6, 9, 10, 13, 16, 17, 21],
            [-0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1],
        )
        controller = JointConstraintsController(biped, world.getTimeStep())
        for _ in range(200):
            torques = controller.compute_torques(biped)
            assert np.isfinite(torques).all()
            np.testing.assert_array_equal(torques[:6], np.zeros(6))
            biped.setForces(torques)
            world.step()
        states.append(np.r_[biped.getPositions(), biped.getVelocities()])
        assert np.isfinite(states[-1]).all()
    np.testing.assert_array_equal(*states)
