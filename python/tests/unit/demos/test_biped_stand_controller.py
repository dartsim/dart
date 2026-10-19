"""Python biped control, timestep validation and owner lifetime."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.biped_stand import BipedStandController
from examples.demos.scenes.common import reorient_z_up


def _biped():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    biped = world.getSkeleton("fullbody1")
    for name, position in (
        ("j_pelvis_rot_y", -0.20),
        ("j_thigh_left_z", 0.15),
        ("j_shin_left", -0.40),
        ("j_heel_left_1", 0.25),
        ("j_thigh_right_z", 0.15),
        ("j_shin_right", -0.40),
        ("j_heel_right_1", 0.25),
        ("j_abdomen_2", 0.00),
    ):
        biped.getDof(name).setPosition(position)
    return world, biped


def test_biped_torque_arrays_own_their_storage_and_retain_skeleton():
    world, biped = _biped()
    controller = BipedStandController(biped, world.getTimeStep())
    assert controller.skeleton is biped
    ref = weakref.ref(biped)
    torques = controller.compute_torques()
    assert torques.shape == (biped.getNumDofs(),)
    np.testing.assert_array_equal(torques[:6], 0)
    assert np.isfinite(torques).all() and np.linalg.norm(torques) > 0
    saved = torques.copy()
    del world, biped
    gc.collect()
    assert ref() is not None
    assert np.isfinite(controller.compute_torques()).all()
    del controller
    gc.collect()
    assert ref() is None
    np.testing.assert_array_equal(torques, saved)


def test_biped_uses_settable_current_timestep():
    world, biped = _biped()
    first = BipedStandController(biped, world.getTimeStep())
    second = BipedStandController(biped, world.getTimeStep())
    first.timestep = 0.01
    assert first.timestep == 0.01
    assert np.linalg.norm(first.compute_torques() - second.compute_torques()) > 0


@pytest.mark.parametrize("timestep", (0, -0.1, np.nan, np.inf, -np.inf))
def test_biped_rejects_invalid_timesteps(timestep):
    world, biped = _biped()
    with pytest.raises(ValueError, match="finite and positive"):
        BipedStandController(biped, timestep)
    controller = BipedStandController(biped, world.getTimeStep())
    previous = controller.timestep
    with pytest.raises(ValueError, match="finite and positive"):
        controller.timestep = timestep
    assert controller.timestep == previous


def test_biped_rejects_missing_or_wrong_skeleton():
    with pytest.raises(ValueError, match="must not be None"):
        BipedStandController(None, 0.001)
    with pytest.raises(TypeError, match="must be a Skeleton"):
        BipedStandController(object(), 0.001)
    with pytest.raises(RuntimeError, match="missing DOF"):
        BipedStandController(dart.dynamics.Skeleton(), 0.001)


def test_biped_rejects_changed_skeleton_structure():
    world, biped = _biped()
    controller = BipedStandController(biped, world.getTimeStep())
    biped.createRevoluteJointAndBodyNodePair()
    with pytest.raises(ValueError, match="structure has changed"):
        controller.compute_torques()


def test_biped_owner_cycles_are_collected():
    class Skeleton(dart.dynamics.Skeleton):
        pass

    biped = Skeleton()
    parent = None
    for name in (
        "j_heel_left_1",
        "j_toe_left",
        "j_heel_right_1",
        "j_toe_right",
    ):
        joint, body = biped.createRevoluteJointAndBodyNodePair(parent)
        joint.setDofName(0, name)
        if parent is None:
            body.setName("h_heel_left")
        parent = body
    controller = BipedStandController(biped, 0.001)
    biped.controller = controller
    skeleton_ref, controller_ref = weakref.ref(biped), weakref.ref(controller)
    del parent, body, joint, biped, controller
    gc.collect()
    assert skeleton_ref() is None and controller_ref() is None


@pytest.mark.parametrize("lean, ankle_gain", [(0.5, 10), (-1.0, 100)])
def test_biped_standing_pose_gains_and_ankle_memory(lean, ankle_gain):
    world, biped = _biped()
    reorient_z_up(world)
    biped.getDof("j_pelvis_rot_y").setPosition(lean)
    pose = biped.getPositions().copy()
    controller = BipedStandController(biped, world.getTimeStep())
    np.testing.assert_array_equal(controller._desired_dofs, pose)
    np.testing.assert_array_equal(controller._kp, np.r_[np.zeros(6), np.full(31, 400)])
    np.testing.assert_array_equal(controller._kd, np.r_[np.zeros(6), np.full(31, 40)])
    first = controller.compute_torques()
    heel = biped.getBodyNode("h_heel_left")
    offset = biped.getCOM()[0] - heel.getTransform().multiply([0.05, 0, 0])[0]
    assert (0 < offset < 0.1) if ankle_gain == 10 else (-0.2 < offset < -0.05)
    second = controller.compute_torques()
    expected = np.zeros(biped.getNumDofs())
    for name in ("j_heel_left_1", "j_toe_left", "j_heel_right_1", "j_toe_right"):
        expected[biped.getDof(name).getIndexInSkeleton()] = ankle_gain * offset
    np.testing.assert_allclose(second - first, expected, atol=1e-12, rtol=0)
    np.testing.assert_array_equal(controller._desired_dofs, pose)
    np.testing.assert_array_equal(biped.getForces(), np.zeros(biped.getNumDofs()))


def test_biped_rejects_renamed_heel_without_dof_count_change():
    world, biped = _biped()
    controller = BipedStandController(biped, world.getTimeStep())
    biped.getBodyNode("h_heel_left").setName("changed_heel")
    with pytest.raises(ValueError, match="structure has changed"):
        controller.compute_torques()


def test_biped_repeats_finite_standing_deterministically():
    states = []
    for _ in range(2):
        world, biped = _biped()
        world.setNumSimulationThreads(1)
        reorient_z_up(world)
        controller = BipedStandController(biped, world.getTimeStep())
        for _ in range(200):
            torques = controller.compute_torques()
            assert np.isfinite(torques).all()
            np.testing.assert_array_equal(torques[:6], np.zeros(6))
            biped.setForces(torques)
            world.step()
        states.append(np.r_[biped.getPositions(), biped.getVelocities()])
        assert np.isfinite(states[-1]).all()
    np.testing.assert_array_equal(*states)
