"""Native biped control, timestep validation and Python owner lifetime."""

import gc
import weakref

import dartpy as dart
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(getattr(dart, "_demos", None), "BipedStandController"),
    reason="native demo models unavailable",
)


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


def test_biped_native_torque_arrays_own_their_storage_and_retain_skeleton():
    world, biped = _biped()
    controller = dart._demos.BipedStandController(biped, world.getTimeStep())
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


def test_biped_native_uses_settable_current_timestep():
    world, biped = _biped()
    first = dart._demos.BipedStandController(biped, world.getTimeStep())
    second = dart._demos.BipedStandController(biped, world.getTimeStep())
    first.timestep = 0.01
    assert first.timestep == 0.01
    assert np.linalg.norm(first.compute_torques() - second.compute_torques()) > 0


@pytest.mark.parametrize("timestep", (0, -0.1, np.nan, np.inf, -np.inf))
def test_biped_native_rejects_invalid_timesteps(timestep):
    world, biped = _biped()
    with pytest.raises(ValueError, match="finite and positive"):
        dart._demos.BipedStandController(biped, timestep)
    controller = dart._demos.BipedStandController(biped, world.getTimeStep())
    previous = controller.timestep
    with pytest.raises(ValueError, match="finite and positive"):
        controller.timestep = timestep
    assert controller.timestep == previous


def test_biped_native_rejects_missing_or_wrong_skeleton():
    with pytest.raises(ValueError, match="must not be None"):
        dart._demos.BipedStandController(None, 0.001)
    with pytest.raises(TypeError, match="must be a Skeleton"):
        dart._demos.BipedStandController(object(), 0.001)
    with pytest.raises(RuntimeError, match="missing DOF"):
        dart._demos.BipedStandController(dart.dynamics.Skeleton(), 0.001)


def test_biped_native_rejects_changed_skeleton_structure():
    world, biped = _biped()
    controller = dart._demos.BipedStandController(biped, world.getTimeStep())
    biped.createRevoluteJointAndBodyNodePair()
    with pytest.raises(ValueError, match="structure has changed"):
        controller.compute_torques()


def test_biped_native_owner_cycles_are_collected():
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
    controller = dart._demos.BipedStandController(biped, 0.001)
    biped.controller = controller
    skeleton_ref, controller_ref = weakref.ref(biped), weakref.ref(controller)
    del parent, body, joint, biped, controller
    gc.collect()
    assert skeleton_ref() is None and controller_ref() is None
