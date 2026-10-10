import dartpy as dart
import numpy as np
import pytest


def test_recorded_config_is_a_copy_and_layout_changes_clear_history():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("body")
    skeleton.createFreeJointAndBodyNodePair()
    world.addSkeleton(skeleton)
    recording = world.getRecording()
    assert recording.getNumFrames() == 0
    assert recording.getNumSkeletons() == 1
    assert recording.getNumDofs(0) == 6
    skeleton.setPositions(np.arange(6, dtype=float) / 10)
    world.bake()
    recorded = recording.getConfig(0, 0)
    np.testing.assert_allclose(recorded, skeleton.getPositions())
    recorded[:] = 100
    assert not np.any(recording.getConfig(0, 0) == 100)
    skeleton.createRevoluteJointAndBodyNodePair(skeleton.getRootBodyNode())
    world.bake()
    assert recording.getNumFrames() == 1
    assert recording.getNumDofs(0) == 7
    other = dart.dynamics.Skeleton("other")
    other.createFreeJointAndBodyNodePair()
    world.addSkeleton(other)
    assert recording.getNumFrames() == 0
    assert recording.getNumSkeletons() == 2
    world.bake()
    recording.clear()
    assert recording.getNumFrames() == 0


def test_recorded_contacts_match_current_collision_result():
    world = dart.simulation.World()
    ground = dart.dynamics.Skeleton("ground")
    floor = ground.createWeldJointAndBodyNodePair()[1]
    shape = floor.createShapeNode(dart.dynamics.BoxShape([10, 10, 0.2]))
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    ball = dart.dynamics.Skeleton("ball")
    body = ball.createFreeJointAndBodyNodePair()[1]
    shape = body.createShapeNode(dart.dynamics.EllipsoidShape([0.5, 0.5, 0.5]))
    shape.createCollisionAspect()
    shape.createDynamicsAspect()
    ball.setPosition(5, 0.34)
    world.addSkeleton(ground)
    world.addSkeleton(ball)
    world.step()
    world.bake()
    result = world.getLastCollisionResult()
    recording = world.getRecording()
    assert result.getNumContacts() > 0
    assert recording.getNumContacts(0) == result.getNumContacts()
    for index in range(result.getNumContacts()):
        contact = result.getContact(index)
        np.testing.assert_allclose(recording.getContactPoint(0, index), contact.point)
        np.testing.assert_allclose(recording.getContactForce(0, index), contact.force)


@pytest.mark.parametrize("index", [-1, 1])
def test_recording_rejects_invalid_indices(index):
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton()
    skeleton.createFreeJointAndBodyNodePair()
    world.addSkeleton(skeleton)
    world.bake()
    recording = world.getRecording()
    for call in (
        lambda: recording.getNumDofs(index),
        lambda: recording.getNumContacts(index),
        lambda: recording.getConfig(index, 0),
        lambda: recording.getConfig(0, index),
        lambda: recording.getContactPoint(index, 0),
        lambda: recording.getContactForce(index, 0),
        lambda: recording.getContactPoint(0, index),
        lambda: recording.getContactForce(0, index),
    ):
        with pytest.raises(IndexError):
            call()


def test_empty_recording_rejects_frame_access():
    recording = dart.simulation.World().getRecording()
    with pytest.raises(IndexError):
        recording.getNumContacts(0)
