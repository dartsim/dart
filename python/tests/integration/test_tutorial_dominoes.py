"""Exercise the domino tutorial callbacks without creating a viewer."""

import importlib.util
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(dart.gui, "osg"), reason="DART_BUILD_GUI_OSG is disabled"
)



def load_tutorial(finished=True):
    filename = "main_finished.py" if finished else "main.py"
    path = Path(__file__).resolve().parents[2] / "tutorials" / "dominoes" / filename
    spec = importlib.util.spec_from_file_location("tutorial_dominoes", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def scene():
    tutorial = load_tutorial()
    return tutorial, tutorial.build_scene()


def press(handler, key):
    class KeyEvent:
        def getEventType(self):
            return dart.gui.osg.GUIEventAdapter.KEYDOWN

        def getKey(self):
            return ord(key)

    return handler.handle(KeyEvent(), None)


def step(node, count=1):
    for _ in range(count):
        node.customPreStep()
        node.world.step()
        node.customPostStep()


def test_domino_scene_and_curved_placement(scene):
    tutorial, node = scene
    world = node.world
    assert world.getNumSkeletons() == 3
    domino = world.getSkeleton("domino")
    assert domino.getMass() == pytest.approx(tutorial.default_domino_mass)
    assert domino.getPosition(5) == pytest.approx(tutorial.default_domino_height / 2)
    assert world.getSkeleton("manipulator").getNumDofs() == 6
    handler = node.handler
    assert press(handler, "q")
    assert press(handler, "w")
    assert len(handler.mDominoes) == 2
    np.testing.assert_allclose(
        handler.mDominoes[1].getPositions()[3:5],
        tutorial.default_distance
        * np.array(
            [1 + np.cos(tutorial.default_angle), np.sin(tutorial.default_angle)]
        ),
    )
    assert handler.mDominoes[1].getPosition(2) == pytest.approx(tutorial.default_angle)
    assert press(handler, "d")
    assert len(handler.mDominoes) == 1
    assert handler.mTotalAngle == pytest.approx(tutorial.default_angle)
    assert press(handler, "d")
    assert press(handler, "d")
    assert world.getNumSkeletons() == 3
    assert handler.mTotalAngle == pytest.approx(0)


def test_domino_collision_rejection_and_edit_lock(scene):
    tutorial, node = scene
    handler = node.handler
    obstacle = tutorial.createDomino()
    obstacle.setName("obstacle")
    obstacle.setPosition(3, tutorial.default_distance)
    node.world.addSkeleton(obstacle)
    group = node.world.getConstraintSolver().getCollisionGroup()
    shape_count = group.getNumShapeFrames()
    assert press(handler, "w")
    assert handler.mDominoes == []
    assert node.world.getNumSkeletons() == 4
    assert group.getNumShapeFrames() == shape_count
    node.world.removeSkeleton(obstacle)
    assert press(handler, "w")
    assert len(handler.mDominoes) == 1
    assert press(handler, " ")
    assert handler.mHasEverRun
    assert not press(handler, "q")
    assert not press(handler, "d")
    assert len(handler.mDominoes) == 1


def test_domino_pd_and_push_callbacks(scene):
    tutorial, node = scene
    manipulator = node.controller.mManipulator
    node.customPreStep()
    np.testing.assert_allclose(
        manipulator.getForces(), manipulator.getCoriolisAndGravityForces(), atol=1e-10
    )
    initial = manipulator.getPositions().copy()
    step(node, 20)
    np.testing.assert_allclose(manipulator.getPositions(), initial, atol=1e-5)
    assert node.world.getRecording().getNumFrames() == 20

    press(node.handler, " ")
    press(node.handler, "f")
    node.customPreStep()
    body = node.handler.mFirstDomino.getBodyNode(0)
    wrench = body.getExternalForceGlobal()
    assert wrench[3] == pytest.approx(tutorial.default_push_force)
    force_point = body.getWorldTransform().multiply(
        [0, 0, tutorial.default_domino_height / 2]
    )
    np.testing.assert_allclose(
        wrench[:3],
        np.cross(force_point, [tutorial.default_push_force, 0, 0]),
        atol=1e-10,
    )
    assert node.handler.mForceCountDown == tutorial.default_force_duration - 1
    node.world.step()
    node.customPostStep()
    step(node, tutorial.default_force_duration - 1)
    assert node.handler.mForceCountDown == 0
    assert node.handler.mFirstDomino.getPosition(3) > 0
    node.customPreStep()
    np.testing.assert_allclose(
        node.handler.mFirstDomino.getExternalForces(), np.zeros(6), atol=1e-12
    )

    press(node.handler, "r")
    node.customPreStep()
    assert node.handler.mPushCountDown == tutorial.default_push_duration - 1
    assert np.isfinite(manipulator.getForces()).all()
    np.testing.assert_allclose(manipulator.getForces(), node.controller.mForces)
    assert (
        np.linalg.norm(
            manipulator.getForces() - manipulator.getCoriolisAndGravityForces()
        )
        > 0
    )
    node.world.step()
    node.customPostStep()
    step(node, 10)
    assert np.isfinite(manipulator.getPositions()).all()


def test_domino_replay_while_paused_and_topology_change(scene):
    tutorial, node = scene
    handler = node.handler
    assert press(handler, "p")
    assert not handler.mPlayingBack
    step(node, 20)
    recording = node.world.getRecording()
    expected = recording.getConfig(0, 0).copy()
    handler.mFirstDomino.setPosition(3, 2)
    assert press(handler, "p")
    assert handler.mPlayingBack
    node.simulate(False)
    node.customPreRefresh()
    np.testing.assert_allclose(handler.mFirstDomino.getPositions(), expected)
    assert handler.mPlayFrame == tutorial.default_playback_frame_step
    node.customPreRefresh()
    assert handler.mPlayFrame == 2 * tutorial.default_playback_frame_step
    node.customPreRefresh()
    assert handler.mPlayFrame == tutorial.default_playback_frame_step
    press(handler, " ")
    assert not handler.mPlayingBack
    handler.togglePlayback()
    node.world.addSkeleton(tutorial.createDomino())
    node.customPreRefresh()
    assert not handler.mPlayingBack


def test_domino_live_and_recorded_contact_arrows(scene):
    _, node = scene
    handler = node.handler
    assert press(handler, "v")
    step(node, 20)
    result = node.world.getLastCollisionResult()
    count = result.getNumContacts()
    assert count > 0
    assert any(np.linalg.norm(result.getContact(i).force) > 1e-8 for i in range(count))
    assert any(
        not frame.getVisualAspect().isHidden() for frame in handler.mContactForceFrames
    )
    handler.ensureContactForceVisuals(count + 2)
    handler.setContactForceVisual(count, np.zeros(3), np.ones(3))
    handler.updateContactForces()
    assert all(
        frame.getVisualAspect().isHidden()
        for frame in handler.mContactForceFrames[count:]
    )
    persistent_count = node.world.getNumSimpleFrames()
    assert press(handler, "p")
    handler.mPlayFrame = 19
    node.customPreRefresh()
    recording = node.world.getRecording()
    recorded_count = recording.getNumContacts(19)
    assert recorded_count > 0
    for i in range(recorded_count):
        force = recording.getContactForce(19, i)
        if np.linalg.norm(force) > 1e-7:
            assert not handler.mContactForceFrames[i].getVisualAspect().isHidden()
            np.testing.assert_allclose(
                handler.mContactForceArrows[i].getTail(),
                recording.getContactPoint(19, i),
            )
    assert press(handler, "v")
    assert all(
        frame.getVisualAspect().isHidden() for frame in handler.mContactForceFrames
    )
    assert node.world.getNumSimpleFrames() == persistent_count


def test_domino_starter_reports_missing_lessons():
    tutorial = load_tutorial(finished=False)
    with pytest.raises(NotImplementedError, match="Lesson 2a"):
        tutorial.build_scene()
    controller = tutorial.Controller(None, None)
    with pytest.raises(NotImplementedError, match="Lessons 2b"):
        controller.setPDForces()
    with pytest.raises(NotImplementedError, match="Lessons 3a"):
        controller.setOperationalSpaceForces()
    world = dart.simulation.World()
    world.addSkeleton(tutorial.createDomino())
    world.addSkeleton(tutorial.createFloor())
    handler = tutorial.DominoEventHandler(world, controller)
    with pytest.raises(NotImplementedError, match="Lessons 1a"):
        press(handler, "w")
    with pytest.raises(NotImplementedError, match="Lesson 1c"):
        press(handler, "d")
    press(handler, " ")
    press(handler, "f")
    with pytest.raises(NotImplementedError, match="Lesson 1d"):
        handler.update()
