"""Exercise the tutorial callbacks without creating an OSG window."""

import gc
import importlib.util
import math
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

pytestmark = pytest.mark.skipif(
    not hasattr(getattr(dart, "gui", None), "osg"),
    reason="the opt-in nanobind binder does not yet include gui.osg",
)


def load_tutorial(name, finished=True):
    filename = "main_finished.py" if finished else "main.py"
    path = Path(__file__).resolve().parents[2] / "tutorials" / name / filename
    spec = importlib.util.spec_from_file_location(f"{name}_{filename[:-3]}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class KeyEvent:
    def __init__(self, key):
        self.key = key

    def getEventType(self):
        return dart.gui.osg.GUIEventAdapter.KEYDOWN

    def getKey(self):
        return ord(self.key)


def step(node):
    node.customPreStep()
    node.world.step()
    node.customPostStep()


def test_pendulum_joint_and_body_force_signs_countdown_and_shapes():
    tutorial = load_tutorial("multi_pendulum")
    node = tutorial.build_scene()
    pendulum = node.controller.pendulum
    assert pendulum.getNumBodyNodes() == 5
    assert pendulum.getNumDofs() == 7
    counts = [pendulum.getBodyNode(i).getNumShapeNodes() for i in range(5)]
    assert counts == [3] * 5
    assert node.handler.handle(KeyEvent("1"), None)
    node.customPreStep()
    assert pendulum.getDof(0).getForce() == tutorial.default_torque
    assert node.controller.force_countdown[0] == tutorial.default_countdown - 1
    node.world.step()
    node.customPostStep()
    assert node.handler.handle(KeyEvent("-"), None)
    node.customPreStep()
    assert pendulum.getDof(0).getForce() == -tutorial.default_torque
    node.world.step()
    node.customPostStep()

    assert node.handler.handle(KeyEvent("f"), None)
    assert node.handler.handle(KeyEvent("1"), None)
    body = pendulum.getBodyNode(0)
    node.customPreStep()
    expected = body.getTransform().rotation() @ [-tutorial.default_force, 0, 0]
    np.testing.assert_allclose(body.getExternalForceGlobal()[3:], expected)
    assert not node.controller.arrow_visuals[0].isHidden()
    node.world.step()
    node.customPostStep()
    for _ in range(tutorial.default_countdown - 1):
        step(node)
    assert node.controller.force_countdown[0] == 0
    node.customPreStep()
    assert node.controller.arrow_visuals[0].isHidden()
    np.testing.assert_allclose(body.getExternalForceGlobal(), np.zeros(6), atol=1e-12)
    assert [pendulum.getBodyNode(i).getNumShapeNodes() for i in range(5)] == counts
    assert np.isfinite(pendulum.getPositions()).all()


def test_pendulum_spring_clamps_and_tip_constraint():
    node = load_tutorial("multi_pendulum").build_scene()
    controller = node.controller
    controller.change_rest_position(10.0)
    assert controller.pendulum.getDof(0).getRestPosition() == 0.0
    assert controller.pendulum.getDof(2).getRestPosition() == 0.0
    assert controller.pendulum.getDof(1).getRestPosition() == math.pi / 2
    controller.change_rest_position(-20.0)
    assert controller.pendulum.getDof(1).getRestPosition() == -math.pi / 2
    controller.change_stiffness(10)
    controller.change_damping(1)
    assert all(dof.getSpringStiffness() == 10 for dof in controller.pendulum.getDofs())
    assert all(
        dof.getDampingCoefficient() == 6 for dof in controller.pendulum.getDofs()
    )
    controller.change_stiffness(-20)
    controller.change_damping(-20)
    assert all(dof.getSpringStiffness() == 0 for dof in controller.pendulum.getDofs())
    assert all(
        dof.getDampingCoefficient() == 0 for dof in controller.pendulum.getDofs()
    )

    solver = node.world.getConstraintSolver()
    count = solver.getNumConstraints()
    assert node.handler.handle(KeyEvent("r"), None)
    assert controller.has_constraint()
    assert solver.getNumConstraints() == count + 1
    tip = controller.pendulum.getBodyNode(4)
    initial_tip = tip.getTransform().multiply([0, 0, 1])
    for _ in range(10):
        step(node)
    np.testing.assert_allclose(
        tip.getTransform().multiply([0, 0, 1]), initial_tip, atol=0.02
    )
    assert node.handler.handle(KeyEvent("r"), None)
    assert not controller.has_constraint()
    assert solver.getNumConstraints() == count


def test_pendulum_replay_callbacks_and_lifetime():
    tutorial = load_tutorial("multi_pendulum")
    node = tutorial.build_scene()
    gc.collect()
    node.handler.toggle_playback()
    assert not node.handler.playing_back
    for _ in range(18):
        step(node)
    recording = node.world.getRecording()
    assert recording.getNumFrames() == 18
    assert node.handler.handle(KeyEvent("p"), None)
    time = node.world.getTime()
    node.customPreRefresh()
    np.testing.assert_allclose(
        node.controller.pendulum.getPositions(), recording.getConfig(0, 0)
    )
    node.customPreRefresh()
    np.testing.assert_allclose(
        node.controller.pendulum.getPositions(), recording.getConfig(16, 0)
    )
    assert node.world.getTime() == time
    assert not node.handler.handle(KeyEvent(" "), None)
    assert not node.handler.playing_back
    node.handler.toggle_playback()
    recording.clear()
    node.customPreRefresh()
    assert not node.handler.playing_back


def test_starter_lesson_omissions_are_explicit():
    pendulum = load_tutorial("multi_pendulum", finished=False).build_scene()
    pendulum.controller.change_stiffness(10.0)
    assert pendulum.controller.pendulum.getDof(0).getSpringStiffness() == 0.0
    biped_node = load_tutorial("biped", finished=False).build_scene()
    biped = biped_node.controller.biped
    assert biped.getBodyNode("main_body") is None
    assert biped.getJoint("joint_front_left") is None
    for _ in range(3):
        step(biped_node)
    assert np.isfinite(biped.getPositions()).all()


def balanced_pose_error(biped):
    heel = biped.getBodyNode("h_heel_left")
    toe = biped.getBodyNode("h_toe_left")
    deviation = biped.getCOM() - heel.getCOM()
    errors = [deviation[0], deviation[2]]
    for body, offset in (
        (heel, [0, -0.04, -0.03]),
        (heel, [0, -0.04, 0.03]),
        (toe, [0.04, -0.04, 0.03]),
        (toe, [0.04, -0.04, -0.03]),
    ):
        errors.append(body.getTransform().multiply(offset)[1] + 0.8)
    return np.linalg.norm(errors)


def test_biped_skateboard_actuators_and_balancing_ik():
    tutorial = load_tutorial("biped")
    biped = tutorial.load_biped()
    tutorial.set_initial_pose(biped)
    original_bodies = biped.getNumBodyNodes()
    tutorial.modify_biped_with_skateboard(biped)
    assert biped.getNumBodyNodes() == original_bodies + 5
    board = biped.getBodyNode("main_body")
    assert board.getParentBodyNode().getName() == "h_heel_left"
    assert board.getParentJoint().getType() == dart.dynamics.EulerJoint.getStaticType()
    np.testing.assert_allclose(
        board.getParentJoint().getTransformFromChildBodyNode().translation(),
        [0, 0.1, 0],
    )
    tutorial.set_velocity_actuators(biped)
    for name in (
        "joint_front_left",
        "joint_front_right",
        "joint_back_left",
        "joint_back_right",
    ):
        assert (
            biped.getJoint(name).getActuatorType()
            == dart.dynamics.ActuatorType.VELOCITY
        )
    biped.getDof("j_shin_right").setPosition(-1.4)
    biped.getDof("j_bicep_left_x").setPosition(0.8)
    biped.getDof("j_bicep_right_x").setPosition(-0.8)
    initial_error = balanced_pose_error(biped)
    pose = tutorial.solve_ik(biped)
    assert np.isfinite(pose).all()
    assert balanced_pose_error(biped) < initial_error * 0.2


def test_biped_pd_spd_and_push_callbacks():
    tutorial = load_tutorial("biped")
    node = tutorial.build_scene()
    gc.collect()
    biped = node.controller.biped
    np.testing.assert_allclose(node.world.getGravity(), [0, -9.81, 0])
    controller = node.controller
    index = biped.getDof("j_shin_right").getIndexInSkeleton()
    biped.setPosition(index, biped.getPosition(index) + 0.01)
    controller.clear_forces()
    controller.add_pd_forces()
    assert biped.getForce(index) == pytest.approx(-10.0)
    np.testing.assert_allclose(biped.getForces()[:6], np.zeros(6))

    assert node.handler.handle(KeyEvent("a"), None)
    assert controller.speed == tutorial.default_speed_increment
    assert node.handler.handle(KeyEvent("."), None)
    node.customPreStep()
    assert np.isfinite(biped.getForces()).all()
    np.testing.assert_allclose(biped.getForces()[:6], np.zeros(6))
    for name in (
        "joint_front_left_2",
        "joint_front_right_2",
        "joint_back_left",
        "joint_back_right",
    ):
        assert biped.getDof(name).getCommand() == tutorial.default_speed_increment
    abdomen = biped.getBodyNode("h_abdomen")
    np.testing.assert_allclose(
        abdomen.getExternalForceGlobal()[3:], [50, 0, 0], atol=1e-12
    )
    assert node.handler.force_countdown == tutorial.default_countdown - 1
    node.world.step()
    assert node.handler.handle(KeyEvent(","), None)
    node.customPreStep()
    np.testing.assert_allclose(
        abdomen.getExternalForceGlobal()[3:], [-50, 0, 0], atol=1e-12
    )
    node.world.step()
    node.handler.force_countdown = 1
    step(node)
    assert node.handler.force_countdown == 0
    node.customPreStep()
    np.testing.assert_allclose(
        abdomen.getExternalForceGlobal(), np.zeros(6), atol=1e-12
    )
    assert node.handler.handle(KeyEvent("s"), None)
    assert controller.speed == 0.0
    for _ in range(5):
        step(node)
    assert np.isfinite(biped.getPositions()).all()
