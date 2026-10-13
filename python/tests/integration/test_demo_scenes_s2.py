"""Construction and scripted interaction parity for the second demo batch."""

import math
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

SCENE_IDS = (
    "hardcoded_design",
    "rigid_loop",
    "dynamic_joint_constraints",
    "tinkertoy",
    "hybrid_dynamics",
    "mixed_chain",
    "vehicle",
)

# C++ debug-print-world: (name, bodies, soft bodies, point masses, DOFs).
STRUCTURES = {
    "hardcoded_design": [("Skeleton", 3, 0, 0, 3)],
    "rigid_loop": [("skeleton 1", 10, 0, 0, 30)],
    "dynamic_joint_constraints": [
        (name, 1, 0, 0, 6) for name in ("ball_body", "cylindrical_body", "weld_body")
    ],
    "tinkertoy": [("toy_#1", 4, 0, 0, 7), ("toy_#2", 5, 0, 0, 8)],
    "hybrid_dynamics": [("ground skeleton", 1, 0, 0, 6), ("fullbody1", 20, 0, 0, 37)],
    "mixed_chain": [("ground skeleton", 1, 0, 0, 0), ("skeleton 1", 10, 5, 130, 33)],
    "vehicle": [
        ("ground skeleton", 1, 0, 0, 0),
        ("car_skeleton", 5, 0, 0, 12),
        ("skeleton_box1", 1, 0, 0, 0),
        ("skeleton_box2", 1, 0, 0, 0),
    ],
    "atlas_v5": [("drc_skeleton", 36, 0, 0, 29)],
    "unitree_g1": [("g1_29dof_mode_15", 38, 0, 0, 29)],
}


def _structure(world):
    result = []
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        soft = _soft_bodies(skeleton)
        result.append(
            (
                skeleton.getName(),
                skeleton.getNumBodyNodes(),
                len(soft),
                sum(body.getNumPointMasses() for body in soft),
                skeleton.getNumDofs(),
            )
        )
    return result


def _scene(scene_id):
    from examples.demos.registry import make_demo_scenes

    return next(scene for scene in make_demo_scenes() if scene.id == scene_id)


def _step(setup, count=1):
    for _ in range(count):
        if setup.pre_refresh:
            setup.pre_refresh()
        if setup.pre_step:
            setup.pre_step()
        setup.world.step()
        if setup.post_step:
            setup.post_step()


def _state(world):
    values = []
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        values.extend((skeleton.getPositions(), skeleton.getVelocities()))
        for soft in _soft_bodies(skeleton):
            values.extend((soft.getCOM(), soft.getCOMLinearVelocity()))
    return np.concatenate(values).copy()


def _soft_bodies(skeleton):
    return [
        body
        for body in skeleton.getBodyNodes()
        if isinstance(body, dart.dynamics.SoftBodyNode)
    ]


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


@pytest.fixture
def controls(monkeypatch):
    """Observe actual native controls without adding a scene-only test API."""
    recorded = {}
    for method in ("add_text", "add_slider", "add_checkbox", "add_button"):
        original = getattr(viewer.Control, method)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded.setdefault(control.label, control)
            return control

        monkeypatch.setattr(viewer.Control, method, record)
    return recorded


@pytest.mark.parametrize("scene_id", (*SCENE_IDS, "atlas_v5", "unitree_g1"))
def test_s2_construction_and_deterministic_steps(scene_id):
    if scene_id in ("atlas_v5", "unitree_g1") and not hasattr(
        dart.utils, "ModelResourceRetriever"
    ):
        pytest.skip("model resource retriever unavailable")
    states = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        try:
            setup = _scene(scene_id).factory()
        except viewer.AssetsNotPrefetchedError as error:
            pytest.skip(str(error))
        assert _structure(setup.world) == STRUCTURES[scene_id]
        if scene_id in ("atlas_v5", "unitree_g1"):
            assert not setup.world.getSkeleton(0).isMobile()
        expected_constraints = {"rigid_loop": 1, "dynamic_joint_constraints": 3}
        assert (
            setup.world.getConstraintSolver().getNumConstraints()
            == expected_constraints.get(scene_id, 0)
        )
        _step(setup, 4)
        states.append(_state(setup.world))
        assert setup.world.getSimFrames() == 4
    assert np.isfinite(states[0]).all()
    np.testing.assert_array_equal(*states)


def test_hardcoded_design_keys_panel_and_wireframe(controls):
    setup = _scene("hardcoded_design").factory()
    skeleton = setup.world.getSkeleton(0)
    actions = {chr(action.key): action for action in setup.key_actions}
    assert {key: action.label for key, action in actions.items()} == {
        "-": "Invert direction",
        "1": "Move LHY",
        "2": "Move LHR",
        "3": "Move LHP",
    }
    for i, key in enumerate("123"):
        actions[key].callback()
        assert skeleton.getPosition(i) == pytest.approx(0.1)
    _edit(controls["Step size"], 0.25)
    actions["-"].callback()
    for i, key in enumerate("123"):
        actions[key].callback()
        assert skeleton.getPosition(i) == pytest.approx(-0.15)
    setup.pre_refresh()
    assert controls["Direction: positive"].label == "Direction: negative"
    assert "-0.15" in controls["Pose: [0.00, 0.00, 0.00]"].label

    class Context:
        def __init__(self):
            self.viewer = dart.gui.Viewer()
            self.teardowns = []

        def add_teardown(self, callback):
            self.teardowns.append(callback)

    ctx = Context()
    setup.on_activate(ctx)
    assert ctx.viewer.isWireframe()
    for callback in reversed(ctx.teardowns):
        callback()
    assert not ctx.viewer.isWireframe()
    assert not setup.shadows


def test_rigid_loop_damping_panel(controls):
    setup = _scene("rigid_loop").factory()
    chain = setup.world.getSkeleton(0)
    velocities = np.arange(1, chain.getNumDofs() + 1, dtype=float)
    chain.setVelocities(velocities)
    setup.pre_step()
    expected = -0.01 * velocities
    expected[1::3] *= 0.1
    np.testing.assert_allclose(chain.getForces(), expected)
    _edit(controls["Damping"], 0.04)
    _edit(controls["Twist scale"], 0.5)
    setup.pre_step()
    expected = -0.04 * velocities
    expected[1::3] *= 0.5
    np.testing.assert_allclose(chain.getForces(), expected)
    setup.pre_refresh()
    assert controls["Speed: 0.000 rad/s"].label != "Speed: 0.000 rad/s"
    np.testing.assert_array_equal(setup.world.getGravity(), [0, 0, -9.81])
    assert setup.world.getTimeStep() == 0.0005
    assert not setup.key_actions


def test_dynamic_joint_constraints_hold_anchor_axis_and_pose(controls):
    setup = _scene("dynamic_joint_constraints").factory()
    world = setup.world
    ball = world.getSkeleton("ball_body").getBodyNode(0)
    anchor = np.array([-3, 0, 1.8])
    local_anchor = ball.getWorldTransform().inverse().multiply(anchor)
    welded = world.getSkeleton("weld_body").getBodyNode(0)
    weld_pose = welded.getWorldTransform().matrix().copy()
    _step(setup, 20)
    np.testing.assert_allclose(
        ball.getWorldTransform().multiply(local_anchor), anchor, atol=0.002
    )
    cylindrical = world.getSkeleton("cylindrical_body").getBodyNode(0)
    np.testing.assert_allclose(
        cylindrical.getWorldTransform().translation()[:2], [0, 0], atol=1e-8
    )
    np.testing.assert_allclose(
        welded.getWorldTransform().matrix(), weld_pose, atol=0.002
    )
    setup.pre_refresh()
    assert controls["Constraints: 3"].label == "Constraints: 3"
    assert not setup.key_actions

    class Context:
        viewer = dart.gui.Viewer()

        def __init__(self):
            self.attachments = []

        def add_attachment(self, attachment):
            self.attachments.append(attachment)

    ctx = Context()
    setup.on_activate(ctx)
    assert len(ctx.attachments) == 1
    assert isinstance(ctx.attachments[0], dart.gui.DebugOverlay)
    assert ctx.attachments[0].getNumLabels() == 4


def test_hybrid_gait_and_harness(controls):
    setup = _scene("hybrid_dynamics").factory()
    world = setup.world
    robot = world.getSkeleton(1)
    world.setTime(0.3)
    setup.pre_step()
    for name, amplitude, frequency in (
        ("j_scapula_left", 1, 4),
        ("j_scapula_right", -1, 4),
        ("j_forearm_left", 0.8, 4),
        ("j_forearm_right", 0.8, 4),
        ("j_shin_left", 0.1, 2),
        ("j_shin_right", 0.1, 2),
    ):
        index = robot.getJoint(name).getIndexInSkeleton(0)
        assert robot.getCommand(index) == pytest.approx(
            amplitude * math.sin(0.3 * frequency)
        )
    (action,) = setup.key_actions
    assert action.key == ord("h") and action.label == "Toggle harness"
    pelvis = robot.getJoint(0)
    action.callback()
    assert pelvis.getActuatorType() == dart.dynamics.ActuatorType.LOCKED
    setup.pre_refresh()
    assert controls["Harness: unlocked"].label == "Harness: locked"
    action.callback()
    assert pelvis.getActuatorType() == dart.dynamics.ActuatorType.PASSIVE


@pytest.mark.parametrize(
    "key, force",
    [
        ("q", [-500, 0, 0]),
        ("w", [500, 0, 0]),
        ("e", [0, 0, -500]),
        ("r", [0, 0, 500]),
        ("t", [0, 500, 0]),
        ("y", [0, -500, 0]),
    ],
)
def test_mixed_chain_impulses_expire(key, force, controls):
    setup = _scene("mixed_chain").factory()
    actions = {chr(action.key): action for action in setup.key_actions}
    soft = _soft_bodies(setup.world.getSkeleton(1))[3]
    actions[key].callback()
    setup.pre_step()
    np.testing.assert_allclose(soft.getExternalForceGlobal()[3:], force, atol=1e-10)
    for _ in range(99):
        soft.clearExternalForces()
        setup.pre_step()
    soft.clearExternalForces()
    setup.pre_step()
    np.testing.assert_array_equal(soft.getExternalForceGlobal(), np.zeros(6))
    setup.pre_refresh()
    assert controls["Impulse remaining: 0 steps"].label == "Impulse remaining: 0 steps"


def test_mixed_chain_force_axes_compose():
    setup = _scene("mixed_chain").factory()
    actions = {chr(action.key): action for action in setup.key_actions}
    actions["w"].callback()
    actions["t"].callback()
    setup.pre_step()
    soft = _soft_bodies(setup.world.getSkeleton(1))[3]
    np.testing.assert_allclose(
        soft.getExternalForceGlobal()[3:], [500, 500, 0], atol=1e-10
    )


def test_vehicle_keys_force_and_steering_limits(controls):
    setup = _scene("vehicle").factory()
    car = setup.world.getSkeleton("car_skeleton")
    actions = {chr(action.key): action for action in setup.key_actions}
    for key, velocity in (("w", -420), ("s", 0), ("x", 420)):
        actions[key].callback()
        setup.pre_step()
        expected = 0.005 * math.radians(velocity)
        np.testing.assert_allclose(car.getForces()[[7, 9, 10, 11]], expected)
    for key, angle in (("a", 30), ("d", -30)):
        for _ in range(8):
            actions[key].callback()
        setup.pre_step()
        np.testing.assert_allclose(car.getForces()[[6, 8]], 0.01 * math.radians(angle))
        setup.pre_refresh()
        assert f"{angle:.1f}" in controls["Steering angle: 0.0 deg"].label
    np.testing.assert_array_equal(setup.world.getGravity(), [0, -9.81, 0])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 1, 0])


@pytest.mark.parametrize("builder", ["build_atlas", "build_g1"])
def test_humanoid_panel_deferred_pose_and_reset(builder, monkeypatch, controls):
    from examples.demos.scenes import modern_humanoids

    world = dart.simulation.World()
    robot = dart.dynamics.Skeleton("humanoid")
    _, parent = robot.createRevoluteJointAndBodyNodePair()
    robot.createRevoluteJointAndBodyNodePair(parent)
    robot.getDof(1).setPositionLowerLimit(-0.4)
    robot.getDof(1).setPositionUpperLimit(0.7)
    robot.setMobile(False)
    world.addSkeleton(robot)
    monkeypatch.setattr(modern_humanoids, "_world", lambda *_: world)
    setup = getattr(modern_humanoids, builder)()
    assert not setup.key_actions and not setup.allow_simulation
    assert controls["Position (rad)"].min == -math.pi
    assert controls["Position (rad)"].max == math.pi
    _edit(controls["Joint"], 1)
    setup.pre_refresh()
    assert controls["Position (rad)"].min == -0.4
    assert controls["Position (rad)"].max == 0.7
    _edit(controls["Position (rad)"], 0.5)
    assert robot.getPosition(1) == 0
    setup.pre_refresh()
    assert robot.getPosition(1) == 0.5
    _edit(controls["Reset pose"])
    assert robot.getPosition(1) == 0.5
    setup.pre_refresh()
    np.testing.assert_array_equal(robot.getPositions(), [0, 0])
    _step(setup, 3)
    np.testing.assert_array_equal(robot.getPositions(), [0, 0])


class _TinkertoyContext:
    def __init__(self, world):
        self.viewer = dart.gui.Viewer()
        self.viewer.addWorldNode(dart.gui.WorldNode(world))
        self.handlers = []
        self.messages = []
        self.drag = None

    def log(self, message):
        self.messages.append(message)

    def add_event_handler(self, callback):
        class Handler(dart.gui.GUIEventHandler):
            def handle(self, event, _action):
                return callback(event)

        self.handlers.append(callback)
        self.viewer.addEventHandler(Handler())

    def enable_drag_and_drop(self, frame):
        self.drag = self.viewer.enableDragAndDrop(frame)
        return self.drag


def _pick_tinkertoy(ctx, body):
    center = body.getCOM()
    ctx.viewer.setCameraHome(center + [0, 3, 0], center, [0, 0, 1])
    event = dart.gui.GUIEventAdapter
    assert ctx.viewer.handleEvent(event(event.PUSH, x=800, y=500, buttons=1))
    ctx.viewer.handleEvent(event(event.RELEASE, x=800, y=500, buttons=0))


def test_tinkertoy_pick_native_drag_and_spring(controls):
    setup = _scene("tinkertoy").factory()
    world = setup.world
    ctx = _TinkertoyContext(world)
    setup.on_activate(ctx)
    body = world.getSkeleton(0).getBodyNode(0)
    _pick_tinkertoy(ctx, body)
    assert not ctx.drag.isMoving()
    setup.pre_refresh()
    assert controls["Selected: (none)"].label == "Selected: block_#1"
    target = world.getSimpleFrame(1)
    before = target.getWorldTransform().translation().copy()
    aim = target.getWorldTransform().multiply([0.17, 0, 0])
    ctx.viewer.setCameraHome(aim + [0, 3, 0], aim, [0, 0, 1])
    event = dart.gui.GUIEventAdapter
    ctx.viewer.handleEvent(event(event.PUSH, x=800, y=500, buttons=1))
    assert ctx.drag.isMoving()
    ctx.viewer.handleEvent(event(event.DRAG, x=900, y=500, buttons=1))
    assert not np.allclose(target.getWorldTransform().translation(), before)
    ctx.viewer.handleEvent(event(event.RELEASE, x=900, y=500, buttons=0))
    assert not ctx.drag.isMoving()
    _edit(controls["Gravity On/Off"], False)
    ctx.viewer.simulate(True)
    initial = body.getCOM().copy()
    setup.pre_step()
    assert np.linalg.norm(body.getExternalForceGlobal()[3:]) > 0
    _step(setup, 20)
    assert np.linalg.norm(body.getCOM() - initial) > 1e-8
    ctx.viewer.disableDragAndDrop(ctx.drag)


def test_tinkertoy_keys_and_all_panel_controls(controls):
    setup = _scene("tinkertoy").factory()
    world = setup.world
    ctx = _TinkertoyContext(world)
    setup.on_activate(ctx)
    event = dart.gui.GUIEventAdapter
    actions = {action.key: action for action in setup.key_actions}
    for key, kind, dofs in (("1", "Weld", 0), ("2", "Revolute", 1), ("3", "Ball", 3)):
        actions[ord(key)].callback()
        assert (
            world.getSkeleton(world.getNumSkeletons() - 1).getRootJoint().getType()
            == f"{kind}Joint"
        )
        assert world.getSkeleton(world.getNumSkeletons() - 1).getNumDofs() == dofs
    assert world.getNumSkeletons() == 5
    for label, dofs in (("Weld", 0), ("Revolute", 1), ("Ball", 3)):
        _edit(controls[label])
        assert world.getSkeleton(world.getNumSkeletons() - 1).getNumDofs() == dofs
    assert world.getNumSkeletons() == 8
    home_ray = np.array(ctx.viewer.getRayUnderCursor(800, 500))
    ctx.viewer.handleEvent(event(event.SCROLL, scrollY=1))
    assert not np.allclose(ctx.viewer.getRayUnderCursor(800, 500), home_ray)
    actions[int(event.KEY_Tab)].callback()
    np.testing.assert_allclose(ctx.viewer.getRayUnderCursor(800, 500), home_ray)
    actions[int(event.KEY_Up)].callback()
    setup.pre_refresh()
    assert controls["Force Coeff"].value == 110
    actions[int(event.KEY_Down)].callback()
    setup.pre_refresh()
    assert controls["Force Coeff"].value == 100
    _edit(controls["Force Coeff"], 333.25)
    setup.pre_refresh()
    assert controls["Force Coeff"].value == 333.25
    actions[ord("g")].callback()
    np.testing.assert_array_equal(world.getGravity(), [0, 0, 0])
    setup.pre_refresh()
    _edit(controls["Gravity On/Off"], True)
    np.testing.assert_array_equal(world.getGravity(), [0, 0, -9.81])
    target = world.getSimpleFrame(1)
    for control in (None, controls["Reorient Target"]):
        target.setTransform(
            dart.math.Isometry3(dart.math.eulerXYZToMatrix([0.3, 0.2, 0.1]), [1, 2, 3])
        )
        if control is not None:
            _edit(control)
        else:
            actions[ord("`")].callback()
        np.testing.assert_array_equal(target.getWorldTransform().rotation(), np.eye(3))
        np.testing.assert_array_equal(
            target.getWorldTransform().translation(), [1, 2, 3]
        )
    for clear in (
        lambda: actions[int(event.KEY_BackSpace)].callback(),
        lambda: _edit(controls["Reset Target"]),
    ):
        _pick_tinkertoy(ctx, world.getSkeleton(0).getBodyNode(0))
        clear()
        setup.pre_refresh()
        assert controls["Selected: (none)"].label == "Selected: (none)"
        np.testing.assert_array_equal(target.getWorldTransform().matrix(), np.eye(4))
    toy = world.getSkeleton(0)
    parent = toy.getBodyNode(0)
    _pick_tinkertoy(ctx, parent)
    attachment_pose = target.getWorldTransform()
    attachment_pose.set_translation(attachment_pose.translation() + [1, 1, 0])
    target.setTransform(attachment_pose)
    target_pose = target.getWorldTransform().matrix().copy()
    count = toy.getNumBodyNodes()
    actions[ord("1")].callback()
    assert toy.getNumBodyNodes() == count + 1 and world.getNumSkeletons() == 8
    child = toy.getBodyNode(count)
    assert child.getParentBodyNode() is parent
    np.testing.assert_allclose(
        child.getWorldTransform().matrix(), target_pose, atol=1e-12
    )
    for delete in (
        lambda: actions[int(event.KEY_Delete)].callback(),
        lambda: _edit(controls["Delete Block"]),
    ):
        toy = world.getSkeleton(0 if world.getSkeleton(0).getNumBodyNodes() else 1)
        _pick_tinkertoy(ctx, toy.getBodyNode(0))
        ctx.viewer.simulate(True)
        count = toy.getNumBodyNodes()
        delete()
        _edit(controls["Ball"])
        assert toy.getNumBodyNodes() == count and world.getNumSkeletons() == 8
        assert "pause the simulation" in ctx.messages[-1]
        ctx.viewer.simulate(False)
        delete()
        assert toy.getNumBodyNodes() == 0
    ctx.viewer.disableDragAndDrop(ctx.drag)
