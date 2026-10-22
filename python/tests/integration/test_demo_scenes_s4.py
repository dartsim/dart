"""Construction and scripted interaction parity for the fourth demo batch."""

import os
import subprocess
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

# Scene-port parity baseline: (name, bodies, soft bodies, point masses, DOFs).
STRUCTURES = {
    "biped_stand": [("ground skeleton", 1, 0, 0, 6), ("fullbody1", 20, 0, 0, 37)],
    "joint_constraints": [
        ("ground skeleton", 1, 0, 0, 6),
        ("fullbody1", 20, 0, 0, 37),
    ],
    "operational_space_control": [
        ("KR5sixxR650WP_description", 7, 0, 0, 6),
        ("ground_skeleton", 1, 0, 0, 0),
    ],
    "point_cloud": [("KR5", 7, 0, 0, 6), ("ground_skeleton", 1, 0, 0, 0)],
}
METADATA = {
    "biped_stand": (
        "Biped Stand",
        "Control & IK",
        "SPD-balanced standing biped with an ankle strategy and push tests.",
    ),
    "joint_constraints": (
        "Joint Constraints",
        "Control & IK",
        "Balanced full-body PD control with a harness constraint.",
    ),
    "operational_space_control": (
        "Operational Space Control",
        "Control & IK",
        "KR5 arm chasing a draggable target with task-space control.",
    ),
    "point_cloud": (
        "Point Cloud",
        "Visualization",
        "Animated point-cloud and voxel-grid sensor rendering on a robot.",
    ),
}


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


def _structure(world):
    result = []
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        soft = [
            body
            for body in skeleton.getBodyNodes()
            if isinstance(body, dart.dynamics.SoftBodyNode)
        ]
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


def _state(world):
    values = [np.array([world.getTime()])]
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        values.extend((skeleton.getPositions(), skeleton.getVelocities()))
    for i in range(world.getNumSimpleFrames()):
        frame = world.getSimpleFrame(i)
        values.append(frame.getWorldTransform().matrix().ravel())
        if isinstance(frame.getShape(), dart.dynamics.PointCloudShape):
            values.extend(frame.getShape().getPoints())
            values.extend(frame.getShape().getColors())
    return np.concatenate(values).copy()


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


@pytest.fixture
def controls(monkeypatch):
    recorded = {}
    for method in (
        "add_text",
        "add_button",
        "add_checkbox",
        "add_slider",
        "add_input",
        "add_vec3",
        "add_choice",
        "add_color",
        "add_swatch",
    ):
        original = getattr(viewer.Control, method)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded[control.label] = control
            return control

        monkeypatch.setattr(viewer.Control, method, record)
    return recorded


@pytest.mark.parametrize("scene_id", STRUCTURES)
def test_s4_construction_and_steps(scene_id):
    if scene_id == "point_cloud" and not hasattr(dart.dynamics, "VoxelGridShape"):
        pytest.skip("OctoMap unavailable")
    scene = _scene(scene_id)
    assert (scene.title, scene.category, scene.summary) == METADATA[scene_id]
    states = []
    for _ in range(2 if scene_id != "point_cloud" else 1):
        dart.math.Random.setSeed(42)
        setup = scene.factory()
        world = setup.world
        assert _structure(world) == STRUCTURES[scene_id]
        assert (
            world.getNumSimpleFrames()
            == {
                "biped_stand": 0,
                "joint_constraints": 0,
                "operational_space_control": 1,
                "point_cloud": 3,
            }[scene_id]
        )
        assert world.getConstraintSolver().getNumConstraints() == 0
        np.testing.assert_array_equal(
            world.getGravity(), [0, 0, 0 if scene_id == "point_cloud" else -9.81]
        )
        assert world.getTimeStep() == 0.001
        assert setup.shadows is (scene_id != "operational_space_control")
        assert setup.allow_simulation
        _step(setup, 4)
        assert world.getSimFrames() == 4
        states.append(_state(world))
    assert np.isfinite(states[0]).all()
    if len(states) == 2:
        np.testing.assert_array_equal(*states)


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="OctoMap unavailable"
)
def test_point_cloud_seeded_steps_are_deterministic(tmp_path):
    # Eigen::Random uses std::rand; fresh processes reset it alongside DART's RNG.
    script = """
import sys
import dartpy as dart
import numpy as np
from examples.demos.scenes import point_cloud
dart.math.Random.setSeed(42)
setup = point_cloud.build()
for _ in range(4):
    if setup.pre_refresh:
        setup.pre_refresh()
    setup.pre_step()
    setup.world.step()
    if setup.post_step:
        setup.post_step()
world = setup.world
shape = world.getSimpleFrame("point cloud").getShape()
np.savez(sys.argv[1],
         positions=world.getSkeleton("KR5").getPositions(),
         velocities=world.getSkeleton("KR5").getVelocities(),
         points=np.asarray(shape.getPoints()),
         colors=np.asarray(shape.getColors()),
         sensor=world.getSimpleFrame("sensor").getWorldTransform().translation())
"""
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(str(path) for path in sys.path)
    files = [tmp_path / f"run{i}.npz" for i in range(2)]
    for path in files:
        subprocess.run([sys.executable, "-c", script, str(path)], env=env, check=True)
    with np.load(files[0]) as first, np.load(files[1]) as second:
        assert set(first.files) == set(second.files)
        for key in first.files:
            np.testing.assert_array_equal(first[key], second[key])


def test_s4_registration_keeps_cpp_catalog_positions():
    from examples.demos.registry import make_demo_scenes

    ids = [scene.id for scene in make_demo_scenes()]
    for previous, scene_id, following in (
        ("tinkertoy", "joint_constraints", "hybrid_dynamics"),
        ("hybrid_dynamics", "biped_stand", "operational_space_control"),
        ("biped_stand", "operational_space_control", "atlas_puppet"),
    ):
        index = ids.index(scene_id)
        assert ids.count(scene_id) == 1
        assert ids.index(previous) < index < ids.index(following)
    assert ("point_cloud" in ids) is hasattr(dart.dynamics, "VoxelGridShape")
    if "point_cloud" in ids:
        assert (
            ids.index("heightmap")
            < ids.index("point_cloud")
            < ids.index("drag_and_drop")
        )


def test_biped_stand_initial_pose_camera_and_panel(controls):
    setup = _scene("biped_stand").factory()
    assert setup.on_activate is None
    assert setup.post_step is None
    assert setup.allow_simulation is True and setup.shadows is True
    assert setup.world.getNumSimpleFrames() == 0
    np.testing.assert_array_equal(setup.world.getGravity(), [0, 0, -9.81])
    np.testing.assert_array_equal(setup.camera_home.eye, [3, -3, 1.5])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 0])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 0, 1])
    biped = setup.world.getSkeleton("fullbody1")
    for name, value in (
        ("j_pelvis_rot_y", -0.20),
        ("j_thigh_left_z", 0.15),
        ("j_shin_left", -0.40),
        ("j_heel_left_1", 0.25),
        ("j_thigh_right_z", 0.15),
        ("j_shin_right", -0.40),
        ("j_heel_right_1", 0.25),
        ("j_abdomen_2", 0.00),
    ):
        assert biped.getDof(name).getPosition() == value
    assert len(controls) == 2 and all(
        control.on_change is None for control in controls.values()
    )
    assert controls["Perturbation active: 0 steps left"].label == (
        "Perturbation active: 0 steps left"
    )


@pytest.mark.parametrize(
    "key,label,force",
    [
        ("1", "Push forward (+x, 50 N)", [50, 0, 0]),
        ("2", "Push backward (-x, 50 N)", [-50, 0, 0]),
        ("3", "Push right (-y, 50 N)", [0, -50, 0]),
        ("4", "Push left (+y, 50 N)", [0, 50, 0]),
    ],
)
def test_biped_stand_all_pushes_last_exactly_100_steps(controls, key, label, force):
    setup = _scene("biped_stand").factory()
    actions = {chr(action.key): action for action in setup.key_actions}
    assert list(actions) == ["1", "2", "3", "4"]
    assert actions[key].label == label
    active = controls["Perturbation active: 0 steps left"]
    biped = setup.world.getSkeleton("fullbody1")
    spine = biped.getBodyNode("h_spine")
    actions[key].callback()
    setup.pre_refresh()
    assert active.label == "Perturbation active: 100 steps left"
    setup.pre_step()
    np.testing.assert_allclose(spine.getExternalForceGlobal()[3:], force, atol=1e-12)
    np.testing.assert_array_equal(biped.getForces()[:6], 0)
    setup.world.step()
    _step(setup, 98)
    setup.pre_refresh()
    assert active.label == "Perturbation active: 1 steps left"
    setup.pre_step()
    np.testing.assert_allclose(spine.getExternalForceGlobal()[3:], force, atol=1e-12)
    setup.world.step()
    setup.pre_refresh()
    assert active.label == "Perturbation active: 0 steps left"
    setup.pre_step()
    np.testing.assert_array_equal(spine.getExternalForceGlobal()[3:], 0)


def test_biped_stand_adapter_reads_toolbar_timestep():
    from examples.demos.models.biped_stand import BipedStandController as Model
    from examples.demos.scenes.biped_stand import BipedStandController

    setup = _scene("biped_stand").factory()
    biped = setup.world.getSkeleton("fullbody1")
    controller = BipedStandController(setup.world, biped)
    setup.world.setTimeStep(0.01)
    controller.pre_step()
    assert controller.controller.timestep == 0.01
    expected = Model(biped, 0.01).compute_torques()
    np.testing.assert_array_equal(biped.getForces(), expected)


def test_joint_constraints_initial_pose_camera_and_panel(controls):
    setup = _scene("joint_constraints").factory()
    biped = setup.world.getSkeleton(1)
    np.testing.assert_array_equal(
        biped.getPositions()[[1, 4, 6, 9, 10, 13, 16, 17, 21]],
        [-0.1, 0.2, 0.2, -0.5, 0.3, 0.2, -0.5, 0.3, -0.1],
    )
    assert setup.on_activate is None and setup.post_step is None
    assert not any(control.on_change for control in controls.values())
    assert set(controls) == {
        "Harness: off",
        "Perturbation active: 0 steps left",
        "1/2 push the spine forward/backward, 3/4 push right/left "
        "(40-50 N for 100 steps); 'h' toggles a pelvis weld-joint harness.",
    }
    np.testing.assert_array_equal(setup.camera_home.eye, [5, -3, 3])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 0])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 0, 1])


@pytest.mark.parametrize(
    "key,force",
    [("1", [40, 0, 0]), ("2", [-40, 0, 0]), ("3", [0, -50, 0]), ("4", [0, 50, 0])],
)
def test_joint_constraints_push_keys_and_countdown(key, force, controls):
    setup = _scene("joint_constraints").factory()
    actions = {chr(action.key): action for action in setup.key_actions}
    assert {key: action.label for key, action in actions.items()} == {
        "1": "Push forward",
        "2": "Push backward",
        "3": "Push right",
        "4": "Push left",
        "h": "Toggle harness",
    }
    spine = setup.world.getSkeleton(1).getBodyNode("h_spine")
    actions[key].callback()
    setup.pre_refresh()
    assert controls["Perturbation active: 0 steps left"].label == (
        "Perturbation active: 100 steps left"
    )
    for _ in range(100):
        spine.clearExternalForces()
        setup.pre_step()
        np.testing.assert_allclose(
            spine.getExternalForceGlobal()[3:], force, atol=1e-12
        )
    setup.pre_refresh()
    assert controls["Perturbation active: 0 steps left"].label == (
        "Perturbation active: 0 steps left"
    )
    spine.clearExternalForces()
    setup.pre_step()
    np.testing.assert_array_equal(spine.getExternalForceGlobal(), np.zeros(6))


def test_joint_constraints_push_axes_compose_and_rearm(controls):
    setup = _scene("joint_constraints").factory()
    actions = {chr(action.key): action for action in setup.key_actions}
    spine = setup.world.getSkeleton(1).getBodyNode("h_spine")
    actions["1"].callback()
    actions["3"].callback()
    setup.pre_step()
    np.testing.assert_allclose(
        spine.getExternalForceGlobal()[3:], [40, -50, 0], atol=1e-12
    )
    actions["2"].callback()
    actions["4"].callback()
    setup.pre_refresh()
    assert controls["Perturbation active: 0 steps left"].label == (
        "Perturbation active: 100 steps left"
    )
    spine.clearExternalForces()
    setup.pre_step()
    np.testing.assert_allclose(
        spine.getExternalForceGlobal()[3:], [-40, 50, 0], atol=1e-12
    )


def test_joint_constraints_harness_toggles_and_refreshes_text(controls):
    setup = _scene("joint_constraints").factory()
    action = next(action for action in setup.key_actions if action.key == ord("h"))
    solver = setup.world.getConstraintSolver()
    assert solver.getNumConstraints() == 0
    action.callback()
    assert solver.getNumConstraints() == 1
    setup.pre_refresh()
    assert controls["Harness: off"].label == "Harness: on"
    _step(setup, 3)
    assert np.isfinite(setup.world.getSkeleton(1).getPositions()).all()
    action.callback()
    assert solver.getNumConstraints() == 0
    setup.pre_refresh()
    assert controls["Harness: off"].label == "Harness: off"


def test_joint_constraints_controller_tracks_toolbar_timestep():
    from examples.demos.models.joint_constraints import JointConstraintsController

    setup = _scene("joint_constraints").factory()
    biped = setup.world.getSkeleton(1)
    setup.world.setTimeStep(0.002)
    expected = JointConstraintsController(biped, 0.002)
    expected_torques = expected.compute_torques(biped)
    setup.pre_step()
    np.testing.assert_array_equal(biped.getForces(), expected_torques)


def test_s4_operational_space_controls_keys_and_target(controls, monkeypatch):
    setup = _scene("operational_space_control").factory()
    world = setup.world
    robot = world.getSkeleton(0)
    target = world.getSimpleFrame(0)
    assert target.getName() == "target"
    assert target.getShape().getRadius() == 0.025
    np.testing.assert_array_equal(target.getVisualAspect().getColor(), [0.9, 0, 0])
    assert not setup.shadows
    np.testing.assert_array_equal(setup.camera_home.eye, [2.57, 3.14, 1.64])
    np.testing.assert_array_equal(setup.camera_home.up, [-0.24, -0.25, 0.94])
    assert len(controls) == 2
    assert all(
        control.kind == viewer.Control.Kind.TEXT for control in controls.values()
    )
    label = controls["Drag constraint: (none)"]
    assert [(action.key, action.label) for action in setup.key_actions] == [
        (ord("s"), "Toggle shadows")
    ]

    class Drag:
        calls = []

        def setObstructable(self, value):
            self.obstructable = value

        def unconstrain(self):
            self.calls.append(("none",))

        def constrainToLine(self, vector):
            self.calls.append(("line", tuple(vector)))

        def constrainToPlane(self, vector):
            self.calls.append(("plane", tuple(vector)))

    class Context:
        world_node = dart.gui.RealTimeWorldNode(world)
        drag = Drag()
        teardowns = []

        def enable_drag_and_drop(self, frame):
            assert frame is target
            return self.drag

        def add_event_handler(self, handler):
            self.handler = handler

        def add_teardown(self, callback):
            self.teardowns.append(callback)

    ctx = Context()
    setup.on_activate(ctx)
    assert not ctx.drag.obstructable
    assert not ctx.world_node.isShadowed()
    setup.key_actions[0].callback()
    assert ctx.world_node.isShadowed()
    setup.key_actions[0].callback()
    assert not ctx.world_node.isShadowed()
    event = dart.gui.GUIEventAdapter

    def key(kind, key, expected, text):
        assert ctx.handler(event(kind, ord(key)))
        assert ctx.drag.calls[-1] == expected
        setup.pre_refresh()
        assert label.label == "Drag constraint: " + text

    key(event.KEYDOWN, "1", ("line", (1.0, 0.0, 0.0)), "x ")
    monkeypatch.setattr(dart.viewer, "keyboard_capture_active", lambda: True)
    previous = len(ctx.drag.calls)
    assert not ctx.handler(event(event.KEYDOWN, ord("2")))
    assert len(ctx.drag.calls) == previous
    setup.pre_refresh()
    assert label.label == "Drag constraint: x "
    key(event.KEYUP, "1", ("none",), "(none)")
    monkeypatch.setattr(dart.viewer, "keyboard_capture_active", lambda: False)
    key(event.KEYDOWN, "1", ("line", (1.0, 0.0, 0.0)), "x ")
    key(event.KEYDOWN, "2", ("plane", (0.0, 0.0, 1.0)), "x y ")
    key(event.KEYDOWN, "3", ("none",), "x y z ")
    key(event.KEYUP, "1", ("plane", (1.0, 0.0, 0.0)), "y z ")
    key(event.KEYUP, "2", ("line", (0.0, 0.0, 1.0)), "z ")
    key(event.KEYUP, "3", ("none",), "(none)")
    key(event.KEYDOWN, "2", ("line", (0.0, 1.0, 0.0)), "y ")
    key(event.KEYUP, "2", ("none",), "(none)")
    assert not ctx.handler(event(event.KEYDOWN, ord("q")))
    # Mouse motion belongs to the native DnD installed by on_activate.
    assert not ctx.handler(event(event.DRAG, ord("1"), x=10, y=20))
    setup.pre_step()
    forces = robot.getForces().copy()
    tf = target.getWorldTransform()
    tf.pretranslate([0, 0.1, 0])
    target.setTransform(tf)
    setup.pre_step()
    assert not np.allclose(robot.getForces(), forces)
    for teardown in reversed(ctx.teardowns):
        teardown()


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="OctoMap unavailable"
)
def test_point_cloud_sensor_panel_and_visibility(controls):
    setup = _scene("point_cloud").factory()
    world = setup.world
    robot = world.getSkeleton("KR5")
    point_frame = world.getSimpleFrame("point cloud")
    voxel_frame = world.getSimpleFrame("voxel")
    sensor_frame = world.getSimpleFrame("sensor")
    points = point_frame.getShape()
    voxels = voxel_frame.getShape()
    assert isinstance(points, dart.dynamics.PointCloudShape)
    assert isinstance(voxels, dart.dynamics.VoxelGridShape)
    assert isinstance(sensor_frame.getShape(), dart.dynamics.SphereShape)
    assert sensor_frame.getShape().getRadius() == 0.05
    assert not setup.key_actions
    np.testing.assert_array_equal(
        point_frame.getVisualAspect().getRGBA(), [0.1, 0.1, 0.9, 1]
    )
    np.testing.assert_array_equal(
        voxel_frame.getVisualAspect().getRGBA(), [1, 0.63, 0, 0.5]
    )
    np.testing.assert_array_equal(
        sensor_frame.getVisualAspect().getRGBA(), [0.9, 0.1, 0.1, 1]
    )
    assert points.getNumPoints() == 0
    before = robot.getPositions().copy()
    setup.pre_step()
    assert points.getNumPoints() == 500
    assert not np.array_equal(before, robot.getPositions())
    np.testing.assert_array_equal(
        sensor_frame.getWorldTransform().translation(), [0, 1, 0.5]
    )
    assert any(voxels.getOccupancy(point) > 0.5 for point in points.getPoints())

    _edit(controls["Run Robot Updating"], False)
    positions = robot.getPositions().copy()
    cloud = np.array(points.getPoints())
    origin = sensor_frame.getWorldTransform().translation().copy()
    setup.pre_step()
    np.testing.assert_array_equal(robot.getPositions(), positions)
    np.testing.assert_array_equal(points.getPoints(), cloud)
    np.testing.assert_array_equal(
        sensor_frame.getWorldTransform().translation(), origin
    )
    _edit(controls["Run Robot Updating"], True)
    _edit(controls["Sampling Mode"], 1)
    setup.pre_step()
    assert np.max(np.abs(points.getPoints())) <= 0.5
    _edit(controls["Sampling Mode"], 0)
    setup.pre_step()
    assert np.max(np.abs(points.getPoints())) > 0.5
    np.testing.assert_allclose(
        sensor_frame.getWorldTransform().translation(),
        [np.sin(0.002), np.cos(0.002), 0.5 + 0.25 * np.sin(0.004)],
    )

    def edit(label, value):
        _edit(controls[label], value)
        setup.pre_refresh()

    edit("Point Cloud", False)
    assert point_frame.getVisualAspect().isHidden()
    for label in ("Color Mode", "Color##PointCloud", "Point Shape Type", "Visual Size"):
        assert not controls[label].visible
    edit("Point Cloud", True)
    assert not point_frame.getVisualAspect().isHidden()
    assert controls["Color Mode"].visible and not controls["Color##PointCloud"].visible
    for index, mode in enumerate(
        (
            dart.dynamics.PointCloudShape.USE_SHAPE_COLOR,
            dart.dynamics.PointCloudShape.BIND_OVERALL,
            dart.dynamics.PointCloudShape.BIND_PER_POINT,
        )
    ):
        edit("Color Mode", index)
        assert points.getColorMode() == mode
        assert controls["Color##PointCloud"].visible == (index == 0)
    edit("Color Mode", 0)
    edit("Color##PointCloud", [0.2, 0.3, 0.4, 0.5])
    np.testing.assert_allclose(
        point_frame.getVisualAspect().getRGBA(), [0.2, 0.3, 0.4, 0.5]
    )
    for index in range(4):
        edit("Point Shape Type", index)
        assert controls["Point Shape Type"].value == index
        assert int(points.getPointShapeType()) == index
    edit("Visual Size", 0.07)
    assert points.getVisualSize() == float(np.float32(0.07))
    edit("Visual Size", -1)
    assert points.getVisualSize() == float(np.float32(0.01))
    for value in (np.nan, np.inf, 1e100):
        edit("Visual Size", value)
        assert points.getVisualSize() == float(np.float32(0.01))
    edit("Voxel Grid", False)
    assert (
        voxel_frame.getVisualAspect().isHidden()
        and not controls["Voxel grid color"].visible
    )
    edit("Voxel Grid", True)
    assert (
        not voxel_frame.getVisualAspect().isHidden()
        and controls["Voxel grid color"].visible
    )
    edit("Voxel grid color", [0.4, 0.3, 0.2, 0.1])
    np.testing.assert_allclose(
        voxel_frame.getVisualAspect().getRGBA(), [0.4, 0.3, 0.2, 0.1]
    )
    np.testing.assert_allclose(setup.camera_home.eye, [2.57, 3.14, 1.64])
    np.testing.assert_allclose(setup.camera_home.center, [0, 0, 0.30])
    np.testing.assert_allclose(setup.camera_home.up, [-0.24, -0.25, 0.94])


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="OctoMap unavailable"
)
def test_point_cloud_attachment_and_every_grid_control(controls):
    setup = _scene("point_cloud").factory()

    class Context:
        def __init__(self):
            self.attachments = []

        def add_attachment(self, attachment):
            self.attachments.append(attachment)

    context = Context()
    setup.on_activate(context)
    (grid,) = context.attachments
    assert isinstance(grid, dart.gui.GridVisual)
    assert {label for label, control in controls.items() if control.on_change} == {
        "Run Robot Updating",
        "Sampling Mode",
        "Point Cloud",
        "Color Mode",
        "Color##PointCloud",
        "Point Shape Type",
        "Visual Size",
        "Voxel Grid",
        "Voxel grid color",
        "Show##Grid",
        "Plane",
        "X",
        "Y",
        "Z",
        "Line Count",
        "Line Step Size",
        "Minor Lines per Major Line",
        "Axis Line Width",
        "Major Line Width",
        "Major Line Color",
        "Minor Line Width",
        "Minor Line Color",
    }
    _edit(controls["Show##Grid"], False)
    assert not grid.isDisplayed() and not controls["Plane"].visible
    _edit(controls["Show##Grid"], True)
    assert grid.isDisplayed() and controls["Plane"].visible
    for index, plane in enumerate(
        (
            dart.gui.GridVisual.PlaneType.XY,
            dart.gui.GridVisual.PlaneType.YZ,
            dart.gui.GridVisual.PlaneType.ZX,
        )
    ):
        _edit(controls["Plane"], index)
        assert grid.getPlaneType() == plane
    for index, label in enumerate(("X", "Y", "Z")):
        _edit(controls[label], (index + 1) * 0.25)
    np.testing.assert_array_equal(grid.getOffset(), [0.25, 0.5, 0.75])
    _edit(controls["X"], 1e100)
    np.testing.assert_array_equal(grid.getOffset(), [0.25, 0.5, 0.75])
    for label, value, getter in (
        ("Line Count", 12, grid.getNumCells),
        ("Minor Lines per Major Line", 3, grid.getNumMinorLinesPerMajorLine),
        ("Line Step Size", 0.04, grid.getMinorLineStepSize),
        ("Axis Line Width", 2, grid.getAxisLineWidth),
        ("Major Line Width", 3, grid.getMajorLineWidth),
        ("Minor Line Width", 4, grid.getMinorLineWidth),
    ):
        _edit(controls[label], value)
        assert getter() == pytest.approx(value)
        _edit(controls[label], np.nan)
        assert getter() == pytest.approx(value)
        if label not in ("Line Count", "Minor Lines per Major Line"):
            _edit(controls[label], 1e100)
            assert getter() == pytest.approx(value)
    for label, getter in (
        ("Line Count", grid.getNumCells),
        ("Minor Lines per Major Line", grid.getNumMinorLinesPerMajorLine),
        ("Line Step Size", grid.getMinorLineStepSize),
    ):
        _edit(controls[label], -1)
        assert getter() == 0
    for label, getter in (
        ("Axis Line Width", grid.getAxisLineWidth),
        ("Major Line Width", grid.getMajorLineWidth),
        ("Minor Line Width", grid.getMinorLineWidth),
    ):
        # A width clamped to zero keeps the previous width, as in the S3 scenes.
        previous = getter()
        _edit(controls[label], -1)
        assert getter() == previous
    for label, getter in (
        ("Major Line Color", grid.getMajorLineColor),
        ("Minor Line Color", grid.getMinorLineColor),
    ):
        alpha = getter()[3]
        _edit(controls[label], [0.1, 0.3, 0.5, 0.2])
        np.testing.assert_allclose(getter(), [0.1, 0.3, 0.5, alpha])


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="OctoMap unavailable"
)
def test_point_cloud_inputs_match_cpp_steps(monkeypatch):
    inputs = {}
    original = viewer.Control.add_input

    def record(self, label, value, **kwargs):
        control = original(self, label, value, **kwargs)
        inputs[label] = (
            kwargs.get("step", 0),
            kwargs.get("step_fast", 0),
            kwargs.get("decimals", 3),
            kwargs.get("integer", False),
        )
        return control

    monkeypatch.setattr(viewer.Control, "add_input", record)
    _scene("point_cloud").factory()
    assert inputs == {
        "Visual Size": (0.01, 0.02, 2, False),
        "X": (0.1, 0.5, 1, False),
        "Y": (0.1, 0.5, 1, False),
        "Z": (0.1, 0.5, 1, False),
        "Line Count": (1, 5, 3, True),
        "Line Step Size": (0.001, 0.1, 3, False),
        "Minor Lines per Major Line": (1, 5, 3, True),
        "Axis Line Width": (1, 2, 0, False),
        "Major Line Width": (1, 2, 0, False),
        "Minor Line Width": (1, 2, 0, False),
    }


def test_point_cloud_repeated_steps_keep_points_and_occupancy_live():
    setup = _scene("point_cloud").factory()
    world = setup.world
    points = world.getSimpleFrame("point cloud").getShape()
    voxels = world.getSimpleFrame("voxel").getShape()
    dart.math.Random.setSeed(42)
    _step(setup)
    before = np.asarray(points.getPoints()).copy()
    _step(setup, 12)
    after = np.asarray(points.getPoints())
    assert points.getNumPoints() == 500
    assert np.isfinite(after).all()
    assert not np.array_equal(before, after)
    assert any(voxels.getOccupancy(point) > 0.5 for point in after)
    assert world.getTime() == pytest.approx(13 * world.getTimeStep())
