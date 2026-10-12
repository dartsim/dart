"""C++ catalog construction and interaction contracts for the first scene batch."""

import gc
import importlib
import sys
from pathlib import Path
from types import SimpleNamespace

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(
    viewer is None, reason="a vanilla system ImGui compiles out the demo app host"
)

# Ordered skeleton name/body/DOF counts from dart-demos --debug-print-world.
STRUCTURES = {
    "empty": [],
    "hello_world": [("box", 1, 6), ("ground", 1, 0)],
    "simple_frames": [],
    "boxes": [(f"box{i}", 1, 6) for i in range(125)] + [("ground", 1, 0)],
    "rigid_cubes": [
        ("ground skeleton", 1, 0),
        ("box skeleton", 1, 6),
        ("skeleton_2", 1, 6),
        ("skeleton_3", 1, 6),
    ],
    "rigid_chain": [("skeleton 1", 10, 30)],
    "add_delete_skels": [("ground skeleton", 1, 0)],
    "simulation_event_handler": [
        ("ground", 1, 0),
        ("box1", 1, 6),
        ("box2", 1, 6),
        ("sphere1", 1, 6),
        ("box3", 1, 6),
    ],
    "rigid_shapes": [("ground skeleton", 1, 0)]
    + [
        (f"{name} skeleton", 1, 6)
        for name in (
            "box",
            "sphere",
            "ellipsoid",
            "cylinder",
            "capsule",
            "cone",
            "pyramid",
            "multi sphere",
            "mesh",
        )
    ],
}
CAMERAS = {
    "empty": ([2.57, 3.14, 1.64], [0, 0, 0], [-0.24, -0.25, 0.94]),
    "hello_world": ([2.57, 3.14, 1.64], [0, 0, 0.5], [-0.24, -0.25, 0.94]),
    "simple_frames": ([0.55, 0.25, 0.5], [0.08, 0, 0.03], [0, 0, 1]),
    "boxes": ([20, 20, 15], [0, 0, 3], [0, 0, 1]),
    "rigid_cubes": ([1.1, 1.1, 0.55], [0, 0, -0.3], [0, 0, 1]),
    "rigid_chain": ([2, 1, 2], [0, 0, 0], [0, 0, 1]),
    "add_delete_skels": ([7, 7, 8], [0, 0, 1], [0, 0, 1]),
    "simulation_event_handler": ([5, 5, 3], [0, 0, 1], [0, 0, 1]),
    "rigid_shapes": ([2, -2, 2], [0, 0, 0], [0, 0, 1]),
}


def build(scene_id):
    module = importlib.import_module(f"examples.demos.scenes.{scene_id}")
    scenes = module.scenes()
    assert len(scenes) == 1 and scenes[0].id == scene_id
    return scenes[0].factory()


def controls(setup):
    # The binding retains panel children through its public Python GC traversal.
    # Control has no child accessor; these are the actual native control wrappers.
    return {
        child.label: child
        for child in gc.get_referents(setup.panel)
        if isinstance(child, viewer.Control)
    }


def edit(control, value):
    control.edit(value)
    if control.on_change is not None:
        control.on_change()


def refresh(setup):
    if setup.pre_refresh is not None:
        setup.pre_refresh()


def step(setup, count=1):
    for _ in range(count):
        refresh(setup)
        if setup.pre_step is not None:
            setup.pre_step()
        setup.world.step()
        if setup.post_step is not None:
            setup.post_step()


def world_state(world):
    values = []
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        values.extend((skeleton.getPositions().copy(), skeleton.getVelocities().copy()))
        for j in range(skeleton.getNumBodyNodes()):
            transform = skeleton.getBodyNode(j).getWorldTransform()
            values.extend(
                (transform.translation().copy(), transform.rotation().ravel())
            )
    for i in range(world.getNumSimpleFrames()):
        transform = world.getSimpleFrame(i).getWorldTransform()
        values.extend((transform.translation().copy(), transform.rotation().ravel()))
    return np.concatenate(values) if values else np.array([])


def actions(setup):
    return {action.key: action for action in setup.key_actions}


@pytest.mark.parametrize("scene_id", STRUCTURES)
def test_cpp_world_structure_and_seeded_steps(scene_id):
    runs = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        setup = build(scene_id)
        world = setup.world
        skeletons = [world.getSkeleton(i) for i in range(world.getNumSkeletons())]
        assert [
            (s.getName(), s.getNumBodyNodes(), s.getNumDofs()) for s in skeletons
        ] == STRUCTURES[scene_id]
        assert all(s.getNumSoftBodyNodes() == 0 for s in skeletons)
        assert world.getConstraintSolver().getNumConstraints() == 0
        assert world.getNumSimpleFrames() == {"empty": 1, "simple_frames": 3}.get(
            scene_id, 0
        )
        assert world.getTimeStep() == pytest.approx(
            0.0005 if scene_id == "rigid_chain" else 0.001
        )
        np.testing.assert_allclose(world.getGravity(), [0, 0, -9.81], atol=1e-14)
        assert setup.allow_simulation
        assert setup.shadows is (scene_id != "empty")
        home = setup.camera_home
        np.testing.assert_allclose((home.eye, home.center, home.up), CAMERAS[scene_id])
        states = [world_state(world)]
        for _ in range(5):
            step(setup)
            states.append(world_state(world))
        assert world.getSimFrames() == 5
        runs.append(np.array(states))
    assert np.isfinite(runs[0]).all()
    np.testing.assert_array_equal(runs[0], runs[1])


def test_catalog_order_and_key_labels():
    from examples.demos.registry import make_demo_scenes

    scenes = make_demo_scenes()
    assert [scene.id for scene in scenes[:9]] == list(STRUCTURES)
    event = dart.gui.GUIEventAdapter
    expected = {
        "rigid_cubes": [
            (ord(key), label)
            for key, label in zip(
                "1234", ("Force -X", "Force +X", "Force +Y", "Force -Y")
            )
        ],
        "add_delete_skels": [
            (ord("q"), "Spawn cube"),
            (ord("w"), "Delete last skeleton"),
        ],
        "rigid_shapes": [
            (ord(key), label)
            for key, label in zip(
                "qwea",
                (
                    "Spawn box",
                    "Spawn ellipsoid",
                    "Spawn cylinder",
                    "Delete last skeleton",
                ),
            )
        ],
        "simulation_event_handler": [
            (int(event.KEY_Tab), "Select next body"),
            (int(event.KEY_BackSpace), "Select previous body"),
            (int(event.KEY_Up), "Force +Y"),
            (int(event.KEY_Down), "Force -Y"),
            (int(event.KEY_Left), "Force -X"),
            (int(event.KEY_Right), "Force +X"),
        ]
        + [
            (ord(key), label)
            for key, label in zip(
                "udqweazc=-v",
                (
                    "Force +Z (up)",
                    "Force -Z (down)",
                    "Torque +X",
                    "Torque +Y",
                    "Torque +Z",
                    "Torque -X",
                    "Torque -Y",
                    "Torque -Z",
                    "Increase magnitude",
                    "Decrease magnitude",
                    "Toggle force arrows",
                ),
            )
        ],
    }
    for scene_id in STRUCTURES:
        assert [(a.key, a.label) for a in build(scene_id).key_actions] == expected.get(
            scene_id, []
        )


def test_empty_gizmo_native_mouse_events_and_position_text():
    setup = build("empty")
    target = setup.world.getSimpleFrame(0)
    native = dart.gui.Viewer()
    node = dart.gui.RealTimeWorldNode(setup.world)
    native.addWorldNode(node)
    # Aim at the positive X translation arrow, outside the rotation rings.
    native.setCameraHome([0.17, -1, 0], [0.17, 0, 0], [0, 0, 1])
    interactions = []

    def enable(frame):
        assert frame is target
        interactions.append(native.enableDragAndDrop(frame))
        return interactions[-1]

    setup.on_activate(SimpleNamespace(enable_drag_and_drop=enable))
    assert len(interactions) == 1
    drag = interactions[0]
    assert isinstance(drag, dart.gui.InteractiveFrameDnD)
    handler = native.getDefaultEventHandler()
    adapter = dart.gui.GUIActionAdapter(native)
    event = dart.gui.GUIEventAdapter
    before = target.getWorldTransform().translation().copy()
    handler.handle(
        event(event.PUSH, x=800, y=500, buttons=int(event.LEFT_MOUSE_BUTTON)), adapter
    )
    assert drag.isMoving()
    handler.handle(
        event(event.DRAG, x=820, y=500, buttons=int(event.LEFT_MOUSE_BUTTON)), adapter
    )
    after = target.getWorldTransform().translation().copy()
    assert after[0] > before[0]
    np.testing.assert_allclose(after[1:], before[1:], atol=1e-12)
    handler.handle(event(event.RELEASE, x=820, y=500), adapter)
    assert not drag.isMoving()
    refresh(setup)
    assert f"Position: ({after[0]:.2f}, {after[1]:.2f}, {after[2]:.2f})" in controls(
        setup
    )
    assert native.disableDragAndDrop(drag)
    assert drag.getEntity() is None


def test_read_only_panels_refresh_from_the_world():
    hello = build("hello_world")
    box = hello.world.getSkeleton("box").getBodyNode(0)
    step(hello, 3)
    refresh(hello)
    assert f"Box height: {box.getWorldTransform().translation()[2]:.2f} m" in controls(
        hello
    )
    assert f"Box velocity: {box.getSpatialVelocity()[5]:.2f} m/s" in controls(hello)
    boxes = build("boxes")
    boxes.world.removeSkeleton(boxes.world.getSkeleton(0))
    refresh(boxes)
    assert "Boxes: 124" in controls(boxes)
    cubes = build("rigid_cubes")
    cubes.world.removeSkeleton(cubes.world.getSkeleton(3))
    refresh(cubes)
    assert "Skeletons: 3" in controls(cubes)


def test_simple_frame_hierarchy_and_show_arrow_control():
    setup = build("simple_frames")
    world = setup.world
    f1, a, arrow = (world.getSimpleFrame(i) for i in range(3))
    f2 = next(iter(f1.getChildFrames()))
    f3 = next(iter(f2.getChildFrames()))
    assert (f1.getName(), f2.getName(), f3.getName(), a.getName(), arrow.getName()) == (
        "F1",
        "F2",
        "F3",
        "A",
        "arrow",
    )
    markers = {child.getName(): child for child in a.getChildFrames()}
    assert set(markers) == {"A1", "A2", "A3"}
    for frame, name in zip((f1, f2, f3), ("A1", "A2", "A3")):
        np.testing.assert_allclose(
            frame.getWorldTransform().matrix(),
            markers[name].getWorldTransform().matrix(),
        )
    show = controls(setup)["Show arrow"]
    for visible in (False, True):
        edit(show, visible)
        assert arrow.getVisualAspect().isHidden() is (not visible)
    gc.collect()
    step(setup)
    assert f3.getParentFrame() is f2


@pytest.mark.parametrize(
    "key, force",
    [
        ("1", [-500, 0, 0]),
        ("2", [500, 0, 0]),
        ("3", [0, 500, 0]),
        ("4", [0, -500, 0]),
    ],
)
def test_cube_impulse_targets_second_skeleton_and_decays(key, force):
    setup = build("rigid_cubes")
    body = setup.world.getSkeleton(1).getBodyNode(0)
    actions(setup)[ord(key)].callback()
    for scale in (1.0, 0.5, 0.25):
        body.clearExternalForces()
        setup.pre_step()
        np.testing.assert_allclose(
            body.getExternalForceGlobal()[3:], np.array(force) * scale
        )
        assert (
            not setup.world.getSkeleton(2).getBodyNode(0).getExternalForceGlobal().any()
        )


def test_chain_panel_edits_preserve_twist_damping_law():
    setup = build("rigid_chain")
    chain = setup.world.getSkeleton(0)
    velocity = np.arange(1, 31, dtype=float) / 10
    chain.setVelocities(velocity)
    setup.pre_step()
    expected = -0.01 * velocity
    expected[1::3] *= 0.1
    np.testing.assert_allclose(chain.getForces(), expected)
    panel = controls(setup)
    edit(panel["Damping"], 0.08)
    edit(panel["Twist scale"], 0.5)
    setup.pre_step()
    expected = -0.08 * velocity
    expected[1::3] *= 0.5
    np.testing.assert_allclose(chain.getForces(), expected)
    refresh(setup)
    assert "DOFs: 30" in controls(setup)
    assert f"Speed: {np.linalg.norm(velocity):.3f} rad/s" in controls(setup)
    edit(panel["Damping"], 0)
    setup.pre_step()
    assert not chain.getForces().any()


def test_add_delete_keys_spawn_cubes_and_preserve_ground():
    dart.math.Random.setSeed(17)
    setup = build("add_delete_skels")
    world = setup.world
    keys = actions(setup)
    ground = world.getSkeleton(0)
    keys[ord("w")].callback()
    assert world.getNumSkeletons() == 1
    for index in range(2):
        keys[ord("q")].callback()
        cube = world.getSkeleton(index + 1)
        assert cube.getName() == f"cube{index}"
        assert cube.getNumBodyNodes() == 1 and cube.getNumDofs() == 6
        body = cube.getBodyNode(0)
        assert body.getName() == "cube_link"
        assert body.getParentJoint().getName() == "cube_joint"
        position = body.getWorldTransform().translation()
        assert np.all(np.abs(position[:2]) <= 1) and 0.5 <= position[2] <= 1
        shape = body.getShapeNode(0).getShape()
        assert isinstance(shape, dart.dynamics.BoxShape)
        assert np.all((shape.getSize() >= 0.1) & (shape.getSize() <= 0.5))
        assert body.getMass() == pytest.approx(0.1)
        np.testing.assert_allclose(
            body.getInertia().getMoment(), shape.computeInertia(0.1)
        )
    refresh(setup)
    assert "Spawned cubes: 2" in controls(setup)
    for _ in range(3):
        keys[ord("w")].callback()
    assert world.getNumSkeletons() == 1 and world.getSkeleton(0) is ground
    keys[ord("q")].callback()
    assert world.getSkeleton(1).getName() == "cube2"


def test_seeded_cube_spawning_and_steps_are_deterministic():
    runs = []
    for _ in range(2):
        dart.math.Random.setSeed(17)
        setup = build("add_delete_skels")
        for _ in range(2):
            actions(setup)[ord("q")].callback()
        step(setup, 3)
        runs.append(world_state(setup.world))
    np.testing.assert_array_equal(runs[0], runs[1])


@pytest.mark.parametrize(
    "key, shape_class, link",
    [
        ("q", "BoxShape", "box"),
        ("w", "EllipsoidShape", "ellipsoid"),
        ("e", "CylinderShape", "cylinder"),
    ],
)
def test_rigid_shape_spawn_and_delete_keys(key, shape_class, link):
    runs = []
    for _ in range(2):
        dart.math.Random.setSeed(17)
        setup = build("rigid_shapes")
        world = setup.world
        keys = actions(setup)
        keys[ord(key)].callback()
        assert world.getNumSkeletons() == 11
        body = world.getSkeleton(10).getBodyNode(0)
        assert body.getName() == f"{link}_link"
        assert body.getParentJoint().getName() == f"{link}_joint"
        assert body.getMass() == pytest.approx(10)
        shape = body.getShapeNode(0).getShape()
        assert isinstance(shape, getattr(dart.dynamics, shape_class))
        position = body.getWorldTransform().translation()
        assert np.all(np.abs(position[:2]) <= 1) and 0.5 <= position[2] <= 1
        if key in "qw":
            size = shape.getSize() if key == "q" else shape.getDiameters()
            assert np.all((size >= 0.05) & (size <= 0.25))
        else:
            assert 0.05 <= shape.getRadius() <= 0.25
            assert 0.1 <= shape.getHeight() <= 0.5
        step(setup, 3)
        runs.append(world_state(world))
        refresh(setup)
        assert "Skeletons: 11" in controls(setup)
        for _ in range(11):
            keys[ord("a")].callback()
        assert world.getNumSkeletons() == 1
        assert world.getSkeleton(0).getName() == "ground skeleton"
    np.testing.assert_array_equal(runs[0], runs[1])


@pytest.mark.parametrize(
    "key, torque, vector",
    [
        ("KEY_Up", False, [0, 100, 0]),
        ("KEY_Down", False, [0, -100, 0]),
        ("KEY_Left", False, [-100, 0, 0]),
        ("KEY_Right", False, [100, 0, 0]),
        ("u", False, [0, 0, 100]),
        ("d", False, [0, 0, -100]),
        ("q", True, [50, 0, 0]),
        ("w", True, [0, 50, 0]),
        ("e", True, [0, 0, 50]),
        ("a", True, [-50, 0, 0]),
        ("z", True, [0, -50, 0]),
        ("c", True, [0, 0, -50]),
    ],
)
def test_simulation_event_force_and_torque_keys(key, torque, vector):
    setup = build("simulation_event_handler")
    selected = setup.world.getSkeleton("box1")
    selected.setPositions([0.2, -0.3, 0.4, 0, 0, 0])
    code = (
        int(getattr(dart.gui.GUIEventAdapter, key))
        if key.startswith("KEY_")
        else ord(key)
    )
    actions(setup)[code].callback()
    body = selected.getBodyNode(0)
    force = body.getExternalForceGlobal()
    np.testing.assert_allclose(force[:3] if torque else force[3:], vector, atol=1e-12)
    assert (
        not setup.world.getSkeleton("box2")
        .getBodyNode(0)
        .getExternalForceGlobal()
        .any()
    )


def test_simulation_event_selection_magnitudes_and_panel_edits():
    setup = build("simulation_event_handler")
    keys = actions(setup)
    panel = controls(setup)
    force_slider = panel["Force mag"]
    torque_slider = panel["Torque mag"]
    assert (force_slider.min, force_slider.max) == (1, 500)
    assert (torque_slider.min, torque_slider.max) == (1, 200)
    event = dart.gui.GUIEventAdapter
    keys[int(event.KEY_BackSpace)].callback()
    refresh(setup)
    assert "Selected: box3_body (4/4)" in controls(setup)
    keys[int(event.KEY_Tab)].callback()
    refresh(setup)
    assert "Selected: box1_body (1/4)" in controls(setup)
    keys[ord("=")].callback()
    assert force_slider.value == 150
    assert torque_slider.value == 75
    keys[ord("-")].callback()
    assert force_slider.value == pytest.approx(100.5)
    assert torque_slider.value == pytest.approx(50.25)
    refresh(setup)
    assert "Force magnitude: 100.5 N" in controls(setup)
    assert "Torque magnitude: 50.2 N m" in controls(setup)
    keys[ord("u")].callback()
    body = setup.world.getSkeleton("box1").getBodyNode(0)
    np.testing.assert_allclose(body.getExternalForceGlobal()[3:], [0, 0, 100.5])
    body.clearExternalForces()
    panel = controls(setup)
    edit(panel["Force mag"], 250)
    edit(panel["Torque mag"], 80)
    keys[ord("u")].callback()
    keys[ord("e")].callback()
    np.testing.assert_allclose(body.getExternalForceGlobal()[3:], [0, 0, 250])
    assert body.getExternalForceGlobal()[2] == pytest.approx(80)
    for _ in range(30):
        keys[ord("=")].callback()
    assert force_slider.value == 500
    assert torque_slider.value == 200
    refresh(setup)
    assert "Force magnitude: 10000.0 N" in controls(setup)
    assert "Torque magnitude: 5000.0 N m" in controls(setup)
    body.clearExternalForces()
    keys[ord("u")].callback()
    keys[ord("e")].callback()
    np.testing.assert_allclose(body.getExternalForceGlobal()[3:], [0, 0, 10000])
    assert body.getExternalForceGlobal()[2] == pytest.approx(5000)
    for _ in range(50):
        keys[ord("-")].callback()
    assert force_slider.value == 1
    assert torque_slider.value == 1
    refresh(setup)
    assert "Force magnitude: 0.1 N" in controls(setup)
    assert "Torque magnitude: 0.1 N m" in controls(setup)
    body.clearExternalForces()
    keys[ord("u")].callback()
    keys[ord("e")].callback()
    np.testing.assert_allclose(body.getExternalForceGlobal()[3:], [0, 0, 0.1])
    assert body.getExternalForceGlobal()[2] == pytest.approx(0.1)
    edit(force_slider, 123.4)
    edit(torque_slider, 56.7)
    body.clearExternalForces()
    keys[ord("u")].callback()
    keys[ord("e")].callback()
    np.testing.assert_allclose(
        body.getExternalForceGlobal()[3:],
        [0, 0, float(np.float32(123.4))],
        rtol=0,
        atol=1e-12,
    )
    assert body.getExternalForceGlobal()[2] == pytest.approx(
        float(np.float32(56.7)), rel=0, abs=1e-12
    )
    refresh(setup)
    assert "Force magnitude: 123.4 N" in controls(setup)
    assert "Torque magnitude: 56.7 N m" in controls(setup)


def test_simulation_event_arrows_update_every_five_steps_and_hide_immediately():
    setup = build("simulation_event_handler")
    world = setup.world
    keys = actions(setup)
    keys[ord("u")].callback()
    keys[ord("e")].callback()
    step(setup, 4)
    assert world.getNumSimpleFrames() == 0
    step(setup)
    assert world.getNumSimpleFrames() == 2
    frames = [world.getSimpleFrame(i) for i in range(2)]
    assert all(not frame.getVisualAspect().isHidden() for frame in frames)
    com = world.getSkeleton("box1").getBodyNode(0).getCOM()
    for frame in frames:
        arrow = frame.getShape()
        assert isinstance(arrow, dart.dynamics.ArrowShape)
        np.testing.assert_allclose(arrow.getTail(), com)
        np.testing.assert_allclose(arrow.getHead(), com + [0, 0, 0.5])
        properties = arrow.getProperties()
        assert properties.mRadius == pytest.approx(0.006)
        assert properties.mHeadRadiusScale == pytest.approx(2.0)
        assert properties.mHeadLengthScale == pytest.approx(0.15)
    keys[ord("v")].callback()
    assert all(frame.getVisualAspect().isHidden() for frame in frames)
    refresh(setup)
    show = controls(setup)["Show force/torque arrows"]
    assert show.value is False
    edit(show, True)
    keys[ord("u")].callback()
    step(setup, 5)
    assert not frames[0].getVisualAspect().isHidden()
    assert frames[1].getVisualAspect().isHidden()
    edit(show, False)
    assert all(frame.getVisualAspect().isHidden() for frame in frames)
    keys[ord("u")].callback()
    step(setup, 5)
    assert all(frame.getVisualAspect().isHidden() for frame in frames)
