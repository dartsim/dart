"""Construction and scripted interaction parity for the third demo batch."""

import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
from examples.demos.models.boxed_lcp_solver import set_boxed_lcp_solver

viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

# Scene-port parity baseline: (name, bodies, soft bodies, point masses, DOFs).
STRUCTURES = {
    "sleeping": [("floor", 1, 0, 0, 0)]
    + [(f"box{i}", 1, 0, 0, 6) for i in range(16)]
    + [(name, 1, 0, 0, 6) for name in ("proj_shot0", "proj_drop0")],
    "box_stacking": [("floor", 1, 0, 0, 0)]
    + [(name, 1, 0, 0, 6) for name in ("box", "box(1)", "box(2)", "box(3)", "box(4)")],
    "heightmap": [],
}
METADATA = {
    "sleeping": (
        "Sleeping",
        "Rigid Body",
        "Automatic body deactivation across four independent box-stack solver islands.",
    ),
    "box_stacking": (
        "Box Stacking",
        "Constraints & Joints",
        "Stacked boxes comparing Dantzig and PGS LCP solvers.",
    ),
    "heightmap": (
        "Heightmap",
        "Visualization",
        "An interactive procedural heightmap with a configurable grid.",
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
        values.append(np.array([skeleton.isResting()]))
        for body in skeleton.getBodyNodes():
            for shape in body.getShapeNodes():
                values.append(shape.getVisualAspect().getRGBA())
    for i in range(world.getNumSimpleFrames()):
        frame = world.getSimpleFrame(i)
        values.append(frame.getWorldTransform().matrix().ravel())
        shape = frame.getShape()
        if isinstance(shape, dart.dynamics.HeightmapShapef):
            values.append(shape.getHeightField().ravel())
        elif isinstance(shape, dart.dynamics.LineSegmentShape):
            values.extend(shape.getVertex(i) for i in range(32))
    return np.concatenate(values).copy()


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


def _cycle(factory, frames=2):
    scene = viewer.Scene(
        "scripted", title="Scripted", category="Tests", factory=factory
    )
    return viewer.run([scene], ["test", "--cycle-scenes", "--frames", str(frames)])


@pytest.fixture
def controls(monkeypatch):
    """Observe native controls without adding a scene-only test API."""
    recorded = {}
    for method in (
        "add_text",
        "add_checkbox",
        "add_slider",
        "add_input",
        "add_choice",
        "add_color",
    ):
        original = getattr(viewer.Control, method)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded[control.label] = control
            return control

        monkeypatch.setattr(viewer.Control, method, record)
    return recorded


@pytest.mark.parametrize("scene_id", STRUCTURES)
def test_s3_construction_and_deterministic_steps(scene_id):
    scene = _scene(scene_id)
    assert (scene.title, scene.category, scene.summary) == METADATA[scene_id]
    states = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        setup = scene.factory()
        world = setup.world
        assert _structure(world) == STRUCTURES[scene_id]
        assert world.getNumSimpleFrames() == (0 if scene_id == "box_stacking" else 1)
        assert world.getConstraintSolver().getNumConstraints() == 0
        np.testing.assert_array_equal(
            world.getGravity(), [0, 0, 0 if scene_id == "heightmap" else -9.81]
        )
        assert world.getTimeStep() == (1 / 60 if scene_id == "sleeping" else 0.001)
        assert setup.shadows and setup.allow_simulation
        _step(setup, 4)
        assert world.getSimFrames() == 4
        states.append(_state(world))
    assert np.isfinite(states[0]).all()
    np.testing.assert_array_equal(*states)


def test_s3_registration_keeps_cpp_catalog_positions():
    from examples.demos.registry import make_demo_scenes

    ids = [scene.id for scene in make_demo_scenes()]
    for previous, scene_id, following in (
        ("rigid_shapes", "sleeping", "hardcoded_design"),
        ("rigid_loop", "box_stacking", "dynamic_joint_constraints"),
        ("vehicle", "heightmap", "drag_and_drop"),
    ):
        index = ids.index(scene_id)
        assert ids.count(scene_id) == 1
        assert ids.index(previous) < index < ids.index(following)


def test_sleeping_initial_stacks_and_projectile_pool():
    setup = _scene("sleeping").factory()
    world = setup.world
    for i in range(16):
        box = world.getSkeleton(f"box{i}")
        np.testing.assert_allclose(
            box.getBodyNode(0).getWorldTransform().translation(),
            [[-2, -0.7, 0.7, 2][i // 4], 0, 0.16 + (i % 4) * 0.301],
        )
        assert box.getBodyNode(0).getMass() == pytest.approx(27)
    for name, y in (("proj_shot0", 60), ("proj_drop0", 61)):
        projectile = world.getSkeleton(name)
        # World insertion wakes parked projectiles in both scene factories.
        assert not projectile.isResting()
        assert not projectile.getBodyNode(0).isCollidable()
        np.testing.assert_array_equal(projectile.getPositions()[3:], [60, y, 10])
        np.testing.assert_array_equal(projectile.getVelocities(), np.zeros(6))
    assert world.getDeactivationOptions().mTimeUntilSleep == 0.3
    np.testing.assert_array_equal(setup.camera_home.eye, [6, 8, 4])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 1])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 0, 1])
    assert setup.on_activate is None


def test_sleeping_keys_and_all_panel_controls(controls):
    setup = _scene("sleeping").factory()
    world = setup.world
    actions = {chr(action.key): action for action in setup.key_actions}
    assert {key: action.label for key, action in actions.items()} == {
        "f": "Shoot sphere",
        "d": "Drop box",
        "t": "Toggle sleeping",
        "c": "Toggle island colors",
    }
    assert {label for label, control in controls.items() if control.on_change} == {
        "Automatic deactivation (sleeping)",
        "Color by solver island",
        "Dwell time",
    }
    shot = world.getSkeleton("proj_shot0")
    drop = world.getSkeleton("proj_drop0")
    aim = world.getSimpleFrame("aim").getShape()
    np.testing.assert_array_equal(aim.getVertex(0), [-2, 3, 2])
    actions["f"].callback()
    assert world.getNumSkeletons() == 19
    assert not shot.isResting() and shot.getBodyNode(0).isCollidable()
    np.testing.assert_array_equal(shot.getPositions()[3:], [-2, 3, 2])
    np.testing.assert_array_equal(shot.getVelocities(), [0, 0, 0, 0, -4.2, 1.7])
    setup.pre_refresh()
    np.testing.assert_array_equal(aim.getVertex(0), [-0.7, 3, 2])
    assert controls["Active  0 sphere(s) / 0 box(es)"].label == (
        "Active  1 sphere(s) / 0 box(es)"
    )
    actions["d"].callback()
    assert world.getNumSkeletons() == 19
    assert shot.isResting() and not shot.getBodyNode(0).isCollidable()
    assert not drop.isResting() and drop.getBodyNode(0).isCollidable()
    np.testing.assert_array_equal(drop.getPositions()[3:], [-3.6, 1.2, 2.7])
    np.testing.assert_allclose(drop.getVelocities()[3:], [2.9 / 0.65, -1.2 / 0.65, 0])
    setup.pre_refresh()
    assert controls["Active  0 sphere(s) / 0 box(es)"].label == (
        "Active  0 sphere(s) / 1 box(es)"
    )
    for _ in range(5):
        actions["f"].callback()
    assert world.getNumSkeletons() == 19

    enabled = world.getDeactivationOptions().mEnabled
    actions["t"].callback()
    assert world.getDeactivationOptions().mEnabled is not enabled
    setup.pre_refresh()
    assert controls["Automatic deactivation (sleeping)"].value is not enabled
    _edit(controls["Automatic deactivation (sleeping)"], enabled)
    assert world.getDeactivationOptions().mEnabled is enabled
    for requested, expected in ((0.75, 0.75), (-1, 0.05), (4, 2)):
        _edit(controls["Dwell time"], requested)
        assert world.getDeactivationOptions().mTimeUntilSleep == pytest.approx(expected)
    _edit(controls["Dwell time"], np.nan)
    assert world.getDeactivationOptions().mTimeUntilSleep == 2

    body = world.getSkeleton("box0").getBodyNode(0)
    color = body.getShapeNode(0).getVisualAspect()
    setup.pre_refresh()
    object_rgba = color.getRGBA().copy()
    actions["c"].callback()
    setup.pre_refresh()
    assert controls["Color by solver island"].value is True
    assert not np.allclose(color.getRGBA(), object_rgba)
    _edit(controls["Color by solver island"], False)
    setup.pre_refresh()
    np.testing.assert_array_equal(color.getRGBA(), object_rgba)
    world.getSkeleton("box0").setResting(True)
    setup.pre_refresh()
    assert np.all(color.getRGBA()[:3] < object_rgba[:3])
    assert (
        controls["Stacks  16 awake / 0 asleep"].label == "Stacks  15 awake / 1 asleep"
    )


def test_box_stacking_solver_panel_and_selection_noop(controls):
    setup = _scene("box_stacking").factory()
    world = setup.world
    assert not setup.key_actions and setup.on_activate is None
    assert {label for label, control in controls.items() if control.on_change} == {
        "LCP solver:"
    }
    choice = controls["LCP solver:"]
    assert choice.options == ["Dantzig", "PGS"]
    box = world.getSkeleton("box")
    for index, backend in (
        (0, "DantzigBoxedLcpSolver"),
        (1, "PgsBoxedLcpSolver"),
        (0, "DantzigBoxedLcpSolver"),
    ):
        _edit(choice, index)
        assert world.getConstraintSolver().getBoxedLcpSolver().getType() == backend
        box.setResting(True)
        _edit(choice, index)
        assert box.isResting()
    _edit(choice, 1)
    assert not box.isResting()
    world.setTime(0.1234)
    setup.pre_refresh()
    assert controls["Time: 0.000"].label == "Time: 0.123"
    np.testing.assert_array_equal(setup.camera_home.eye, [12, 12, 9])
    np.testing.assert_array_equal(setup.camera_home.center, [0, 0, 2])
    np.testing.assert_array_equal(setup.camera_home.up, [0, 0, 1])


def test_box_stacking_initial_geometry_and_transforms():
    world = _scene("box_stacking").factory().world
    floor = world.getSkeleton("floor").getBodyNode(0)
    np.testing.assert_array_equal(
        floor.getShapeNode(0).getShape().getSize(), [10, 10, 0.01]
    )
    np.testing.assert_array_equal(
        floor.getWorldTransform().translation(), [0, 0, -0.005]
    )
    np.testing.assert_array_equal(
        floor.getShapeNode(0).getVisualAspect().getRGBA(), [0.9, 0.9, 0.9, 1]
    )
    for index in range(5):
        box = world.getSkeleton(index + 1)
        body = box.getBodyNode(0)
        node = body.getShapeNode(0)
        np.testing.assert_array_equal(node.getShape().getSize(), [1, 1, 0.5])
        np.testing.assert_array_equal(
            body.getWorldTransform().translation(), [0, 0, 0.75 + index * 0.5]
        )
        np.testing.assert_array_equal(box.getPositions(), np.zeros(6))
        np.testing.assert_array_equal(box.getVelocities(), np.zeros(6))
        assert body.isCollidable()
        assert np.all(
            (node.getVisualAspect().getRGBA()[:3] >= 0)
            & (node.getVisualAspect().getRGBA()[:3] <= 1)
        )


@pytest.mark.parametrize("kind", ["dantzig", "pgs"])
def test_boxed_lcp_solver_switch_preserves_simbicon_harness(kind):
    from examples.demos.models import simbicon, soft_foot_simbicon

    model = soft_foot_simbicon.create_model(soft_foot_simbicon.Feet.RIGID)
    controller = simbicon.Controller(model.atlas, model.world)
    controller.harness_pelvis()
    # Switching the boxed backend keeps the controller's solver and harness.
    set_boxed_lcp_solver(model.world, kind)
    controller.unharness_pelvis()
    assert model.world.getConstraintSolver().getNumConstraints() == 0
    for _ in range(4):
        controller.update()
        model.world.step()
    assert model.world.getSimFrames() == 4
    assert np.isfinite(controller.atlas.getPositions()).all()
    assert np.isfinite(controller.atlas.getVelocities()).all()


def test_heightmap_terrain_panel_and_regeneration(controls):
    setup = _scene("heightmap").factory()
    frame = setup.world.getSimpleFrame(0)

    def edit(label, value):
        _edit(controls[label], value)
        setup.pre_refresh()

    assert not setup.key_actions
    shape = frame.getShape()
    assert isinstance(shape, dart.dynamics.HeightmapShapef)
    assert shape.getHeightField().shape == (100, 100)
    assert 0 <= shape.getMinHeight() <= shape.getMaxHeight() <= 0.1
    np.testing.assert_allclose(shape.getScale(), [0.02, 0.02, 1])
    np.testing.assert_array_equal(
        frame.getRelativeTransform().translation(), [-1, 1, 0]
    )
    visual = frame.getVisualAspect()
    _edit(controls["Show##Terrain"], False)
    assert visual.isHidden()
    _edit(controls["Show##Terrain"], True)
    assert not visual.isHidden()
    for label, value, dimensions in (
        ("X Resolution", 15, (100, 15)),
        ("Y Resolution", 10, (10, 15)),
        ("X Resolution", -1, (10, 5)),
        ("Y Resolution", -1, (5, 5)),
    ):
        edit(label, value)
        assert frame.getShape().getHeightField().shape == dimensions
    before = frame.getShape().getHeightField().copy()
    edit("X Resolution", -1)
    np.testing.assert_array_equal(frame.getShape().getHeightField(), before)
    for label, value, scale, origin in (
        ("X Size", 3, [0.6, 0.4, 1], [-1.5, 1, 0]),
        ("Y Size", 4, [0.6, 0.8, 1], [-1.5, 2, 0]),
        ("X Size", -1, [0.02, 0.8, 1], [-0.05, 2, 0]),
        ("Y Size", -1, [0.02, 0.02, 1], [-0.05, 0.05, 0]),
    ):
        edit(label, value)
        np.testing.assert_allclose(frame.getShape().getScale(), scale)
        np.testing.assert_allclose(frame.getRelativeTransform().translation(), origin)
    edit("Z Min", 0.04)
    assert frame.getShape().getMinHeight() >= np.float32(0.04)
    edit("Z Max", 0.12)
    assert frame.getShape().getMaxHeight() <= np.float32(0.12)
    edit("Z Min", 0.2)
    shape = frame.getShape()
    assert (
        np.float32(0.12)
        <= shape.getMinHeight()
        <= shape.getMaxHeight()
        <= np.float32(0.2)
    )
    for label in ("X Size", "Y Size", "Z Min", "Z Max"):
        before = frame.getShape().getHeightField().copy()
        for invalid in (np.inf, np.nan, 1e100):
            edit(label, invalid)
            np.testing.assert_array_equal(frame.getShape().getHeightField(), before)
    _edit(controls["Color##Heightmap"], [0.1, 0.3, 0.5, 0.4])
    np.testing.assert_allclose(visual.getRGBA(), [0.1, 0.3, 0.5, 0.4])
    np.testing.assert_allclose(setup.camera_home.eye, [2.57, 3.14, 1.64])
    np.testing.assert_allclose(setup.camera_home.center, [0, 0, 0.30])
    np.testing.assert_allclose(setup.camera_home.up, [-0.24, -0.25, 0.94])


def test_heightmap_and_shared_grid_inputs_match_cpp_steps(monkeypatch):
    from examples.demos.scenes.common import add_grid_editor

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
    _scene("heightmap").factory()
    grid_inputs = {
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
    assert inputs == {
        "X Resolution": (5, 10, 3, True),
        "Y Resolution": (5, 10, 3, True),
        "X Size": (0.1, 0.2, 3, False),
        "Y Size": (0.1, 0.2, 3, False),
        "Z Min": (0.05, 0.1, 3, False),
        "Z Max": (0.05, 0.1, 3, False),
        **grid_inputs,
    }
    inputs.clear()
    add_grid_editor(viewer.Panel(), dart.gui.GridVisual())
    assert inputs == grid_inputs


@pytest.mark.parametrize(
    "label, getter_name",
    [
        ("Axis Line Width", "getAxisLineWidth"),
        ("Major Line Width", "getMajorLineWidth"),
        ("Minor Line Width", "getMinorLineWidth"),
    ],
)
@pytest.mark.parametrize("invalid", [0, -1])
def test_grid_width_invalid_edits_preserve_host_callbacks(
    controls, label, getter_name, invalid
):
    checked = []

    def build():
        setup = _scene("heightmap").factory()
        control = controls[label]
        getter = None
        previous = None
        step = 0

        def on_activate(ctx):
            nonlocal getter, previous
            setup.on_activate(ctx)
            getter = getattr(ctx.viewer.getAttachments()[-1], getter_name)
            previous = getter()
            control.edit(invalid)

        def pre_step():
            nonlocal step
            expected = previous if step == 0 else previous + 1
            assert control.enabled and control.on_change is not None
            assert control.value == expected
            assert getter() == pytest.approx(expected)
            checked.append(step)
            control.edit(previous + 1)
            step += 1

        return viewer.SceneSetup(
            setup.world,
            panel=setup.panel,
            pre_refresh=setup.pre_refresh,
            pre_step=pre_step,
            on_activate=on_activate,
        )

    assert _cycle(build) == 0
    assert checked == [0, 1] * 2


def test_grid_offsets_use_separate_rows_through_host_dispatch(controls):
    checked = []

    def build():
        setup = _scene("heightmap").factory()
        offsets = [controls[label] for label in ("X", "Y", "Z")]
        grid = None
        step = 0

        def on_activate(ctx):
            nonlocal grid
            setup.on_activate(ctx)
            grid = ctx.viewer.getAttachments()[-1]
            for control, value in zip(offsets, [0.25, 0.5, 0.75]):
                control.edit(value)

        def pre_step():
            nonlocal step
            sign = 1 if step == 0 else -1
            np.testing.assert_allclose(
                grid.getOffset(), sign * np.array([0.25, 0.5, 0.75])
            )
            assert all(control.visible and not control.same_line for control in offsets)
            checked.append(step)
            for control, value in zip(offsets, [-0.25, -0.5, -0.75]):
                control.edit(value)
            step += 1

        return viewer.SceneSetup(
            setup.world,
            panel=setup.panel,
            pre_refresh=setup.pre_refresh,
            pre_step=pre_step,
            on_activate=on_activate,
        )

    assert _cycle(build) == 0
    assert checked == [0, 1] * 2


def test_heightmap_overflowing_span_recovers_through_host_dispatch(controls):
    checked = []

    def build():
        setup = _scene("heightmap").factory()
        terrain = setup.world.getSimpleFrame(0)
        original = terrain.getShape()
        z_min, z_max = controls["Z Min"], controls["Z Max"]
        previous = (z_min.value, z_max.value)
        step = 0

        def on_activate(ctx):
            setup.on_activate(ctx)
            z_min.edit(-3e38)
            z_max.edit(3e38)

        def pre_step():
            nonlocal step
            shape = terrain.getShape()
            if step == 0:
                assert shape is original
                assert (z_min.value, z_max.value) == previous
                z_min.edit(0.02)
                z_max.edit(0.04)
            else:
                assert shape is not original
                assert (
                    np.float32(0.02)
                    <= shape.getMinHeight()
                    <= shape.getMaxHeight()
                    <= np.float32(0.04)
                )
            checked.append(step)
            step += 1

        return viewer.SceneSetup(
            setup.world,
            panel=setup.panel,
            pre_refresh=setup.pre_refresh,
            pre_step=pre_step,
            on_activate=on_activate,
        )

    assert _cycle(build) == 0
    assert checked == [0, 1] * 2


def test_heightmap_grid_attachment_and_every_grid_control(controls):
    setup = _scene("heightmap").factory()

    class Context:
        def __init__(self):
            self.attachments = []

        def add_attachment(self, attachment):
            self.attachments.append(attachment)

    ctx = Context()
    setup.on_activate(ctx)
    (grid,) = ctx.attachments
    assert isinstance(grid, dart.gui.GridVisual)
    expected = {
        "Show##Terrain",
        "X Resolution",
        "Y Resolution",
        "X Size",
        "Y Size",
        "Z Min",
        "Z Max",
        "Color##Heightmap",
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
    assert {
        label for label, control in controls.items() if control.on_change
    } == expected
    _edit(controls["Show##Grid"], False)
    assert not grid.isDisplayed()
    assert not controls["Plane"].visible
    _edit(controls["Show##Grid"], True)
    assert grid.isDisplayed()
    assert controls["Plane"].visible
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
        before = getter()
        _edit(controls[label], -1)
        assert getter() == before
        assert controls[label].value == before
    for label, getter in (
        ("Major Line Color", grid.getMajorLineColor),
        ("Minor Line Color", grid.getMinorLineColor),
    ):
        alpha = getter()[3]
        _edit(controls[label], [0.1, 0.3, 0.5, 0.2])
        np.testing.assert_allclose(getter(), [0.1, 0.3, 0.5, alpha])
