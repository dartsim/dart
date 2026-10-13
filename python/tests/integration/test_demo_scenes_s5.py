"""Construction, deterministic stepping, and controls for the fifth demo batch."""

import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "python"))
viewer = getattr(dart, "viewer", None)
pytestmark = pytest.mark.skipif(viewer is None, reason="demo app host unavailable")

SCENE_IDS = (
    "atlas_simbicon",
    "adaptive_soft_contact",
    "soft_worm",
    "soft_foot_simbicon",
    "soft_bodies",
    "soft_cubes",
    "soft_open_chain",
)
PLAYBACK_IDS = SCENE_IDS[-3:]


def _scene(scene_id):
    from examples.demos.registry import make_demo_scenes

    return next(scene for scene in make_demo_scenes() if scene.id == scene_id)


def _soft_bodies(skeleton):
    return [
        body
        for body in skeleton.getBodyNodes()
        if isinstance(body, dart.dynamics.SoftBodyNode)
    ]


def _structure(world):
    return [
        (
            skeleton.getName(),
            skeleton.getNumBodyNodes(),
            skeleton.getNumSoftBodyNodes(),
            sum(body.getNumPointMasses() for body in _soft_bodies(skeleton)),
            skeleton.getNumDofs(),
        )
        for skeleton in (world.getSkeleton(i) for i in range(world.getNumSkeletons()))
    ]


def _state(world):
    values = []
    for i in range(world.getNumSkeletons()):
        skeleton = world.getSkeleton(i)
        values.extend((skeleton.getPositions(), skeleton.getVelocities()))
        for body in _soft_bodies(skeleton):
            values.extend((body.getCOM(), body.getCOMLinearVelocity()))
        if skeleton.getName() == "soft_worm":
            values.append([dart._demos.soft_worm.position_checksum(skeleton)])
    for i in range(world.getNumSimpleFrames()):
        values.append(world.getSimpleFrame(i).getRelativeTransform().translation())
    return np.concatenate(values).copy()


def _step(setup, count=1):
    for _ in range(count):
        if setup.pre_refresh:
            setup.pre_refresh()
        if setup.pre_step:
            setup.pre_step()
        setup.world.step()
        if setup.post_step:
            setup.post_step()


def _actions(setup):
    return {chr(action.key): action for action in setup.key_actions}


def _edit(control, value=None):
    control.edit(value)
    if control.on_change:
        control.on_change()


@pytest.fixture
def controls(monkeypatch):
    recorded = []
    for name in ("add_text", "add_slider", "add_checkbox", "add_button", "add_choice"):
        original = getattr(viewer.Control, name)

        def record(self, *args, _original=original, **kwargs):
            control = _original(self, *args, **kwargs)
            recorded.append(control)
            return control

        monkeypatch.setattr(viewer.Control, name, record)
    return recorded


def _control(controls, label):
    return next(control for control in controls if control.label == label)


class _Context:
    def __init__(self, world):
        self.viewer = dart.gui.Viewer()
        self.world_node = dart.gui.RealTimeWorldNode(world)
        self.viewer.addWorldNode(self.world_node)
        self.teardowns = []
        self.handlers = []
        self.attachments = []

    def add_teardown(self, callback):
        self.teardowns.append(callback)

    def add_event_handler(self, callback):
        self.handlers.append(callback)

    def add_attachment(self, attachment):
        self.attachments.append(attachment)


def test_s5_registry_order():
    from examples.demos.registry import make_demo_scenes

    ids = [scene.id for scene in make_demo_scenes()]
    assert [scene_id for scene_id in ids if scene_id in SCENE_IDS] == list(SCENE_IDS)
    assert (
        ids.index("atlas_puppet")
        < ids.index("atlas_simbicon")
        < ids.index("mixed_chain")
    )
    assert ids.index("soft_open_chain") < ids.index("vehicle")


@pytest.mark.parametrize("scene_id", SCENE_IDS)
def test_s5_construction_and_deterministic_steps(scene_id, monkeypatch):
    for name in (
        "DART_DEMO_SOFT_FOOT_FEET",
        "DART_DEMO_SOFT_FOOT_PUSH_STEP",
        "DART_DEMO_SOFT_FOOT_PUSH_N",
    ):
        monkeypatch.delenv(name, raising=False)
    states = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        setup = _scene(scene_id).factory()
        assert _structure(setup.world) == STRUCTURES[scene_id]
        assert setup.allow_simulation
        assert setup.world.getConstraintSolver().getNumConstraints() == 0
        _step(setup, 4)
        assert setup.world.getSimFrames() == 4
        states.append(
            dart._demos.soft_foot_simbicon.state_vector(
                setup.pre_step.__self__.model
            ).copy()
            if scene_id == "soft_foot_simbicon"
            else _state(setup.world)
        )
    assert np.isfinite(states[0]).all()
    np.testing.assert_array_equal(*states)


@pytest.mark.parametrize("scene_id", PLAYBACK_IDS)
def test_soft_playback_keys_and_display(scene_id, controls):
    setup = _scene(scene_id).factory()
    world = setup.world
    ctx = _Context(world)
    setup.on_activate(ctx)
    initial = _state(world)
    _step(setup, 12)
    latest = _state(world)
    actions = _actions(setup)
    assert {key: action.label for key, action in actions.items()} == {
        "[": "Step back 1 frame",
        "]": "Step forward 1 frame",
        "{": "Step back 10 frames",
        "}": "Step forward 10 frames",
        "r": "Restart (frame 0)",
        "\\": "Jump to latest frame",
    }
    for key, index in (("[", 11), ("{", 1), ("]", 2), ("}", 12), ("r", 0), ("\\", 12)):
        ctx.viewer.simulate(True)
        actions[key].callback()
        assert not ctx.viewer.isSimulating()
        setup.pre_refresh()
        assert any(c.label == f"Playback frame: {index} / 12" for c in controls)
        if index == 0:
            np.testing.assert_array_equal(_state(world), initial)
        if index == 12:
            np.testing.assert_array_equal(_state(world), latest)
    actions["r"].callback()
    _step(setup)
    setup.pre_refresh()
    assert any(c.label == "Playback frame: 1 / 1" for c in controls)
    assert any(c.label == "Frames recorded (lifetime): 14" for c in controls)
    _edit(_control(controls, ""), 0.35)
    _edit(_control(controls, "Embedded visuals"), False)
    for si in range(world.getNumSkeletons()):
        for body in _soft_bodies(world.getSkeleton(si)):
            for i in range(body.getNumShapeNodes()):
                node = body.getShapeNode(i)
                visual = node.getVisualAspect(False)
                if visual is None:
                    continue
                if isinstance(node.getShape(), dart.dynamics.SoftMeshShape):
                    assert visual.getRGBA()[3] == float(np.float32(0.35))
                    assert not visual.isHidden()
                else:
                    assert visual.isHidden()
    _edit(_control(controls, "Embedded visuals"), True)
    for si in range(world.getNumSkeletons()):
        for body in _soft_bodies(world.getSkeleton(si)):
            for node in body.getShapeNodes():
                visual = node.getVisualAspect(False)
                if visual is not None:
                    assert not visual.isHidden()
    np.testing.assert_array_equal(world.getGravity(), [0, 0, -9.81])


def test_adaptive_controls_and_native_markers(controls, monkeypatch):
    native = dart._demos.adaptive_soft_contact
    models = []
    settings = []
    create_model, configure = native.create_model, native.configure

    def create(*args, **kwargs):
        model = create_model(*args, **kwargs)
        models.append(model)
        return model

    def apply(model, **kwargs):
        settings.append(kwargs)
        configure(model, **kwargs)

    monkeypatch.setattr(native, "create_model", create)
    monkeypatch.setattr(native, "configure", apply)
    setup = _scene("adaptive_soft_contact").factory()
    model = models[0]
    checkbox = _control(controls, "Adaptive activation")
    action = _actions(setup)["a"]
    assert action.label == "Toggle adaptive activation"
    action.callback()
    assert not checkbox.value
    setup.pre_refresh()
    assert native.compute_checksum(model).active == model.soft_body.getNumPointMasses()
    _edit(checkbox, True)
    _edit(_control(controls, "Contact rings"), 3)
    _edit(_control(controls, "Linger steps"), 27)
    _step(setup, 3)
    setup.pre_refresh()
    assert settings[-1] == {
        "adaptive_enabled": True,
        "ring_count": 3,
        "linger_steps": 27,
    }
    positions, flags = native.point_positions(model), native.active_flags(model)
    assert setup.world.getNumSimpleFrames() == len(flags)
    for i, (position, active) in enumerate(zip(positions, flags)):
        marker = setup.world.getSimpleFrame(i)
        assert marker.getName() == f"activation_marker_{i}"
        np.testing.assert_array_equal(
            marker.getRelativeTransform().translation(), position
        )
        np.testing.assert_array_equal(
            marker.getVisualAspect().getRGBA(),
            [1.0, 0.78, 0.08, 1.0] if active else [0.08, 0.18, 0.48, 0.45],
        )
    assert any(c.label == "Simulation step:       3" for c in controls)
    assert not _control(controls, "State is non-finite").visible
    last_positions = [
        setup.world.getSimpleFrame(i).getRelativeTransform().translation().copy()
        for i in range(len(flags))
    ]
    model.soft_skeleton.setPosition(3, float("nan"))
    invalid_positions = native.point_positions(model)
    invalid_positions[0] = np.nan
    monkeypatch.setattr(native, "point_positions", lambda _model: invalid_positions)
    setup.post_step()
    setup.pre_refresh()
    assert _control(controls, "State is non-finite").visible
    for i, position in enumerate(invalid_positions):
        np.testing.assert_array_equal(
            setup.world.getSimpleFrame(i).getRelativeTransform().translation(),
            last_positions[i] if i == 0 else position,
        )


def test_worm_gait_key_panel_and_nonfinite_guard(controls):
    setup = _scene("soft_worm").factory()
    worm = setup.world.getSkeleton("soft_worm")
    gait = _control(controls, "Traveling-wave gait")
    action = _actions(setup)["g"]
    assert action.label == "Toggle traveling-wave gait"
    setup.pre_step()
    assert np.linalg.norm(worm.getCommands()[6:]) > 0
    action.callback()
    setup.pre_step()
    assert not gait.value
    np.testing.assert_array_equal(worm.getCommands()[6:], np.zeros(4))
    _edit(gait, True)
    _step(setup, 3)
    setup.pre_step()
    assert np.linalg.norm(worm.getCommands()[6:]) > 0
    setup.pre_refresh()
    assert any(c.label.startswith("Frame: 3   contacts:") for c in controls)
    assert any(c.label.startswith("Position checksum:") for c in controls)
    worm.setPosition(6, float("nan"))
    setup.post_step()
    setup.pre_refresh()
    assert not gait.value
    assert _control(controls, "State is non-finite").visible
    setup.pre_step()
    np.testing.assert_array_equal(worm.getCommands()[6:], np.zeros(4))


def test_atlas_keys_and_every_panel_control(controls):
    setup = _scene("atlas_simbicon").factory()
    state = setup.pre_step.__self__
    initial = state.controller.atlas.getPositions().copy()
    actions = _actions(setup)
    assert {key: action.label for key, action in actions.items()} == {
        "a": "Push forward",
        "s": "Push backward",
        "d": "Push left",
        "f": "Push right",
        "r": "Reset Atlas",
    }
    for key, axis, sign in (("a", 0, 1), ("s", 0, -1), ("d", 2, 1), ("f", 2, -1)):
        state.external_force[:] = 0
        state.pelvis.clearExternalForces()
        actions[key].callback()
        setup.pre_step()
        expected = np.zeros(3)
        expected[axis] = sign * 500
        np.testing.assert_array_equal(state.external_force, expected)
        np.testing.assert_allclose(
            state.pelvis.getExternalForceGlobal()[3:], expected, atol=1e-10
        )
        assert state.force_duration == 99
    actions["a"].callback()
    actions["d"].callback()
    np.testing.assert_array_equal(state.external_force, [500, 0, 500])
    for reset in (
        actions["r"].callback,
        lambda: _edit(_control(controls, "Reset Atlas")),
    ):
        state.controller.atlas.setPosition(7, 0.7)
        reset()
        np.testing.assert_array_equal(state.controller.atlas.getPositions(), initial)
        np.testing.assert_array_equal(state.external_force, np.zeros(3))
    _edit(_control(controls, "Gravity Acc."), 12.25)
    np.testing.assert_array_equal(setup.world.getGravity(), [0, -12.25, 0])
    setup.pre_refresh()
    assert any(c.label == "Gravity Acc.: -12.25 m/s^2" for c in controls)
    mode = next(c for c in controls if c.kind == viewer.Control.Kind.CHOICE)
    assert mode.options == [
        "No Control",
        "Short-Stride Walking",
        "Normal-Stride Walking",
    ]
    for index, gait in enumerate(("standing", "running", "walking")):
        _edit(mode, index)
        setup.pre_refresh()
        assert state.controller.current_state_machine == gait
    ctx = _Context(setup.world)
    prior_mode = ctx.viewer.getCameraMode()
    prior_headlights = ctx.viewer.checkHeadlights()
    setup.on_activate(ctx)
    assert ctx.world_node.isShadowed()
    for value in (False, True):
        _edit(_control(controls, "Shadow On/Off"), value)
        assert ctx.world_node.isShadowed() == value
        _edit(_control(controls, "Headlights On/Off"), value)
        assert ctx.viewer.checkHeadlights() == value
        _edit(_control(controls, "Depth mode"), value)
        assert ctx.viewer.getCameraMode() == (
            dart.gui.CameraMode.DEPTH if value else dart.gui.CameraMode.RGBA
        )
    for teardown in reversed(ctx.teardowns):
        teardown()
    assert ctx.viewer.getCameraMode() == prior_mode
    assert ctx.viewer.checkHeadlights() == prior_headlights
    assert not setup.shadows


def test_soft_foot_keys_restart_and_deferred_feet(controls, monkeypatch):
    monkeypatch.delenv("DART_DEMO_SOFT_FOOT_FEET", raising=False)
    scene = _scene("soft_foot_simbicon")
    setup = scene.factory()
    state = setup.pre_step.__self__
    native = dart._demos.soft_foot_simbicon
    initial = native.state_vector(state.model).copy()
    actions = _actions(setup)
    assert {key: action.label for key, action in actions.items()} == {
        "a": "Push forward (+X)",
        "s": "Push backward (-X)",
        "d": "Push left (+Z)",
        "f": "Push right (-Z)",
        "r": "Restart biped (keeps current feet)",
        "t": "Toggle feet (applies on host Reset/Rebuild)",
    }
    for key, direction in (
        ("a", [1, 0, 0]),
        ("s", [-1, 0, 0]),
        ("d", [0, 0, 1]),
        ("f", [0, 0, -1]),
    ):
        actions[key].callback()
        np.testing.assert_array_equal(
            state.model.external_force,
            np.array(direction) * native.default_push_magnitude,
        )
        assert state.model.force_duration == native.push_steps
        _step(setup)
        assert state.model.force_duration == native.push_steps - 1
    actions["t"].callback()
    setup.pre_refresh()
    assert state.model.feet == native.Feet.SOFT
    assert state.pending_text.visible and "Rigid" in state.pending_text.label
    actions["r"].callback()
    assert state.model.feet == native.Feet.SOFT
    np.testing.assert_array_equal(native.state_vector(state.model), initial)
    assert state.model.force_duration == 0
    rebuilt = scene.factory()
    assert rebuilt.pre_step.__self__.model.feet == native.Feet.RIGID
    _actions(rebuilt)["t"].callback()
    assert scene.factory().pre_step.__self__.model.feet == native.Feet.SOFT


@pytest.mark.parametrize("text, expected", [(" +12tail", 12), ("-.5", 0), ("bad", 0)])
def test_soft_foot_atoi(text, expected):
    from examples.demos.scenes.soft_foot_simbicon import _atoi

    assert _atoi(text) == expected


@pytest.mark.parametrize(
    "text, expected",
    [(" 500.5tail", 500.5), ("1e+", 1), ("0x1.8p+2tail", 6), ("bad", 0)],
)
def test_soft_foot_atof(text, expected):
    from examples.demos.scenes.soft_foot_simbicon import _atof

    assert _atof(text) == expected


def test_soft_foot_environment_scripted_push(monkeypatch):
    native = dart._demos.soft_foot_simbicon
    monkeypatch.setenv("DART_DEMO_SOFT_FOOT_FEET", "rigid")
    monkeypatch.setenv("DART_DEMO_SOFT_FOOT_PUSH_STEP", " 2steps")
    monkeypatch.setenv("DART_DEMO_SOFT_FOOT_PUSH_N", "125.5N")
    setup = _scene("soft_foot_simbicon").factory()
    model = setup.pre_step.__self__.model
    assert model.feet == native.Feet.RIGID
    _step(setup, 2)
    assert model.force_duration == 0
    _step(setup)
    np.testing.assert_array_equal(model.external_force, [0, 0, 125.5])
    assert model.force_duration == native.push_steps - 1


# Scene-port parity baseline: (name, bodies, soft bodies, point masses, DOFs).
STRUCTURES = {
    "atlas_simbicon": [("ground_skeleton", 1, 0, 0, 0), ("drc_skeleton", 28, 0, 0, 33)],
    "adaptive_soft_contact": [
        ("ground", 1, 0, 0, 0),
        ("adaptive_ellipsoid", 1, 1, 86, 6),
        ("periodic_pusher", 1, 0, 0, 6),
    ],
    "soft_worm": [("ground", 1, 0, 0, 0), ("soft_worm", 5, 5, 130, 10)],
    "soft_foot_simbicon": [("ground_skeleton", 1, 0, 0, 0), ("drc_skeleton", 28, 2, 52, 33)],
    "soft_bodies": [
        ("ground skeleton", 1, 0, 0, 0),
        ("skeleton 1", 1, 1, 26, 6),
        ("skeleton 2", 1, 1, 58, 6),
        ("skeleton 3", 1, 1, 58, 6),
        ("skeleton 4", 1, 1, 122, 6),
        ("skeleton 5", 1, 1, 26, 6),
    ],
    "soft_cubes": [
        ("ground skeleton", 1, 0, 0, 0),
        ("skeleton 1", 1, 1, 26, 6),
        ("skeleton 2", 1, 1, 26, 6),
    ],
    "soft_open_chain": [("ground skeleton", 1, 0, 0, 0), ("skeleton 1", 5, 5, 130, 15)],
}
