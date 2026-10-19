"""GUI-free squat replay, contact results, histories, controls and ownership."""

import gc
import sys
import weakref
from pathlib import Path
from types import SimpleNamespace

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models.contact_inverse_dynamics import ContactInverseDynamicsModel
from examples.demos.scenes.common import reorient_z_up


def _world():
    world = dart.utils.SkelParser.readWorld("dart://sample/skel/fullbody1.skel")
    reorient_z_up(world)
    return world


@pytest.fixture
def model():
    return ContactInverseDynamicsModel(_world())


def test_replay_is_deterministic_and_wraps_histories(model):
    second = ContactInverseDynamicsModel(_world())
    assert model.keyframe_count == 60
    assert model.world.getNumSimpleFrames() == 4
    assert model.biped.getNumBodyNodes() == 20
    assert model.biped.getNumDofs() == 37
    np.testing.assert_array_equal(model.biped.getPositions(), model._initial_positions)
    assert model.torque_names == ["j_thigh_left_z", "j_shin_left", "j_heel_left_1"]
    assert model._solver.getRegularization() == 1e-4
    assert model._solver.getResidualTolerance() == 1e-4
    for _ in range(241):
        model.update_frame()
        second.update_frame()
        for getter in ("getPositions", "getVelocities", "getAccelerations"):
            first = getattr(model.biped, getter)()
            assert np.isfinite(first).all()
            np.testing.assert_array_equal(first, getattr(second.biped, getter)())
        assert model.feasible == second.feasible
        assert model.residual_norm == second.residual_norm
        assert model.total_vertical_force == second.total_vertical_force
        assert len(model.contact_forces) == 4
        assert np.isfinite(model.contact_forces).all()
        np.testing.assert_array_equal(model.contact_forces, second.contact_forces)
    assert model.sample_index == 1
    assert len(model.plot_samples) == 240
    assert model.plot_samples == second.plot_samples
    assert model.torque_plots == second.torque_plots
    assert model.plot_samples[0] == float(np.float32(model.total_vertical_force))


def test_pausing_regeneration_and_arrow_scaling(model):
    model.playback_speed = 2.0
    model.update_frame()
    assert model.time == 1 / 30
    model.playing = False
    model._time = 1.5
    model.update_frame()
    squat = model.biped.getPositions().copy()
    assert model.time == 1.5
    np.testing.assert_array_equal(model.biped.getVelocities(), np.zeros(37))
    np.testing.assert_array_equal(model.biped.getAccelerations(), np.zeros(37))
    model.squat_depth = 0.0
    model.request_regeneration()
    assert model.regeneration_requested
    np.testing.assert_array_equal(model.biped.getPositions(), squat)
    model.update_frame()
    assert not model.regeneration_requested
    assert model.keyframe_count == 60
    assert np.linalg.norm(model.biped.getPositions() - squat) > 0.01
    model.friction_coeff = 0.3
    model.force_scale = 0.004
    model.update_frame()
    contacts = model._solver.getContacts()
    assert [contact.bodyNode.getName() for contact in contacts] == [
        "h_heel_left",
        "h_toe_left",
        "h_heel_right",
        "h_toe_right",
    ]
    for index, contact in enumerate(contacts):
        assert contact.frictionCoeff == float(np.float32(0.3))
        assert contact.numBasis == 8
        np.testing.assert_array_equal(contact.normal, [0, 0, 1])
        np.testing.assert_array_equal(contact.localOffset, [0, 0, 0])
        frame = model.world.getSimpleFrame(index)
        force = model.contact_forces[index]
        if np.linalg.norm(force) < 1e-8:
            assert frame.getVisualAspect().isHidden()
            continue
        assert not frame.getVisualAspect().isHidden()
        tail = contact.bodyNode.getWorldTransform().multiply(contact.localOffset)
        np.testing.assert_array_equal(frame.getShape().getTail(), tail)
        np.testing.assert_array_equal(
            frame.getShape().getHead(), tail + model.force_scale * force
        )
        properties = frame.getShape().getProperties()
        assert (
            properties.mRadius,
            properties.mHeadRadiusScale,
            properties.mHeadLengthScale,
        ) == (0.01, 2.0, 0.15)


def test_spline_knots_and_analytic_derivatives(model):
    for index, keyframe in enumerate(model._keyframes):
        q, _, _ = model._spline(index / 60)
        np.testing.assert_allclose(q, keyframe, atol=1e-14, rtol=0)
    phase, epsilon = 0.273, 1e-6
    q, velocity, acceleration = model._spline(phase)
    previous = model._spline(phase - epsilon)
    following = model._spline(phase + epsilon)
    np.testing.assert_allclose(
        velocity, (following[0] - previous[0]) / (2 * epsilon), atol=1e-6, rtol=0
    )
    np.testing.assert_allclose(
        acceleration, (following[1] - previous[1]) / (2 * epsilon), atol=1e-6, rtol=0
    )
    np.testing.assert_array_equal(model._spline(0)[0], model._spline(1)[0])
    model._keyframes = []
    q, velocity, acceleration = model._spline(phase)
    np.testing.assert_array_equal(q, model.biped.getPositions())
    assert not np.any(velocity) and not np.any(acceleration)


def test_hidden_arrows_and_feasibility_colors(model, monkeypatch):
    colors = []
    notify = dart.dynamics.ArrowShape.notifyColorUpdated

    def record(arrow, color):
        colors.append((arrow, list(color)))
        notify(arrow, color)

    monkeypatch.setattr(dart.dynamics.ArrowShape, "notifyColorUpdated", record)
    result = SimpleNamespace(
        feasible=False,
        unactuatedResidual=np.array([-2.0, 1.0, 0.0]),
        jointForces=np.arange(37.0),
        contactForces=[
            np.zeros(3),
            np.array([0.0, 0.0, 1e-9]),
            np.array([0.0, 0.0, 0.5]),
            np.array([1.0, 2.0, 3.0]),
        ],
    )

    class Solver:
        def setContacts(self, contacts):
            self.contacts = contacts

        def compute(self):
            return result

    model._solver = Solver()
    model.update_frame()
    assert model.residual_norm == 2
    assert [
        model.world.getSimpleFrame(i).getVisualAspect().isHidden() for i in range(4)
    ] == [True, True, False, False]
    assert [arrow for arrow, _ in colors] == model._arrows[2:]
    assert all(color == [1, 0, 0, 1] for _, color in colors)
    result.feasible = True
    model.update_frame()
    assert all(color == [0, 1, 0, 1] for _, color in colors[2:])


def test_status_arrays_and_lists_are_independent_copies(model):
    model.update_frame()
    for getter in ("contact_forces", "torque_plots", "plot_samples", "torque_names"):
        value = getattr(model, getter)
        expected = getattr(model, getter)
        if getter == "contact_forces":
            value[0][:] = 123
        elif getter == "torque_plots":
            value[0][0] = 123
        else:
            value[0] = "changed" if getter == "torque_names" else 123
        np.testing.assert_array_equal(getattr(model, getter), expected)
    forces = model.contact_forces
    expected = [force.copy() for force in forces]
    model.update_frame()
    np.testing.assert_array_equal(forces, expected)
    del model
    gc.collect()
    np.testing.assert_array_equal(forces, expected)


def test_model_retains_world_and_collects_cycles():
    class World(dart.simulation.World):
        pass

    parsed = _world()
    world = World()
    world.setGravity(parsed.getGravity())
    for index in range(parsed.getNumSkeletons()):
        world.addSkeleton(parsed.getSkeleton(index))
    del parsed
    model = ContactInverseDynamicsModel(world)
    reference = weakref.ref(world)
    del world
    gc.collect()
    assert reference() is model.world
    world = model.world
    world.model = model
    refs = weakref.ref(world), weakref.ref(model)
    del world, model
    gc.collect()
    assert all(ref() is None for ref in refs)


def test_invalid_world_and_missing_foot_nodes():
    with pytest.raises(ValueError, match="must not be None"):
        ContactInverseDynamicsModel(None)
    with pytest.raises(TypeError, match="must be a World"):
        ContactInverseDynamicsModel("world")
    with pytest.raises(RuntimeError, match="missing skeleton"):
        ContactInverseDynamicsModel(dart.simulation.World())
    world = _world()
    world.getSkeleton("fullbody1").getBodyNode("h_heel_left").setName("missing")
    with pytest.raises(RuntimeError, match="missing one or more foot"):
        ContactInverseDynamicsModel(world)


@pytest.mark.parametrize("gravity", ([0, 0, 0], [0, 0, np.nan], [0, 0, np.inf]))
def test_invalid_gravity(gravity):
    world = _world()
    world.setGravity(gravity)
    with pytest.raises(ValueError, match="gravity must be finite and nonzero"):
        ContactInverseDynamicsModel(world)


@pytest.mark.parametrize(
    "name, lower, upper",
    [
        ("playback_speed", 0.1, 2.0),
        ("squat_depth", 0.0, 0.25),
        ("friction_coeff", 0.0, 1.5),
        ("force_scale", 0.0001, 0.01),
    ],
)
def test_controls_reject_nonfinite_and_overflow_then_clamp(model, name, lower, upper):
    initial = getattr(model, name)
    for value in (np.nan, np.inf, -np.inf, 1e40):
        with pytest.raises(ValueError, match="finite"):
            setattr(model, name, value)
        assert getattr(model, name) == initial
    for value in ("0.5", None, object(), 10**400):
        with pytest.raises(TypeError):
            setattr(model, name, value)
        assert getattr(model, name) == initial
    setattr(model, name, -1.0)
    assert getattr(model, name) == float(np.float32(lower))
    setattr(model, name, 9.0)
    assert getattr(model, name) == float(np.float32(upper))


@pytest.mark.parametrize("playing", (1, None, np.bool_(True)))
def test_playing_requires_a_python_bool(model, playing):
    with pytest.raises(TypeError, match="must be a bool"):
        model.playing = playing
    assert model.playing is True
