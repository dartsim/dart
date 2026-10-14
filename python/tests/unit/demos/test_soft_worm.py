"""The Python soft-worm model preserves construction, finite state, and locomotion."""

import gc
import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models import soft_worm as worm_model


def _points(worm):
    return [
        body.get_point_mass(index)
        for body in worm.getBodyNodes()
        if isinstance(body, dart.dynamics.SoftBodyNode)
        for index in range(body.getNumPointMasses())
    ]


def test_soft_worm_construction_materials_and_initial_checksum():
    world, worm = worm_model.create_world()
    assert world.getName() == "soft_worm_world"
    assert world.getNumSkeletons() == 2
    assert world.getTimeStep() == 0.001
    np.testing.assert_array_equal(world.getGravity(), [0, 0, -9.81])
    ground = world.getSkeleton("ground")
    assert not ground.isMobile()
    np.testing.assert_array_equal(
        ground.getBodyNode(0).getShapeNode(0).getShape().getSize(), [14, 5, 0.08]
    )
    np.testing.assert_array_equal(
        ground.getBodyNode(0).getWorldTransform().translation(), [0, 0, -0.04]
    )
    assert (
        worm.getNumBodyNodes()
        == worm.getNumSoftBodyNodes()
        == worm_model.num_links
        == 5
    )
    assert worm.getNumDofs() == 10
    assert worm_model.gait_amplitude == 0.56
    assert worm_model.gait_frequency_hz == 1.35
    assert world.getNumSimpleFrames() == 0
    assert len(_points(worm)) == 130
    assert worm_model.position_checksum(worm) == pytest.approx(
        43392.44, rel=0, abs=1e-10
    )
    assert worm_model.root_x(worm) == 0.0
    colors = (
        [0.18, 0.66, 0.83, 0.88],
        [0.24, 0.77, 0.63, 0.88],
        [0.78, 0.82, 0.31, 0.88],
        [0.95, 0.62, 0.25, 0.88],
        [0.88, 0.35, 0.42, 0.88],
    )
    for index, body in enumerate(worm.getBodyNodes()):
        assert body.getName() == f"soft_link_{index}"
        assert body.getMass() == pytest.approx(0.70)
        assert body.getInertia().getMass() == 0.16
        assert body.getNumPointMasses() == 26
        assert body.getNumMarkers() == 0
        assert body.getVertexSpringStiffness() == 850.0
        assert body.getEdgeSpringStiffness() == 120.0
        assert body.getDampingCoefficient() == 8.0
        shape = body.getShapeNode(0)
        assert shape.getShape().getType() == "SoftMeshShape"
        np.testing.assert_array_equal(shape.getVisualAspect().getRGBA(), colors[index])
        assert shape.getDynamicsAspect().getFrictionCoeff() == 1.25
        assert shape.getDynamicsAspect().getRestitutionCoeff() == 0.0
        joint = body.getParentJoint()
        if index == 0:
            assert joint.getName() == "root_free_joint"
            assert joint.getActuatorType() == dart.dynamics.ActuatorType.PASSIVE
        else:
            assert joint.getName() == f"spine_joint_{index}"
            assert joint.getActuatorType() == dart.dynamics.ActuatorType.SERVO
            assert joint.getPositionLowerLimit(0) == -0.92
            assert joint.getPositionUpperLimit(0) == 0.92
            assert joint.getVelocityLowerLimit(0) == -4.0
            assert joint.getVelocityUpperLimit(0) == 4.0


def test_soft_worm_gait_clamps_joint_speeds_and_pauses_without_moving_pose():
    _, worm = worm_model.create_world()
    for position, command in ((-2.0, 4.0), (2.0, -4.0)):
        worm.setPositions([6, 7, 8, 9], [position] * 4)
        worm_model.apply_gait(worm, 0.0, True)
        np.testing.assert_array_equal(worm.getCommands()[6:], [command] * 4)
        pose = worm.getPositions().copy()
        worm_model.apply_gait(worm, 0.0, False)
        np.testing.assert_array_equal(worm.getCommands()[6:], np.zeros(4))
        np.testing.assert_array_equal(worm.getPositions(), pose)


@pytest.mark.parametrize("enabled", [1, np.bool_(True), np.array(True), None, "yes"])
def test_soft_worm_gait_requires_a_python_bool(enabled):
    _, worm = worm_model.create_world()
    with pytest.raises(TypeError, match="enabled must be"):
        worm_model.apply_gait(worm, 0.0, enabled)
    np.testing.assert_array_equal(worm.getCommands(), np.zeros(10))


@pytest.mark.parametrize(
    "time",
    [None, "0.1", b"0.1", bytearray(b"0.1"), memoryview(b"0.1"), object(), 10**500],
)
def test_soft_worm_gait_validates_time_even_when_disabled(time):
    _, worm = worm_model.create_world()
    with pytest.raises(TypeError, match="time must be"):
        worm_model.apply_gait(worm, time, False)
    np.testing.assert_array_equal(worm.getCommands(), np.zeros(10))


@pytest.mark.parametrize("time", [True, np.float32(0.123), np.uint64(1)])
def test_soft_worm_gait_uses_double_precision_for_numeric_time(time):
    _, worm = worm_model.create_world()
    worm_model.apply_gait(worm, float(time), True)
    commands = worm.getCommands().copy()
    worm_model.apply_gait(worm, time, True)
    np.testing.assert_array_equal(worm.getCommands(), commands)


@pytest.mark.parametrize("time", [np.inf, -np.inf, np.nan, 1e308])
def test_soft_worm_gait_nonfinite_phase_preserves_the_joint_command_guard(time):
    _, worm = worm_model.create_world()
    worm_model.apply_gait(worm, 0.123, True)
    commands = worm.getCommands().copy()
    worm_model.apply_gait(worm, time, True)
    np.testing.assert_array_equal(worm.getCommands(), commands)
    assert worm_model.is_finite(worm)
    worm_model.apply_gait(worm, time, False)
    np.testing.assert_array_equal(worm.getCommands(), np.zeros(10))


def test_soft_worm_repeats_original_3000_step_locomotion_gate():
    outcomes = []
    states = []
    for _ in range(2):
        world, worm = worm_model.create_world()
        world.setNumSimulationThreads(1)
        initial_x = worm_model.root_x(worm)
        for step in range(1, 3001):
            worm_model.apply_gait(worm, world.getTime(), True)
            world.step()
            assert worm_model.is_finite(worm), f"non-finite state at step {step}"
        displacement = worm_model.root_x(worm) - initial_x
        assert displacement > 0.2
        outcomes.append((displacement, worm_model.position_checksum(worm)))
        states.append(
            np.r_[
                worm.getPositions(),
                worm.getVelocities(),
                np.array(
                    [
                        [
                            point.positions,
                            point.velocities,
                            point.world_position,
                            point.forces,
                        ]
                        for point in _points(worm)
                    ]
                ).ravel(),
            ]
        )
    assert outcomes[0] == outcomes[1]
    np.testing.assert_array_equal(*states)


def test_soft_worm_disabled_gait_steps_finitely():
    world, worm = worm_model.create_world()
    for _ in range(8):
        worm_model.apply_gait(worm, world.getTime(), False)
        world.step()
        np.testing.assert_array_equal(worm.getCommands()[6:], np.zeros(4))
        assert worm_model.is_finite(worm)


@pytest.mark.parametrize("field", ["positions", "velocities"])
def test_soft_worm_finite_guard_checks_independent_soft_point_state(field):
    _, worm = worm_model.create_world()
    point = _points(worm)[0]
    setattr(point, field, [np.nan, 0, 0])
    assert np.isfinite(worm.getPositions()).all()
    assert np.isfinite(worm.getVelocities()).all()
    assert not worm_model.is_finite(worm)


def test_soft_worm_finite_guard_checks_world_point_overflow():
    _, worm = worm_model.create_world()
    worm.setPosition(3, 1e308)
    point = _points(worm)[0]
    point.positions = [1e308, 0, 0]
    assert np.isfinite(worm.getPositions()).all()
    assert np.isfinite(worm.getRootBodyNode().getTransform().matrix()).all()
    assert np.isfinite(point.positions).all()
    assert not np.isfinite(point.world_position).all()
    assert not worm_model.is_finite(worm)


@pytest.mark.parametrize("setter", ["setPosition", "setVelocity"])
def test_soft_worm_finite_guard_checks_skeleton_state(setter):
    _, worm = worm_model.create_world()
    getattr(worm, setter)(6, np.nan)
    assert not worm_model.is_finite(worm)


@pytest.mark.parametrize(
    "function,args",
    [
        ("apply_gait", (0.0, True)),
        ("root_x", ()),
        ("position_checksum", ()),
        ("is_finite", ()),
    ],
)
def test_soft_worm_rejects_null_or_wrong_skeleton_types(function, args):
    for invalid in (None, object()):
        with pytest.raises(TypeError):
            getattr(worm_model, function)(invalid, *args)


def test_soft_worm_helpers_preserve_empty_skeleton_behavior():
    empty = dart.dynamics.Skeleton()
    with pytest.raises(ValueError, match="root body"):
        worm_model.root_x(empty)
    assert worm_model.apply_gait(empty, 0.0, True) is None
    assert worm_model.position_checksum(empty) == 0.0
    assert worm_model.is_finite(empty)


def test_worm_survives_dropping_its_world():
    world, worm = worm_model.create_world()
    del world
    gc.collect()
    worm_model.apply_gait(worm, 0.1, True)
    assert np.isfinite(worm_model.position_checksum(worm))
    assert worm_model.is_finite(worm)
