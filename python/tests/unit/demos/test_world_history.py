"""Playback restores owning state snapshots and retains its World safely."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models import soft_worm
from examples.demos.models.world_history import WorldHistory

FIELDS = ("Positions", "Velocities", "Accelerations", "Forces", "Commands")
MAX_SIZE_T = 2 * sys.maxsize + 1


def _rigid_world():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("recorded")
    joint, _ = skeleton.createFreeJointAndBodyNodePair()
    joint.setActuatorType(dart.dynamics.ActuatorType.SERVO)
    world.addSkeleton(skeleton)
    return world, skeleton


def _configuration(skeleton):
    return [getattr(skeleton, f"get{field}")().copy() for field in FIELDS]


def _seed_configuration(skeleton, frame):
    for i, field in enumerate(FIELDS):
        getattr(skeleton, f"set{field}")(
            frame * (i + 1) * 0.01 + np.arange(skeleton.getNumDofs()) * 0.001
        )


def _soft_state(skeleton):
    values = _configuration(skeleton)
    for body in skeleton.getBodyNodes():
        values.append(body.getWorldTransform().matrix().copy())
        if isinstance(body, dart.dynamics.SoftBodyNode):
            for i in range(body.getNumPointMasses()):
                point = body.get_point_mass(i)
                values.extend(
                    getattr(point, field).copy()
                    for field in (
                        "positions", "velocities", "accelerations", "forces",
                        "world_position",
                    )
                )
    return np.concatenate([value.ravel() for value in values])


def test_history_restores_skeleton_and_soft_point_states():
    world, worm = soft_worm.create_world()
    history = WorldHistory(world)
    assert history.world is world
    assert history.max_frames == 4000
    snapshots = [_soft_state(worm)]
    for frame in range(1, 4):
        soft_worm.apply_gait(worm, world.getTime(), True)
        world.step()
        worm.setVelocities(frame * np.linspace(0.001, 0.01, worm.getNumDofs()))
        worm.setAccelerations(frame * np.linspace(0.1, 1.0, worm.getNumDofs()))
        for bi, body in enumerate(worm.getBodyNodes()):
            for pi in range(body.getNumPointMasses()):
                point = body.get_point_mass(pi)
                point.positions = [frame * (pi + 1) * 0.0001, -bi * 0.0002, 0.0003]
                point.velocities = [frame * 0.001, bi * 0.002, pi * 0.0001]
        worm.computeInverseDynamics(True, True, True)
        worm.setForces(frame * np.linspace(0.2, 2.0, worm.getNumDofs()))
        worm.setCommands(np.r_[np.zeros(6), frame * np.linspace(0.3, 1.2, 4)])
        history.record()
        snapshots.append(_soft_state(worm))
    point = worm.getBodyNode(0).get_point_mass(0)
    assert np.any(point.accelerations != 0)
    assert np.any(point.forces != 0)
    assert history.size == history.frames_recorded == 4
    world.setTime(77.25)
    sim_frames = world.getSimFrames()
    for index in (3, 0, 2, 1):
        history.move_to(index)
        assert history.current_index == index
        np.testing.assert_array_equal(_soft_state(worm), snapshots[index])
        assert world.getTime() == 77.25
        assert world.getSimFrames() == sim_frames


def test_history_caps_frames_branches_and_clamps_playback():
    world, skeleton = _rigid_world()
    history = WorldHistory(world, max_frames=3)
    for frame in range(1, 5):
        skeleton.setPosition(3, frame)
        history.record()
    assert history.size == 3
    assert history.frames_recorded == 5
    history.move_backward(MAX_SIZE_T)
    assert history.current_index == 0
    assert skeleton.getPosition(3) == 2
    history.move_forward()
    assert skeleton.getPosition(3) == 3
    history.move_backward()
    skeleton.setPosition(3, 9)
    history.record()
    assert history.size == 2
    assert history.frames_recorded == 6
    history.move_to(0)
    assert skeleton.getPosition(3) == 2
    history.move_forward(MAX_SIZE_T)
    assert history.current_index == 1
    assert skeleton.getPosition(3) == 9
    history.move_to(MAX_SIZE_T)
    assert history.current_index == 1


def test_history_accepts_empty_world_and_one_frame_cap():
    history = WorldHistory(dart.simulation.World(), max_frames=1)
    history.record()
    history.move_backward()
    history.move_forward()
    assert history.size == 1
    assert history.current_index == 0
    assert history.frames_recorded == 2


def test_history_restores_all_configuration_fields():
    world, skeleton = _rigid_world()
    _seed_configuration(skeleton, 1)
    initial = _configuration(skeleton)
    history = WorldHistory(world)
    _seed_configuration(skeleton, 2)
    later = _configuration(skeleton)
    history.record()
    for index, expected in ((0, initial), (1, later), (0, initial)):
        history.move_to(index)
        np.testing.assert_array_equal(_configuration(skeleton), expected)


def test_history_skips_empty_slices_and_restores_replacement_by_world_index():
    world, original = _rigid_world()
    _seed_configuration(original, 1)
    initial = _configuration(original)
    history = WorldHistory(world)
    world.removeAllSkeletons()
    history.move_to(0)
    history.record()
    replacement_world, replacement = _rigid_world()
    replacement_world.removeSkeleton(replacement)
    world.addSkeleton(replacement)
    _seed_configuration(replacement, 9)
    unchanged = _configuration(replacement)
    history.move_to(1)
    np.testing.assert_array_equal(_configuration(replacement), unchanged)
    history.move_to(0)
    np.testing.assert_array_equal(_configuration(replacement), initial)


def test_history_preserves_extra_dofs_and_skips_invalid_saved_indices():
    world, skeleton = _rigid_world()
    _seed_configuration(skeleton, 1)
    initial = _configuration(skeleton)
    history = WorldHistory(world)
    joint, _ = skeleton.createRevoluteJointAndBodyNodePair(skeleton.getBodyNode(0))
    joint.setActuatorType(dart.dynamics.ActuatorType.SERVO)
    _seed_configuration(skeleton, 9)
    extra = [values[-1] for values in _configuration(skeleton)]
    history.move_to(0)
    for actual, expected, tail in zip(_configuration(skeleton), initial, extra):
        np.testing.assert_array_equal(actual[:6], expected)
        assert actual[-1] == tail

    history = WorldHistory(world)
    world.removeAllSkeletons()
    replacement_world, replacement = _rigid_world()
    replacement_world.removeSkeleton(replacement)
    world.addSkeleton(replacement)
    _seed_configuration(replacement, 8)
    unchanged = _configuration(replacement)
    # Native configuration rejects the entire vector if any saved index is invalid.
    history.move_to(0)
    np.testing.assert_array_equal(_configuration(replacement), unchanged)


def test_history_ignores_extra_skeletons_and_uses_common_prefix_after_removal():
    world, original = _rigid_world()
    _seed_configuration(original, 1)
    initial = _configuration(original)
    history = WorldHistory(world)
    tail_world, tail = _rigid_world()
    tail_world.removeSkeleton(tail)
    world.addSkeleton(tail)
    _seed_configuration(tail, 9)
    tail_state = _configuration(tail)
    history.move_to(0)
    np.testing.assert_array_equal(_configuration(tail), tail_state)
    history.record()
    world.removeSkeleton(original)
    history.move_to(1)
    np.testing.assert_array_equal(_configuration(tail), initial)


class _Index:
    def __index__(self):
        return 3


@pytest.mark.parametrize("value", [True, np.int64(3), np.uint64(3), _Index(), MAX_SIZE_T])
def test_history_accepts_unsigned_integer_protocol(value):
    history = WorldHistory(dart.simulation.World(), max_frames=value)
    for method in (history.move_to, history.move_forward, history.move_backward):
        method(value)
        method(0)
        assert history.current_index == 0


@pytest.mark.parametrize("error", [ValueError, OverflowError, KeyboardInterrupt, SystemExit])
def test_history_converts_index_protocol_errors_to_type_error(error):
    class Index:
        def __index__(self):
            raise error

    value = Index()
    with pytest.raises(TypeError):
        WorldHistory(dart.simulation.World(), max_frames=value)
    history = WorldHistory(dart.simulation.World())
    for method in (history.move_to, history.move_forward, history.move_backward):
        with pytest.raises(TypeError):
            method(value)


@pytest.mark.parametrize("world", [None, object(), 1])
def test_history_rejects_invalid_world(world):
    with pytest.raises(TypeError):
        WorldHistory(world)


@pytest.mark.parametrize("world, calls", [(None, 0), (object(), 1)])
def test_history_matches_native_constructor_conversion_order(world, calls):
    converted = []

    class Index:
        def __index__(self):
            converted.append(True)
            return 3

    with pytest.raises(TypeError):
        WorldHistory(world, max_frames=Index())
    assert len(converted) == calls


def test_history_rejects_object_spoofing_world_class():
    class Object:
        @property
        def __class__(self):
            return dart.simulation.World

    with pytest.raises(TypeError):
        WorldHistory(Object())


@pytest.mark.parametrize("capacity", [0, -1, 1.5, MAX_SIZE_T + 1, "3", None])
def test_history_rejects_invalid_capacity(capacity):
    with pytest.raises(ValueError if capacity == 0 else TypeError):
        WorldHistory(dart.simulation.World(), max_frames=capacity)


@pytest.mark.parametrize("method", ["move_to", "move_forward", "move_backward"])
def test_history_rejects_negative_or_fractional_indices(method):
    history = WorldHistory(dart.simulation.World())
    for index in (-1, 1.5, MAX_SIZE_T + 1, np.float64(3), "3", None):
        with pytest.raises(TypeError):
            getattr(history, method)(index)


def test_history_keeps_original_world_wrapper_alive():
    class World(dart.simulation.World):
        pass

    world = World()
    world.label = "recorded world"
    reference = weakref.ref(world)
    history = WorldHistory(world)
    del world
    gc.collect()
    assert reference() is history.world
    assert history.world.label == "recorded world"
    history.world.step()
    history.record()
    del history
    gc.collect()
    assert reference() is None


def test_history_world_python_backref_cycle_collects():
    class World(dart.simulation.World):
        pass

    world = World()
    history = WorldHistory(world)
    world.owner = history
    references = weakref.ref(world), weakref.ref(history)
    del world, history
    gc.collect()
    collected = all(reference() is None for reference in references)
    if references[0]() is not None:
        references[0]().owner = None
    gc.collect()
    assert all(reference() is None for reference in references)
    assert collected, "WorldHistory's retained World edge was invisible to GC"
