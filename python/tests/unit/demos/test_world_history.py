"""Native playback restores soft points and retains its World safely."""

import gc
import sys
import weakref
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest
from dartpy import _demos

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from examples.demos.models import soft_worm


def _rigid_world():
    world = dart.simulation.World()
    skeleton = dart.dynamics.Skeleton("recorded")
    skeleton.createFreeJointAndBodyNodePair()
    world.addSkeleton(skeleton)
    return world, skeleton


def test_history_restores_skeleton_and_soft_point_states():
    world, worm = soft_worm.create_world()
    history = _demos.WorldHistory(world)
    assert history.world is world
    assert history.max_frames == 4000
    snapshots = [
        (worm.getPositions().copy(), soft_worm.position_checksum(worm))
    ]
    for _ in range(3):
        soft_worm.apply_gait(worm, world.getTime(), True)
        world.step()
        history.record()
        snapshots.append(
            (worm.getPositions().copy(), soft_worm.position_checksum(worm))
        )
    assert history.size == history.frames_recorded == 4
    for index, (positions, checksum) in enumerate(snapshots):
        history.move_to(index)
        assert history.current_index == index
        np.testing.assert_array_equal(worm.getPositions(), positions)
        assert soft_worm.position_checksum(worm) == checksum


def test_history_caps_frames_branches_and_clamps_playback():
    world, skeleton = _rigid_world()
    history = _demos.WorldHistory(world, max_frames=3)
    for frame in range(1, 5):
        skeleton.setPosition(3, frame)
        history.record()
    assert history.size == 3
    assert history.frames_recorded == 5
    history.move_backward(10)
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
    history.move_forward(10)
    assert history.current_index == 1
    assert skeleton.getPosition(3) == 9
    history.move_to(10)
    assert history.current_index == 1


def test_history_accepts_empty_world_and_one_frame_cap():
    history = _demos.WorldHistory(dart.simulation.World(), max_frames=1)
    history.record()
    history.move_backward()
    history.move_forward()
    assert history.size == 1
    assert history.current_index == 0
    assert history.frames_recorded == 2


@pytest.mark.parametrize("world", [None, object(), 1])
def test_history_rejects_invalid_world(world):
    with pytest.raises((TypeError, ValueError)):
        _demos.WorldHistory(world)


@pytest.mark.parametrize("capacity", [0, -1, 1.5])
def test_history_rejects_invalid_capacity(capacity):
    with pytest.raises((TypeError, ValueError)):
        _demos.WorldHistory(dart.simulation.World(), max_frames=capacity)


@pytest.mark.parametrize("method", ["move_to", "move_forward", "move_backward"])
def test_history_rejects_negative_or_fractional_indices(method):
    history = _demos.WorldHistory(dart.simulation.World())
    for index in (-1, 1.5):
        with pytest.raises(TypeError):
            getattr(history, method)(index)


def test_history_keeps_original_world_wrapper_alive():
    class World(dart.simulation.World):
        pass

    world = World()
    world.label = "recorded world"
    reference = weakref.ref(world)
    history = _demos.WorldHistory(world)
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
    history = _demos.WorldHistory(world)
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
