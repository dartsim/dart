"""The private soft-worm model can be driven without a viewer."""

import gc

import dartpy as dart
import numpy as np
import pytest
from dartpy import _demos

worm_model = _demos.soft_worm


@pytest.mark.parametrize("enabled", [False, True])
def test_soft_worm_construction_steps_and_determinism(enabled):
    worlds_and_worms = [worm_model.create_world() for _ in range(2)]
    for world, worm in worlds_and_worms:
        assert world.getNumSkeletons() == 2
        assert worm.getNumSoftBodyNodes() == worm_model.num_links
        assert worm_model.gait_amplitude > 0
        assert worm_model.gait_frequency_hz > 0
        assert worm_model.is_finite(worm)
        for _ in range(8):
            worm_model.apply_gait(worm, world.getTime(), enabled)
            world.step()
        assert np.isfinite(worm_model.root_x(worm))
        assert worm_model.is_finite(worm)

    lhs, rhs = (worm for _, worm in worlds_and_worms)
    assert worm_model.position_checksum(lhs) == worm_model.position_checksum(rhs)
    np.testing.assert_array_equal(lhs.getPositions(), rhs.getPositions())


@pytest.mark.parametrize(
    "function,args",
    [
        ("apply_gait", (0.0, True)),
        ("root_x", ()),
        ("position_checksum", ()),
        ("is_finite", ()),
    ],
)
def test_soft_worm_rejects_null_skeleton(function, args):
    with pytest.raises((TypeError, ValueError)):
        getattr(worm_model, function)(None, *args)


def test_root_x_rejects_empty_skeleton():
    with pytest.raises(ValueError):
        worm_model.root_x(dart.dynamics.Skeleton())


def test_worm_survives_dropping_its_world():
    world, worm = worm_model.create_world()
    del world
    gc.collect()
    worm_model.apply_gait(worm, 0.1, True)
    assert np.isfinite(worm_model.position_checksum(worm))
    assert worm_model.is_finite(worm)
