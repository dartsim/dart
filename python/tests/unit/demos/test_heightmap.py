"""Procedural terrain, storage ownership, validation and float RNG parity."""

import sys
from pathlib import Path

import dartpy as dart
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "python"))
from examples.demos.models.heightmap import create_heightmap_shape


def test_heightmap_shape_owns_data_and_repeats_seeded_randomness():
    fields = []
    for _ in range(2):
        dart.math.Random.setSeed(42)
        shape = create_heightmap_shape(8, 6, 2, 3, -0.2, 0.1)
        assert isinstance(shape, dart.dynamics.HeightmapShapef)
        field = shape.getHeightField()
        assert field.shape == (6, 8) and field.dtype == np.float32
        assert np.float32(-0.2) <= field.min() <= field.max() <= np.float32(0.1)
        np.testing.assert_array_equal(shape.getScale(), [0.25, 0.5, 1])
        fields.append(field.copy())
        with pytest.raises(ValueError, match="assignment destination is read-only"):
            field[:] = 99
        field = field.copy()
        field[:] = 99
        np.testing.assert_array_equal(shape.getHeightField(), fields[-1])
    np.testing.assert_array_equal(*fields)


def test_heightmap_preserves_float_draw_order_and_random_stream():
    dart.math.Random.setSeed(1234)
    expected = np.array(
        [dart.math.Random.uniform_float(-0.2, 0.5) for _ in range(12)],
        dtype=np.float32,
    ).reshape(3, 4)
    next_float = dart.math.Random.uniform_float(0, 1)
    next_double = dart.math.Random.uniform(0, 1)

    dart.math.Random.setSeed(1234)
    shape = create_heightmap_shape(4, 3, 2, 3, -0.2, 0.5)
    np.testing.assert_array_equal(shape.getHeightField(), expected)
    np.testing.assert_array_equal(shape.getScale(), [0.5, 1, 1])
    assert dart.math.Random.uniform_float(0, 1) == next_float
    assert dart.math.Random.uniform(0, 1) == next_double


def test_heightmap_builds_default_terrain():
    shape = create_heightmap_shape(100, 100, 2, 2, 0, 0.1)
    assert shape.getHeightField().shape == (100, 100)
    np.testing.assert_array_equal(
        shape.getScale(), np.array([0.02, 0.02, 1], dtype=np.float32)
    )
    assert shape.getMinHeight() >= 0
    assert shape.getMaxHeight() <= np.float32(0.1)


@pytest.mark.parametrize(
    "parameters",
    [
        (0, 6, 2, 3, 0, 0.1),
        (-1, 6, 2, 3, 0, 0.1),
        (8, 0, 2, 3, 0, 0.1),
        (8, -1, 2, 3, 0, 0.1),
        (sys.maxsize, 2, 2, 3, 0, 0.1),
        (np.iinfo(np.uintp).max // 4 + 1, 1, 2, 3, 0, 0.1),
        (8, 6, 0, 3, 0, 0.1),
        (8, 6, -1, 3, 0, 0.1),
        (8, 6, 2, 0, 0, 0.1),
        (8, 6, 2, -1, 0, 0.1),
        (8, 6, np.nan, 3, 0, 0.1),
        (8, 6, 1e100, 3, 0, 0.1),
        (8, 6, 2, np.inf, 0, 0.1),
        (8, 6, 2, 3, np.nan, 0.1),
        (8, 6, 2, 3, 0, np.inf),
        (8, 6, 2, 3, 0.2, 0.1),
        (8, 6, 2, 3, -np.finfo(np.float32).max, np.finfo(np.float32).max),
        (5, 5, np.finfo(np.float32).smallest_subnormal, 2, 0, 0.1),
    ],
)
def test_heightmap_shape_rejects_invalid_inputs_before_random_draws(parameters):
    dart.math.Random.setSeed(1234)
    next_float = dart.math.Random.uniform_float(0, 1)
    next_double = dart.math.Random.uniform(0, 1)
    dart.math.Random.setSeed(1234)
    with pytest.raises(ValueError):
        create_heightmap_shape(*parameters)
    assert dart.math.Random.uniform_float(0, 1) == next_float
    assert dart.math.Random.uniform(0, 1) == next_double
