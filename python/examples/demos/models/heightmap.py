"""Generate demo terrain with DART's single-precision random distribution."""

import operator

import dartpy as dart
import numpy as np


def create_heightmap_shape(x_resolution, y_resolution, x_size, y_size, z_min, z_max):
    x_resolution = operator.index(x_resolution)
    y_resolution = operator.index(y_resolution)
    max_elements = min(
        np.iinfo(np.intp).max,
        np.iinfo(np.uintp).max // np.dtype(np.float32).itemsize,
    )
    if (
        x_resolution <= 0
        or y_resolution <= 0
        or x_resolution > max_elements // y_resolution
    ):
        raise ValueError("Heightmap resolution must be positive and fit Eigen storage")

    with np.errstate(over="ignore", under="ignore"):
        x_size, y_size, z_min, z_max = (
            np.float32(value) for value in (x_size, y_size, z_min, z_max)
        )
        span = z_max - z_min
        scale = np.array(
            [
                x_size / np.float32(x_resolution),
                y_size / np.float32(y_resolution),
                1,
            ],
            dtype=np.float32,
        )
    if not np.isfinite([x_size, y_size]).all() or x_size <= 0 or y_size <= 0:
        raise ValueError("Heightmap size must be finite and positive")
    if not np.isfinite([z_min, z_max, span]).all() or z_min > z_max:
        raise ValueError("Heightmap heights must be finite and ordered")
    if not np.isfinite(scale).all() or (scale <= 0).any():
        raise ValueError("Heightmap scale must be finite and positive")

    # Float draws consume the native stream once per cell, in row-first order.
    data = np.fromiter(
        (
            dart.math.Random.uniform_float(float(z_min), float(z_max))
            for _ in range(x_resolution * y_resolution)
        ),
        dtype=np.float32,
        count=x_resolution * y_resolution,
    ).reshape(y_resolution, x_resolution)
    shape = dart.dynamics.HeightmapShapef()
    shape.setScale(scale)
    shape.setHeightField(data)
    return shape
