import dartpy as dart
import numpy as np
import pytest


@pytest.mark.parametrize("layout", ["C", "F", "strided"])
def test_heightmap_round_trip(layout):
    shape = dart.dynamics.HeightmapShapef()
    assert isinstance(shape, dart.dynamics.Shape)
    np.testing.assert_array_equal(shape.getScale(), [1, 1, 1])
    scale = np.array([2, 3, 4], dtype=np.float32)
    assert shape.setScale(scale=scale) is None
    np.testing.assert_array_equal(shape.getScale(), scale)

    # Rows are depth along -y; columns are width along x; values are z heights.
    heights = np.array([[1, -2, 3], [4, 5, 6]], dtype=np.float32)
    if layout == "F":
        heights = np.asfortranarray(heights)
    elif layout == "strided":
        heights = np.repeat(heights, 2, axis=1)[:, ::2]
    assert shape.setHeightField(heights=heights) is None
    actual = shape.getHeightField()
    assert actual.shape == (2, 3)
    assert actual.dtype == np.float32
    np.testing.assert_array_equal(actual, heights)
    assert actual[0, 1] == -2
    assert actual[1, 0] == 4
    assert shape.getMinHeight() == -2
    assert shape.getMaxHeight() == 6
    assert shape.getType() == dart.dynamics.HeightmapShapef.getStaticType()

    heights[:] = 0
    assert shape.getHeightField()[1, 2] == 6
    actual[1, 2] = 99
    assert shape.getHeightField()[1, 2] == 6
    shape.setHeightField(np.zeros((3, 2), dtype=np.float32))
    assert actual[0, 1] == -2


@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_heightmap_ignores_nonfinite_input(invalid):
    shape = dart.dynamics.HeightmapShapef()
    heights = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
    shape.setHeightField(heights)
    bad_heights = heights.copy()
    bad_heights[1, 2] = invalid
    shape.setHeightField(bad_heights)
    np.testing.assert_array_equal(shape.getHeightField(), heights)
    assert shape.getMinHeight() == 1
    assert shape.getMaxHeight() == 6

    shape.setScale([1, invalid, 1])
    np.testing.assert_array_equal(shape.getScale(), [1, 1, 1])


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_construction():
    for shape in (
        dart.dynamics.VoxelGridShape(),
        dart.dynamics.VoxelGridShape(resolution=0.05),
    ):
        assert isinstance(shape, dart.dynamics.Shape)
        assert shape.getType() == dart.dynamics.VoxelGridShape.getStaticType()


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("point", [[1, 0, 0], np.array([1, 0, 0], dtype=np.float64)])
def test_voxel_grid_point_occupancy(point):
    shape = dart.dynamics.VoxelGridShape(resolution=0.1)
    assert shape.updateOccupancy(point) is None
    assert shape.getOccupancy(point=point) > 0.5
    assert shape.getOccupancy([0.5, 0, 0]) == 0
    occupied = shape.getOccupancy(point)
    shape.updateOccupancy(point=point, occupied=False)
    assert shape.getOccupancy(point) < occupied
    assert not hasattr(shape, "getOctree")


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("sensor_origin", [None, [1, 1, 0]])
@pytest.mark.parametrize("layout", ["C", "F", "strided"])
def test_voxel_grid_batch_occupancy(sensor_origin, layout):
    shape = dart.dynamics.VoxelGridShape(resolution=0.1)
    origin = np.zeros(3) if sensor_origin is None else np.array(sensor_origin)
    points = origin + np.array([[2, 0, 0], [0, 2, 0]], dtype=np.float64)
    if layout == "F":
        points = np.asfortranarray(points)
    elif layout == "strided":
        points = np.repeat(points, 2, axis=1)[:, ::2]
    midpoint = origin + [1, 0, 0]
    shape.updateOccupancy(midpoint)
    occupied = shape.getOccupancy(midpoint)
    if sensor_origin is None:
        assert shape.updateOccupancy(points) is None
    else:
        assert shape.updateOccupancy(points=points, sensorOrigin=origin) is None
    for point in points:
        assert shape.getOccupancy(point) > 0.5
    assert shape.getOccupancy(midpoint) < occupied


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize(
    "points",
    [
        np.zeros(()),
        np.zeros(3),
        np.zeros((2, 2)),
        np.zeros((2, 4)),
        np.zeros((2, 3, 1)),
        np.array([[1, 0, 0], [np.nan, 0, 0]]),
        np.array([[np.inf, 0, 0]]),
        np.array([[-np.inf, 0, 0]]),
    ],
)
def test_voxel_grid_rejects_invalid_batch(points):
    shape = dart.dynamics.VoxelGridShape()
    with pytest.raises(ValueError):
        shape.updateOccupancy(points=points)
    assert shape.getOccupancy([1, 0, 0]) == 0


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_voxel_grid_rejects_nonfinite_sensor_origin(invalid):
    shape = dart.dynamics.VoxelGridShape()
    with pytest.raises(ValueError):
        shape.updateOccupancy(
            points=np.array([[1, 0, 0]], dtype=np.float64),
            sensorOrigin=[invalid, 0, 0],
        )


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("invalid", [np.nan, np.inf, -np.inf])
def test_voxel_grid_rejects_nonfinite_point(invalid):
    shape = dart.dynamics.VoxelGridShape()
    with pytest.raises(ValueError):
        shape.updateOccupancy([invalid, 0, 0])
    with pytest.raises(ValueError):
        shape.getOccupancy([invalid, 0, 0])
