import dartpy as dart
import numpy as np
import pytest


def test_mesh_vertex_groups_preserve_order_scale_and_own_their_arrays(tmp_path):
    (tmp_path / "groups.mtl").write_text(
        "newmtl first\nKd 1 0 0\nnewmtl second\nKd 0 1 0\n"
    )
    (tmp_path / "groups.obj").write_text(
        "mtllib groups.mtl\n"
        "v 0.117 0 0\nv 0.25 0 0\nv 0.117 0.5 0\n"
        "v 1 0 0\nv 2 0 0\nv 2 1 0\nv 1 1 0\n"
        "o first\nusemtl first\nf 1 2 3\n"
        "o second\nusemtl second\nf 4 5 6\nf 4 6 7\n"
    )
    urdf = tmp_path / "groups.urdf"
    urdf.write_text(
        '<robot name="groups"><link name="body"><visual><geometry>'
        '<mesh filename="groups.obj" scale="2 3 4"/>'
        "</geometry></visual></link></robot>"
    )
    skeleton = dart.utils.DartLoader().parseSkeleton(str(urdf))
    assert skeleton is not None
    shape = skeleton.getBodyNode(0).getShapeNode(0).getShape()
    expected = [
        np.array(
            [[0.117, 0, 0], [0.25, 0, 0], [0.117, 0.5, 0]], dtype=np.float32
        ).astype(float),
        np.array([[1, 0, 0], [2, 0, 0], [2, 1, 0], [1, 1, 0]], dtype=float),
    ]
    groups = shape.mesh_vertex_groups
    assert [group.shape for group in groups] == [(3, 3), (4, 3)]
    np.testing.assert_array_equal(shape.getScale(), [2, 3, 4])
    for actual, vertices in zip(groups, expected):
        assert actual.dtype == np.float64
        np.testing.assert_array_equal(actual, vertices)
        actual[:] = 99
    for actual, vertices in zip(shape.mesh_vertex_groups, expected):
        np.testing.assert_array_equal(actual, vertices)
    with pytest.raises(AttributeError):
        shape.mesh_vertex_groups = []
    copies = shape.mesh_vertex_groups
    shape.setMesh(None)
    assert shape.mesh_vertex_groups == []
    for actual, vertices in zip(copies, expected):
        np.testing.assert_array_equal(actual, vertices)


def test_mesh_vertex_groups_for_an_empty_mesh():
    assert dart.dynamics.MeshShape([1, 1, 1], None).mesh_vertex_groups == []


def test_point_cloud_supports_native_point_shape_type():
    shape = dart.dynamics.PointCloudShape()
    point = dart.dynamics.PointCloudShape.PointShapeType.POINT
    assert int(point) == 3
    shape.setPointShapeType(point)
    assert shape.getPointShapeType() == point


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
@pytest.mark.parametrize(
    "resolution",
    [
        0,
        -0.1,
        np.nan,
        np.inf,
        -np.inf,
        np.nextafter(1e6, 2e6),
        1e18,
        np.nextafter(1e-6, 0),
        1e-320,
    ],
)
def test_voxel_grid_rejects_invalid_resolution(resolution):
    with pytest.raises(ValueError, match=r"resolution must be within \[1e-6, 1e6\]"):
        dart.dynamics.VoxelGridShape(resolution=resolution)


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_works_at_the_smallest_resolution():
    shape = dart.dynamics.VoxelGridShape(resolution=1e-6)
    shape.updateOccupancy([0, 0, 0])
    assert shape.getOccupancy([0, 0, 0]) > 0.5


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("coordinate", [0.01, 0.03, 0.06, -0.05])
def test_voxel_grid_queries_updated_boundary_cell(coordinate):
    shape = dart.dynamics.VoxelGridShape(resolution=0.01)
    point = [coordinate, coordinate, coordinate]
    shape.updateOccupancy(point)
    assert shape.getOccupancy(point) > 0.5


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("resolution", [0.01, 0.125, 0.25])
@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("path", ["single", "batch", "sensorOrigin", "query"])
@pytest.mark.parametrize(
    "invalid", [np.nan, np.inf, -np.inf, 1e39, -1e39, "below", "upper", "above"]
)
def test_voxel_grid_rejects_unaddressable_points(resolution, axis, path, invalid):
    shape = dart.dynamics.VoxelGridShape(resolution=resolution)
    # OctoMap's depth is 16; the negative bound is inclusive, positive exclusive.
    bound = np.float32(32768 * resolution)
    outside = float(np.nextafter(bound, np.float32(np.inf)))
    upper = float(bound) if float(bound) >= 32768 * resolution else outside
    if isinstance(invalid, str):
        coordinate = {
            "below": -outside,
            "upper": upper,
            "above": float(np.nextafter(np.float32(upper), np.float32(np.inf))),
        }[invalid]
        message = "lies outside the voxel grid"
    elif np.isfinite(invalid) and path in ("single", "query"):
        # Single points reach OctoMap as doubles, so 1e39 stays finite.
        coordinate = invalid
        message = "lies outside the voxel grid"
    else:
        coordinate = invalid
        message = "must be finite"
    point = np.zeros(3)
    point[axis] = coordinate
    existing = [1, 1, 1]
    untouched = [2, 2, 2]
    shape.updateOccupancy(existing)
    occupancy = shape.getOccupancy(existing)
    name = "sensorOrigin" if path == "sensorOrigin" else "point"
    with pytest.raises(ValueError, match=f"{name} {message}"):
        if path == "single":
            shape.updateOccupancy(point)
        elif path == "batch":
            shape.updateOccupancy(points=np.array([untouched, point]))
        elif path == "sensorOrigin":
            shape.updateOccupancy(points=np.array([untouched]), sensorOrigin=point)
        else:
            shape.getOccupancy(point)
    assert shape.getOccupancy(existing) == occupancy
    assert shape.getOccupancy(untouched) == 0


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("resolution", [0.01, 0.125, 0.25])
@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("path", ["single", "batch", "sensorOrigin", "query"])
@pytest.mark.parametrize("boundary", ["lower", "lower_inside", "upper_inside"])
def test_voxel_grid_accepts_addressable_boundaries(resolution, axis, path, boundary):
    shape = dart.dynamics.VoxelGridShape(resolution=resolution)
    bound = np.float32(32768 * resolution)
    inside = float(np.nextafter(bound, np.float32(0)))
    coordinate = {
        "lower": -float(bound),
        "lower_inside": -inside,
        "upper_inside": float(bound) if float(bound) < 32768 * resolution else inside,
    }[boundary]
    point = np.zeros(3)
    point[axis] = coordinate
    if path == "batch":
        shape.updateOccupancy(points=np.array([point]), sensorOrigin=point)
    elif path == "sensorOrigin":
        endpoint = point.copy()
        # Inside a cell: OctoMap stores ray endpoints at float precision.
        endpoint[axis] += 2.5 * resolution if coordinate < 0 else -2.5 * resolution
        shape.updateOccupancy(points=np.array([endpoint]), sensorOrigin=point)
        assert shape.getOccupancy(endpoint) > 0.5
        assert 0 < shape.getOccupancy(point) < 0.5
        return
    else:
        shape.updateOccupancy(point)
    assert shape.getOccupancy(point) > 0.5


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
    "points, error",
    [
        (np.zeros(()), TypeError),
        (np.zeros(3), TypeError),
        (np.zeros((2, 2)), TypeError),
        (np.zeros((2, 4)), TypeError),
        (np.zeros((2, 3, 1)), TypeError),
        (np.array([[1, 0, 0], [np.nan, 0, 0]]), ValueError),
        (np.array([[np.inf, 0, 0]]), ValueError),
        (np.array([[-np.inf, 0, 0]]), ValueError),
    ],
)
def test_voxel_grid_rejects_invalid_batch(points, error):
    shape = dart.dynamics.VoxelGridShape()
    with pytest.raises(error):
        shape.updateOccupancy(points=points)
    assert shape.getOccupancy([1, 0, 0]) == 0


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_takes_a_column_vector_as_one_point():
    shape = dart.dynamics.VoxelGridShape(resolution=0.1)
    point = np.array([[1.0], [0.0], [0.0]])
    shape.updateOccupancy(point)
    assert shape.getOccupancy(point) > 0.5


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


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_rejects_rays_longer_than_octomap_traces():
    shape = dart.dynamics.VoxelGridShape(resolution=0.001)
    with pytest.raises(ValueError, match="too many voxels"):
        shape.updateOccupancy(
            points=np.array([[20.0, 20.0, 20.0]]), sensorOrigin=[-20.0, -20.0, -20.0]
        )
    assert shape.getOccupancy([20.0, 20.0, 20.0]) == 0
    # A long ray that fits the buffer is still traced.
    shape.updateOccupancy(points=np.array([[30.0, 0.0, 0.0]]))
    assert shape.getOccupancy([30.0, 0.0, 0.0]) > 0.5


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_rejects_rays_float_cannot_measure():
    shape = dart.dynamics.VoxelGridShape(resolution=1e-6)
    # The endpoints straddle a cell boundary, yet their float distance
    # squares to zero.
    with pytest.raises(ValueError, match="float arithmetic"):
        shape.updateOccupancy(
            points=np.array([[1e-30, 0, 0]]), sensorOrigin=[-1e-30, 0, 0]
        )


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_traces_across_the_grid_at_the_largest_resolution():
    shape = dart.dynamics.VoxelGridShape(resolution=1e6)
    inside = float(np.nextafter(np.float32(32768 * 1e6), np.float32(0)))
    shape.updateOccupancy(
        points=np.array([[inside, 0, 0]]), sensorOrigin=[-inside, 0, 0]
    )
    assert shape.getOccupancy([inside, 0, 0]) > 0.5
    assert 0 < shape.getOccupancy([0, 0, 0]) < 0.5


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
def test_voxel_grid_validates_single_points_at_double_precision():
    shape = dart.dynamics.VoxelGridShape(resolution=0.125)
    # Addressable as a double, although it rounds to the bound as a float.
    shape.updateOccupancy([4095.99999, 0, 0])
    assert shape.getOccupancy([4095.99999, 0, 0]) > 0.5
    # Outside as a double, although it rounds into range as a float.
    with pytest.raises(ValueError, match="lies outside the voxel grid"):
        shape.updateOccupancy([-4096.00001, 0, 0])


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize("layout", ["C", "F", "strided"])
def test_voxel_grid_occupancy_preflight_preserves_all_occupancy(layout):
    shape = dart.dynamics.VoxelGridShape(resolution=0.1)
    shape.updateOccupancy([1, 0, 0])
    occupied = shape.getOccupancy([1, 0, 0])
    points = np.array([[2, 0, 0], [0, 2, 0]], dtype=float)
    if layout == "F":
        points = np.asfortranarray(points)
    elif layout == "strided":
        points = np.repeat(points, 2, axis=1)[:, ::2]
    assert (
        shape.validate_occupancy_update(points=points, sensor_origin=[0, 0, 0]) is None
    )
    assert shape.getOccupancy([1, 0, 0]) == occupied
    assert shape.getOccupancy([2, 0, 0]) == 0
    assert shape.getOccupancy([0, 2, 0]) == 0


@pytest.mark.skipif(
    not hasattr(dart.dynamics, "VoxelGridShape"), reason="Octomap is unavailable"
)
@pytest.mark.parametrize(
    "points, origin, resolution",
    [
        (np.zeros((2, 2)), [0, 0, 0], 0.01),
        (np.array([[np.nan, 0, 0]]), [0, 0, 0], 0.01),
        (np.array([[1e39, 0, 0]]), [0, 0, 0], 0.01),
        (np.array([[400, 0, 0]]), [0, 0, 0], 0.01),
        (np.array([[2, 0, 0]]), [np.inf, 0, 0], 0.01),
        (np.array([[20, 20, 20]]), [-20, -20, -20], 0.001),
        (np.array([[1e-30, 0, 0]]), [-1e-30, 0, 0], 1e-6),
    ],
)
def test_voxel_grid_preflight_matches_update_validation(points, origin, resolution):
    shape = dart.dynamics.VoxelGridShape(resolution)
    shape.updateOccupancy([0, 0, 0])
    occupied = shape.getOccupancy([0, 0, 0])
    with pytest.raises((TypeError, ValueError)) as preflight:
        shape.validate_occupancy_update(points, sensor_origin=origin)
    with pytest.raises(type(preflight.value)) as update:
        shape.updateOccupancy(points=points, sensorOrigin=origin)
    if isinstance(preflight.value, ValueError):
        assert str(update.value) == str(preflight.value)
    assert shape.getOccupancy([0, 0, 0]) == occupied
