"""Seeded visual-mesh sampling, sensor orbit, colors, and voxel updates."""

import math
import operator

import dartpy as dart
import numpy as np

_NUM_POINTS = 500
_MAX_ATTEMPTS = 200000


def _real(value):
    try:
        if issubclass(type(value), float):
            return float.__float__(value)
        if hasattr(type(value), "__float__") or hasattr(type(value), "__index__"):
            return float(value)
    except BaseException:
        pass
    raise TypeError("time_step must be a real number")


def _integer(value):
    try:
        if issubclass(type(value), np.bool_):
            raise TypeError
        result = operator.index(value)
        if -(2**31) <= result < 2**31:
            return result
    except BaseException:
        pass
    raise TypeError("value must be an integer")


def _box_points(count):
    return [
        [dart.math.Random.uniform(-0.5, 0.5) for _ in range(3)] for _ in range(count)
    ]


def _robot_points(robot):
    points = []
    bodies = robot.getBodyNodes()
    visuals = [
        [node for node in body.getShapeNodes() if node.hasVisualAspect()]
        for body in bodies
    ]
    groups = {}
    attempts = 0
    while bodies and len(points) < _NUM_POINTS and attempts < _MAX_ATTEMPTS:
        attempts += 1
        body_index = dart.math.Random.uniform_int(0, len(bodies) - 1)
        nodes = visuals[body_index]
        if not nodes:
            continue
        node = nodes[dart.math.Random.uniform_int(0, len(nodes) - 1)]
        shape = node.getShape()
        if not issubclass(type(shape), dart.dynamics.MeshShape):
            continue
        key = id(shape)
        if key not in groups:
            # Native sampling uses raw Assimp vertices, ignoring MeshShape scale.
            groups[key] = (
                shape,
                [
                    np.asarray(group, dtype=np.float32).astype(float)
                    for group in shape.mesh_vertex_groups
                ],
            )
        meshes = groups[key][1]
        if not meshes:
            continue
        mesh = meshes[dart.math.Random.uniform_int(0, len(meshes) - 1)]
        if len(mesh) == 0:
            continue
        vertex = mesh[dart.math.Random.uniform_int(0, len(mesh) - 1)]
        transform = node.getWorldTransform()
        points.append(transform.rotation() @ vertex + transform.translation())
    points.extend(_box_points(_NUM_POINTS - len(points)))
    return points


def _colors(points):
    minimum = float(points[:, 2].min())
    # Preserve the original positive DBL_MIN initialization of maxZ.
    maximum = max(float(points[:, 2].max()), float(np.finfo(float).tiny))
    difference = max(abs(maximum - minimum), float(np.finfo(float).tiny))
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        red = (points[:, 2].astype(np.float32) - np.float32(minimum)) / np.float32(
            difference
        )
        blue = np.float32(1.0) - red
    # std::max(lower, std::min(value, upper)) maps NaN to the lower bound.
    red = np.fmax(np.float32(0.1), np.minimum(red, np.float32(0.9)))
    blue = np.fmax(np.float32(0.1), np.minimum(blue, np.float32(0.9)))
    colors = np.empty((_NUM_POINTS, 4), dtype=np.float32)
    colors[:, 0] = red
    colors[:, 1] = np.float32(0.1)
    colors[:, 2] = blue
    colors[:, 3] = np.float32(0.75)
    return colors.astype(float)


class PointCloudSensor:
    """Own the four scene resources and update their demo sampling state."""

    def __init__(self, robot, sensor_frame, point_cloud, voxel_grid):
        owners = robot, sensor_frame, point_cloud, voxel_grid
        if any(owner is None for owner in owners):
            raise ValueError(
                "robot, sensor_frame, point_cloud and voxel_grid must not be None"
            )
        types = (
            dart.dynamics.Skeleton,
            dart.dynamics.SimpleFrame,
            dart.dynamics.PointCloudShape,
            getattr(dart.dynamics, "VoxelGridShape", type(None)),
        )
        if any(not issubclass(type(owner), kind) for owner, kind in zip(owners, types)):
            raise TypeError(
                "expected Skeleton, SimpleFrame, PointCloudShape and VoxelGridShape"
            )
        self._robot = robot
        self._sensor_frame = sensor_frame
        self._point_cloud = point_cloud
        self._voxel_grid = voxel_grid
        self._updating = True
        self._sampling_mode = 0
        self._orbit_time = 0.0

    @property
    def updating(self):
        return self._updating

    @updating.setter
    def updating(self, value):
        if type(value) is not bool:
            raise TypeError("updating must be a bool")
        self._updating = value

    @property
    def sampling_mode(self):
        return self._sampling_mode

    @sampling_mode.setter
    def sampling_mode(self, value):
        value = _integer(value)
        if value not in (0, 1):
            raise ValueError("sampling_mode must be 0 (robot) or 1 (box)")
        self._sampling_mode = value

    @property
    def point_shape_type(self):
        return int(self._point_cloud.getPointShapeType())

    @point_shape_type.setter
    def point_shape_type(self, value):
        value = _integer(value)
        modes = (
            dart.dynamics.PointCloudShape.BOX,
            dart.dynamics.PointCloudShape.BILLBOARD_SQUARE,
            dart.dynamics.PointCloudShape.BILLBOARD_CIRCLE,
            dart.dynamics.PointCloudShape.POINT,
        )
        if not 0 <= value < len(modes):
            raise ValueError("point_shape_type must be between 0 and 3")
        self._point_cloud.setPointShapeType(modes[value])

    def update(self, time_step):
        time_step = _real(time_step)
        if not math.isfinite(time_step) or time_step <= 0:
            raise ValueError("time_step must be positive and finite")
        if not self._updating:
            return
        positions = self._robot.getPositions().copy()
        for index in range(len(positions)):
            positions[index] += 0.01 * dart.math.Random.uniform(-1.0, 1.0)
        self._robot.setPositions(positions)
        sampled = (
            _robot_points(self._robot)
            if self._sampling_mode == 0
            else _box_points(_NUM_POINTS)
        )
        points = np.asarray(sampled, dtype=np.float32).astype(float)
        phase = self._orbit_time
        doubled = phase * 2.0
        origin = np.array(
            [
                math.sin(phase) if math.isfinite(phase) else math.nan,
                math.cos(phase) if math.isfinite(phase) else math.nan,
                0.5 + 0.25 * math.sin(doubled) if math.isfinite(doubled) else math.nan,
            ]
        )
        # Joint motion precedes preflight; all render/voxel mutations follow it.
        self._voxel_grid.validate_occupancy_update(points, origin)
        self._orbit_time += time_step
        self._sensor_frame.setTranslation(origin)
        self._point_cloud.setPoint(points)
        self._point_cloud.setColors(_colors(points))
        self._voxel_grid.updateOccupancy(points, origin)
