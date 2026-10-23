"""Point-cloud sampling and voxel occupancy on a moving six-axis arm."""

from __future__ import annotations

import dartpy as dart
import numpy as np

from ..models.point_cloud import PointCloudSensor
from .common import add_grid_editor


def build() -> dart.viewer.SceneSetup:
    world = dart.simulation.World()
    world.setGravity([0, 0, 0])
    loader = dart.utils.DartLoader()
    robot = loader.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    if robot is None:
        raise RuntimeError("failed to load dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
    robot.getJoint(0).setTransformFromParentBodyNode(dart.math.Isometry3())
    world.addSkeleton(robot)
    ground = loader.parseSkeleton("dart://sample/urdf/six_axis_arm/ground.urdf")
    if ground is None:
        raise RuntimeError("failed to load dart://sample/urdf/six_axis_arm/ground.urdf")
    world.addSkeleton(ground)

    point_cloud = dart.dynamics.PointCloudShape()
    point_cloud.setPointShapeType(dart.dynamics.PointCloudShape.BOX)
    point_frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "point cloud")
    point_frame.setShape(point_cloud)
    point_visual = point_frame.createVisualAspect()
    point_visual.setRGB([0.1, 0.1, 0.9])
    world.addSimpleFrame(point_frame)
    point_cloud.setColorMode(dart.dynamics.PointCloudShape.BIND_PER_POINT)

    voxel_grid = dart.dynamics.VoxelGridShape(0.05)
    voxel_frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "voxel")
    voxel_frame.setShape(voxel_grid)
    voxel_visual = voxel_frame.createVisualAspect()
    voxel_visual.setRGBA([1, 0.63, 0, 0.5])
    voxel_visual.show()
    world.addSimpleFrame(voxel_frame)

    sensor_frame = dart.dynamics.SimpleFrame(dart.dynamics.Frame.World(), "sensor")
    sensor_frame.setShape(dart.dynamics.SphereShape(0.05))
    sensor_frame.createVisualAspect().setRGB([0.9, 0.1, 0.1])
    world.addSimpleFrame(sensor_frame)
    sensor = PointCloudSensor(robot, sensor_frame, point_cloud, voxel_grid)
    grid = dart.gui.GridVisual()

    panel = dart.viewer.Panel()
    panel.add_text(
        "The robot moves by random joint velocities. Blue boxes are the "
        "point cloud; orange boxes are the voxel grid. The red sphere is "
        "the orbiting sensor origin.",
        wrap=True,
    )
    simulation = panel.add_group("Simulation")
    updating = simulation.add_checkbox("Run Robot Updating", sensor.updating)
    updating.on_change = lambda: setattr(sensor, "updating", updating.value)
    sampling = simulation.add_choice(
        "Sampling Mode",
        ["Sample on robot", "Sample in box"],
        index=sensor.sampling_mode,
    )
    sampling.on_change = lambda: setattr(sensor, "sampling_mode", sampling.value)

    view = panel.add_group("View")
    show_points = view.add_checkbox("Point Cloud", not point_visual.isHidden())
    show_points.on_change = lambda: (
        point_visual.show() if show_points.value else point_visual.hide()
    )
    color_modes = [
        dart.dynamics.PointCloudShape.USE_SHAPE_COLOR,
        dart.dynamics.PointCloudShape.BIND_OVERALL,
        dart.dynamics.PointCloudShape.BIND_PER_POINT,
    ]
    color_mode = view.add_choice(
        "Color Mode",
        ["Use shape color", "Bind overall", "Bind per point"],
        index=color_modes.index(point_cloud.getColorMode()),
    )
    color_mode.on_change = lambda: point_cloud.setColorMode(
        color_modes[color_mode.value]
    )
    point_color = view.add_color("Color##PointCloud", point_visual.getRGBA())
    point_color.on_change = lambda: point_visual.setColor(
        np.asarray(point_color.value, dtype=np.float32).astype(float)
    )
    point_type = view.add_choice(
        "Point Shape Type",
        ["Box", "Billboard Square", "Billboard Circle", "Point"],
        index=sensor.point_shape_type,
    )
    point_type.on_change = lambda: setattr(sensor, "point_shape_type", point_type.value)
    size = view.add_input(
        "Visual Size",
        float(np.float32(point_cloud.getVisualSize())),
        step=0.01,
        step_fast=0.02,
        decimals=2,
        min=float(np.float32(0.01)),
    )

    def change_size():
        if abs(size.value) > float(np.finfo(np.float32).max):
            size.value = float(np.float32(point_cloud.getVisualSize()))
            return
        point_cloud.setVisualSize(float(np.float32(size.value)))

    size.on_change = change_size
    view.add_separator()
    show_voxels = view.add_checkbox("Voxel Grid", not voxel_visual.isHidden())
    show_voxels.on_change = lambda: (
        voxel_visual.show() if show_voxels.value else voxel_visual.hide()
    )
    voxel_color = view.add_color("Voxel grid color", voxel_visual.getRGBA())
    voxel_color.on_change = lambda: voxel_visual.setColor(
        np.asarray(voxel_color.value, dtype=np.float32).astype(float)
    )
    refresh_grid = add_grid_editor(panel, grid)

    def pre_refresh():
        updating.value = sensor.updating
        sampling.value = sensor.sampling_mode
        show_points.value = not point_visual.isHidden()
        color_mode.value = color_modes.index(point_cloud.getColorMode())
        point_type.value = sensor.point_shape_type
        size.value = float(np.float32(point_cloud.getVisualSize()))
        point_color.value = point_visual.getRGBA()
        for control in (color_mode, point_type, size):
            control.visible = show_points.value
        point_color.visible = show_points.value and color_mode.value == 0
        show_voxels.value = not voxel_visual.isHidden()
        voxel_color.value = voxel_visual.getRGBA()
        voxel_color.visible = show_voxels.value
        refresh_grid()

    pre_refresh()
    return dart.viewer.SceneSetup(
        world,
        pre_step=lambda: sensor.update(world.getTimeStep()),
        pre_refresh=pre_refresh,
        on_activate=lambda ctx: ctx.add_attachment(grid),
        panel=panel,
        camera_home=dart.viewer.CameraHome(
            eye=[2.57, 3.14, 1.64], center=[0, 0, 0.30], up=[-0.24, -0.25, 0.94]
        ),
    )


def scenes():
    if not hasattr(dart.dynamics, "VoxelGridShape"):
        return []
    return [
        dart.viewer.Scene(
            "point_cloud",
            title="Point Cloud",
            category="Visualization",
            summary="Animated point-cloud and voxel-grid sensor rendering on a robot.",
            factory=build,
        )
    ]
