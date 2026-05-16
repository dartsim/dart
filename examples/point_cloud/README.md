# Point Cloud Example

## Summary

- Goal: sample points from the KR5 robot mesh or a box volume and visualize
  the resulting point cloud, sensor origin, and voxel occupancy.
- Concepts/APIs: `dart::gui::ApplicationOptions`, `PointCloudShape`,
  `VoxelGridShape`, source-owned DART worlds, and renderer-neutral panels.
- Expected output: the KR5 robot and ground with blue point samples, an orange
  voxel grid when OctoMap is available, a red moving sensor sphere, and a
  small scene grid.

## Run In Tree

From the repository root:

```bash
pixi run ex point_cloud
```

Useful promoted runner options:

```bash
pixi run ex point_cloud --headless --frames 2 --screenshot /tmp/point_cloud.ppm
pixi run ex point_cloud --width 960 --height 540 --gui-scale 1.25
```

The example defaults to a 1280x720 viewer and preserves command-line overrides
handled by the shared `dart::gui` runner.

## Controls

- `Play`, `Pause`, `Step`, and `Exit` control simulation from the panel.
- `Run Robot Updating` toggles the live point-cloud update loop.
- `Sample on robot` and `Sample in box` switch the sample source.
- The panel can show or hide the point cloud, sensor origin, scene grid, and
  voxel grid when OctoMap support is enabled.
- `Cycle Color Mode`, `Cycle Point Shape Type`, and `Visual Size` adjust the
  point-cloud rendering through public DART shape APIs.
