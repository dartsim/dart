# Example Parity Audit

This file is the live checklist for restoring pre-existing examples after the
renderer promotion. It exists because source-owned `dart::gui` examples,
successful builds, runner coverage, and headless screenshots do not prove full
restoration.

## Restoration Standard

For each historical user-facing example:

1. Compare the current source against the pre-promotion historical source.
2. List the user-visible behaviors, controls, command-line options, camera/run
   defaults, panels, visual helpers, and README expectations from that source.
3. Mark each item as one of:
   - Restored through public `dart::gui`
   - Superseded by the promoted renderer/application direction
   - Public API gap requiring a follow-up
4. Only call the example restored when every historical item has one of those
   outcomes and the source-marker tests guard the important replacement
   behavior.

Backend-specific sample surfaces are treated separately:

- `examples/filament_gui` was a backend-named MVP surface and is replaced by
  the scoped `dartsim` application direction.
- `examples/raylib` is not restored because Raylib is not a maintained renderer
  path.

## Active Correction

`examples/fetch/` remains open as the concrete reminder case. Earlier
checkpoints restored MJCF loading, Bullet preference, robot/object initial
positions, mocap weld reset, target sync, visible target cross, work-area grid,
camera home, 1280x960 run defaults, GUI scaling, selection/drag/nudge
instructions, Play/Pause/Step/Exit panel controls, README, and source-marker
tests. The next action is still an itemized historical-source inventory before
declaring Fetch complete.

### Fetch Itemized Inventory

Historical source compared: `520993d7301^:examples/fetch/main.cpp`.

| Historical item                                                                                                              | Current outcome                                     | Notes                                                                                                                          |
| ---------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| Load `dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml`.                                                          | Restored through public `dart::gui`.                | Current source calls `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                           |
| Prefer Bullet collision for the Fetch world.                                                                                 | Restored through public `dart::gui` example source. | Current source selects `CollisionDetectorType::Bullet` when Bullet is available.                                               |
| Lock the Fetch root joint, set spring stiffness on all DOFs, stiffen torso lift, and restore the initial base/arm positions. | Restored.                                           | Current source keeps the historical numeric positions and stiffness values.                                                    |
| Restore the object initial position.                                                                                         | Restored.                                           | Current source keeps the historical object coordinates.                                                                        |
| Find `robot0:mocap`, reset the first weld constraint, and sync the mocap root to the interactive target before each step.    | Restored.                                           | Current source uses `ApplicationOptions::preStep` and `syncMocapTarget`.                                                       |
| Create an interactive target at `(1.3, 0.75, 0.50)` with a 90-degree Y rotation.                                             | Restored with promoted selection geometry.          | Current source uses a draggable `SimpleFrame` cross instead of OSG `InteractiveFrame`.                                         |
| Show the target as the visible end-effector control affordance.                                                              | Restored.                                           | Current source uses `LineSegmentShape` cross geometry.                                                                         |
| Add a work-area grid offset at `(1.3, 0.75, 0)`.                                                                             | Restored.                                           | Current source uses DART-owned line-segment geometry instead of OSG `GridVisual`.                                              |
| Provide an ImGui-scale option.                                                                                               | Restored through the promoted runner.               | `--gui-scale` is parsed by the shared `dart::gui` application runner.                                                          |
| Default window size 1280x960 while preserving CLI overrides.                                                                 | Restored.                                           | Current source uses `ApplicationOptions::runDefaults`.                                                                         |
| Camera home from eye `(4, 4, 2.5)` to target `(0.1, -0.3, 0.3)`.                                                             | Restored.                                           | Current source uses `ApplicationOptions::camera`.                                                                              |
| Show the historical panel title and instructional text.                                                                      | Restored.                                           | Current source uses the historical title, example description, whole-body motion explanation, and user-guide label.            |
| Expose Play/Pause simulation controls.                                                                                       | Restored.                                           | Current source uses `ViewerLifecycleState::paused` through `PanelContext`.                                                     |
| Expose Exit and About DART affordances.                                                                                      | Restored with promoted panel buttons.               | Historical menu-bar placement is superseded by the current renderer-neutral panel API.                                         |
| Show general viewer instructions/help.                                                                                       | Restored with promoted flat panel text.             | Current source includes selection/drag text; historical collapsible placement is superseded by the renderer-neutral panel API. |
| Enable drag-and-drop of the target.                                                                                          | Restored through promoted selection/drag controls.  | Current runner supports Ctrl-left drag plus keyboard nudge controls on selectable frames.                                      |
| README documents how to run the example.                                                                                     | Restored for promoted runner.                       | Current README documents `pixi run ex fetch` and headless capture.                                                             |

### Heightmap Itemized Inventory

Historical source compared: `520993d7301^:examples/heightmap/main.cpp`.

| Historical item                                                                                        | Current outcome                 | Notes                                                                                         |
| ------------------------------------------------------------------------------------------------------ | ------------------------------- | --------------------------------------------------------------------------------------------- |
| Launch `--demo interactive` by default and accept `--demo alignment`.                                  | Restored.                       | Current source parses the local demo selector before calling the promoted runner.             |
| Preserve `--gui-scale` through the promoted runner.                                                    | Restored through shared runner. | `--gui-scale` is parsed by `dart::gui::runApplication`.                                       |
| Default launch size 1280x720.                                                                          | Restored.                       | Current source uses `ApplicationOptions::runDefaults`.                                        |
| Interactive camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0.30)`.                        | Restored.                       | Current source converts the historical home to `dart::gui::OrbitCamera`.                      |
| Alignment camera home from eye `(5.2, 4.4, 2.3)` to target `(1.5, 0, 0.3)`.                            | Restored.                       | Current source converts the historical home to `dart::gui::OrbitCamera`.                      |
| Interactive heightmap uses a world-owned `SimpleFrame` with `HeightmapShape`.                          | Restored.                       | Current source owns a mutable `visual_heightmap` frame.                                       |
| Interactive panel title `Heightmap Demo` and description `Heightmap rendering example`.                | Restored.                       | Current source uses `dart::gui::Panel`.                                                       |
| Panel controls terrain visibility.                                                                     | Restored.                       | Current source uses `PanelBuilder::checkbox`.                                                 |
| Panel controls X/Y resolution, X/Y size, and Z min/max, then regenerates the height field.             | Restored.                       | Current source uses renderer-neutral sliders and rounds/clamps the resolution values.         |
| Panel exposes Play/Pause and Exit controls.                                                            | Restored.                       | Current source uses `PanelContext::lifecycle`, `requestSingleStep`, and `requestExit`.        |
| Grid is visible in interactive mode and offset slightly below the heightmap to avoid z-fighting.       | Restored.                       | Current source restores it as DART-owned line geometry rather than OSG `GridVisual`.          |
| Fine-grained grid plane, offset, line count, line step, minor/major line, width, and color controls.   | Public API gap.                 | Requires a DART-owned debug-grid API; do not revive OSG `GridVisual` or expose backend hooks. |
| Terrain color editor.                                                                                  | Public API gap.                 | Requires a DART-owned color-edit panel control; current source keeps a fixed terrain color.   |
| Alignment mode creates collision heightmap, reference box, and two ball grids.                         | Restored.                       | Current source owns the alignment world construction.                                         |
| Alignment mode prefers ODE collision when available, uses gravity `(0, 0, -9.81)`, and timestep 0.001. | Restored.                       | Current source restores the ODE preference under `#if DART_HAVE_ODE`.                         |
| README documents the promoted runner and both demo modes.                                              | Restored.                       | Current README documents default, alignment, and headless capture commands.                   |

### Point Cloud Itemized Inventory

Historical source compared: `520993d7301^:examples/point_cloud/main.cpp`.

The static fixture checkpoint has been replaced by a live source-owned
`dart::gui` example. The remaining gaps are public GUI API gaps for color-edit
widgets and fine-grained debug-grid controls; backend-specific ImGui, OSG, and
GridVisual concepts must stay out of the example source.

| Historical item                                                                                                    | Current outcome                                     | Notes                                                                                                                                |
| ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| Load `dart://sample/urdf/KR5/KR5 sixx R650.urdf` as `KR5` and reset its root transform.                            | Restored through public `dart::gui` example source. | Current source uses `dart::io::readSkeleton` and keeps a fallback visual skeleton for missing resources.                             |
| Load `dart://sample/urdf/KR5/ground.urdf`, translate it upward, and rotate it by `pi / 2` around X.                | Restored.                                           | Current source owns the historical ground transform and keeps a fallback ground.                                                     |
| Keep world gravity at zero.                                                                                        | Restored.                                           | Current source sets zero gravity and the historical 0.001 timestep.                                                                  |
| Create point-cloud, voxel-grid, and sensor frames.                                                                 | Restored.                                           | Current source preserves promoted marker names `visual_point_cloud`, `visual_voxel_grid`, and `point_cloud_sensor`.                  |
| Update the robot pose every step while robot updating is enabled.                                                  | Restored with deterministic motion.                 | Current source uses `ApplicationOptions::preStep`; deterministic motion replaces random joint perturbations for repeatable captures. |
| Support sample-on-robot and sample-in-box modes.                                                                   | Restored.                                           | Robot sampling reads mesh vertices from loaded KR5 visuals and falls back to the box sampler when mesh data is unavailable.          |
| Move the red sensor sphere on the historical circular path.                                                        | Restored.                                           | Current source updates the sensor `SimpleFrame` from the same circular path.                                                         |
| Update `PointCloudShape` points and per-point colors every step.                                                   | Restored.                                           | Current source uses public `PointCloudShape::setPoint` and `setColors`.                                                              |
| Update `VoxelGridShape` occupancy from the current point cloud and sensor position.                                | Restored when OctoMap is available.                 | Current source guards voxel-grid creation and occupancy updates with `#if DART_HAVE_OCTOMAP`.                                        |
| Panel title `Point Cloud & Voxel Grid Demo` and description `Point cloud and voxel grid rendering example`.        | Restored.                                           | Current source uses `dart::gui::Panel`.                                                                                              |
| Panel explanatory text describing random robot motion, blue/orange point/voxel boxes, and red sensor origin.       | Restored with deterministic wording.                | Current panel documents deterministic KR5 motion for promoted headless capture repeatability.                                        |
| Expose Play/Pause controls.                                                                                        | Restored.                                           | Current source uses `ViewerLifecycleState` via `PanelContext`.                                                                       |
| Expose Run/Stop Robot Updating controls.                                                                           | Restored.                                           | Current source uses a renderer-neutral checkbox.                                                                                     |
| Expose Sample on robot / Sample in box controls.                                                                   | Restored.                                           | Current source uses renderer-neutral panel buttons.                                                                                  |
| Expose point-cloud visibility, color mode, point shape type, and visual-size controls.                             | Restored.                                           | Current source uses visibility checkboxes, mode/type cycle buttons, and a visual-size slider.                                        |
| Expose point-cloud and voxel-grid color editors.                                                                   | Public API gap.                                     | Requires a DART-owned color-edit panel control; current source keeps fixed colors.                                                   |
| Expose voxel-grid visibility.                                                                                      | Restored.                                           | Current source restores it under `#if DART_HAVE_OCTOMAP` and shows an unavailable note when OctoMap is disabled.                     |
| Show a scene grid.                                                                                                 | Restored.                                           | Current source uses DART-owned line geometry named `point_cloud_grid` instead of OSG `GridVisual`.                                   |
| Expose fine-grained grid display, plane, offset, line count, line step, minor/major line, width, and color fields. | Public API gap.                                     | Requires a DART-owned debug-grid API and color/input panel controls; do not revive OSG `GridVisual` or backend-specific hooks.       |
| Preserve `--gui-scale` through the promoted runner.                                                                | Restored through runner.                            | `--gui-scale` is parsed by `dart::gui::runApplication`; no local backend parser owns the shared option.                              |
| Default launch size 1280x720.                                                                                      | Restored.                                           | Current source uses `ApplicationOptions::runDefaults`.                                                                               |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0.30)`.                                                | Restored.                                           | Current source converts the historical home to `dart::gui::OrbitCamera`.                                                             |
| README documents the promoted runner and no longer says the standalone source remains OSG/ImGui.                   | Restored.                                           | Current README documents `pixi run ex point_cloud`, headless capture, and the public `dart::gui` migration.                          |

### Polyhedron Visual Itemized Inventory

Historical source compared:
`520993d7301^:examples/polyhedron_visual/main.cpp`.

The source-owned checkpoint restores the convex surface, wireframe, scene-grid
replacement, run defaults, camera home, and README through DART shape
primitives and promoted `dart::gui` options.

| Historical item                                                                                  | Current outcome              | Notes                                                                                                                     |
| ------------------------------------------------------------------------------------------------ | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------- |
| Build a convex polyhedron from the historical eight vertex positions.                            | Restored.                    | Current source preserves the same V-representation in the source-owned DART mesh.                                         |
| Render a translucent green surface.                                                              | Restored.                    | Current source uses `ConvexMeshShape`.                                                                                    |
| Render a dark wireframe over the surface.                                                        | Restored.                    | Current source uses `LineSegmentShape`.                                                                                   |
| Add a viewer grid for reference.                                                                 | Restored.                    | Current source restores it as DART-owned line geometry named `visual_polyhedron_grid` instead of OSG `GridVisual`.        |
| Default launch size 640x480.                                                                     | Restored.                    | Current source uses `ApplicationOptions::runDefaults`.                                                                    |
| Camera home from eye `(2.0, 2.0, 1.5)` to target `(0, 0, 0.4)` with world-up `(0, 0, 1)`.        | Restored.                    | Current source converts the historical home to `dart::gui::OrbitCamera`.                                                  |
| README documents the promoted runner and no longer says the standalone source remains OSG/ImGui. | Restored.                    | Current README documents `pixi run ex polyhedron_visual`, headless capture, and the public `dart::gui` migration.         |
| Keep the example free of backend-specific viewer/visual types.                                   | Restored through guard test. | Current source uses `ApplicationOptions`, DART shape primitives, and `dart::gui::runApplication`, with no backend tokens. |

## Example Inventory

| Example                     | Current Audit State                                                                | Next Required Action                                                                     |
| --------------------------- | ---------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `add_delete_skels`          | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm historical controls/defaults are itemized and guarded.                           |
| `atlas_puppet`              | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, posture, and camera items are itemized.                   |
| `atlas_simbicon`            | Needs strict audit.                                                                | Compare historical source and list controls/defaults/panels.                             |
| `biped_stand`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, camera, run defaults, README, and guards.                 |
| `box_stacking`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver controls, camera/defaults, README, and guards.                            |
| `boxes`                     | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm Bullet preference, camera/defaults, README, and guards.                          |
| `capsule_ground_contact`    | Needs strict audit.                                                                | Compare historical source and list visual/default behavior.                              |
| `coupler_constraint`        | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                    |
| `csv_logger`                | Needs strict audit.                                                                | Confirm non-GUI logging behavior and README remain intact.                               |
| `drag_and_drop`             | Needs strict audit.                                                                | Compare historical selection/drag affordances and README.                                |
| `empty`                     | Needs strict audit.                                                                | Confirm historical minimal-run behavior and defaults.                                    |
| `fetch`                     | Itemized first-pass audit; panel-title/text gap repaired in current checkpoint.    | Keep re-openable if further historical behavior gaps are identified.                     |
| `free_joint_cases`          | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                    |
| `g1_puppet`                 | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, teleoperation, camera, and guards.                        |
| `gui_scene_diagnostics`     | Needs strict audit.                                                                | Confirm diagnostic GUI behavior and capture expectations.                                |
| `hardcoded_design`          | Recent parity checkpoint; OSG wireframe remains public API gap.                    | Confirm checklist entry and keep wireframe follow-up explicit.                           |
| `headless_simulation`       | Needs strict audit.                                                                | Confirm non-GUI simulation/capture behavior remains intact.                              |
| `heightmap`                 | Restored except public debug-grid/color editor gaps.                               | Keep OSG grid style controls tracked as public API follow-up.                            |
| `hello_world`               | Needs strict audit.                                                                | Confirm historical camera/default behavior and README.                                   |
| `hubo_puppet`               | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK, teleoperation, target activation semantics, and guards.                      |
| `human_joint_limits`        | Needs strict audit.                                                                | Compare historical controls/defaults and visual diagnostics.                             |
| `hybrid_dynamics`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm harness toggle, camera, and guards.                                              |
| `imgui`                     | Needs strict audit.                                                                | Confirm promoted panel behavior preserves the user-facing ImGui example scope.           |
| `joint_constraints`         | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, harness toggle, camera, and guards.                       |
| `lcp_physics`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver/collision controls, camera/defaults, README, and guards.                  |
| `mimic_pendulums`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm diagnostics, solver/collision flags, README, and guards.                         |
| `mixed_chain`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm impulse controls, camera, and guards.                                            |
| `operational_space_control` | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK/control behavior, teleoperation, camera, and guards.                          |
| `point_cloud`               | Restored except public color-editor/debug-grid API gaps.                           | Keep color editors and fine-grained grid controls tracked as public API follow-ups.      |
| `polyhedron_visual`         | Restored by strict audit.                                                          | Keep marker guards for surface, wireframe, grid, camera/defaults, and README.            |
| `rerun`                     | Needs strict audit.                                                                | Confirm non-Filament logging/viewer behavior remains correct.                            |
| `rigid_chain`               | Needs strict audit.                                                                | Compare historical controls/defaults and camera.                                         |
| `rigid_cubes`               | Needs strict audit.                                                                | Confirm restored launcher captures historical cube scene behavior.                       |
| `rigid_loop`                | Needs strict audit.                                                                | Compare historical controls/defaults and camera.                                         |
| `rigid_shapes`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm spawn/delete/contact controls, CLI options, camera/defaults, README, and guards. |
| `simple_frames`             | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm camera/defaults, README, and guards.                                             |
| `simulation_event_handler`  | Needs strict audit.                                                                | Compare historical event-handler controls/defaults.                                      |
| `soft_bodies`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm playback controls, shifted keys, camera/defaults, and guards.                    |
| `speed_test`                | Needs strict audit.                                                                | Confirm performance-test behavior and README remain intact.                              |
| `tinkertoy`                 | Recent builder/keyboard/camera checkpoints; still subject to strict audit re-open. | Confirm all construction controls, force controls, camera home, and recording gap.       |
| `unified_loading`           | Needs strict audit.                                                                | Confirm model-loading workflows, CLI behavior, and README.                               |
| `vehicle`                   | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm command keys, camera, and guards.                                                |
| `wam_ikfast`                | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK target behavior, teleoperation, camera, and guards.                           |
