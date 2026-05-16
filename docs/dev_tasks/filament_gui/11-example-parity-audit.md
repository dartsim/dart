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

### Hello World Itemized Inventory

Historical source compared: `520993d7301^:examples/hello_world/main.cpp`.

The source-owned checkpoint restores the single dynamic blue box, gray ground,
gravity, collision, instruction text, camera/run defaults, profiling markers,
README, and promoted `ApplicationOptions::world` handoff through public
`dart::gui`.

| Historical item                                                                                  | Current outcome                      | Notes                                                                                                  |
| ------------------------------------------------------------------------------------------------ | ------------------------------------ | ------------------------------------------------------------------------------------------------------ |
| Create one blue dynamic box at `(0, 0, 1)` with size `(0.3, 0.3, 0.3)` and mass `1`.             | Restored through public `dart::gui`. | Current source owns the DART skeleton, collision, visual, and dynamics aspects.                        |
| Give the box a randomized initial orientation.                                                   | Restored with deterministic variant. | Current source uses a non-axis-aligned orientation while keeping promoted screenshots repeatable.      |
| Create a light gray 10x10 ground box.                                                            | Restored.                            | Current source owns the DART ground skeleton and collision/visual aspects.                             |
| Let the world simulate the box falling under gravity.                                            | Restored.                            | Current source sets gravity `(0, 0, -9.81)` and the shared runner handles Space pause/resume.          |
| Print/show `Press space to start free falling the box.` viewer instructions.                     | Restored.                            | Current source emits the text to the console and in a promoted panel.                                  |
| Enable default OSG shadow setup.                                                                 | Superseded by promoted renderer.     | Filament configures maintained scene lighting/shadows globally; do not expose backend shadow settings. |
| Default launch size 640x480.                                                                     | Restored.                            | Current source uses `ApplicationOptions::runDefaults`.                                                 |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0.50)` with up `(-0.24,-0.25,0.94)`. | Restored.                            | Current source converts the historical home to `dart::gui::OrbitCamera`.                               |
| Emit scoped profiling markers and text dump.                                                     | Restored.                            | Current source preserves `DART_PROFILE_SCOPED_N` and `DART_PROFILE_TEXT_DUMP`.                         |
| README documents the promoted runner and no longer says the standalone source remains OSG.       | Restored.                            | Current README documents `pixi run ex hello_world`, controls, and headless capture.                    |

### Capsule Ground Contact Itemized Inventory

Historical source compared:
`520993d7301^:examples/capsule_ground_contact/main.cpp`.

The source-owned checkpoint restores the capsule, ground plane, ODE preference
when available, keyboard reset/velocity controls, camera/run defaults, README,
and promoted panel controls through public `dart::gui`.

| Historical item                                                                                                 | Current outcome                           | Notes                                                                                                 |
| --------------------------------------------------------------------------------------------------------------- | ----------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Require/exercise ODE capsule-plane contact with persistent manifolds.                                           | Restored when ODE is available.           | Current source selects ODE under `#if DART_HAVE_ODE`; no backend-specific GUI path is needed.         |
| Create an infinite collision plane plus visible gray ground box.                                                | Restored through public `dart::gui`.      | Current source uses `PlaneShape` for collision and a DART visual box for the ground.                  |
| Create one blue dynamic capsule with radius `0.2`, height `0.6`, mass `1.0`, and free joint name/body metadata. | Restored.                                 | Current source restores the shape, mass, `capsule_joint`, and `capsule_body`.                         |
| Start the capsule in the historical horizontal pose.                                                            | Restored with a slight contact-safe lift. | Current source uses the historical horizontal orientation and a small lift to avoid initial clipping. |
| `h`/`H` resets to horizontal pose.                                                                              | Restored.                                 | Current source uses a promoted `KeyboardAction`.                                                      |
| `v`/`V` resets to vertical pose.                                                                                | Restored.                                 | Current source uses a promoted `KeyboardAction`.                                                      |
| Space clears capsule velocities.                                                                                | Restored with promoted key handling.      | Shared Space also toggles pause; current source also clears capsule velocities on the same key.       |
| Print controls and persistent-manifold explanation to the console.                                              | Restored.                                 | Current source prints the controls and also shows the persistent-manifold note in the panel.          |
| Default launch size 1024x768.                                                                                   | Restored.                                 | Current source uses `ApplicationOptions::runDefaults`.                                                |
| Camera home from eye `(2.5, 2.5, 1.5)` to target `(0, 0, 0.2)` with up `(-0.2,-0.2,0.95)`.                      | Restored.                                 | Current source converts the historical home to `dart::gui::OrbitCamera`.                              |
| README documents the promoted runner and no longer says the standalone source remains OSG.                      | Restored.                                 | Current README documents `pixi run ex capsule_ground_contact`, controls, and headless capture.        |

### Rigid Chain Itemized Inventory

Historical source compared: `520993d7301^:examples/rigid_chain/main.cpp`.

The source-owned checkpoint keeps the promoted `dart::gui` world handoff and
panel, and restores the historical random initial pose, pre-simulation damping
behavior, 640x480 default launch size, camera home, example README, and
source-marker guards through public APIs.

| Historical item                                                                                | Current outcome                      | Notes                                                                                     |
| ---------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/chain.skel`.                                                          | Restored through public `dart::gui`. | Current source owns the DART world and uses promoted `ApplicationOptions::world`.         |
| Use gravity `(0, -9.81, 0)` and timestep `1 / 2000`.                                           | Restored.                            | Current source sets both on the example world.                                            |
| Initialize every DOF with `dart::math::Random::uniform(-0.5, 0.5)`.                            | Restored.                            | Current source uses the same DART random helper as the historical example.                |
| Apply damping before each simulation step, with twist DOFs damped by an extra factor of `0.1`. | Restored through `preStep`.          | Current source uses promoted `ApplicationOptions::preStep` rather than an OSG world node. |
| Default launch size 640x480.                                                                   | Restored.                            | Current source uses `ApplicationOptions::runDefaults` so CLI overrides keep working.      |
| Camera home from eye `(2, 1, 2)` to target `(0, 0, 0)` with world-up `(0, 0, 1)`.              | Restored.                            | Current source converts the historical home to `dart::gui::OrbitCamera`.                  |
| README documents the promoted runner, controls, default size, and headless capture.            | Restored.                            | Current README documents `pixi run ex rigid_chain`, controls, and headless capture.       |
| Keep the example free of backend-specific world-node, event-handler, viewer, and OSG types.    | Restored through guard test.         | Marker coverage prevents reintroducing removed renderer surfaces or `defaultScene`.       |

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

### Empty Itemized Inventory

Historical source compared: `520993d7301^:examples/empty/main.cpp`.

The source-owned checkpoint restores a minimal selectable frame world,
renderer-neutral keydown callbacks, run defaults, camera home, and README. The
remaining gaps are public GUI API gaps for key-release callbacks and pre/post
render hooks; backend-specific event-handler types must stay out of the
example source.

| Historical item                                                                                   | Current outcome                             | Notes                                                                                                                             |
| ------------------------------------------------------------------------------------------------- | ------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| Create a world and add a target `InteractiveFrame` to it.                                         | Restored with promoted frame.               | Current source uses public `SimpleFrame`/selection affordances instead of OSG `InteractiveFrame`.                                 |
| Enable drag-and-drop for the target.                                                              | Restored by promoted runner.                | Current source and README describe Ctrl-left drag / keyboard nudge controls.                                                      |
| Demonstrate a custom world node with pre/post refresh and pre/post step hooks.                    | Partially restored; partial public API gap. | Current source preserves the `preStep` hook through `ApplicationOptions::preStep`; pre/post render and post-step hooks need APIs. |
| Demonstrate custom keydown handling for `q`, `Q`, Left, and Right.                                | Restored.                                   | Current source uses renderer-neutral `KeyboardAction` callbacks.                                                                  |
| Demonstrate custom keyup handling for `q`, `Q`, Left, and Right.                                  | Public API gap.                             | Requires a renderer-neutral key-release callback surface; do not expose backend event types.                                      |
| Default launch size 640x480.                                                                      | Restored.                                   | Current source uses `ApplicationOptions::runDefaults`.                                                                            |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0)` with up `(-0.24, -0.25, 0.94)`.   | Restored.                                   | Current source converts the historical home to `dart::gui::OrbitCamera`.                                                          |
| README documents the promoted runner and no longer says the standalone source remains OSG/legacy. | Restored.                                   | Current README documents `pixi run ex empty`, headless capture, promoted controls, and the remaining public API gaps.             |

### Simulation Event Handler Itemized Inventory

Historical sources compared:
`520993d7301^:examples/simulation_event_handler/main.cpp`,
`520993d7301^:examples/simulation_event_handler/simulation_event_handler.cpp`,
and `520993d7301^:examples/simulation_event_handler/README.md`.

The source-owned checkpoint restores the historical event-handler controls
through promoted `dart::gui` keyboard actions and DART-owned line geometry,
without reviving OSG viewer/event types. The only deliberate substitution is
that `h` is the promoted help key until shifted slash is represented as a
first-class keyboard shortcut.

| Historical item                                                                                                     | Current outcome                      | Notes                                                                                                                               |
| ------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- |
| Create ground, three dynamic boxes, and a dynamic sphere at the historical positions, sizes, and colors.            | Restored through public `dart::gui`. | Current source owns the DART world and uses promoted `ApplicationOptions::world`.                                                   |
| Use gravity `(0, 0, -9.81)` and timestep `0.001`.                                                                   | Restored.                            | Current source sets both on the example world.                                                                                      |
| Attach fast and slow blinking sensors to the red box, update them at 30 Hz and 5 Hz, and show green/orange markers. | Restored.                            | Current source uses `dart::sensor::Sensor` and world-owned `SimpleFrame` markers.                                                   |
| Print and display the example purpose and sensor-rate instructions.                                                 | Restored.                            | Current panel and console help include the historical purpose, controls, and sensor-rate text.                                      |
| Toggle simulation with Space, step with `s`, and reset with `r`.                                                    | Restored.                            | Space is restored by the shared runner; `s` and `r` are restored as public keyboard actions.                                        |
| Select next/previous rigid body with Tab and Backspace.                                                             | Restored.                            | Current source owns selected-body state for force/torque application.                                                               |
| Apply forces to the selected body with arrow keys plus `u`/`d`.                                                     | Restored.                            | Current source uses `KeyboardAction` callbacks calling `BodyNode::addExtForce`.                                                     |
| Apply torques to the selected body with `q`/`w`/`e`/`a`/`z`/`c`.                                                    | Restored.                            | Current source uses `KeyboardAction` callbacks calling `BodyNode::addExtTorque`.                                                    |
| Adjust force/torque magnitude with `+`/`=` and `-`/`_`.                                                             | Restored with promoted key handling. | Current source binds the underlying `=` and `-` keys; shifted punctuation routes through the same physical keys in the GLFW bridge. |
| Adjust timestep with `>`/`.` and `<`/`,` .                                                                          | Restored with promoted key handling. | Current source binds the underlying `.` and `,` keys and updates `World::setTimeStep`.                                              |
| Toggle force-arrow visualization with `v`.                                                                          | Restored.                            | Current source uses world-owned `LineSegmentShape` arrow visuals rather than OSG nodes.                                             |
| Print simulation state with `i` and help with `h`/`?`.                                                              | Restored except shifted slash alias. | Current source restores `i` and `h`; `?` needs a first-class shifted-slash shortcut if strict alias parity is needed later.         |
| Default launch size 1280x960.                                                                                       | Restored.                            | Current source uses `ApplicationOptions::runDefaults`.                                                                              |
| Camera home from eye `(5, 5, 3)` to target `(0, 0, 1)` with world-up `(0, 0, 1)`.                                   | Restored.                            | Current source converts the historical home to `dart::gui::OrbitCamera`.                                                            |
| README documents controls and promoted runner usage.                                                                | Restored.                            | Current README documents `pixi run ex simulation_event_handler` and headless capture.                                               |
| Keep the example free of backend-specific event-handler and viewer types.                                           | Restored through guard test.         | Continue to avoid OSG includes and private renderer types.                                                                          |

## Example Inventory

| Example                     | Current Audit State                                                                | Next Required Action                                                                           |
| --------------------------- | ---------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| `add_delete_skels`          | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm historical controls/defaults are itemized and guarded.                                 |
| `atlas_puppet`              | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, posture, and camera items are itemized.                         |
| `atlas_simbicon`            | Needs strict audit.                                                                | Compare historical source and list controls/defaults/panels.                                   |
| `biped_stand`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, camera, run defaults, README, and guards.                       |
| `box_stacking`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver controls, camera/defaults, README, and guards.                                  |
| `boxes`                     | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm Bullet preference, camera/defaults, README, and guards.                                |
| `capsule_ground_contact`    | Restored by strict audit.                                                          | Keep marker guards for controls, camera/defaults, README, and no backend types.                |
| `coupler_constraint`        | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                          |
| `csv_logger`                | Needs strict audit.                                                                | Confirm non-GUI logging behavior and README remain intact.                                     |
| `drag_and_drop`             | Needs strict audit.                                                                | Compare historical selection/drag affordances and README.                                      |
| `empty`                     | Restored except public key-release/render-hook API gaps.                           | Keep key-release and pre/post-render hooks tracked as public API follow-ups.                   |
| `fetch`                     | Itemized first-pass audit; panel-title/text gap repaired in current checkpoint.    | Keep re-openable if further historical behavior gaps are identified.                           |
| `free_joint_cases`          | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                          |
| `g1_puppet`                 | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, teleoperation, camera, and guards.                              |
| `gui_scene_diagnostics`     | Needs strict audit.                                                                | Confirm diagnostic GUI behavior and capture expectations.                                      |
| `hardcoded_design`          | Recent parity checkpoint; OSG wireframe remains public API gap.                    | Confirm checklist entry and keep wireframe follow-up explicit.                                 |
| `headless_simulation`       | Needs strict audit.                                                                | Confirm non-GUI simulation/capture behavior remains intact.                                    |
| `heightmap`                 | Restored except public debug-grid/color editor gaps.                               | Keep OSG grid style controls tracked as public API follow-up.                                  |
| `hello_world`               | Restored by strict audit.                                                          | Keep marker guards for instructions, camera/defaults, profiling, README, and no backend types. |
| `hubo_puppet`               | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK, teleoperation, target activation semantics, and guards.                            |
| `human_joint_limits`        | Needs strict audit.                                                                | Compare historical controls/defaults and visual diagnostics.                                   |
| `hybrid_dynamics`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm harness toggle, camera, and guards.                                                    |
| `imgui`                     | Needs strict audit.                                                                | Confirm promoted panel behavior preserves the user-facing ImGui example scope.                 |
| `joint_constraints`         | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, harness toggle, camera, and guards.                             |
| `lcp_physics`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver/collision controls, camera/defaults, README, and guards.                        |
| `mimic_pendulums`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm diagnostics, solver/collision flags, README, and guards.                               |
| `mixed_chain`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm impulse controls, camera, and guards.                                                  |
| `operational_space_control` | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK/control behavior, teleoperation, camera, and guards.                                |
| `point_cloud`               | Restored except public color-editor/debug-grid API gaps.                           | Keep color editors and fine-grained grid controls tracked as public API follow-ups.            |
| `polyhedron_visual`         | Restored by strict audit.                                                          | Keep marker guards for surface, wireframe, grid, camera/defaults, and README.                  |
| `rerun`                     | Needs strict audit.                                                                | Confirm non-Filament logging/viewer behavior remains correct.                                  |
| `rigid_chain`               | Restored by strict audit.                                                          | Keep marker guards for random pose, damping, camera/defaults, README, and no backend types.    |
| `rigid_cubes`               | Needs strict audit.                                                                | Confirm restored launcher captures historical cube scene behavior.                             |
| `rigid_loop`                | Needs strict audit.                                                                | Compare historical controls/defaults and camera.                                               |
| `rigid_shapes`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm spawn/delete/contact controls, CLI options, camera/defaults, README, and guards.       |
| `simple_frames`             | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm camera/defaults, README, and guards.                                                   |
| `simulation_event_handler`  | Restored except shifted-slash help alias follow-up.                                | Keep source-marker guards for controls, arrows, camera/defaults, README, and no backend types. |
| `soft_bodies`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm playback controls, shifted keys, camera/defaults, and guards.                          |
| `speed_test`                | Needs strict audit.                                                                | Confirm performance-test behavior and README remain intact.                                    |
| `tinkertoy`                 | Recent builder/keyboard/camera checkpoints; still subject to strict audit re-open. | Confirm all construction controls, force controls, camera home, and recording gap.             |
| `unified_loading`           | Needs strict audit.                                                                | Confirm model-loading workflows, CLI behavior, and README.                                     |
| `vehicle`                   | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm command keys, camera, and guards.                                                      |
| `wam_ikfast`                | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK target behavior, teleoperation, camera, and guards.                                 |
