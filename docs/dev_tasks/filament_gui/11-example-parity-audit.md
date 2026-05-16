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
positions, mocap weld reset, target sync, visible target affordance,
work-area grid, camera home, 1280x960 run defaults, GUI scaling,
selection/drag/nudge instructions, Play/Pause/Step/Exit panel controls,
README, and source-marker tests. That evidence does not close Fetch if another
historical user-visible behavior is found. Treat Fetch as re-openable until a
fresh comparison against `520993d7301^:examples/fetch/main.cpp` confirms that
the remaining differences are either public `dart::gui` restorations,
official-renderer supersessions, or explicit public API gaps.

The broader correction is documented in `12-strict-example-restoration.md`:
every pre-existing user-facing example, including ones with recent restoration
commits, remains subject to strict re-open until this file has an itemized
historical-source inventory for it.

2026-05-16 follow-up after the human joint-limits checkpoint: maintainer
steering again called out that many examples remain incompletely restored, with
`examples/fetch/` as the representative case. The active Fetch re-open now
focuses on behavior that was easy to undercount in earlier source-owned passes:
the old interactive target exposed a full manipulation affordance, not only a
visible marker, and the old help panel surfaced the shared viewer instructions.

2026-05-16 follow-up after the unified-loading checkpoint: Fetch is re-opened
again because the current source still documents exact mouse rotation as a
public API gap. The active repair scope is to promote a renderer-neutral
selected-frame rotation path through `dart::gui`, use it from the shared
Filament/GLFW selection controller, and then update Fetch so Ctrl-Shift-left
drag rotates the selected target with X/Y/Z local-axis constraints. Until that
lands and validates locally, Fetch is not complete.

Implementation update: the current source adds public selected-frame rotation
helpers, wires Ctrl-Shift-left drag through the Filament/GLFW selection
controller, and updates Fetch help/README/tests to describe mouse rotation as
implemented renderer-neutral manipulation.

### Fetch Itemized Inventory

Historical source compared: `520993d7301^:examples/fetch/main.cpp`.

| Historical item                                                                                                              | Current outcome                                     | Notes                                                                                                                                                                                                               |
| ---------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml`.                                                          | Restored through public `dart::gui`.                | Current source calls `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                                                                                                                |
| Prefer Bullet collision for the Fetch world.                                                                                 | Restored through public `dart::gui` example source. | Current source selects `CollisionDetectorType::Bullet` when Bullet is available.                                                                                                                                    |
| Lock the Fetch root joint, set spring stiffness on all DOFs, stiffen torso lift, and restore the initial base/arm positions. | Restored.                                           | Current source keeps the historical numeric positions and stiffness values.                                                                                                                                         |
| Restore the object initial position.                                                                                         | Restored.                                           | Current source keeps the historical object coordinates.                                                                                                                                                             |
| Find `robot0:mocap`, reset the first weld constraint, and sync the mocap root to the interactive target before each step.    | Restored.                                           | Current source uses `ApplicationOptions::preStep` and `syncMocapTarget`.                                                                                                                                            |
| Create an interactive target at `(1.3, 0.75, 0.50)` with a 90-degree Y rotation.                                             | Restored through public `dart::gui`.                | Current source uses one draggable `SimpleFrame`, a richer source-owned line handle, keyboard rotation/reset actions, and mouse-driven selected-frame rotation through public `dart::gui`.                           |
| Show the target as the visible end-effector control affordance.                                                              | Restored through public DART geometry.              | Current source uses one transparent green `LineSegmentShape` target handle with local axes and rings so selecting the visible affordance moves the same frame used by mocap sync.                                   |
| Add a work-area grid offset at `(1.3, 0.75, 0)`.                                                                             | Restored.                                           | Current source uses DART-owned line-segment geometry instead of OSG `GridVisual`.                                                                                                                                   |
| Provide an ImGui-scale option.                                                                                               | Restored through the promoted runner.               | `--gui-scale` is parsed by the shared `dart::gui` application runner.                                                                                                                                               |
| Default window size 1280x960 while preserving CLI overrides.                                                                 | Restored.                                           | Current source uses `ApplicationOptions::runDefaults`.                                                                                                                                                              |
| Camera home from eye `(4, 4, 2.5)` to target `(0.1, -0.3, 0.3)`.                                                             | Restored.                                           | Current source uses `ApplicationOptions::camera`.                                                                                                                                                                   |
| Show the historical panel title and instructional text.                                                                      | Restored.                                           | Current source uses the historical title, example description, whole-body motion explanation, and user-guide label.                                                                                                 |
| Expose Play/Pause simulation controls.                                                                                       | Restored.                                           | Current source uses `ViewerLifecycleState::paused` through `PanelContext`.                                                                                                                                          |
| Expose Exit and About DART affordances.                                                                                      | Restored with promoted panel buttons.               | Historical menu-bar placement is superseded by the current renderer-neutral panel API.                                                                                                                              |
| Show general viewer instructions/help.                                                                                       | Restored with promoted flat panel text.             | Current source includes selection/drag, target rotation/reset, orbit/pan/zoom, pause/step, screenshot/capture, and exit guidance. Historical collapsible placement is superseded by the renderer-neutral panel API. |
| Enable drag-and-drop of the target.                                                                                          | Restored through public `dart::gui`.                | Current runner supports Ctrl-left drag plus keyboard nudge controls on selectable frames and Ctrl-Shift-left rotation for selected `SimpleFrame`s, with X/Y/Z local-axis rotation constraints.                      |
| README documents how to run the example.                                                                                     | Restored for promoted runner.                       | Current README documents `pixi run ex fetch` and headless capture.                                                                                                                                                  |
| Keep Fetch open if another historical behavior gap is identified.                                                            | Active strict-audit rule.                           | Do not treat the first-pass inventory as final without re-checking the current source against the old OSG/ImGui source.                                                                                             |

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

### Rigid Loop Itemized Inventory

Historical source compared: `520993d7301^:examples/rigid_loop/main.cpp`.

The source-owned checkpoint keeps the promoted `dart::gui` world handoff and
pre-step damping behavior, and restores the exact red constrained link colors,
console instructions, 640x480 default launch size, README, and source-marker
guards through public APIs.

| Historical item                                                                                | Current outcome                      | Notes                                                                                     |
| ---------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/chain.skel`.                                                          | Restored through public `dart::gui`. | Current source owns the DART world and uses promoted `ApplicationOptions::world`.         |
| Use gravity `(0, -9.81, 0)` and timestep `1 / 2000`.                                           | Restored.                            | Current source sets both on the example world.                                            |
| Initialize DOFs 20, 23, 26, and 29 to `0.4 * pi`, with other DOFs zero.                        | Restored.                            | Current source preserves the historical loop pose.                                        |
| Add a ball-joint constraint between `link 6` and `link 10` at offset `(0, 0.025, 0)`.          | Restored.                            | Current source uses `BallJointConstraint` through the promoted world handoff.             |
| Color `link 6` and `link 10` exactly red.                                                      | Restored.                            | Current source uses the exact historical red color for both constrained links.            |
| Apply damping before each simulation step, with twist DOFs damped by an extra factor of `0.1`. | Restored through `preStep`.          | Current source uses promoted `ApplicationOptions::preStep` rather than an OSG world node. |
| Print the historical example title, closed-loop explanation, Space, and Escape instructions.   | Restored.                            | Current source prints the historical console instructions and keeps panel controls.       |
| Default launch size 640x480.                                                                   | Restored.                            | Current source uses `ApplicationOptions::runDefaults` so CLI overrides keep working.      |
| README documents the promoted runner, controls, default size, and headless capture.            | Restored.                            | Current README documents `pixi run ex rigid_loop`, controls, and headless capture.        |
| Keep the example free of backend-specific world-node, event-handler, viewer, and OSG types.    | Restored through guard test.         | Marker coverage prevents reintroducing removed renderer surfaces or `defaultScene`.       |

### Rigid Cubes Itemized Inventory

Historical source compared: `520993d7301^:examples/rigid_cubes/main.cpp`.

The source-owned checkpoint replaces the hand-built three-cube scene with the
historical `cubes.skel` world, restores the directional force keys with force
decay, recovers the historical gravity/camera/defaults and instruction text,
and keeps headless capture on the promoted `dart::gui` runner instead of
reintroducing OSG viewer code.

| Historical item                                                                                       | Current outcome              | Notes                                                                                            |
| ----------------------------------------------------------------------------------------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------ |
| Load `dart://sample/skel/cubes.skel`.                                                                 | Restored.                    | Current source loads the shipped sample world with `dart::io::readWorld`.                        |
| Use gravity `(0, -9.81, 0)`.                                                                          | Restored.                    | Current source restores the historical Y-down world setting.                                     |
| Apply a decaying force to skeleton index 1, body node 0 before each simulation step.                  | Restored through `preStep`.  | Current source stores pending force and halves it after every promoted pre-step callback.        |
| Space toggles simulation through the viewer, `p` toggles playback/default event handling.             | Restored/superseded.         | Space is shared runner behavior; `p` maps to the promoted lifecycle toggle without OSG handlers. |
| `v` toggles visualization marker state and prints a status line.                                      | Restored.                    | Current source restores renderer-neutral marker state and console feedback.                      |
| `1`/`2`/`3`/`4` apply `-X`, `+X`, `-Z`, and `+Z` forces of magnitude 500.                             | Restored.                    | Current source restores all four keys and matching panel buttons.                                |
| Default launch size 640x480 and CLI/headless capture options.                                         | Restored.                    | Current source uses `ApplicationOptions::runDefaults`; promoted runner owns capture arguments.   |
| Camera home from eye `(5, 5, 5)` to target `(0, 0, 0)` with world-up `(0, 0, 1)`.                     | Restored.                    | Current source converts the historical home to `dart::gui::OrbitCamera`.                         |
| Print and display the historical title and controls.                                                  | Restored.                    | Current source prints instructions and keeps panel instruction coverage.                         |
| README documents the promoted runner, controls, default size, and headless/image-sequence capture.    | Restored.                    | Current README documents screenshot and `--out` image-sequence capture.                          |
| Keep the example free of backend-specific world-node, event-handler, viewer, and OSG recording types. | Restored through guard test. | Marker coverage prevents reintroducing removed renderer surfaces or `defaultScene`.              |

### Coupler Constraint Itemized Inventory

Historical source compared:
`520993d7301^:examples/coupler_constraint/main.cpp`.

The source-owned checkpoint restores the historical dynamic comparison through
public `dart::gui` pre-step, keyboard, panel, camera, and run-default APIs. The
old OSG world-node/event-handler/viewer and direct ImGui types stay removed.

| Historical item                                                                                                                  | Current outcome                      | Notes                                                                                                        |
| -------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------ |
| Build left `coupler` and right `motor` mimic assemblies with matching offsets, rod colors, joint limits, and mimic setup.        | Restored.                            | Current source restores the assemblies, colors, limits, zero initial positions, and mimic/coupler settings.  |
| Use zero gravity and timestep `1e-3`.                                                                                            | Restored.                            | Current source sets both on the promoted `World`.                                                            |
| Compare left bilateral `CouplerConstraint` behavior against right legacy `MimicMotorConstraint` behavior.                        | Restored.                            | Current source restores the controller-driven comparison and status panel.                                   |
| Drive both reference joints toward a 45-degree command with torque limit 90, proportional gain 320, and damping gain 25.         | Restored through public `dart::gui`. | Current source uses a source-owned controller from `ApplicationOptions::preStep`.                            |
| Refresh the reference/follower link lines every frame and color them by mimic error severity.                                    | Restored.                            | Current source updates DART-owned line geometry and visual colors from the controller.                       |
| Reset both rigs to their initial positions, zero velocities, and clear external forces with `r`.                                 | Restored.                            | Current source uses promoted `KeyboardAction` instead of OSG event handlers, and keeps a panel reset button. |
| Show live status for target angle, reference position, follower position, limits, desired mimic position, error, and saturation. | Restored.                            | Current source uses the promoted flat panel API; exact ImGui window flags are superseded.                    |
| Print the historical explanation and controls to the console.                                                                    | Restored.                            | Current source keeps ASCII wording while preserving the old meaning.                                         |
| Preserve `--gui-scale`.                                                                                                          | Restored through shared runner.      | The promoted runner parses shared GUI scale options; no local backend parser should return.                  |
| Add the historical XY grid with 25 cells and 0.05 minor step.                                                                    | Restored.                            | Current source uses source-owned DART line geometry instead of OSG `GridVisual`.                             |
| Default launch size 960x720.                                                                                                     | Restored.                            | Current source uses `ApplicationOptions::runDefaults`.                                                       |
| Camera home from eye `(1.5, 1.5, 1.2)` to target `(0.4, 0, 0.2)` with world-up `(0, 0, 1)`.                                      | Restored.                            | Current source converts the historical home to `dart::gui::OrbitCamera`.                                     |
| README documents the promoted runner, controls, default size, and capture.                                                       | Restored.                            | Current README documents `pixi run ex coupler_constraint`, controls, defaults, and headless capture.         |
| Keep the source free of OSG world-node, event-handler, viewer, and direct ImGui types.                                           | Restored through guard test.         | Marker coverage prevents reintroducing removed renderer surfaces.                                            |

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

### Drag and Drop Itemized Inventory

Historical sources compared:
`520993d7301^:examples/drag_and_drop/main.cpp` and
`520993d7301^:examples/drag_and_drop/README.md`.

The strict-audit slice restores the standalone educational example as a
source-owned `dart::gui` world. The old OSG `InteractiveFrame`, viewer, world
node, and direct drag-and-drop hooks stay removed; public `dart::gui` selection
and frame translation replace the draggable-frame path. True
InteractiveFrame-style rotation handles remain a named public API gap.

| Historical item                                                                                | Current outcome                       | Notes                                                                                                          |
| ---------------------------------------------------------------------------------------------- | ------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Create an interactive frame named `interactive frame` at `(4, -4, 0)` with scale `2.0`.        | Restored with promoted frame visuals. | Current source uses one selectable `SimpleFrame` and source-owned DART line geometry for the visible handle.   |
| Attach the red `draggable` box as a child frame at local `(-4, 4, 0)` with unit box geometry.  | Restored.                             | Current source preserves the parent/child transform relationship and red visual.                               |
| Add world X/Y/Z marker boxes at `(8, 0, 0)`, `(0, 8, 0)`, and `(0, 0, 8)` with size `0.2`.     | Restored.                             | Current source keeps the historical positions, names, sizes, and red/green/blue colors.                        |
| Enable drag-and-drop on the interactive frame and child box.                                   | Restored through promoted selection.  | The shared runner supports click selection, Ctrl-left drag, X/Y/Z axis-constrained drag, and keyboard nudging. |
| Show the historical visible manipulation affordance rather than only a tiny placeholder box.   | Restored as DART geometry.            | Current source adds an orthogonal line-handle gizmo on the selectable frame.                                   |
| Support the historical Ctrl-left rotation manipulation.                                        | Public API gap.                       | Requires a renderer-neutral transform/rotation manipulator API; do not reintroduce OSG `InteractiveFrame`.     |
| Print/show the example interaction instructions.                                               | Restored with promoted wording.       | Current source explains selection, Ctrl-left drag, axis constraints, nudging, and the rotation API gap.        |
| Default launch size 640x480.                                                                   | Restored.                             | Current source uses `ApplicationOptions::runDefaults`.                                                         |
| Camera home from eye `(20, 17, 17)` to target `(0, 0, 0)` with world-up `(0, 0, 1)`.           | Restored.                             | Current source converts the historical home to `dart::gui::OrbitCamera`.                                       |
| README documents the promoted runner and no longer calls the standalone source legacy OSG.     | Restored.                             | Current README documents `pixi run ex drag_and_drop`, controls, defaults, headless capture, and the API gap.   |
| Keep the source free of OSG viewer, world-node, InteractiveFrame, and backend-specific tokens. | Restored through guard test.          | Marker coverage prevents reintroducing removed renderer surfaces.                                              |

### Human Joint Limits Itemized Inventory

Historical sources compared:
`520993d7301^:examples/human_joint_limits/main.cpp`,
`520993d7301^:examples/human_joint_limits/README.md`,
`520993d7301^:examples/human_joint_limits/human_arm_joint_limit_constraint.*`,
and
`520993d7301^:examples/human_joint_limits/human_leg_joint_limit_constraint.*`.

The strict-audit slice restores the example as a live `dart::gui` world where
the current public dependency stack allows it. The historical TinyDNN-backed
custom arm/leg constraints are not silently replaced by a visual-only scene;
they remain a named dependency/API follow-up until DART has a maintained
replacement for that neural-network constraint path.

| Historical item                                                                                   | Current outcome                   | Notes                                                                                                             |
| ------------------------------------------------------------------------------------------------- | --------------------------------- | ----------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/kima/kima_human_edited.skel`.                                            | Restored.                         | Current source uses `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.               |
| Preserve the SKEL world timestep and Y-down gravity.                                              | Restored.                         | Current source no longer converts the human into a zero-gravity visual-only fixture.                              |
| Enable `Joint::setLimitEnforcement(true)` on every human joint.                                   | Restored.                         | Current source keeps the historical DART joint-limit enforcement pass.                                            |
| Install left/right `HumanArmJointLimitConstraint` and `HumanLegJointLimitConstraint` instances.   | Dependency/API follow-up.         | Requires a maintained TinyDNN dependency or replacement public neural-network constraint implementation.          |
| Step the world through the viewer and let Space toggle simulation.                                | Restored through promoted runner. | Current source uses the shared runner lifecycle instead of an OSG `WorldNode`; Space remains shared runner input. |
| Print `space bar: simulation on/off` and show the example purpose.                                | Restored.                         | Current source prints the historical control and mirrors the purpose in the promoted panel.                       |
| Default launch size 640x480.                                                                      | Restored.                         | Current source uses `ApplicationOptions::runDefaults`.                                                            |
| README documents the promoted runner and no longer says the source is legacy OSG comparison code. | Restored.                         | Current README documents controls, capture, and the TinyDNN/custom-constraint follow-up.                          |
| Keep the source free of OSG viewer, world-node, event-handler, and backend-specific tokens.       | Restored through guard test.      | Marker coverage prevents reintroducing removed renderer surfaces.                                                 |

### Panel Extension Example Itemized Inventory

Historical source compared: `520993d7301^:examples/imgui/main.cpp`.

The current source still behaves like a generic box/slider panel demo. The
strict restoration target is the historical panel-extension sample, migrated to
public `dart::gui` without direct backend UI or OSG event types.

| Historical item                                                                                        | Current outcome                           | Notes                                                                                                                                                     |
| ------------------------------------------------------------------------------------------------------ | ----------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Parse `--gui-scale`.                                                                                   | Restored through the promoted runner.     | Shared `dart::gui::runApplication` owns the GUI-scale option.                                                                                             |
| Create an empty world and add a target frame.                                                          | Restored through public `dart::gui`.      | Current source creates an empty world and one selectable `SimpleFrame` target with source-owned line handle geometry.                                     |
| Enable drag-and-drop for the target.                                                                   | Restored through promoted controls.       | Shared selection/drag controls move the visible target frame without legacy interaction types.                                                            |
| Demonstrate custom pre-step hook behavior.                                                             | Restored.                                 | Current source uses `ApplicationOptions::preStep` and displays a pre-step callback count.                                                                 |
| Demonstrate custom pre/post refresh and post-step hooks.                                               | Public API gap.                           | Requires renderer-neutral render-hook and post-step callback APIs; do not revive world-node types.                                                        |
| Demonstrate keydown handling for `q`, `Q`, Left, and Right.                                            | Restored except shifted `Q` shortcut gap. | Current source restores `q`, Left, and Right through public `KeyboardAction`. Shift-distinct `Q` needs a richer shortcut API if required.                 |
| Demonstrate key-release handling for `q`, `Q`, Left, and Right.                                        | Public API gap.                           | Requires a renderer-neutral key-release callback surface.                                                                                                 |
| Panel title `Tinkertoy Control`.                                                                       | Restored.                                 | Current source uses the historical panel title through public `dart::gui::Panel`.                                                                         |
| Historical backend-specific text `An empty OSG example with ImGui`.                                    | Superseded by promoted renderer wording.  | Restore the educational meaning without using removed backend names in source.                                                                            |
| Menu affordances for Exit and About DART.                                                              | Restored with promoted panel controls.    | Public panel button/text affordances replace menu-bar placement.                                                                                          |
| Simulation section with Play/Pause and Time display.                                                   | Restored.                                 | Current source uses `PanelContext::lifecycle` and `simulationTime`.                                                                                       |
| World Options section with Gravity On/Off.                                                             | Restored.                                 | Current source toggles `World::setGravity` between `-9.81 * UnitZ` and zero.                                                                              |
| World Options section with Headlights On/Off.                                                          | Public API gap.                           | Requires a renderer-neutral lighting/headlight control API.                                                                                               |
| View section with Eye, Center, and Up text.                                                            | Public API gap.                           | Requires public camera-basis data in `PanelContext` or another viewer-inspection API.                                                                     |
| Help section showing shared viewer instructions.                                                       | Restored with promoted flat panel text.   | Current source documents orbit/pan/zoom, target drag, nudge controls, and keydown callbacks.                                                              |
| Default launch size 640x480.                                                                           | Restored.                                 | Current source uses `ApplicationOptions::runDefaults`.                                                                                                    |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0)` with up `(-0.24, -0.25, 0.94)`.        | Restored within orbit-camera API limits.  | Current source converts the historical eye/target distance/yaw/pitch to `dart::gui::OrbitCamera`; camera roll/up remains tied to the shared orbit camera. |
| README documents the promoted runner, controls, defaults, headless capture, and remaining public gaps. | Restored.                                 | Current README documents `pixi run ex imgui`, controls, defaults, capture, and API gaps.                                                                  |
| Keep source free of direct backend UI, viewer, event-handler, and OSG types.                           | Restored through marker guard.            | Source avoids the forbidden backend tokens scanned by `UNIT_gui_FilamentSceneExtraction`.                                                                 |

### CSV Logger Itemized Inventory

Historical sources compared:
`520993d7301^:examples/csv_logger/main.cpp` and
`520993d7301^:examples/csv_logger/README.md`.

This is a preserved non-GUI example. The current source and README match the
pre-promotion files exactly, so restoration means keeping the command-line
logging workflow intact while the renderer migration proceeds around it.

| Historical item                                                                                                       | Current outcome                | Notes                                                                                                                                             |
| --------------------------------------------------------------------------------------------------------------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/sdf/double_pendulum.world` by default.                                                            | Preserved.                     | Current source keeps the historical `Options::worldUri` default and `dart::io::readWorld` path.                                                   |
| Write `simulation_log.csv` by default.                                                                                | Preserved.                     | Current source keeps the historical output path default and `--output` override.                                                                  |
| Simulate 1000 steps by default and log the initial state plus every stepped state.                                    | Preserved.                     | Current source keeps `steps = 1000`, writes one row before stepping, then writes `steps` more rows.                                               |
| Accept `--world`, `--output`, `--steps`, `--dt`, `--skeleton`, `--body`, `-h`, and `--help`.                          | Preserved.                     | Current source keeps the historical parser and error handling.                                                                                    |
| Reject invalid step counts, non-positive time steps, unknown skeletons, unknown bodies, and bad output paths.         | Preserved.                     | Current source keeps the historical validation and diagnostic messages.                                                                           |
| Use the first skeleton/body by default, with named skeleton/body selection when requested.                            | Preserved.                     | Current source keeps the historical selection behavior.                                                                                           |
| CSV header is `time,com_x,com_y,com_z,body_x,body_y,body_z,body_qw,body_qx,body_qy,body_qz`.                          | Preserved.                     | Current source keeps the exact header and fixed six-decimal formatting.                                                                           |
| Log world time, skeleton COM, and selected body translation/orientation quaternion.                                   | Preserved.                     | Current source still computes those values directly from DART simulation state.                                                                   |
| README explains the external-tooling CSV goal, DART IO/world-step concepts, output, CLI flags, build, and help usage. | Preserved.                     | Current README is unchanged from the historical file.                                                                                             |
| No GUI renderer migration is required for this example.                                                               | Superseded by non-GUI scope.   | Do not add `dart::gui` or renderer dependencies; the official renderer promotion should leave this CLI example intact.                            |
| Keep source-marker coverage for the preserved CLI/logging contract.                                                   | Restored through marker guard. | `UNIT_gui_FilamentSceneExtraction` now guards the CLI flags, defaults, CSV header, row count expression, README, and absence of GUI dependencies. |

### Headless Simulation Itemized Inventory

Historical sources compared:
`520993d7301^:examples/headless_simulation/main.cpp` and
`520993d7301^:examples/headless_simulation/README.md`.

This is a preserved non-GUI example. The current source and README match the
pre-promotion files exactly, so restoration means keeping the deterministic
batch-simulation workflow intact while the renderer migration proceeds around
it.

| Historical item                                                                                                                 | Current outcome                | Notes                                                                                                                                           |
| ------------------------------------------------------------------------------------------------------------------------------- | ------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/sdf/double_pendulum.world` by default.                                                                      | Preserved.                     | Current source keeps the historical `Options::worldUri` default and `dart::io::readWorld` path.                                                 |
| Simulate 1000 steps by default, rejecting zero or invalid step counts.                                                          | Preserved.                     | Current source keeps `steps = 1000`, `--steps <n>`, and the historical validation path.                                                         |
| Use time step `0.001` unless `--dt <seconds>` is provided.                                                                      | Preserved.                     | Current source keeps the default time step value and only applies it when the flag is present.                                                  |
| Use deterministic random seed `42` unless `--seed <n>` is provided.                                                             | Preserved.                     | Current source keeps `dart::math::Random::setSeed(options.seed)` before loading/stepping the world.                                             |
| Accept `--world`, `--steps`, `--dt`, `--seed`, `-h`, and `--help`.                                                              | Preserved.                     | Current source keeps the historical parser and error handling.                                                                                  |
| Print world skeleton count, time step, step count, simulated time, and elapsed wall time.                                       | Preserved.                     | Current source keeps the same console progress/timing output.                                                                                   |
| Step the loaded world exactly `steps` times.                                                                                    | Preserved.                     | Current source keeps the tight `for` loop around `world->step()`.                                                                               |
| README explains deterministic headless batch simulation, DART IO/world-step concepts, output, CLI flags, build, and help usage. | Preserved.                     | Current README is unchanged from the historical file.                                                                                           |
| No GUI renderer migration is required for this example.                                                                         | Superseded by non-GUI scope.   | Do not add `dart::gui` or renderer dependencies; the official renderer promotion should leave this CLI example intact.                          |
| Keep source-marker coverage for the preserved headless simulation contract.                                                     | Restored through marker guard. | `UNIT_gui_FilamentSceneExtraction` now guards the CLI flags, default seed/time/steps, progress output, README, and absence of GUI dependencies. |

### Speed Test Itemized Inventory

Historical sources compared:
`520993d7301^:examples/speed_test/main.cpp` and
`520993d7301^:examples/speed_test/README.md`.

This is a preserved non-GUI benchmark example. The current source and README
match the pre-promotion files exactly, so restoration means keeping the
historical dynamics/kinematics timing workflow intact while the renderer
migration proceeds around it.

| Historical item                                                                                                           | Current outcome                | Notes                                                                                                                              |
| ------------------------------------------------------------------------------------------------------------------------- | ------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------- |
| Load the historical SKEL benchmark scene list, including pendulums, serial chains, tree structures, and `fullbody1.skel`. | Preserved.                     | Current source keeps the same `getSceneFiles()` list and `dart::io::readWorld` loading path.                                       |
| Default mode runs dynamics speed tests.                                                                                   | Preserved.                     | Current source keeps `Testing Dynamics`, ten trials, `runDynamicsTest`, and `testDynamicsSpeed`.                                   |
| `-k` switches to kinematics speed tests.                                                                                  | Preserved.                     | Current source keeps the hidden historical `-k` parser, ten kinematics trials, and position/velocity/acceleration summaries.       |
| Dynamics benchmark resets positions, velocities, and accelerations before stepping.                                       | Preserved.                     | Current source keeps `world->eachSkeleton`, `resetPositions`, `resetVelocities`, and `resetAccelerations`.                         |
| Dynamics benchmark steps each world 10000 times by default.                                                               | Preserved.                     | Current source keeps the historical `numIterations = 10000` default and loop around `world->step()`.                               |
| Kinematics benchmark randomizes every DOF within clamped limits and samples transforms/velocities/accelerations.          | Preserved.                     | Current source keeps the historical `dart::math::Random::uniform` and body-node accessor calls.                                    |
| Print per-trial `Result`, final section headers, average, and standard deviation.                                         | Preserved.                     | Current source keeps the historical console summary text and `print_results` implementation.                                       |
| README explains the timing goal, DART IO/kinematics/world-step concepts, timing output, and no controls.                  | Preserved.                     | Current README is unchanged from the historical file.                                                                              |
| No GUI renderer migration is required for this example.                                                                   | Superseded by non-GUI scope.   | Do not add `dart::gui` or renderer dependencies; the official renderer promotion should leave this benchmark intact.               |
| No quick smoke execution is required for this checkpoint.                                                                 | Preserved benchmark scope.     | The historical executable intentionally performs long timing loops and has no bounded trial-count option.                          |
| Keep source-marker coverage for the preserved speed-test benchmark contract.                                              | Restored through marker guard. | `UNIT_gui_FilamentSceneExtraction` now guards the scene list, benchmark loops, summaries, README, and absence of GUI dependencies. |

### Unified Loading Itemized Inventory

Historical sources compared:
`520993d7301^:examples/unified_loading/main.cpp` and
`520993d7301^:examples/unified_loading/README.md`.

This is a preserved non-GUI loading example. The current source and README
match the pre-promotion files exactly, so restoration means keeping the shared
`dart::io::ReadOptions` workflow intact while the renderer migration proceeds
around it.

| Historical item                                                                                                                     | Current outcome                             | Notes                                                                                                                                                              |
| ----------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------- | ------------------------------------------ | ---------- | ------------------------------------------------------------------------------------ |
| Load `dart://sample/sdf/double_pendulum.world` by default through `readWorld`.                                                      | Preserved.                                  | Current source keeps the historical world URI default and world load path.                                                                                         |
| Load `dart://sample/urdf/test/primitive_geometry.urdf` by default through `readSkeleton`.                                           | Preserved.                                  | Current source keeps the historical skeleton URI default and skeleton load path.                                                                                   |
| Accept `--world`, `--skeleton`, `--no-world`, and `--no-skeleton`.                                                                  | Preserved.                                  | Current source keeps both load toggles and rejects the no-load combination.                                                                                        |
| Accept `--format <auto                                                                                                              | skel                                        | sdf                                                                                                                                                                | urdf                                                                                               | mjcf>`and store it in shared`ReadOptions`. | Preserved. | Current source keeps `parseModelFormat`, `formatToString`, and `readOptions.format`. |
| Accept `--sdf-root-joint <floating                                                                                                  | fixed>`and store it in shared`ReadOptions`. | Preserved.                                                                                                                                                         | Current source keeps `parseRootJointType`, `rootJointTypeToString`, and `sdfDefaultRootJointType`. |
| Accept repeatable `--package <name=path>` mappings.                                                                                 | Preserved.                                  | Current source keeps `parsePackageMapping`, `packageMappings`, and `ReadOptions::addPackageDirectory`.                                                             |
| Print read options plus loaded world/skeleton summaries.                                                                            | Preserved.                                  | Current source keeps `Read options:`, `World loaded with`, and `Skeleton ... has ... DOFs.` output.                                                                |
| README explains shared `ReadOptions`, `readWorld`, `readSkeleton`, model format/root-joint/package controls, build, and help usage. | Preserved.                                  | Current README is unchanged from the historical file.                                                                                                              |
| No GUI renderer migration is required for this example.                                                                             | Superseded by non-GUI scope.                | Do not add `dart::gui` or renderer dependencies; the official renderer promotion should leave this loader intact.                                                  |
| Keep source-marker coverage for the preserved unified-loading contract.                                                             | Restored through marker guard.              | `UNIT_gui_FilamentSceneExtraction` now guards `ReadOptions`, load toggles, format/root-joint/package controls, summaries, README, and absence of GUI dependencies. |

### Atlas Simbicon Itemized Inventory

Historical source compared: `520993d7301^:examples/atlas_simbicon`.

The current source is only a visual Atlas shell, so this strict-audit slice
must restore the historical Simbicon example rather than treating the existing
build/screenshot path as sufficient.

| Historical item                                                                                                                 | Current outcome                      | Notes                                                                                                                                             |
| ------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Build the example from source-owned controller, state, state-machine, and terminal-condition files.                             | Restored through public `dart::gui`. | These files are restored as maintained example code without OSG types.                                                                            |
| Load `dart://sample/sdf/atlas/ground.urdf` and `dart://sample/sdf/atlas/atlas_v3_no_head.sdf`.                                  | Restored.                            | Current source loads the historical ground and Atlas skeletons.                                                                                   |
| Set Atlas root position 0 to `-0.5 * pi` and world gravity to `(0, -9.81, 0)`.                                                  | Restored.                            | Current source restores the root position and Y-down gravity.                                                                                     |
| Run the Simbicon controller before each simulation step and apply temporary external pelvis pushes.                             | Restored through public `dart::gui`. | `ApplicationOptions::preStep` now runs the controller and external-force countdown.                                                               |
| Keyboard actions: `r` reset, `a`/`s`/`d`/`f` perturb forward/back/left/right.                                                   | Restored.                            | Historical event-handler behavior is migrated to renderer-neutral `KeyboardAction`s.                                                              |
| Controller actions: harness pelvis/feet, manual next-state, and `1`/`2`/`3`/`4` state-machine selection.                        | Restored.                            | Harness and next-state controls are exposed in the panel; state-machine selection is exposed by keyboard actions and panel buttons.               |
| ImGui panel title `Atlas Control`, Help text, Play/Pause, gravity slider, reset button, and stride mode controls.               | Restored through public `dart::gui`. | `PanelBuilder` provides the promoted text/button/checkbox/slider replacement.                                                                     |
| Headlights, OSG shadow toggle, and depth mode toggles.                                                                          | Public API gap.                      | These need renderer-neutral lighting/render-mode concepts before exact parity; do not revive OSG or private Filament hooks in the example source. |
| Window title `Atlas Simbicon`, 1280x960 launch size, and camera home from `(5.14, 3.28, 6.28) * 2` to `(1, 0, 0)`.              | Mostly restored.                     | Run defaults and camera target are restored through public `ApplicationOptions`; exact native window title remains a public window-options gap.   |
| README documents the Simbicon controller workflow and promoted runner.                                                          | Restored.                            | Current README documents the promoted runner and remaining render-settings gaps.                                                                  |
| Keep source-marker coverage for controller files, pre-step update, controls, camera/defaults, README, and absence of OSG types. | Restored through marker guard.       | `UNIT_gui_FilamentSceneExtraction` guards the restored contract.                                                                                  |

### GUI Scene Diagnostics Itemized Inventory

Historical source compared:
`520993d7301^:examples/gui_scene_diagnostics`.

The historical source was already a renderer-independent descriptor diagnostic,
not an interactive viewer. The current source intentionally differs only by
using promoted `dart::gui` headers/names and by naming `apps/dartsim` as the
maintained GUI application.

| Historical item                                                                                           | Current outcome                | Notes                                                                                                                  |
| --------------------------------------------------------------------------------------------------------- | ------------------------------ | ---------------------------------------------------------------------------------------------------------------------- |
| Build only when the GUI component is available.                                                           | Restored through `dart::gui`.  | CMake now checks `dart-gui` instead of the old `dart-gui-experimental` target.                                         |
| CLI flags: `--frames`, `--width`, `--height`, and `--help`, with a default bounded 10-frame simulation.   | Restored.                      | Current source preserves the diagnostic CLI and normalizes `RunOptions`.                                               |
| Create a `gui_scene_diagnostics` world with a dynamic blue box, gray ground, and yellow `SimpleFrame`.    | Restored.                      | The diagnostic world and object names are preserved.                                                                   |
| Print frame count, renderable count, debug-line count, selection-line count, camera eye, and pick hit.    | Restored.                      | Direct run `gui_scene_diagnostics --frames 3 --width 320 --height 240` produced the expected diagnostic fields.        |
| Exercise renderable extraction, debug-line extraction, selection-line generation, orbit camera, and pick. | Restored through `dart::gui`.  | Current source uses promoted `dart::gui` APIs for the same descriptor checks.                                          |
| README documents descriptor diagnostics and the promoted application direction.                           | Restored.                      | README points users to `apps/dartsim` instead of the removed backend-named example.                                    |
| Keep marker coverage for CLI, world setup, descriptor extraction, diagnostic output, README, and no OSG.  | Restored through marker guard. | `UNIT_gui_FilamentSceneExtraction` now guards the preserved non-interactive diagnostic contract and promoted boundary. |

### Rerun Itemized Inventory

Historical source compared: `520993d7301^:examples/rerun`.

The historical and current trees are identical: `rerun` is a placeholder
directory with no source files. It should not be migrated to `dart::gui` or a
renderer dependency until a real Rerun integration example is added.

| Historical item                                                                                       | Current outcome                | Notes                                                                                                            |
| ----------------------------------------------------------------------------------------------------- | ------------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| Directory contains README and CMake scaffolding only, with no `.cpp` or `.hpp` sources.               | Preserved.                     | The current tree still has no sources for this placeholder.                                                      |
| CMake discovers local sources and returns early when none exist.                                      | Preserved.                     | This avoids creating a broken empty executable target in source builds and external builds.                      |
| Placeholder links only future `utils-urdf` requirements after sources exist.                          | Preserved.                     | No GUI renderer dependency is introduced.                                                                        |
| README describes future Rerun integration scaffolding and states that source files are pending.       | Preserved.                     | Current README is unchanged from the historical source.                                                          |
| Keep marker coverage for no-source status, early CMake skip, pending-source README, and no GUI types. | Restored through marker guard. | `UNIT_gui_FilamentSceneExtraction` guards the preserved placeholder contract.                                    |
| Runtime or capture validation.                                                                        | Not applicable until sourced.  | There is intentionally no `rerun` executable in this checkpoint, so validation is CMake/test/build-only focused. |

### Add/Delete Skeletons Itemized Inventory

Historical source compared: `520993d7301^:examples/add_delete_skels`.

The recent checkpoint restored the main keyboard actions and source-owned world,
but strict re-open found remaining user-visible gaps: the README was deleted,
the 640x480 launch default was not restored, and startup cubes were added even
though the historical source spawned cubes only through the `q` action.

| Historical item                                                                                        | Current outcome                      | Notes                                                                                                                |
| ------------------------------------------------------------------------------------------------------ | ------------------------------------ | -------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/ground.skel`, set gravity to `(0, -9.81, 0)`, and prefer Bullet if available. | Restored with promoted fallback.     | Current source loads the ground, sets Y-down gravity, and selects Bullet behind `DART_HAVE_BULLET` when available.   |
| Start with the loaded ground world; spawn cube skeletons only from the add action.                     | Restored.                            | Startup pre-spawned cubes were removed to match the historical interaction.                                          |
| `q`/`Q` spawns a random cube with a `FreeJoint`, box shape, mass `0.1`, random size, position, color.  | Restored through public `dart::gui`. | Current source exposes `q` through renderer-neutral keyboard actions and a panel button.                             |
| `w`/`W` deletes the most recent spawned cube while preserving the ground skeleton.                     | Restored through public `dart::gui`. | Current source tracks spawned cube names and removes the newest one.                                                 |
| Console instructions for `q`, `w`, and space pause.                                                    | Restored through promoted UI.        | Current source shows the same controls in the panel; console text is superseded by the maintained `dart::gui` panel. |
| 640x480 default window and camera home from `(5, 3, 3)` to the origin.                                 | Restored.                            | Camera home and `ApplicationOptions::runDefaults` now preserve the historical defaults.                              |
| README documents the runner and controls.                                                              | Restored.                            | README was restored with promoted `dart::gui` wording and headless smoke guidance.                                   |
| Keep marker coverage for world setup, controls, no startup cubes, camera/defaults, README, no OSG.     | Restored through marker guard.       | Marker guards cover the restored source contract.                                                                    |

### Atlas Puppet Itemized Inventory

Historical source compared: `520993d7301^:examples/atlas_puppet`.

The current source is a real `dart::gui` example and already restores the Atlas
load, visual target handles, continuous IK solving, and root teleoperation. The
strict re-open still found missing README/default launch behavior and deeper
historical interaction gaps that require additional public API or source-owned
state machinery.

| Historical item                                                                                               | Current outcome                                  | Notes                                                                                                                                      |
| ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------ |
| Load Atlas from `dart://sample/sdf/atlas/atlas_v3_no_head.urdf` and create a ground plane.                    | Restored through public `dart::gui`.             | Current source owns the loaded world and passes it through `ApplicationOptions::world`.                                                    |
| Initial squat/arm posture and feet on the ground.                                                             | Restored.                                        | Current source sets the historical leg/arm DOFs and computes ground clearance from visual bounds.                                          |
| Four IK targets for left/right hand and left/right foot, with foot support geometry.                          | Partially restored.                              | Current source creates visible target handles and support geometry; historical target activation/deactivation semantics remain incomplete. |
| Number keys `1`-`4` toggle target constraints and add/remove visible target frames.                           | Needs follow-up.                                 | Current promoted hotkeys select target handles but do not yet toggle constraint activation or target visibility.                           |
| WASD/QE/FZ root teleoperation.                                                                                | Restored through public `dart::gui`.             | Current source maps these to repeatable keyboard actions and resolves IK after root motion.                                                |
| `X`/`C` support toggles, `R` posture optimization, `T` relaxed-pose reset, and `P` DOF printing.              | Needs follow-up.                                 | These historical event-handler keys are not yet represented in the promoted source.                                                        |
| Whole-body `RelaxedPosture` objective and `BalanceConstraint` recovery behavior.                              | Needs follow-up.                                 | Current source uses per-target Jacobian-transpose IK but does not restore the historical whole-body objective/constraint stack.            |
| Default support polygon visual at elevation `0.05`, COM marker, and support-area feedback.                    | Public API gap / needs source-owned replacement. | Public debug-line helpers exist, but `ApplicationOptions` does not yet expose default support-polygon overlay options for examples.        |
| 1280x960 default window and camera home from `(5.34, 3.00, 2.41)` to `(0, 0, 1)`.                             | Restored.                                        | `ApplicationOptions::runDefaults` and `ApplicationOptions::camera` now preserve the historical defaults.                                   |
| README documents the runner, kinematic mode, IK targets, root teleoperation, and remaining promoted gaps.     | Restored.                                        | README was restored with promoted `dart::gui` wording and explicit remaining gap notes.                                                    |
| Keep marker coverage for load/posture/targets/teleop/defaults/README/no OSG plus named remaining parity gaps. | Restored for this slice.                         | Marker guards now cover README/defaults and the named remaining gaps.                                                                      |

## Example Inventory

| Example                     | Current Audit State                                                                | Next Required Action                                                                                                                                                                        |
| --------------------------- | ---------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `add_delete_skels`          | Restored by strict re-open.                                                        | Keep marker guards for controls, no startup cubes, camera/defaults, README, and no backend types.                                                                                           |
| `atlas_puppet`              | Restored except target-activation/posture/balance/support-visualization gaps.      | Keep remaining gaps explicit until repaired through source-owned state or public debug/IK APIs.                                                                                             |
| `atlas_simbicon`            | Restored except render-settings/window-title API gaps.                             | Keep marker guards for controller/state files, pre-step control, perturbation and stride controls, camera/defaults, README, and no OSG types.                                               |
| `biped_stand`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, camera, run defaults, README, and guards.                                                                                                                    |
| `box_stacking`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver controls, camera/defaults, README, and guards.                                                                                                                               |
| `boxes`                     | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm Bullet preference, camera/defaults, README, and guards.                                                                                                                             |
| `capsule_ground_contact`    | Restored by strict audit.                                                          | Keep marker guards for controls, camera/defaults, README, and no backend types.                                                                                                             |
| `coupler_constraint`        | Restored by strict audit.                                                          | Keep marker guards for controller, reset key, diagnostics, grid, camera/defaults, README, and no backend types.                                                                             |
| `csv_logger`                | Preserved by strict audit as a non-GUI example.                                    | Keep marker guards for CLI flags, CSV header/row count, README, and no GUI renderer dependency.                                                                                             |
| `drag_and_drop`             | Restored except public rotation-manipulator API gap.                               | Keep marker guards for frame handle, child box, markers, camera/defaults, README, and no backend types.                                                                                     |
| `empty`                     | Restored except public key-release/render-hook API gaps.                           | Keep key-release and pre/post-render hooks tracked as public API follow-ups.                                                                                                                |
| `fetch`                     | Re-opened by maintainer correction; full restoration not yet trusted.              | Re-compare against `520993d7301^:examples/fetch/main.cpp`, challenge target manipulation, panel/help affordances, camera/defaults, README, and marker guards before marking restored again. |
| `free_joint_cases`          | Restored by strict audit.                                                          | Keep marker guards for numeric/reference controls, local CLI flags, camera/defaults, README, and no backend types.                                                                          |
| `g1_puppet`                 | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, teleoperation, camera, and guards.                                                                                                                           |
| `gui_scene_diagnostics`     | Preserved by strict audit as a descriptor diagnostic example.                      | Keep marker guards for CLI flags, descriptor extraction, debug/selection lines, pick output, README, and no backend types.                                                                  |
| `hardcoded_design`          | Recent parity checkpoint; OSG wireframe remains public API gap.                    | Confirm checklist entry and keep wireframe follow-up explicit.                                                                                                                              |
| `headless_simulation`       | Preserved by strict audit as a non-GUI example.                                    | Keep marker guards for CLI flags, seed setup, progress output, README, and no GUI renderer dependency.                                                                                      |
| `heightmap`                 | Restored except public debug-grid/color editor gaps.                               | Keep OSG grid style controls tracked as public API follow-up.                                                                                                                               |
| `hello_world`               | Restored by strict audit.                                                          | Keep marker guards for instructions, camera/defaults, profiling, README, and no backend types.                                                                                              |
| `hubo_puppet`               | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK, teleoperation, target activation semantics, and guards.                                                                                                                         |
| `human_joint_limits`        | Restored except TinyDNN/custom-constraint dependency follow-up.                    | Keep marker guards for live world, joint-limit enforcement, defaults, README, and no backend types.                                                                                         |
| `hybrid_dynamics`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm harness toggle, camera, and guards.                                                                                                                                                 |
| `imgui`                     | Restored except public headlight/camera/key-release/render-hook API gaps.          | Keep marker guards for target frame, keydown callbacks, gravity control, help text, camera/defaults, README, and no backend types.                                                          |
| `joint_constraints`         | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, harness toggle, camera, and guards.                                                                                                                          |
| `lcp_physics`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver/collision controls, camera/defaults, README, and guards.                                                                                                                     |
| `mimic_pendulums`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm diagnostics, solver/collision flags, README, and guards.                                                                                                                            |
| `mixed_chain`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm impulse controls, camera, and guards.                                                                                                                                               |
| `operational_space_control` | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK/control behavior, teleoperation, camera, and guards.                                                                                                                             |
| `point_cloud`               | Restored except public color-editor/debug-grid API gaps.                           | Keep color editors and fine-grained grid controls tracked as public API follow-ups.                                                                                                         |
| `polyhedron_visual`         | Restored by strict audit.                                                          | Keep marker guards for surface, wireframe, grid, camera/defaults, and README.                                                                                                               |
| `rerun`                     | Preserved by strict audit as a no-source placeholder.                              | Keep marker guards for CMake early skip, pending-source README, no sources, and no GUI renderer dependency.                                                                                 |
| `rigid_chain`               | Restored by strict audit.                                                          | Keep marker guards for random pose, damping, camera/defaults, README, and no backend types.                                                                                                 |
| `rigid_cubes`               | Restored by strict audit.                                                          | Keep marker guards for `cubes.skel`, force keys/decay, camera/defaults, instructions, README, and no backend types.                                                                         |
| `rigid_loop`                | Restored by strict audit.                                                          | Keep marker guards for constrained-link colors, instructions, defaults, README, and no backend types.                                                                                       |
| `rigid_shapes`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm spawn/delete/contact controls, CLI options, camera/defaults, README, and guards.                                                                                                    |
| `simple_frames`             | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm camera/defaults, README, and guards.                                                                                                                                                |
| `simulation_event_handler`  | Restored except shifted-slash help alias follow-up.                                | Keep source-marker guards for controls, arrows, camera/defaults, README, and no backend types.                                                                                              |
| `soft_bodies`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm playback controls, shifted keys, camera/defaults, and guards.                                                                                                                       |
| `speed_test`                | Preserved by strict audit as a non-GUI benchmark example.                          | Keep marker guards for scene list, dynamics/kinematics loops, summaries, README, and no GUI renderer dependency.                                                                            |
| `tinkertoy`                 | Recent builder/keyboard/camera checkpoints; still subject to strict audit re-open. | Confirm all construction controls, force controls, camera home, and recording gap.                                                                                                          |
| `unified_loading`           | Preserved by strict audit as a non-GUI loading example.                            | Keep marker guards for `ReadOptions`, load toggles, format/root-joint/package controls, README, and no GUI renderer dependency.                                                             |
| `vehicle`                   | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm command keys, camera, and guards.                                                                                                                                                   |
| `wam_ikfast`                | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK target behavior, teleoperation, camera, and guards.                                                                                                                              |
