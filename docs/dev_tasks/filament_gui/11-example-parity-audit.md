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

| Historical item                                                                                                              | Current outcome                                     | Notes                                                                                                                                                          |
| ---------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml`.                                                          | Restored through public `dart::gui`.                | Current source calls `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                                                           |
| Prefer Bullet collision for the Fetch world.                                                                                 | Restored through public `dart::gui` example source. | Current source selects `CollisionDetectorType::Bullet` when Bullet is available.                                                                               |
| Lock the Fetch root joint, set spring stiffness on all DOFs, stiffen torso lift, and restore the initial base/arm positions. | Restored.                                           | Current source keeps the historical numeric positions and stiffness values.                                                                                    |
| Restore the object initial position.                                                                                         | Restored.                                           | Current source keeps the historical object coordinates.                                                                                                        |
| Find `robot0:mocap`, reset the first weld constraint, and sync the mocap root to the interactive target before each step.    | Restored.                                           | Current source uses `ApplicationOptions::preStep` and `syncMocapTarget`.                                                                                       |
| Create an interactive target at `(1.3, 0.75, 0.50)` with a 90-degree Y rotation.                                             | Restored with promoted selection geometry.          | Current source uses a draggable `SimpleFrame` cross instead of OSG `InteractiveFrame`.                                                                         |
| Show the target as the visible end-effector control affordance.                                                              | Restored.                                           | Current source uses `LineSegmentShape` cross geometry.                                                                                                         |
| Add a work-area grid offset at `(1.3, 0.75, 0)`.                                                                             | Restored.                                           | Current source uses DART-owned line-segment geometry instead of OSG `GridVisual`.                                                                              |
| Provide an ImGui-scale option.                                                                                               | Restored through the promoted runner.               | `--gui-scale` is parsed by the shared `dart::gui` application runner.                                                                                          |
| Default window size 1280x960 while preserving CLI overrides.                                                                 | Restored.                                           | Current source uses `ApplicationOptions::runDefaults`.                                                                                                         |
| Camera home from eye `(4, 4, 2.5)` to target `(0.1, -0.3, 0.3)`.                                                             | Restored.                                           | Current source uses `ApplicationOptions::camera`.                                                                                                              |
| Show the historical panel title and instructional text.                                                                      | Open in this audit slice.                           | Current panel title/text are close but not itemized against the historical strings. Restore the title/text first.                                              |
| Expose Play/Pause simulation controls.                                                                                       | Restored.                                           | Current source uses `ViewerLifecycleState::paused` through `PanelContext`.                                                                                     |
| Expose Exit and About DART affordances.                                                                                      | Restored with promoted panel buttons.               | Historical menu-bar placement is superseded by the current renderer-neutral panel API.                                                                         |
| Show general viewer instructions/help.                                                                                       | Partially restored.                                 | Current source includes selection/drag text. The historical collapsible Help section maps to a future DART-owned panel-section API if needed by more examples. |
| Enable drag-and-drop of the target.                                                                                          | Restored through promoted selection/drag controls.  | Current runner supports Ctrl-left drag plus keyboard nudge controls on selectable frames.                                                                      |
| README documents how to run the example.                                                                                     | Restored for promoted runner.                       | Current README documents `pixi run ex fetch` and headless capture.                                                                                             |

## Example Inventory

| Example                     | Current Audit State                                                                | Next Required Action                                                                         |
| --------------------------- | ---------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| `add_delete_skels`          | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm historical controls/defaults are itemized and guarded.                               |
| `atlas_puppet`              | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, posture, and camera items are itemized.                       |
| `atlas_simbicon`            | Needs strict audit.                                                                | Compare historical source and list controls/defaults/panels.                                 |
| `biped_stand`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, camera, run defaults, README, and guards.                     |
| `box_stacking`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver controls, camera/defaults, README, and guards.                                |
| `boxes`                     | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm Bullet preference, camera/defaults, README, and guards.                              |
| `capsule_ground_contact`    | Needs strict audit.                                                                | Compare historical source and list visual/default behavior.                                  |
| `coupler_constraint`        | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                        |
| `csv_logger`                | Needs strict audit.                                                                | Confirm non-GUI logging behavior and README remain intact.                                   |
| `drag_and_drop`             | Needs strict audit.                                                                | Compare historical selection/drag affordances and README.                                    |
| `empty`                     | Needs strict audit.                                                                | Confirm historical minimal-run behavior and defaults.                                        |
| `fetch`                     | Re-opened by maintainer correction.                                                | Fill itemized inventory before declaring complete; repair or record any gaps.                |
| `free_joint_cases`          | Needs strict audit.                                                                | Compare historical source and list controls/defaults.                                        |
| `g1_puppet`                 | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm target activation, IK, teleoperation, camera, and guards.                            |
| `gui_scene_diagnostics`     | Needs strict audit.                                                                | Confirm diagnostic GUI behavior and capture expectations.                                    |
| `hardcoded_design`          | Recent parity checkpoint; OSG wireframe remains public API gap.                    | Confirm checklist entry and keep wireframe follow-up explicit.                               |
| `headless_simulation`       | Needs strict audit.                                                                | Confirm non-GUI simulation/capture behavior remains intact.                                  |
| `heightmap`                 | Known incomplete.                                                                  | Restore `--demo`, interactive controls, alignment mode, camera/defaults, README, and guards. |
| `hello_world`               | Needs strict audit.                                                                | Confirm historical camera/default behavior and README.                                       |
| `hubo_puppet`               | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK, teleoperation, target activation semantics, and guards.                          |
| `human_joint_limits`        | Needs strict audit.                                                                | Compare historical controls/defaults and visual diagnostics.                                 |
| `hybrid_dynamics`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm harness toggle, camera, and guards.                                                  |
| `imgui`                     | Needs strict audit.                                                                | Confirm promoted panel behavior preserves the user-facing ImGui example scope.               |
| `joint_constraints`         | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm perturbation controls, harness toggle, camera, and guards.                           |
| `lcp_physics`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm solver/collision controls, camera/defaults, README, and guards.                      |
| `mimic_pendulums`           | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm diagnostics, solver/collision flags, README, and guards.                             |
| `mixed_chain`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm impulse controls, camera, and guards.                                                |
| `operational_space_control` | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK/control behavior, teleoperation, camera, and guards.                              |
| `point_cloud`               | Needs strict audit.                                                                | Compare historical point cloud controls/defaults and README.                                 |
| `polyhedron_visual`         | Needs strict audit.                                                                | Compare historical visual behavior and defaults.                                             |
| `rerun`                     | Needs strict audit.                                                                | Confirm non-Filament logging/viewer behavior remains correct.                                |
| `rigid_chain`               | Needs strict audit.                                                                | Compare historical controls/defaults and camera.                                             |
| `rigid_cubes`               | Needs strict audit.                                                                | Confirm restored launcher captures historical cube scene behavior.                           |
| `rigid_loop`                | Needs strict audit.                                                                | Compare historical controls/defaults and camera.                                             |
| `rigid_shapes`              | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm spawn/delete/contact controls, CLI options, camera/defaults, README, and guards.     |
| `simple_frames`             | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm camera/defaults, README, and guards.                                                 |
| `simulation_event_handler`  | Needs strict audit.                                                                | Compare historical event-handler controls/defaults.                                          |
| `soft_bodies`               | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm playback controls, shifted keys, camera/defaults, and guards.                        |
| `speed_test`                | Needs strict audit.                                                                | Confirm performance-test behavior and README remain intact.                                  |
| `tinkertoy`                 | Recent builder/keyboard/camera checkpoints; still subject to strict audit re-open. | Confirm all construction controls, force controls, camera home, and recording gap.           |
| `unified_loading`           | Needs strict audit.                                                                | Confirm model-loading workflows, CLI behavior, and README.                                   |
| `vehicle`                   | Recent parity checkpoint; still subject to strict audit re-open.                   | Confirm command keys, camera, and guards.                                                    |
| `wam_ikfast`                | Recent robot/IK checkpoint; still subject to strict audit re-open.                 | Confirm IK target behavior, teleoperation, camera, and guards.                               |
