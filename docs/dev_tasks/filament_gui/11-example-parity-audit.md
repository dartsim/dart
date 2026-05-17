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
- `examples/rerun` was previously preserved as an unchanged no-source
  placeholder, then removed after maintainer re-open found no concrete Rerun
  integration workflow or downstream dependency. The follow-up support audit
  found no remaining product CMake/package/API/runtime support and no path named
  Rerun; retained references are CI retry docs, generated command guards,
  removal marker tests, and historical dev-task notes.

## Active Correction

`examples/fetch/` was kept open as the concrete reminder case. Earlier
checkpoints restored MJCF loading, Bullet preference, robot/object initial
positions, mocap weld reset, target sync, visible target affordance, work-area
grid, camera home, 1280x960 run defaults, GUI scaling, selection/drag/nudge
instructions, Play/Pause/Step/Exit panel controls, README, and source-marker
tests. The post-WAM strict re-open rechecked the current source and README
against `520993d7301^:examples/fetch/main.cpp`; the source behavior remains
restored or superseded by public `dart::gui`, and the concrete follow-up gap was
README/marker coverage for the promoted `--out` image-sequence capture path.

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

2026-05-16 follow-up during the Simple Frames checkpoint: maintainer steering
again warned that many more examples are not fully restored and named
`examples/fetch/` specifically. Do not let the linear cursor through recently
touched examples imply that Fetch is closed. After the in-flight Simple Frames
checkpoint is committed and pushed, the next active audit must re-open Fetch
with a fresh historical-source comparison, then continue through every row that
is still marked as a recent checkpoint, partial restoration, or public API gap.

2026-05-16 follow-up during the Hubo Puppet checkpoint: maintainer steering
again called out that many examples remain incompletely restored, with
`examples/fetch/` named as the representative case. After Hubo is pushed, make
Fetch the next active slice even though it has several previous restoration
commits. The itemized inventory below is not a terminal closure; re-check it
against the historical OSG/ImGui source and update this file before any Fetch
code change.

Fresh Fetch re-open after the Hubo checkpoint: the historical panel says the
dummy object position is indicated "at the cross of the two transparent green
bars." The source had drifted to a custom line/ring/arrow target gizmo. The
current source restores the selectable target as a source-owned mesh cross made
from two transparent green bars, while keeping promoted selection, rotation,
and keyboard reset controls as renderer-neutral replacements for the old
`InteractiveFrame` manipulation.

Fresh Fetch re-open after the WAM IKFast checkpoint: the current source still
matches the historical behavior through public `dart::gui`, including target
sync, target manipulation, grid, panel, camera/defaults, and menu/help
affordances. The README and source-marker test are updated in this slice to
cover `--out` image-sequence capture alongside `--screenshot`, keeping Fetch
aligned with the promoted capture contract.

### Fetch Itemized Inventory

Historical source compared: `520993d7301^:examples/fetch/main.cpp`.

| Historical item                                                                                                              | Current outcome                                     | Notes                                                                                                                                                                                                                             |
| ---------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/mjcf/openai/robotics/fetch/pick_and_place.xml`.                                                          | Restored through public `dart::gui`.                | Current source calls `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                                                                                                                              |
| Prefer Bullet collision for the Fetch world.                                                                                 | Restored through public `dart::gui` example source. | Current source selects `CollisionDetectorType::Bullet` when Bullet is available.                                                                                                                                                  |
| Lock the Fetch root joint, set spring stiffness on all DOFs, stiffen torso lift, and restore the initial base/arm positions. | Restored.                                           | Current source keeps the historical numeric positions and stiffness values.                                                                                                                                                       |
| Restore the object initial position.                                                                                         | Restored.                                           | Current source keeps the historical object coordinates.                                                                                                                                                                           |
| Find `robot0:mocap`, reset the first weld constraint, and sync the mocap root to the interactive target before each step.    | Restored.                                           | Current source uses `ApplicationOptions::preStep` and `syncMocapTarget`.                                                                                                                                                          |
| Create an interactive target at `(1.3, 0.75, 0.50)` with a 90-degree Y rotation.                                             | Restored through public `dart::gui`.                | Current source uses one target `SimpleFrame`, a public `dart::gui::Gizmo`, keyboard rotation/reset actions, and mocap sync.                                                                                                       |
| Show the target as the visible end-effector control affordance: the cross of two transparent green bars.                     | Restored.                                           | Current source keeps the source-owned `MeshShape` cross as the Fetch-specific target cue while the public gizmo owns mouse manipulation.                                                                                          |
| Add a work-area grid offset at `(1.3, 0.75, 0)`.                                                                             | Restored.                                           | Current source uses DART-owned line-segment geometry instead of OSG `GridVisual`.                                                                                                                                                 |
| Provide an ImGui-scale option.                                                                                               | Restored through the promoted runner.               | `--gui-scale` is parsed by the shared `dart::gui` application runner.                                                                                                                                                             |
| Default window size 1280x960 while preserving CLI overrides.                                                                 | Restored.                                           | Current source uses `ApplicationOptions::runDefaults`.                                                                                                                                                                            |
| Camera home from eye `(4, 4, 2.5)` to target `(0.1, -0.3, 0.3)`.                                                             | Restored.                                           | Current source uses `ApplicationOptions::camera`.                                                                                                                                                                                 |
| Show the historical panel title and instructional text.                                                                      | Restored.                                           | Current source uses the historical title, example description, whole-body motion explanation, and user-guide label.                                                                                                               |
| Set the panel to position `(10, 20)`, size `(360, 600)`, background alpha `0.5`, menu bar, and horizontal scrollbar.         | Restored through public `dart::gui`.                | Current source uses promoted panel window options instead of direct ImGui or private Filament hooks.                                                                                                                              |
| Use collapsible `Help` and default-open `Simulation` sections.                                                               | Restored through public `dart::gui`.                | Current source uses the promoted `PanelBuilder::collapsingHeader` helper.                                                                                                                                                         |
| Expose `Menu -> Exit` and `Help -> About DART` from the panel menu bar.                                                      | Restored through public `dart::gui`.                | Current source uses promoted panel menu helpers; Exit requests lifecycle shutdown and About DART reveals promoted project/library text.                                                                                           |
| Expose Play/Pause simulation controls.                                                                                       | Restored.                                           | Current source uses `ViewerLifecycleState::paused` through `PanelContext`.                                                                                                                                                        |
| Expose Exit and About DART affordances.                                                                                      | Restored with promoted panel/menu controls.         | Current source exposes Exit both from the menu bar and the Simulation section, and exposes About DART from the Help menu.                                                                                                         |
| Show general viewer instructions/help.                                                                                       | Restored with promoted flat panel text.             | Current source includes target gizmo, keyboard nudge, target rotation/reset, orbit/pan/zoom, pause/step, screenshot/capture, and exit guidance. Historical collapsible placement is superseded by the renderer-neutral panel API. |
| Enable drag-and-drop of the target.                                                                                          | Restored through public `dart::gui`.                | Current source registers a public target gizmo with axis arrows, plane handles, and rotation rings while retaining keyboard nudging and local-axis rotation keys.                                                                 |
| README documents how to run the example.                                                                                     | Restored.                                           | Current README documents `pixi run ex fetch`, `--screenshot`, `--out`, and the historical standalone build/execute instruction sections.                                                                                          |
| Keep Fetch open if another historical behavior gap is identified.                                                            | Active strict-audit rule.                           | Do not treat the first-pass inventory as final without re-checking the current source against the old OSG/ImGui source.                                                                                                           |

### Biped Stand Itemized Inventory

Historical source compared: `520993d7301^:examples/biped_stand`.

The current source is a real `dart::gui` example and restores the live world and
controller behavior, but the strict re-open found remaining user-facing
documentation/instruction gaps.

| Historical item                                                                                  | Current outcome                      | Notes                                                                                                                                     |
| ------------------------------------------------------------------------------------------------ | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/fullbody1.skel` and set gravity to `(0, -9.81, 0)`.                     | Restored through public `dart::gui`. | Current source calls `dart::io::readWorld`, preserves the Y-down gravity, and passes the world through `ApplicationOptions::world`.       |
| Set the historical pelvis, thigh, shin, heel, and abdomen initial DOF positions.                 | Restored.                            | Current source preserves the same numeric DOF values before controller setup.                                                             |
| SPD tracking controller with free-root gains disabled, `Kp=400`, `Kd=40`, and ankle strategy.    | Restored.                            | Current `BipedStandController::preStep` preserves the historical mass-matrix, Coriolis/gravity, constraint force, and ankle torque logic. |
| Keys `1`-`4` apply +/-X and +/-Z pushes with force `50` for `100` frames.                        | Restored.                            | Current source exposes renderer-neutral keyboard actions and matching panel buttons.                                                      |
| User-visible text prints "Press space to start simulation" and the exact four push instructions. | Restored.                            | Current panel preserves the historical start text and all four push-instruction lines.                                                    |
| Default 640x480 window size and camera home from `(3, 1.5, 3)` to `(0, 0, 0)`.                   | Restored.                            | Current source uses `ApplicationOptions::runDefaults` and `ApplicationOptions::camera`.                                                   |
| README includes standalone build and execute instructions.                                       | Restored.                            | Current README documents `pixi run ex biped_stand`, headless capture, and standalone build/execute sections.                              |

### Box Stacking Itemized Inventory

Historical source compared:
`520993d7301^:examples/box_stacking/main.cpp` and
`520993d7301^:examples/box_stacking/README.md`.

The strict re-open found that the earlier solver-controls checkpoint restored
the high-level `dart::gui` handoff, but still missed concrete historical scene,
panel, and README details. The current slice repairs the expressible behavior
through public `dart::gui` and records remaining OSG-viewer-specific surfaces
as explicit public API follow-ups. The local R24-15 camera-inspection slice then
promotes panel camera state and restores the Eye/Center/Up readout.

| Historical item                                                                                        | Current outcome                                | Notes                                                                                                                                                       |
| ------------------------------------------------------------------------------------------------------ | ---------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Build five dynamic boxes with size `(1, 1, 0.5)`, random colors, and stack positions from `z=0.75`.    | Restored through public `dart::gui`.           | Current source restores historical random colors and the non-intersecting `0.5 + 0.25 + i * 0.5` placement.                                                 |
| Create a static 10x10 floor box with height `0.01` and light gray visual color.                        | Restored.                                      | Current source uses the historical floor height and `dart::Color::LightGray()`.                                                                             |
| Use gravity `(0, 0, -9.81)` with a panel checkbox to disable/enable it.                                | Restored.                                      | Current source owns gravity and restores the historical `Gravity On/Off` label.                                                                             |
| Expose Dantzig/PGS LCP solver selection and split-impulse toggling without losing split-impulse state. | Restored through public `dart::gui`.           | Current source uses promoted panel controls plus a launch-time `--solver` option.                                                                           |
| Show Play/Pause simulation controls and current simulation time.                                       | Restored through public `dart::gui`.           | Current source uses the promoted lifecycle pause/resume and step controls and shows simulation time/contact count.                                          |
| Show title `Box Stacking`, text `Box stacking demo`, `LCP solver:`, and `User Guide` help text.        | Restored through public `dart::gui`.           | Current panel restores the historical title, labels, and help heading using the promoted flat panel API.                                                    |
| Provide Exit and About DART affordances in the ImGui menu bar.                                         | Superseded by promoted application/panel flow. | The promoted renderer-neutral panel API has no menu-bar abstraction; examples should use shared application exit/about behavior once exposed publicly.      |
| Toggle viewer headlights from the panel.                                                               | Restored through public `dart::gui`.           | `PanelContext::lighting.headlightsEnabled` exposes renderer-neutral scene-light state without backend hooks.                                                |
| Show camera Eye/Center/Up readout in the panel.                                                        | Restored through public `dart::gui`.           | `PanelContext::camera` exposes live eye, target, and up vectors for renderer-neutral panel readouts.                                                        |
| Print custom keydown/keyup messages for `q`, `Q`, Left, and Right.                                     | Restored through public `dart::gui`.           | `KeyboardActionTrigger::Release` restores key-release callbacks; shifted `Q` is distinct from lowercase `q` in the promoted shortcut mapping.               |
| Preserve `--gui-scale`.                                                                                | Restored through shared runner.                | The promoted `dart::gui` runner parses shared GUI scale options.                                                                                            |
| Default launch size 800x640 and camera home from eye `(12, 12, 9)` to target `(0, 0, 2)`.              | Restored.                                      | Current source uses `ApplicationOptions::runDefaults` and `ApplicationOptions::camera`.                                                                     |
| README documents in-tree run, solver option, headless capture, and historical build/execute sections.  | Restored.                                      | Current README documents promoted runner usage, `--gui-scale`, headless capture, default size, and standalone build/execute sections.                       |
| Keep marker coverage for stack placement, floor geometry, controls, defaults, README, and no OSG path. | Restored through marker guard.                 | The source-marker test now guards the restored stack placement, floor geometry, panel labels, README sections, and absence of legacy renderer entry points. |

### Boxes Itemized Inventory

Historical source compared: `520993d7301^:examples/boxes/main.cpp` and
`520993d7301^:examples/boxes/README.md`.

The current source is already a real `dart::gui` example and preserves the main
box-grid behavior. The strict re-open found remaining user-facing naming,
instruction, README, and marker gaps that should be repaired before marking the
example restored.

| Historical item                                                                                      | Current outcome                          | Notes                                                                                                                                                     |
| ---------------------------------------------------------------------------------------------------- | ---------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Prefer Bullet collision for the world.                                                               | Restored with optional dependency guard. | Current source selects `CollisionDetectorType::Bullet` when `DART_HAVE_BULLET` is enabled, preserving the historical preference without requiring Bullet. |
| Build a 5x5x5 grid of boxes at `(i - dim / 2, j - dim / 2, k + 5)` with size `(0.9, 0.9, 0.9)`.      | Restored.                                | Current source preserves the grid dimensions, positions, sizes, and color ramp.                                                                           |
| Name boxes as `box0`, `box1`, and so on, so selected labels match the historical source.             | Restored.                                | Current source uses the historical `box` prefix without an underscore.                                                                                    |
| Set restitution coefficient `0.9` on boxes and ground.                                               | Restored.                                | Current source preserves restitution through `DynamicsAspect`.                                                                                            |
| Create a 25x25x0.1 light-gray ground box.                                                            | Restored.                                | Current source preserves the dimensions and uses `dart::Color::LightGray()`.                                                                              |
| Print/show the experimental warning and `Press space to start free falling the box.` instructions.   | Restored through public `dart::gui`.     | Current source prints the historical text and shows it in a promoted panel.                                                                               |
| Enable OSG shadow-map configuration.                                                                 | Superseded by promoted renderer.         | Filament owns maintained lighting/shadow behavior; do not restore OSG shadow-map controls.                                                                |
| Default launch size 1360x768 and camera home from `(20, 20, 15)` to target `(0, 0, 3)`.              | Restored.                                | Current source uses `ApplicationOptions::runDefaults` and `ApplicationOptions::camera`.                                                                   |
| README documents in-tree run, controls, headless capture, and historical build/execute sections.     | Restored.                                | Current README documents promoted runner usage, controls, headless capture, default size, and standalone build/execute sections.                          |
| Keep marker coverage for Bullet preference, grid, names, ground color, instructions, README, no OSG. | Restored through marker guard.           | The source-marker test now guards the restored labels, ground color, instruction text, README sections, and absence of legacy renderer entry points.      |

### Simple Frames Itemized Inventory

Historical source compared:
`520993d7301^:examples/simple_frames/main.cpp` and
`520993d7301^:examples/simple_frames/README.md`.

The current source is already a real `dart::gui` example and restores the frame
transforms, default window, and camera home. The strict re-open found remaining
historical naming, arrow-shape, README, and marker gaps.

| Historical item                                                                                       | Current outcome                               | Notes                                                                                                                                                     |
| ----------------------------------------------------------------------------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Create frame hierarchy `F1 -> F2 -> F3` with transforms `(0.1,-0.1,0)`, 45-degree X, and 60-degree Y. | Restored through public `dart::gui`.          | Current source preserves transforms and restores the historical frame names.                                                                              |
| Add marker hierarchy `A -> A1/A2/A3` at the corresponding frame transforms.                           | Restored through public `dart::gui`.          | Current source preserves marker transforms and restores the historical marker names.                                                                      |
| Render the arrow using DART `ArrowShape` from `(0.1,-0.1,0)` to `(0.1,0,0)` with pink color.          | Restored through public `dart::gui`.          | Current source uses the historical DART `ArrowShape`; generated in-memory mesh shapes no longer attempt empty file-URI material loads.                    |
| Add root frames to the world and render their descendants.                                            | Restored visually with explicit registration. | Public extraction currently visits world-registered simple frames, so current source registers descendants explicitly while preserving parent transforms. |
| Default launch size 640x480 and camera home from `(2, 1, 2)` to the origin.                           | Restored.                                     | Current source uses `ApplicationOptions::runDefaults` and `ApplicationOptions::camera`.                                                                   |
| README documents in-tree run, controls, headless capture, and historical build/execute sections.      | Restored.                                     | Current README documents promoted runner usage, controls, headless capture, default size, standalone build/execute sections, and feature list.            |
| Keep marker coverage for names, geometry, arrow shape, defaults, README, and no backend renderer.     | Restored through marker guard.                | The source-marker test now guards historical names, `ArrowShape`, README sections, and absence of legacy renderer entry points.                           |

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
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0.50)` with up `(-0.24,-0.25,0.94)`. | Restored.                            | Current source restores the historical home and up vector through `dart::gui::OrbitCamera`.            |
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
| Camera home from eye `(2.5, 2.5, 1.5)` to target `(0, 0, 0.2)` with up `(-0.2,-0.2,0.95)`.                      | Restored.                                 | Current source restores the historical home and up vector through `dart::gui::OrbitCamera`.           |
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
| Fine-grained grid plane, offset, line count, line step, minor/major line, width, and color controls.   | Restored.                       | Current source uses shared source-owned DART line-grid controls without OSG or backend hooks. |
| Terrain color editor.                                                                                  | Restored through public API.    | Current source uses `PanelBuilder::colorEdit` to update the terrain visual color.             |
| Alignment mode creates collision heightmap, reference box, and two ball grids.                         | Restored.                       | Current source owns the alignment world construction.                                         |
| Alignment mode prefers ODE collision when available, uses gravity `(0, 0, -9.81)`, and timestep 0.001. | Restored.                       | Current source restores the ODE preference under `#if DART_HAVE_ODE`.                         |
| README documents the promoted runner and both demo modes.                                              | Restored.                       | Current README documents default, alignment, and headless capture commands.                   |

### Point Cloud Itemized Inventory

Historical source compared: `520993d7301^:examples/point_cloud/main.cpp`.

The static fixture checkpoint has been replaced by a live source-owned
`dart::gui` example. The local R24-20 panel-color checkpoint restores the
color-edit widgets through public `PanelBuilder::colorEdit`; the later
source-owned grid checkpoint restores fine-grained grid controls with DART line
geometry. Backend-specific ImGui, OSG, and GridVisual concepts must stay out of
the example source.

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
| Expose point-cloud and voxel-grid color editors.                                                                   | Restored through public API.                        | Current source uses `PanelBuilder::colorEdit` to update the point-cloud and voxel-grid visual colors.                                |
| Expose voxel-grid visibility.                                                                                      | Restored.                                           | Current source restores it under `#if DART_HAVE_OCTOMAP` and shows an unavailable note when OctoMap is disabled.                     |
| Show a scene grid.                                                                                                 | Restored.                                           | Current source uses DART-owned line geometry named `point_cloud_grid` instead of OSG `GridVisual`.                                   |
| Expose fine-grained grid display, plane, offset, line count, line step, minor/major line, width, and color fields. | Restored.                                           | Current source uses shared source-owned DART line-grid controls without OSG `GridVisual` or backend-specific hooks.                  |
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
renderer-neutral keydown/key-release callbacks, run defaults, camera home, and
README. The local R24-18 callback checkpoint restores pre/post render and
post-step hooks through public `ApplicationOptions` callbacks;
backend-specific event-handler types must stay out of the example source.

| Historical item                                                                                   | Current outcome                      | Notes                                                                                                                                   |
| ------------------------------------------------------------------------------------------------- | ------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------- |
| Create a world and add a target `InteractiveFrame` to it.                                         | Restored with promoted frame.        | Current source uses public `SimpleFrame`/selection affordances instead of OSG `InteractiveFrame`.                                       |
| Enable drag-and-drop for the target.                                                              | Restored by promoted runner.         | Current source and README describe Ctrl-left drag / keyboard nudge controls.                                                            |
| Demonstrate a custom world node with pre/post refresh and pre/post step hooks.                    | Restored through public `dart::gui`. | `ApplicationOptions::preStep`, `postStep`, `preRender`, and `postRender` preserve the lifecycle-hook scaffold without world-node types. |
| Demonstrate custom keydown handling for `q`, `Q`, Left, and Right.                                | Restored.                            | Current source uses renderer-neutral `KeyboardAction` callbacks.                                                                        |
| Demonstrate custom keyup handling for `q`, `Q`, Left, and Right.                                  | Restored through public `dart::gui`. | `KeyboardActionTrigger::Release` restores key-release callbacks without exposing backend event types.                                   |
| Default launch size 640x480.                                                                      | Restored.                            | Current source uses `ApplicationOptions::runDefaults`.                                                                                  |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0)` with up `(-0.24, -0.25, 0.94)`.   | Restored.                            | Current source restores the historical home and up vector through `dart::gui::OrbitCamera`.                                             |
| README documents the promoted runner and no longer says the standalone source remains OSG/legacy. | Restored.                            | Current README documents `pixi run ex empty`, headless capture, promoted controls, and application callback hooks.                      |

### Simulation Event Handler Itemized Inventory

Historical sources compared:
`520993d7301^:examples/simulation_event_handler/main.cpp`,
`520993d7301^:examples/simulation_event_handler/simulation_event_handler.cpp`,
and `520993d7301^:examples/simulation_event_handler/README.md`.

The source-owned checkpoint restores the historical event-handler controls
through promoted `dart::gui` keyboard actions and DART-owned line geometry,
without reviving OSG viewer/event types. The shifted-slash help alias is now
represented as `KeyboardShortcut::characterKey('?')` and mapped by the private
Filament input bridge.

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
| Print simulation state with `i` and help with `h`/`?`.                                                              | Restored.                            | Current source restores `i`, `h`, and shifted-slash `?` through promoted keyboard actions.                                          |
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
and a public `dart::gui::Gizmo` replace the draggable-frame path.

| Historical item                                                                                | Current outcome                       | Notes                                                                                                          |
| ---------------------------------------------------------------------------------------------- | ------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Create an interactive frame named `interactive frame` at `(4, -4, 0)` with scale `2.0`.        | Restored with promoted frame visuals. | Current source uses one selectable `SimpleFrame` and a public `dart::gui::Gizmo` for the visible handle.       |
| Attach the red `draggable` box as a child frame at local `(-4, 4, 0)` with unit box geometry.  | Restored.                             | Current source preserves the parent/child transform relationship and red visual.                               |
| Add world X/Y/Z marker boxes at `(8, 0, 0)`, `(0, 8, 0)`, and `(0, 0, 8)` with size `0.2`.     | Restored.                             | Current source keeps the historical positions, names, sizes, and red/green/blue colors.                        |
| Enable drag-and-drop on the interactive frame and child box.                                   | Restored through promoted selection.  | The shared runner supports click selection, public gizmo dragging, and keyboard nudging.                       |
| Show the historical visible manipulation affordance rather than only a tiny placeholder box.   | Restored with public `dart::gui`.     | Current source adds a public gizmo with axis arrows, plane handles, and rotation rings on the frame.           |
| Support the historical rotation manipulation.                                                  | Restored with public `dart::gui`.     | The public gizmo exposes rotation rings without reintroducing OSG `InteractiveFrame`.                          |
| Print/show the example interaction instructions.                                               | Restored with promoted wording.       | Current source explains selection, public gizmo dragging, and keyboard nudging.                                |
| Default launch size 640x480.                                                                   | Restored.                             | Current source uses `ApplicationOptions::runDefaults`.                                                         |
| Camera home from eye `(20, 17, 17)` to target `(0, 0, 0)` with world-up `(0, 0, 1)`.           | Restored.                             | Current source converts the historical home to `dart::gui::OrbitCamera`.                                       |
| README documents the promoted runner and no longer calls the standalone source legacy OSG.     | Restored.                             | Current README documents `pixi run ex drag_and_drop`, controls, defaults, headless capture, and public gizmos. |
| Keep the source free of OSG viewer, world-node, InteractiveFrame, and backend-specific tokens. | Restored through guard test.          | Marker coverage prevents reintroducing removed renderer surfaces and the old source-owned line handle.         |

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

| Historical item                                                                                        | Current outcome                          | Notes                                                                                                                                                        |
| ------------------------------------------------------------------------------------------------------ | ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Parse `--gui-scale`.                                                                                   | Restored through the promoted runner.    | Shared `dart::gui::runApplication` owns the GUI-scale option.                                                                                                |
| Create an empty world and add a target frame.                                                          | Restored through public `dart::gui`.     | Current source creates an empty world and one `SimpleFrame` target with a public `dart::gui::Gizmo` affordance instead of source-owned line-handle geometry. |
| Enable drag-and-drop for the target.                                                                   | Restored through promoted controls.      | Public gizmo arrows, planes, and rings move the target frame without legacy interaction types.                                                               |
| Demonstrate custom pre-step hook behavior.                                                             | Restored.                                | Current source uses `ApplicationOptions::preStep` and displays a pre-step callback count.                                                                    |
| Demonstrate custom pre/post refresh and post-step hooks.                                               | Restored through public `dart::gui`.     | `ApplicationOptions::postStep`, `preRender`, and `postRender` complete the lifecycle-hook scaffold without world-node types.                                 |
| Demonstrate keydown handling for `q`, `Q`, Left, and Right.                                            | Restored through public `dart::gui`.     | Current source restores shifted `Q` separately from lowercase `q` through promoted `KeyboardShortcut` case handling.                                         |
| Demonstrate key-release handling for `q`, `Q`, Left, and Right.                                        | Restored through public `dart::gui`.     | `KeyboardActionTrigger::Release` restores key-release callbacks without exposing backend event types.                                                        |
| Panel title `Tinkertoy Control`.                                                                       | Restored.                                | Current source uses the historical panel title through public `dart::gui::Panel`.                                                                            |
| Historical backend-specific text `An empty OSG example with ImGui`.                                    | Superseded by promoted renderer wording. | Restore the educational meaning without using removed backend names in source.                                                                               |
| Menu affordances for Exit and About DART.                                                              | Restored with promoted panel controls.   | Public panel button/text affordances replace menu-bar placement.                                                                                             |
| Simulation section with Play/Pause and Time display.                                                   | Restored.                                | Current source uses `PanelContext::lifecycle` and `simulationTime`.                                                                                          |
| World Options section with Gravity On/Off.                                                             | Restored.                                | Current source toggles `World::setGravity` between `-9.81 * UnitZ` and zero.                                                                                 |
| World Options section with Headlights On/Off.                                                          | Restored through public `dart::gui`.     | `PanelContext::lighting.headlightsEnabled` exposes renderer-neutral scene-light state without backend hooks.                                                 |
| View section with Eye, Center, and Up text.                                                            | Restored through public `dart::gui`.     | `PanelContext::camera` exposes live eye, target, and up vectors for renderer-neutral panel readouts.                                                         |
| Help section showing shared viewer instructions.                                                       | Restored with promoted flat panel text.  | Current source documents orbit/pan/zoom, target gizmo dragging, and keydown callbacks.                                                                       |
| Default launch size 640x480.                                                                           | Restored.                                | Current source uses `ApplicationOptions::runDefaults`.                                                                                                       |
| Camera home from eye `(2.57, 3.14, 1.64)` to target `(0, 0, 0)` with up `(-0.24, -0.25, 0.94)`.        | Restored.                                | Current source restores the historical eye/target framing and up vector through `dart::gui::OrbitCamera`.                                                    |
| README documents the promoted runner, controls, defaults, headless capture, and remaining public gaps. | Restored.                                | Current README documents `pixi run ex imgui`, controls, defaults, capture, and application callback hooks.                                                   |
| Keep source free of direct backend UI, viewer, event-handler, and OSG types.                           | Restored through marker guard.           | Source avoids the forbidden backend tokens scanned by `UNIT_gui_FilamentSceneExtraction`.                                                                    |

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

| Historical item                                                                                                                 | Current outcome                      | Notes                                                                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------- |
| Build the example from source-owned controller, state, state-machine, and terminal-condition files.                             | Restored through public `dart::gui`. | These files are restored as maintained example code without OSG types.                                                              |
| Load `dart://sample/sdf/atlas/ground.urdf` and `dart://sample/sdf/atlas/atlas_v3_no_head.sdf`.                                  | Restored.                            | Current source loads the historical ground and Atlas skeletons.                                                                     |
| Set Atlas root position 0 to `-0.5 * pi` and world gravity to `(0, -9.81, 0)`.                                                  | Restored.                            | Current source restores the root position and Y-down gravity.                                                                       |
| Run the Simbicon controller before each simulation step and apply temporary external pelvis pushes.                             | Restored through public `dart::gui`. | `ApplicationOptions::preStep` now runs the controller and external-force countdown.                                                 |
| Keyboard actions: `r` reset, `a`/`s`/`d`/`f` perturb forward/back/left/right.                                                   | Restored.                            | Historical event-handler behavior is migrated to renderer-neutral `KeyboardAction`s.                                                |
| Controller actions: harness pelvis/feet, manual next-state, and `1`/`2`/`3`/`4` state-machine selection.                        | Restored.                            | Harness and next-state controls are exposed in the panel; state-machine selection is exposed by keyboard actions and panel buttons. |
| ImGui panel title `Atlas Control`, Help text, Play/Pause, gravity slider, reset button, and stride mode controls.               | Restored through public `dart::gui`. | `PanelBuilder` provides the promoted text/button/checkbox/slider replacement.                                                       |
| Headlights On/Off checkbox.                                                                                                     | Restored through public `dart::gui`. | `PanelContext::lighting.headlightsEnabled` exposes renderer-neutral scene-light state without backend hooks.                        |
| OSG shadow toggle.                                                                                                              | Restored through public `dart::gui`. | Public `RenderSettings::shadowsEnabled` exposes renderer-neutral shadow state without backend hooks.                                |
| OSG depth mode toggle.                                                                                                          | Public API gap.                      | Needs a renderer-neutral render-output mode before exact parity; do not revive OSG or private Filament hooks in the example source. |
| Window title `Atlas Simbicon`, 1280x960 launch size, and camera home from `(5.14, 3.28, 6.28) * 2` to `(1, 0, 0)`.              | Restored through public `dart::gui`. | Run defaults, native window title, and camera target are restored through public `RunOptions` and `ApplicationOptions`.             |
| README documents the Simbicon controller workflow and promoted runner.                                                          | Restored.                            | Current README documents the promoted runner and remaining depth-output gap.                                                        |
| Keep source-marker coverage for controller files, pre-step update, controls, camera/defaults, README, and absence of OSG types. | Restored through marker guard.       | `UNIT_gui_FilamentSceneExtraction` guards the restored contract.                                                                    |

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

The historical and pre-removal branch trees were identical: `rerun` was only a
placeholder directory with no source files. New maintainer steering re-opened
the decision and found no concrete integration workflow, executable, or known
downstream user, so the placeholder is removed instead of carried as an
official example.

2026-05-16 support audit after the soft-bodies checkpoint found no remaining
product support surface to delete: no CMake/package/API/runtime references and
no filesystem path named Rerun remain. Future Rerun work should enter only as a
real sourced example or application feature with a concrete use case.

| Historical item                                                                                       | Current outcome                     | Notes                                                                                                                            |
| ----------------------------------------------------------------------------------------------------- | ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| Directory contains README and CMake scaffolding only, with no `.cpp` or `.hpp` sources.               | Removed as maintenance-only stub.   | There is no source to migrate to public `dart::gui`, no executable contract, and no concrete workflow to validate.               |
| CMake discovers local sources and returns early when none exist.                                      | Removed.                            | The official examples build no longer carries an empty early-skip subdirectory.                                                  |
| Placeholder links only future `utils-urdf` requirements after sources exist.                          | Removed.                            | Future Rerun integration should be added as a real example with its actual dependencies when a use case exists.                  |
| README describes future Rerun integration scaffolding and states that source files are pending.       | Removed.                            | Pending-source documentation is not useful enough to justify a maintained example entry.                                         |
| Keep marker coverage for no-source status, early CMake skip, pending-source README, and no GUI types. | Replaced by removal guard.          | `UNIT_gui_FilamentSceneExtraction` now guards that the placeholder directory, CMake subdirectory, and README entry stay removed. |
| Runtime or capture validation.                                                                        | Not applicable; no example remains. | There was intentionally no `rerun` executable before removal.                                                                    |

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

| Historical item                                                                                                 | Current outcome                      | Notes                                                                                                                                             |
| --------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Load Atlas from `dart://sample/sdf/atlas/atlas_v3_no_head.urdf`, set Z-down gravity, and create a ground plane. | Restored through public `dart::gui`. | Current source owns the loaded world and passes it through `ApplicationOptions::world`; gravity also keeps support-polygon queries populated.     |
| Initial squat/arm posture and feet on the ground.                                                               | Restored.                            | Current source sets the historical leg/arm DOFs and computes ground clearance from visual bounds.                                                 |
| Four IK targets for left/right hand and left/right foot, with foot support geometry.                            | Restored through public `dart::gui`. | Current source creates target states, visible active-target handles, and support geometry without backend types.                                  |
| Number keys `1`-`4` toggle target constraints and add/remove visible target frames.                             | Restored through keyboard actions.   | Current source toggles active target state, resets activated targets to the current end-effector transform, and adds/removes target frames.       |
| WASD/QE/FZ root teleoperation.                                                                                  | Restored through public `dart::gui`. | Current source maps these to repeatable keyboard actions and resolves IK after root motion.                                                       |
| `X`/`C` support toggles, `R` posture optimization, `T` relaxed-pose reset, and `P` DOF printing.                | Restored through keyboard actions.   | X/C/P/T and R press/release are public keyboard actions.                                                                                          |
| Whole-body `RelaxedPosture` objective and `BalanceConstraint` recovery behavior.                                | Restored through public DART APIs.   | Current source restores the source-owned relaxed-posture objective, balance constraint, and whole-body IK solve path.                             |
| Default support polygon visual at elevation `0.05`, COM marker, and support-area feedback.                      | Restored with source-owned geometry. | Current source restores the always-on support polygon and centroid as green line geometry, plus a blue/red COM validity marker.                   |
| 1280x960 default window and camera home from `(5.34, 3.00, 2.41)` to `(0, 0, 1)`.                               | Restored.                            | `ApplicationOptions::runDefaults` and `ApplicationOptions::camera` now preserve the historical defaults.                                          |
| README documents the runner, kinematic mode, IK targets, root teleoperation, and balance controls.              | Restored.                            | README was restored with promoted `dart::gui` wording.                                                                                            |
| Keep marker coverage for load/posture/targets/teleop/defaults/README/no OSG plus named remaining parity gaps.   | Restored for this slice.             | Marker guards now cover active target toggles, support controls, R balance controls, support/COM overlays, README/defaults, and no backend types. |

Follow-up Atlas target/support validation passed before and after lint:
focused `atlas_puppet` and `UNIT_gui_FilamentSceneExtraction` build, focused
CTest, and `pixi run ex atlas_puppet --headless --frames 2 --width 640
--height 480 --screenshot ...` with the headless smoke analyzer
(`304623/307200` nonzero pixels post-lint).

Follow-up Atlas whole-body solver validation passed after lint: focused
`atlas_puppet` and `UNIT_gui_FilamentSceneExtraction` build, focused CTest, and
Atlas headless smoke analyzer coverage (`304624/307200` nonzero pixels).

### G1 Puppet Itemized Inventory

Historical source compared: `520993d7301^:examples/g1_puppet`.

The current source is already a real promoted `dart::gui` example and restores
remote package loading, visible IK targets, and active-target solving. Strict
re-open found remaining user-visible gaps: the README is missing, run defaults
and camera home are absent, the world gravity/ground/root pose drifted from the
historical scene, the OSG grid is not yet represented as source-owned geometry,
and support/body manipulation affordances need explicit promoted replacements.

| Historical item                                                                                          | Current outcome                                | Notes                                                                                                                                       |
| -------------------------------------------------------------------------------------------------------- | ---------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `package://g1_description/g1_29dof.urdf` through package/http/file retrievers with short CLI flags. | Restored through public `dart::gui`.           | Current source accepts historical `--package-uri`, `--robot-uri`, and `--package-name` plus explicit `--g1-*` aliases.                      |
| World gravity `(0, 0, -9.81)`, 8x8x0.1 gray ground named `ground`, and G1 root FreeJoint z `0.75`.       | Restored.                                      | Current source now matches the historical scene setup through public DART objects.                                                          |
| XY grid with 40 cells, 0.1 minor line step, and zero offset.                                             | Restored with source-owned geometry.           | Current source uses DART `LineSegmentShape` geometry rather than OSG `GridVisual`.                                                          |
| Four number-key IK targets for hands/feet, inactive until toggled, then added/removed from the world.    | Restored through promoted target states.       | Current source toggles visible `SimpleFrame` targets and solves only active targets. Add marker coverage for activation/deactivation text.  |
| Enable mouse drag on G1 body nodes.                                                                      | Public API gap / partial promoted replacement. | Public selection can translate free-root renderables and simple frames, but per-body articulated drag parity is not a dedicated public API. |
| Default support polygon visual at elevation `0.02`.                                                      | Restored with source-owned debug lines.        | Current source builds an always-visible support overlay from public `dart::gui::makeSupportPolygonDebugLines`.                              |
| 1280x960 window and camera home from `(3.0, 1.6, 1.4)` to `(0, 0, 0.75)`.                                | Restored.                                      | `ApplicationOptions::runDefaults` and `OrbitCamera` now preserve the historical defaults.                                                   |
| README documents package overrides, kinematic mode, IK targets, controls, build/run instructions.        | Restored.                                      | README was restored with promoted `dart::gui` wording and the remaining body-drag API follow-up.                                            |
| Keep marker coverage for package loading, scene defaults, grid, support overlay, IK toggles, README.     | Restored for this slice.                       | Marker guards now cover the strict scene/default/README contract.                                                                           |

### Hubo Puppet Itemized Inventory

Historical source compared: `520993d7301^:examples/hubo_puppet`.

The current source is a promoted `dart::gui` example and already restores local
URDF loading, finger removal, startup posture, visible IK targets, root
teleoperation, target-handle dragging, target toggles, support overlays, and
source-owned posture/balance recovery, analytical IK, and Shift-amplified root
movement.

| Historical item                                                                                        | Current outcome                           | Notes                                                                                                                                          |
| ------------------------------------------------------------------------------------------------------ | ----------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| Load `urdf/drchubo/drchubo.urdf`, remove finger bodies, and restore the squat/arm startup posture.     | Restored through public `dart::gui`.      | Current source owns the loaded Hubo skeleton and startup configuration.                                                                        |
| 10x10x0.01 blue ground named `ground`.                                                                 | Restored.                                 | Current source now matches the historical ground through public DART objects.                                                                  |
| Six IK targets named `lh/rh/lf/rf/lp/rp` historically toggled by keys `1`-`6`.                         | Restored through promoted target states.  | Current source adds/removes target frames on number-key actions and solves only active target states.                                          |
| Source-owned `HuboArmIK` and `HuboLegIK` analytical gradient methods.                                  | Restored.                                 | Current source restores analytical arm/leg IK methods and post-analytical extra DOF behavior for hand targets.                                 |
| WASD/QE/FZ root teleoperation.                                                                         | Restored through public keyboard actions. | Current source maps normal movement and Shift-uppercase amplified movement to repeatable actions.                                              |
| `X`/`C` toggles left/right foot support, `P` prints DOFs, `T` resets relaxed posture.                  | Restored through keyboard actions.        | Current source implements these as renderer-neutral actions without OSG event handlers.                                                        |
| Hold `R` to optimize posture and balance, then release to restore centroid mode.                       | Restored through keyboard actions.        | Current source restores the source-owned relaxed-posture objective, balance constraint, whole-body IK solve path, and R press/release actions. |
| Default support polygon visual at elevation `0.05`, COM marker, and support centroid feedback.         | Restored with source-owned geometry.      | Current source restores the always-on support polygon and centroid as green line geometry, plus a blue/red COM validity marker.                |
| 1280x960 window and camera home from `(5.34, 3.00, 1.91)` to `(0, 0, 0.50)` with custom up vector.     | Restored through public `dart::gui`.      | `OrbitCamera` restores the historical eye/target framing and up vector.                                                                        |
| README documents kinematic mode, support polygon, keyboard controls, build/run instructions.           | Restored.                                 | README was restored with promoted `dart::gui` wording, analytical IK, and Shift-amplified movement controls.                                   |
| Keep marker coverage for load/start pose, target toggles, support overlay, controls, defaults, README. | Restored for this slice.                  | Marker guards now cover the strict target/default/README contract, source-owned COM validity overlay, and R balance controls.                  |

Follow-up COM-overlay validation passed: focused `hubo_puppet` and
`UNIT_gui_FilamentSceneExtraction` build, focused CTest, and `pixi run ex
hubo_puppet --headless --frames 2 --width 640 --height 480 --screenshot ...`
with the headless smoke analyzer.

Follow-up Hubo whole-body solver validation passed after lint: focused
`hubo_puppet` and `UNIT_gui_FilamentSceneExtraction` build, focused CTest, and
Hubo headless smoke analyzer coverage (`307200/307200` nonzero pixels).

Follow-up Hubo analytical IK validation passed after lint: focused `hubo_puppet`
and `UNIT_gui_FilamentSceneExtraction` build, focused CTest, and Hubo headless
smoke analyzer coverage (`307200/307200` nonzero pixels).

Follow-up Hubo Shift movement validation passed after lint: focused
`hubo_puppet` and `UNIT_gui_FilamentSceneExtraction` build, focused CTest, and
Hubo headless smoke analyzer coverage (`307200/307200` nonzero pixels).

### Hybrid Dynamics Itemized Inventory

Historical source compared: `520993d7301^:examples/hybrid_dynamics`.

The current source is a promoted `dart::gui` example and restores the loaded
`fullbody1.skel` world, gravity, initial generalized coordinates, scripted
velocity commands, `h` harness keyboard action, camera direction, panel
controls, README, and historical console messages. The historical custom camera
up vector is now restored through public `OrbitCamera::up`.

| Historical item                                                                                       | Current outcome                                 | Notes                                                                                                                      |
| ----------------------------------------------------------------------------------------------------- | ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/fullbody1.skel` and set gravity to `(0, -9.81, 0)`.                          | Restored through public `dart::gui`.            | Current source loads the SKEL world and passes it through `ApplicationOptions::world`.                                     |
| Preserve the loaded ground/biped names and SKEL-authored visuals.                                     | Restored.                                       | Current source now uses the historical `ground skeleton` and `fullbody1` names and no longer recolors the loaded bodies.   |
| Set the same eight initial generalized coordinates.                                                   | Restored.                                       | Current source keeps the historical DOF ids and numeric pose values.                                                       |
| Root joint passive; all other joints velocity controlled.                                             | Restored.                                       | Current source uses the same actuator-type split.                                                                          |
| Pre-step scripted commands for scapula, forearm, and shin joints using `sin(world time * 4/2)`.       | Restored through `ApplicationOptions::preStep`. | Current source preserves the command waveforms and joint names.                                                            |
| `h`/`H` toggles pelvis harness lock/passive and prints "The pelvis is locked/unlocked."               | Restored.                                       | Current source prints the historical messages from the promoted harness action used by the keyboard and panel.             |
| User-visible instructions include `'h': toggle harness on/off` and `space bar: simulation on/off`.    | Restored.                                       | Current source prints the instructions to the console and shows them in the panel.                                         |
| Default 640x480 window and camera home from `(5, 3, 3)` to `(0, 0, 0)` with up vector `(0, 1, 0)`.    | Restored through public `dart::gui`.            | `ApplicationOptions::runDefaults` and `OrbitCamera` restore the historical window size, eye/target framing, and up vector. |
| README documents concepts, controls, standalone build/execute instructions, and promoted runner path. | Restored.                                       | README is restored with promoted `dart::gui` wording and headless capture guidance.                                        |
| Keep marker coverage for world setup, harness console messages, instructions, run defaults, README.   | Restored for this slice.                        | Marker guard now covers the strict restored contract.                                                                      |

### Joint Constraints Itemized Inventory

Historical source compared: `520993d7301^:examples/joint_constraints`.

The current source is a promoted `dart::gui` example and already restores the
`fullbody1.skel` world, SPD controller, perturbation forces, harness
constraint, camera direction, and panel controls. Strict re-open found concrete
remaining gaps: missing README, missing 640x480 launch default, renamed and
recolored loaded SKEL skeletons/body visuals, missing historical perturbation
and harness console messages, and incomplete marker coverage for the restored
contract. The historical custom camera up vector is now restored through
public `OrbitCamera::up`.

| Historical item                                                                                          | Current outcome                                 | Notes                                                                                                                      |
| -------------------------------------------------------------------------------------------------------- | ----------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/fullbody1.skel` and set gravity to `(0, -9.81, 0)`.                             | Restored through public `dart::gui`.            | Current source loads the SKEL world and passes it through `ApplicationOptions::world`.                                     |
| Preserve the loaded ground/biped names and SKEL-authored visuals.                                        | Restored.                                       | Current source uses the historical `ground skeleton` and `fullbody1` names and no longer recolors the loaded bodies.       |
| Set the same nine initial generalized coordinates and SPD desired pose.                                  | Restored.                                       | Current source keeps the historical DOF ids, numeric pose values, controller gains, and ankle compensation behavior.       |
| Pre-step SPD control and perturbation-force decay.                                                       | Restored through `ApplicationOptions::preStep`. | Current source preserves controller pre-step behavior and force duration state.                                            |
| `1`/`2`/`3`/`4` apply forward/backward/right/left perturbations and print `push ...` messages.           | Restored through public keyboard actions.       | Current source emits the historical console messages from both keyboard actions and promoted panel buttons.                |
| `h`/`H` toggles the pelvis harness and prints `Harness on` / `Harness off`.                              | Restored.                                       | Current source prints the historical messages from the promoted harness action used by the keyboard and panel.             |
| User-visible instructions include perturbation keys, harness toggle, and `space bar: simulation on/off`. | Restored.                                       | Current source prints the instructions to the console and shows equivalent controls in the panel/README.                   |
| Default 640x480 window and camera home from `(5, 3, 3)` to `(0, 0, 0)` with up vector `(0, 1, 0)`.       | Restored through public `dart::gui`.            | `ApplicationOptions::runDefaults` and `OrbitCamera` restore the historical window size, eye/target framing, and up vector. |
| README documents concepts, controls, standalone build/execute instructions, and promoted runner path.    | Restored.                                       | README is restored with promoted `dart::gui` wording and headless capture guidance.                                        |
| Keep marker coverage for world setup, perturb/harness console messages, defaults, README, no OSG path.   | Restored for this slice.                        | Marker guard now covers the strict restored contract.                                                                      |

### LCP Physics Itemized Inventory

Historical source compared: `520993d7301^:examples/lcp_physics`.

The current source is a promoted `dart::gui` example with local scenario
builders, Dantzig/PGS solver selection, 1280x720 defaults, and shared
headless/capture parsing. Strict re-open found concrete gaps: the scenes were
scaled down from the historical source, names/placement drifted, the panel lost
live scenario/solver/reset/timestep/gravity controls, README lost the
command-line option inventory, and marker coverage guarded only a subset of the
strict restored contract. The implementation now repairs the expressible
behavior through public `dart::gui`; source-owned FPS and step-time summaries
are restored, while exact ImGui line plots and display/font debug metrics
remain a public panel plotting/backend-debug API gap.

| Historical item                                                                                                 | Current outcome                           | Notes                                                                                                                                                                                                   |
| --------------------------------------------------------------------------------------------------------------- | ----------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Provide five scenarios: mass ratio, box pyramid, 75-ball drop, 20-domino chain, and inclined plane.             | Restored through source-owned DART world. | Current source restores historical scene sizes, counts, names, colors, placements, seeded jitter, and friction/restitution setup.                                                                       |
| Preserve Dantzig and PGS LCP solver selection from CLI and panel controls.                                      | Restored through public `dart::gui`.      | CLI `--solver` works; promoted panel buttons switch the active solver on the live world.                                                                                                                |
| Preserve scenario selection from CLI, `--list`, and panel controls.                                             | Restored through public `dart::gui`.      | CLI scenario parsing works; promoted panel buttons replace the live world contents with the selected scenario.                                                                                          |
| Preserve Simulation controls: Play/Pause, Step, Reset, simulation time, and step count.                         | Restored through public `dart::gui`.      | Promoted lifecycle buttons cover Play/Pause/Step; the source-owned state adds Reset and step count.                                                                                                     |
| Preserve parameter controls: timestep slider from 100 to 2000 Hz and gravity slider from 0 to 20 m/s^2.         | Restored through public `dart::gui`.      | `PanelBuilder::slider` updates the current world timestep and gravity.                                                                                                                                  |
| Preserve performance/debug information: FPS, contacts, body count, step-time plot, ImGui display/font metrics.  | Mostly restored; public API gap.          | Render FPS, rendered/skipped frames, contacts, body count, and step-time summary are shown now. Exact line plots and ImGui-specific display/font metrics still need public plotting/backend-debug APIs. |
| Preserve scenario explanations and research references in the panel/README.                                     | Restored.                                 | Current source carries scenario explanation text and README restores options/references.                                                                                                                |
| Preserve 1280x720 default launch size, camera home from `(3, 2, 3)` to `(0, 0.3, 0)`, and `--gui-scale`.        | Restored through public `dart::gui`.      | `RunOptions`, `OrbitCamera`, and shared runner parsing cover these.                                                                                                                                     |
| Preserve headless mode, `--frames`, `--out`, `--width`, `--height`, `--gui-scale`, and list/selection examples. | Restored with promoted capture semantics. | Shared `dart::gui` runner owns headless/capture flags; README documents `--out` image sequences and `--screenshot`.                                                                                     |
| Keep marker coverage for scene counts, solver/scenario controls, panel controls, README, and no backend types.  | Restored for this slice.                  | Marker guards now cover scene counts/names, live controls, README options, capture flags, public API gap text, and absence of legacy launcher.                                                          |

Pre-lint validation for this slice passed: focused `lcp_physics` and
`UNIT_gui_FilamentSceneExtraction` build, focused CTest, direct `--list`, direct
software-GL screenshot and `--out` image-sequence analyzer checks, pixi runner
screenshot analyzer check, Python example-runner tests, aggregate `examples`
build, and `git diff --check`. Mandatory `pixi run lint`, post-lint focused
build/CTest, direct `--list`, software-GL screenshot analyzer check, and
post-lint `git diff --check` also passed.

Follow-up metric-diagnostics validation passed: focused `lcp_physics` and
`UNIT_gui_FilamentSceneExtraction` build, focused CTest, `pixi run ex
lcp_physics --list`, direct `dominos`/PGS headless screenshot analyzer smoke,
`pixi run lint`, post-lint focused build/CTest, aggregate `build-examples`, and
post-lint `pixi run ex lcp_physics --scenario dominos --solver pgs --headless
--frames 2 --screenshot ...` analyzer smoke.

### Mimic Pendulums Itemized Inventory

Historical source compared: `520993d7301^:examples/mimic_pendulums`.

The promoted `dart::gui` source now loads the SDF world, retargets mimic joints
to the baseline pendulum, preserves the original rig names, colors the
historical rig bases, adds a source-owned XY grid, exposes live reset,
ODE-collision, and PGS-solver controls, reports reference/follower/error/base
drift diagnostics, and documents shared headless capture. The local R24-21
panel-layout checkpoint restores the legend color swatches and diagnostics
table through public `PanelBuilder` primitives.

| Historical item                                                                                      | Current outcome / repair plan           | Notes                                                                                                                   |
| ---------------------------------------------------------------------------------------------------- | --------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/sdf/test/mimic_fast_slow_pendulums_world.sdf`.                                   | Restored through public `dart::gui`.    | Current source uses `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                     |
| Retarget mimic joints to the uncoupled `pendulum_with_base` baseline and clear baseline mimic props. | Restored.                               | Current source keeps the historical retargeting behavior and collects reference/follower pairs.                         |
| Preserve original rig names and color the baseline/red/blue rig bases for the legend.                | Restored.                               | Source keeps SDF skeleton names and uses the historical base-color palette for the panel legend.                        |
| Preserve the 20-cell XY grid attachment as a visual reference.                                       | Restored with source-owned `dart::gui`. | The old OSG `GridVisual` is represented by a DART-owned `LineSegmentShape` grid rather than a renderer-private fixture. |
| Preserve live simulation controls: Run simulation, Step 1, and Reset world.                          | Restored.                               | Reset refreshes the base-drift anchors after resetting the world.                                                       |
| Preserve live collision/solver controls: ODE collision checkbox and PGS solver checkbox.             | Restored.                               | Launch flags still work; promoted panel checkboxes reconfigure the current world.                                       |
| Preserve mimic diagnostics: pair label, reference/follower angles, position error, velocity error.   | Restored through public panel table.    | Current source uses `PanelBuilder::beginTable`, row/column calls, and text fallback.                                    |
| Preserve base drift diagnostics and resettable drift anchors.                                        | Restored.                               | Reset world updates anchor positions through source-owned state.                                                        |
| Preserve default 1280x720 launch size, camera home, and `--gui-scale`.                               | Restored through public `dart::gui`.    | `RunOptions`, `OrbitCamera`, and shared runner parsing cover these.                                                     |
| Preserve headless/capture options and document solver/collision flags.                               | Restored.                               | README includes promoted `--headless`, `--frames`, `--screenshot`, `--out`, `--width`, `--height`, and `--gui-scale`.   |
| Keep marker coverage for world URI, rig names/colors, grid, controls, diagnostics, README.           | Restored.                               | Marker guards cover the restored contract and reject the old private-scene handoff.                                     |

Validation for this slice includes focused `mimic_pendulums` and
`UNIT_gui_FilamentSceneExtraction` builds, focused CTest, direct `--help`,
software-GL screenshot and image-sequence analyzer checks, aggregate
`build-examples`, Python C++ example runner tests, mandatory `pixi run lint`,
post-lint focused build/CTest/screenshot checks, and `git diff --check`.

### Mixed Chain Itemized Inventory

Historical source compared: `520993d7301^:examples/mixed_chain`.

The promoted `dart::gui` source now owns the SKEL-loaded world, restores the
historical random startup pose, applies short impulses through
`ApplicationOptions::preStep`, wires six keyboard actions, exposes panel
impulse buttons, prints the historical console instructions, sets the
historical 640x480 default and camera, and documents shared headless capture.

| Historical item                                                                                       | Current outcome / repair plan        | Notes                                                                                |
| ----------------------------------------------------------------------------------------------------- | ------------------------------------ | ------------------------------------------------------------------------------------ |
| Load `dart://sample/skel/test/test_articulated_bodies_10bodies.skel` and operate on skeleton index 1. | Restored through public `dart::gui`. | Current source loads the same SKEL world and renames the active chain for lookup.    |
| Initialize the first three DOFs with `dart::math::Random::uniform(-0.5, 0.5)`.                        | Restored.                            | Screenshot smoke checks stay nonblank despite the historical startup randomness.     |
| Apply a 500 N force to soft body node index 3 for 100 frames.                                         | Restored.                            | Current source uses `kForceMagnitude`, `kImpulseFrames`, and `getSoftBodyNode(3)`.   |
| Preserve `q`/`w`, `e`/`r`, and `t`/`y` impulse keyboard controls.                                     | Restored.                            | Current source uses renderer-neutral `KeyboardAction`s.                              |
| Preserve the printed control instructions.                                                            | Restored.                            | Source prints the historical control block before running the public application.    |
| Preserve the 640x480 launch default and historical camera framing.                                    | Restored.                            | `RunOptions` and `OrbitCamera` cover these.                                          |
| Preserve promoted panel controls and diagnostics for remaining impulse frames/time.                   | Restored.                            | Public panel text/buttons are acceptable for this example.                           |
| Preserve README/run/capture documentation.                                                            | Restored.                            | README covers `pixi run ex mixed_chain`, `--screenshot`, and `--out`.                |
| Keep marker coverage for URI, random pose, force magnitude/duration, keys, defaults, README.          | Restored.                            | Marker guard covers the restored contract and rejects the old private-scene handoff. |

Validation for this slice includes focused `mixed_chain` and
`UNIT_gui_FilamentSceneExtraction` builds, focused CTest, direct and pixi
software-GL screenshot analyzer checks, image-sequence analyzer checks,
aggregate `build-examples`, Python C++ example runner tests, mandatory
`pixi run lint`, post-lint focused build/CTest/screenshot checks, and
`git diff --check`.

### Operational Space Control Itemized Inventory

Historical source compared:
`520993d7301^:examples/operational_space_control`.

Strict re-open found and repaired a major scope drift: the promoted source was
loading a WAM arm, while the historical example is a KR5 operational-space
controller. The current source restores the KR5 robot and ground, target ball,
controller math, 640x480 default, camera home, console/panel instructions,
README/capture docs, and marker coverage through public `dart::gui`. The
promoted selection controller supports selectable frames, Ctrl-left drag,
keyboard nudging, X/Y/Z axis-constrained drag, and now 1/2/3 axis aliases so
the historical constraint keys work as well.

| Historical item                                                                                                                            | Current outcome                      | Notes                                                                                                               |
| ------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/urdf/KR5/KR5 sixx R650.urdf` and reset the root transform to identity.                                                 | Restored through public `dart::gui`. | Current source uses `dart::io::readSkeleton`, resets the root joint to identity, and names the robot `KR5`.         |
| Load `dart://sample/urdf/KR5/ground.urdf`, translate it by `(0, 0, 0.5)`, and rotate X by `pi / 2`.                                        | Restored.                            | Current source reuses the historical transform through public `dart::io`.                                           |
| Disable joint limit enforcement and set damping coefficient `0` to `0.5` on each joint.                                                    | Restored.                            | Current source preserves the historical one-DOF damping setup for KR5 joints.                                       |
| Create the red target `SimpleFrame` from a `SphereShape(0.025)` at end-effector plus `(0.05, 0, 0)`.                                       | Restored.                            | Current source restores the red ball target instead of the prior line-segment handle.                               |
| Preserve operational-space controller math: mass matrix, Jacobian/Jacobian derivative, `Kp=50`, `Kd=5`, Coriolis/gravity, and `setForces`. | Restored on the KR5 end effector.    | Current controller keeps the historical formula through `ApplicationOptions::preStep`.                              |
| Preserve drag target controls and axis constraints.                                                                                        | Restored through public `dart::gui`. | Public runner uses click selection, Ctrl-left drag, arrow/PageUp/PageDown nudges, and X/Y/Z plus 1/2/3 constraints. |
| Preserve printed instructions.                                                                                                             | Restored.                            | Source prints console instructions and mirrors the interaction contract in a promoted panel.                        |
| Preserve `s` shadow toggle.                                                                                                                | Restored through public `dart::gui`. | Public `KeyboardActionContext::renderSettings` toggles renderer-neutral shadow state without backend hooks.         |
| Preserve 640x480 launch default and camera eye/target framing.                                                                             | Restored through public `dart::gui`. | `OrbitCamera` restores the historical target, eye framing, and custom up vector.                                    |
| Preserve README/run/capture documentation.                                                                                                 | Restored.                            | README covers `pixi run ex operational_space_control`, controls, `--screenshot`, and `--out`.                       |
| Keep marker coverage for KR5 load, ground transform, controller math, target ball, controls, defaults, README, and no private scene.       | Restored.                            | Marker guard covers the restored contract and rejects the old WAM/private-scene drift.                              |

Validation for this slice includes focused `operational_space_control` and
`UNIT_gui_FilamentSceneExtraction` builds, focused CTest, direct and pixi
software-GL screenshot analyzer checks, image-sequence analyzer checks,
aggregate `build-examples`, Python C++ example runner tests, and direct visual
inspection of the KR5/ground/red-target capture. Mandatory `pixi run lint`,
post-lint focused rebuild/CTest, post-lint direct screenshot analyzer check,
and `git diff --check` also passed.

### Rigid Shapes Itemized Inventory

Historical source compared: `520993d7301^:examples/rigid_shapes`.

Strict re-open found that the prior parity checkpoint restored many controls
but still drifted from the historical example by replacing the loaded
`shapes.skel` scene with a source-owned startup scene, omitting the historical
README, and dropping the console instruction printout. The current source
restores those items while keeping the promoted `dart::gui` keyboard actions,
panel buttons, and example-owned contact-marker frame.

| Historical item                                                                                                                                  | Current outcome                      | Notes                                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/shapes.skel` as the initial rigid-shape scene.                                                                          | Restored through public `dart::gui`. | Current source loads the historical SKEL world with `dart::io::readWorld`.                                                               |
| Preserve `--collision-detector`, `--max-contacts`, and `--ground-thickness`.                                                                     | Restored.                            | Current source keeps local option parsing, detector selection, max-contact assignment, and ground-thickness updates on the loaded world. |
| Print the selected collision detector.                                                                                                           | Restored.                            | Current source prints `Collision detector: ...` after applying detector selection.                                                       |
| Preserve contact-point toggling with `c`/`C`.                                                                                                    | Restored through public `dart::gui`. | Current source uses keyboard actions and an example-owned `PointCloudShape` instead of OSG `CustomWorldNode`.                            |
| Preserve `q`/`w`/`e`/`r` random shape spawning for box, ellipsoid, cylinder, and convex mesh.                                                    | Restored.                            | Current source restores random transform, color, size/radius/height, and convex-mesh vertex generation.                                  |
| Preserve `a`/`A` delete-last behavior.                                                                                                           | Restored with safer filtering.       | Current source removes the latest spawned `rigid_shape_` skeleton without deleting loaded scene skeletons.                               |
| Preserve printed instructions and 640x480 launch default.                                                                                        | Restored.                            | Current source prints the historical instruction block and uses `RunOptions` defaults.                                                   |
| Preserve camera home from eye `(2, 2, 2)` to target `(0, 0, 0)` with up `(0, 1, 0)`.                                                             | Restored through public `dart::gui`. | `OrbitCamera` restores the historical eye/target framing and up vector.                                                                  |
| Preserve README/run/build/execute documentation.                                                                                                 | Restored.                            | README documents promoted runner usage, capture, local options, and standalone build/execute sections.                                   |
| Keep marker coverage for SKEL load, CLI options, contact markers, key controls, random spawning, README, defaults, and no private scene handoff. | Restored.                            | Marker guard covers the restored contract and rejects private scene handoff.                                                             |

Validation for this slice includes focused `rigid_shapes` and
`UNIT_gui_FilamentSceneExtraction` builds, focused CTest, direct and pixi
software-GL screenshot analyzer checks, direct CLI-options screenshot smoke,
image-sequence analyzer checks, aggregate `build-examples`, Python C++ example
runner tests, and direct visual inspection of the loaded SKEL scene. Mandatory
`pixi run lint`, post-lint focused rebuild/CTest, post-lint direct screenshot
analyzer check, and `git diff --check` also passed.

### Soft Bodies Itemized Inventory

Historical source compared: `520993d7301^:examples/soft_bodies`.

Strict re-open found that the prior parity checkpoint restored the core
recorded-playback behavior but still missed the historical launch default,
console instructions, and README. The current source now restores those while
keeping the promoted `dart::gui` keyboard actions and panel playback controls.

| Historical item                                                                                                       | Current outcome                      | Notes                                                                                                                         |
| --------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/softBodies.skel`.                                                                            | Restored through public `dart::gui`. | Current source uses `dart::io::readWorld` and passes the world through `ApplicationOptions::world`.                           |
| Record world state history and truncate future history when simulating from an earlier frame.                         | Restored.                            | Current `SoftBodyHistory` captures skeleton configurations and body composite states before each promoted simulation step.    |
| Preserve `[`/`]` one-frame playback and `{`/`}` ten-frame playback controls.                                          | Restored through public `dart::gui`. | Current source uses promoted `KeyboardAction`s; shifted `{`/`}` support depends on the promoted GLFW key/shift input mapping. |
| Preserve `r` restart and `\` jump-to-end controls.                                                                    | Restored through public `dart::gui`. | Current source uses promoted keyboard actions and panel buttons.                                                              |
| Print viewer instructions and recorded-playback control text.                                                         | Restored.                            | Current source prints the promoted console instruction block before launching.                                                |
| Preserve 640x480 launch default.                                                                                      | Restored.                            | Current source sets `ApplicationOptions::runDefaults` to the historical dimensions.                                           |
| Preserve README/run/build/execute documentation.                                                                      | Restored.                            | Current README documents the promoted runner, capture flags, and standalone build/execute paths.                              |
| Keep marker coverage for SKEL load, history, keys, shifted shortcuts, defaults, README, and no private scene handoff. | Restored.                            | Marker guards cover the restored source and README behavior.                                                                  |

### Tinkertoy Itemized Inventory

Historical source compared: `520993d7301^:examples/tinkertoy`.

Strict re-open found that the prior builder/keyboard/camera checkpoints restored
the core block-building behavior but still missed user-facing panel/README
parity and exact pick-point force behavior. The current source repairs those
through public `dart::gui` by exposing selected hit point/normal in
`PanelContext`, restoring the historical panel shape/help/menu labels, and
adding README/default markers.

| Historical item                                                                                                      | Current outcome                      | Notes                                                                                                                                                                       |
| -------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Start with two prebuilt tinkertoy assemblies using weld, revolute, and ball joint blocks with historical dimensions. | Restored through public `dart::gui`. | Current source builds the assemblies directly with DART dynamics shapes, collision, and dynamics aspects.                                                                   |
| Select a block with left click and place the target at the clicked point plus surface-normal offset.                 | Restored through public `dart::gui`. | Public `PanelContext` now carries selected hit point/normal, and Tinkertoy uses that data before falling back to body COM.                                                  |
| Drag/move the target and apply clamped force from picked point to target while simulating.                           | Restored.                            | Current public `dart::gui::Gizmo` target, force line, and `preStep` force application use the restored picked point.                                                        |
| Keys `1`/`2`/`3`, Backspace, Delete, Up/Down, GraveAccent, and Tab.                                                  | Restored through public `dart::gui`. | Current `KeyboardAction`s cover block add, pick clear/delete, force coefficient, target orientation, and camera reset.                                                      |
| Enter toggles runtime recording to `dataPath("screencap")`.                                                          | Public API gap / superseded for now. | The promoted runner supports `--screenshot` and `--out`; runtime image/video recording remains a future public capture API.                                                 |
| ImGui panel title, position `(10,20)`, size `(360,640)`, alpha `0.5`, menu bar, no resize, and horizontal scrollbar. | Restored through public `dart::gui`. | Current `Panel` sets the historical renderer-neutral window options.                                                                                                        |
| Panel Menu/Exit and Help/About DART entries.                                                                         | Restored except exact modal gap.     | `requestExit` is public; About DART is represented as panel text until a public about modal exists.                                                                         |
| Panel help text, simulation controls, gravity toggle, force slider, target buttons, add/delete buttons.              | Restored.                            | Current panel restores historical labels/layout through `PanelBuilder` and documents public target gizmo dragging.                                                          |
| Headlights On/Off checkbox.                                                                                          | Restored through public `dart::gui`. | `PanelContext::lighting.headlightsEnabled` exposes renderer-neutral scene-light state without backend hooks.                                                                |
| Default 1280x720 launch size and shared `--gui-scale`.                                                               | Restored.                            | Current source sets explicit `runDefaults`; the shared runner parses `--gui-scale`.                                                                                         |
| README documents summary, controls, in-tree run, build, and execute instructions.                                    | Restored.                            | Current README documents promoted run/capture commands plus standalone build/execute sections and the capture/recording gap.                                                |
| Keep marker coverage for controls, selected hit data, panel options/labels, README, defaults, and no backend types.  | Restored.                            | Marker guards cover the public target gizmo, selection-hit data, panel options, labels, README, defaults, runtime recording gap, and absence of private scene/defaultScene. |

### Vehicle Itemized Inventory

Historical source compared: `520993d7301^:examples/vehicle`.

Strict re-open found that the prior command-key checkpoint restored the vehicle
world/controller behavior, keyboard actions, camera, and panel controls, but it
still missed the historical 640x480 launch default, printed instruction block,
README, and markers for those restored surfaces. This slice restores those
through public `dart::gui` run defaults and source-owned documentation while
keeping the promoted panel buttons/sliders as additive affordances.

| Historical item                                                                                                     | Current outcome                      | Notes                                                                                                                  |
| ------------------------------------------------------------------------------------------------------------------- | ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------- |
| Load `dart://sample/skel/vehicle.skel`, set Y-down gravity, and simulate the loaded car world.                      | Restored through public `dart::gui`. | Current source uses `dart::io::readWorld`, preserves the loaded skeleton layout, and passes the world via `options`.   |
| Apply the historical PD steering and wheel-velocity controller in the simulation pre-step.                          | Restored.                            | Current `VehicleController::preStep` writes the same wheel and steering force indices as the OSG `VehicleWorld`.       |
| Preserve `w`/`s`/`x` throttle and `a`/`d` steering commands, including upper-case letter key events.                | Restored through public `dart::gui`. | Current keyboard actions use `KeyboardShortcut::characterKey`; the input bridge maps letter shortcuts to GLFW keys.    |
| Print viewer instructions with vehicle command text and simulation toggle text.                                     | Restored in this slice.              | Current source prints the promoted console instruction block before launching.                                         |
| Preserve 640x480 launch default while keeping command-line overrides and capture options.                           | Restored in this slice.              | Current source sets `ApplicationOptions::runDefaults`; shared parsing still honors `--width`, `--height`, and `--out`. |
| Preserve camera home from eye `(5, 3, 3)` to target `(0, 0, 0)` with Y-up framing.                                  | Restored through public `dart::gui`. | Current `OrbitCamera` matches the historical eye/target framing with the promoted camera model.                        |
| README documents summary, controls, promoted run/capture, build, and execute instructions.                          | Restored in this slice.              | README now documents `pixi run ex vehicle`, `--screenshot`, `--out`, and standalone build/execute sections.            |
| Keep marker coverage for SKEL load, controller, keys, printed instructions, defaults, README, and no private scene. | Restored in this slice.              | Marker guards cover the restored source and README behavior and reject private scene handoff.                          |

### WAM IKFast Itemized Inventory

Historical source compared:
`520993d7301^:examples/wam_ikfast/{osgWamIkFast.cpp,helpers.*,input_handler.*,wam_world.*,ikfast/}`.

Strict re-open found that the current promoted source preserves WAM URDF
loading and a visible target handle, but it dropped the historical IKFast
solver setup, target activation keyboard workflow, reset/print keys, console
instructions, README, 1280x960 launch default, camera home, and the local
IKFast shared-library target. This slice restores the feasible behavior through
public `dart::gui` keyboard actions, pre-step IK solving, run defaults, camera,
README, and marker guards. Exact OSG drag-mode semantics remain partially
superseded by the promoted selection tool: public `dart::gui` supports
selection, Ctrl-left drag translation, Ctrl+Shift rotation, keyboard nudges,
and axis constraints, but not the old Alt/Ctrl/Shift parent-joint drag mode
split.

| Historical item                                                                                             | Current outcome                      | Notes                                                                                                                                        |
| ----------------------------------------------------------------------------------------------------------- | ------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------- |
| Load WAM from `urdf/wam/wam.urdf` with the `herb_description` package path and zero joint positions.        | Restored through public `dart::gui`. | Current source keeps DART URI loading and validates `/j1` through `/j7`.                                                                     |
| Create a ground plane and kinematic WAM world with physical simulation disabled.                            | Partially restored.                  | Current source uses zero gravity and kinematic-style pre-step IK; exact `allowSimulation(false)` remains a public lifecycle/default gap.     |
| Create end-effector `ee`, target frame `lh_target`, and configure `SharedLibraryIkFast` with WAM DOFs.      | Restored in this slice.              | The local `wamIk` shared library target is restored and the example configures `SharedLibraryIkFast` through public DART dynamics APIs.      |
| Run IK solving every refresh/step while an IK target is active.                                             | Restored through public `dart::gui`. | Current source solves active targets from `ApplicationOptions::preStep` and after keyboard activation/reset.                                 |
| Key `1` toggles target activation, `P` prints joint values, and `T` resets the relaxed posture.             | Restored in this slice.              | Current source uses promoted `KeyboardAction`s and pauses after discrete target/reset commands.                                              |
| Alt/Ctrl/Shift drag-mode instructions for translation, rotation, and parent-joint-only manipulation.        | Superseded / public API gap.         | Promoted selection supports target selection, Ctrl-left translation, Ctrl+Shift rotation, nudges, and axis constraints, not the exact modes. |
| Print viewer instructions and kinematic-mode note.                                                          | Restored in this slice.              | Current source prints the promoted console instruction block before launching.                                                               |
| Preserve 1280x960 launch default and camera home from eye `(5.34,3.00,1.91)` to target `(0,0,0.50)`.        | Restored in this slice.              | Current source sets `ApplicationOptions::runDefaults` and promoted `OrbitCamera` framing.                                                    |
| README documents summary, controls, promoted run/capture, build, and execute instructions.                  | Restored in this slice.              | README documents `pixi run ex wam_ikfast`, `--screenshot`, `--out`, and the drag-mode API gap.                                               |
| Keep marker coverage for IKFast setup, activation/reset/print keys, defaults, README, and no private scene. | Restored in this slice.              | Marker guards cover restored source/README behavior and reject private scene handoff.                                                        |

## Example Inventory

| Example                     | Current Audit State                                                                                                                                  | Next Required Action                                                                                                                                                                                                                                                                                                                  |
| --------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `add_delete_skels`          | Restored by strict re-open.                                                                                                                          | Keep marker guards for controls, no startup cubes, camera/defaults, README, and no backend types.                                                                                                                                                                                                                                     |
| `atlas_puppet`              | Restored.                                                                                                                                            | Keep marker guards for active target toggles, support controls, R balance controls, support polygon/centroid/COM overlays, camera/defaults, README, and no backend types.                                                                                                                                                             |
| `atlas_simbicon`            | Restored except depth-output API gap.                                                                                                                | Keep marker guards for controller/state files, pre-step control, perturbation and stride controls, native window title, shadow/headlight controls, camera/defaults, README, and no OSG types.                                                                                                                                         |
| `biped_stand`               | Restored by strict re-open.                                                                                                                          | Keep marker guards for controller, perturbation controls, camera/run defaults, historical instruction text, README, and no backend types.                                                                                                                                                                                             |
| `box_stacking`              | Restored by strict re-open.                                                                                                                          | Keep marker guards for stack placement, floor geometry, solver/gravity/headlight controls, keydown/key-release callbacks, panel text, camera readout/defaults, README, and no backend types.                                                                                                                                          |
| `boxes`                     | Restored by strict re-open.                                                                                                                          | Keep marker guards for Bullet preference, grid, names, ground color, instructions, camera/defaults, README, and no backend types; keep OSG shadow-map setup superseded by Filament.                                                                                                                                                   |
| `capsule_ground_contact`    | Restored by strict audit.                                                                                                                            | Keep marker guards for controls, camera/defaults, README, and no backend types.                                                                                                                                                                                                                                                       |
| `coupler_constraint`        | Restored by strict audit.                                                                                                                            | Keep marker guards for controller, reset key, diagnostics, grid, camera/defaults, README, and no backend types.                                                                                                                                                                                                                       |
| `csv_logger`                | Preserved by strict audit as a non-GUI example.                                                                                                      | Keep marker guards for CLI flags, CSV header/row count, README, and no GUI renderer dependency.                                                                                                                                                                                                                                       |
| `drag_and_drop`             | Restored with public `dart::gui::Gizmo` frame affordances.                                                                                           | Keep marker guards for public gizmo handle, child box, markers, camera/defaults, README, and no backend types.                                                                                                                                                                                                                        |
| `empty`                     | Restored by strict re-open.                                                                                                                          | Keep marker guards for frame scaffold, lifecycle/render callbacks, keyboard actions, camera/defaults, README, and no backend types.                                                                                                                                                                                                   |
| `fetch`                     | Restored by strict re-open after WAM IKFast; R24 adds a public `dart::gui::Gizmo` for target manipulation while preserving the green-bar target cue. | Keep marker guards for target gizmo manipulation, green bars, panel/help behavior, camera/defaults, README, `--screenshot`, `--out`, and no backend/private-scene handoff.                                                                                                                                                            |
| `free_joint_cases`          | Restored by strict audit.                                                                                                                            | Keep marker guards for numeric/reference controls, local CLI flags, camera/defaults, README, and no backend types.                                                                                                                                                                                                                    |
| `g1_puppet`                 | Restored by strict re-open except per-body drag API gap.                                                                                             | Keep marker guards for package loading, gravity, ground, root pose, grid, support overlay, camera/defaults, README, target toggles, and no backend types.                                                                                                                                                                             |
| `gui_scene_diagnostics`     | Preserved by strict audit as a descriptor diagnostic example.                                                                                        | Keep marker guards for CLI flags, descriptor extraction, debug/selection lines, pick output, README, and no backend types.                                                                                                                                                                                                            |
| `hardcoded_design`          | Restored by strict re-open.                                                                                                                          | Keep number-key joint controls, direction toggle, source-owned wireframe link edges, camera/defaults, README/help text, and marker guards.                                                                                                                                                                                            |
| `headless_simulation`       | Preserved by strict audit as a non-GUI example.                                                                                                      | Keep marker guards for CLI flags, seed setup, progress output, README, and no GUI renderer dependency.                                                                                                                                                                                                                                |
| `heightmap`                 | Restored by strict re-open.                                                                                                                          | Keep marker guards for source-owned grid style controls, terrain color editing through `PanelBuilder::colorEdit`, demo modes, camera/defaults, README, and no backend types.                                                                                                                                                          |
| `hello_world`               | Restored by strict audit.                                                                                                                            | Keep marker guards for instructions, camera/defaults, profiling, README, and no backend types.                                                                                                                                                                                                                                        |
| `hubo_puppet`               | Restored by strict re-open.                                                                                                                          | Keep marker guards for load/start pose, analytical IK, Shift-amplified movement, target toggles, R balance controls, support actions, support polygon/centroid/COM overlays, camera/defaults, README, and no backend types.                                                                                                           |
| `human_joint_limits`        | Restored except TinyDNN/custom-constraint dependency follow-up.                                                                                      | Keep marker guards for live world, joint-limit enforcement, defaults, README, and no backend types.                                                                                                                                                                                                                                   |
| `hybrid_dynamics`           | Restored by strict re-open.                                                                                                                          | Keep marker guards for README, 640x480 defaults, loaded-scene names/colors, harness console messages, camera up vector, and no backend types.                                                                                                                                                                                         |
| `imgui`                     | Restored by strict re-open.                                                                                                                          | Keep marker guards for target frame, lifecycle/render callbacks, keydown/key-release callbacks, gravity/headlight controls, camera readout/defaults, help text, README, and no backend types.                                                                                                                                         |
| `joint_constraints`         | Restored by strict re-open.                                                                                                                          | Keep marker guards for README, 640x480 defaults, loaded-scene visuals, perturb/harness console messages, camera up vector, and no backend types.                                                                                                                                                                                      |
| `lcp_physics`               | Restored by strict re-open except exact plot/backend-debug API gap.                                                                                  | Keep marker guards for historical scene counts/names/placements, live solver/scenario/reset/timestep/gravity panel controls, FPS/step-time diagnostics, README options, capture flags, and no backend types.                                                                                                                          |
| `mimic_pendulums`           | Restored by strict re-open.                                                                                                                          | Keep marker guards for live controls, source-owned grid, original rig names/base colors, public table/color-swatch diagnostics, README options, capture flags, and no backend/private-scene handoff.                                                                                                                                  |
| `mixed_chain`               | Restored by strict re-open.                                                                                                                          | Keep marker guards for random startup pose, impulse controls, force duration/magnitude, 640x480 defaults, console instructions, README options, capture flags, and no private-scene handoff.                                                                                                                                          |
| `operational_space_control` | Restored by strict re-open.                                                                                                                          | Keep marker guards for KR5 robot/ground, target ball, 1/2/3 axis aliases, shadow toggle, defaults, camera, README, instructions, and no WAM/private-scene drift.                                                                                                                                                                      |
| `point_cloud`               | Restored by strict re-open.                                                                                                                          | Keep marker guards for source-owned grid style controls, point-cloud and voxel-grid color editors through `PanelBuilder::colorEdit`, live sampling, camera/defaults, README, and no backend types.                                                                                                                                    |
| `polyhedron_visual`         | Restored by strict audit.                                                                                                                            | Keep marker guards for surface, wireframe, grid, camera/defaults, and README.                                                                                                                                                                                                                                                         |
| `rerun`                     | Removed after maintainer re-open and support audit.                                                                                                  | Keep removal guards for absent directory, absent CMake subdirectory, and absent README entry. Remaining references are CI retry docs, generated command guards, marker tests, and historical dev-task notes; future Rerun integration should return only as a real sourced example with a concrete use case or known downstream user. |
| `rigid_chain`               | Restored by strict audit.                                                                                                                            | Keep marker guards for random pose, damping, camera/defaults, README, and no backend types.                                                                                                                                                                                                                                           |
| `rigid_cubes`               | Restored by strict audit.                                                                                                                            | Keep marker guards for `cubes.skel`, force keys/decay, camera/defaults, instructions, README, and no backend types.                                                                                                                                                                                                                   |
| `rigid_loop`                | Restored by strict audit.                                                                                                                            | Keep marker guards for constrained-link colors, instructions, defaults, README, and no backend types.                                                                                                                                                                                                                                 |
| `rigid_shapes`              | Restored by strict re-open.                                                                                                                          | Keep marker guards for SKEL load, spawn/delete/contact controls, CLI options, random spawning, console instructions, camera/defaults, README, and no backend types.                                                                                                                                                                   |
| `simple_frames`             | Restored by strict re-open.                                                                                                                          | Keep marker guards for historical names, ArrowShape, camera/defaults, README, and no backend types.                                                                                                                                                                                                                                   |
| `simulation_event_handler`  | Restored by strict re-open.                                                                                                                          | Keep source-marker guards for controls, arrows, camera/defaults, shifted-slash help alias, README, and no backend types.                                                                                                                                                                                                              |
| `soft_bodies`               | Restored by strict re-open.                                                                                                                          | Keep marker guards for SKEL loading, recorded playback, bracket/brace/reset/latest controls, console instructions, 640x480 defaults, README, capture flags, and no private scene handoff.                                                                                                                                             |
| `speed_test`                | Preserved by strict audit as a non-GUI benchmark example.                                                                                            | Keep marker guards for scene list, dynamics/kinematics loops, summaries, README, and no GUI renderer dependency.                                                                                                                                                                                                                      |
| `tinkertoy`                 | Restored by strict re-open except runtime recording API gap.                                                                                         | Keep marker guards for selected hit point/normal, panel options/help/buttons, headlight checkbox, explicit defaults, README, capture flags, and no private scene handoff; keep runtime Enter recording as a public API gap.                                                                                                           |
| `unified_loading`           | Preserved by strict audit as a non-GUI loading example.                                                                                              | Keep marker guards for `ReadOptions`, load toggles, format/root-joint/package controls, README, and no GUI renderer dependency.                                                                                                                                                                                                       |
| `vehicle`                   | Restored by strict re-open.                                                                                                                          | Keep marker guards for SKEL load, controller, command keys, printed instructions, 640x480 defaults, README/capture docs, and no private scene handoff.                                                                                                                                                                                |
| `wam_ikfast`                | Restored by strict re-open except exact drag-mode/lifecycle API gaps.                                                                                | Keep marker guards for WAM URDF loading, `SharedLibraryIkFast`, `lh_target`, activation/reset/print keys, defaults, camera, README, capture, and no private scene handoff; keep parent-joint drag-mode and explicit no-simulation lifecycle as public API follow-ups.                                                                 |
