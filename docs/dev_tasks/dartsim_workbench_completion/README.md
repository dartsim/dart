# Dartsim Workbench Completion - Dev Task

## Current Status

- [x] Phase 0: Start the tracked feature-parity plan from the post-MVP
      `dartsim` GUI.
- [ ] Phase 1: Complete project lifecycle and edit-history foundations.
- [ ] Phase 2: Add viewport picking, selection sync, transform gizmos, and a
      real outliner. Selection, visibility, keyboard movement, and transform
      gizmo seams are in progress, and the first expand/rename/context action
      outliner primitives are in place.
- [ ] Phase 3: Add the model-building palette, relationship tools, and
      inspector coverage for authored scene data. A first tested Create menu
      palette seam now covers primitives, multibodies, links, and frames; the
      Inspector now has a tested property/action seam for transform, mass, joint,
      shape-dimension, color, and delete edits; and relationship actions now
      attach/detach frames while preserving world transforms.
- [ ] Phase 4: Add simulation workbench panels for watch values, sensor views,
      charts, and console automation.
- [ ] Phase 5: Add viewport controls, visibility filters, camera presets,
      workflows, and multi-view layouts.
- [ ] Phase 6: Raise focused `dartsim` engine/UI coverage toward 95%+ and run
      specialized review passes before the completion PR.

## Goal

Turn the `dartsim` MVP into a complete robotics simulation workbench: users can
author a scene, inspect and edit object properties, attach model elements, run
and reset simulations, view signals/sensors, automate common operations, and
persist their workspace through a tested, backend-hidden GUI.

## Non-Goals For Early Phases

- Do not expose `dartsim` engine internals as a public downstream C++ API.
- Do not make renderer backend types part of the editor model.
- Do not add network/device-controller runtime integration until the local
  editor, scene model, and test surfaces can support it cleanly.
- Do not chase coverage with brittle OpenGL pixel assertions where headless
  engine and UI-state tests can prove the behavior.

## Feature Map

| Area                     | Target Capability                                                                                                                                    | Current State                                                                                                                                                                                                                                                                   | Plan                                                                                                          |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| Viewport selection       | Pick objects from the rendered scene, keep viewport/tree/inspector selection in sync, highlight selected objects, and apply undoable transform edits | Rendering provider exposes object ids; outliner and viewport selection now share `dartsim_ui/outliner_actions`; `dart::gui` has a renderer-neutral renderable selection bridge; viewport keyboard movement and transform gizmo commits go through `dartsim_ui/viewport_actions` | Add richer viewport state such as camera presets and fit/focus controls after outliner basics are stronger    |
| Scene outliner           | Expandable hierarchy, object-type icons, inline rename, context actions, refresh, visibility toggles, and bidirectional selection sync               | Scene Tree now renders from `dartsim_ui/outliner_actions` rows with persistent expand/collapse state, inline rename over a backend-hidden text input, selected-row context actions, selection, and visibility actions                                                           | Replace selected-row action buttons with a popup/menu affordance once the panel API exposes context menus     |
| Project lifecycle        | New/open/save/save-as, current path, dirty marker, recent files, close/load prompts, progress/status events                                          | Phase 1 now has headless dirty/path/new/save/load state, blocks dirty replacement by default, and routes Open/Save As through native file pickers with an in-app path modal fallback behind an injectable `ProjectFileDialog` seam                                              | Add recent files, close/load confirmation prompts, and visible dirty-state affordances                        |
| Edit history             | Named undo/redo history, transactions, dirty tracking from the last save point                                                                       | Snapshot undo/redo and macro commands exist                                                                                                                                                                                                                                     | Expose history labels/counts, track clean history index, and test redo preservation                           |
| Creation palette         | Links, joints, collision shapes, sensors, frames, primitive rigid bodies, and predefined examples                                                    | `dartsim_ui/palette_actions` now exposes a tested, context-sensitive Create menu model for box/sphere/cylinder/capsule/plane rigid bodies, multibodies, root and fixed/revolute/prismatic child links, free/fixed frames, and starter example scenes with single-step undo      | Add richer inspector metadata before broadening into sensors or collision authoring                           |
| Attach and relationships | Parent/child link attachment, joint relationship editing, safe detach/delete                                                                         | Frame attach/detach and link parent/root relationship actions now have undoable command support with invalid-pair and cycle rejection through a tested `dartsim_ui/relationship_actions` seam                                                                                   | Add richer joint relationship affordances and multi-selection behavior after the primary link workflows land  |
| Inspector                | Categorized property editor with defaults, read-only states, enum choices, and multi-selection behavior                                              | `dartsim_ui/inspector_actions` now builds a typed property model for the primary selection, exposes read-only Simulation Mode state, and routes transform/mass/joint/shape/color/delete, shape-type enum, joint-kind enum, and joint-axis edits through undoable commands       | Add multi-selection behavior and richer grouping once more object types land                                  |
| Scene explorer           | Tree refresh, rename, context actions, watch selection, selection sync                                                                               | Basic scene tree and rename exist                                                                                                                                                                                                                                               | Add context actions and selection-driven watch registration                                                   |
| Simulation control       | Clear Edit Mode vs Simulation Mode boundary, play/pause/step/reset within Simulation Mode, initial-state reset, simulation status                    | Play/pause/step/reset and record/replay exist; `dartsim_ui/simulation_actions` now exposes tested panel status/actions, including captured Edit Mode reset target and replay status                                                                                             | Add richer timeline controls and status affordances once the panel API supports denser controls               |
| Watch/chart panels       | Watch list values and plotted simulation signals                                                                                                     | Console log panel only                                                                                                                                                                                                                                                          | Add headless signal registry, watch list, and chart data buffer before rendering widgets                      |
| Sensor views             | Camera/range/contact-like sensor panes                                                                                                               | No sensor model in `dartsim` scene data                                                                                                                                                                                                                                         | Add sensor descriptors and simulated outputs only after scene model supports sensors                          |
| Viewport controls        | rotate/pan/zoom modes, camera lock, front/side/top/perspective, fit view, four-view layout                                                           | Rendering provider exists; generic GUI camera behavior is outside the `dartsim` engine                                                                                                                                                                                          | Add editor-owned viewport state and backend-hidden commands; verify through UI state tests and headless smoke |
| Visibility filters       | Show/hide links, joints, collisions, sensors, and frames                                                                                             | Per-object `visible` is undoable, wired into the outliner, and honored through hidden ancestors; no typed layer filters yet                                                                                                                                                     | Add layer visibility state and filter render items before broader UI wiring                                   |
| Console automation       | Text commands for create/delete/attach/select/save/load and debug messages                                                                           | Console displays log notes only                                                                                                                                                                                                                                                 | Add parsed command dispatcher over `SimEngine`; test parser and side effects                                  |

## Specialized Review Findings

- Source-feature inventory review prioritized viewport picking, selection sync,
  transform gizmos, and a real outliner as the largest post-MVP usability gaps.
- DART architecture review confirmed the current MVP already has a strong
  headless engine split, command snapshots, scene I/O, simulation controls, and
  record/replay, but warned that `SceneObject` will become brittle if future
  sensors/controllers/inspector metadata are added as more type-dependent raw
  fields.
- Test review identified the need for failure-path project tests,
  project-save/new events, dirty-state regression coverage through undo/redo,
  and a future `UNIT_dartsim_ui`-style target for testable editor view-model
  logic.
- The lifecycle menu path now has a first testable UI action seam
  (`UNIT_dartsim_ui_ProjectActions`), so future menu/palette behavior should
  follow that pattern instead of adding more anonymous `editor.cpp` logic.
  Open Project and Save Project As now invoke the nativefiledialog-extended
  picker as a child of the GUI window and fall back to an in-app project path
  modal when the platform dialog is unavailable or a selected path fails; the
  testable `project_actions` seam owns selected/canceled/failed dialog
  outcomes and load/save failure messages without launching desktop UI.
- The outliner path now has the same seam (`UNIT_dartsim_ui_OutlinerActions`):
  build rows from `SimEngine`, select existing rows only, toggle visibility
  through undoable commands, expand/collapse hierarchy state, perform inline
  rename through undoable commands, expose selected-row context actions, and
  seed transform-gizmo movement through `moveSelectedBy()`.
- Viewport picking and highlighting now use a backend-hidden
  `dart::gui::ApplicationOptions` renderable selection bridge. The editor maps
  renderable ids back to visible `dartsim` object ids through the outliner
  action seam, so viewport, tree, and inspector selection share one engine-owned
  source of truth.
- Simulation controls now use `dartsim_ui/simulation_actions` instead of direct
  panel calls into `SimulationController`. The UI has tested labels for Edit
  Mode vs Simulation Mode, playback state, reset target, recording state, replay
  seek, and real-time factor updates; Reset consumes the captured Edit Mode
  snapshot so later edit-mode changes are not reverted by a stale runtime
  snapshot.
- The mode boundary now has a tested action view-model with explicit
  "Enter Simulation Mode", "Resume Simulation", "Step Simulation", and "Return
  to Edit Mode" commands. The editor chooses "Edit Mode" for authoring and
  "Simulation Mode" for runtime playback, matching the existing engine state
  names while making the play-mode boundary visible in menus and the Simulation
  panel.
- The Create menu now uses `dartsim_ui/palette_actions` instead of anonymous
  `editor.cpp` dispatch. The seam is context-sensitive, rejects Simulation Mode
  edits, creates every primitive rigid-body shape with useful defaults, creates
  multibody links with explicit joint kinds, creates starter example scenes as
  single undoable macro transactions, and fixed-frame creation now requires a
  real parent frame so the experimental World no longer throws on root fixed
  frames.
- The Inspector now uses `dartsim_ui/inspector_actions` instead of direct
  `editor.cpp` model mutation. The seam builds a typed property model for the
  primary selection, exposes Simulation Mode read-only states, edits rigid-body
  and frame transforms, rigid-body mass, link joint position and joint axis,
  visual shape dimensions, shape color, shape type, and child-link joint kind
  through undoable commands, and handles delete via the same subtree-safe command
  path as the Scene Tree.
- Relationship edits now use `dartsim_ui/relationship_actions` instead of
  ad-hoc menu mutation. The seam attaches exactly one selected frame to the
  primary selected frame-like parent, detaches the primary selected frame back
  to the world root, preserves world transforms through undoable
  `attachFrame`/`detachFrame` commands, converts detached fixed frames to free
  frames because root fixed frames are not representable in the experimental
  World, reparents a selected child link to a primary parent link in the same
  multibody, and makes a child link a multibody root while rejecting cycles and
  cross-multibody parents.
- The first review pass found and fixed root issues before this slice moved on:
  dirty project replacement is blocked by default, parent visibility hides
  descendant render items, project/selection state events are explicit, and
  negative UI action paths have regression coverage.

## Coverage And Review Gates

- Every headless engine feature gets focused unit tests in
  `tests/unit/dartsim_engine/test_dartsim_engine.cpp` or a split target when the
  file becomes too large.
- UI wiring gets state-oriented tests where possible and headless smoke tests
  where rendering behavior matters. The next UI slice should extract a
  testable editor view-model/action layer instead of leaving menu dispatch
  trapped in `editor.cpp` anonymous functions.
- Feature completion requires `pixi run lint`, the focused `dartsim` test
  targets, and `pixi run coverage-report-dartsim`. The 95%+ target applies to
  the `dartsim/engine` and extracted testable `dartsim/ui` action/view-model
  files, not to unrelated legacy GUI/OpenGL paths.
- Coverage enforcement must become explicit before the completion PR:
  1. Record a baseline with `pixi run coverage-report-dartsim`.
  2. Use `coverage-dartsim.info`, which extracts `dartsim/engine/*`,
     `dartsim/ui/include/dartsim_ui/*_actions.hpp`, and
     `dartsim/ui/src/*_actions.cpp` from the broader `coverage.info` report.
  3. Record per-directory line/function coverage in this task before promotion
     to durable docs.
  4. Treat coverage below 95% on that filtered surface as incomplete unless a
     line is documented as intentionally untestable GUI/backend glue.
- On 2026-05-25, a first `pixi run coverage-report-dartsim` attempt completed
  the configured Debug test suite (`265/265`) but extracted zero `dartsim`
  records because `config-coverage` did not enable the experimental simulator
  targets. The coverage config now enables `DART_BUILD_SIMULATION_EXPERIMENTAL`
  for this filtered surface; rerun the task after the file-dialog slice.
- On 2026-05-25, a focused Debug lcov capture over
  `build/default/cpp/Debug/dartsim/{engine,ui}` after the palette slice reported
  89.4% line coverage (`1948` lines) and 95.0% function coverage (`258`
  functions) on the filtered `dartsim/engine` plus testable `dartsim/ui`
  action/view-model surface. This is still below the 95% line target; the largest
  remaining gaps are `scene_model.cpp`, `object_manager.cpp`, `scene_io.cpp`, and
  the negative paths in extracted UI action seams.
- On 2026-05-26, focused Debug lcov coverage over the same filtered surface
  reached 95.3% line coverage (`2325` lines) and 98.0% function coverage (`295`
  functions) after adding relationship, malformed scene/project, inspector,
  outliner, palette, viewport, simulation, command-history, event-bus, and
  selection-state regression coverage. The 95% line target is currently met.
- On 2026-05-26, after the explicit Edit Mode / Simulation Mode workflow slice,
  the same focused Debug lcov capture reported 95.0% line coverage (`2726`
  lines) and 97.9% function coverage (`332` functions). The filtered 95% line
  target is still met, but further feature slices should add tests alongside new
  action-surface code instead of spending this margin.
- Before the completion PR, run specialized review passes for architecture,
  UI workflow, tests/coverage, and backend-hidden renderer boundaries.

## Key Decisions

- Build headless editor state first, then wire panels. This keeps new behavior
  testable without requiring a live renderer.
- Prefer metadata-driven inspector and console command surfaces over bespoke UI
  branches for every object type.
- Treat project dirty state as an engine concern, not a menu-only UI flag.
- Keep the active task folder until the completion PR, then promote durable
  design decisions into onboarding docs and delete this folder in the same PR.

## Immediate Next Steps

1. Keep the filtered `dartsim/engine` + testable `dartsim/ui` line coverage
   above 95% while adding the next feature slices.
2. Continue viewport workflow controls with fit/focus, camera presets, and
   visibility layer filters after the outliner can represent state cleanly.
3. Continue Phase 3 with multi-selection behavior for authored objects and
   richer joint relationship affordances.
