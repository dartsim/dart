# Dartsim Workbench Completion - Dev Task

## Current Status

- [x] Phase 0: Start the tracked feature-parity plan from the post-MVP
      `dartsim` GUI.
- [ ] Phase 1: Complete project lifecycle and edit-history foundations. Edit
      history now has tested labels, undo/redo counts, and clean-revision
      tracking for saved states and divergent redo branches; project lifecycle
      now has tested current-path/dirty labels plus session recent-project
      actions and dirty-replacement confirmation prompts.
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
      charts, and console automation. A first tested Console command seam now
      dispatches create/select/edit/project/simulation commands through the
      existing action layers.
- [ ] Phase 5: Add viewport controls, visibility filters, camera presets,
      workflows, and multi-view layouts. Tested View menu camera seams now cover
      fit scene, focus selection, canonical camera presets, Orbit/Pan/Zoom mouse
      modes, selection tracking, and view-only layer filters for rigid bodies,
      links, and frames. A renderer-neutral single/quad viewport layout seam now
      drives Filament pane views, and the View menu exposes the four-view
      layout through tested editor actions. Four-view panes now show stable
      labels plus an active-pane status affordance from the renderer-owned pane
      geometry, and clicking an inactive pane activates it while preserving the
      pane camera.
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

| Area                     | Target Capability                                                                                                                                    | Current State                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        | Plan                                                                                                           |
| ------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| Viewport selection       | Pick objects from the rendered scene, keep viewport/tree/inspector selection in sync, highlight selected objects, and apply undoable transform edits | Rendering provider exposes object ids; outliner and viewport selection now share `dartsim_ui/outliner_actions`; `dart::gui` has a renderer-neutral renderable selection bridge; viewport keyboard movement and transform gizmo commits go through `dartsim_ui/viewport_actions`                                                                                                                                                                                                                                                                                                                      | Add richer viewport state such as camera presets and fit/focus controls after outliner basics are stronger     |
| Scene outliner           | Expandable hierarchy, object-type icons, inline rename, context actions, refresh, visibility toggles, and bidirectional selection sync               | Scene Tree now renders from `dartsim_ui/outliner_actions` rows with persistent expand/collapse state, inline rename over a backend-hidden text input, selected-row context actions, selection toggles, selection clear, and visibility actions                                                                                                                                                                                                                                                                                                                                                       | Replace selected-row action buttons with a popup/menu affordance once the panel API exposes context menus      |
| Project lifecycle        | New/open/save/save-as, current path, dirty marker, recent files, close/load prompts, progress/status events                                          | Phase 1 now has headless dirty/path/new/save/load state, blocks dirty replacement by default, routes Open and Save As through an injectable native `ProjectFileDialog` seam with an in-app project browser fallback, exposes current project path/dirty labels through `dartsim_ui/project_actions`, tracks session recent projects for the File menu, and uses a tested confirmation request before New/Open/Open Recent can discard dirty edits                                                                                                                                                    | Add an explicit close-project action if the application lifecycle grows a close-workspace command              |
| Edit history             | Named undo/redo history, transactions, dirty tracking from the last save point                                                                       | Snapshot undo/redo and macro commands exist; the engine now exposes undo/redo counts, current and clean history revisions, clean history index, and a tested `dartsim_ui/history_actions` seam for Edit menu labels/actions that preserves redo state and locks edits in Simulation Mode                                                                                                                                                                                                                                                                                                             | Surface history status in a denser panel once the panel API supports compact history affordances               |
| Creation palette         | Links, joints, collision shapes, sensors, frames, primitive rigid bodies, and predefined examples                                                    | `dartsim_ui/palette_actions` now exposes a tested, context-sensitive Create menu model for box/sphere/cylinder/capsule/plane rigid bodies, multibodies, root and fixed/revolute/prismatic child links, free/fixed frames, and starter example scenes with single-step undo                                                                                                                                                                                                                                                                                                                           | Add richer inspector metadata before broadening into sensors or collision authoring                            |
| Attach and relationships | Parent/child link attachment, joint relationship editing, safe detach/delete                                                                         | Frame attach/detach and link parent/root relationship actions now have undoable command support with invalid-pair and cycle rejection through a tested `dartsim_ui/relationship_actions` seam; valid actions preview the object names they will modify                                                                                                                                                                                                                                                                                                                                               | Add richer joint relationship affordances after the primary link workflows land                                |
| Inspector                | Categorized property editor with defaults, read-only states, enum choices, and multi-selection behavior                                              | `dartsim_ui/inspector_actions` now builds a typed property model for the primary selection, exposes multi-selection summaries and Simulation Mode read-only states, routes primary-object property edits through undoable commands, and deletes a multi-selection as one macro                                                                                                                                                                                                                                                                                                                       | Add richer grouping once more object types land                                                                |
| Scene explorer           | Tree refresh, rename, context actions, watch selection, selection sync                                                                               | Basic scene tree and rename exist                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Add context actions and selection-driven watch registration                                                    |
| Simulation control       | Clear Edit Mode vs Simulation Mode boundary, play/pause/step/reset within Simulation Mode, initial-state reset, simulation status                    | Play/pause/step/reset and record/replay exist; `dartsim_ui/simulation_actions` now exposes tested panel status/actions, including captured Edit Mode reset target and replay status                                                                                                                                                                                                                                                                                                                                                                                                                  | Add richer timeline controls and status affordances once the panel API supports denser controls                |
| Watch/chart panels       | Watch list values and plotted simulation signals                                                                                                     | Console log panel only                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               | Add headless signal registry, watch list, and chart data buffer before rendering widgets                       |
| Sensor views             | Camera/range/contact-like sensor panes                                                                                                               | No sensor model in `dartsim` scene data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              | Add sensor descriptors and simulated outputs only after scene model supports sensors                           |
| Viewport controls        | rotate/pan/zoom modes, camera lock, front/side/top/perspective, fit view, four-view layout                                                           | View menu camera actions now compute fit scene, focus selection, perspective/front/back/left/right/top/bottom presets, Orbit/Pan/Zoom mouse modes, selected-object tracking, and single/four-view layout from `dartsim_ui/viewport_actions`, then apply them through renderer-neutral camera and layout seams. Filament renders the active one or four pane views from the public layout provider, with active-pane camera input, cursor-pane picking/dragging, active-pane debug labels, visible pane labels, active-pane status, and click-to-activate pane switching using the same pane geometry | Add richer per-pane interaction polish after the first four-view slice                                         |
| Visibility filters       | Show/hide links, joints, collisions, sensors, and frames                                                                                             | Per-object `visible` is undoable, wired into the outliner, and honored through hidden ancestors; view-only layer filters now hide/show rigid bodies, links, and frames without mutating scene visibility, dirty state, or undo history                                                                                                                                                                                                                                                                                                                                                               | Add collision/sensor/joint filters after those object/render layers exist                                      |
| Console automation       | Text commands for create/delete/select/save/load/simulation controls and debug messages                                                              | `dartsim_ui/console_actions` now tokenizes quoted commands and dispatches project lifecycle, Create menu, selection, visibility, rename/delete, Edit/Simulation mode, record, and replay commands through the same tested action seams used by the panels                                                                                                                                                                                                                                                                                                                                            | Add relationship-specific commands and richer watch/chart registration after the command vocabulary stabilizes |

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
  cross-multibody parents. Valid two-object actions now name both endpoints in
  the menu label, so the user can verify the selected child and primary parent
  before applying the edit.
- Multi-selection now uses engine-owned facade methods for select, deselect,
  toggle, and clear. The Scene Tree exposes explicit toggle and clear controls,
  the Inspector reports the selected count and primary object, and deleting a
  multi-selection removes only selected root objects in a single undoable macro
  so selected descendants are not double-deleted.
- Console automation now uses `dartsim_ui/console_actions` instead of direct
  `editor.cpp` mutation. The seam parses quoted arguments, reports stable
  usage errors, resolves object targets by id/name/selection, and routes
  project, create, select, visibility, rename/delete, simulation mode,
  recording, and replay commands through the same headless action helpers as
  the menu and panel UI.
- Project open now starts from the in-app project browser/manual path modal so
  the workflow remains usable even when the platform native picker backend is
  unavailable. Browse still invokes nativefiledialog-extended as a child of the
  GUI window, retries without the parent when that handle is rejected, and
  reports failures in-place; extensionless explicit paths resolve to `.dartsim`
  files when present.
- Project status now comes from `dartsim_ui/project_actions`: the Menu bar
  shows the current project name with an unsaved marker, the action seam exposes
  saved/dirty labels and the full path, and the File menu keeps a session-local
  recent-project list that de-duplicates paths, marks the current project, and
  respects the existing dirty-project guard when reopening a recent file.
- Dirty project replacement now has a tested confirmation request layer in
  `dartsim_ui/project_actions`: New Project, modal Open, and Open Recent defer
  destructive replacement until the user confirms, while clean replacements and
  macro/open-transaction rejection continue through the same action seam.
- Viewport camera workflow controls now use the same action-seam pattern:
  `dartsim_ui/viewport_actions` computes fit scene, focus selection, and
  canonical camera preset requests from visible render items and selected
  frames, while `dart::gui::PanelContext` exposes only a public
  renderer-neutral `OrbitCamera` setter. These actions are view-only and do not
  dirty the project or add undo history.
- Viewport layer filters live in `ViewportLayerFilterState` on the editor, not
  in the scene model. The render provider, viewport picking, selected-label
  bridge, camera fit/focus, transform gizmo, and keyboard movement all query
  filtered viewport visibility through `dartsim_ui/viewport_actions`; outliner
  visibility remains the undoable scene-authored visibility source of truth.
- Viewport camera controls now expose Orbit, Pan, and Zoom mouse modes through
  a renderer-neutral camera input seam, while selected-object tracking updates
  the orbit-camera target from visible selected objects without mutating scene
  state, dirty state, or undo history.
- Viewport layout actions now provide a tested single-view vs four-view state
  and active pane selection through `dartsim_ui/viewport_actions`. The editor
  converts that state into the public `dart::gui::ViewportLayoutOptions` seam,
  and `dart::gui` renders the requested pane cameras through multiple Filament
  views without exposing backend types to `dartsim/ui`.
- Four-view pane labels now come from the renderer-owned pane geometry instead
  of editor/backend state. The overlay labels Perspective/Top/Front/Right panes
  and highlights the active pane so the visible status matches the pane used for
  camera input, picking, dragging, and debug labels.
- Four-view pane activation also uses the renderer-owned pane geometry: clicking
  an inactive pane switches the editor-owned active pane, carries the clicked
  pane camera into the shared camera controller, and keeps the transition
  view-only so scene dirty state and undo history stay unchanged.
- Edit history now uses `dartsim_ui/history_actions` instead of direct
  `editor.cpp` undo/redo calls. The seam exposes named menu labels, undo/redo
  counts, dirty/clean labels, clean history index, and monotonic clean-history
  revisions so a save point can be distinguished from a divergent redo branch
  even when the undo cursor index matches.
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
- On 2026-05-26, after the multi-selection and relationship-label slice,
  `coverage-dartsim.info` reported 95.7% line coverage (`2748` of `2871` lines)
  and 100.0% function coverage (`361` of `361` functions) on the filtered
  surface.
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
2. Continue Phase 5 with richer per-pane interaction polish for the
   renderer-side four-view layout.
3. Continue Phase 3 with richer relationship inspectors and grouping once more
   authored object metadata lands.
