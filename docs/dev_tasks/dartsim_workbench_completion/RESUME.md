# Resume: Dartsim Workbench Completion

## Last Session Summary

The post-MVP `dartsim` GUI work is continuing on a topic branch with the
tracked dev task owning the feature map, coverage target, and review gates.
Phase 1 project
lifecycle, Phase 2 outliner/viewport selection, and the first Phase 3
palette/inspector/relationship seams are implemented through tested headless UI
action/view-model helpers. The project-open path now has a native picker plus
in-app project browser fallback, and the Simulation panel now exposes explicit
Edit Mode vs Simulation Mode workflow actions.

## Current Branch

`feature/dartsim-editor-project-browser-layers` targets a PR into `main`. Open
Project now opens the in-app project path modal/browser first so it remains
usable even when the platform native picker backend is unavailable. Browse still
invokes nativefiledialog-extended as a child of the GUI window and reports
picker failures in-place; tests inject fake dialog responses through
`ProjectFileDialog` so selected, canceled, failed, dirty guard, missing-file,
invalid-file, and directory paths are covered without launching desktop UI.
The Create menu is now routed through `dartsim_ui/palette_actions`: the tested
palette model covers primitive rigid bodies, multibodies, root/child links with
explicit joint kinds, free frames, and fixed frames attached to an existing
frame-like parent. It also provides tested starter examples for a ground-and-box
scene and a two-link arm, each created as a single undoable macro transaction. A
fixed-frame regression prevents root fixed-frame creation from throwing inside
the experimental World rebuild path.
The Inspector is now routed through `dartsim_ui/inspector_actions`: the tested
property model covers primary-selection metadata, multi-selection summaries,
Simulation Mode read-only state, undoable transform/mass/joint/shape/color
edits, shape-type and joint-kind enum choices, joint-axis edits, single-object
subtree-safe delete, and multi-selection delete as one undoable root-set macro.
Relationship editing is now routed through `dartsim_ui/relationship_actions`:
frame attach/detach commands preserve world transforms, reject invalid pairs,
convert detached fixed frames to root free frames, reparent child links within
their owning multibody, reject link-parent cycles, and make child links into
root links. Valid relationship menu items name the selected child and primary
parent/root target before applying the edit.
Simulation workflow controls are now routed through a
`buildSimulationModeActions` view-model: the editor presents "Enter Simulation
Mode", "Resume Simulation", "Step Simulation", and "Return to Edit Mode" labels
with disabled reasons instead of ambiguous transport-only buttons.
Console automation is now routed through `dartsim_ui/console_actions`: the
Console panel has a command input, quoted-argument parser, help/status text,
and tested project, create, selection, visibility, rename/delete, simulation
mode, recording, and replay command dispatch over the existing action seams.
File > Open Project now opens the in-app browser/manual path modal first.
Browse still invokes the native file dialog from inside the modal, and native
dialog failures or invalid selected paths leave the user in the modal instead
of making Open unusable; extensionless paths such as `scene` resolve to
`scene.dartsim` when that file exists.
Viewport camera workflow controls are now routed through
`dartsim_ui/viewport_actions`: the View menu exposes Fit Scene,
Focus Selection, and perspective/front/back/left/right/top/bottom presets, and
applies the resulting public `dart::gui::OrbitCamera` through
`PanelContext::CameraViewState` without exposing renderer backend state.
Viewport layer filters now live in `ViewportLayerFilterState` and are view-only:
the render provider, selected-renderable bridge, picking, selected label, camera
fit/focus, transform gizmo, and keyboard movement all reject layer-hidden rigid
bodies, links, and frames without dirtying the project or adding undo history.
Viewport camera controls now have tested Orbit/Pan/Zoom mouse mode state and
selected-object tracking. The generic `dart::gui::ApplicationOptions` camera
seams remain renderer-neutral, while `dartsim_ui/viewport_actions` owns the
editor-specific action labels, disabled reasons, and selection-tracking target.
Viewport layout actions now have tested single-view/four-view state and active
pane selection. The View menu exposes the four-view layout, the editor converts
`ViewportLayoutState` into public `dart::gui::ViewportLayoutOptions`, and
`dart::gui` renders one or four Filament views from that backend-hidden layout
provider. The renderer keeps one shared pane geometry model for Filament
viewports, active-pane camera controls, cursor-pane picking/dragging,
active-pane debug labels, visible pane labels with active-pane status, and
tiny-framebuffer fallback to single pane.

## Immediate Next Step

Continue Phase 5 by adding richer per-pane interaction polish for the four-view
layout, or continue Phase 3 with richer relationship inspectors and object
grouping. Keep behavior in testable engine or UI action/view-model helpers
before wiring it into `editor.cpp`, and keep the filtered coverage line total
above 95%.

## Context That Would Be Lost

- Keep tracked docs and code free of external source-project references; use
  DART-owned feature names and behavior descriptions.
- The first implementation slice is headless and tested in
  `tests/unit/dartsim_engine/test_dartsim_engine.cpp`.
- UI lifecycle menu behavior is covered through `dartsim_ui/project_actions`
  and `UNIT_dartsim_ui_ProjectActions`.
- Scene Tree behavior is now covered through `dartsim_ui/outliner_actions` and
  `UNIT_dartsim_ui_OutlinerActions`, including expansion state, inline rename,
  context actions, visibility, replace-selection, multi-selection toggle, clear
  selection, and viewport id mapping.
- Simulation panel behavior is now covered through
  `dartsim_ui/simulation_actions` and `UNIT_dartsim_ui_SimulationActions`;
  Reset consumes the captured Edit Mode snapshot so stale runtime snapshots do
  not later overwrite edit-mode changes. The panel/menu use explicit mode action
  labels and disabled reasons from the same seam.
- Create menu behavior is now covered through `dartsim_ui/palette_actions` and
  `UNIT_dartsim_ui_PaletteActions`; fixed frames require an existing parent frame
  instead of being attached directly to the world frame, and preset examples are
  grouped as single undoable macro transactions.
- Inspector behavior is now covered through `dartsim_ui/inspector_actions` and
  `UNIT_dartsim_ui_InspectorActions`; shape edits use an undoable `setShape`
  command that sanitizes dimensions and color, shape/joint kind enums use the
  renderer-neutral `PanelBuilder::select`, and child-link joint axes use an
  undoable `setJointAxis` command.
- Relationship behavior is now covered through
  `dartsim_ui/relationship_actions` and
  `UNIT_dartsim_ui_RelationshipActions`; frame attach/detach and link
  reparent/root actions go through undoable engine commands instead of direct
  `editor.cpp` mutation, and valid two-object actions preview the affected
  object names in their labels.
- Console behavior is now covered through `dartsim_ui/console_actions` and
  `UNIT_dartsim_ui_ConsoleActions`; commands reuse project, palette, outliner,
  inspector, and simulation action seams, so command automation stays
  backend-hidden and testable.
- Project open remains usable without a working native picker: the File menu
  opens the in-app browser/manual path modal first. Browse reports native picker
  failures in-place, and `openProject()` accepts extensionless paths when the
  corresponding `.dartsim` file exists.
- Viewport camera workflow behavior is covered through
  `dartsim_ui/viewport_actions` and `UNIT_dartsim_ui_ViewportActions`; fit uses
  visible render-item bounds, focus uses selected visible renderables and
  selected frames, presets keep target/distance while setting canonical
  orientations, and the actions do not mutate project dirty state or undo
  history.
- Viewport layer filter behavior is covered through
  `UNIT_dartsim_ui_ViewportActions`; rigid bodies, links, frames, scene-hidden
  objects, hidden ancestors, unsupported object types, selection clearing,
  filtered viewport picks, camera fit/focus, gizmo sync/apply, and keyboard
  movement are all tested through the filtered action overloads.
- Viewport camera-control behavior is covered through
  `UNIT_dartsim_ui_ViewportActions` and `UNIT_gui_FilamentSceneExtraction`;
  Orbit/Pan/Zoom mouse mode state, selection-tracking enable/disable paths,
  hidden-selection rejection, generic camera input zoom dragging, and
  `ApplicationOptions` camera callbacks are tested without renderer backend
  leakage into `dartsim/ui`.
- Viewport layout behavior is covered through
  `UNIT_dartsim_ui_ViewportActions` and `UNIT_gui_FilamentSceneExtraction`;
  single/four-view mode state, active perspective/front/right/top panes, pane
  camera mapping, public action rejection for pane activation while single-view
  is active, renderer layout payload conversion, split-pane geometry,
  stable pane display names, active-pane label/status overlay plumbing,
  tiny-framebuffer fallback, GUI provider plumbing, and view-only dirty/undo
  invariants are tested.
- `pixi run coverage-report-dartsim` now extracts the filtered
  `dartsim/engine` plus testable `dartsim/ui` action/view-model coverage
  surface from the broader lcov report. A 2026-05-25 attempt ran 265/265 Debug
  tests but extracted zero `dartsim` records because the coverage config did
  not enable the experimental simulator targets; the config now forces
  `DART_BUILD_SIMULATION_EXPERIMENTAL=ON` for coverage.
- A focused Debug lcov capture after the relationship and coverage-hardening
  slices reported 95.3% line and 98.0% function coverage over the filtered
  `dartsim/engine` plus testable `dartsim/ui` action/view-model surface.
- A Debug lcov capture after the multi-selection and relationship-label slice
  reported 95.7% line and 100.0% function coverage over the same filtered
  surface.
- The source-feature review prioritized viewport picking/selection/gizmo work
  and a real outliner before deeper inspector/palette work.
- Specialized review agents should be run before completion and can be run
  earlier after each substantial slice.
- The viewport bridge is now split between
  `dart::gui::ApplicationOptions::{selectedRenderableProvider,onRenderableSelected}`
  and `dartsim_ui/outliner_actions` helpers that reject hidden or unknown
  renderables before selecting engine objects.

## How To Resume

```bash
git checkout feature/dartsim-editor-project-browser-layers
git fetch origin main
git merge --no-ff origin/main
git status --short --branch
git log -3 --oneline
```

Then continue with the viewport/outliner slice and preserve the validation
pattern:

```bash
pixi run lint
cmake --build build/default/cpp/Release --target UNIT_dartsim_engine UNIT_dartsim_ui_ProjectActions UNIT_dartsim_ui_ConsoleActions UNIT_dartsim_ui_InspectorActions UNIT_dartsim_ui_OutlinerActions UNIT_dartsim_ui_PaletteActions UNIT_dartsim_ui_RelationshipActions UNIT_dartsim_ui_ViewportActions UNIT_dartsim_ui_SimulationActions UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release -R '^(UNIT_gui_FilamentSceneExtraction|UNIT_dartsim_(engine|ui_(ProjectActions|ConsoleActions|InspectorActions|OutlinerActions|PaletteActions|RelationshipActions|ViewportActions|SimulationActions)))$' --output-on-failure -j 1
```
