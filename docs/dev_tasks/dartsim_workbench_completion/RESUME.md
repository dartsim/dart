# Resume: Dartsim Workbench Completion

## Last Session Summary

The post-MVP `dartsim` GUI work has been moved onto
`feature/dartsim-simulator-workbench`. A tracked dev task now owns the feature
map, coverage target, and review gates for completing the simulator workbench.
Phase 1 project lifecycle state has a first local implementation and focused
unit/UI-action coverage. Phase 2 now has an outliner action/view-model seam
with undoable hierarchical visibility actions, primary-selection movement
helpers, and a backend-hidden viewport selection/highlight bridge over public
`dart::gui` renderable ids. The Simulation panel now has a tested
`dartsim_ui/simulation_actions` seam for Edit Mode vs Simulation Mode status,
Play/Pause/Step/Reset actions, real-time factor updates, recording, and replay
seek. The outliner seam now tracks persistent expand/collapse state, inline
rename drafts, selected-row context actions, and undoable rename/delete/show/hide
behavior.

## Current Branch

`feature/dartsim-simulator-workbench` - uncommitted dev-task planning, Phase 1
project lifecycle changes, a testable UI project-action seam, and the first
Phase 2 outliner/visibility action seam. The first specialized review pass was
run locally, and its root-cause findings for dirty replacement, visibility
semantics, event contracts, and missing negative tests were addressed.
Open Project and Save Project As now invoke the nativefiledialog-extended
picker as a child of the GUI window and fall back to an in-app project path
modal when the platform dialog is unavailable or a selected path fails; tests
inject fake dialog responses through `ProjectFileDialog` so selected, canceled,
failed, dirty guard, and missing-file paths are covered without launching
desktop UI.
The Create menu is now routed through `dartsim_ui/palette_actions`: the tested
palette model covers primitive rigid bodies, multibodies, root/child links with
explicit joint kinds, free frames, and fixed frames attached to an existing
frame-like parent. It also provides tested starter examples for a ground-and-box
scene and a two-link arm, each created as a single undoable macro transaction. A
fixed-frame regression prevents root fixed-frame creation from throwing inside
the experimental World rebuild path.
The Inspector is now routed through `dartsim_ui/inspector_actions`: the tested
property model covers primary-selection metadata, Simulation Mode read-only
state, undoable transform/mass/joint/shape/color edits, shape-type and
joint-kind enum choices, joint-axis edits, and subtree-safe delete.
Relationship editing is now routed through `dartsim_ui/relationship_actions`:
frame attach/detach commands preserve world transforms, reject invalid pairs,
convert detached fixed frames to root free frames, reparent child links within
their owning multibody, reject link-parent cycles, and make child links into
root links.

## Immediate Next Step

Continue Phase 3 with multi-selection behavior for authored objects and richer
joint relationship affordances. Keep behavior in testable engine or UI
action/view-model helpers before wiring it into `editor.cpp`, and keep the
filtered coverage line total above 95%.

## Context That Would Be Lost

- Keep tracked docs and code free of external source-project references; use
  DART-owned feature names and behavior descriptions.
- The first implementation slice is headless and tested in
  `tests/unit/dartsim_engine/test_dartsim_engine.cpp`.
- UI lifecycle menu behavior is covered through `dartsim_ui/project_actions`
  and `UNIT_dartsim_ui_ProjectActions`.
- Scene Tree behavior is now covered through `dartsim_ui/outliner_actions` and
  `UNIT_dartsim_ui_OutlinerActions`, including expansion state, inline rename,
  context actions, visibility, selection, and viewport id mapping.
- Simulation panel behavior is now covered through
  `dartsim_ui/simulation_actions` and `UNIT_dartsim_ui_SimulationActions`;
  Reset consumes the captured Edit Mode snapshot so stale runtime snapshots do
  not later overwrite edit-mode changes.
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
  `editor.cpp` mutation.
- `pixi run coverage-report-dartsim` now extracts the filtered
  `dartsim/engine` plus testable `dartsim/ui` action/view-model coverage
  surface from the broader lcov report. A 2026-05-25 attempt ran 265/265 Debug
  tests but extracted zero `dartsim` records because the coverage config did
  not enable the experimental simulator targets; the config now forces
  `DART_BUILD_SIMULATION_EXPERIMENTAL=ON` for coverage.
- A focused Debug lcov capture after the relationship and coverage-hardening
  slices reported 95.3% line and 98.0% function coverage over the filtered
  `dartsim/engine` plus testable `dartsim/ui` action/view-model surface.
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
git checkout feature/dartsim-simulator-workbench
git status --short --branch
git log -3 --oneline
```

Then continue with the viewport/outliner slice and preserve the validation
pattern:

```bash
pixi run lint
cmake --build build/default/cpp/Release --target UNIT_dartsim_engine UNIT_dartsim_ui_ProjectActions UNIT_dartsim_ui_InspectorActions UNIT_dartsim_ui_OutlinerActions UNIT_dartsim_ui_PaletteActions UNIT_dartsim_ui_RelationshipActions UNIT_dartsim_ui_ViewportActions UNIT_dartsim_ui_SimulationActions UNIT_gui_FilamentSceneExtraction
ctest --test-dir build/default/cpp/Release -R '^(UNIT_gui_FilamentSceneExtraction|UNIT_dartsim_(engine|ui_(ProjectActions|InspectorActions|OutlinerActions|PaletteActions|RelationshipActions|ViewportActions|SimulationActions)))$' --output-on-failure -j 1
```
