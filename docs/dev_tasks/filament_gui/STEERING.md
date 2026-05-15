# Filament GUI - Live Steering Note (2026-05-14)

> Live supervisor note from a parallel evaluator pass. Read after `RESUME.md`.
> This is **strategic guidance**, not a task list. Owner: Codex.

## Current branch update

Since this steering note was first written, the post-MVP branch landed the
recommended maintainability slices:

- `dart/gui/experimental/debug.cpp` owns debug descriptor generation.
- `dart/gui/experimental/geometry.hpp` and `.cpp` own backend-hidden mesh
  builders for generated primitive and descriptor geometry.
- `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp` own reusable
  DART scene fixtures and scene option parsing.
- `dart/gui/experimental/interaction.cpp` owns picking, plane-drag,
  frame-translation, and renderable set update planning helpers.
- `dart/gui/experimental/renderable.hpp`, `interaction.hpp`, and `debug.hpp`
  now split the constrained public surface while `scene.hpp` remains an
  aggregate compatibility include.
- `dart/gui/experimental/shape_descriptions.cpp` owns `describeShape`, leaving
  `renderable.cpp` focused on renderable identity, resource versioning, and
  world extraction.
- `dart/gui/experimental/detail/filament/panel.hpp` and `.cpp` own the
  private built-in status panel rendering while promoted panel/tool API design
  remains deferred.
- `dart/gui/experimental/detail/filament/debug_overlay.hpp` and `.cpp` own
  debug-line overlay option defaults and refresh/cleanup for static, contact,
  and selection overlays, including selected-renderable descriptor lookup.
- `dart/gui/experimental/detail/filament/imgui_overlay.hpp` and `.cpp` own
  ImGui context setup, style/font initialization, current IO access, draw-data
  upload, and overlay teardown.
- `dart/gui/experimental/detail/filament/ui_frame.hpp` and `.cpp` own
  per-frame UI input sync, built-in status panel dispatch, panel-triggered
  debug-overlay refreshes, and ImGui overlay upload.
- `dart/gui/experimental/detail/filament/application_teardown.hpp` and `.cpp`
  own ordered teardown of the private Filament application resources.
- `dart/gui/experimental/detail/filament/frame_renderer.hpp` and `.cpp` own the
  private render-frame, skipped-frame, screenshot-request, and frame-accounting
  policy.
- `dart/gui/experimental/detail/filament/frame_viewport.hpp` and `.cpp` own
  private per-frame framebuffer sizing, ImGui display metrics,
  viewport/camera updates, and camera-controller suppression while selection
  or the built-in UI consumes the pointer.
- `dart/gui/experimental/detail/filament/screenshot.hpp` and `.cpp` own the
  private Filament screenshot readback, wait/save finalization, and screenshot
  profile accounting.
- `dart/gui/experimental/detail/filament/input.hpp` and `.cpp` own private
  GLFW/ImGui input bridging, orbit-camera scroll callback attachment,
  camera-controller input translation, and application hotkey polling/key-edge
  state.
- `dart/gui/experimental/detail/filament/selection.hpp` and `.cpp` own the
  private selection controller state and GLFW event translation for keyboard
  nudging, click selection, and Ctrl-left dragging.
- `dart/gui/experimental/detail/filament/native_window.hpp` and `.cpp` own the
  private GLFW initialization/window lifecycle, window-close loop predicate, and
  platform-native handle selection.
- `dart/gui/experimental/detail/filament/renderable_sync.hpp` and `.cpp` own
  private initial scene renderable creation, scene-entity synchronization,
  unsupported-renderable logging, and per-frame renderable
  transform/selection/shadow application.
- `dart/gui/experimental/detail/filament/materials/` owns the Filament material
  shader sources, and `dart/gui/experimental/detail/filament/testing/` owns the
  opt-in headless smoke CMake/Python helpers.
- `dart/gui/experimental/detail/filament/filament_sources.cmake` owns the
  private backend source/header list, dependency wiring, target setup, material
  inputs, material header generation, and smoke-test registration that the
  minimal example CMake file consumes.
- `dart/gui/experimental/detail/application.hpp` owns the generic private GUI
  application entry point consumed by `examples/filament_gui/main.cpp`; the
  backend-specific application entry point stays under
  `dart/gui/experimental/detail/filament`.
- `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`
  own created-renderable content counting for startup validation.
- `dart/gui/experimental/detail/filament/scene_startup.hpp` and `.cpp` own
  initial scene extraction, startup validation, first renderable
  synchronization, and startup debug-overlay validation.
- `dart/gui/experimental/detail/filament/scene_frame.hpp` and `.cpp` own
  per-frame simulation stepping, descriptor extraction, scene synchronization,
  selected-renderable interaction updates, selection-debug refresh, and
  orbiting-light updates.
- `dart/gui/experimental/detail/filament/simulation_stepper.hpp` and `.cpp` own
  the bounded realtime simulation step-count accumulator, world-step
  application, lifecycle marking, and simulation profile accounting.
- `dart/gui/experimental/detail/filament/application.cpp` now depends on the
  private GUI runtime surfaces directly instead of retaining stale DART scene
  fixture/import dependencies after the `scenes.*` split.
- `dart/gui/experimental/profile.hpp` and `.cpp` own viewer-loop profile
  accumulation, and `dart/gui/experimental/detail/filament/render_context.*`
  plus `screenshot.*`, `render_environment.*`, `textures.*`, and
  `renderable_*.*` now own the first Filament-private runtime slices outside
  the example tree.

The remaining guidance is still useful for promotion sequencing, but the
original "next" ordering below is partially complete.

## What just shipped (verified)

- Commit `877fababbc6` "Split experimental GUI viewer helpers" landed:
  `dart/gui/experimental/viewer.{hpp,cpp}` (228 + 432 lines) is now the
  formal home for `RunOptions`, `ViewerLifecycleState`, `OrbitCamera*`,
  `PerspectiveProjection`, `PickRay`, `writeRgbaPpm`, frame
  accounting, and pause/step helpers. `scene.hpp` remains an aggregate
  compatibility include for the constrained experimental surface.

### Independent verification results

- **Public-header purity (HARD gate):** PASS. `viewer.hpp` includes only
  Eigen + STL + `dart/gui/export.hpp`. Zero forbidden tokens
  (`filament::`, `GLFW`, `ImGui`, `osg::`, `Vulkan`, `Metal`, `Raylib`, etc.).
  `tests/unit/gui/test_filament_scene_extraction.cpp:232-250` uses a
  `directory_iterator` glob over `*.hpp`, so the new header is picked up
  automatically by `ExperimentalPublicHeadersStayBackendHidden`.
- **Build system:** `dart/gui/experimental/CMakeLists.txt` uses
  `file(GLOB hdrs "*.hpp")` and `file(GLOB srcs "*.cpp")`. No CMake edit
  required — focused experimental headers and sources are picked up
  automatically after reconfigure.
- **Behavior preservation:** `git show 877fababbc6 -- scene.cpp` showed the
  then-current file had zero added lines, only deletions. Pure mechanical move.
  Bodies in
  `viewer.cpp` byte-equivalent to deleted bodies.
- **API symmetry:** All callers (tests, dartpy bindings, example) reach
  the moved symbols transitively through `scene.hpp`. No downstream
  include updates required.

**Verdict:** the viewer split is safe to keep. No follow-up code edits needed.

## Completed debug split

`dart/gui/experimental/debug.cpp` now owns debug-line generation
(`makeGridDebugLines`, `makeFrameDebugLines`, `makeSelectionDebugLines`,
`makeInertiaDebugLines`, `makeCollisionShapeDebugLines`,
`makeSupportPolygonDebugLines`, `extractContactDebugLines`,
`extractDebugLines`, plus internal helpers). Its public surface remains in
`scene.hpp` for source compatibility.

## Completed scene interface split

`scene.hpp` is now only an aggregate compatibility include. Focused public
headers carry the constrained concepts: `renderable.hpp` for descriptors and
extraction planning, `interaction.hpp` for picking/dragging/translation, and
`debug.hpp` for debug-line descriptors. `shape_descriptions.cpp` owns
`describeShape`, while `renderable.cpp` owns renderable IDs, render-resource
version hashing, and `extractRenderables`.

## Original recommended priority order

1. **Completed: promote example-local geometry builders into
   `dart-gui-experimental`.**
   `dart/gui/experimental/geometry.hpp` and `.cpp` now expose backend-hidden
   mesh buffers and builders consumed by the Filament example. Graphics-free
   tests cover the generated geometry path.
   - Anchors: `08-north-star-migration.md` Phase C deliverables and exit
     criterion ("The Filament example is a thin consumer of the GUI
     library rather than the owner of renderer architecture");
     `07-completion-audit.md` row 34 "Provide renderer-hidden scene
     extraction" — currently Satisfied for the constrained API but the
     geometry path is the obvious next descriptor surface.
2. **Completed: move scene fixtures under private GUI detail.**
   `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp` now own scene
   option parsing plus MVP, hello-world, drag-and-drop, polyhedron, heightmap,
   and G1 DART world fixtures.
   `dart/gui/experimental/detail/filament/application.hpp` and `.cpp` now own
   Filament frame orchestration and the built-in panel, leaving `main.cpp` as a
   minimal entry point.
   `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`
   now own scene fixture requirement counting and MVP/G1/hello-world/drag/
   polyhedron/heightmap validation gates.
3. **Completed: generalize the public-header leakage scan.**
   `UNIT_gui_FilamentSceneExtraction` now routes the backend-token check
   through `scanHeadersForBackendTokens`, and
   `guiHeaderDirectoriesForBackendTokenScan` is the reserved hook for promoted
   `dart/gui/*.hpp` headers before first-class promotion.

## Doc hygiene notes

The docs-sync slice should keep these items current as commits land:

- `RESUME.md` should mention the concrete implementation split so resumed
  sessions do not treat `scene.hpp` as the only experimental implementation
  surface.
- `09-legacy-surface-audit.md` should describe the experimental headers in
  `dart/gui/experimental/` as the clean API fence, not only `scene.hpp`.
- `02-mvp-example.md` and `05-testing.md` should distinguish scene extraction,
  interaction helpers, and viewer-runtime helpers.
- `CHANGELOG.md` should mention that `scene.hpp` remains an aggregate
  compatibility include while implementation moves into focused `.cpp` files.
- `07-completion-audit.md` should keep external packaging and macOS/Windows
  platform gates framed as tracked promotion blockers, not branch-local work.

## Explicitly do NOT do next

- **Do not draft the promoted `dart::gui::Viewer` / `ViewerOptions`
  public API yet** (the sketch in `08-north-star-migration.md:331-344`).
  Phase H requires Phases C, E, F, G to be substantially complete first.
  The private implementation still owns Filament frame orchestration and
  built-in overlay policy. Defining the promoted API now would either bake in
  MVP-shaped concessions or force a second rewrite when Phase C extraction
  reveals the right resource boundaries.
- **Do not add panel/tool abstractions** (`08-north-star-migration.md`
  §11). Keep the built-in ImGui panel/tool policy MVP-scoped until Phase E
  demands it.
- **Do not start macOS/Windows port work** — no machines in this
  branch, not actionable for Codex.
- **Do not modify the conda-forge feedstock PR** — external dependency,
  Codex can only track via `gh pr checks 33297 --repo conda-forge/staged-recipes`.

## Stay-in-lane summary

The Phase C/D promotion lane (move the example from "owns the
renderer architecture" to "thin consumer of the GUI library") is
the highest-leverage path right now. Every commit in the recent
log fits that lane: GUI scale → projection → nudge → camera
controller → viewer split → debug-line split → geometry builders → scene
fixtures → interaction helpers. Stay in this lane.
The completion metric for this lane is stronger than direct include hygiene:
any surviving `examples/filament_gui/` directory should become a minimal
entry-point shell while renderer setup, frame lifecycle, resources,
synchronization, capture, overlays, input translation, and fixture logic live
under `dart::gui` or private GUI implementation units.
The promoted `dart::gui` API shape will fall out naturally from what remains
private implementation detail after the example entry point bottoms out.
