# Filament GUI - Live Steering Note (2026-05-14)

> Live supervisor note from a parallel evaluator pass. Read after `RESUME.md`.
> This is **strategic guidance**, not a task list. Owner: Codex.

## Current branch update

Since this steering note was first written, the post-MVP branch landed the
recommended maintainability slices:

- `dart/gui/experimental/debug.cpp` owns debug descriptor generation.
- `dart/gui/experimental/geometry.hpp` and `.cpp` own backend-hidden mesh
  builders for generated primitive and descriptor geometry.
- `examples/filament_gui/scenes.hpp` and `.cpp` own reusable DART scene
  fixtures and scene option parsing.
- `dart/gui/experimental/interaction.cpp` owns picking, plane-drag,
  frame-translation, and renderable set update planning helpers.
- `dart/gui/experimental/renderable.hpp`, `interaction.hpp`, and `debug.hpp`
  now split the constrained public surface while `scene.hpp` remains an
  aggregate compatibility include.
- `dart/gui/experimental/shape_descriptions.cpp` owns `describeShape`, leaving
  `scene.cpp` focused on renderable identity, resource versioning, and world
  extraction.
- `dart/gui/experimental/profile.hpp` and `.cpp` own viewer-loop profile
  accumulation, and `dart/gui/experimental/detail/filament/render_context.*`
  plus `screenshot.*` now own the first Filament-private runtime slice outside
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
- **Behavior preservation:** `git show 877fababbc6 -- scene.cpp` had zero
  added lines, only deletions. Pure mechanical move. Bodies in
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
`describeShape`, while `scene.cpp` owns renderable IDs, render-resource version
hashing, and `extractRenderables`.

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
2. **Completed: split scene fixtures into `examples/filament_gui/scenes.*`.**
   `examples/filament_gui/scenes.hpp` and `.cpp` now own scene option parsing
   plus MVP, drag-and-drop, and G1 DART world fixtures. `main.cpp` is focused
   on Filament renderer resources, synchronization, input, and the panel.
3. **Generalize the public-header leakage scan.**
   `08-north-star-migration.md` Phase A explicitly promises that the
   leakage check will be extended to promoted `dart::gui` headers
   "before first-class promotion." Refactor
   `tests/unit/gui/test_filament_scene_extraction.cpp:232-291` into a
   reusable `scan_headers_for_backend_tokens` helper, document the
   forbidden-token list in `03-milestones.md` Phase 5, and reserve a
   hook for `dart/gui/*.hpp`. Doc-heavy, small code, unblocks future
   promotion PRs.

## Doc hygiene notes

The docs-sync slice should keep these items current as commits land:

- `RESUME.md` should mention the concrete implementation split so resumed
  sessions do not treat `scene.cpp` as the only experimental implementation
  file.
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
  The example still owns Filament resource management, material caches,
  ImGui overlay, geometry builders, and scene fixtures. Defining the
  promoted API now would either bake in MVP-shaped concessions or
  force a second rewrite when Phase C extraction reveals the right
  resource boundaries.
- **Do not add panel/tool abstractions** (`08-north-star-migration.md`
  §11). Keep the ImGui overlay example-local until Phase E demands it.
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
The promoted `dart::gui` API shape will fall out naturally from
what's left in `main.cpp` when the lane bottoms out.
