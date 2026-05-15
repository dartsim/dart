# Filament GUI - Live Steering Note (2026-05-14)

> Live supervisor note from a parallel evaluator pass. Read after `RESUME.md`.
> This is **strategic guidance**, not a task list. Owner: Codex.

## 2026-05-15 supervisor update (translation of live human direction)

Synthesis of a parallel architect / code-reviewer / verifier / critic pass over
this folder plus the current code state. Sticky — do not delete; mark items
`~~done~~` with a one-line evidence pointer when they land, and surface
disagreement under "Open Issues" instead of editing "Decisions in force".

### Status (verified at c9ccedfebe8)

- `examples/filament_gui/` is renamed to `examples/dartsim/`. The README
  documents `dartsim` as the application-level viewer with legacy launchers as
  sibling examples.
- `examples/dartsim/main.cpp` includes `<dart/gui/application.hpp>` and calls
  `dart::gui::runApplication(argc, argv)`. Public include path is therefore
  `dart/gui/application.hpp` (NOT `detail/application.hpp` as some older
  doc passages still claim).
- CMake target / `OUTPUT_NAME` is `dartsim`. Checkpoint `30b879458f8` renamed
  the old private `dart_filament_gui_add_example` helper to
  `dart_gui_filament_add_example`, keeping backend details out of user-facing
  example names.
- Linux headless CI was red because the workflow invoked the removed legacy
  `rigid_cubes` executable with the old `--out` contract. The local repair
  restores `rigid_cubes` as a `dart::gui` launcher and updates the workflow to
  validate `--screenshot` PPM output.
- The boundary-guard test `UNIT_gui_FilamentSceneExtraction`
  (`tests/unit/gui/test_filament_scene_extraction.cpp`) is retargeted locally
  to the promoted `examples/dartsim/` entry point and `dart/gui` public header
  surface.
- CI Lint, CI Linux, CI macOS, CI Windows, and CodeQL were manually dispatched
  for `d343c3c64bc` without opening a PR.
- `c9ccedfebe8` adds promoted `dart/gui/*.hpp` wrapper headers and explicit
  `dart::gui` aliases for renderer-independent scene, viewer, geometry,
  interaction, debug, and profiling concepts, while keeping
  `dart/gui/experimental/*` as compatibility shims for now.
- CI Lint on `d343c3c64bc` failed at CMake configure because
  `examples/simple_frames/CMakeLists.txt` was ignored by `.gitignore`'s
  `*_frames/` rule. The local repair force-adds that launcher before the next
  CI dispatch.

### Decisions in force (do NOT reopen)

1. **Naming is final**. Application-level binary is `dartsim`; example
   directory is `examples/dartsim/`; public include is
   `<dart/gui/application.hpp>`; entry symbol is `dart::gui::runApplication`.
   No `gui_viewer`, no `filament_gui`, no `dart_filament_gui` in user-facing
   strings. Private helper names were also moved away from the old
   `filament_gui` compound in `30b879458f8`; keep new private names aligned
   with `gui_filament`/`DART_GUI_FILAMENT` wording unless a real backend
   identifier is needed.
   Branding distinction: `DART` is the project and C++ library family identity,
   `libdart` is appropriate for package/library artifacts, and `dartsim` is the
   application-level simulator/viewer identity, analogous to Isaac Sim.
2. **Restoration semantics for legacy examples**: each historical user-facing
   example (`hello_world`, `rigid_cubes`, `drag_and_drop`, `imgui`,
   `simple_frames`, etc.) becomes its own `examples/<name>/` directory that
   builds a real executable through the shared `gui_scene_launcher.cpp` and
   supplies that example's default scene to
   `dart::gui::runApplication(argc, argv, defaultScene)`. Pixi `ex` aliases
   are not a substitute — CI invokes the binary on disk.
3. **`--scene <name>` fixtures inside `dartsim`** stay as the developer
   fixture menu and the source of truth for headless smoke generation. Per-
   example binaries become the user-facing surface; `dartsim --scene foo`
   remains the developer entry point. This is a permanent split, not interim
   debt — the `--scene` retirement plan called for in the critic pass is
   resolved by accepting this split.
4. **Public API surface promoted in this branch is exactly
   `dart::gui::runApplication` plus its default-scene selector overload.
   Nothing else.** Do NOT draft `dart::gui::Viewer` /
   `ViewerOptions` until Phases C/E/F/G in `08-north-star-migration.md` are
   substantially complete. This resolves the `STEERING.md` ↔
   `10-active-execution.md` contradiction by reading "promoted application
   header" narrowly.
5. **Out of scope for this branch (do not start, even opportunistically)**:
   ImGui Docking, dockable 3D widget, video capture, font atlas/label work,
   broader product/packaging work beyond the `dartsim` naming distinction,
   macOS/Windows port work, conda-forge feedstock changes. Track in a follow-up
   task; do not let a refactor "accidentally" land any of these.

### Order of operations (CI repair first, then promotion debt)

1. ~~**Unblock Linux headless CI**~~: `d343c3c64bc` restores the `rigid_cubes`
   executable and updates the workflow to validate
   `rigid_cubes --headless --frames 10 --screenshot ...` PPM output. This
   intentionally chooses the promoted capture contract over restoring `--out`
   PNG sequences in the same checkpoint.
2. ~~**Retarget the boundary-guard test**~~ (`UNIT_gui_FilamentSceneExtraction`
   in `tests/unit/gui/test_filament_scene_extraction.cpp`) to scan
   `examples/dartsim/` and the promoted `dart/gui` header surface. The
   configure-time application check is now exposed through
   `dart_gui_add_application()` and continues to guard the `dartsim` entry
   point.
3. ~~**Restore legacy examples as real per-dir binaries**~~: `d343c3c64bc`
   restores the historical GUI example executable names as thin launchers using
   `examples/gui_scene_example.cmake` and `examples/gui_scene_launcher.cpp`,
   not just the first three examples.
4. ~~**Sweep stale user-facing `filament_gui` strings**~~ across the repo:
   runner
   `scripts/run_cpp_example.py`, `python/tests/unit/test_run_cpp_example.py`,
   smoke test names, status messages in
   `dart/gui/experimental/detail/filament/testing/run_headless_smoke.cmake`,
   `CHANGELOG.md`, `dart/gui/AGENTS.md`, and skill docs under
   `.claude/skills/dart-build/` and `.codex/skills/dart-build/`. Private
   CMake helper names under `dart/gui/experimental/detail/filament/` were
   swept in `30b879458f8`.
5. ~~Rerun `pixi run lint`, commit, and push the checkpoint as usual. Do not
   open a PR.~~ `d343c3c64bc` is pushed to the tracked remote branch.
6. **Track ignored `simple_frames` launcher**: force-add
   `examples/simple_frames/CMakeLists.txt`, rerun lint, commit, push, and
   redispatch CI. This is a Git tracking repair for the restored examples, not
   a renderer behavior change.

### Next promoted-header slice

- The initial promoted-header checkpoint has landed in `c9ccedfebe8`.
- Continue moving maintained public-facing examples, tests, and bindings off
  `dart/gui/experimental/*.hpp` / `dartpy.gui.experimental` where those
  concepts are now official.
- Keep `dart/gui/experimental/*.hpp` as compatibility shims for this checkpoint.
- Keep private Filament/GLFW/ImGui implementation under
  `dart/gui/experimental/detail` until a later file-layout sweep.

### Open issues raised by the parallel review pass

- **[verifier]** Per-scene headless smokes (~30 scenes) all run
  `ANALYSIS_MODE basic` (nonzero-pixel only). The "dark/mid/bright +
  luminance spread" gate documented in `03-milestones.md` only applies to
  the single aggregate `EXAMPLE_dartsim_headless_smoke`. Either upgrade the
  per-scene smokes to `contrast` mode or amend the docs to match reality.
- ~~**[verifier]** `scanHeadersForBackendTokens` has a documented hook for
  promoted `dart/gui/*.hpp` headers, but
  `guiHeaderDirectoriesForBackendTokenScan()` returns only the experimental
  directory. Now that `dart/gui/application.hpp` is promoted, register
  `dart/gui` in that hook so the guard becomes live, not aspirational.~~
  Local repair adds `dart/gui` to that scan.
- **[code-reviewer] over-fragmentation** under
  `dart/gui/experimental/detail/filament/`: collapse the per-frame trio
  `simulation_stepper.{hpp,cpp}` (72/110), `frame_renderer.{hpp,cpp}`
  (67/112), and `frame_viewport.{hpp,cpp}` (75/85) into one `frame_loop`
  unit (~520 LOC); fold `application_teardown.{hpp,cpp}` (61/73) back into
  `application.cpp`; consider merging `scene_startup.*` + `scene_frame.*`
  into `scene_lifecycle`. 19 of 49 files in that directory are <100 LOC.
- **[code-reviewer] under-fragmentation**: `scene_fixtures.cpp` is **4082
  LOC** and `scene_requirements.cpp` is **1844 LOC** with a 130-field
  `SceneContentCounts` God Struct that is a Shotgun-Surgery magnet. Split
  fixtures by family (rigid_demos, joint_demos, robot_demos, geometry_demos)
  and replace `SceneContentCounts` with a `std::unordered_map<SceneKind,
std::size_t>`.
- **[code-reviewer] encapsulation leaks under `detail/`**: `scene_frame.hpp`,
  `frame_viewport.hpp`, `renderable_resources.hpp`, and `scene_startup.hpp`
  expose `GLFWwindow*` / `::filament::Engine` / `::filament::View&` /
  `ImGuiIO&` in public function signatures. Move these behind pImpl or a
  renderer-context handle so `detail/` becomes a real seam, not just a
  directory.
- **[critic]** Reconcile this file with `10-active-execution.md` regarding
  the promoted application header. Decision 4 above is the resolution;
  record an explicit cross-reference in both docs so future sessions don't
  reopen the question.
- ~~**[critic] no branch-level definition of done**. Add an explicit "Branch
  is done when:" checklist to `10-active-execution.md` covering: (a) zero
  `experimental` token in promoted names, (b) Linux headless CI green,
  (c) the agreed N restored example binaries exist and produce screenshots,
  (d) no `filament_gui` user-facing strings remain in any maintained file.~~
  Local repair adds `Branch Done Checklist` to `10-active-execution.md`.

### How the supervisor uses this section

- Each new live human direction gets translated into the lists above (not
  into a new file). Codex reads `STEERING.md` after `RESUME.md` per the
  existing convention.
- When a numbered step in "Order of operations" lands, mark it `~~done~~`
  with a one-line evidence pointer (commit short SHA + file). Do not delete
  it for one review cycle so review agents can tell what just shipped.
- The "Decisions in force" block is sticky. If you need to change one, flag
  it under "Open issues" and wait for the next supervisor pass to confirm.

---

## Original 2026-05-14 evaluator pass (kept for history)

## Current branch update

Since this steering note was first written, the post-MVP branch landed the
recommended maintainability slices:

- `dart/gui/experimental/debug.cpp` owns debug descriptor generation.
- `dart/gui/experimental/geometry.hpp` and `.cpp` own backend-hidden mesh
  builders for generated primitive and descriptor geometry.
- `dart/gui/experimental/detail/filament/scenes.hpp` and `.cpp` own scene
  option parsing and dispatch, while `scene_fixtures.hpp` and `.cpp` own
  reusable DART world fixtures.
- `dart/gui/experimental/interaction.cpp` owns picking, plane/axis-drag,
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
   option parsing and dispatch, while
   `dart/gui/experimental/detail/filament/scene_fixtures.hpp` and `.cpp` own
   MVP, hello-world, boxes, hardcoded-design, rigid-chain, rigid-loop,
   mixed-chain, coupler-constraint, add-delete-skels, vehicle,
   hybrid-dynamics, joint-constraints, free-joint-cases, mimic-pendulums,
   atlas-puppet, operational-space-control, drag-and-drop, simple-frames, soft-bodies,
   point-cloud, capsule-ground-contact, simulation-event-handler, polyhedron,
   heightmap, and G1 DART world fixtures. The `biped_stand` runner reuses the
   hybrid-dynamics fullbody fixture rather than adding duplicate scene code.
   `dart/gui/experimental/detail/filament/application.hpp` and `.cpp` now own
   Filament frame orchestration and the built-in panel, leaving `main.cpp` as a
   minimal entry point.
   `dart/gui/experimental/detail/filament/scene_requirements.hpp` and `.cpp`
   now own scene fixture requirement counting and MVP/G1/hello-world/boxes/
   hardcoded-design/rigid-chain/rigid-loop/mixed-chain/coupler-constraint/
   add-delete-skels/vehicle/hybrid-dynamics/joint-constraints/
   free-joint-cases/mimic-pendulums/atlas-puppet/operational-space-control/
   drag/simple-frames/soft-bodies/point-cloud/
   capsule-ground-contact/simulation-event-handler/polyhedron/heightmap
   validation gates. Runner-only reuse, such as `biped_stand` using
   hybrid-dynamics, should stay in the runner/docs layer unless it needs a
   distinct scene fixture.
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
any surviving `examples/filament_gui/` directory should contain only a minimal
`main.cpp` entry-point shell plus unavoidable build/docs files, while renderer
setup, frame lifecycle, resources, synchronization, capture, overlays, input
translation, and fixture logic live under `dart::gui` or private GUI
implementation units.
The promoted `dart::gui` API shape will fall out naturally from what remains
private implementation detail after the example entry point bottoms out.
