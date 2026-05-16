# Active Execution Notes

This file is the current source of truth for the long-running promotion branch.
Update it before discussing or acting on new scope, plan, or design changes so
context survives across sessions.

## Maintainer Steering

- Filament with GLFW3 and Dear ImGui is the only official built-in renderer.
- OpenSceneGraph is fully replaced by the new `dart::gui` direction.
- Raylib is not a maintained renderer path.
- Continue making reasonable checkpoint commits and pushing them to the tracked
  remote branch so GitHub Actions can run.
- Do not open a pull request for this branch.
- Do not wait for CI to finish before making independent progress.
- Remove the remaining `experimental` naming from promoted Filament GUI
  concepts.
- Restore all pre-existing user-facing examples by migrating them to the new
  `dart::gui` API and adding required replacement features.
- Do not call the MVP executable `filament_gui`; example names should describe
  their scope because there is now only one official renderer.
- Branding rule: `DART` is the project and library family identity, `libdart`
  is appropriate for package/library naming, and `dartsim` is the
  application-level simulator/viewer identity analogous to Isaac Sim.
- Current application pivot: `dartsim` should live under `apps/dartsim/` as a
  real application, not under `examples/`. The examples should return to
  educational per-directory programs that consume public `dart::gui` APIs.

## Branch And CI State

- Branch: `feature/filament-gui-full-execution`
- Upstream: `origin/feature/filament-gui-full-execution`
- Latest pushed checkpoint before this in-progress slice:
  `95a6ad0a737 Restore simple frames run defaults`
- GitHub Actions were manually dispatched for the pushed checkpoint without
  opening a PR:
  - CI Lint: https://github.com/dartsim/dart/actions/runs/25945043484
  - CI Linux: https://github.com/dartsim/dart/actions/runs/25945043509
  - CI macOS: https://github.com/dartsim/dart/actions/runs/25945043561
  - CI Windows: https://github.com/dartsim/dart/actions/runs/25945043471
  - CodeQL: https://github.com/dartsim/dart/actions/runs/25945043499
- Linux Filament smoke tests passed at the latest inspected run.
- Linux headless rendering failed earlier because the workflow still invoked the
  legacy `rigid_cubes` executable, which was removed in the earlier example
  cleanup. Restoring legacy example entry points and capture compatibility is
  therefore a concrete CI requirement, not only documentation polish.
- CI Lint on `d343c3c64bc` and `c9ccedfebe8` exposed that
  `examples/simple_frames/CMakeLists.txt` was ignored by the repository
  `*_frames/` ignore rule and therefore missing on GitHub. `cd04ba4862ac`
  force-adds that launcher so CMake can configure the restored examples on CI.
- CI Lint on `cd04ba4862ac` passed; Linux, macOS, Windows, and CodeQL were still
  running when the Python GUI promotion checkpoint was pushed.
- CI Lint on `858ab55cacd1` passed; Linux, macOS, Windows, and CodeQL were
  cancelled by newer checkpoint dispatches.
- CI Lint on `30b879458f8` passed; Linux, macOS, Windows, and CodeQL were
  cancelled by the newer `8796ed5ad99` checkpoint dispatch.
- CI for `8796ed5ad99` was manually dispatched without opening a PR:
  - CI Lint: https://github.com/dartsim/dart/actions/runs/25945825085
    completed successfully.
  - CI Linux: https://github.com/dartsim/dart/actions/runs/25945825122 was
    running at the latest check.
  - CI macOS: https://github.com/dartsim/dart/actions/runs/25945825123 was
    running at the latest check.
  - CI Windows: https://github.com/dartsim/dart/actions/runs/25945825130 was
    running at the latest check.
  - CodeQL: https://github.com/dartsim/dart/actions/runs/25945825158 was
    running at the latest check.
- The restored-example repair is pushed:
  - At that checkpoint, `examples/dartsim` was the renamed app-level viewer;
    the current application pivot moves that source to `apps/dartsim`.
  - `dart/gui/application.hpp` exposes the narrow promoted launch API.
  - Historical GUI example executables are restored as thin `dart::gui`
    launchers.
  - The Linux headless workflow now validates `rigid_cubes --screenshot`
    output instead of the removed OSG-style `--out` PNG sequence.
  - Local evidence: `pixi run lint`, full `examples` target build,
    `UNIT_gui_FilamentSceneExtraction`,
    `python/tests/unit/test_run_cpp_example.py`, direct `rigid_cubes`
    headless PPM capture, and `pixi run test-dart-gui-smoke` all passed
    before or during the checkpoint.
- Recent manual `gh workflow run ... --ref feature/filament-gui-full-execution`
  attempts after the WAM/Atlas checkpoints failed with HTTP 403 because the
  token lacks workflow-dispatch permission on this repository. Continue
  pushing checkpoints so push-triggered CI can run; do not block local
  progress on manual dispatch.
- The Fetch parity checkpoint was pushed to the tracked remote branch without
  opening a PR. Push-triggered CI runs were created for lint, Linux, macOS,
  Windows, and CodeQL. Do not wait for those runs before continuing independent
  parity work.
- The renderer-neutral keyboard-action checkpoint
  (`c91adf346e5 Add GUI keyboard actions`) was pushed to the tracked remote
  branch without opening a PR. It restores the Tinkertoy hotkeys that can be
  expressed without new camera-reset or recording APIs and provides the public
  input surface needed for robot/IK parity work.
- The robot/IK behavior checkpoint
  (`39d1c51de47 Restore puppet teleoperation actions`) was pushed to the
  tracked remote branch without opening a PR. It restores Atlas/Hubo continuous
  IK solving and WASD/QE/FZ root teleoperation through promoted `dart::gui`
  concepts. Do not wait for CI before continuing independent parity work.
- The Tinkertoy camera-home checkpoint
  (`5c922f27a7d Restore Tinkertoy camera home action`) was pushed to the
  tracked remote branch without opening a PR. It adds the public
  keyboard-action camera reset callback and restores Tab camera-home in
  Tinkertoy. Do not wait for CI before continuing independent parity work.

## Authoritative Current Status

Use this section first when resuming; older checkpoint notes below preserve
history but are not guaranteed to be in chronological order.

- Latest pushed commit on the tracked branch:
  `7ec64c3609e Audit CSV logger example`.
- Latest pushed code checkpoint:
  `7ec64c3609e Audit CSV logger example`.
- Current worktree note: `docs/dev_tasks/filament_gui/STEERING.md` has
  pre-existing local edits and should remain unstaged unless the maintainer
  explicitly asks to include it.
- Recent pushed example-parity checkpoints after Fetch include:
  - `619af5649bc Restore LCP physics controls`
  - `2bc6a0e168d Restore mimic pendulums diagnostics`
  - `3493d7065c0 Restore box stacking solver controls`
  - `74870cc5cf3 Restore boxes run defaults`
  - `95a6ad0a737 Restore simple frames run defaults`
  - `9e111631eb5 Restore point cloud example controls`
  - `227c2498a53 Restore polyhedron visual defaults`
  - `158e10d4cc2 Restore empty viewer scaffold`
  - `6ee2b97a20f Restore simulation event handler controls`
  - `bcc31d495c8 Restore hello world defaults`
  - `b00e054c45f Restore capsule contact controls`
  - `b864e5a31d3 Restore rigid chain defaults`
  - `4a957790bef Restore rigid loop defaults`
  - `9f4af05ef1c Restore rigid cubes controls`
  - `ce8ba83d2f3 Restore Fetch target affordance`
  - `3945c65852c Restore coupler constraint controls`
  - `1297aca1fe6 Restore drag and drop affordances`
  - `ae175ef6981 Restore Fetch target manipulation`
  - `63f990d5251 Restore panel extension example`
  - `7ec64c3609e Audit CSV logger example`
- Maintainer correction for the active slice: source ownership, build success,
  and headless screenshot output are not sufficient evidence that an example is
  fully restored. Every pre-existing user-facing example must be compared
  against its historical source for user-visible behavior.
- `examples/fetch/` is the concrete reminder for that rule. Known restored
  Fetch behavior currently includes MJCF loading, Bullet preference,
  robot/object initial positions, mocap weld reset, target sync, visible target
  cross, work-area grid, camera home, 1280x960 default launch size, GUI scale,
  selection/drag/nudge instructions, Play/Pause/Step/Exit panel controls,
  README, and source-marker tests. If any further Fetch-specific historical
  behavior is identified, repair it through promoted `dart::gui` APIs.
- Completed implementation checkpoint: `examples/hardcoded_design` now
  restores number-key joint increments for DOFs 0-2, `-` direction toggling,
  camera home from eye `(2, 2, 2)` to target `(0, 0, 0)`, README/help text,
  changelog coverage, and source-marker tests through public `dart::gui`. The
  old OSG wireframe mode remains a documented follow-up requiring a DART-owned
  render-style/debug concept rather than OSG or private Filament hooks.
- Hardcoded-design validation completed before push: focused C++ target build,
  focused `UNIT_gui_FilamentSceneExtraction` CTest, direct and pixi
  hardcoded-design screenshots with analyzer coverage, Python C++ example
  runner tests, full `examples` target build, `git diff --check`,
  `pixi run lint`, and post-lint focused rebuild/CTest/direct screenshot
  checks.
- Maintainer correction after the `rigid_cubes` checkpoint: there are many
  more examples that are not fully restored, and `examples/fetch/` is still the
  concrete reminder. The next work must use an explicit historical-source
  inventory for every pre-existing example before calling it restored.
  `docs/dev_tasks/filament_gui/11-example-parity-audit.md` is the live
  per-example checklist, and
  `docs/dev_tasks/filament_gui/12-strict-example-restoration.md` records the
  broader rule and active queue.
- Immediate audit priority: re-open `examples/fetch/` as an itemized
  historical-source comparison, even though previous checkpoints restored many
  Fetch behaviors. Any remaining Fetch-specific behavior gap must either be
  repaired through promoted `dart::gui` APIs or recorded as a named public API
  follow-up.
- Current Fetch status: previous checkpoints restored many visible behaviors,
  but Fetch remains re-openable. Re-check the current source against
  `520993d7301^:examples/fetch/main.cpp` before declaring it complete; repair
  any new behavior gap through promoted `dart::gui` or record the exact public
  API gap.
- Fetch panel-text validation completed before this checkpoint commit: focused
  C++ build for `fetch` and `UNIT_gui_FilamentSceneExtraction`, focused CTest
  for `UNIT_gui_FilamentSceneExtraction`, direct and pixi Fetch headless
  screenshots with analyzer coverage, and
  `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  (67 passed).
- Fetch panel-text post-lint validation completed: `pixi run lint`, focused
  rebuild and CTest for `fetch` plus `UNIT_gui_FilamentSceneExtraction`, and a
  direct llvmpipe Fetch screenshot with analyzer coverage
  (`/tmp/dart_fetch_panel_text_direct_postlint.ppm`, 303694/307200 nonzero
  pixels).
- Current implementation checkpoint in the worktree:
  `examples/csv_logger/` strict audit. The historical and current source are
  byte-for-byte identical, so this slice preserves the non-GUI CSV logging
  behavior rather than migrating a GUI viewer. The planned work is to document
  the itemized audit, add source-marker coverage for the CLI/logging contract,
  and validate the executable writes the expected CSV rows before committing.
- Previous human-joint-limits implementation checkpoint:
  `examples/human_joint_limits/` strict audit. The implementation restores the
  live SKEL world, preserves the file timestep/Y-down gravity,
  keeps the ground static, enables joint-limit enforcement on every human
  joint, restores the 640x480 default launch size, prints Space instructions,
  adds README coverage, and adds marker guards through public `dart::gui`.
  The removed TinyDNN dependency/custom neural-network constraint path remains
  a named follow-up unless a maintained dependency or replacement public
  constraint implementation is added.
- Latest Fetch target-bar checkpoint state: committed and pushed as
  `ce8ba83d2f3 Restore Fetch target affordance`. Validation covered focused
  C++ build/CTest, direct and pixi Fetch headless screenshots with analyzer
  coverage, Python C++ example-runner tests, aggregate `examples` build,
  `git diff --check`, mandatory `pixi run lint`, and post-lint focused
  checks.
- Coupler validation completed before lint: focused C++ build for
  `coupler_constraint` and `UNIT_gui_FilamentSceneExtraction`, focused CTest
  for `UNIT_gui_FilamentSceneExtraction`, direct and pixi Coupler headless
  screenshots with basic analyzer coverage, Python C++ example-runner tests
  (67 passed), and aggregate `examples` build.
- Coupler post-lint validation completed and the checkpoint was pushed as
  `3945c65852c Restore coupler constraint controls`. Post-lint evidence
  covered the focused build, focused CTest, and direct llvmpipe headless
  screenshot with basic analyzer coverage.
- Drag-and-drop validation completed before this checkpoint commit: focused
  C++ build for `drag_and_drop` and `UNIT_gui_FilamentSceneExtraction`,
  focused CTest for `UNIT_gui_FilamentSceneExtraction`, direct and pixi
  drag-and-drop headless screenshots with basic analyzer coverage, Python C++
  example-runner tests (67 passed), aggregate `examples` build, mandatory
  `pixi run lint`, and post-lint focused build/CTest/direct screenshot smoke.
- The drag-and-drop checkpoint was pushed as
  `1297aca1fe6 Restore drag and drop affordances`; do not wait for CI before
  continuing independent strict-audit work.
- Human joint-limits validation completed before lint: focused C++ build for
  `human_joint_limits` and `UNIT_gui_FilamentSceneExtraction`, focused CTest
  for `UNIT_gui_FilamentSceneExtraction`, direct and pixi human-joint-limits
  headless screenshots with basic analyzer coverage, Python C++ example-runner
  tests (67 passed), and aggregate `examples` build.
- Human joint-limits post-lint validation completed: mandatory
  `pixi run lint`, focused rebuild and CTest for `human_joint_limits` plus
  `UNIT_gui_FilamentSceneExtraction`, and direct llvmpipe headless screenshot
  with basic analyzer coverage.
- The human joint-limits checkpoint was pushed as
  `cf0ed62209e Restore human joint limits live example`; do not wait for CI
  before continuing independent strict-audit work.
- Latest maintainer correction after the human checkpoint: do not move on as if
  the remaining queue were only small untouched examples. There are still many
  examples that are not fully restored, and `examples/fetch/` is again the
  concrete reminder case.
- Active slice is re-opened `examples/fetch/`: compare the current source to
  `520993d7301^:examples/fetch/main.cpp` again, especially the old
  interactive-frame manipulation affordance and viewer help text. Restore
  anything expressible through public `dart::gui`; if exact mouse rotation
  rings/tools require a new renderer-neutral API, spell out that API gap rather
  than reviving OSG types.
- Implementation state for this Fetch re-open slice: `examples/fetch/main.cpp`
  now replaces the two-bar-only marker with one transparent green line target
  handle that carries local axes/rings on the same selectable `SimpleFrame`,
  adds renderer-neutral target rotation/reset keyboard actions, and restores
  the shared viewer help text in the promoted panel. Exact mouse rotation
  rings/tools remain a public manipulation API follow-up.
- Fetch re-open validation completed before checkpoint commit: focused C++
  build for `fetch` and `UNIT_gui_FilamentSceneExtraction`, focused CTest for
  `UNIT_gui_FilamentSceneExtraction`, direct and pixi Fetch headless
  screenshots with basic analyzer coverage, Python C++ example-runner tests
  (67 passed), aggregate `examples` build, `git diff --check`, mandatory
  `pixi run lint`, and post-lint focused build/CTest/direct screenshot smoke.
- The Fetch target-manipulation checkpoint was pushed as
  `ae175ef6981 Restore Fetch target manipulation`; do not wait for CI before
  continuing independent strict-audit work.
- Active slice is now `examples/imgui/`. Historical-source comparison shows
  the current source is still a generic panel demo rather than the old
  panel-extension example: restore the empty-world target frame, promoted
  keydown callbacks, panel title/sections, gravity control, viewer help,
  run/camera defaults, README, and marker guards. Keep direct backend UI types
  out of source. Record headlight toggles, live camera-inspector text,
  key-release callbacks, and pre/post render or post-step hooks as public
  `dart::gui` API gaps unless the public API is added.
- Implementation state for this panel-extension slice:
  `examples/imgui/main.cpp` now owns an empty world with a selectable target
  frame, restores the historical panel title, Play/Pause/Time, gravity
  controls, viewer help, keydown callbacks for `q`/Left/Right, pre-step
  callback demonstration, 640x480 default size, and camera home through public
  `dart::gui`. The README and marker guards are in place. Headlight toggles,
  live camera readout, key-release callbacks, shifted-`Q` distinction, and
  pre/post render or post-step hooks remain named public API gaps.
- Panel-extension validation completed before checkpoint commit: focused C++
  build for `imgui` and `UNIT_gui_FilamentSceneExtraction`, focused CTest for
  `UNIT_gui_FilamentSceneExtraction`, direct and pixi `imgui` headless
  screenshots with basic analyzer coverage, Python C++ example-runner tests
  (67 passed), aggregate `examples` build, `git diff --check`, mandatory
  `pixi run lint`, and post-lint focused build/CTest/direct screenshot smoke.
- The panel-extension checkpoint was pushed as
  `63f990d5251 Restore panel extension example`; do not wait for CI before
  continuing independent strict-audit work.
- Active slice is now `examples/csv_logger/`. The historical source and README
  match the current files exactly, so the slice should preserve the non-GUI
  command-line logging workflow, document the itemized historical inventory,
  add source-marker coverage for CLI flags, default world/output/step behavior,
  CSV header/row count, skeleton/body selection, and README text, then validate
  the executable by writing a short CSV log.
- CSV logger validation completed before lint: focused C++ build for
  `csv_logger` and `UNIT_gui_FilamentSceneExtraction`, focused CTest for
  `UNIT_gui_FilamentSceneExtraction`, a direct `csv_logger --steps 3` run that
  wrote the expected CSV header plus four data rows, Python C++ example-runner
  tests (67 passed), and aggregate `examples` build.
- CSV logger post-lint validation completed: mandatory `pixi run lint`,
  focused rebuild and CTest for `csv_logger` plus
  `UNIT_gui_FilamentSceneExtraction`, direct `csv_logger --steps 3` CSV
  verification, and `git diff --check`.
- The CSV logger checkpoint was pushed as
  `7ec64c3609e Audit CSV logger example`; do not wait for CI before
  continuing independent strict-audit work.
- Active slice is now `examples/headless_simulation/`. The historical source
  and README match the current files exactly, so this slice should preserve the
  non-GUI deterministic batch-simulation workflow, document the itemized
  historical inventory, add source-marker coverage for CLI flags, deterministic
  seed setup, world loading, simulation stepping, progress/timing output, README
  text, and absence of GUI renderer dependencies, then validate a short
  executable run.
- Headless simulation validation completed before lint: focused C++ build for
  `headless_simulation` and `UNIT_gui_FilamentSceneExtraction`, focused CTest
  for `UNIT_gui_FilamentSceneExtraction`, a direct
  `headless_simulation --steps 3 --dt 0.002 --seed 17` run that printed the
  expected step, time-step, and simulated-time output, Python C++ example-runner
  tests (67 passed), and aggregate `examples` build.
- Headless simulation post-lint validation completed: mandatory
  `pixi run lint`, focused rebuild and CTest for `headless_simulation` plus
  `UNIT_gui_FilamentSceneExtraction`, direct short headless output
  verification, and `git diff --check`.

## Current Code Shape

- The branch has removed legacy OSG/Raylib renderer build, dependency,
  tutorial, example, Python, and documentation surfaces.
- `DART_BUILD_GUI` now defaults according to platform support for the pinned
  Filament path.
- The backend-named MVP example has been promoted into the `apps/dartsim`
  application source tree while preserving the `dartsim` executable name.
- Stable `dart/gui/*.hpp` headers now own renderer-independent scene, viewer,
  geometry, interaction, debug, and profiling declarations in `dart::gui`.
  `dart/gui/experimental/*` remains as compatibility shims only.
- The C++ implementation definitions for promoted renderer-independent symbols
  now live in `namespace dart::gui`. Private Filament implementation names are
  under `dart::gui::filament`, while the physical
  `dart/gui/experimental/detail/filament` path remains file-layout debt for a
  later sweep.
- Private implementation details live under
  `dart/gui/experimental/detail/filament`. They can remain private while the
  promoted public names move to `dart::gui`.
- Existing command-line support includes bounded frames, window size,
  headless mode, UI visibility, scene selection, profiling, a single PPM
  screenshot path, and a `--out <dir>` PPM image-sequence path.
- Current execution decision: keep the restored historical executable names as
  thin `dart::gui` launchers, keep CI validating the promoted `--screenshot`
  capture contract, and keep the restored historical `--out <dir>`
  image-sequence compatibility in the DART GUI capture layer without reviving
  OSG.

## Naming Decisions

- The MVP executable should be renamed by scope:
  - Decision for this slice: `apps/dartsim` / `dartsim`
  - Earlier neutral fallback: `examples/gui_viewer` / `dart_gui_viewer` /
    `gui_viewer`
- Legacy user-facing examples should keep their historical example names where
  those names describe the simulation or workflow, such as `hello_world`,
  `rigid_cubes`, `drag_and_drop`, and `imgui`.
- Backend-named examples should not be treated as official public surfaces.
  `filament_gui` is rejected by the runner with a migration message, and Raylib
  should not be restored as a renderer.
- Branding direction: `DART` names the overall project and the C++ library
  family. Use `libdart` only where package managers or library artifacts need
  that convention. Use `dartsim` for application-level simulator/viewer
  surfaces, analogous to an application product such as Isaac Sim. Do not use
  renderer backend names for public app or example branding.

## Application Pivot Slice

The live steering in `STEERING.md` now treats `dartsim` as a top-level
application, not as an example. The next architecture checkpoints should keep
the public executable name `dartsim` while moving the source directory from
`examples/dartsim/` to `apps/dartsim/`.

Implementation direction:

- Add a top-level `apps/` tree with `apps/dartsim/`.
- Keep `dartsim` built whenever `DART_BUILD_GUI_FILAMENT` is enabled, and keep
  the `examples` aggregate target depending on it only as a developer
  convenience.
- Keep the application entry point on promoted `dart::gui` APIs. The app may
  use private CMake helper plumbing while the file-layout debt still lives
  under `dart/gui/experimental/detail/filament`, but source code should not
  include Filament, GLFW, Dear ImGui, or `dart/gui/experimental` headers.
- Split future app-only behavior under `apps/dartsim/app/`: scene loading,
  docking layout, timeline, inspector, log, recording, project file handling,
  and eventual replay/plotting workflows.
- Use `--dev-scene <name>` or a future test-fixture target for fixture scenes.
  Do not make `--scene <name>` the long-term user-facing app model.
- Treat Dear ImGui Docking and a docked 3D viewport as first-class application
  goals for `apps/dartsim/`; image-sequence capture is required now, while
  video capture remains a later recording feature.

Implementation state:

- `5d514a4558e` documents the application pivot and branding distinction before
  code changes.
- The current code slice moves the existing `dartsim` source directory to
  `apps/dartsim/` while preserving the `dartsim` target, executable name, pixi
  runner behavior, and CI smoke-test names.
- This extraction is intentionally structural. Real app features such as file
  loading, docking layout, timeline, inspector, log, and recording panels remain
  follow-up work under `apps/dartsim/app/`.

## API Promotion Direction

- Public headers should expose DART concepts: viewer options, worlds,
  renderables, cameras, debug overlays, selection, tools, panels, screenshots,
  frame capture, and simulation control.
- Public headers and examples should not expose Filament, GLFW, Dear ImGui,
  OpenGL, Vulkan, Metal, OSG, or Raylib types.
- Stable includes should move toward `dart/gui/*.hpp` and namespace
  `dart::gui`.
- Backend resource ownership should remain private: engine, renderer, swap
  chains, windows, materials, textures, vertex/index buffers, entity maps,
  ImGui draw resources, and native handles.
- Keep renderer-independent helpers only when they represent stable DART GUI
  concepts or make behavior testable without a graphics context. Do not keep a
  multi-backend abstraction only to select between renderer backends.

## Python Binding Promotion Slice

- `858ab55cacd1` promotes the existing backend-hidden GUI descriptor/helper
  symbols from
  `dartpy.gui.experimental` onto `dartpy.gui`, matching the promoted C++
  `dart::gui` surface.
- Keep `dartpy.gui.experimental` importable as a compatibility namespace for
  this checkpoint so existing users and tests do not break abruptly.
- Update Python stubs, Sphinx API docs, tests, and Python onboarding text to use
  `dartpy.gui` as the official surface.
- Do not expose Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or
  Raylib types through Python-facing contracts.

## Private CMake Helper Rename Slice

- `30b879458f8` renames private helper functions and CMake variables that used
  the backend-named `filament_gui` compound to `gui_filament` wording.
- Keep `Filament` in private backend implementation names where it identifies
  the rendering technology; the problem is the old example/product compound
  `filament_gui`.
- Preserve public compatibility rejection for old user-facing names such as
  `filament_gui` and `dart_filament_gui`.
- Keep target/component renames involving `dart-gui-experimental` as a larger
  compatibility step unless a local mechanical rename is clearly safe.

## C++ Namespace Promotion Slice

- `8796ed5ad99` promotes renderer-independent declarations from
  `dart::gui::experimental` to
  first-class `dart::gui` declarations in the stable `dart/gui/*.hpp` headers.
- Keep `dart/gui/experimental/*.hpp` installed as compatibility shims that
  include the stable headers and alias the old namespace names to `dart::gui`.
- Move implementation definitions to `namespace dart::gui` so promoted symbols
  are no longer aliases of an experimental ABI surface.
- Update private Filament implementation includes/usages to prefer
  `dart/gui/*.hpp` and `dart::gui`; the private Filament namespace is promoted
  to `dart::gui::filament` while the physical
  `dart/gui/experimental/detail/filament` layout remains private file-layout
  debt for a later, larger sweep.
- Keep any target/component rename for `dart-gui-experimental` separate unless
  it remains a local CMake-only change with no compatibility risk.
- Add a unit guard that promoted `dart/gui/*.hpp` headers cannot reintroduce
  `dart/gui/experimental`, `dart::gui::experimental`, or
  `dart-gui-experimental` tokens.
- Local verification for this slice:
  - C++ GUI target build for `dart-gui`, `dartsim`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run build-py-dev`
  - `pixi run python -m pytest python/tests/unit/gui/test_gui_scene.py -q`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct `dartpy.gui` / `dartpy.gui.experimental` identity import check
  - `pixi run test-dart-gui-smoke`
  - `pixi run lint`

## Example Restoration Plan

Restore the pre-existing user-facing examples as migrated `dart::gui` examples
instead of leaving them collapsed into one backend-named executable or one
shared fixture launcher.

Implementation direction:

- Keep each restored example's `main.cpp` small, educational, and DART-owned.
  It should call promoted `dart::gui` APIs directly, using public panel/tool
  hooks where the historical example had custom controls.
- Preserve historical example executable names for CI and user muscle memory.
- Keep Filament-specific setup in private GUI implementation units.
- Ensure `scripts/run_cpp_example.py`, CMake targets, tests, and CI workflows
  refer to scope-based or historical example names, not `filament_gui`.
- Restore examples before relying on CI jobs that invoke historical binaries
  such as `rigid_cubes`.
- Use the shared `gui_scene_launcher.cpp` and `scene_fixtures.cpp` path only as
  transitional infrastructure. The branch stop condition is real
  `examples/<name>/main.cpp` files and no example behavior defined inside
  private GUI library fixture code.

The restoration should cover all maintained simulation/workflow examples that
existed before the cleanup, including simple scenes, robotics scenes,
interaction scenes, ImGui/panel workflows, headless capture workflows, and mesh
or debug-visual examples. Backend-only renderer demos are not restored as
alternate renderer implementations.

## Required Feature Parity For This Slice

- Windowed viewer launch through the promoted `dart::gui` entry point.
- Headless bounded rendering through the promoted entry point.
- Screenshot capture through the promoted entry point.
- Compatibility for CI headless rendering expectations, either through a
  restored `--out` frame-output mode or through an updated workflow that checks
  the new capture contract.
- ImGui-based built-in controls remain private implementation policy.
- Existing scene defaults and scene validation should continue to exercise the
  migrated examples.

## Capture Compatibility Slice

This implementation slice restores the historical `--out <dir>` capture
contract as image-sequence output while retaining the current promoted
`--screenshot <path>` single-frame contract.

Implementation state:

- A renderer-independent capture option has been added to the promoted `dart::gui`
  run-options layer instead of encoding the behavior in an example launcher.
- The shared `dartsim` / restored-example command-line path parses
  `--out <dir>` and creates the directory when needed.
- For each rendered frame in a bounded headless or windowed run, the Filament
  backend writes a PPM frame image with stable `frame_000001.ppm` numbering.
  This first version prioritizes correctness over throughput by waiting for
  each readback before the next frame.
- Keep `--screenshot <path>` as the final/single-frame capture API used by CI
  smoke tests.
- If both `--out` and `--screenshot` are provided, sequence frames and the
  final screenshot are both saved while reusing the same readback on the final
  frame.
- Local evidence so far:
  - C++ GUI target build for `dart-gui`, `dartsim`, and `rigid_cubes`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run build-py-dev`
  - `pixi run python -m pytest python/tests/unit/gui/test_gui_scene.py -q`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct `rigid_cubes --headless --frames 2 --out ...` frame-sequence run
  - `pixi run ex rigid_cubes --headless --frames 2 --out ...` frame-sequence
    run with final screenshot preservation

## Branding Cleanup Slice

This implementation slice removes small remaining user-facing backend or
experimental labels where no compatibility contract depends on the old names.

Implementation direction:

- Change the GLFW window title from the old experimental Filament wording to
  the application-level `dartsim` brand.
- Rename private CMake default variables that still use the old
  `filament_gui` compound.
- Prefer `DART GUI` wording for CI labels and CMake comments where the label is
  about the maintained GUI surface rather than a private backend technology.
- Keep compatibility shims such as `dart/gui/experimental/*.hpp`,
  `dartpy.gui.experimental`, and the rejected legacy `filament_gui` runner
  aliases until a deliberate compatibility-removal checkpoint.
- The new `test-dart-gui-smoke` pixi task is the promoted smoke-test entry
  point; the old `test-filament-gui-smoke` task remains as a compatibility
  alias for now.

## Per-Example Source Migration Direction

The latest steering in `STEERING.md` supersedes the earlier Tier-A/Tier-B
split: every pre-existing user-facing example should regain a real
`examples/<name>/main.cpp` and plain per-example CMake target. The restored
example binaries currently share the generic launcher and `--scene <name>`
fixtures. That keeps the executable names and headless coverage working, but it
does not restore the educational example sources or move behavior out of the
private GUI library fixture code.

Execution order:

1. Keep extending the promoted `dart::gui` panel/tool/callback hook only when a
   restored example or `apps/dartsim` needs it; do not expose Dear ImGui or
   Filament types.
2. Migrate examples to real per-example source files that include
   `dart/gui/*.hpp`, link `dart-gui`, and avoid private backend headers.
3. Move matching behavior out of `scene_fixtures.cpp` once the public API is
   sufficient for that example.
4. Only after the application extraction and example restoration proof points,
   start the mechanical directory cleanup that moves private implementation
   paths out of `dart/gui/experimental/detail`.

First API checkpoint:

- Add `dart/gui/panel.hpp` with a renderer-neutral `PanelBuilder` interface
  for text, separators, same-line layout, buttons, checkboxes, and numeric
  sliders.
- Extend `dart::gui::ApplicationOptions` with `defaultScene` and custom
  panels, then add `runApplication(argc, argv, ApplicationOptions)`.
- Render those panels inside the private Filament/ImGui implementation without
  exposing `imgui.h`, GLFW, or Filament headers through `dart/gui/*.hpp`.
- Migrate `examples/imgui` first because it is the smallest Tier-A proof that
  an example can own a real `main.cpp` and user-facing controls while linking
  only against `dart-gui`.

Implementation state:

- `dart/gui/panel.hpp` and `dart::gui::ApplicationOptions` are implemented.
- The private Filament implementation renders application panels inside the
  existing UI frame through an internal ImGui adapter.
- `examples/imgui` now has its own `main.cpp` using promoted `dart::gui`
  headers and `ApplicationOptions.panels`.
- Local evidence so far:
  - C++ GUI target build for `dart-gui`, `dartsim`, `imgui`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Full `examples` target build
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct `imgui --headless --frames 1 --screenshot ...` capture
  - Direct `imgui --headless --show-ui --frames 1 --screenshot ...` capture
    to exercise the custom panel render path

Second API checkpoint:

- Extend panel callbacks with a renderer-neutral `PanelContext` containing the
  current world pointer, lifecycle state, selection label, simulation time, and
  contact count.
- Keep `PanelContext` free of Filament, GLFW, and Dear ImGui types.
- Migrate `examples/drag_and_drop` and `examples/tinkertoy` to real
  per-example source files that use context-aware panels for example-owned
  controls and status.
- Current implementation state:
  - `PanelContext` is wired into private panel rendering.
  - `examples/drag_and_drop` and `examples/tinkertoy` have real promoted
    `dart::gui` entry points with context-aware panels.
  - Local evidence so far:
    - C++ GUI target build for `dart-gui`, `imgui`, `drag_and_drop`,
      `tinkertoy`, and `UNIT_gui_FilamentSceneExtraction`
    - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
    - Direct `drag_and_drop --headless --show-ui --frames 1 --screenshot ...`
      capture
    - Direct `tinkertoy --headless --show-ui --frames 1 --screenshot ...`
      capture
    - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`

Third API/example checkpoint:

- Add `dart::gui::ApplicationOptions::world` so application and example code
  can hand a DART `WorldPtr` to the official GUI runner without reaching into
  private Filament scene fixtures.
- Restore `examples/hello_world/main.cpp` as a real source file that constructs
  the falling blue box and ground through public DART dynamics/simulation APIs,
  then passes that world to `dart::gui::runApplication`.
- Drop the `hello_world` runner default `--scene hello-world` argument so the
  restored binary runs its own source-defined world by default.
- Keep the private `createHelloWorldScene()` fixture as transitional dev/test
  infrastructure until fixture deletion is batched with the remaining example
  migrations.
- Local evidence so far:
  - C++ GUI target build for `dart-gui`, `hello_world`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct `hello_world --headless --frames 1 --screenshot ...` capture
  - `pixi run ex hello_world --headless --frames 1 --screenshot ...` capture

Fourth source-migration checkpoint:

- Restore the rigid-box example family as source-defined public-API examples:
  `examples/boxes`, `examples/rigid_cubes`, and `examples/box_stacking`.
- Keep the examples educational and self-contained: construct DART worlds with
  public dynamics/simulation APIs, pass them through
  `dart::gui::ApplicationOptions::world`, and keep GUI controls on
  `dart::gui::Panel` where needed.
- Remove the runner default `--scene` injection for these binaries so direct
  launches and `pixi run ex <name>` execute the restored example source by
  default.
- Keep the private `createBoxesScene()` fixture and related validation counters
  as transitional dev/test infrastructure for `dartsim --scene boxes` until
  the remaining examples no longer depend on shared fixture coverage.
- Local acceptance for this checkpoint:
  - `f812a4afc1d` restores all three rigid-box sources.
  - C++ GUI target build for `dart-gui`, the three migrated examples, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for all three migrated rigid-box
    binaries
  - `pixi run ex boxes --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`

Fifth source-migration checkpoint:

- Restore `examples/simple_frames` and `examples/capsule_ground_contact` as
  source-defined public-API examples.
- `simple_frames` should demonstrate `dart::dynamics::SimpleFrame` visual
  hierarchy construction with public DART shapes and no private fixture
  launcher.
- `capsule_ground_contact` should build the capsule, plane collision, and
  visible ground through public dynamics/simulation APIs, with a small
  `dart::gui::Panel` replacing the old OSG key-handler reset affordances.
- Remove the runner default `--scene` injection for these binaries so direct
  launches and `pixi run ex <name>` execute the restored example source by
  default.
- Keep the matching private scene fixtures as transitional dev/test
  infrastructure until the remaining shared-fixture examples have migrated.
- Local acceptance for this checkpoint:
  - `45e44a0318a` restores both sources.
  - C++ GUI target build for `simple_frames`, `capsule_ground_contact`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both migrated binaries
  - `pixi run ex simple_frames --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`

Correction after checkpoint `3b4ac2cae7b`:

- The examples restored so far are only early proof points. Many historical
  examples remain fixture-backed and therefore not fully restored yet, including
  `examples/fetch`.
- Treat each remaining `dart_build_gui_scene_example(...)` target as incomplete
  until it has a real `examples/<name>/main.cpp`, public-API CMake wiring, and
  no runner-injected `--scene <name>` default.

Sixth source-migration checkpoint:

- Restore `examples/fetch` as a source-defined public-API example. Current
  status correction: `fetch` is source-owned, but not yet fully restored to
  parity with the historical example.
- Add the smallest renderer-neutral lifecycle hook required by the original
  Fetch behavior: `dart::gui::ApplicationOptions::preStep`, invoked before each
  simulation step when an example supplies its own world.
- Use the hook to keep the MJCF mocap skeleton aligned with the example-owned
  target `SimpleFrame`, replacing the private fixture-only `DartScene::preStep`
  path for this example.
- Keep renderer-specific drag/manipulation details private. The restored source
  may expose status and play/step controls through `dart::gui::Panel`, but it
  must not include Filament, GLFW, Dear ImGui, OSG, or
  `dart/gui/experimental` headers.
- Remove the `fetch` runner default `--scene fetch` after the real source is
  active.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `dart-gui`, `fetch`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct `fetch --headless --frames 1 --screenshot ...` capture
  - `pixi run ex fetch --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`

Fetch parity correction after user review:

- Treat `examples/fetch` as **source-owned, parity pending** until the public
  example preserves the old Fetch-specific affordances: Bullet preference when
  available, visible draggable target/cross handles, historical live
  target-following simulation behavior, stronger target-following panel
  content, and documented camera-default gap until `dart::gui` exposes
  per-example camera defaults.
- The next code slice repairs `examples/fetch/` before continuing broader
  robot/IK migration, because it is a concrete example where `options.world`
  was mistaken for full restoration.
- Implementation state for this slice: `examples/fetch/main.cpp` now prefers
  Bullet when available, preserves the live mocap target-following `preStep`,
  restores visible green target bars from public `SimpleFrame`/shape APIs,
  and has a focused boundary guard for those parity markers.
- Local evidence for the in-progress checkpoint:
  - C++ GUI target build for `fetch` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct `fetch --headless --frames 1 --screenshot ...` capture
  - `pixi run ex fetch --headless --frames 1 --screenshot ...`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused rebuild/CTest for `fetch` and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint direct `fetch --headless --frames 1 --screenshot ...` capture

Seventh source-migration checkpoint:

- Continue the "many examples remain" correction by restoring the next
  related macro-shim batch: `rigid_chain`, `rigid_loop`, `mixed_chain`,
  `coupler_constraint`, `add_delete_skels`, and `rigid_shapes`.
- Recover the removed legacy sources from history, then migrate their world
  setup and simple controls to public DART + `dart::gui` APIs. Keep the
  examples educational and standalone; they should not include Filament, GLFW,
  Dear ImGui, OSG, or `dart/gui/experimental` headers.
- Use `dart::gui::ApplicationOptions::world` for source-owned worlds and
  `dart::gui::ApplicationOptions::preStep` where a fixture had simulation-loop
  behavior that must move out of private scene fixtures.
- Remove the matching runner default `--scene ...` entries once each binary
  owns its scene by default.
- Keep private fixtures temporarily for `dartsim --scene ...` developer
  coverage until the entire example migration and fixture cleanup are ready.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `dart-gui`, the six migrated examples, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for at least two representatives in the
    batch, including one constraint-heavy example
  - `pixi run ex rigid_chain --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Evidence: `fdd9d248033` restores the six sources, removes their runner
  defaults, validates the boundary/unit/Python runner path, captures
  representative headless screenshots, builds all examples, runs lint, pushes,
  and dispatches GitHub Actions without opening a PR.

Eighth source-migration checkpoint:

- Restore the joint/dynamics macro-shim batch:
  `examples/hybrid_dynamics`, `examples/biped_stand`,
  `examples/joint_constraints`, `examples/free_joint_cases`, and
  `examples/human_joint_limits`.
- Recover the legacy simulation setup from history and the current private
  fixtures, then migrate it into public DART + `dart::gui` example sources.
  Preserve the important per-step controller/reference behavior through
  `dart::gui::ApplicationOptions::preStep`.
- Use promoted `dart::gui::Panel` for compact simulation status and controls
  that fit the current renderer-neutral panel API. Do not expose Filament,
  GLFW, Dear ImGui, OSG, or `dart/gui/experimental` headers from these
  examples.
- Remove the matching runner default `--scene ...` entries after the binaries
  own their worlds by default.
- Keep the matching private fixtures temporarily for `dartsim --scene ...`
  developer coverage until all example behavior has migrated and fixture
  cleanup can be batched.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `dart-gui`, the five migrated examples, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshots for the migrated examples, including the
    controller-heavy and free-joint cases
  - `pixi run ex hybrid_dynamics --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Evidence: this checkpoint restores all five sources, removes their runner
  defaults, validates the boundary/unit/Python runner path, captures all five
  migrated binaries headlessly, validates the Pixi `hybrid_dynamics` route,
  smoke-checks inherited `--gui-scale` parsing, builds all examples, and runs
  lint before commit/push/CI dispatch.

Ninth source-ownership checkpoint:

- Remove the remaining per-example `options.defaultScene` inverse dependency
  from `examples/imgui` and `examples/tinkertoy`.
- `imgui` should own a small public-API DART world suitable for demonstrating
  renderer-neutral `dart::gui::Panel` controls instead of borrowing the private
  `mvp` developer fixture.
- `tinkertoy` should move the current Tinkertoy reference axes, target,
  force-line, and initial block assembly out of the private named-scene
  fixture path and into `examples/tinkertoy/main.cpp`.
- This checkpoint is an ownership repair, not a full claim that every old OSG
  Tinkertoy input affordance has been rebuilt. Any missing historical keyboard,
  recording, and builder-edit controls remain parity gaps until public
  `dart::gui` exposes the required input/tool surfaces.
- Remove the `tinkertoy` runner default `--scene tinkertoy` after the binary
  owns its world.
- Local acceptance for this checkpoint:
  - `git grep -nE 'options\.defaultScene\s*=' examples/` returns no matches.
  - C++ GUI target build for `imgui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `imgui` and `tinkertoy`
  - `pixi run ex tinkertoy --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/imgui/main.cpp` now owns a
  small public-API panel-demo world, `examples/tinkertoy/main.cpp` now owns
  the Tinkertoy axes, target, force-line, and initial assemblies, and the
  runner no longer injects `--scene tinkertoy`.
- Local evidence so far:
  - `git grep -nE 'options\.defaultScene\s*=' examples/` returns no matches.
  - C++ GUI target build for `imgui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `imgui` and `tinkertoy`
  - `pixi run ex tinkertoy --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
  - Post-lint C++ GUI target rebuild for `imgui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`

Tenth source-ownership checkpoint:

- Restore the WAM robot/IK pair before the larger Atlas/Hubo puppet slice:
  `examples/operational_space_control` and `examples/wam_ikfast`.
- These examples still launch through `dart_build_gui_scene_example(...)` and
  runner-injected `--scene` values. Move them directly to source-owned
  promoted `dart::gui::ApplicationOptions` without introducing any new
  `options.defaultScene` dependency.
- `operational_space_control` should load the WAM URDF with a public
  `dart::io::ReadOptions` package directory, create its target frame and
  ground in the example source, and carry the operational-space controller as
  example-owned pre-step behavior.
- `wam_ikfast` should load the WAM URDF, create the visible target frame and
  ground in the example source, and keep the WAM visual-only so the example
  remains a renderer/geometry restoration checkpoint rather than a hidden
  private fixture lookup.
- CMake for both examples should switch from `dart_build_gui_scene_example` to
  `dart_build_gui_example` and link `dart-io`. The Python runner defaults and
  runner tests should stop injecting `--scene` for these executables.
- This checkpoint intentionally leaves `atlas_puppet`, `hubo_puppet`,
  `atlas_simbicon`, and `g1_puppet` for the next robot/IK pass. The puppet
  examples need a public promoted way to pass IK handles/hotkeys into the
  viewer before their historical target-selection affordances can move out of
  the private `DartScene` fixture path without feature loss.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `operational_space_control`, `wam_ikfast`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - `pixi run ex operational_space_control --headless --frames 1
--screenshot ...`
  - `pixi run ex wam_ikfast --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
  - `rg -n "options\.defaultScene" examples` returns no matches.
  - `rg -n "dart_build_gui_scene_example" examples/operational_space_control
examples/wam_ikfast ...` returns no matches for the WAM pair.
  - Post-lint C++ GUI target rebuild for `operational_space_control`,
    `wam_ikfast`, and `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`

Eleventh source-ownership checkpoint:

- Restore `examples/atlas_simbicon` before the puppet examples. It still uses
  the macro scene launcher, but unlike Atlas/Hubo/G1 puppet it does not depend
  on private viewer IK-handle plumbing.
- Move Atlas Simbicon to a real `main.cpp` that loads the Atlas SDF through
  public `dart::io`, creates the ground and visual-only Atlas setup in source,
  and passes `options.world` through promoted `dart::gui`.
- Switch its CMake target from `dart_build_gui_scene_example(...)` to
  `dart_build_gui_example(...)` linked with `dart-io`, and remove the Python
  runner's injected `--scene atlas-simbicon` default.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `atlas_simbicon` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `atlas_simbicon`
  - `pixi run ex atlas_simbicon --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/atlas_simbicon/main.cpp` now
  owns the Atlas SDF load, visual-only collision/gravity disabling, Simbicon
  starting pose, ground, and status panel, and passes `options.world` through
  promoted `dart::gui`. Its CMake target no longer uses
  `dart_build_gui_scene_example(...)`, and the Python runner no longer injects
  `--scene atlas-simbicon`.
- Local evidence so far:
  - C++ GUI target build for `atlas_simbicon` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `atlas_simbicon`
  - `pixi run ex atlas_simbicon --headless --frames 1 --screenshot ...`
  - `rg -n "options\.defaultScene" examples` returns no matches.
  - `rg -n "dart_build_gui_scene_example" examples/atlas_simbicon ...`
    returns no matches for `atlas_simbicon`.
  - Full `examples` aggregate target build

Twelfth promoted-API checkpoint:

- Before migrating `atlas_puppet`, `hubo_puppet`, and `g1_puppet`, expose a
  renderer-neutral public `dart::gui::ApplicationOptions` handoff for IK target
  handles.
- The private Filament `DartScene::ikHandles` vector currently owns three
  user-visible behaviors: number-key selection, selected-target labels, and
  solve-on-drag/keyboard-nudge. Moving puppet examples to `options.world`
  without a public handoff would silently regress those behaviors.
- Add a public DART-owned handle type that contains only DART concepts:
  label, hotkey, target `SimpleFrame`, and `InverseKinematics` pointer. The
  backend should compute any renderable IDs internally; examples must not
  include Filament, GLFW, ImGui, or private backend headers.
- Map those public handles into the private runtime only when the application
  supplies `ApplicationOptions::world`; explicit `--scene ...` developer
  fixtures keep their existing private fixture handles.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `dart-gui` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`

Thirteenth source-ownership checkpoint:

- Restore `examples/atlas_puppet` now that promoted `ApplicationOptions`
  carries IK handles.
- Move the Atlas URDF load, start pose, ground, visible root handle, four IK
  targets, support-foot geometry, hotkey labels, and solver configuration into
  `examples/atlas_puppet/main.cpp`.
- The example should pass `options.world` and `options.ikHandles` through
  promoted `dart::gui`, with no private `DartScene`, no scene-string launcher,
  and no backend headers.
- Switch CMake to `dart_build_gui_example(...)` linked with `dart-io`, and
  remove the Python runner's injected `--scene atlas-puppet` default.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `atlas_puppet` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `atlas_puppet`
  - `pixi run ex atlas_puppet --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/atlas_puppet` now owns the
  Atlas URDF load, standing start pose, visual-only robot setup, ground,
  visible root handle, four selectable IK target frames, support-foot geometry,
  target hotkeys, solve-on-drag IK handles, and status panel in its own
  `main.cpp`. The executable now passes `options.world` and
  `options.ikHandles` through promoted `dart::gui`, and the Python runner no
  longer injects `--scene atlas-puppet`.
- `dart/gui/application.hpp` now includes the full DART dynamics types needed
  by the public IK-handle vector. The full examples aggregate build caught that
  this public API cannot rely on forward declarations when examples include
  only `<dart/gui/application.hpp>`.
- Local evidence so far:
  - C++ GUI target build for `atlas_puppet` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `atlas_puppet`
    (`/tmp/atlas_puppet_self_owned.ppm`, 2764816 bytes)
  - Pixi example-runner screenshot capture for
    `/tmp/atlas_puppet_pixi_self_owned.ppm` (921615 bytes)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused build for `atlas_puppet` and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner test:
    `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - `git diff --check`

Fourteenth source-ownership checkpoint:

- Restore the remaining IK puppet macro launchers, `examples/hubo_puppet` and
  `examples/g1_puppet`, as real public-API example sources.
- Move the Hubo DRCHubo URDF load, finger-body cleanup, start pose, DOF limit
  setup, ground, six IK targets, support-foot geometry, hotkeys, and status
  panel into `examples/hubo_puppet/main.cpp`.
- Move the G1 package/resource resolver, robot load, ground, four IK targets,
  support-foot geometry, hotkeys, package override parsing, and status panel
  into `examples/g1_puppet/main.cpp`.
- Both examples should pass `options.world` and `options.ikHandles` through
  promoted `dart::gui`, with no private `DartScene`, no scene-string launcher,
  and no backend headers.
- Switch both CMake files to `dart_build_gui_example(...)` linked with
  `dart-io`, and remove the Python runner's injected `--scene hubo-puppet` and
  `--scene g1` defaults.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `hubo_puppet`, `g1_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - `pixi run ex hubo_puppet --headless --frames 1 --screenshot ...`
  - `pixi run ex g1_puppet --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/hubo_puppet` now owns the
  DRCHubo URDF load, finger-body cleanup, standing start pose, visual-only
  robot setup, ground, six selectable IK target frames, support-foot geometry,
  target hotkeys, solve-on-drag IK handles, and status panel in its own
  `main.cpp`. `examples/g1_puppet` now owns the G1 package resource resolver,
  robot load, ground, four selectable IK target frames, support-foot geometry,
  target hotkeys, package/robot override parsing, solve-on-drag IK handles, and
  status panel in its own `main.cpp`. The Python runner no longer injects
  `--scene hubo-puppet` or `--scene g1` for either executable.
- Local evidence so far:
  - C++ GUI target build for `hubo_puppet`, `g1_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for `hubo_puppet`
    (`/tmp/hubo_puppet_self_owned.ppm`, 2764816 bytes)
  - Direct headless screenshot capture for `g1_puppet`
    (`/tmp/g1_puppet_self_owned.ppm`, 2764816 bytes)
  - Pixi example-runner screenshot capture for
    `/tmp/hubo_puppet_pixi_self_owned.ppm` (921615 bytes)
  - Pixi example-runner screenshot capture for
    `/tmp/g1_puppet_pixi_self_owned.ppm` (921615 bytes)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused build for `hubo_puppet`, `g1_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner test:
    `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - `git diff --check`
- Remaining GUI macro launchers after this slice: `empty`, `hardcoded_design`,
  `heightmap`, `lcp_physics`, `mimic_pendulums`, `point_cloud`,
  `polyhedron_visual`, `simulation_event_handler`, `soft_bodies`, and
  `vehicle`.

Fifteenth source-ownership checkpoint:

- Restore the static geometry/visual macro launchers as real public-API
  example sources: `hardcoded_design`, `heightmap`, `point_cloud`, and
  `polyhedron_visual`.
- Move the hardcoded revolute-chain construction into
  `examples/hardcoded_design/main.cpp`.
- Move the authored height field, reference box, and sample marker layout into
  `examples/heightmap/main.cpp`.
- Move the point-cloud shape, per-point colors, optional voxel-grid fixture,
  and sensor marker frame into `examples/point_cloud/main.cpp`.
- Move the convex polyhedron mesh and wireframe line-segment shape into
  `examples/polyhedron_visual/main.cpp`.
- Each example should pass `options.world` through promoted `dart::gui`, with
  no private `DartScene`, no scene-string launcher, and no backend headers.
- Switch each CMake file to `dart_build_gui_example(...)`, and remove the
  Python runner's injected scene defaults for these four executables.
- Local acceptance for this checkpoint:
  - C++ GUI target build for the four examples and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for each example
  - `pixi run ex <example> --headless --frames 1 --screenshot ...` for each
    example
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/hardcoded_design`,
  `examples/heightmap`, `examples/point_cloud`, and
  `examples/polyhedron_visual` now own their visual DART worlds in
  per-example `main.cpp` files, call promoted
  `dart::gui::runApplication(argc, argv, options)`, and no longer rely on
  private scene strings from CMake or the Python runner.
- Local evidence so far:
  - C++ GUI target build for `hardcoded_design`, `heightmap`, `point_cloud`,
    `polyhedron_visual`, and `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot captures for all four examples
  - `pixi run ex <example> --headless --frames 1 --screenshot ...` captures
    for all four examples
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint C++ GUI target rebuild for `hardcoded_design`, `heightmap`,
    `point_cloud`, `polyhedron_visual`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner test:
    `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - `git diff --check`

User correction during the fifteenth checkpoint:

- Do not treat the remaining `dart_build_gui_scene_example(...)` list as the
  complete restoration backlog. It is only the source-ownership backlog.
- Examples such as `examples/fetch/` can already have their own `main.cpp` and
  still require parity scrutiny against the historical example behavior.
- Current Fetch status: `examples/fetch/main.cpp` is source-owned, uses
  promoted `dart::gui`, preserves the MJCF load, Bullet preference when
  available, live mocap target following, visible target-bar affordance, and
  has a focused parity-marker guard. Keep it in the final example-parity audit
  rather than assuming "source-owned" means "fully restored".
- After this static geometry checkpoint lands, the source-ownership backlog is
  expected to be only `empty`, `lcp_physics`, `mimic_pendulums`,
  `simulation_event_handler`, `soft_bodies`, and `vehicle`, but the full
  restored-example acceptance still includes rechecking source-owned examples
  such as `fetch` for missing controls, visual affordances, camera defaults,
  and behavior parity.
- Immediate Fetch recheck on the current pushed head rendered both direct and
  pixi-runner screenshots:
  - Direct output: `/tmp/fetch_parity_recheck.ppm`
  - Pixi runner output: `/tmp/fetch_pixi_parity_recheck.ppm`

Sixteenth source-ownership checkpoint:

- Restore the LCP/mimic dynamics pair as real public-API example sources:
  `lcp_physics` and `mimic_pendulums`.
- Move the LCP physics contact stack, mass-ratio boxes, domino chain,
  ball-drop spheres, ground, timestep, gravity, and Dantzig solver selection
  into `examples/lcp_physics/main.cpp`.
- Move the mimic pendulum SDF load, ground rename/visual ground, per-rig color
  setup, and mimic-world gravity into `examples/mimic_pendulums/main.cpp`.
- Keep both examples on promoted `dart::gui::ApplicationOptions`, with no
  private `DartScene`, no scene-string launcher, and no backend headers. Add
  small public panels only for status/play-step controls and example context.
- Switch both CMake files to `dart_build_gui_example(...)`; link
  `mimic_pendulums` with `dart-io`; remove the Python runner's injected
  `--scene lcp-physics` and `--scene mimic-pendulums` defaults.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `lcp_physics`, `mimic_pendulums`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - `pixi run ex lcp_physics --headless --frames 1 --screenshot ...`
  - `pixi run ex mimic_pendulums --headless --frames 1 --screenshot ...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/lcp_physics/main.cpp` now
  owns the contact solver benchmark world, including the Dantzig LCP solver,
  ground, mass-ratio boxes, box stack, domino chain, ball-drop spheres, and
  status panel. `examples/mimic_pendulums/main.cpp` now owns the SDF load,
  imported/visual ground setup, mimic-rig color coding, gravity, and status
  panel. The Python runner no longer injects `--scene` defaults for either
  executable.
- Local evidence so far:
  - C++ GUI target build for `lcp_physics`, `mimic_pendulums`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - Sequential pixi-runner screenshot capture for `lcp_physics`
  - Sequential pixi-runner screenshot capture for `mimic_pendulums`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint C++ GUI target rebuild for `lcp_physics`,
    `mimic_pendulums`, and `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner test:
    `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - `git diff --check`
- Remaining GUI macro launchers after this slice: `empty`,
  `simulation_event_handler`, `soft_bodies`, and `vehicle`.

Seventeenth source-ownership checkpoint:

- Restore the interaction/event examples as real public-API sources: `empty`
  and `simulation_event_handler`.
- Move the historical "empty viewer" draggable frame scaffold into
  `examples/empty/main.cpp`: an anchor frame, child draggable frame, and axis
  marker frames, all using public `SimpleFrame` and `dart::gui` APIs.
- Move the simulation-event example's world, dynamic boxes/sphere, blinking
  sensor marker frames, custom `dart::sensor::Sensor`, timestep, gravity, and
  status panel into `examples/simulation_event_handler/main.cpp`.
- Keep both examples on promoted `dart::gui::ApplicationOptions`, with no
  private `DartScene`, no scene-string launcher, and no backend headers. The
  replacement for old raw OSG key handlers is a small public GUI panel plus
  standard viewer selection/drag controls.
- Switch both CMake files to `dart_build_gui_example(...)`, and remove the
  Python runner's injected `--scene drag-and-drop` and
  `--scene simulation-event-handler` defaults.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `empty`, `simulation_event_handler`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - `pixi run ex empty --headless --frames 1 --screenshot ...`
  - `pixi run ex simulation_event_handler --headless --frames 1 --screenshot
...`
  - Full `examples` aggregate target build
  - `pixi run lint`
- Implementation state for this slice: `examples/empty/main.cpp` now owns the
  selectable anchor frame, child draggable frame, axis marker frames, and
  minimal status panel. `examples/simulation_event_handler/main.cpp` now owns
  the dynamic boxes/sphere, blinking sensor marker frames, custom
  `dart::sensor::Sensor` implementation, gravity/timestep setup, and status
  panel. The Python runner no longer injects `--scene` defaults for either
  executable.
- Local evidence so far:
  - C++ GUI target build for `empty`, `simulation_event_handler`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - Sequential pixi-runner screenshot capture for `empty`
  - Sequential pixi-runner screenshot capture for `simulation_event_handler`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused C++ GUI target build for `empty`,
    `simulation_event_handler`, and `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`66 passed`)
  - `git diff --check`
- Remaining GUI macro launchers after this slice: `soft_bodies` and `vehicle`.

Eighteenth source-ownership checkpoint:

- Restore the final GUI scene macro launchers as real public-API sources:
  `soft_bodies` and `vehicle`.
- Move `soft_bodies` back to `examples/soft_bodies/main.cpp` by loading
  `dart://sample/skel/softBodies.skel` directly, owning the `World`, and
  replacing the old OSG recording key handler with a public `dart::gui::Panel`
  that records, rewinds, advances, restarts, and jumps to the latest captured
  soft-body state.
- Move `vehicle` back to `examples/vehicle/main.cpp` by loading
  `dart://sample/skel/vehicle.skel` directly, preserving the historical
  gravity, car/ground/obstacle naming and colors, and replacing the old OSG
  keyboard handler with a public `dart::gui::Panel` plus `options.preStep`
  controller that applies wheel velocities and steering forces to the vehicle
  skeleton.
- Switch both CMake files to `dart_build_gui_example(...)`, and remove the
  Python runner's injected `--scene vehicle` and `--scene soft-bodies`
  defaults.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `soft_bodies`, `vehicle`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Direct headless screenshot capture for both examples
  - Sequential pixi-runner screenshot capture for both examples
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Implementation state for this slice: `examples/soft_bodies/main.cpp` now owns
  `softBodies.skel`, `SoftBodyHistory`, state capture/restore, and the public
  playback panel. `examples/vehicle/main.cpp` now owns `vehicle.skel`, gravity,
  visual naming/color parity, a public panel, and an `options.preStep`
  controller for wheel/steering forces. The Python runner no longer injects
  scene defaults for either executable.
- Local evidence so far:
  - C++ GUI target build for `soft_bodies`, `vehicle`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (`66 passed`)
  - Direct headless screenshot capture for `soft_bodies` and `vehicle`
    (`2764816` bytes each)
  - Sequential pixi-runner screenshot capture for `soft_bodies` and `vehicle`
    (`921615` bytes each)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused C++ GUI target build for `soft_bodies`, `vehicle`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`66 passed`)
  - `git diff --check`
- Remaining GUI macro launchers after this slice: none. The source-ownership
  phase still requires a final parity audit of all restored examples and
  follow-up cleanup of now-unused fixture-only private scene plumbing.

Nineteenth source-ownership cleanup checkpoint:

- Remove the now-unused `dart_build_gui_scene_example(...)` helper from
  `examples/gui_scene_example.cmake` and delete `examples/gui_scene_launcher.cpp`.
  The public example helper should only build real sources; examples must not be
  able to route themselves through a private named-scene fixture again.
- Add/extend audit coverage so the source-ownership state is explicit:
  `examples/` should have no `dart_build_gui_scene_example(...)` references,
  `scripts/run_cpp_example.py` should not inject per-example `--scene` defaults,
  and every restored GUI example should be represented in the promoted
  `dart::gui` boundary scan.
- Keep `dartsim --scene ...` private fixture paths only as transitional app
  diagnostics for now; this cleanup checkpoint removes the example build
  escape hatch first. Physical app scene-fixture pruning belongs to the next
  checkpoint because it changes `dartsim --scene all` smoke coverage and CI
  registration.
- Local acceptance for this checkpoint:
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Implementation state for this slice: `examples/gui_scene_launcher.cpp` is
  deleted, `examples/gui_scene_example.cmake` only exposes
  `dart_build_gui_example(...)`, the C++ GUI unit test rejects the old
  scene-launcher shim, and the Python runner tests reject per-example `--scene`
  defaults.
- Local evidence so far:
  - Full `examples` aggregate target build plus
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (`67 passed`)
  - `pixi run lint`
  - Post-lint full `examples` aggregate target build plus
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`67 passed`)
  - `git diff --check`

Twentieth official-target checkpoint:

- Rename the internal GUI extraction/rendering CMake target and component away
  from `dart-gui-experimental` / `gui-experimental` to `dart-gui-core` /
  `gui-core`.
- Keep `dart/gui/experimental/*.hpp` compatibility headers and the
  `dart/gui/experimental/detail/filament` file layout in this checkpoint. Those
  names are source-layout and compatibility debt; this step removes the build
  target/component branding that downstream CMake consumers and tests see.
- Retarget `dart-gui`, dartpy GUI bindings, and
  `UNIT_gui_FilamentSceneExtraction` to the official/core target names.
- Extend audit coverage so promoted example and public GUI surfaces keep
  rejecting `dart-gui-experimental` references, and the test target no longer
  links to an experimental GUI library.
- Local acceptance for this checkpoint:
  - CMake configure/build for `dart-gui`, `dart-gui-core`, `dartpy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
  - `pixi run lint`
  - `git diff --check`
- Implementation state for this slice: the core GUI implementation target is
  now `dart-gui-core` with component `gui-core`; `dart-gui`, dartpy, and
  `UNIT_gui_FilamentSceneExtraction` no longer link against
  `dart-gui-experimental`; audit coverage rejects the old target/component name
  from active GUI CMake files.
- Local evidence so far:
  - CMake configure/build for `dart-gui-core`, `dart-gui`, `dartpy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Full `examples` aggregate target build
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (`67 passed`)
  - Build-tree dartpy GUI compatibility import
    (`dartpy.gui.experimental.ShapeKind.Box`)
  - `pixi run lint`
  - Post-lint CMake configure/build for `dart-gui-core`, `dart-gui`, `dartpy`,
    and `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`67 passed`)
  - Post-lint build-tree dartpy GUI compatibility import
    (`dartpy.gui.experimental.ShapeKind.Box`)
  - `git diff --check`

Twenty-first parity-audit correction checkpoint:

- Maintainer correction: many examples are source-owned but still not fully
  restored to the historical GUI behavior; `examples/fetch/` is the concrete
  proof case. Do not treat "has a real `examples/<name>/main.cpp`" as
  completion.
- New operating rule for the remaining branch work: every pre-existing
  user-facing GUI example remains **parity pending** until its legacy source
  has a feature inventory and the migrated `dart::gui` source either preserves
  each user-visible behavior or records an explicit, justified follow-up gap.
- Use the historical OSG source immediately before renderer removal as the
  baseline when available, for example
  `git show 520993d7301^:examples/<name>/main.cpp`. Compare the restored
  source against that baseline before claiming parity.
- Audit dimensions for each example:
  - world/model loading and initial configuration
  - collision-engine preferences and required optional components
  - simulation-loop callbacks/controllers
  - interactive frames, drag targets, hotkeys, reset/step/play controls, and
    visible affordances
  - panels/help/status text and `--gui-scale` behavior
  - camera defaults, grid/axis/debug visuals, and screenshot/headless behavior
  - runner/CMake wiring through public `dart::gui`, with no private fixture
    launcher or backend/experimental headers
- Immediate `fetch` repair scope:
  - Keep the already-restored public source ownership, Bullet preference when
    available, live mocap target following, and green target bars.
  - Restore remaining historical user experience where the current public API
    can support it: camera home framing, clearer target/status/help panel
    content, and a stronger visible target affordance.
  - If a behavior requires a missing promoted API, add the smallest
    renderer-neutral `dart::gui` API needed for the example instead of using
    private Filament/GLFW/ImGui types from the example.
- Broader parity audit remains active after `fetch`; all already migrated
  robot/IK, interaction, shape, mesh, capture, and controller examples need the
  same legacy-source comparison before this dev task can be considered done.
- Implementation state for this slice: `dart::gui::ApplicationOptions` now has
  a renderer-neutral optional `OrbitCamera` override. `examples/fetch` uses it
  to restore the historical camera home framing from the OSG viewer, keeps the
  example on public `dart::gui` APIs, and replaces the separate child target
  bars with one `LineSegmentShape` cross on the draggable target frame so
  selecting the visible cross moves the mocap target rather than detaching a
  child marker from it.
- Local evidence so far:
  - C++ GUI target build for `dart-gui`, `fetch`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct Fetch headless capture with project-equivalent llvmpipe environment
    (`LIBGL_ALWAYS_SOFTWARE=1`, `MESA_LOADER_DRIVER_OVERRIDE=llvmpipe`) and
    the headless analyzer in basic mode
  - `pixi run ex fetch --headless --frames 10 --screenshot ...` capture and
    visual inspection of the rendered image
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (`67 passed`)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused C++ GUI target build for `dart-gui`, `fetch`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`67 passed`)
  - Post-lint project-runner Fetch headless capture and direct llvmpipe Fetch
    headless capture with analyzer coverage
  - `git diff --check`
- Local environment note: a direct system-Filament headless Fetch run returned
  an all-zero PPM on this workstation, while the project runner's llvmpipe path
  rendered correctly. Treat the project runner environment as the relevant
  local headless validation path for this checkpoint.

Twenty-second robot/IK parity-audit checkpoint:

- The next slice starts from the robot and IK examples because they expose the
  largest difference between "restored source ownership" and historical GUI
  behavior. The first audited examples are `examples/g1_puppet/`,
  `examples/atlas_puppet/`, and `examples/hubo_puppet/`, followed by
  `examples/operational_space_control/`, `examples/wam_ikfast/`, and
  `examples/tinkertoy/`.
- Initial legacy-source comparison shows that the migrated puppet examples are
  runnable but still miss important OSG-era user-visible behavior:
  target-frame affordances, key-driven target activation/deactivation,
  WASD/Q/E/F/Z target teleoperation, whole-body/per-step IK solving behavior,
  relaxed-posture or balance objectives where the historical example had them,
  and example-specific camera/help text polish.
- Immediate implementation scope for this checkpoint is deliberately narrow:
  restore the visible target-handle affordance and the public `dart::gui`
  wiring needed by the puppet examples without exposing Filament, GLFW, Dear
  ImGui, or private fixture APIs. Larger solver-behavior parity such as Hubo
  analytical IK, Atlas relaxed-posture objectives, and full OSG hotkey parity
  remains tracked as explicit follow-up until it is implemented.
- Do not claim full parity for a robot/IK example until its historical source
  has been inventoried and each visible behavior is either restored or recorded
  as a justified remaining gap.
- Implementation state for this slice: `g1_puppet`, `atlas_puppet`, and
  `hubo_puppet` now use line-segment target handles on the draggable
  `SimpleFrame` targets instead of small spheres, matching the Fetch repair's
  "visible affordance is the thing being dragged" pattern. Their panels now
  document the promoted selection controls: number-key target selection,
  Ctrl-left drag, arrow/PageUp/PageDown nudging, and X/Y/Z constrained drag.
  Atlas also restores the historical comfortable arm start pose and gives the
  hand IK targets unconstrained linear/angular bounds.
- Remaining explicit gaps after this slice: Atlas relaxed posture and balance
  objective parity, Hubo analytical IK and relaxed-posture/balance parity,
  legacy target activation/deactivation semantics, and full OSG
  WASD/Q/E/F/Z-style teleoperation. These should be restored through
  renderer-neutral `dart::gui` concepts before declaring those examples fully
  restored.
- Local evidence so far:
  - C++ GUI target build for `g1_puppet`, `atlas_puppet`, `hubo_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - `pixi run ex g1_puppet --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex atlas_puppet --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex hubo_puppet --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - Basic analyzer checks for all three 640x480 screenshots
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (`67 passed`)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused C++ GUI target build for `g1_puppet`, `atlas_puppet`,
    `hubo_puppet`, and `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint Python runner tests (`67 passed`)
  - Post-lint direct llvmpipe headless captures for all three puppet examples
    plus basic analyzer checks for their 640x480 screenshots
  - Post-lint full `examples` aggregate target build

Twenty-third target-affordance parity checkpoint:

- Continue the target-handle audit across the remaining task-space and
  construction examples that expose a movable target frame:
  `operational_space_control`, `wam_ikfast`, and `tinkertoy`.
- Historical `operational_space_control` used an OSG drag-and-drop target with
  explicit help text for constrained movement. The migrated source has the
  controller and pre-step callback restored but still renders the target as a
  small sphere. Replace that with the same line-segment target-handle affordance
  pattern and document the promoted Ctrl-drag/X/Y/Z controls.
- `tinkertoy` still has broader builder parity gaps from the OSG example:
  add/delete block hotkeys, picking semantics, force coefficient controls, and
  recording toggles. This checkpoint only addresses the visible movable target
  affordance and panel guidance; builder workflow parity remains a follow-up.
- `wam_ikfast` is a current source-owned visual target scene. Keep it aligned
  with the promoted target affordance even though the exact historical baseline
  is the branch-local WAM restoration rather than the pre-OSG-removal source.
- Implementation state for this slice: `operational_space_control`,
  `wam_ikfast`, and `tinkertoy` now render their user-movable target frames as
  line-segment handles instead of small spheres. Their panels document the
  promoted target movement controls: Ctrl-left drag, arrow/PageUp/PageDown
  nudging, and X/Y/Z constrained drag.
- Remaining explicit gaps after this slice: `tinkertoy` still needs builder
  workflow parity for add/delete block hotkeys, picking semantics, force
  coefficient controls, target reorientation, and recording toggles. Those
  should be restored through promoted `dart::gui` input/panel concepts rather
  than by reintroducing OSG handlers.
- Local evidence so far:
  - C++ GUI target build for `operational_space_control`, `wam_ikfast`,
    `tinkertoy`, and `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Python runner tests (`67 passed`)
  - `pixi run ex operational_space_control --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex wam_ikfast --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - Basic analyzer checks for all three 640x480 screenshots
  - Full `examples` aggregate target build

Twenty-fourth Fetch work-area grid parity checkpoint:

- Follow-up maintainer correction: `examples/fetch/` must stay in the parity
  audit set, not only in the completed source-ownership set. The prior Fetch
  repair restored the public source, Bullet preference, live mocap target,
  camera framing, and draggable target bars; the historical source also added
  a visible `GridVisual` offset at the Fetch pick-and-place work area.
- Current gap: the promoted `dart::gui` path has a generic debug grid, but the
  Fetch example source does not recreate the old example-specific grid anchor
  at `(1.3, 0.75, 0.0)`. That makes the source less faithful to the historical
  user-facing scene even though the robot/object/target behavior is present.
- Implementation state for this slice: `examples/fetch/main.cpp` now adds a
  source-owned, non-dynamic line-segment grid as ordinary DART world geometry
  on a weld-joint skeleton. This keeps the example on public `dart::gui` and
  `dart::dynamics` APIs, avoids private Filament debug-overlay controls, and
  gives headless screenshots the same visible work-area reference as the old
  OSG example.
- Guard state: the Fetch source marker test now requires the grid skeleton
  name, public `WeldJoint`/`LineSegmentShape` construction, and the historical
  grid offset.
- Local evidence so far:
  - Focused C++ target build for `fetch` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct llvmpipe `fetch --headless --frames 10 --width 640 --height 480 --screenshot ...`
  - `pixi run ex fetch --headless --frames 10 --width 640 --height 480 --screenshot ...`
  - Basic analyzer checks for both Fetch screenshots
  - Python runner tests (`67 passed`)
  - Full `examples` aggregate target build

Twenty-fifth Tinkertoy builder-state parity checkpoint:

- Historical baseline: the OSG Tinkertoy example kept a stateful picked block,
  created blocks dynamically from the selected body and target frame, applied a
  capped spring-like external force from the picked point to the target,
  rendered a force line, changed colors for paused/simulating/selected states,
  exposed gravity and force-coefficient controls, and provided add/delete,
  clear, and target-reorientation actions in both the ImGui widget and
  keyboard handler.
- Implementation state for this checkpoint: `examples/tinkertoy/main.cpp` now
  has a local `TinkertoyState`, keeps the implementation on public DART and
  `dart::gui` APIs, and restores the builder workflow through
  renderer-neutral panel controls. Users can select a block through the
  promoted scene selection label, then use panel buttons to add
  Weld/Revolute/Ball blocks, delete/clear the pick, reorient the target, toggle
  gravity, and adjust force coefficient. The example restores per-step force
  application and dynamic force-line updates through
  `ApplicationOptions::preStep`/panel context.
- The restored block construction now uses `VisualAspect`, `CollisionAspect`,
  and `DynamicsAspect`, so dynamically added Tinkertoy bodies participate in
  physics instead of being visual-only geometry.
- Explicit follow-up gap: raw keyboard parity (`1`/`2`/`3`,
  Backspace, Delete, Up/Down, backtick, Tab) and legacy Enter recording toggle
  require a renderer-neutral public input/action API. Do not wire the example
  to private GLFW or Filament input handlers to close that gap.
- Guard state: the Tinkertoy source marker test now requires stateful builder
  markers (`TinkertoyState`, `addWeldJointBlock`, `deletePick`,
  `setGravityEnabled`, `setForceCoeff`, `options.preStep`) and restored
  `CollisionAspect`/`DynamicsAspect` block construction.
- Local evidence so far:
  - Focused C++ target build for `tinkertoy` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct llvmpipe `tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - Basic analyzer checks for both Tinkertoy screenshots
  - Python runner tests (`67 passed`)
  - Full `examples` aggregate target build

Twenty-sixth renderer-neutral keyboard-action checkpoint:

- Motivation: Tinkertoy still has raw keyboard parity gaps (`1`/`2`/`3`,
  Backspace, Delete, Up/Down, backtick, Tab), and the robot/IK examples still
  need promoted hotkey/teleoperation behavior. The examples must not include
  GLFW, Filament, or other backend-private input headers to close those gaps.
- Implementation state: `dart::gui::ApplicationOptions` now has a small
  renderer-neutral keyboard action surface: printable-character shortcuts plus
  named non-character keys, edge-triggered by default, with callbacks receiving
  a public action context containing the `ViewerLifecycleState`. The backend
  maps those public shortcuts to GLFW internally; examples still do not include
  private GLFW or Filament input headers.
- First consumer state: Tinkertoy now registers historical keyboard actions
  that do not require camera reset or recording APIs: `1`/`2`/`3` add
  Weld/Revolute/Ball blocks, Backspace clears the pick, Delete deletes the
  picked subtree, Up/Down adjust force coefficient, and backtick reorients the
  target. Tab camera-home and Enter recording toggle remain explicit follow-up
  gaps until public camera-reset and recording APIs exist.
- Guard state: unit/source markers cover `KeyboardShortcut`, `KeyboardAction`,
  Tinkertoy `options.keyboardActions`, and the restored non-renderer-private
  hotkey registrations.
- Local evidence so far:
  - Focused C++ target build for `dart-gui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct llvmpipe `tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - `pixi run ex tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
  - Basic analyzer checks for both Tinkertoy screenshots
  - Python runner tests (`67 passed`)
  - Full `examples` aggregate target build

Twenty-seventh robot/IK behavior parity checkpoint:

- Source ownership is not the remaining blocker for the legacy example set:
  an audit against `origin/main:examples` found that every legacy example
  directory with `main.cpp` has a current `main.cpp`, except intentionally
  removed backend-only renderer demos (`filament_gui` and `raylib`).
  `wam_ikfast` is branch-new. This does not mean feature parity is complete.
- Continue comparing source-owned examples against the historical source
  baseline before marking them restored. `examples/fetch/` remains the model
  correction: it was source-owned before camera, target-cross, and work-area
  grid parity were restored.
- Immediate implementation scope: use the new public
  `ApplicationOptions::keyboardActions` surface to restore the Atlas/Hubo
  whole-body puppet behavior that can be expressed without exposing GLFW or
  private Filament APIs:
  - continuous IK solving from the example-owned world state rather than only
    solving on backend drag/nudge operations
  - WASD planar root translation, Q/E yaw rotation, and F/Z vertical root
    translation
  - compact panel text documenting the restored teleoperation controls
- Keep this checkpoint focused. Atlas relaxed-posture/balance optimization,
  Hubo analytical IK, target activation/deactivation semantics, G1 target
  toggle semantics, and recording/camera-reset shortcuts remain explicit
  parity gaps unless the slice adds the smallest renderer-neutral public API
  needed to implement them cleanly.
- Implementation state for this slice: `examples/atlas_puppet` and
  `examples/hubo_puppet` now register repeatable public
  `dart::gui::KeyboardAction` callbacks for W/A/S/D/F/Z/Q/E root movement.
  Each callback applies the same root-frame step sizes as the historical
  puppet teleoperation path, solves the example-owned IK handles immediately,
  and pauses the viewer through the public lifecycle context. Both examples
  also solve their IK handles from `ApplicationOptions::preStep` while the
  simulation advances, so IK behavior is no longer limited to backend drag or
  nudge events.
- Guard state: the GUI source-marker test now requires Atlas/Hubo
  `options.preStep`, `options.keyboardActions`, root teleoperation helper
  markers, IK-solve helper markers, and panel text for the restored WASD/QE/FZ
  controls.
- Local evidence so far:
  - Focused C++ target build for `atlas_puppet`, `hubo_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct llvmpipe headless captures for `atlas_puppet` and `hubo_puppet`
    with basic analyzer checks at 640x480
  - Sequential `pixi run ex ... --headless --frames 2 --width 640 --height
480 --screenshot ...` captures for `atlas_puppet` and `hubo_puppet` with
    basic analyzer checks
  - Python runner tests (`67 passed`)
  - Full `examples` aggregate target build
  - `pixi run lint`
  - Post-lint focused C++ target build for `atlas_puppet`, `hubo_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Post-lint focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Post-lint direct llvmpipe headless captures for `atlas_puppet` and
    `hubo_puppet` with basic analyzer checks at 640x480
  - Post-lint Python runner tests (`67 passed`)
  - Post-lint full `examples` aggregate target build
  - `git diff --check`
- Local acceptance for this checkpoint:
  - C++ GUI target build for `atlas_puppet`, `hubo_puppet`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct or pixi headless screenshots for `atlas_puppet` and `hubo_puppet`
    with basic analyzer checks
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`

Twenty-eighth camera-home keyboard-action checkpoint:

- Tinkertoy still has an explicit keyboard parity gap from the historical OSG
  example: Tab should return the camera to the example's home framing. The
  existing `ApplicationOptions::camera` hook sets the initial camera, but
  keyboard actions currently cannot request a runtime camera reset.
- Add the smallest renderer-neutral public handoff needed by examples:
  `KeyboardActionContext` should expose a camera-home/reset callback supplied
  by the backend. The callback resets the public `OrbitCameraController` to the
  application home camera; it must not expose GLFW, Filament, ImGui, native
  window handles, or backend camera objects through public headers.
- Use that callback in `examples/tinkertoy` to restore the Tab camera-home
  hotkey through `ApplicationOptions::keyboardActions`.
- Keep Enter recording as a separate explicit gap. Runtime recording needs a
  public capture/session API, not only a camera reset callback.
- Implementation state for this slice: `KeyboardActionContext` now carries an
  optional `resetCamera` callback. The private Filament input bridge binds it
  to the application home `OrbitCamera` and resets orbit tracking internally,
  so public examples can request camera home without seeing backend camera or
  window objects. `examples/tinkertoy` registers Tab as a keyboard action and
  documents the restored hotkey in its builder panel.
- Guard state: unit/source markers cover the public `resetCamera` context
  callback and Tinkertoy's `KeyboardKey::Tab` binding.
- Local evidence so far:
  - Focused C++ target build for `dart-gui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct llvmpipe Tinkertoy headless capture with basic analyzer coverage at
    640x480
  - `pixi run ex tinkertoy --headless --frames 2 --width 640 --height 480 --screenshot ...`
    with basic analyzer coverage
  - Python runner tests (`67 passed`)
  - `git diff --check`
  - Full `examples` aggregate target build
- Local acceptance for this checkpoint:
  - C++ GUI target build for `dart-gui`, `tinkertoy`, and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi Tinkertoy headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`

Twenty-ninth G1 target activation parity checkpoint:

- Historical `examples/g1_puppet` used number keys as target
  activation/deactivation toggles, not only as selection shortcuts. The
  migrated source currently owns the G1 scene and visible target handles, but
  all targets are present from startup and IK solving is limited to backend
  drag/nudge operations.
- Immediate implementation scope: restore G1's number-key target toggle
  semantics using existing public DART and `dart::gui` APIs. Keep a local
  source-owned handle state that owns the target `SimpleFrame`, the
  `InverseKinematics` pointer, and the world membership flag. Number-key
  keyboard actions should add/remove the target frame from the world, reset an
  activated target to its end-effector transform, and solve only active
  targets from `ApplicationOptions::preStep`.
- Keep this checkpoint focused on G1. Atlas/Hubo target activation semantics,
  Atlas relaxed-posture/balance optimization, Hubo analytical IK, and Enter
  recording remain explicit follow-up gaps.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `g1_puppet` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi G1 headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `pixi run lint`
  - `cmake --build build/default/cpp/Release --target g1_puppet
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe G1 screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_g1_toggle_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex g1_puppet --headless --frames 2 --width 640 --height 480
--screenshot /tmp/dart_g1_toggle_pixi.ppm` plus analyzer
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `git diff --check`
- Implementation status: pushed as
  `1db37cf8275 Restore G1 target toggles`.

Thirtieth source-owned example restoration checkpoint:

- The user explicitly called out that many examples are still not fully
  restored, including `examples/fetch/`. Treat this as a source-owned example
  parity gap, not a renderer backend smoke-test gap.
- Previous Fetch evidence covered camera framing, target-cross visibility,
  work-area grid visibility, successful build, and headless image output. That
  does not by itself prove the historical Fetch example is fully restored in
  public `dart::gui` source.
- Immediate implementation scope after the G1 checkpoint: re-audit
  `examples/fetch/` against its historical OSG source and move any remaining
  example behavior, UI controls, display text, and interaction semantics into
  the maintained Fetch source through promoted `dart::gui` APIs.
- Re-audit result for the first Fetch slice: the maintained source already
  restores the live MJCF simulation, Bullet preference, initial robot/object
  poses, weld reset, mocap target following, visible draggable target bars,
  work-area grid, `--gui-scale`, and camera default. The remaining user-facing
  gap in this slice is the legacy viewer panel affordance set: Exit,
  Help/About text, and explicit Play/Pause controls instead of only
  Pause/Step/status text.
- Implementation scope for this Fetch checkpoint: add a narrow public
  `dart::gui` lifecycle exit request helper and use it from the Fetch panel;
  update the Fetch panel to expose the historical Play/Pause controls and
  viewer-help/about copy through existing public panel widgets. Do not expose
  Filament, GLFW, Dear ImGui, or backend window types to the example.
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target fetch
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe Fetch screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_fetch_panel_direct.ppm`, 303695/307200 nonzero pixels)
  - `pixi run ex fetch --headless --frames 2 --width 640 --height 480
--screenshot /tmp/dart_fetch_panel_pixi.ppm` plus analyzer
    (303694/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target lcp_physics
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe LCP physics screenshot with `--scenario dominos
--solver pgs` plus `analyze_headless_smoke.py`
    (`/tmp/dart_lcp_physics_direct_postlint.ppm`, 921600/921600 nonzero
    pixels)
- Implementation status: pushed as
  `619af5649bc Restore LCP physics controls`.

Forty-second mimic-pendulums parity checkpoint:

- Historical `examples/mimic_pendulums` loaded the SDF mimic pendulum world,
  retargeted mimic references to the uncoupled baseline rig, colored the rigs,
  exposed solver/collision controls, showed mimic pair error and base-drift
  diagnostics, used a 1280x720 viewer, set camera home from eye `(8, -7, 4)` to
  target `(0.5, 0, 1.5)`, and had an example README.
- Current promoted source loads and colors the visual rigs, but it does not
  retarget mimic joints to the baseline, does not expose solver/collision
  launch controls, does not display mimic error/base-drift diagnostics, and
  lacks README plus camera/run defaults.
- Scope before code changes: restore baseline retargeting, pair collection,
  readable metric rows in the renderer-neutral panel, local `--solver` and
  `--collision` launch flags, camera/run defaults, README, tests, and
  changelog through promoted `dart::gui`; keep detailed ImGui tables and
  in-window solver/collision mutation as public control-surface follow-up.
- Implementation status: mimic-pendulums now restores baseline retargeting,
  mimic-pair collection, panel diagnostics for pair error and base drift,
  launch-time `--solver` and `--collision` flags, camera/run defaults, and the
  example README through promoted `dart::gui`.
- Local validation completed so far:
  - `cmake --build build/default/cpp/Release --target mimic_pendulums
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe `mimic_pendulums --solver pgs --collision dart
--headless --frames 2 --screenshot /tmp/dart_mimic_pendulums_direct.ppm`
    plus `analyze_headless_smoke.py` (921600/921600 nonzero pixels). The
    forced DART collision-detector path reports expected unsupported-cylinder
    diagnostics while exiting successfully.
  - `pixi run ex mimic_pendulums --headless --frames 2 --screenshot
/tmp/dart_mimic_pendulums_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels at the pixi wrapper's 640x480 size)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target
mimic_pendulums UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe `mimic_pendulums --solver pgs --collision dart
--headless --frames 2 --screenshot
/tmp/dart_mimic_pendulums_direct_postlint.ppm` plus
    `analyze_headless_smoke.py` (921600/921600 nonzero pixels)
- Remaining before checkpoint handoff: commit and push without opening a PR.

Forty-third box-stacking parity checkpoint:

- Historical `examples/box_stacking` built a five-box stack on a floor, exposed
  Play/Pause, gravity, split-impulse, and LCP solver controls in its ImGui
  panel, printed viewer help/instructions, used an 800x640 launch size, set
  camera home from eye `(12, 12, 9)` to target `(0, 0, 2)`, and had an example
  README. It also carried sample custom world-node and OSG event-handler
  scaffolding that is not a DART-owned user-facing feature to preserve.
- Current promoted source owns the world, gravity control, split-impulse
  control, and pause/step panel through public `dart::gui`, but it lacks the
  README, solver selection, camera/run defaults, and viewer-help/about text.
- Scope before code changes: restore solver selection through renderer-neutral
  panel buttons plus launch-time `--solver`, keep split-impulse state when the
  solver changes, restore camera/run defaults through public
  `ApplicationOptions`, restore the README with promoted `dart::gui` wording,
  and update tests/changelog. Keep headlights, raw camera matrices, and the
  old OSG custom event-handler sample out of scope unless future public
  DART-owned concepts need them.
- Implementation status: box-stacking now restores launch-time `--solver`,
  renderer-neutral Dantzig/PGS panel buttons, split-impulse preservation across
  solver changes, 800x640 run defaults, historical camera home, and the example
  README through promoted `dart::gui`.
- Local validation completed so far:
  - `cmake --build build/default/cpp/Release --target box_stacking
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe `box_stacking --solver pgs --headless --frames 2
--screenshot /tmp/dart_box_stacking_direct.ppm` plus
    `analyze_headless_smoke.py` (512000/512000 nonzero pixels at the restored
    800x640 default)
  - `pixi run ex box_stacking --headless --frames 2 --screenshot
/tmp/dart_box_stacking_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels at the pixi wrapper's 640x480 size)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target box_stacking
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe `box_stacking --solver pgs --headless --frames 2
--screenshot /tmp/dart_box_stacking_direct_postlint.ppm` plus
    `analyze_headless_smoke.py` (512000/512000 nonzero pixels)
- Remaining before checkpoint handoff: commit and push without opening a PR.

Forty-fourth boxes parity checkpoint:

- Historical `examples/boxes` spawned a 5x5x5 grid of rigid boxes over a large
  ground plane, preferred Bullet collision when available, used a 1360x768
  launch size, set camera home from eye `(20, 20, 15)` to target `(0, 0, 3)`,
  printed simple viewer instructions, and had an example README.
- Current promoted source owns the box grid and ground plane through public
  `dart::gui`, but it lacks the Bullet preference, camera/run defaults, and
  README.
- Scope before code changes: restore Bullet preference behind `DART_HAVE_BULLET`,
  restore camera/run defaults through `ApplicationOptions`, restore the README
  with promoted `dart::gui` capture wording, and add source-extraction tests.
  Keep the old OSG shadow-map tuning and instruction-text plumbing out of
  scope because the promoted renderer owns those concepts differently.
- Implementation status: boxes now restores Bullet preference when available,
  1360x768 run defaults, historical camera home, and the example README through
  promoted `dart::gui`.
- Local validation completed so far:
  - `cmake --build build/default/cpp/Release --target boxes
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe `boxes --headless --frames 2 --screenshot
/tmp/dart_boxes_direct.ppm` plus `analyze_headless_smoke.py`
    (1044480/1044480 nonzero pixels at the restored 1360x768 default)
  - `pixi run ex boxes --headless --frames 2 --screenshot
/tmp/dart_boxes_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels at the pixi wrapper's 640x480 size)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target boxes
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe `boxes --headless --frames 2 --screenshot
/tmp/dart_boxes_direct_postlint.ppm` plus `analyze_headless_smoke.py`
    (1044480/1044480 nonzero pixels)
- Remaining before checkpoint handoff: commit and push without opening a PR.

Forty-fifth simple-frames parity checkpoint:

- Historical `examples/simple_frames` demonstrated a `SimpleFrame` hierarchy
  with box shapes, marker ellipsoids, and an arrow marker, used a 640x480
  launch size, set camera home from eye `(2, 1, 2)` to target `(0, 0, 0)`, and
  had an example README.
- Current promoted source owns the frame hierarchy, marker ellipsoids, and
  renderer-supported line-segment arrow through public `dart::gui`, but it
  lacks the README plus camera/run defaults.
- Scope before code changes: restore the 640x480 run default, historical camera
  home, README with promoted `dart::gui` wording, and source-extraction tests.
  Keep the old OSG `ArrowShape` implementation detail out of scope because the
  current public descriptor path already renders the arrow marker as line
  segments.
- Implementation status: simple-frames now restores 640x480 run defaults, the
  historical camera home, and the example README through promoted `dart::gui`.
- Local validation completed so far:
  - `cmake --build build/default/cpp/Release --target simple_frames
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe `simple_frames --headless --frames 2 --screenshot
/tmp/dart_simple_frames_direct.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels at the restored 640x480 default)
  - `pixi run ex simple_frames --headless --frames 2 --screenshot
/tmp/dart_simple_frames_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target simple_frames
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe `simple_frames --headless --frames 2
--screenshot /tmp/dart_simple_frames_direct_postlint.ppm` plus
    `analyze_headless_smoke.py` (307200/307200 nonzero pixels)
- Remaining before checkpoint handoff: commit and push without opening a PR.

Forty-first LCP physics parity checkpoint:

- Historical `examples/lcp_physics` exposed separate LCP benchmark scenarios
  (`mass_ratio`, `box_stack`, `ball_drop`, `dominos`, `inclined_plane`), solver
  selection (`dantzig` or `pgs`), `--list`, frame-capture-friendly headless
  defaults, 1280x720 launch size, camera home from eye `(3, 2, 3)` to target
  `(0, 0.3, 0)`, and a README documenting scenarios, CLI options, headless
  capture, and references.
- Current promoted source owns a renderer-neutral combined benchmark scene, but
  it dropped the README, local `--scenario`, `--solver`, and `--list` CLI
  surface, PGS solver selection, launch-size/camera defaults, and per-scenario
  explanatory state. The old in-window scenario/solver radio controls require a
  public mutable-world application API or richer renderer-neutral controls, so
  this checkpoint will not reintroduce private ImGui or OSG widget code.
- Scope before code changes: restore launch-time scenario selection, solver
  selection, `--list`, README, camera/run defaults, and panel text through
  promoted `dart::gui`; keep shared capture/window flags (`--headless`,
  `--frames`, `--out`, `--width`, `--height`, `--gui-scale`) under the common
  runner; update tests/changelog; record any remaining dynamic in-window
  switching as public API follow-up rather than backend-specific code.
- Implementation status: LCP physics now restores launch-time scenario
  selection, solver selection, list mode, camera/run defaults, panel context,
  and README through promoted `dart::gui`; dynamic in-window scenario/solver
  replacement remains a public API follow-up.
- Local acceptance completed so far for this checkpoint:
  - `cmake --build build/default/cpp/Release --target lcp_physics
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - `build/default/cpp/Release/bin/lcp_physics --list`
  - Direct llvmpipe `lcp_physics --scenario dominos --solver pgs --headless
--frames 2 --screenshot /tmp/dart_lcp_physics_direct.ppm` plus
    `analyze_headless_smoke.py` (921600/921600 nonzero pixels)
  - `pixi run ex lcp_physics --scenario box_stack --headless --frames 2
--screenshot /tmp/dart_lcp_physics_pixi.ppm` plus
    `analyze_headless_smoke.py` (307200/307200 nonzero pixels at the pixi
    wrapper's 640x480 size)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target fetch
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct and pixi Fetch screenshots plus
    `analyze_headless_smoke.py` (`/tmp/dart_fetch_panel_direct_postlint.ppm`
    and `/tmp/dart_fetch_panel_pixi_postlint.ppm`, both 303695/307200 nonzero
    pixels)
- Implementation status: pushed as
  `5939dbb1482 Restore Fetch panel controls`.
- After Fetch, continue the same source-owned audit pattern across the rest of
  the pre-existing examples before treating the migration as complete.

Thirty-first add/delete skeleton parity checkpoint:

- Historical `examples/add_delete_skels` exposed live `q`/`w` keyboard
  controls to spawn and delete cube skeletons, preferred Bullet collision when
  available, printed those controls as viewer instructions, and set an
  example-specific camera home view. The current promoted source owns the
  world and panel but only exposes panel buttons and does not wire the legacy
  keyboard actions or camera default through public `dart::gui`.
- Implementation scope: restore `q` and `w` as renderer-neutral
  `dart::gui::KeyboardAction`s, add a Bullet preference helper guarded by
  `DART_HAVE_BULLET`, set a promoted `ApplicationOptions::camera` default that
  matches the historical viewer home-position intent, and update the panel copy
  so the example itself documents `q`/`w` controls.
- Keep this checkpoint focused on add/delete skeleton parity. Do not delete the
  private fixture yet; fixture cleanup remains a later promotion-debt sweep
  after enough source-owned examples prove the public API surface.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `add_delete_skels` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi add/delete headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target add_delete_skels
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe add/delete screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_add_delete_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex add_delete_skels --headless --frames 2 --width 640 --height
480 --screenshot /tmp/dart_add_delete_pixi.ppm` plus analyzer
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target
add_delete_skels UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe add/delete screenshot plus
    `analyze_headless_smoke.py` (`/tmp/dart_add_delete_direct_postlint.ppm`,
    307200/307200 nonzero pixels)
- Implementation status: pushed as
  `184396a4cf0 Restore add delete skeleton controls`.

Thirty-second mixed chain parity checkpoint:

- Historical `examples/mixed_chain` exposed `q`/`w`, `e`/`r`, and `t`/`y`
  keyboard controls for short -/+ X, -/+ Y, and -/+ Z impulses on a soft link,
  printed those controls as viewer instructions, and set an example-specific
  camera home view. The current promoted source owns the world and
  `preStep`-driven impulse application but only exposes panel buttons and does
  not wire the legacy keyboard actions or camera default through public
  `dart::gui`.
- Implementation scope: restore those six impulse keys as renderer-neutral
  `dart::gui::KeyboardAction`s, add a promoted `ApplicationOptions::camera`
  default matching the historical home-position intent, and update the panel
  text so the example itself documents the keyboard controls.
- Keep this checkpoint focused on mixed-chain parity. Do not delete the
  private fixture yet; fixture cleanup remains a later promotion-debt sweep
  after enough source-owned examples prove the public API surface.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `mixed_chain` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi mixed-chain headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target mixed_chain
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Fixed the first source-marker test pass after the implementation routed all
    six keys through `makeImpulseAction(...)`; the guard now checks the helper,
    six labels, and `KeyboardShortcut::characterKey(key)`.
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe mixed-chain screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_mixed_chain_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex mixed_chain --headless --frames 2 --width 640 --height 480
--screenshot /tmp/dart_mixed_chain_pixi.ppm` plus analyzer
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target mixed_chain
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe mixed-chain screenshot plus
    `analyze_headless_smoke.py`
    (`/tmp/dart_mixed_chain_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
  - Post-lint `pixi run ex mixed_chain --headless --frames 2 --width 640
--height 480 --screenshot /tmp/dart_mixed_chain_pixi_postlint.ppm` plus
    analyzer (307200/307200 nonzero pixels)
- Implementation status: pushed as
  `d24b69a4149 Restore mixed chain impulse controls`.

Thirty-third vehicle parity checkpoint:

- Historical `examples/vehicle` exposed `w`, `s`, `x`, `a`, and `d` keyboard
  controls for forward, stop, reverse, left steering, and right steering,
  printed those controls as viewer instructions, and set an example-specific
  camera home view. The current promoted source owns the world and pre-step
  vehicle controller, plus equivalent panel buttons/sliders, but it does not
  wire the legacy keyboard actions or camera default through public
  `dart::gui`.
- Implementation scope: restore those five vehicle command keys as
  renderer-neutral `dart::gui::KeyboardAction`s, add a promoted
  `ApplicationOptions::camera` default matching the historical home-position
  intent, and update the panel text so the example itself documents the
  keyboard controls.
- Keep this checkpoint focused on vehicle parity. The existing panel sliders
  are kept as additive promoted-GUI affordances; the historical keyboard
  semantics and camera default are the parity gate for this slice.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `vehicle` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi vehicle headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target vehicle
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe vehicle screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_vehicle_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex vehicle --headless --frames 2 --width 640 --height 480
--screenshot /tmp/dart_vehicle_pixi.ppm` plus analyzer (307200/307200 nonzero
    pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target vehicle
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe vehicle screenshot plus
    `analyze_headless_smoke.py` (`/tmp/dart_vehicle_direct_postlint.ppm`,
    307200/307200 nonzero pixels)
- Implementation status: pushed as
  `3f603fec25b Restore vehicle keyboard controls`.

Thirty-fourth soft-bodies playback parity checkpoint:

- Historical `examples/soft_bodies` exposed `[`, `]`, `{`, `}`, `r`, and `\`
  keyboard controls for stepping backward/forward by one, stepping
  backward/forward by ten, restarting, and jumping to the latest recorded
  state. The current promoted source owns the soft-body world, history capture,
  and equivalent panel buttons, but it does not wire the legacy playback keys
  through public `dart::gui`.
- Implementation scope: restore those six playback keys as renderer-neutral
  `dart::gui::KeyboardAction`s, update the panel text so the example itself
  documents the keyboard controls, and teach the private GLFW input bridge to
  distinguish shifted bracket shortcuts so `KeyboardShortcut::characterKey('{')`
  and `KeyboardShortcut::characterKey('}')` can coexist with `[` and `]`.
- Keep this checkpoint focused on soft-body playback parity. The example did
  not set a historical camera home view, so this slice should not invent one.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `soft_bodies` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi soft-bodies headless screenshot with analyzer coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target soft_bodies
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe soft-bodies screenshot plus `analyze_headless_smoke.py`
    (`/tmp/dart_soft_bodies_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex soft_bodies --headless --frames 2 --width 640 --height 480
--screenshot /tmp/dart_soft_bodies_pixi.ppm` plus analyzer (307200/307200
    nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target soft_bodies
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe soft-bodies screenshot plus
    `analyze_headless_smoke.py`
    (`/tmp/dart_soft_bodies_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
- Implementation status: pushed as
  `5a00359a1f4 Restore soft bodies playback controls`.

Thirty-fifth hybrid-dynamics parity checkpoint:

- Historical `examples/hybrid_dynamics` exposed `h` as the harness toggle,
  printed the harness and simulation instructions, and set an example-specific
  camera home view. The current promoted source owns the world, scripted
  controller, and equivalent panel button, but it does not wire the legacy
  keyboard action or camera default through public `dart::gui`.
- Implementation scope: restore `h` as a renderer-neutral
  `dart::gui::KeyboardAction`, add a promoted `ApplicationOptions::camera`
  default matching the historical home-position intent, and update the panel
  text so the example itself documents the keyboard control.
- Keep this checkpoint focused on hybrid-dynamics parity. The existing harness
  panel button is kept as an additive promoted-GUI affordance.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `hybrid_dynamics` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi hybrid-dynamics headless screenshot with analyzer
    coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target hybrid_dynamics
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe hybrid-dynamics screenshot plus
    `analyze_headless_smoke.py` (`/tmp/dart_hybrid_dynamics_direct.ppm`,
    307200/307200 nonzero pixels)
  - `pixi run ex hybrid_dynamics --headless --frames 2 --width 640 --height
480 --screenshot /tmp/dart_hybrid_dynamics_pixi.ppm` plus analyzer
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target
hybrid_dynamics UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe hybrid-dynamics screenshot plus
    `analyze_headless_smoke.py`
    (`/tmp/dart_hybrid_dynamics_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
- Implementation status: pushed as
  `d6545957998 Restore hybrid dynamics harness controls`.

Thirty-sixth joint-constraints parity checkpoint:

- Historical `examples/joint_constraints` exposed `1` through `4` as
  programmed perturbation keys, `h` as the harness toggle, printed those
  controls as viewer instructions, and set an example-specific camera home
  view. The current promoted source owns the fullbody controller, perturbation
  state, harness state, and equivalent panel buttons, but it does not wire the
  legacy keyboard actions or camera default through public `dart::gui`.
- Implementation scope: restore the five keyboard actions as renderer-neutral
  `dart::gui::KeyboardAction`s, add a promoted `ApplicationOptions::camera`
  default matching the historical home-position intent, and update the panel
  text so the example itself documents the keyboard controls.
- Keep this checkpoint focused on joint-constraints parity. The existing panel
  buttons are kept as additive promoted-GUI affordances.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `joint_constraints` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and/or pixi joint-constraints headless screenshot with analyzer
    coverage
  - Python runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target joint_constraints
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Fixed the first source-marker test pass after the implementation routed
    perturbation keys through `makePerturbAction(...)`; the guard now checks
    the helper, labels, and `KeyboardShortcut::characterKey(key)`.
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe joint-constraints screenshot plus
    `analyze_headless_smoke.py` (`/tmp/dart_joint_constraints_direct.ppm`,
    307200/307200 nonzero pixels)
  - `pixi run ex joint_constraints --headless --frames 2 --width 640 --height
480 --screenshot /tmp/dart_joint_constraints_pixi.ppm` plus analyzer
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target
joint_constraints UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe joint-constraints screenshot plus
    `analyze_headless_smoke.py`
    (`/tmp/dart_joint_constraints_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
- Implementation status: pushed as
  `1ab02df5766 Restore joint constraints controls`.

Thirty-seventh Fetch parity re-audit checkpoint:

- Maintainer correction: source ownership is still not sufficient evidence of
  full restoration. `examples/fetch/` remains the concrete example to re-open
  even though earlier checkpoints restored the promoted source file, camera,
  visible target bars, work-area grid, and legacy panel controls.
- Scope before code changes: compare the current `examples/fetch/main.cpp`
  against the historical OSG example source again, identify any remaining
  user-visible behavior gaps, and move repairable behavior into the promoted
  `dart::gui` public-API example. Do not claim Fetch complete based only on
  headless screenshots, source ownership, or previous smoke evidence.
- Audit finding before code changes: the restored source already covers the
  historical MJCF load, Bullet preference, robot/object initial positions,
  mocap weld reset, target sync loop, visible target bars, work-area grid,
  camera home, panel controls, and `--gui-scale` command-line support through
  the promoted runner. The remaining repairable gaps for this checkpoint are:
  restore the missing `examples/fetch/README.md`, and add a renderer-neutral
  way for a source-owned example to set its default run/window options so Fetch
  can recover the historical 1280x960 launch size while preserving command-line
  overrides.
- Keep any unrepaired behavior gap explicit with the public API it needs. Do
  not revive OSG, private Filament hooks, or backend-specific example code to
  close a parity item.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `fetch` and `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and pixi Fetch headless screenshots with analyzer coverage
  - Python C++ example runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Implementation state for this checkpoint: `dart::gui::ApplicationOptions`
  now accepts optional renderer-neutral `runDefaults`, the private Filament
  option parser seeds its defaults from that value before applying command-line
  overrides, `examples/fetch/main.cpp` uses it to recover the historical
  1280x960 default launch size, and `examples/fetch/README.md` is restored
  with promoted `dart::gui` run/capture instructions.
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target fetch
UNIT_gui_FilamentSceneExtraction --parallel 4`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe Fetch screenshot without explicit width/height plus
    `analyze_headless_smoke.py`
    (`/tmp/dart_fetch_run_defaults_direct.ppm`, 1212842/1228800 nonzero
    pixels)
  - `pixi run ex fetch --headless --frames 2 --screenshot
/tmp/dart_fetch_run_defaults_pixi.ppm`; the pixi runner intentionally appends
    its 640x480 smoke size, and analyzer coverage passed at 303695/307200
    nonzero pixels.
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target fetch
UNIT_gui_FilamentSceneExtraction --parallel 4`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe Fetch screenshot without explicit width/height
    plus `analyze_headless_smoke.py`
    (`/tmp/dart_fetch_run_defaults_direct_postlint.ppm`,
    1212842/1228800 nonzero pixels)
- Next audit after this checkpoint: continue the remaining historical-source
  parity pass, with `examples/rigid_shapes/` already identified as a likely
  next slice because its historical keyboard spawn/delete/contact controls and
  camera default still need a current-source audit.
- Implementation status: pushed as
  `ddc5e113853 Restore Fetch run defaults and README`.

Thirty-eighth rigid-shapes parity checkpoint:

- Historical `examples/rigid_shapes` loaded a rigid-shape world, set a 640x480
  viewer, used a camera home from eye `(2, 2, 2)` to target `(0, 0, 0)`, and
  exposed keyboard controls: `q` spawns a cube, `w` spawns an ellipsoid, `e`
  spawns a cylinder, `r` spawns a convex mesh, `a` deletes the last spawned
  object, and `c` toggles contact-point rendering.
- Current promoted source owns its world and panel, but the historical
  keyboard controls, convex-mesh spawn path, contact-point toggle, camera
  default, 640x480 default launch size, and rigid-shapes-specific
  `--collision-detector`, `--max-contacts`, and `--ground-thickness` command
  line options are not yet restored.
- Scope before code changes: keep the source-owned public `dart::gui` example,
  add the historical keyboard actions through `ApplicationOptions`, restore
  convex mesh spawning as public DART shape construction, represent contact
  points as an example-owned `PointCloudShape` simple frame instead of private
  Filament debug hooks, parse the rigid-shapes-specific command-line options
  locally while leaving promoted runner options to `dart::gui`, set the
  historical camera/run defaults through public options, and update
  panel/test/changelog coverage.
- Local acceptance for this checkpoint:
  - C++ GUI target build for `rigid_shapes` and
    `UNIT_gui_FilamentSceneExtraction`
  - Focused CTest run for `UNIT_gui_FilamentSceneExtraction`
  - Direct and pixi rigid-shapes headless screenshots with analyzer coverage
  - Python C++ example runner tests
  - Full `examples` aggregate target build
  - `pixi run lint`
  - `git diff --check`
- Implementation state for this checkpoint: `examples/rigid_shapes/main.cpp`
  now restores source-owned public `dart::gui` keyboard actions for
  `q`/`w`/`e`/`r` shape spawning, `a` delete-last, and `c` contact-point
  toggling. Convex mesh spawning is restored with public
  `ConvexMeshShape::fromMesh`, contact markers are restored as an
  example-owned `PointCloudShape` simple frame, and the historical camera home
  plus 640x480 default run size are set through `ApplicationOptions`. The
  example also parses the historical rigid-shapes-specific
  `--collision-detector`, `--max-contacts`, and `--ground-thickness` options
  locally before handing remaining promoted runner options to `dart::gui`.
- Local validation completed:
  - `cmake --build build/default/cpp/Release --target rigid_shapes
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe rigid-shapes screenshot without explicit width/height plus
    `analyze_headless_smoke.py` (`/tmp/dart_rigid_shapes_direct.ppm`,
    307200/307200 nonzero pixels)
  - Direct rigid-shapes CLI smoke with `--collision-detector dart
--max-contacts 0 --ground-thickness 0.12` plus analyzer coverage
    (`/tmp/dart_rigid_shapes_cli_direct.ppm`, 307200/307200 nonzero pixels)
  - `pixi run ex rigid_shapes --headless --frames 2 --screenshot
/tmp/dart_rigid_shapes_pixi.ppm` plus analyzer coverage (307200/307200
    nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target rigid_shapes
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe rigid-shapes screenshot without explicit
    width/height plus `analyze_headless_smoke.py`
    (`/tmp/dart_rigid_shapes_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
- Implementation status: pushed as
  `161d6a16be4 Restore rigid shapes controls`.

Thirty-ninth biped-stand parity checkpoint:

- Historical `examples/biped_stand` loaded `fullbody1.skel`, ran the SPD
  standing controller, exposed `1` through `4` as perturbation keys
  (`+X`, `-X`, `+Z`, `-Z` for 100 frames), printed those controls, used a
  640x480 viewer, set camera home from eye `(3, 1.5, 3)` to target
  `(0, 0, 0)`, and had an example README.
- Current promoted source owns the controller and panel perturbation buttons,
  but it does not restore the legacy number-key actions, camera/run defaults,
  or README.
- Scope before code changes: add renderer-neutral `KeyboardAction`s for
  `1` through `4`, set the historical camera and 640x480 defaults through
  public `ApplicationOptions`, restore the README with promoted `dart::gui`
  wording, update the panel text, tests, and changelog, and keep the controller
  source-owned without private renderer hooks.
- Implementation status: biped-stand source now restores renderer-neutral
  number-key perturbation actions, panel help text, camera/run defaults, and
  the example README through promoted `dart::gui`.
- Local acceptance completed for this checkpoint:
  - `cmake --build build/default/cpp/Release --target biped_stand
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe biped-stand screenshot without explicit width/height plus
    `analyze_headless_smoke.py` (`/tmp/dart_biped_stand_direct.ppm`,
    307200/307200 nonzero pixels)
  - `pixi run ex biped_stand --headless --frames 2 --screenshot
/tmp/dart_biped_stand_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`
  - `git diff --check`
  - `pixi run lint`
  - Post-lint `cmake --build build/default/cpp/Release --target biped_stand
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - Post-lint `ctest --test-dir build/default/cpp/Release --output-on-failure
-R '^UNIT_gui_FilamentSceneExtraction$'`
  - Post-lint direct llvmpipe biped-stand screenshot without explicit
    width/height plus `analyze_headless_smoke.py`
    (`/tmp/dart_biped_stand_direct_postlint.ppm`, 307200/307200 nonzero
    pixels)
- Implementation status: pushed as
  `90b373beb14 Restore biped stand controls`.

Fortieth free-joint-cases parity checkpoint:

- Historical `examples/free_joint_cases` had a standalone ImGui panel for
  pause/step/reset, numeric Jacobian checks, reference-body visibility,
  spherical-inertia toggling, torque-free versus constant-world-twist reference
  models, torque-free substeps, grid visibility, per-case metric rows,
  example-specific CLI flags (`--numeric-dt`, `--dt`, `--spherical-inertia`,
  `--ground-truth`, `--ground-truth-substeps`), 1280x720 defaults, camera home
  from eye `(2.5, -10, 4)` to target `(2.5, 0, 0)`, and an example README.
- Current promoted source owns the free-joint visual cases and torque-free
  reference bodies, but it dropped the README, example-specific CLI controls,
  constant-twist reference mode, spherical-inertia toggle, numeric metric
  recomputation, metric display, and camera/run defaults.
- Scope before code changes: restore the source-owned controls through the
  existing renderer-neutral `PanelBuilder` primitives, keep any unsupported
  table/combo affordances as readable text/button/check/slider controls rather
  than private ImGui calls, parse the historical free-joint CLI flags locally
  while letting `dart::gui` own shared viewer flags, set the historical camera
  and 1280x720 defaults through public `ApplicationOptions`, restore the
  README with promoted `dart::gui` wording, and update tests/changelog.
- Implementation status: free-joint-cases source now restores numeric metric
  recomputation/display, reference visibility, spherical-inertia and
  constant-world-twist controls, torque-free substeps, local CLI flags, camera
  and run defaults, and the example README through promoted `dart::gui`.
- Local acceptance completed so far for this checkpoint:
  - `cmake --build build/default/cpp/Release --target free_joint_cases
UNIT_gui_FilamentSceneExtraction --parallel 5`
  - `ctest --test-dir build/default/cpp/Release --output-on-failure -R
'^UNIT_gui_FilamentSceneExtraction$'`
  - Direct llvmpipe free-joint-cases screenshot with
    `--ground-truth constant --spherical-inertia --numeric-dt 1e-6 --dt 0.001
--ground-truth-substeps 20`, no explicit width/height, and
    `analyze_headless_smoke.py` (`/tmp/dart_free_joint_cases_direct.ppm`,
    921600/921600 nonzero pixels)
  - `pixi run ex free_joint_cases --headless --frames 2 --screenshot
/tmp/dart_free_joint_cases_pixi.ppm` plus `analyze_headless_smoke.py`
    (307200/307200 nonzero pixels at the pixi wrapper's 640x480 size)
  - `pixi run python -m pytest python/tests/unit/test_run_cpp_example.py -q`
    (67 passed)
  - `cmake --build build/default/cpp/Release --target examples --parallel 5`

## Stretch Direction

These should be designed for but do not block the immediate restoration slice:

- Dear ImGui Docking.
- A Filament 3D scene rendered as one docked ImGui window/widget.
- Offscreen rendering as a first-class `dart::gui` workflow.
- Automated video capture. Image-sequence capture via `--out <dir>` is now a
  compatibility requirement for the active restoration work, not only stretch
  scope.

## Branch Done Checklist

The branch is ready to hand off for review only when:

- Promoted public names and maintained examples use `dart::gui`/`dartsim`
  branding, with no `experimental` token in promoted public concepts.
- Linux headless CI is green on the tracked branch after the restored-example
  repair lands.
- The agreed restored historical example executables exist and can produce
  headless screenshots through the promoted capture path, and the historical
  `--out <dir>` image-sequence workflow exists without reviving OSG.
- No maintained user-facing file refers to `filament_gui` as an official
  executable, example, or workflow. Private backend helper names may be renamed
  in a separate promotion-debt sweep.
- Documentation consistently distinguishes `DART`/`libdart` library identity
  from `dartsim` application identity.

## Immediate Next Steps

1. Run mandatory `pixi run lint`, post-lint focused Coupler checks, then commit
   and push the `examples/coupler_constraint/` parity checkpoint.
2. Continue the same historical-source parity audit across every remaining
   pre-existing example; do not treat source ownership, build success, runner
   coverage, or screenshots as sufficient restoration evidence.
3. Keep `scene_fixtures.cpp` as transitional dev/test infrastructure until the
   corresponding example behavior has moved into public-API example code.
4. Do not start the physical `experimental/` directory move until the
   application extraction and enough real example sources prove the consumed
   public API surface.
5. Run `pixi run lint` before every checkpoint commit, then push the commit to
   the tracked remote branch.
