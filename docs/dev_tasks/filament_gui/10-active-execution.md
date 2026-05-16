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
  `9f195a0738c Expose GUI IK handles for examples`
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
  restores a visible green cross target from public `SimpleFrame`/shape APIs,
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

1. Finish the `atlas_puppet` checkpoint with lint, post-lint focused rebuild,
   commit, push, and CI dispatch if permissions allow it.
2. Prioritize the remaining robot/IK macro-launcher examples after
   `atlas_puppet`: `hubo_puppet` and `g1_puppet`.
3. Continue across the remaining pre-existing examples in small related
   families, documenting each slice here before implementation.
4. Keep `scene_fixtures.cpp` as transitional dev/test infrastructure until the
   corresponding example behavior has moved into public-API example code.
5. Do not start the physical `experimental/` directory move until the
   application extraction and enough real example sources prove the consumed
   public API surface.
6. Run `pixi run lint` before every checkpoint commit, then push the commit to
   the tracked remote branch.
