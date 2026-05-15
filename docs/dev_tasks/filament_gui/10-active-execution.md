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

## Branch And CI State

- Branch: `feature/filament-gui-full-execution`
- Upstream: `origin/feature/filament-gui-full-execution`
- Latest pushed checkpoint before this in-progress slice:
  `8796ed5ad99 Promote DART GUI C++ namespace`
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
  - `examples/dartsim` is the renamed app-level viewer.
  - `dart/gui/application.hpp` exposes the narrow promoted launch API.
  - Historical GUI example executables are restored as thin `dart::gui`
    launchers.
  - The Linux headless workflow now validates `rigid_cubes --screenshot`
    output instead of the removed OSG-style `--out` PNG sequence.
  - Local evidence: `pixi run lint`, full `examples` target build,
    `UNIT_gui_FilamentSceneExtraction`,
    `python/tests/unit/test_run_cpp_example.py`, direct `rigid_cubes`
    headless PPM capture, and `pixi run test-filament-gui-smoke` all passed
    before or during the checkpoint.

## Current Code Shape

- The branch has removed legacy OSG/Raylib renderer build, dependency,
  tutorial, example, Python, and documentation surfaces.
- `DART_BUILD_GUI` now defaults according to platform support for the pinned
  Filament path.
- The backend-named MVP example has been renamed to `examples/dartsim`.
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
  - Decision for this slice: `examples/dartsim` / `dartsim`
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
  - `pixi run test-filament-gui-smoke`
  - `pixi run lint`

## Example Restoration Plan

Restore the pre-existing user-facing examples as migrated `dart::gui` examples
instead of leaving them collapsed into one backend-named executable.

Implementation direction:

- Add a promoted public application entry point, for example
  `dart/gui/application.hpp`, that provides a generic viewer launcher.
- Keep each restored example's `main.cpp` small and DART-owned. It should call
  the generic `dart::gui` launcher with the example's default scene or workflow.
- Preserve historical example executable names for CI and user muscle memory.
- Keep Filament-specific setup in private GUI implementation units.
- Ensure `scripts/run_cpp_example.py`, CMake targets, tests, and CI workflows
  refer to scope-based or historical example names, not `filament_gui`.
- Restore examples before relying on CI jobs that invoke historical binaries
  such as `rigid_cubes`.

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

1. Continue private target/helper-name cleanup where it does not affect
   compatibility.
2. Do not expose Filament, GLFW, or Dear ImGui types in promoted headers.
   Private implementation can remain under `dart/gui/experimental/detail` until
   a later file-layout sweep.
3. Keep ImGui Docking, docked 3D widgets, first-class offscreen APIs, and video
   capture as follow-up application/capture work.
4. Run `pixi run lint` before every checkpoint commit, then push the commit to
   the tracked remote branch.
