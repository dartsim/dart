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

## Branch And CI State

- Branch: `feature/filament-gui-full-execution`
- Upstream: `origin/feature/filament-gui-full-execution`
- Latest pushed checkpoint: `64284fbcda5` (active migration plan docs)
- GitHub Actions were manually dispatched without opening a PR.
- Linux Filament smoke tests passed at the latest inspected run.
- Linux headless rendering failed earlier because the workflow still invoked the
  legacy `rigid_cubes` executable, which was removed in the earlier example
  cleanup. Restoring legacy example entry points and capture compatibility is
  therefore a concrete CI requirement, not only documentation polish.
- Local repair checkpoint in progress:
  - `examples/dartsim` is the renamed app-level viewer.
  - `dart/gui/application.hpp` exposes the narrow promoted launch API.
  - Historical GUI example executables are restored as thin `dart::gui`
    launchers.
  - The Linux headless workflow now validates `rigid_cubes --screenshot`
    output instead of the removed OSG-style `--out` PNG sequence.
  - Local evidence: targeted CMake configure/build passed, the
    `UNIT_gui_FilamentSceneExtraction` boundary test passed,
    `python/tests/unit/test_run_cpp_example.py` passed, direct `rigid_cubes`
    headless PPM capture passed, and `pixi run test-filament-gui-smoke` passed
    31/31 tests with `EXAMPLE_dartsim_*` smoke names.

## Current Code Shape

- The branch has removed legacy OSG/Raylib renderer build, dependency,
  tutorial, example, Python, and documentation surfaces.
- `DART_BUILD_GUI` now defaults according to platform support for the pinned
  Filament path.
- The maintained renderer implementation still lives behind names such as
  `dart::gui::experimental`; that namespace is now promotion debt. The
  backend-named MVP example has been renamed to `examples/dartsim`.
- Private implementation details live under
  `dart/gui/experimental/detail/filament`. They can remain private while the
  promoted public names move to `dart::gui`.
- Existing command-line support includes bounded frames, window size,
  headless mode, UI visibility, scene selection, profiling, and a single PPM
  screenshot path.
- Current execution decision: restore the historical executable names as thin
  `dart::gui` launchers and update CI to validate the promoted
  `--screenshot` capture contract. Multi-frame `--out` output remains useful
  future capture work, but it is not required for this repair checkpoint.

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
- Branding direction: `DART` remains the overall project and library identity,
  with `libdart` remaining appropriate for packaging/library contexts.
  `dartsim` is the application-level identity, analogous to an application
  product such as Isaac Sim. Use this distinction when choosing promoted
  executable names, app documentation, and future GUI packaging language.

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

## Stretch Direction

These should be designed for but do not block the immediate restoration slice:

- Dear ImGui Docking.
- A Filament 3D scene rendered as one docked ImGui window/widget.
- Offscreen rendering as a first-class `dart::gui` workflow.
- Automated screenshot sequences and video capture.

## Branch Done Checklist

The branch is ready to hand off for review only when:

- Promoted public names and maintained examples use `dart::gui`/`dartsim`
  branding, with no `experimental` token in promoted public concepts.
- Linux headless CI is green on the tracked branch after the restored-example
  repair lands.
- The agreed restored historical example executables exist and can produce
  headless screenshots through the promoted capture path.
- No maintained user-facing file refers to `filament_gui` as an official
  executable, example, or workflow. Private backend helper names may be renamed
  in a separate promotion-debt sweep.
- Documentation consistently distinguishes `DART`/`libdart` library identity
  from `dartsim` application identity.

## Immediate Next Steps

1. Run `pixi run lint` for the active checkpoint.
2. Commit and push the `dartsim` rename/restored-example repair.
3. Continue with the next promotion slice: reduce remaining
   `dart::gui::experimental` naming in promoted headers and bindings while
   keeping backend resources private.
4. Keep multi-frame image sequence and video capture as follow-up capture work.
5. Run `pixi run lint` before every checkpoint commit, then push the commit to
   the tracked remote branch.
