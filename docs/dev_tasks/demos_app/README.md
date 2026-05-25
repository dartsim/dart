# Demos App — Dev Task

Consolidate DART's GUI examples into a single standalone **Demos** app
(`dart-demos`) where each example is a small, focused, registered scene picked
from a categorized, ordered sidebar and switchable at runtime. Keep `hello_world`
standalone as the minimal "link DART in a CMake project" template.

- Plan: [`../../plans/102-demos-app.md`](../../plans/102-demos-app.md)
  (PLAN-102) — operating status in
  [`../../plans/dashboard.md`](../../plans/dashboard.md).
- Architecture & decisions: [`01-design.md`](01-design.md) (architecture owner
  while this task is active; promoted to `docs/design/demos_app.md` on
  completion).

## Current Status

- [ ] Phase 1: add `dart::gui` runtime scene-swap host (`runDemos`) + Demos
      sidebar; scaffold `examples/demos/` (`demo_scene.hpp`, `registry.cpp`,
      `app/main.cpp`); seed 3 scenes; scene-switching headless smoke.
- [ ] Phase 2: migrate all GUI examples into scene modules, deleting each
      standalone dir as it lands.
- [ ] Phase 3: retarget GUI tests to the registry; delete the
      `dart/gui/detail/scenes.*` `ExampleScene` catalog and `--scene` overloads;
      keep the `dart-gui` MVP self-smoke.
- [ ] Phase 4: update tooling/docs (`run_cpp_example.py`, pixi tasks,
      `examples/README.md`, `dartsim/README.md`, dartpy docs, `CHANGELOG.md`).
- [ ] Phase 5: promote design to `docs/design/`, add onboarding note, delete this
      folder in the completing PR.

## Goal

A new researcher launches one app and surveys DART's capabilities by clicking
through categorized demos; a contributor adds an example by writing one focused
scene file and one registry line. Each scene is the single source consumed by
both the app and the GUI tests.

## Non-Goals (early phases)

- No scene authoring/editor (that is `dartsim`, PLAN-101).
- No renderer/backend changes; build on `dart::gui`.
- Headless CLI examples (`csv_logger`, `headless_simulation`, `speed_test`,
  `unified_loading`) stay separate CLI examples (revisit; see design Open
  Questions).
- Dockable workspace deferred; ship a fixed system-ImGui sidebar first.

## Key Decisions

- **Name: Demos** (`examples/demos/`, `dart-demos`, `pixi run demos`,
  `dart::examples::demos`), per the user's instruction.
- **Single source of truth**: one `DemoScene` per example feeds both the app and
  the tests. Delete the standalone dirs and collapse the `dart/gui/detail`
  `ExampleScene` catalog. Backward compatibility (`--scene` enum, per-example
  binaries, `pixi run ex <gui-name>`) is intentionally broken.
- **Lazy scene factories** so launch is fast and asset/remote failures are
  per-scene, not fatal.
- **Runtime scene swap** restructures the backend loop into an outer scene loop
  around the inner frame loop, persisting the window/engine and rebuilding only
  the scene-bound state; no window recreation.

## Immediate Next Steps

1. Add `ViewerLifecycleState::sceneSwitchRequested`, restructure
   `runGuiBackendApplicationImpl` into an outer scene loop, and add the
   `dart::gui::runDemos` host + a Demos sidebar panel.
2. Scaffold `examples/demos/` with `demo_scene.hpp`, `registry.cpp`,
   `app/main.cpp`, and CMake producing `demos_scenes` + `dart-demos`.
3. Port 3 seed scenes (hello-world-as-scene, boxes, one robot) and a headless
   smoke that switches scenes; confirm `pixi run demos` and the smoke run.
