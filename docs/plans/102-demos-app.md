# PLAN-102: Demos App

- Operating state: `PLAN-102` in [`dashboard.md`](dashboard.md)
- Outcome: a single standalone **Demos** application (`examples/demos/`,
  executable `dart-demos`) hosts every GUI example as a small, focused, registered
  scene that users pick from a categorized, ordered sidebar and switch between at
  runtime; each scene is the single source consumed by both the app and the GUI
  tests. `hello_world` remains the standalone minimal "link DART in a CMake
  project" template.
- Architecture owner (active): [`../dev_tasks/demos_app/01-design.md`](../dev_tasks/demos_app/01-design.md)
  (promoted to `../design/demos_app.md` on completion).
- Execution tracking: [`../dev_tasks/demos_app/README.md`](../dev_tasks/demos_app/README.md)

This file owns scope, workstreams, sequencing, acceptance criteria, and open
gaps. `dashboard.md` owns priority, status, horizon, dimension, next step, and
gate.

## Why

DART's north star includes an "easy start": a new researcher should reach a
sense of what DART can do with minimal friction. Today ~39 GUI examples are
separate executables, and most scenes exist in triplicate — a standalone
`examples/<name>/main.cpp`, a private `createXxxScene()` in
`dart/gui/detail/scenes.cpp` (the `--scene` catalog), and a registered
smoke/renderable test. Surveying capabilities means building and launching many
binaries, and changing one example touches several files. A single Demos app with
a categorized sidebar lets users browse capabilities in one place and lets
contributors add an example by writing one focused scene plus one registry line.

## Scope

In scope (architecture in [`../dev_tasks/demos_app/01-design.md`](../dev_tasks/demos_app/01-design.md)):

- A `dart::gui` runtime scene-swap host (`runDemos`) that switches the active
  world/panels/handlers without recreating the window, by restructuring the
  backend loop into an outer scene loop around the inner frame loop.
- `examples/demos/`: a `demos_scenes` library (one focused file per example, an
  ordered registry, scene/category types) and a thin `dart-demos` app; a built-in
  Demos sidebar with categories, ordering, search, and per-scene info/controls.
- Migration of all GUI examples into scene modules; deletion of the standalone
  GUI example directories as each lands.
- A single source of truth: GUI smoke + renderable-extraction tests consume the
  Demos registry; the `dart/gui/detail` `ExampleScene` catalog and `--scene`
  example overloads are removed, leaving one tiny `dart-gui` MVP self-smoke.
- Tooling/doc updates: `scripts/run_cpp_example.py`, pixi tasks,
  `examples/README.md`, `dartsim/README.md` cross-references, dartpy example
  docs, and `CHANGELOG.md`.

Out of scope: scene authoring/editing (that is `dartsim`, PLAN-101); any
renderer/backend change; the four headless CLI examples (`csv_logger`,
`headless_simulation`, `speed_test`, `unified_loading`), which stay separate CLI
examples; a dockable workspace (deferred follow-up — ship a fixed system-ImGui
sidebar first).

## Workstreams (phases)

1. **Scaffold + prove the pattern**: runtime scene-swap host (`runDemos`) +
   Demos sidebar in `dart::gui`; `examples/demos/` skeleton; 3 seed scenes across
   categories (hello-world-as-scene, boxes, one robot); a headless smoke that
   switches scenes.
2. **Migrate scenes by category**: fold each standalone example's focused
   behavior into its module and delete the standalone dir as it lands; prefer the
   richer standalone logic where it differs from the old fixture.
3. **Single-source the tests**: retarget the renderable-extraction and per-scene
   smokes to the registry; delete the `dart/gui/detail` example catalog and
   `--scene` overloads; keep the `dart-gui` MVP self-smoke.
4. **Tooling + docs**: update the runner, pixi tasks, READMEs, dartpy docs, and
   `CHANGELOG.md` to the Demos model.
5. **Complete**: promote the design to `docs/design/demos_app.md` (registered in
   `docs/design/README.md`), add a brief note to
   `docs/onboarding/gui-rendering.md`, and delete the dev-task folder in the
   completing PR.

Phase 1 is the de-risking slice (the runtime swap is the only genuinely new
capability). Phases 2–4 are largely mechanical once the pattern holds; phase 3
must land with or right after the scenes it depends on so the test suite is never
red.

## Acceptance Criteria

The initiative is complete (and durable output has moved to owner docs) when:

- `dart-demos` builds and launches, lists every migrated example grouped by
  ordered category, and switches between scenes at runtime in one window with no
  window recreation and no leak/dangling-handle across a switch-stress run.
- Each migrated scene is one focused `DemoScene` (id/title/category/order/
  summary/help/lazy `build`) and is the single source consumed by both the app
  and its headless smoke + renderable check.
- No standalone GUI example directories remain except `hello_world`, which still
  builds as a standalone CMake project; the four headless CLI examples still
  build and run.
- The `dart/gui/detail` `ExampleScene` example catalog, `parseSceneName`, and the
  `runApplication(argc, argv, const char*)` / `--scene` example overloads are
  removed; `dart-gui` keeps exactly one internal scene for a renderer self-smoke
  that does not depend on `examples/`.
- `scripts/run_cpp_example.py`, pixi tasks, `examples/README.md`,
  `dartsim/README.md`, and dartpy example docs describe the Demos model;
  `CHANGELOG.md` records the consolidation and the removed entry points.
- `pixi run lint`, `pixi run build`, and `pixi run test-all` pass for touched
  surfaces; `pixi run check-docs-policy` passes; durable architecture lives in
  `docs/design/demos_app.md` and the dev-task folder is deleted in the completing
  PR.

## Gate

See `dashboard.md` PLAN-102 for the operating gate. Objective-specific proof: a
headless run launches Demos, switches across several scenes (including an
asset-backed robot) without leaks, and renders non-blank frames per scene; the
example renderable-count checks run against the Demos registry; and no standalone
GUI example binary or `--scene` example path remains.

## Open Gaps

- Fate of the four headless CLI examples (keep standalone, recommended, vs
  represent as no-viewport Demos entries with a console panel).
- Timing of the dockable workspace (deferred; fixed sidebar first).
- Renderable-count expectations: exact counts are brittle; prefer a per-scene
  floor co-located with each scene.

## Current Evidence

- Every GUI example builds a `dart::gui::ApplicationOptions` and calls
  `dart::gui::runApplication(argc, argv, options)` (see `examples/*/main.cpp`),
  so a scene reduces to a factory producing those options.
- The `dartsim` editor (`dartsim/ui/src/editor.cpp`) swaps the rendered scene
  every frame via `ApplicationOptions::renderableProvider` over an empty canvas
  world, proving runtime scene mutation without window recreation.
- The duplicated catalog lives in `dart/gui/detail/scenes.{hpp,cpp}`
  (`enum class ExampleScene`, `createDartScene`), reached via `--scene` and
  exercised by `tests/unit/gui/test_filament_scene_extraction.cpp` and the
  `EXAMPLE_dartsim_<scene>_headless_smoke` smokes.
- Shared example chrome already exists: `examples/gui_scene_example.cmake`
  (`dart_build_gui_example`) and `examples/gui_source_grid.hpp`.
