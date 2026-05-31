# dart-demos Example App Architecture

Durable architecture and rationale for `dart-demos`, the application that
consolidates DART's GUI examples into a single window. Roadmap state lives in
`docs/plans/dashboard.md` (PLAN-102); user instructions live in
`examples/README.md`.

## Problem

DART's C++ examples had grown to ~43 directories under `examples/`, each a
standalone executable with its own `main.cpp` and CMake target. Surveying DART's
capabilities meant building and launching many binaries, and adding an example
touched several places. The proliferation did not scale.

## Outcome

`examples/` now contains:

- **`hello_world`** — the minimal, copy-pasteable standalone template for an
  executable CMake project that links DART.
- **`demos`** — the `dart-demos` application: one window hosting most GUI
  examples as scenes the user picks from a categorized sidebar and switches
  between at runtime.
- A few standalone programs that do not fit the single-window model: the headless
  CLI examples (`csv_logger`, `headless_simulation`, `speed_test`,
  `unified_loading`); `gui_scene_diagnostics` (a headless GUI-descriptor
  inspector); and `atlas_simbicon`, `collision_sandbox`, `wam_ikfast` (see
  Standalone exceptions).

## Strategy: Python-first, C++ frozen

Per PLAN-103, Python is DART's primary, growing example surface; this C++
`dart-demos` app is **frozen** (maintained, not grown). New example content is
authored Python-first (`pixi run py-demos`, the headless runner, capture
helpers, and the Colab notebook gallery), and only a small golden subset is
mirrored in C++ and kept honest by a thin parity smoke. `dart-demos` is retired
only later, when the Python surface covers the breadth and the `dartsim` editor
(PLAN-101) can open curated example scenes interactively; the retire-later
checklist lives in PLAN-103. Renderer regression coverage is independent (see
below), so a future retirement loses no renderer coverage.

## Architecture

### Scene as data, not an entry point

Every former example's `main()` built a `dart::gui::ApplicationOptions` (world,
panels, gizmos, IK/drag handles, keyboard actions, camera) and called
`dart::gui::runApplication`. A demos scene is exactly that builder, minus the
window/argv concerns:

```cpp
// examples/demos/scenes/<name>.cpp
dart::gui::ApplicationOptions dart::examples::demos::make<Name>Scene();
```

Scenes are declared in `examples/demos/scenes.hpp` and registered — with a
stable `id`, display `title`, `category`, and one-line `summary` — in
`examples/demos/registry.cpp`. The registry vector order defines display order;
categories appear in first-appearance order. Adding an example is: one scene
file, one header declaration, one registry entry, one CMake source line.

The factory is lazy (built when the scene is first selected), so launch stays
fast and an asset/remote-load failure affects only that scene. The host
soft-fails a scene whose factory throws by showing an empty world, keeping the
sidebar usable.

### Runtime scene switching (`dart::gui`)

The new capability is `dart::gui::runDemos(argc, argv, std::vector<DemoSceneEntry>)`
plus a runtime scene swap in the GUI application loop
(`dart/gui/detail/application.cpp`). The window, Filament engine/view/scene,
materials, ImGui overlay, and lights are created once; the scene-bound state
(world, renderables, `SceneFrameUpdater`, camera) is rebuilt when a switch is
requested via `ViewerLifecycleState::sceneSwitchRequested`. Teardown is split
into `destroySceneRenderables` (per scene) and
`destroyPersistentApplicationResources` (at shutdown). The single-scene
`runApplication` path is unchanged.

The built-in **Demos** sidebar is an ordinary `dart::gui::Panel`: it lists
categories and scenes and calls `dart::gui::requestSceneSwitch` on selection, so
no renderer/UI-toolkit code leaks above the `dart::gui` boundary (PLAN-060).

CLI: `--scene <id>` selects the initial scene; `--cycle-scenes` advances through
every scene for a few frames and exits (the headless smoke,
`EXAMPLE_dart_demos_cycle_headless_smoke`).

### Python `py-demos` workspace

`pixi run py-demos` uses the same `dart::gui::runDemos` host through
`dartpy.gui.run_demos`, so Python examples can run as an interactive multi-scene
workspace rather than only a headless smoke runner. The workspace docks a top
`Simulation` toolbar, a searchable/category-grouped `Demos` navigator, optional
scene-specific panels on the right, and DART diagnostics at the bottom. Python
scenes return `SceneSetup` objects with optional `pre_step`, `force_drag`, and
`ScenePanel` callbacks; those callbacks render through the renderer-neutral
`PanelBuilder`/`PanelContext` abstraction rather than direct ImGui calls.

`pixi run py-demo-capture` drives the same Filament render path headlessly,
including optional ImGui panels via `--show-ui`, and writes screenshots, frame
sequences, and MP4s for visual debugging. It rejects blank/noop captures so
layout, camera, lighting, and material changes have inspectable artifacts.

External-force interactions are a user-facing scene state, not just an input
callback. The common sx bridge panel shows whether force application is idle,
disabled, rejected for a static/unmapped target, or actively applying force; it
also exposes target and magnitude so a single headless `--show-ui` capture can
prove what the viewport spring/arrow is doing.

Runtime demo switches are transactional. If a requested scene throws while
building, misses the startup budget, cannot create render state, or fails its
first frame, the host restores the previous active scene and leaves the reason
visible in the `Simulation` and `Demos` panels. Python scene builders are
bounded by the same `DART_DEMO_SCENE_STARTUP_TIMEOUT_MS` budget by default, with
`DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS` available as a Python-specific override.

The Python workspace rebuilds a deterministic dock layout on startup instead of
trusting any saved panel layout from a previous run. `Reset Layout` is the
explicit recovery action after interactive rearrangement. This keeps stale
`imgui.ini` state from obscuring the viewport or hiding panels when
scene-specific controls change. The layout builder runs before dockspace
submission so ImGui applies the initial `Demos`, `Simulation`, scene-panel, and
diagnostics placements as docked windows on the first rendered frames.

### Build layout

`examples/demos/` builds the `dart-demos` executable via the shared
`dart_build_gui_example` helper, linking `dart-io` (scene asset loading) and
`dart-simulation-experimental` (the experimental rigid-body scene). Collision
backends (ODE/Bullet) are selected through runtime factories and need no extra
link.

## Standalone exceptions

These remain standalone rather than demos scenes:

- **`atlas_simbicon`, `collision_sandbox`** — their controller and
  pair-registry source files are compiled directly by the test suite
  (`tests/integration/CMakeLists.txt`, `tests/unit/collision/CMakeLists.txt`).
  Keeping them as multi-file standalone examples keeps those sources available to
  the tests without duplicating them into a merged scene.
- **`wam_ikfast`** — depends on a generated IKFast library selected via
  `DART_WAM_IKFAST_LIB_PATH` at configure time.
- **`gui_scene_diagnostics`** — a headless GUI-descriptor inspector, not a
  viewer scene.

## Examples vs renderer test fixtures

The `dart/gui/detail/scenes.{hpp,cpp}` `ExampleScene` set is **retained as the
renderer's internal test fixtures**, not as an examples source. It backs the
Filament renderable-extraction regression test
(`tests/unit/gui/test_filament_scene_extraction.cpp`) and the
`EXAMPLE_dartsim_<scene>_headless_smoke` graphics smokes, which validate that the
renderer extracts geometry correctly across many shape types. These are
renderer-regression assets owned by `dart::gui`, conceptually distinct from the
user-facing examples (which are single-sourced in `examples/demos`). They are
intentionally not deleted: doing so would remove that renderable-extraction
coverage, and `dart-gui` library tests should not depend on `examples/`. The
demos cycle smoke covers the example scenes; the `ExampleScene` fixtures cover
the renderer.

## Non-goals

- Not a scene editor (authoring/undo/redo/project I/O on the experimental World
  is `dartsim`'s role; see `dartsim_gui_simulator.md`).
- No renderer/backend changes; demos builds on `dart::gui`.
- No Python-side scene authoring API; `py-demos` is an examples workspace for
  playback, controls, diagnostics, and capture, not an editor.
- `examples/demos` builds the scenes straight into the `dart-demos` executable.
  Splitting them into a `demos_scenes` library is a future option, only needed if
  something other than the app (e.g. a test) must link the scene registry.
