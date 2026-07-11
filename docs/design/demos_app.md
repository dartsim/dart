# dart-demos World App Architecture

Durable architecture and rationale for `dart-demos`, the C++ World demo
application. Roadmap state lives in `docs/plans/dashboard.md` (PLAN-102); user
instructions live in
`examples/README.md`.

## Problem

DART's C++ GUI example catalog was originally consolidated from many standalone
executables into one `dart-demos` scene runner. With DART 7 moving to the World
solver as the official simulation surface, keeping the old DART 6 demo catalog
in `dart-demos` became misleading. The app now demonstrates the current World
solver surface directly instead of preserving the historical DART 6 examples.

## Outcome

`examples/` now contains:

- **`demos`** — the `dart-demos` application: one window hosting C++ World
  solver scenes the user picks from a categorized sidebar and switches between
  at runtime.
- The standalone `dartsim` authoring/viewer application. The old CLI examples
  that taught DART 6 whole-World loading were retired from `main`; use
  `release-6.*` branches for that parity source.

## Strategy: World-only Demos

Per PLAN-103, Python is DART's primary, growing example surface. Both `py-demos`
and C++ `dart-demos` now present World solver demos without DART 6 scene ids or
experimental/sim labels. Python carries the broader catalog (World rigid body,
Rigid IPC, Variational Integrators, Differentiable, Vertex Block Descent, and
IPC Deformable); C++ keeps the smaller companion set needed for native
viewer/smoke coverage. A shared `Planned World Ports` category keeps important
DART 6 concepts visible as launchable placeholders only; it does not keep legacy
scene implementations. Renderer regression coverage is independent (see below),
so deleting DART 6 demo scene files does not remove renderer fixture coverage.

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
categories appear in first-appearance order. World demos use solver/domain
categories rather than a catch-all bucket so the navigator scales as the catalog
grows. Placeholder rows belong in `Planned World Ports` until a real
World-native scene exists. Adding an example is: one scene file, one header
declaration, one registry entry, one CMake source line.

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

Each implemented World scene also installs a renderer-neutral **Memory
Diagnostics** panel from `examples/demos/memory_diagnostics.*`. The shared
`memory_diagnostics_model.*` owns opt-in cadence, bounded history, baselines,
comparison compatibility, process-resident probes, and evidence quality. The
branch-local adapter owns World collection and `PanelBuilder` presentation.
Construction is deliberately cheap: until enabled, the panel does not query the
OS, walk World/ECS diagnostics, allocate its history ring, or format a sample.
The panel consumes the public type-erased `WorldMemoryDiagnostics` facade; it
does not expose EnTT types, component types, registry access, or stable
component IDs through the examples layer. It explicitly requests packed-layout
detail only on a due sample; default World diagnostics consumers retain the
per-storage summary path without scanning every packed component slot.

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

The Python catalog is broad enough that navigation needs domain categories, not
only search. Users can replace a queued sidebar switch by clicking a different
target before the candidate starts, so the UI does not trap the user on a stale
pending row while the transactional activation path decides what to load.

`pixi run py-demo-capture` drives the same Filament render path headlessly,
including optional ImGui panels via `--show-ui`, and writes screenshots, frame
sequences, and MP4s for visual debugging. It rejects blank/noop captures so
layout, camera, lighting, and material changes have inspectable artifacts. With
`--show-ui`, capture also checks that the docked ImGui workspace is present and
drops warm-up frame-output images captured before the UI becomes visible, so
recorded PNG sequences and MP4s start from a useful workspace frame.

External-force interactions are a user-facing scene state, not just an input
callback. The common World bridge panel shows whether force application is idle,
disabled, rejected for a static/unmapped target, or actively applying force; it
also exposes target and magnitude so a single headless `--show-ui` capture can
prove what the viewport spring/arrow is doing.

Runtime demo switches are transactional. If a requested scene throws while
building, misses the startup budget, cannot create render state, or fails its
first frame, the host restores the previous active scene and leaves the reason
visible in the `Simulation` and `Demos` panels. Python scene builders are
bounded by the same `DART_DEMO_SCENE_STARTUP_TIMEOUT_MS` budget by default, with
`DART_PY_DEMO_SCENE_BUILD_TIMEOUT_MS` available as a Python-specific override.
Python `pre_step` callbacks are also bounded during candidate startup, and the
host rolls back a switched demo whose first frame returns over budget. A native
scene that never returns from construction or simulation would still need
process isolation; this guard is for failures that return or Python callbacks
that can be interrupted by the watchdog.

The Python workspace rebuilds a deterministic dock layout on startup instead of
trusting any saved panel layout from a previous run. `Reset Layout` is the
explicit recovery action after interactive rearrangement. This keeps stale
`imgui.ini` state from obscuring the viewport or hiding panels when
scene-specific controls change. The layout builder runs before dockspace
submission so ImGui applies the initial `Demos`, `Simulation`, scene-panel, and
diagnostics placements as docked windows on the first rendered frames.
Dock layout initialization is viewer-lifetime state, not per-scene state, so a
user-resized splitter survives runtime demo switches until `Reset Layout` is
pressed. The default dock builder also clears ImGui no-resize flags from its
nodes so top, left, right, and bottom dock regions expose normal draggable
splitters.

### Build layout

`examples/demos/` builds the `dart-demos` executable via the shared
`dart_build_gui_example` helper. The scene registry is compiled in
`demos_scenes`; the branch-local panel and shared session/process model are in
`demos_memory_diagnostics`. The app links those libraries with `dart-io`,
`dart-collision-native`, and `dart-simulation` while the World implementation
still lives in that component.

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

- Not a scene editor (authoring/undo/redo/project I/O is `dartsim`'s role; see
  `dartsim_gui_simulator.md`).
- No renderer/backend changes; demos builds on `dart::gui`.
- No Python-side scene authoring API; `py-demos` is an examples workspace for
  playback, controls, diagnostics, and capture, not an editor.
- The examples diagnostics are not a general heap profiler, allocation hook, or
  GPU-memory surface. Their categories intentionally retain distinct scopes and
  must not be summed into process RSS.
