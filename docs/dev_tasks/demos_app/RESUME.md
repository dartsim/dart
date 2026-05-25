# Resume: Demos App

## Last Session Summary

Executing the consolidation of ~39 GUI examples into one standalone **Demos** app
(`dart-demos`), keeping `hello_world` standalone. Working on branch `demos-app`
with checkpoint commits. Planning docs renamed Gallery→Demos.

## Current Branch

`demos-app` — checkpoint commits in progress.

## Immediate Next Step

Phase 1: add `ViewerLifecycleState::sceneSwitchRequested`, restructure
`dart/gui/detail/application.cpp` (`runGuiBackendApplicationImpl`) into an outer
scene loop around the inner frame loop, add `dart::gui::runDemos` + a Demos
sidebar panel, then scaffold `examples/demos/` (`demo_scene.hpp`, `registry.cpp`,
`app/main.cpp`, CMake building `demos_scenes` + `dart-demos`) with 3 seed scenes
and a scene-switching headless smoke.

## Context That Would Be Lost

- **Triple duplication is the core motivation**: each example exists as a
  standalone `examples/<name>/main.cpp`, a `createXxxScene()` in
  `dart/gui/detail/scenes.cpp` (the `--scene` catalog), and a registered
  smoke/renderable test. Demos makes one `DemoScene` the single source for all
  three.
- **Runtime swap mechanism**: restructure `runGuiBackendApplicationImpl` so the
  window/engine/view/materials/imgui/lights are created once and the scene-bound
  state (`DartScene`, `InitialSceneState` renderables, `SceneFrameUpdater`) is
  rebuilt per active scene; the `dartsim` editor's per-frame `renderableProvider`
  swap proves runtime mutation is supported.
- **Decisions locked**: name = Demos; break backward compatibility freely
  (`--scene` enum, per-example binaries, `pixi run ex <gui-name>` all go); single
  source of truth; lazy scene factories returning `dart::gui::ApplicationOptions`;
  non-docking system-ImGui sidebar first.
- **Test layering**: keep one tiny `dart-gui` internal MVP smoke independent of
  `examples/`; move the example renderable-count expectations
  (`tests/unit/gui/test_filament_scene_extraction.cpp`, `kXxxFixture*Count`) next
  to the scenes and link `demos_scenes`.
- **Tooling to update**: `scripts/run_cpp_example.py`
  (`GUI_SCENE_EXAMPLE_DEFAULT_ARGS`, `FILAMENT_*_SCENES`), pixi `ex`/per-example
  tasks, `examples/README.md`, `dartsim/README.md`, dartpy example docs.
- **Open questions**: fate of the 4 headless CLI examples; when to adopt the
  dockable workspace.

## How to Resume

```bash
git checkout demos-app
git status && git log -8 --oneline
```

Then: read `01-design.md`, check the latest checkpoint commit, and continue the
current phase from "Immediate Next Step".
