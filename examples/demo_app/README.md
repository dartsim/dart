# DART Demo App

A single GUI browser that collects small, focused scenes in one place. Each scene is a self-contained library under `examples/demo_app/scenes/<name>` and plugs into a shared ImGui shell.

## Build and run

- Configure with `-DDART_BUILD_EXAMPLES=ON -DDART_BUILD_DEMO_APP=ON` (demo app is on by default when examples are enabled).
- Build: `cmake --build <build_dir> --target demo_app`
- Run from your build tree: `./examples/demo_app/demo_app`
- Controls: play/pause/step/reset the active scene, adjust playback speed, and capture screenshots or PNG sequences.

## Adding a new scene

1. Create a folder under `examples/demo_app/scenes/<your_scene>` with a `CMakeLists.txt`, `<SceneName>.hpp/.cpp`.
2. Derive from `dart::demo::Scene`; provide metadata, a world, and a `HookedWorldNode` (see existing scenes).
3. Export a factory (e.g., `createMyScene()`) and register it in `core/SceneRegistry.cpp`.
4. Keep assets scene-local; only share common items via `examples/demo_app/assets/` if absolutely needed.

The shell handles UI, camera presets, playback toggles, and capture so scenes can focus on demonstrating a specific feature.
