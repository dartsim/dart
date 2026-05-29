# DART Examples README

## Overview

DART's C++ examples are organized around one consolidated application plus a few
standalone programs:

- **`dart-demos`** (`examples/demos/`) — a single GUI application that hosts most
  examples as scenes. Pick a scene from the categorized sidebar and switch
  between them at runtime, without relaunching. This is the place to browse
  DART's capabilities.
- **`hello_world`** (`examples/hello_world/`) — kept standalone as the minimal,
  copy-pasteable template for setting up an executable, CMake-based C++ project
  that links DART. Every other interactive GUI example is now a scene in
  `dart-demos`. `gui_scene_diagnostics` is a headless GUI-descriptor inspector
  rather than a viewer scene.
- **Headless / CLI examples**: `csv_logger`, `headless_simulation`,
  `speed_test`, and `unified_loading` are console programs and stay standalone.

Filament (with GLFW3 and Dear ImGui) is the maintained visualization
implementation behind `dart::gui`. The standalone `dartsim` editor (top-level
`dartsim/`) is a separate authoring application and is not an educational
example.

## Run the Demos App

From inside the DART repo:

```bash
pixi run demos                     # launch the Demos app (interactive)
pixi run demos -- --scene boxes    # launch with a specific scene selected
```

In the app, use the **Demos** sidebar to switch scenes. Scenes are grouped into
ordered categories: Getting Started, Visualization, Rigid Body, Collision,
Constraints & Joints, Control & IK, Soft Bodies, and Robots.

`dart-demos` accepts the same options as other GUI programs (`--headless`,
`--frames`, `--screenshot <path>`, `--width`, `--height`, `--backend`,
`--perf-hud`). `--cycle-scenes` advances through every scene for a few frames and
exits; it backs the headless smoke test.

### Adding a scene

1. Add `examples/demos/scenes/<name>.cpp` defining
   `dart::gui::ApplicationOptions dart::examples::demos::make<Name>Scene();`
   (build the world, panels, gizmos, handlers, and camera, then return the
   options). See an existing scene such as `scenes/boxes.cpp` for the pattern.
2. Declare it in `examples/demos/scenes.hpp`.
3. Register it (id, title, category, summary, factory) in
   `examples/demos/registry.cpp`, placed within its category group.
4. Add the source to `examples/demos/CMakeLists.txt`.

## Run Other Examples

```bash
pixi run ex hello_world
pixi run ex csv_logger
```

The generic in-tree runner is `pixi run ex <target>`. GUI programs with extra
CMake requirements declare them in `scripts/run_cpp_example.py`.

## Build hello_world Standalone

`hello_world` demonstrates a minimal external project that links an installed
DART. Copy the directory to your workspace and build it:

```bash
cp -r examples/hello_world /your/workspace/hello_world
cd /your/workspace/hello_world
mkdir build && cd build
cmake ..
make
./hello_world
```

This project depends on an installed DART; make sure a compatible version of
DART is installed before building.
