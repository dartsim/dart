# DART Examples README

## Overview

DART's C++ examples are organized around the DART 7 World demo app:

- **`dart-demos`** (`examples/demos/`) — a single GUI application for the DART 7
  World solver demos. Pick a scene from the categorized sidebar and switch
  between them at runtime, without relaunching.
  Filament (with GLFW3 and Dear ImGui) is the maintained visualization
  implementation behind `dart::gui`. The standalone `dartsim` editor (top-level
  `dartsim/`) is a separate authoring application and is not an educational
  example.

## Run the Demos App

From inside the DART repo:

```bash
pixi run demos                                  # launch the Demos app
pixi run demos -- --list                        # list scenes without opening the GUI
pixi run demos -- --scene rigid_body
```

In the app, use the **Demos** sidebar to switch scenes. Scenes are grouped into
ordered World-solver categories: World Rigid Body, IPC Deformable, Vertex Block
Descent, and Planned World Ports. The planned ports are launchable placeholders
for high-value DART 6 concepts that still need World-native implementations
(IK, SIMBICON walking, operational-space control, robot puppets, and mobile
manipulation); they do not keep the removed DART 6 scene code. The old
collision sandbox placeholder is retired; use the concrete Python GUI rows
`rigid_contact_inspector`, `rigid_collision_query_options`, and
`rigid_collision_casts` for collision-debugging workflows.

`dart-demos` accepts the same options as other GUI programs (`--headless`,
`--frames`, `--screenshot <path>`, `--width`, `--height`, `--backend`,
`--perf-hud`). `--cycle-scenes` advances through every scene for a few frames and
exits; it backs the headless smoke test.

### Adding a scene

1. Add `examples/demos/scenes/<name>.cpp` defining
   `dart::gui::ApplicationOptions dart::examples::demos::make<Name>Scene();`
   (build the world, panels, gizmos, handlers, and camera, then return the
   options). See an existing scene such as `scenes/rigid_body.cpp` for the
   pattern.
2. Declare it in `examples/demos/scenes.hpp`.
3. Register it (id, title, category, summary, factory) in
   `examples/demos/registry.cpp`, placed within its category group.
4. Add the source to `examples/demos/CMakeLists.txt`.

## Run C++ Examples

```bash
pixi run ex demos -- --scene rigid_body
```

The generic in-tree runner is `pixi run ex <target>`. The old standalone
DART 6 whole-World loading examples were retired from `main`; use a release-6.\*
branch if you need their parity source.
