# DART Examples README

## Overview

DART's C++ examples are organized around the DART 7 World demo app plus a few
standalone programs:

- **`dart-demos`** (`examples/demos/`) — a single GUI application for the DART 7
  World solver demos. Pick a scene from the categorized sidebar and switch
  between them at runtime, without relaunching.
- **`hello_world`** (`examples/hello_world/`) — kept standalone as the minimal,
  copy-pasteable template for setting up an executable, CMake-based C++ project
  that links DART. `gui_scene_diagnostics` is a headless GUI-descriptor
  inspector rather than a viewer scene.
- **Headless / CLI examples**: `csv_logger`, `headless_simulation`,
  `speed_test`, and `unified_loading` are console programs and stay standalone.

Filament (with GLFW3 and Dear ImGui) is the maintained visualization
implementation behind `dart::gui`. The standalone `dartsim` editor (top-level
`dartsim/`) is a separate authoring application and is not an educational
example.

## Run the Demos App

From inside the DART repo:

```bash
pixi run demos                                  # launch the Demos app
pixi run demos -- --scene dynamic_joint_constraints
```

In the app, use the **Demos** sidebar to switch scenes. Scenes are grouped into
ordered World-solver categories: World Rigid Body, IPC Deformable, Vertex Block
Descent, and Planned World Ports. The planned ports are launchable placeholders
for high-value DART 6 concepts that still need World-native implementations
(IK, SIMBICON walking, operational-space control, robot puppets, collision
sandbox, and mobile manipulation); they do not keep the removed DART 6 scene
code.

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

## Run Other Examples

```bash
pixi run ex hello_world
pixi run ex csv_logger
pixi run ex cylindrical_constraint
```

The generic in-tree runner is `pixi run ex <target>`. GUI programs with extra
CMake requirements declare them in `scripts/run_cpp_example.py`.

## Dynamic Joint Constraint Examples

`dart-demos` includes a Dynamic Joint Constraints scene cataloging the runtime
dynamic `BallJointConstraint`, `RevoluteJointConstraint`,
`CylindricalJointConstraint`, and `WeldJointConstraint` APIs side by side.

The standalone `cylindrical_constraint` console example is a focused smoke test
for `dart::constraint::CylindricalJointConstraint`. It keeps two bodies on a
shared axis while leaving translation along that axis and rotation about that
axis free.

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
