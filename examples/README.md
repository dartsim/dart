# DART Examples README

Most of DART's C++ example programs live in one consolidated application,
`demos`, so there is a single place to browse and run them. A handful of
examples remain standalone because they serve a purpose the consolidated
catalog does not: a minimal "first program", a console-only regression check,
a benchmark, and a CI-load-bearing microbenchmark harness.

## `demos`: the consolidated example catalog

`demos/` hosts `dart-demos`, an OSG/ImGui application that bundles every
former single-purpose GUI example (add/delete skeletons, box stacking, rigid
chains/cubes/loops, joint constraints, humanoid controllers, soft bodies,
drag-and-drop, IK, and more) as selectable scenes in one window.

Build and run it:

    $ pixi run demos

Soft-body scene aliases are also available for compatibility with the former
standalone examples:

    $ pixi run ex soft_bodies -- --collision-detector dart --threads 16
    $ pixi run ex soft_cubes -- --collision-detector fcl --threads 1
    $ pixi run ex soft_open_chain -- --collision-detector native --threads 4

or, from a manual build directory:

    $ cmake --build build --target dart-demos
    $ ./build/bin/dart-demos

Useful flags (`./dart-demos --help` for the full list):

    --list-scenes            Print the catalog grouped by category and exit.
    --scene <id>              Start on a specific scene (e.g. --scene boxes).
    --cycle-scenes            Headless smoke test: step every scene and exit.

Each scene's source lives under `demos/scenes/`; scene ids match their
original standalone example names (e.g. `demos/scenes/BoxesScene.cpp` is
`--scene boxes`).

## Standalone examples

These are kept outside `demos` because folding them in would not simplify
anything:

- **`hello_world`** — the canonical minimal "first program"; start here if
  you are new to DART.
- **`cylindrical_constraint`** — a headless, console-only convergence check
  for `dart::constraint::CylindricalJointConstraint`; not a GUI demo, so it
  does not fit the `demos` catalog.
- **`speed_test`** — a console performance benchmark.
- **`contact_benchmark`** — backs `tests/benchmark/integration/` and the
  performance-dashboard CI workflow; its header is compiled directly by
  those targets, so it cannot be removed or relocated without touching CI.

## Build Instructions

This project is dependent on DART. Please make sure a proper version of DART
is installed before building this project.

Copy this directory to your workspace (e.g., in Linux):

    $ cp -r examples /your/workspace/directory/dart_examples
    $ cd /your/workspace/directory/dart_examples

From the workspace directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch an executable from the build directory above (e.g.):

    $ ./hello_world

Follow the instructions detailed in the console, or see each example's own
`README.md` for details.
