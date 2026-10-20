# DART Examples README

Most of DART's former C++ GUI examples now have Python scenes in one
consolidated application, `python/examples/demos/`, so there is a single place
to browse and run them. A handful of C++ examples remain standalone because
they serve a purpose the consolidated catalog does not: a minimal "first
program", a console-only regression check,
a benchmark, and a CI-load-bearing microbenchmark harness.

## `demos`: the consolidated example catalog

The Python catalog runs on the native Filament/ImGui app host through
`dartpy.viewer`. It bundles the former single-purpose GUI examples
(add/delete skeletons, box stacking, rigid chains/cubes/loops, joint
constraints, humanoid controllers, soft bodies,
drag-and-drop, IK, and more) as selectable scenes in one window.

Build dartpy and run the catalog:

    $ pixi run demos

The UI automatically follows desktop DPI, and the default window uses the
primary monitor's content scale. Use View > **GUI scale** or `--gui-scale 1.5`
for a manual override; see [display scaling](../docs/readthedocs/dart/user_guide/visualization.rst)
for platform behavior.

The flagship deformable-body scenes can be opened directly:

    $ pixi run demos -- --scene adaptive_soft_contact
    $ pixi run demos -- --scene soft_worm

Open the former standalone soft-body examples by scene id:

    $ pixi run demos -- --scene soft_bodies --collision-detector dart --threads 16
    $ pixi run demos -- --scene soft_cubes --collision-detector fcl --threads 1
    $ pixi run demos -- --scene soft_open_chain --collision-detector dart --threads 4

Use `dart` or `fcl` for apples-to-apples soft-body comparisons. The
consolidated `dart` detector covers `SoftMeshShape` in these scenes.

For a manual source build, configure with `-DDART_BUILD_DARTPY=ON` and
`-DDART_USE_SYSTEM_IMGUI=OFF`, then build dartpy and run the package from
`python/` (the default `DART_BUILD_GUI=ON` needs a Filament SDK):

    $ cmake --build build --target dartpy
    $ cd python
    $ PYTHONPATH=../build/python python -m examples.demos

Useful flags (`pixi run demos -- --help` for the full list):

    --list-scenes            Print the catalog grouped by category and exit.
    --scene <id>              Start on a specific scene (e.g. --scene boxes).
    --cycle-scenes            Headless smoke test: step every scene and exit.

`pixi run cycle-demos` runs that smoke test without a graphics context.

Each scene's source lives under `python/examples/demos/scenes/`; scene ids match
their original standalone example names (e.g. `scenes/boxes.py` is
`--scene boxes`), except `wam_analytical_ik`, which replaced `wam_ikfast`.

## Adding catalog scenes

Add scenes under `python/examples/demos/scenes/` and register them in
`registry.py`. Use `dartpy.viewer.Scene` factories returning `SceneSetup`;
declare a `Panel`, `KeyAction`s, and camera home in the setup. C++ draws the
controls and validates edits. Put light simulation hooks in `pre_step` or
`post_step`, and per-frame logic in `pre_refresh` so it runs even while the
scene tab is hidden. Use `SceneContext` for native attachments, event
handlers, drag and drop, and teardown. Demo controllers, sensors, histories,
and model helpers belong in `python/examples/demos/models/` and use the engine,
IO, and GUI bindings. Expose missing library APIs in their regular bindings.
Demo implementation lives entirely in Python.

Preserve each port's physical setup, controls, and coordinate convention.
Use `scenes/common.py`'s `reorient_z_up` for Y-up SKEL models that need the
catalog's Z-up view; vehicle, Atlas SIMBICON, and soft-foot SIMBICON retain
their Y-up worlds. The humanoid factories `atlas_world` and `g1_world` keep
returning Worlds for `agent-capture --factory`.

The soft-foot SIMBICON experiment explicitly uses unbounded joint velocities
for both rigid and soft variants, preserving its historical push-recovery
settings. General SDF loading honors the velocity limits authored in each model.

Gate registration on optional binding availability: `point_cloud` needs
`VoxelGridShape`; the humanoids need `ModelResourceRetriever`. A factory
raises `AssetsNotPrefetchedError` for unavailable cached robot models or the
optional `ssik` package, so catalog cycling reports a skip. A source build
with assets and OctoMap supports 37 scenes; a build without the asset
retriever omits Atlas v5 and Unitree G1. Wheels include the host, while the
catalog stays in the source tree. Its wheel smoke test uses a small test-owned
scene.

The WAM scene uses ssik's closed-form analytical IK through
`setPythonAnalytical`. Like `ssik_ik_gui`, it is an optional-ssik scene that
skips when ssik is unavailable.
The unbuilt TinyDNN-only `human_joint_limits` scene and the duplicate
`hello_world_gui` and `contacts_pointcloud` selectors are retired.

Follow [catalog verification](../docs/onboarding/testing.md#demo-catalog-verification)
for construction, behavior, and assessed capture evidence.

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
  It builds without the GUI; its `--gui` modes need `DART_BUILD_GUI=ON`.

## Build Instructions

This project is dependent on DART. Please make sure a proper version of DART
is installed before building this project. `hello_world` needs the Filament
`gui` component, built with `DART_BUILD_GUI=ON` (the default). CMake detects its
installed `dart/gui/Viewer.hpp` header independently of the reported DART
version. Without it, including with an older GLUT GUI, `hello_world` is skipped
and the other examples still build. Configuring a GUI example individually
requires the Filament component and fails clearly when it is absent.
`contact_benchmark` still builds without it, but its GUI modes are unavailable.

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
