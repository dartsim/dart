# dart-demos

`dart-demos` is the consolidated DART 6 demo application: a single
`dart::gui::osg::ImGuiViewer` host that runs every GUI demo as a
runtime-switchable *scene*, selected from a categorized navigator. It replaces
the scattered per-example programs that used to live under `examples/*`.

Run it with `pixi run demos` (or `pixi run demos --scene <id>`). The dartpy
counterpart is `pixi run py-demos` (see `python/examples/demos/`).

## Architecture

- **Host** (`DemoHost`, `main.cpp`): owns the one window, the ImGui theme
  (`Theme.*`), the scene registry, and the persistent chrome (simulation
  toolbar, `Demos` navigator, `Diagnostics` panel, and the visual-debugging
  panels). It swaps the active scene at runtime.
- **Registry** (`Registry.cpp`, `Scenes.hpp`): `makeDemoScenes()` returns an
  ordered `std::vector<DemoScene>`. A `DemoScene` is data — `{id, title,
  category, summary, factory}` — where `factory` is a lazily-invoked
  `std::function` that builds the scene only when it is first selected.
  Categories render in first-appearance order; scenes in registry order
  within a category.
- **Scenes** (`scenes/*.cpp`, one factory each; multi-file scenes in
  subdirectories): each factory returns a `DemoSceneSetup` (`DemoScene.hpp`) —
  the world, optional `preStep`/`postStep`/`preRefresh` controller hooks, a
  per-scene ImGui `renderPanel`, `KeyAction`s (auto-mirrored as panel
  buttons), a camera home, drag frames, and an `onActivate` hook for
  viewer-level extras. Per-scene mutable state lives in a `shared_ptr`
  captured by the lambdas.
- **Visual debugging** (`Inspector`, `ContactVisualizer`, `DragForce`,
  `LogCapture`, `Profiler`): host-level, scene-agnostic facilities wired into
  the composed step/refresh hooks and the Diagnostics panel.

## Adding a scene

1. Write `scenes/MyScene.cpp` with a `dart_demos::makeMyScene()` factory
   returning a `DemoSceneSetup` (copy an existing scene as a template;
   `RigidCubesScene.cpp` is a good controller+visual example).
2. Declare it in `Scenes.hpp` and register it in `Registry.cpp`.
3. The executable globs `scenes/*.cpp`, so no CMake edit is needed for a
   single-file scene. Multi-file scenes and optional-dependency scenes are
   added in `CMakeLists.txt` (see the `wam_ikfast`, `atlas_simbicon`,
   `human_joint_limits`, and `ssik_ik_gui` blocks).

## Host conventions (enforced in review)

These keep the app crash-safe and consistent — a demo must **never crash**,
whatever the user changes at runtime:

- **Clamp every tunable.** Every `ImGui::SliderFloat`/`InputFloat` passes
  `ImGuiSliderFlags_AlwaysClamp` *and* commits through a local copy guarded by
  `std::clamp` + `std::isfinite` (ImGui's Ctrl+click text entry bypasses the
  drag range, so `AlwaysClamp` alone is not enough — a typed NaN must never
  reach physics or geometry).
- **World-owned visuals only.** Debug geometry is a `SimpleFrame`/`ArrowShape`
  added to the scene's world (torn down with it), never a raw node injected
  into the OSG graph.
- **Full teardown.** Anything registered through `DemoHostContext`
  (attachments, event handlers, drag-and-drop) is released via
  `ctx.addTeardown`. Facilities that hold `BodyNode*`/`Skeleton*` across a
  possible mid-scene deletion derive from `dart::common::Observer` and drop
  the pointer in `handleDestructionNotification` (see `Inspector`,
  `DragForce`).
- **Frame-loop thread only.** All world mutation happens on the frame-loop
  thread (the viewer runs `SingleThreaded`); panel edits apply in
  `preStep`/`preRefresh` or via queued commands, never mid-render.
- **Re-read `dt` per step.** Controllers that use the timestep read
  `world->getTimeStep()` each step — the toolbar Timestep control changes it
  live.
- **Lowercase keys, ImGui-gated.** Key actions register lowercase (the host
  matches case-insensitively) and skip when
  `ImGui::GetIO().WantCaptureKeyboard` (typing in the search box must not
  drive a scene).
- **Z-up.** Y-up `.skel` worlds are reoriented via `scenes/ZUp.hpp`, which
  preserves the source gravity magnitude; force/velocity vectors and their
  UI labels are remapped together.

## CLI

- `--list-scenes` — print the catalog grouped by category, exit 0.
- `--scene <id>` — start on a specific scene.
- `--cycle-scenes [--frames N]` — headless: build every scene, step N frames
  each, twice (a leak/robustness audit); exit nonzero on any factory failure.
- `--headless --shot <path> [--steps N]` — off-screen pbuffer capture
  (requires a DISPLAY/GPU; a local self-verification tool, not a CI gate).
- `--debug-select-body <name>` / `--debug-record-profile` — hidden hooks that
  let a headless capture exercise the inspector/profiler panels.
