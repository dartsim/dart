# Demos App — Architecture & Decisions

Durable architecture for consolidating DART's GUI examples into a single
standalone **Demos** application. While this task is active, this file is the
architecture owner; on completion its durable content is promoted to
`docs/design/demos_app.md` and this folder is deleted (see
`docs/dev_tasks/README.md`).

## Problem

DART's C++ examples have grown to ~43 directories under `examples/`, and the
same scene exists in up to **three** independent representations:

1. A standalone educational `examples/<name>/main.cpp` that builds its own
   `dart::gui::ApplicationOptions` and calls `runApplication(...)` — one
   executable per example (~39 GUI executables).
2. A private `createXxxScene()` in `dart/gui/detail/scenes.cpp` (816 lines,
   `enum class ExampleScene`), reached via `runApplication(argc, argv,
"<scene>")` / `--scene <name>`, that rebuilds an equivalent `visual_*`
   fixture world.
3. A registered headless smoke + renderable-count test per scene
   (`EXAMPLE_dartsim_<scene>_headless_smoke`,
   `tests/unit/gui/test_filament_scene_extraction.cpp`, the `kXxxFixture*`
   constants), plus a `GUI_SCENE_EXAMPLE_DEFAULT_ARGS` entry in
   `scripts/run_cpp_example.py`.

The result: adding or changing one example touches several files; the `--scene`
catalog is flat, private to `dart-gui`, and selectable only at launch (one scene
per process); and users must build/run many executables to survey DART's
capabilities. This does not scale.

## Goal

One standalone app — **Demos** (`dart-demos`) — that hosts every GUI example as a
small, focused, registered scene. Users pick a scene from a categorized, ordered
sidebar and switch between scenes at runtime in a single window. Each scene
defines only its own focus (world + controls); all common chrome (window,
camera, simulation controls, grid, picking) is shared. Each scene is the
**single source of truth**, consumed by both the app and the GUI tests.

`hello_world` stays a standalone, copy-paste-buildable CMake/C++ project — the
minimal template for "how to set up an executable that links DART."

## Non-Goals

- Not a scene editor. Authoring/undo/redo/project I/O on the experimental World
  is `dartsim`'s job (PLAN-101). Demos runs curated, code-defined scenes on the
  legacy `dart::simulation::World` rendering path the examples already use.
- No new renderer/backend work; build on `dart::gui` (`ApplicationOptions`,
  `runApplication`, `PanelBuilder`, gizmos, orbit camera).
- The four headless/CLI examples (`csv_logger`, `headless_simulation`,
  `speed_test`, `unified_loading`) are not visual and stay separate CLI examples
  (see Open Questions).

## Naming

The pattern (one app, a categorized list of demos picked from a sidebar) is
common; the chosen name follows the user's instruction to use **Demos**. Prior
art differs by engine: Bullet calls it the **Example Browser** (the dartsim
design doc already rejected "browser" as it reads as an asset/model browser),
Box2D the **Testbed**, Jolt/PhysX **Samples**, ODE and Project Chrono **demos**,
MuJoCo **simulate** / **Playground**.

**Chosen: Demos.** Directory `examples/demos/`, executable `dart-demos`, task
`pixi run demos`, namespace `dart::examples::demos`.

## Architecture

### Layering (mirrors `dartsim/` proportionally)

```
examples/demos/
  scenes/                 scene library: one focused file per example
    <category>/<name>.cpp  -> DemoScene makeXxxScene();
    registry.cpp           -> ordered list of all scenes, grouped by category
    demo_scene.hpp         -> DemoScene / SceneSetup / Category types
  app/main.cpp            thin entry: registry -> dart::gui::runDemos(...)
  CMakeLists.txt          builds `demos_scenes` (static lib) + `dart-demos`
```

Splitting `scenes` (a library) from `app` (a single `main.cpp`) lets the GUI
tests link `demos_scenes` without the executable, the same reason
`dartsim/engine` is separate from `dartsim/app`.

### Scene module interface

A scene contributes data, not an entry point. `SceneSetup` is the per-scene
subset of `ApplicationOptions`; common chrome stays in the host.

```cpp
namespace dart::examples::demos {

enum class Category {       // ordered for display
  GettingStarted,
  Visualization,
  RigidBody,
  Robots,                   // articulated robots & humanoids
  ControlAndIk,
  ConstraintsAndJoints,
  SoftBodies,
  Collision,
};

// Per-scene subset of dart::gui::ApplicationOptions (no window/argv concerns).
struct SceneSetup {
  dart::simulation::WorldPtr world;
  std::optional<dart::gui::OrbitCamera> camera;
  bool simulateWorld = true;
  std::vector<dart::gui::Panel> panels;
  std::vector<dart::gui::Gizmo> gizmos;
  std::vector<dart::gui::InverseKinematicsHandle> ikHandles;
  std::vector<dart::gui::BodyNodeDragHandle> bodyNodeDragHandles;
  std::vector<dart::gui::KeyboardAction> keyboardActions;
  std::function<void()> preStep, postStep;
  std::function<std::vector<dart::gui::RenderableDescriptor>()> renderableProvider;
  dart::gui::RenderSettings renderSettings;
};

struct DemoScene {
  std::string id;            // "boxes" (stable; used by tests + CLI)
  std::string title;         // "Boxes"
  Category category;
  int order = 0;             // order within category
  std::string summary;       // one line, shown in the sidebar
  std::string help;          // longer text, shown in the scene's info panel
  std::function<SceneSetup()> build;   // lazy: constructed on first selection
};

std::vector<DemoScene> registerDemoScenes();   // registry.cpp

} // namespace dart::examples::demos
```

`build` is a factory (lazy): heavy/asset-backed scenes (atlas, hubo, fetch, g1,
wam) are only constructed when first selected, so launch is fast and a missing
asset fails one scene, not the app. Per-scene state (e.g. the Simbicon
controller, recorders) is owned by the closure/`SceneSetup`, so switching scenes
destroys it cleanly.

Registration is an **explicit ordered list** in `registry.cpp` (not
static-init self-registration) to keep ordering deterministic and avoid
static-initialization-order pitfalls across translation units.

### Runtime scene switching (new `dart::gui` capability)

Today `runApplication` builds one `DartScene` and runs the frame loop until exit
(`dart/gui/detail/application.cpp`). Demos needs to swap the active scene without
recreating the window. The implementation restructures the backend loop into an
**outer scene loop** around the existing **inner frame loop**:

- The window, Filament engine/view/scene, materials, ImGui overlay, and lights
  are created once and persist.
- The scene-bound state (`DartScene`, `InitialSceneState`/renderables,
  `SceneFrameUpdater`, camera home) is built per active scene. When a switch is
  requested, the inner loop breaks, the scene renderables/overlays are destroyed,
  and the outer loop rebuilds from the newly selected scene factory.
- A new `ViewerLifecycleState::sceneSwitchRequested` (+ requested id) carries the
  request from the sidebar to the loop.
- A host helper `dart::gui::runDemos(argc, argv, DemoCatalog)` accepts the
  ordered scene list (id/title/category/order/summary + a factory producing
  `ApplicationOptions`) and an initial selection (CLI `--scene <id>` or the first
  entry). It owns the built-in **Demos** sidebar panel: categories as collapsing
  headers, scenes as selectable rows, a search box, and the active scene's
  summary/help.

The factory returns a `dart::gui::ApplicationOptions` (the public per-scene
surface), so scene content lives in `examples/demos` and never depends on the
private `ExampleScene` enum.

Start with **system ImGui + a simple fixed left panel** (no docking) to avoid the
`DART_USE_SYSTEM_IMGUI` build-flip friction documented for `dartsim`; adopting
the dockable workspace later is a follow-up, not a blocker.

### Categories (ordered)

`GettingStarted → Visualization → RigidBody → Robots → ControlAndIk →
ConstraintsAndJoints → SoftBodies → Collision`. This extends the ordering
already sketched in `examples/README.md`. Each scene sets `category` + `order`.

### Assets and offline behavior

- Local models resolve through the existing resource retriever / `DART_DATA_PATH`
  used by the current examples.
- Remote models (`g1_puppet`) load lazily on selection and fail **soft**: the
  scene shows an error/help message in its panel instead of crashing the app.
- Headless test runs only exercise scenes whose assets are present in-tree.

## Single-source test strategy

The Demos registry becomes the one source of scene content; tests consume it
instead of the deleted `dart/gui/detail` catalog:

- The Filament renderable-extraction test and the per-scene headless screenshot
  smokes link `demos_scenes` and iterate the registry (id → expected renderable
  floor). The `kXxxFixture*Count` expectations move next to the scenes they
  describe.
- `dart-gui` keeps **one** tiny internal scene (the current `Mvp`) for a pure
  renderer self-smoke that must not depend on `examples/`. Everything else in
  `dart/gui/detail/scenes.{hpp,cpp}` — the `ExampleScene` enum, `createDartScene`
  example branches, `parseSceneName`, and the `runApplication(argc, argv,
const char*)` / `--scene` example overloads — is removed.
- Layering note: GUI tests linking `examples/demos` is acceptable because they
  are example-rendering tests; the library's own smoke stays independent of
  `examples/`.

## Backward-compatibility breaks (approved)

Clean consolidation over compatibility (per task decision):

- Per-example executables are removed; `pixi run ex <gui-name>` is replaced by
  `pixi run demos` (and `pixi run demos --scene <id>` headless). `pixi run ex
hello_world` stays.
- `--scene` on the old viewer path and the `ExampleScene` catalog are removed
  (replaced by the Demos `--scene <id>` selection over the registry).
- `scripts/run_cpp_example.py` (`GUI_SCENE_EXAMPLE_DEFAULT_ARGS`,
  `FILAMENT_*_SCENES`, smoke-name mapping), the per-example pixi tasks,
  `examples/CMakeLists.txt`, `examples/README.md`, the `dartsim/README.md`
  cross-references, and any dartpy docs that name C++ example binaries are
  updated to the Demos model.

## Migration sequencing

1. **Scaffold + prove the pattern.** Add the `dart::gui` runtime scene-swap host
   (`runDemos`) + Demos sidebar; create `examples/demos/` (types, registry, app
   shell). Seed 3 scenes across categories (hello-world-as-scene, boxes, one
   robot) and a headless smoke that switches scenes. Exit: `dart-demos` launches,
   lists categories, switches scenes without leaks; smoke green.
2. **Migrate scenes by category**, folding each standalone example's focused
   behavior into its module and deleting the standalone dir as it lands. Prefer
   reusing the richer standalone `main.cpp` logic over the terser
   `createXxxScene` fixture where they differ.
3. **Move tests to the registry**; delete `dart/gui/detail/scenes.{hpp,cpp}`
   example content and the `--scene` overloads; keep the `dart-gui` MVP smoke.
4. **Update tooling/docs**: `run_cpp_example.py`, pixi tasks, `examples/README`,
   `dartsim/README`, dartpy example docs, `CHANGELOG.md`.
5. **Complete**: promote this design to `docs/design/demos_app.md` (register it
   in `docs/design/README.md`), add a brief Demos note to
   `docs/onboarding/gui-rendering.md`, and delete this dev-task folder in the
   completing PR.

## Risks & mitigations

- **Scene-switch teardown leaks / dangling handles** → each scene owns its state
  via the factory closure; the host fully clears the previous scene before
  building the next; verify with a switch-stress headless run.
- **Stateful controllers** (Simbicon, recorders) → constructed inside `build`,
  destroyed on switch; no global state.
- **Asset/remote failures** → lazy build + soft-fail into a panel.
- **ImGui system/docking flip** → ship non-docking first; docking is a follow-up.
- **Renderable-count test churn** → expectations move next to scenes; a small
  per-scene floor is more robust than exact counts.

## Open questions

- Headless CLI examples (`csv_logger`, `headless_simulation`, `speed_test`,
  `unified_loading`): keep as standalone CLI examples (recommended) or represent
  them in Demos as no-viewport entries with a console output panel?
- Dockable workspace now or as a follow-up (recommended: follow-up).
