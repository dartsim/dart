# Phase 1 Brief: `dart-demos` Host Skeleton (C++)

Worker brief. Read `PLAN.md` + all `EVIDENCE-*.md` in this folder first.
Branch: work on `feature/dart6-consolidated-demos` (already created). Do not
push or open PRs. Local commits allowed, focused and imperative-style.

## Deliverable

New example `examples/demos/` building executable `dart-demos` (register in
`examples/CMakeLists.txt` under the OSG examples; follow the existing
per-example CMake boilerplate incl. `dart_build_example_in_source` in-source
path with `LINK_LIBRARIES dart dart-utils dart-utils-urdf dart-gui-osg`; set
output name `dart-demos`). C++17 (match existing examples), two-space indent,
camelCase functions, PascalCase classes, no cuddled braces.

**No changes outside `examples/` are allowed in this phase** (no `dart/`
library edits, no public header changes).

## Files

- `examples/demos/main.cpp` — CLI parse + host run.
- `examples/demos/DemoScene.hpp` — scene contract (below).
- `examples/demos/DemoHost.hpp/.cpp` — viewer/host: owns ImGuiViewer, world
  node lifecycle, scene switching, panels, capture.
- `examples/demos/Theme.hpp/.cpp` — ImGui style: port `applyModernDarkColors`
  + `applyModernDarkMetrics` palette/metrics documented in
  `EVIDENCE-dart7-demos.md` (skip docking-only bits; vanilla ImGui 1.92).
  Scale-aware via `ImGuiHandler::getGuiScale()`.
- `examples/demos/Registry.hpp/.cpp` — `std::vector<DemoScene> makeDemoScenes()`.
- `examples/demos/scenes/BoxesScene.cpp`, `RigidCubesScene.cpp`,
  `EmptyScene.cpp` — seed scenes ported from `examples/boxes`,
  `examples/rigid_cubes`, `examples/empty` (behavior parity; keyboard actions
  become both key handlers and panel buttons).

## Scene contract (DART 6 adaptation of DART 7 DemoSceneEntry)

```cpp
struct DemoSceneSetup {
  dart::simulation::WorldPtr world;                 // required
  std::function<void()> preStep, postStep;          // controller hooks
  std::function<void()> renderPanel;                // scene-specific ImGui
                                                    // content (host embeds it
                                                    // in the right panel)
  std::vector<KeyAction> keyActions;                // {key, label, callback}
  std::optional<CameraHome> cameraHome;             // eye/center/up
  bool enableShadows = true;
  std::vector<dart::dynamics::SimpleFramePtr> dragFrames;  // enableDragAndDrop
  std::function<void(DemoHostContext&)> onActivate; // optional extras
};
struct DemoScene {
  std::string id, title, category, summary;         // stable snake_case id
  std::function<DemoSceneSetup()> factory;          // lazy; may throw
};
```

`DemoHostContext` exposes viewer, world node, and log sink so scenes can
register extras (attachments etc.); host tracks everything registered through
it and removes it all on switch.

## Host behaviors (the hard requirements)

1. **Single window, runtime switching.** ImGuiViewer +
   `setThreadingModel(SingleThreaded)`. Per scene: `RealTimeWorldNode`
   subclass that invokes the scene's preStep/postStep. Switch = queued
   request; executed between frames (in the run loop, not mid-ImGui-render):
   deactivate + `removeWorldNode(old)`, remove scene widgets/attachments/
   drag-and-drops/event handlers, build new scene in try/catch.
2. **Never crash / soft-fail.** Factory throw → keep previous scene if any,
   else empty world; surface reason in navigator status line exactly in the
   spirit of: "Starting demo 'X'...", "Failed to start demo 'X': <what>",
   "Restored previous demo 'A' after 'B' failed: <reason>". Also wrap
   preStep/renderPanel calls in try/catch that disables the offending callback
   after logging (never terminate).
3. **Panels** (programmatic layout vs viewport work area, guiScale-aware,
   theme applied):
   - Left `Demos` navigator: search filter box + Clear, "Showing N/M",
     categories as collapsing headers (first-appearance order), selectable
     rows with tooltip summaries, "(starting)" marker, status line at top.
   - Top `Simulation` toolbar: Play/Pause, Step, Rebuild (re-run factory),
     Reset (restore initial world state via world->reset + re-run factory —
     document which), sim time, RTF (smoothed) readout, target RTF slider
     (0.1–4x, clamped), gravity toggle.
   - Bottom `Diagnostics`: collapsible log console (host log sink; scene
     failures land here), stats line (FPS, RTF min/smoothed/max, steps, sim
     time, #skeletons/#bodies/#dofs).
4. **CLI**: `--list-scenes` (print catalog grouped by category, exit 0),
   `--scene <id>`, `--cycle-scenes [--frames N]` (headless-friendly: advance
   through every scene N frames (default 30), exit 0 on success, nonzero on
   any factory failure), `--headless --shot <path> [--steps N]` (pbuffer
   offscreen capture exactly per the recipe in
   `EVIDENCE-gui-capabilities.md`; capture includes ImGui panels).
5. **Robustness gate**: `--cycle-scenes --frames 5` twice in-process (loop
   catalog twice) must pass; rapid switching must not leak world nodes
   (assert host's registered-node count returns to baseline after teardown).

## Verification (do all before reporting done)

```bash
pixi run config   # if needed
pixi run cmake --build build/default/cpp/Release --target dart-demos  # adjust target name if helper dictates
DISPLAY=:0 ./build/default/cpp/Release/bin/dart-demos --list-scenes
DISPLAY=:0 ./build/default/cpp/Release/bin/dart-demos --cycle-scenes --frames 10
DISPLAY=:0 ./build/default/cpp/Release/bin/dart-demos --headless --shot /tmp/demos_p1.png --steps 300
DISPLAY=:0 ./build/default/cpp/Release/bin/dart-demos --headless --shot /tmp/demos_p1_cubes.png --scene rigid_cubes --steps 300
pixi run lint
```

Report: what was built, deviations from this brief and why, verification
output summary, and the PNG paths for review. Do not mark done if any command
fails.
