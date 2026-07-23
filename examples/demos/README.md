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

## Memory diagnostics

Memory diagnostics are compiled out by default so ordinary `dart-demos` and
scene-alias executables contain no diagnostics session, environment probe,
Memory tab, per-frame branch, collector code, or diagnostics-only platform
link. Enable the feature explicitly in the existing Pixi-configured build:

```bash
pixi run config
pixi run cmake -S . -B build/default/cpp/Release \
  -DDART_BUILD_DEMOS_MEMORY_DIAGNOSTICS=ON
pixi run cmake --build build/default/cpp/Release --target dart-demos
```

The enabled build adds a persistent **Memory** tab for opt-in process and
active-World inspection. Collection remains off until it is enabled in the tab,
or the application is launched with `DART_DEMOS_MEMORY_DIAGNOSTICS=1` to start
enabled and select the tab during a headless capture. The sample interval is
adjustable from 0.1 to 5 seconds (default 0.5 seconds); World traversal and
formatting occur only on a due or manually requested sample. A compiled-in
build with collection unchecked still renders the diagnostics UI integration;
the strict zero-overhead guarantee applies to the default compile-time-OFF
configuration.

The tab deliberately separates values with different scopes:

- **Process resident memory** is the OS-reported current resident set,
  process-lifetime peak, and peak observed by this sampled session. It includes
  OSG/ImGui, collision backends, libraries, assets, allocator-retained pages,
  and the diagnostics buffers themselves; it is not DART-owned heap
  attribution. A scene switch can briefly overlap the outgoing and candidate
  Worlds, so the process-lifetime peak may include both.
- **DART 6 graph counts** traverse the active World and report Skeletons,
  rigid/soft BodyNodes, PointMasses, Joints, degrees of freedom, ShapeNodes,
  unique Shape objects, SimpleFrames, collision-group ShapeFrames, last-step
  contacts, and manually registered constraints. The last item is not the full
  set of solver-generated/contact constraints.
- **World MemoryManager reservation arena** reports the frame arena's
  capacity/use/overflow and pool backing blocks. Its exact address map draws
  each real free-list backing chunk, frame arena, and frame overflow allocation
  independently, with relative byte offsets and a complete allocator-state
  partition. The World reserves and resets these arenas, but current classic
  DART 6 solver paths do not allocate their scratch from them. These rows
  therefore do not measure legacy solver temporaries or the
  Skeleton/BodyNode/Joint/Shape graph.
- **Known object shallow-size floor** adds `sizeof` only when a traversed
  object's runtime type exactly matches a known concrete type. It is an
  estimate that omits derived objects without an exact match,
  nested/container capacity, shared-pointer control blocks, collision-backend
  state, alignment, and other allocations.
- **Typed classic-object address atlas** orders the traversed Skeleton,
  BodyNode, Joint, degree-of-freedom, Shape, PointMass, SimpleFrame, and manual
  constraint observations by virtual address. Contiguous host-page runs use the
  runtime page-size query. Colored ranges are `sizeof`-based shallow lower
  bounds; dotted gaps are unobserved address space, never allocator-free
  memory. The atlas does not establish complete allocation extents, allocator
  ownership, physical placement, fragmentation, cache misses, prefetch
  behavior, or execution speed. Normal demo builds do not interpose global
  allocation functions, so active heap allocation counts/bytes remain
  explicitly unavailable rather than zero.

The map legends use hue for semantic data category and hatch, border, opacity,
and text for storage state. Increase **Address-map detail rows / region** to
resolve smaller spans without changing their address order. Each region keeps
up to 48 logical rows in an inner scroll area with about eight visible at once;
offscreen primitives are culled and each region has a conservatively bounded
draw list for the 16-bit-index OpenGL2 backend. Raw process addresses are
capture-local and never shown; offsets are relative to each exact region or
host-page run. A collapsed exact-range table exposes the same labels, extent
evidence, and limitations without requiring pointer hover.

The runtime host page size groups the object atlas into virtual-page runs, but
this version does not draw page separators. Allocator-region page placement and
all cache-line placement are unavailable because the snapshot does not retain
the required scrubbed base-address remainders or a host cache-line size. Atlas
pages are virtual buckets, not proof of residency or physical contiguity; no
raster element measures memory accesses or cache misses.

**Capture now** records a sample and makes it the comparison baseline; **Set
baseline** uses the latest sample. Compatible rows show current-minus-baseline
deltas. **Reset** clears the latest sample, baseline, bounded history, and
session-observed peak while retaining the allocated history ring; it cannot
reset the OS process-lifetime peak. Scene changes start a new generation so
deltas never silently compare different Worlds. Sampling continues at the
configured cadence while another scene-panel tab is active.

Linux reads `/proc/self/status` (`VmRSS`/`VmHWM`), Windows uses
`GetProcessMemoryInfo` working-set counters, and macOS uses Mach task resident
size. Missing platform values stay unavailable because the APIs differ in
accounting and update timing.

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
- `--collision-detector <name>` / `COLLISION_DETECTOR=<name>` — start a
  scene with a specific registered backend (`fcl`, `dart`, `native`, `bullet`,
  or `ode` when available). The toolbar can switch backends while the scene is
  running. Soft-body scenes should use `dart` or `fcl` for apples-to-apples
  comparisons; the `native` adapter is selectable for diagnostics, but does not
  yet cover `SoftMeshShape`.
- `--threads <n>` / `THREADS=<n>` — start with a specific simulation worker
  count (`0` selects hardware concurrency). The toolbar can change this live.
- `--debug-select-body <name>` / `--debug-record-profile` — hidden hooks that
  let a headless capture exercise the inspector/profiler panels.

The legacy soft-body commands are retained as thin aliases over `dart-demos`:

    $ pixi run ex soft_bodies -- --collision-detector dart --threads 16
    $ pixi run ex soft_cubes -- --collision-detector fcl --threads 1
    $ pixi run ex soft_open_chain -- --collision-detector dart --threads 4
