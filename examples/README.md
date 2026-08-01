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

### Inspecting memory and layout

The **Memory Diagnostics** panel is compiled out of ordinary `dart-demos`
builds. Build and launch the opt-in diagnostic variant with:

```bash
DART_BUILD_DEMOS_MEMORY_DIAGNOSTICS_OVERRIDE=ON pixi run demos
```

This default-OFF build boundary omits the panel, diagnostic session/model object
code, process probes, platform-specific links, and every feature call from the
`dart-demos` executable. Test builds still compile the isolated model/collector
library to retain coverage; it is not linked into the app. Once compiled into
the app, collection is still off by default. Enable it in the panel to sample
at a bounded cadence, or set `DART_DEMOS_MEMORY_DIAGNOSTICS=1` when launching
the app to start enabled (useful for UI capture and automated inspection).

The panel separates evidence by scope and does not add the rows into a total:

- **Process memory** is the current and process-lifetime peak resident set from
  the host OS, plus the peak observed by the current sampled session. It
  includes the renderer, libraries, assets, retained allocator pages, and the
  diagnostic buffers themselves; it is not DART-owned heap attribution.
- **World allocator categories** report user-requested live/peak bytes and live
  allocation counts where the allocator has counters enabled. Unavailable
  instrumentation is shown as unavailable, never as zero.
- **Frame scratch** reports current use, arena capacity, peak use, resets, and
  overflow allocations for the current World.
- **ECS aggregate and storage layout** report live/capacity slot counts,
  materialized packed slots, unused reserved slots, holes, sparse index extent,
  and packed-region/layout proxies. These are slot counts and virtual layout
  observations, not component byte totals or cache-miss measurements. Storage
  rows have best-effort human-readable roles; their numeric tokens remain
  internal grouping labels for the current build, not stable component
  identifiers. The demo requests the opt-in detailed scan only at its bounded
  sample cadence; ordinary World diagnostic summaries do not scan packed slots.

The default-open **Address map (actual regions)** section is the byte-layout
view. Each row is one real contiguous virtual-memory allocation owned by the
World free-list or frame allocator. Bytes run from relative offset zero
left-to-right and then top-to-bottom; separate rows are discontinuous and are
never concatenated into a synthetic global arena. The map partitions allocator
metadata, allocated payload, free payload, reserved arena capacity, and padding.
Known component payload pages and entity-index ranges refine allocated spans
with Model, Geometry, State, Control, Contact/Solver, Cache, Entity/Index, and
Metadata categories. Allocator headers use the separate allocator-infrastructure
category. Allocated bytes that cannot be attributed safely remain Unknown.

Large regions use between 32 and 4096 user-adjustable display cells, but a
mixed cell keeps its contributing byte ranges in address order as
proportional stripes and its tooltip reports exact offsets and byte counts. A
collapsed exact-range table provides the same labels and evidence without
requiring pointer hover. Hue identifies semantic category; patterns, borders,
labels, and tooltips also encode storage/logical state so the map does not
depend on color alone. ECS tombstones and spare slots remain allocated
component-page bytes; they are not rendered as allocator-free holes.

The collapsed **Capacity composition (logical)** section retains the earlier
grouped frame/ECS summaries. Those cells communicate used/live, hole, and
reserved counts, not address order. Process RSS remains a table because it
includes the whole process and overlaps the World-owned observations.

Both maps describe process virtual-address layout only. They can expose
adjacency, alignment, fragmentation, and potential locality opportunities, but
they do not measure access order, physical-page adjacency, cache residency,
cache misses, latency, or GPU memory. Those claims require a separately named
hardware-counter, sampling, or cache-simulation result. This version does not
capture the host page/cache-line geometry and scrubbed base-address remainders
needed to place boundary guides; the UI reports that evidence as unavailable.

Use **Sample + set baseline** or **Baseline = latest** to create a comparison;
compatible rows then show signed deltas. **Reset session** clears the baseline,
bounded history, and session-observed peak while retaining history storage. It
cannot reset the OS process-lifetime peak. The default 0.5-second interval is
adjustable from 0.1 to 5 seconds.

Resident-memory sources are `/proc/self/status` (`VmRSS`/`VmHWM`) on Linux,
`GetProcessMemoryInfo` working-set counters on Windows, and Mach task resident
size on macOS. A missing platform value remains explicitly unavailable because
these APIs have different accounting semantics and update timing.

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
