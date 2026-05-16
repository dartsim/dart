# Filament GUI North-Star Migration Plan

This plan has been executed in the current promotion branch. See
`07-completion-audit.md` for the current state and remaining follow-up scope.

## North star

DART's maintained built-in visualization is now a Filament-backed `dart::gui`
implementation. OpenSceneGraph (OSG) and the Raylib smoke path were removed in
the same major-version migration window.
This was a full replacement target: both the main visualization API and
experimental visualization paths converged onto Filament, and redundant
renderer-neutral layers that existed only to keep multiple backends viable were
deleted or collapsed after the Filament API boundary was promoted.

The public API should be owned by DART and should describe DART concepts:
worlds, cameras, renderables, debug draws, selections, tools, panels, frame
capture, and simulation control. Public headers should not expose Filament,
GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or Raylib types.
Keep abstractions only when they express stable DART concepts or make the
Filament implementation testable; do not preserve indirection whose only job is
to support alternate renderer implementations.

The MVP `examples/filament_gui` provided evidence that the renderer could work.
The current promotion branch moved the renderer plumbing under private
`dart::gui` implementation units and keeps the example tree to a minimal
`main.cpp` entry point plus unavoidable build/docs files. Filament headers live
only in private implementation units below the promoted GUI component, while
examples exercise DART-owned viewer, scene, debug, capture, and tool APIs.

## Compatibility position

Renderer compatibility is not a north-star constraint. Downstream projects such
as Gazebo physics integrations primarily depend on DART's headless physics
core, not the current OSG renderer. Code that subclasses OSG GUI classes,
manipulates raw OSG scene graph nodes, or depends on OSG event types should
either migrate to a DART-owned concept or become explicitly unsupported.

This keeps the replacement design from inheriting OSG's public type leakage:

- no `Viewer : osgViewer::Viewer`
- no public `osg::Group`, `osg::Node`, `osg::Vec*`, or OSG event adapters
- no raw ImGui extension hooks in the stable API
- no multi-backend abstraction just to preserve legacy renderer behavior
- no source compatibility promise for renderer-specific downstream code

## Current capability inventory

The current OSG GUI provides more than drawing. It combines rendering,
simulation stepping, event handling, picking, manipulation, capture, and
extension points:

- `dart::gui::Viewer` inherits from `osgViewer::Viewer`, owns the scene root,
  lights, world nodes, event handler, attachments, recording, capture, and drag
  and drop registration.
- `WorldNode` owns DART world refresh, optional simulation stepping, shape-frame
  node lifecycle, shadow groups, and `customPreRefresh`, `customPostRefresh`,
  `customPreStep`, and `customPostStep` hooks.
- `RealTimeWorldNode` provides adaptive real-time simulation playback.
- `ShapeFrameNode` and `render::ShapeNode` cover primitive, mesh, soft-body,
  point, voxel, heightmap, line, warning, and helper visuals.
- `DefaultEventHandler` exposes OSG event adapters, pick records with hit point
  and normal, cursor deltas, mouse handlers, line constraints, and plane
  constraints.
- `DragAndDrop`, `InteractiveFrame`, and `BodyNodeDnD` provide higher-level
  manipulation workflows, including IK-backed body manipulation.
- `ImGuiViewer` and related hooks provide debug panels on top of the OSG
  viewer.
- Headless pbuffer rendering, screenshots, and frame recording are part of the
  viewer surface.

The experimental Filament work already covers a narrower backend-hidden slice:
renderable descriptors, common primitive descriptors, TriMesh mesh references,
finite plane proxies, visual shadow flags, debug line descriptors, bounds-based
picking, primitive and triangle-backed surface picking, simple selection,
selected-frame translation, plane and axis drag helpers, orbit camera math,
lifecycle state, screenshot storage, constrained Python bindings, renderable
set update planning, and headless smoke tests.

`09-legacy-surface-audit.md` maps these legacy surfaces at the file/API level
into DART concepts to keep, Filament-private implementation details, and
renderer-specific surfaces to remove or leave unsupported.

## Alternative renderer survey

Filament remains the best north-star candidate for the built-in GUI, but it is
not a drop-in OSG replacement.

- **OSG** has a mature scene graph, event stack, custom node extension model,
  and DART's existing feature coverage. Its main blocker is that those benefits
  are exposed directly in DART's public API, forcing DART to keep renderer
  implementation details as user-facing contracts.
- **VulkanSceneGraph (VSG)** is the closest conceptual successor to OSG for
  users who want a modern scene graph. It would still require DART-owned
  simulation, GUI, picking, tool, packaging, and Python APIs. It also has less
  evidence in this branch than Filament and does not by itself solve the public
  API leakage problem.
- **Raylib** is useful for small examples, but its renderer quality ceiling,
  material model, asset pipeline, and robotics visualization headroom are below
  the target for replacing OSG examples.
- **External remote, telemetry, and streaming viewers** may be useful adjacent
  tools for remote inspection, time-series logging, or browser workflows. They
  are not a native built-in replacement for DART's maintained C++ GUI library,
  and they should not become maintained integrations without concrete demand.
- **Dear ImGui and GLFW** are windowing and debug UI dependencies, not renderer
  replacements. They should stay private implementation details unless DART
  intentionally adds stable DART-owned panel abstractions.

## Capability and gap analysis

### 1. Public viewer API and extension model

- **Existing support:** OSG users can subclass or directly manipulate
  `osgViewer::Viewer`, `osg::Group`, `ViewerAttachment`, and OSG scene graph
  nodes through public DART headers.
- **Filament gap:** Filament is an entity/resource renderer, not a public scene
  graph framework. It should not be exposed as a node hierarchy.
- **North-star plan:** Replace renderer inheritance with composition. Provide
  DART-owned extension points for viewer lifecycle callbacks, debug draw
  providers, tools, and panels. Do not preserve direct scene graph mutation.

### 2. World refresh and simulation stepping

- **Existing support:** `WorldNode` and `RealTimeWorldNode` combine world
  refresh, optional stepping, real-time playback, and subclass hooks.
- **Filament gap:** Filament does not define a simulation loop or DART world
  lifecycle.
- **North-star plan:** Add a DART viewer runtime layer that owns stepping,
  pause, single-step, fixed timestep, real-time pacing, and callbacks without
  depending on renderer classes.

### 3. Shape coverage

- **Existing support:** OSG shape nodes cover box, sphere, ellipsoid, cylinder,
  capsule, cone, pyramid, plane, convex mesh, multi-sphere convex hull, mesh,
  soft mesh, line segment, point cloud, voxel grid, heightmap, and warning
  fallback visuals.
- **Filament gap:** The current experimental descriptor layer only covers box,
  sphere, ellipsoid, cylinder, capsule, cone, pyramid, multi-sphere,
  line-segment, convex mesh, point cloud, heightmap, soft mesh, voxel grid when
  OctoMap is available, mesh, finite plane proxy, and unsupported diagnostic
  fallback. Filament can render the missing geometry once DART supplies dynamic
  buffers, materials, and update policies.
- **North-star plan:** Harden descriptor behavior before promotion: extend
  fallback diagnostics to unsupported assets, upgrade soft meshes from static
  creation-time triangle buffers to dynamic vertex buffers, voxel grids from
  generated occupied-cell box meshes to instanced cubes or another GPU-friendly
  representation, point clouds from the MVP box-point visual to point or
  billboard geometry, and heightmaps from the MVP generated terrain mesh to
  streaming terrain buffers, if those are needed for parity with maintained
  examples.

### 4. Mesh and material fidelity

- **Existing support:** OSG mesh rendering uses DART's mesh import path and
  visual aspects, but the current public API is tied to OSG render nodes.
- **Filament gap:** Filament has a stronger PBR renderer, and the experimental
  descriptors now carry imported mesh materials, texture paths, texture
  coordinate availability, and submesh ranges, but DART still needs stable asset
  descriptors for alpha behavior, UV set policy, resource caching, and update
  invalidation.
- **North-star plan:** Continue promoting mesh/material behavior out of the MVP
  example. Keep material behavior testable without a graphics context, and keep
  backend material instances private.

### 5. Lighting, environment, and shadows

- **Existing support:** OSG exposes light groups, light sources, headlights,
  shadowed groups, and shadow techniques directly.
- **Filament gap:** Filament provides modern lighting and shadows, but it does
  not map to OSG's public light group model.
- **North-star plan:** Add DART-owned lighting presets and optional explicit
  light descriptors: sun or directional lights, environment/skybox, ambient or
  indirect light, shadow quality, contact shadows, exposure, and color grading.
  Keep renderer light handles private.

### 6. Camera control and clipping

- **Existing support:** OSG supplies a trackball manipulator, camera modes,
  depth capture mode, near/far behavior, and viewport integration.
- **Filament gap:** Filament provides cameras but not DART-specific camera
  controls or interaction semantics.
- **North-star plan:** Own camera controllers in DART: orbit, pan, zoom,
  first-person or free camera if needed, follow-target, fit-to-world,
  practical near/far defaults, explicit clipping configuration, and stable
  headless camera setup.

### 7. Picking and hit information

- **Existing support:** OSG intersection visitors return picked shape frames,
  hit positions, normals, and ordered intersections.
- **Filament gap:** Filament does not provide DART object picking semantics.
  The current experimental path uses local bounds for shapes without richer
  data, primitive sphere/ellipsoid/cylinder/capsule/cone/pyramid/multi-sphere,
  point-cloud/voxel-grid box-proxy, finite plane-proxy, and triangle-backed
  convex-mesh/heightmap/soft-mesh/MeshShape intersections for surface hit
  points and normals. This is sufficient for an MVP but not precise enough for
  all promoted tools.
- **North-star plan:** Add renderer-independent picking services. Start with
  bounding volumes, then add triangle or primitive intersection for precise
  hit point/normal data. If GPU ID picking is later needed, keep it behind the
  same DART pick-result API.

### 8. Input and manipulation tools

- **Existing support:** OSG event handlers, `DragAndDrop`, `InteractiveFrame`,
  `BodyNodeDnD`, line constraints, plane constraints, and IK-backed body
  manipulation support interactive workflows.
- **Filament gap:** Filament does not define input events or manipulation
  tools.
- **North-star plan:** Add a backend-hidden input model and a public DART tool
  layer. Tool primitives should include selection, transform manipulators,
  camera-plane drag, axis or plane constraints, simple-frame movement, free
  joint movement, and an explicit replacement for body-node drag and drop.

### 9. Debug overlays and diagnostics

- **Existing support:** OSG examples and DART GUI code can render frames,
  contacts, force vectors, support polygons, interactive frames, line segment
  shapes, labels, and custom nodes.
- **Filament gap:** Filament can draw these, but DART must own the debug draw
  model. The current experimental descriptors cover grid, frames, COM markers,
  inertia boxes, collision-shape bounds, contacts, normal/force arrows,
  support-polygon outlines, support-centroid markers, and selection bounds
  only.
- **North-star plan:** Grow a DART debug overlay registry with stable descriptor
  types for lines, points, arrows, labels, text, frames, contact patches,
  support polygons, additional inertia/COM helpers, richer collision shape
  toggles, and user-provided debug draw providers.

### 10. Text and labels

- **Existing support:** OSG can render text and arbitrary node annotations.
- **Filament gap:** Filament does not provide a DART label system by itself.
- **North-star plan:** Add a text/label subsystem through a private font atlas
  or ImGui-backed overlay path. The public API should expose label descriptors,
  not font backend objects.

### 11. GUI panels and user controls

- **Existing support:** OSG `ImGuiViewer` allows debug UI integration in the
  viewer process.
- **Filament gap:** Filament is not a GUI toolkit. The current MVP panel is
  example-local Dear ImGui code.
- **North-star plan:** Keep Dear ImGui private. If users need extension, expose
  DART-owned panel/tool abstractions such as toggles, numeric controls, command
  buttons, selection inspectors, and per-world diagnostics instead of raw ImGui
  draw callbacks.

### 12. Headless rendering, screenshots, and recording

- **Existing support:** OSG supports headless pbuffer rendering, screen
  capture, and frame recording.
- **Filament gap:** Filament supports offscreen rendering and readback, but
  DART must define lifecycle, timing, image formats, origin convention, and CI
  validation.
- **North-star plan:** Promote bounded frame loops, screenshots, and frame
  recording as renderer-hidden viewer APIs. Keep deterministic screenshot smoke
  tests, but add higher-resolution visual review gates before promotion.

### 13. Resource ownership and lifetime

- **Existing support:** OSG scene graph nodes own many renderer resources
  implicitly, and DART's current GUI maps frame lifetimes into node lifetimes.
- **Filament gap:** Filament resources are explicit and must be destroyed in
  the correct engine context. The current experimental API can plan renderable
  add/remove/visibility updates from descriptor snapshots, but full renderer
  caches still live in the MVP example.
- **North-star plan:** Centralize renderer resource caches behind the GUI
  implementation: mesh buffers, material instances, textures, samplers,
  renderable entities, debug buffers, and per-world descriptor snapshots.
  Public objects should have ordinary DART lifetimes and should not require
  users to manage Filament resources.

### 14. Python bindings

- **Existing support:** Legacy `dartpy.gui` reflects OSG-shaped APIs and event
  concepts where exposed.
- **Filament gap:** Python should not expose Filament, GLFW, ImGui, or OSG
  types.
- **North-star plan:** Build `dartpy.gui` around the same DART-owned concepts:
  viewer options, camera, debug toggles, selection, screenshot capture,
  descriptors for headless tests, and simple run-loop helpers.

### 15. Packaging, platform support, and compilers

- **Existing support:** OSG is already integrated into DART's dependency
  matrix.
- **Filament gap:** Filament package availability is still the main promotion
  risk. The pinned fetch path proves Linux smoke coverage but is not the
  preferred release path.
- **North-star plan:** Prefer a package-managed `filament-static` dependency
  once available. Until then, keep explicit fetch opt-in for experiments. The
  promotion matrix should include Linux, macOS, Windows where supported,
  source builds, wheels, headless smokes, and both GCC and Clang coverage for
  the Filament target.

### 16. Recording and remote inspection alternatives

- **Existing support in alternatives:** External telemetry tools can be
  stronger than Filament for time-series logging, remote playback, and
  inspection outside the native process.
- **Filament gap:** Filament is an in-process renderer, not a recording or
  telemetry product.
- **North-star plan:** Keep the built-in GUI focused on native interactive
  visualization. Treat remote viewers and recording systems as future
  exporters that consume DART-owned descriptors, not as requirements for the
  core GUI API.

## API design principles

The promoted API should be small, high-level, and testable:

- A user attaches one or more DART worlds to a viewer without constructing
  renderer objects.
- Viewer options describe window size, headless mode, GUI scale, camera,
  lighting preset, frame limit, screenshot path, and debug defaults.
- Camera, debug overlays, selections, tools, and panels are DART objects or
  value descriptors.
- Renderer resources, shader/material objects, swap chains, native windows,
  and ImGui contexts remain private.
- The scene extraction layer can be unit-tested without a graphics context.
- Examples should prefer stable viewer APIs over reaching into the renderer.
- Advanced customization should be additive through tools, debug draw
  providers, and panel descriptors, not through inheritance from backend
  classes.

Directional example:

```cpp
dart::gui::ViewerOptions options;
options.windowTitle = "DART Viewer";
options.guiScale = 2.0;
options.debug.drawGrid = true;
options.debug.drawContacts = true;

dart::gui::Viewer viewer(options);
viewer.attachWorld(world);
viewer.camera().lookAt({4.0, -5.0, 3.0}, {0.0, 0.0, 0.6});
viewer.tools().enableSelection();
viewer.tools().enableFrameTranslation();
viewer.run();
```

This sketch is only a design target. Exact names should be chosen when the
experimental surface is ready to promote.

## Multi-phase migration plan

### Phase A - API fence and OSG audit

**Deliverables**

- This north-star plan is linked from the Filament dev task index.
- Public API principles are accepted: DART concepts in public headers,
  renderer types private, no source compatibility promise for OSG renderer
  code.
- Public header leakage checks cover the experimental GUI headers and should be
  extended to the promoted `dart::gui` headers before first-class promotion.
- Existing OSG GUI surfaces are audited into supported DART concepts,
  unsupported renderer-specific behaviors, and optional future extensions. The
  current file-level audit lives in `09-legacy-surface-audit.md`.

**Exit criteria**

- New experimental APIs avoid backend types.
- Maintainers agree that OSG-specific downstream renderer compatibility is not
  a blocker for the replacement.

### Phase B - Scene descriptor completion

**Deliverables**

- Complete shape descriptor coverage for DART-maintained examples.
- Continue promoting material, texture, alpha, UV, and mesh-subset metadata out
  of the MVP example and into testable descriptor code.
- Extend fallback diagnostics to unsupported assets.

**Exit criteria**

- Descriptor tests cover common primitives, mesh materials, soft/dynamic
  geometry policies, point or line data, and unsupported fallback behavior
  without requiring a graphics context.

### Phase C - Filament renderer core

**Deliverables**

- Move reusable renderer resource management out of `examples/filament_gui`.
- Add private caches for materials, textures, meshes, debug buffers, entities,
  and per-world render state.
- Keep the public boundary free of Filament types.

**Exit criteria**

- The Filament example is a thin consumer of the GUI library rather than the
  owner of renderer architecture.
- Resource destruction is centralized and covered by targeted tests or
  sanitizer-friendly smoke runs where practical.

### Phase D - Viewer runtime API

**Deliverables**

- Add viewer options, camera controls, simulation run control, pause/step,
  real-time pacing, screenshot capture, bounded runs, and headless rendering
  through DART-owned APIs.
- Add practical near/far clipping defaults and user override options.
- Keep GUI scale configurable for high-DPI displays.

**Exit criteria**

- Simple maintained examples can run with the new API in windowed and headless
  modes.
- Screenshot and bounded-run behavior is validated in CI.

### Phase E - Interaction and tools

**Deliverables**

- Add backend-hidden input events and public DART tool abstractions.
- Provide selection, hit results, transform manipulation, plane/axis
  constraints, simple-frame movement, free-joint movement, and a replacement
  path for legacy drag-and-drop workflows.
- Add panel abstractions only if users need stable extension points.

**Exit criteria**

- Interaction-heavy DART examples no longer require OSG.
- Picking returns enough information for user-facing tools: selected object,
  hit point, normal when available, distance, and modifier state.

### Phase F - Examples and documentation migration

**Deliverables**

- Port one simple example, one mesh/material-heavy example, one robotics
  example, one interaction-heavy example, and one headless screenshot example
  to the clean GUI API.
- Update user-facing docs so examples show the intended API rather than the MVP
  internals.
- Keep `examples/filament_gui` as a development fixture or retire it once real
  examples cover the same evidence.

**Exit criteria**

- Maintained examples demonstrate camera control, selection, debug overlays,
  screenshots, dynamic lighting/shadows, mesh assets, and interaction through
  the promoted API.

### Phase G - Python and packaging readiness

**Deliverables**

- Bind the stable DART-owned GUI concepts into `dartpy.gui`.
- Validate source builds and wheels with Filament enabled where supported.
- Prefer packaged Filament through Pixi/conda-forge; keep explicit fetch only
  for experiments until package coverage is ready.
- Maintain GCC and Clang smoke coverage for the Filament GUI target.

**Exit criteria**

- Python users can create or inspect the supported GUI workflows without OSG
  types.
- CI covers the supported compiler and package matrix well enough for
  promotion.

### Phase H - First-class promotion

**Deliverables**

- Promote the experimental API to primary `dart::gui` in the planned major
  version.
- Make `DART_BUILD_GUI=ON` select the Filament implementation by default.
- Update release notes with dependency requirements, migration guidance, and
  removal timeline.

**Exit criteria**

- Maintainers accept visual quality, interaction coverage, packaging, and API
  stability gates.
- Default GUI builds no longer require users to install OSG for maintained
  examples.

### Phase I - OSG and Raylib removal

**Deliverables**

- Remove OSG implementation files, public OSG-shaped APIs, OSG-specific Python
  bindings, and OSG-only examples.
- Remove the Raylib smoke experiment and related build options.
- Delete or collapse backend-neutral scaffolding that exists only to support
  multiple renderer implementations, while keeping DART-owned concepts such as
  scene descriptors, tools, panels, debug draws, and camera/run options.
- Remove migration-only shims after examples and docs no longer use them.

**Exit criteria**

- The repository has one maintained built-in GUI stack.
- DART core physics and headless workflows remain independent of GUI
  dependencies.

## Promotion gates

Promotion should not depend on the MVP looking good in one scene. Required
evidence should include:

- clean public API review with no backend type leakage
- shape/material descriptor coverage for maintained examples
- visual review of primitive, mesh, robot, PBR, transparency, and shadow scenes
- interaction review for selection, camera, manipulation, and debug toggles
- headless screenshot tests with nonblank and contrast assertions
- source-build and wheel evidence where GUI support is advertised
- GCC and Clang smoke coverage for the Filament GUI target
- documentation and examples that use only the new GUI API
- explicit release notes for unsupported OSG-specific extension points
