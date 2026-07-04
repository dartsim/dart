# Evidence: DART 6.20 GUI Building Blocks for the Demos App

Collected 2026-07-04 from release-6.20 sources by recon agents; key file:line
references verified in-tree. This is the capability map the demos app design
builds on.

## Interaction (drag-force, gizmo, picking)

- **Mouse picking**: `dart::gui::osg::DefaultEventHandler`
  (`dart/gui/osg/DefaultEventHandler.hpp`) — `PickInfo{frame, shape, position,
  normal}`, `getButtonEvent(button)`, `getButtonPicks(button, event)`,
  `getMovePicks()`; BodyNode from a pick via
  `dynamic_cast<BodyNode*>(pick.frame->getParentFrame())`. Custom hooks via
  `addMouseEventHandler(MouseEventHandler*)`.
- **3D gizmo**: `dart::gui::osg::InteractiveFrame` (a `SimpleFrame`) with 9
  `InteractiveTool`s (LINEAR arrows / ANGULAR rings / PLANAR planes × xyz),
  `InteractiveFrame.cpp:270-572` builds visuals; hooked up via
  `viewer.enableDragAndDrop(interactiveFrame)` → `InteractiveFrameDnD` with
  per-axis constraints and hover highlighting. Read pose with
  `getWorldTransform()`.
- **DragAndDrop family** (`dart/gui/osg/DragAndDrop.hpp`): `SimpleFrameDnD`,
  `SimpleFrameShapeDnD`, `InteractiveFrameDnD`, `BodyNodeDnD`. Note:
  `BodyNodeDnD` repositions via IK (JacobianTranspose, solveAndApply) — it does
  **not** apply forces.
- **Drag-to-apply-force blueprint = Tinkertoy pattern**
  (`examples/tinkertoy/TinkertoyWorldNode.hpp:123-154`): MouseEventHandler
  forwards pick → remember `mPickedNode` + local point → user drags an
  `InteractiveFrame` target → `customPreStep` applies spring force
  `F = coeff * (target - pickedWorldPoint)` clamped, via
  `BodyNode::addExtForce(F, point)`; `LineSegmentShape` renders the force line.

## Simulation control & stats

- `RealTimeWorldNode` (`dart/gui/osg/RealTimeWorldNode.hpp`): target
  frequency/RTF setters, `getLastRealTimeFactor()`,
  `getSmoothedRealTimeFactor()` (EMA α=0.02), lowest/highest RTF. `refresh()`
  steps the world in a budget loop (max 2048 steps/refresh);
  `customPreStep/customPostStep/customPreRefresh/customPostRefresh` are the
  safe world-mutation hooks.
- `Viewer::simulate(bool)` toggles stepping; Spacebar handled by
  `DefaultEventHandler`.

## Profiling

- `dart/common/Profile.hpp` (backport of DART 7 front-end): master CMake switch
  `DART_BUILD_PROFILE` (default OFF; **ON in pixi config**), text backend
  `DART_PROFILE_BUILTIN` (default ON), Tracy backend `DART_PROFILE_TRACY`
  (OFF by default; pixi uses system tracy).
- `World::step` is instrumented: `DART_PROFILE_FRAME` at `World.cpp:897` plus
  named scopes ("Integrate velocity", "Solve constraints", …).
- Text profiler: `dart::common::profile::getProfileSummaryText()`,
  `resetProfile()` — thread-local collection; ideal for an in-app live
  profiling panel without Tracy.
- `Stopwatch` (`dart/common/Stopwatch.hpp`) for app-side timing.

## Threading / crash-safety ground rules

- All ImGui-heavy examples force
  `viewer->setThreadingModel(SingleThreaded)`; library default is OSG
  AutomaticSelection. Event handlers, ImGui widget callbacks, and
  `WorldNode::refresh()` all run on the frame-loop thread → world mutations
  from UI are safe **only** on that thread. Design rule for demos app: UI
  writes go through per-frame applied state; scene rebuilds happen between
  frames, never mid-render with multithreaded draw.

## Rendering / visual-debug utilities

- **Shadows**: `WorldNode::setShadowTechnique(nullptr|technique)` toggles;
  `WorldNode::createDefaultShadowTechnique(viewer)` (4096² ShadowMap);
  SoftShadowMap variant in `examples/sleeping/main.cpp:1020-1031`.
- **Grid**: `dart::gui::osg::GridVisual` (ViewerAttachment) — plane XY/YZ/ZX,
  cell size, colors, widths, `display(bool)`; added via
  `viewer.addAttachment(...)`.
- **Support polygon**: `dart::gui::osg::SupportPolygonVisual` with COM
  valid/invalid coloring.
- **Contact forces**: **no built-in visualizer**; canonical manual pattern in
  `examples/contact_inverse_dynamics/main.cpp:291-305,569-591` (ArrowShape per
  contact, hide when |F| small, recolor by state). Demos app should own a
  reusable contact-force visual.
- **Point cloud / voxel**: `PointCloudShape` (+4 point render styles) and
  `VoxelGridShape` (octomap) with OSG render nodes.

## Headless capture (verification harness)

- `Viewer::captureScreen(path)` sets a flag; `SaveScreen` final-draw callback
  writes the PNG on the **next** `frame()` (`Viewer.cpp:68-127,237-250`);
  `Viewer::record(dir, prefix, ...)` for sequences.
- Full offscreen recipe (working in `examples/sleeping/main.cpp:1131-1249` and
  `examples/ssik_ik_gui/main.cpp:1475+`): `GraphicsContext::Traits` with
  `pbuffer=true` + `readDISPLAY()`, `createGraphicsContext`, attach to camera
  with viewport/projection, `setThreadingModel(SingleThreaded)` (required for
  clean PNG teardown), drop manipulator, pin `setViewMatrixAsLookAt`, step
  world deterministically, then `frame(); frame(); captureScreen(path);
  frame();`. Local-only (needs DISPLAY/GPU) — debug tool, not CTest.

## ImGui

- Conda-forge system imgui 1.92.8 = master branch: **no docking/DockSpace**.
  Bundled fallback `dart/gui/imgui/` (external-imgui component) when
  `DART_USE_SYSTEM_IMGUI=OFF` (default OFF; pixi uses system).
- Workspace layout must therefore be programmatic (SetNextWindowPos/Size
  against viewport work area), not dockspace-based.
