# Legacy GUI Surface Audit

This audit supports Phase A of the north-star migration plan. It is not a
removal patch. It identifies the current OSG/Raylib-shaped surfaces that must be
replaced, made private, or deleted before a Filament-backed `dart::gui` can be
called complete.

## Audit Commands

```bash
find dart/gui -maxdepth 2 -type f \( -name '*.hpp' -o -name '*.h' \) | sort
rg -n "#include <osg|::osg|osg[A-Za-z]+::|OpenThreads|ImGui" dart/gui -g '*.hpp'
rg -n "DART_BUILD_GUI_RAYLIB|raylib|Raylib|examples/raylib|DART_BUILD_RAYLIB" \
  . -g '!build/**' -g '!docs/readthedocs/_generated/**' \
  -g '!docs/readthedocs/_build/**' -g '!external/**'
```

## Conclusions

- The current public `dart/gui` API is the legacy OSG implementation surface,
  not a backend-neutral abstraction to preserve.
- Promotion should not keep a multi-backend layer whose only purpose is to
  route between OSG, Raylib, and Filament. Keep only DART-owned concepts that
  remain useful with one maintained Filament implementation.
- Renderer-specific downstream source compatibility is out of scope for the
  north-star migration. Code that subclasses OSG viewer/node classes or consumes
  raw OSG event types should migrate to DART-owned concepts.
- The experimental headers in `dart/gui/experimental/` are the current clean
  API fence. During promotion, their concepts should either become the stable
  `dart::gui` API or be collapsed into private implementation details. They
  should not remain as a permanent generic-renderer adapter after Filament is
  the only maintained built-in renderer.
- Raylib is isolated to an experimental example/build option and should be
  removed in the same major-version window as the OSG GUI removal.

## Surface Classification

### Viewer

Surface: `dart/gui/viewer.hpp`

Current public shape: `Viewer` inherits `osgViewer::Viewer`;
`ViewerAttachment` inherits `osg::Group`; public methods expose root groups,
light groups, OSG light sources, OSG vectors, OSG threading, screen capture,
recording, world nodes, and drag-and-drop hooks.

North-star action: replace inheritance with composition. Keep DART concepts for
viewer options, attached worlds, camera, lighting presets, run control,
screenshots, recording, debug defaults, and tools. Remove public scene-graph
mutation and raw OSG light/threading access.

### Viewer Configuration

Surface: `dart/gui/viewer_config.hpp`

Current public shape: headless/windowed configuration is useful, but
`clearColor` is `osg::Vec4`.

North-star action: keep the viewer-options concept, replace renderer
vector/color types with Eigen or DART value descriptors, and extend the options
with Filament-ready camera, clipping, lighting, GUI scale, screenshot, and
frame-limit settings.

### World Runtime

Surfaces: `dart/gui/world_node.hpp`, `dart/gui/real_time_world_node.hpp`

Current public shape: `WorldNode` inherits `osg::Group`, owns OSG shadow groups,
and combines world refresh, optional stepping, and subclass hooks.
`RealTimeWorldNode` adds OSG timer-backed pacing.

North-star action: replace with a DART viewer runtime/world attachment layer.
Keep stepping, pause, single-step, real-time pacing, and pre/post callbacks as
DART concepts. Keep renderer resource ownership private.

### Input And Camera Events

Surfaces: `dart/gui/default_event_handler.hpp`,
`dart/gui/mouse_event_handler.hpp`, `dart/gui/trackball_manipulator.hpp`

Current public shape: public input classes inherit OSG event
handlers/manipulators and accept `osgGA::GUIEventAdapter` objects.

North-star action: replace with backend-hidden input events plus DART-owned
camera and tool controllers. Keep pick results, cursor rays, modifier state,
and camera manipulation semantics without exposing windowing or renderer event
types.

### Manipulation Tools

Surfaces: `dart/gui/drag_and_drop.hpp`, `dart/gui/interactive_frame.hpp`

Current public shape: drag-and-drop and interactive frame workflows are useful,
but key/modifier plumbing depends on OSG event enums and viewer integration.

North-star action: keep the DART manipulation concepts: selection,
frame/simple-frame translation, axis/plane constraints, body-node IK movement,
and interactive handles. Rebuild them on the new input/tool layer and remove
OSG modifier/event types.

### Shape Rendering

Surfaces: `dart/gui/shape_frame_node.hpp`, `dart/gui/render/*.hpp`

Current public shape: shape visuals are public OSG node classes. Most headers
inherit `osg::Group`, `osg::MatrixTransform`, `osg::Geode`, `osg::Geometry`, or
expose OSG materials/nodes directly.

North-star action: replace public render nodes with DART renderable descriptors
and private Filament resource code. Keep shape/material coverage, diagnostic
fallback behavior, and dynamic-geometry update policies as tested DART concepts.

### Debug And Analysis Visuals

Surfaces: `dart/gui/grid_visual.hpp`, `dart/gui/polyhedron_visual.hpp`,
`dart/gui/support_polygon_visual.hpp`

Current public shape: debug/analysis visuals inherit or own OSG geometry and are
attached as viewer scene nodes.

Current progress: the `polyhedron_visual` in-tree runner now routes to a
Filament fixture that represents the convex hull and wireframe as
renderer-hidden DART shape descriptors, while the legacy standalone OSG source
remains for comparison.

North-star action: replace with debug draw providers/descriptors for grids,
frames, support polygons, centroids, COM markers, collision bounds, contacts,
arrows, labels, and user-provided overlays.

### Panels And Widgets

Surfaces: `dart/gui/im_gui_handler.hpp`, `dart/gui/im_gui_viewer.hpp`,
`dart/gui/im_gui_widget.hpp`, `dart/gui/include_im_gui.hpp`

Current public shape: the public API exposes Dear ImGui widgets, handler,
viewer, and OSG render callbacks.

North-star action: keep Dear ImGui private for built-in panels. Add DART-owned
panel/tool descriptors only if stable user extension points are required. Do
not expose raw ImGui callbacks in promoted headers.

### OSG Utilities

Surfaces: `dart/gui/utils.hpp`, `dart/gui/detail/*.hpp`

Current public shape: utilities convert Eigen values to and from OSG matrices,
vectors, cameras, and HUD/RTT nodes.

North-star action: remove public OSG conversion helpers. Keep any math/image
helpers only as private implementation details or as renderer-independent DART
value helpers.

### Python Bindings

Surface: `python/dartpy/gui/*.cpp`

Current public shape: `dartpy.gui` exposes OSG-shaped viewer, world-node, event
adapter, shadow technique, ImGui, attachment, drag-and-drop, and visual classes.

North-star action: rebuild stable `dartpy.gui` around the same DART-owned
concepts as C++: viewer options, camera, debug toggles, picking, tools,
screenshots, run control, and descriptors. Keep `dartpy.gui.experimental` as
the current backend-hidden bridge until promotion.

### Raylib Experiment

Surfaces: `examples/raylib`, `DART_BUILD_GUI_RAYLIB`, `DART_BUILD_RAYLIB`,
`DART_USE_SYSTEM_RAYLIB`, `cmake/dart_find_raylib.cmake`,
`scripts/run_cpp_example.py`

Current public shape: standalone experimental smoke path and convenience runner
support for enabling the Raylib target.

North-star action: remove after Filament promotion and accepted migration
timing. Do not replace with another maintained renderer option.

### Maintained Examples

Surfaces: OSG-heavy examples such as `examples/imgui`,
`examples/drag_and_drop`, and maintained robotics/interaction examples using
`dart::gui::Viewer`/`WorldNode`.

Current public shape: user-facing examples still teach OSG-shaped APIs.

Current progress: the in-tree `hello_world`, `boxes`, `drag_and_drop`,
`simple_frames`, `soft_bodies`, `point_cloud`, `polyhedron_visual`,
`heightmap`, and `g1_puppet` runners now route to focused Filament scenes by
default while their standalone sources remain as legacy OSG comparison
material until the promoted API can replace the missing panel/tool-specific
workflows.

North-star action: port representative examples to the promoted
Filament-backed `dart::gui` API before deleting the legacy examples. Keep at
least one simple, one mesh/material-heavy, one robotics, one interaction-heavy,
and one headless screenshot example.

## Concepts To Keep

- Viewer options and lifecycle state.
- Attached worlds and simulation run control.
- Orbit/camera controls, clipping, and fit-to-scene helpers.
- Renderable descriptors for DART shapes, materials, transforms, visibility,
  shadow flags, and resource revision tracking.
- Debug draw descriptors and providers.
- Picking results with selected object, hit point, normal, distance, and
  modifier state.
- Tool concepts for selection, simple-frame movement, free-joint movement,
  plane/axis constraints, and IK-backed body manipulation.
- Screenshot, bounded-frame, and recording workflows.
- Optional DART-owned panel descriptors if maintainers decide user extension
  points are needed.

## Surfaces To Remove Or Make Unsupported

- Public inheritance from `osgViewer::Viewer`, `osg::Group`,
  `osg::MatrixTransform`, `osg::Geode`, `osg::Geometry`, and OSG event handler
  classes.
- Public `osg::ref_ptr`, `osg::Vec*`, `osg::Matrix`, `osg::Camera`,
  `osg::Node`, `osg::Light*`, `osgShadow::*`, `osgGA::*`, and
  `osgViewer::*` types.
- Public Dear ImGui callbacks and raw ImGui widget APIs unless intentionally
  wrapped by DART-owned panel descriptors.
- Raylib build options, dependency discovery, example target, runner aliases,
  and docs after the Filament migration window.
- Any backend-neutral adapter that remains only to choose among renderers after
  Filament is the sole maintained built-in GUI stack.

## Promotion Implications

1. New public `dart::gui` headers should be checked for the same backend-token
   leakage already enforced on `dart/gui/experimental/*.hpp`.
2. The promoted Filament implementation should own renderer resources privately:
   materials, textures, mesh buffers, entity caches, swap chains, windows, and
   optional ImGui draw resources.
3. The maintained-example include gate is stricter than the current MVP header
   guard. At full migration, files under `examples/filament_gui/` or its
   surviving promoted replacement should have zero direct
   `#include <filament/...>` or `#include "filament/..."` directives; Filament
   headers should live only in private promoted GUI implementation units.
4. The maintained-example shape gate should also keep
   `examples/filament_gui/` to a minimal `main.cpp` entry point plus
   unavoidable build/docs files, with renderer and fixture logic encapsulated
   by `dart::gui` or private GUI implementation units.
5. Example migration must happen before deletion. Removing legacy examples
   first would discard useful coverage for drag-and-drop, support polygons,
   ImGui panels, headless capture, and robotics scenes.
6. The migration should delete compatibility shims after examples, docs, and
   Python bindings no longer use them. Keeping shims indefinitely would recreate
   the multi-backend abstraction problem under a different name.
