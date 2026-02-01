# Resume: Visualization Migration (OSG → Raylib)

## Last Session Summary

Completed Phase 2 (interaction parity). The scene API now supports orbit camera
control, ray-cast picking, and visual selection highlighting. Users can orbit
(right-drag), pan (middle-drag), zoom (scroll), and click to select objects
(yellow wireframe overlay). The raylib example includes 4 shapes (ground, box,
sphere, cylinder). Build, lint, and 234/234 unit tests pass.

Commits on `feature/viz-migration`:

1. `8d40597` Phase 0+1: renderer-agnostic scene API with Raylib backend
2. `978cb0b` Wave 1: OrbitCameraController + HitResult/selected_node_id
3. `c2db653` Wave 2+3: orbit camera integration, ray-cast picking, selection highlight

## Current Branch

`feature/viz-migration` — clean (all changes committed)

## Immediate Next Step

**Phase 3**: Manipulation tools parity (drag-and-drop, IK).

## Context That Would Be Lost

- The new scene API lives in `dart/gui/` (not `dart/viz/`). Namespace: `dart::gui`.
- `OrbitCameraController` uses right-drag for orbit, middle-drag for pan, scroll for zoom.
  It operates on `Camera&` (Eigen-based), not Raylib types.
- `pickNode()` is a pure virtual on `ViewerBackend`. RaylibBackend implements it via
  `GetScreenToWorldRay` + `GetRayCollisionBox` against per-node AABBs computed from
  shape data + transform rotation. Planes and lines are skipped (zero-size AABB).
- Selection highlight draws shape-specific wireframe (not AABB wireframe) in YELLOW
  inside the 3D rendering pass, before EndMode3D().
- `SceneViewer::frame()` wires events: camera_controller first, then key handling
  (Space=pause, Enter=step, Escape=deselect), then left-click→pickNode.
- `scene.selected_node_id` is `std::optional<uint64_t>`, propagated to backend for
  both highlight rendering and HUD text.
- Raylib `Material` type name collides with `dart::gui::Material`. Backend uses `::Material`.
- `raymath.h` cannot be included (triggers -Werror=missing-field-initializers).
- CMake: examples guards `add_subdirectory(raylib)` with `if(NOT DART_BUILD_GUI_RAYLIB)`.
- 13 "Not Run" test failures are pre-existing simulation-experimental tests.

## How to Resume

```bash
git checkout feature/viz-migration
git status && git log -5 --oneline
```

Then proceed to Phase 3 (manipulation tools) or Phase 4 (UI overlay).

Key files:

- `dart/gui/scene_viewer.hpp/.cpp` — main viewer with camera + selection
- `dart/gui/orbit_camera_controller.hpp/.cpp` — orbit/pan/zoom
- `dart/gui/viewer_backend.hpp` — abstract backend (has pickNode)
- `dart/gui/raylib/raylib_backend.hpp/.cpp` — Raylib implementation
- `dart/gui/scene.hpp` — data types (HitResult, Scene::selected_node_id)
- `examples/raylib/main.cpp` — demo with 4 shapes
