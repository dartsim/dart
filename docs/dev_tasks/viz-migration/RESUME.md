# Resume: Visualization Migration (OSG → Raylib)

## Last Session Summary

Completed Phase 1 MVP. The abstract scene API (`dart::gui`) and Raylib backend
(`dart::gui::RaylibBackend`) compile and run end-to-end. The `examples/raylib`
demo creates a physics scene (ground plane + falling box + sphere), renders it
through `SceneViewer`, and exits cleanly. Build with `DART_BUILD_GUI_RAYLIB=ON`
verified. Lint and unit tests pass.

Key fixes applied this session:

- Removed duplicate `paused_` state from `RaylibBackend` (now owned by `SceneViewer`)
- Added `paused` and `sim_time` fields to `Scene` struct for backend HUD display
- Fixed `Material` name collision (Raylib vs `dart::gui::Material`) using `::Material`
- Replaced `MatrixIdentity()` (requires `raymath.h` which triggers `-Werror`) with
  `eigenToRaylib(Eigen::Isometry3d::Identity())`
- Fixed CMake duplicate `add_subdirectory(raylib)` conflict between `examples/CMakeLists.txt`
  and root `CMakeLists.txt`
- Fixed `find_package(raylib)` conflict when Raylib is already fetched via FetchContent
- Fixed DART 7 API usage in example (`getVisualAspect` on ShapeNode, not BodyNode)

## Current Branch

`feature/viz-migration` — uncommitted changes ready for commit

## Immediate Next Step

**Commit current changes**, then proceed to Phase 2 (interaction/picking via ray cast).

## Context That Would Be Lost

- The new scene API lives in `dart/gui/` (not `dart/viz/` as originally planned in docs).
  Namespace is `dart::gui`. Classes: `SceneViewer`, `SceneExtractor`, `ViewerBackend`.
- `RaylibBackend` does NOT own pause state. `SceneViewer` handles Space/Enter keys
  in its `frame()` method and populates `scene.paused` for the backend's HUD.
- Raylib `Material` type name collides with `dart::gui::Material`. Backend code uses
  `::Material` for the Raylib type and `rlMaterial` as variable name.
- `raymath.h` cannot be included directly because its C-style `{0}` initializers
  trigger `-Werror=missing-field-initializers` under DART's strict compiler flags.
- CMake: `examples/CMakeLists.txt` guards `add_subdirectory(raylib)` with
  `if(NOT DART_BUILD_GUI_RAYLIB)` to avoid duplicate binary directory with root CMakeLists.
- CMake: `dart/gui/raylib/CMakeLists.txt` skips `find_package(raylib)` when the target
  already exists (FetchContent in-source builds).

## How to Resume

```bash
git checkout feature/viz-migration
# Verify state:
git status && git log -3 --oneline
```

Then:

1. Commit current changes
2. Proceed to Phase 2: interaction parity (picking/selection via ray cast)
3. Key files: `dart/gui/scene_viewer.hpp`, `dart/gui/raylib/raylib_backend.cpp`

## Files Created/Modified

**New files (dart/gui/):**

- `scene.hpp` — Core scene data types (ShapeData variants, SceneNode, Camera, etc.)
- `input_event.hpp` — Backend-agnostic input events
- `viewer_backend.hpp` — Abstract backend interface + ViewerConfig
- `scene_extractor.hpp/.cpp` — World → Scene traversal
- `scene_viewer.hpp/.cpp` — Main viewer class with simulation stepping

**New files (dart/gui/raylib/):**

- `raylib_backend.hpp/.cpp` — Full Raylib implementation of ViewerBackend
- `CMakeLists.txt` — Raylib backend build rules

**New files (docs/dev_tasks/viz-migration/):**

- `README.md` — Epic status and decisions
- `RESUME.md` — This file
- `01-architecture.md` — Architecture design document
- `02-milestones.md` — Phase definitions and exit criteria

**Modified files:**

- `dart/gui/CMakeLists.txt` — Added `add_subdirectory(raylib)` gate
- `dart/gui/fwd.hpp` — Forward declarations for new types
- `examples/CMakeLists.txt` — Guarded raylib subdirectory to avoid duplicates
- `examples/raylib/CMakeLists.txt` — Links dart-gui + dart-gui-raylib
- `examples/raylib/main.cpp` — Physics demo using SceneViewer
