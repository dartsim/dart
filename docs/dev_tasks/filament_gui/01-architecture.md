# Filament GUI Replacement - Architecture

## Design posture

DART needs a maintainable built-in visualization/debugging tool, not a general
graphics engine. The new implementation should therefore have one maintained
renderer stack and a constrained DART-owned public API.

The intended stack is:

```text
dart::gui public API
  -> DART-owned scene/debug visualization layer
  -> private Filament renderer integration
  -> GLFW window/input integration
  -> private Dear ImGui debug UI integration
```

## Public API goals

- Keep the user-facing namespace as `dart::gui`.
- Do not expose Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, or native
  window handles from normal public viewer APIs.
- Cover the common DART workflows with a small surface:
  - create/run/close viewer
  - attach a `simulation::World`
  - configure camera and viewport
  - pause, step once, or run realtime-ish simulation
  - select frames/bodies/shapes
  - expose constrained debug panels/tools
  - capture screenshots or run a bounded frame loop for smoke tests
- Keep OSG compatibility out of the new API. If legacy OSG APIs must remain
  during transition, isolate them as legacy compatibility.

## Internal scene layer

The first reusable implementation layer should be independent of Filament. It
should extract a render/debug scene from DART objects:

- stable renderable IDs for selection and cache invalidation
- world transforms
- geometry descriptors for common DART shapes
- material/color/visibility/shadow flags from visual aspects
- mesh asset references and scale
- debug primitives for contacts, forces, frames, constraints, inertia boxes,
  center of mass, support polygons, grids, and labels

This is not a multi-backend promise. It is a maintainability boundary that makes
the simulation-to-visualization conversion testable and keeps renderer code
contained.

## Filament integration responsibilities

The Filament side owns:

- engine, renderer, swap chain, view, scene, camera, and resource lifetimes
- conversion from DART scene descriptors to Filament entities/resources
- geometry buffers and cache updates
- material instances, lighting, shadows, image-based lighting, and tone mapping
- picking support when implemented through render IDs or ray queries
- screenshot/offscreen capture implementation

Keep the first implementation conservative: one default lit material, one
unlit/debug material, simple directional lighting, and a Z-up camera convention.
Do not add asset pipelines or material variants until a maintained example needs
them.

## Dear ImGui integration

Dear ImGui is the right tool for DART's debug controls, but should remain an
implementation detail. Public extension points should be DART-owned concepts
such as panels and tools, not raw ImGui callbacks.

The MVP can use ImGui directly inside the example. A promoted API should hide
ImGui behind DART-owned panel/tool abstractions unless a deliberate
backend-specific escape hatch is accepted.

Do not assume Dear ImGui's OpenGL renderer backend can be used directly in the
native Filament path. Filament owns the rendering backend and may run OpenGL
work on its own backend thread. The durable integration should either render
ImGui draw data through Filament resources or use an officially supported
Filament-compatible helper if one is packaged with the selected Filament
distribution.

Overlay decision for this task: keep the current Filament-native ImGui draw-data
path as an example-local MVP implementation. It is enough for the built-in
status/debug panel, pause/step controls, and debug overlay toggles, and it keeps
ImGui out of public DART headers. Do not promote this exact overlay as a stable
extension API. Before first-class `dart::gui` promotion, either keep the overlay
strictly internal to built-in controls or wrap it in DART-owned panel/tool
abstractions with explicit support expectations for clipping, multiple textures,
and user-provided widgets.

## Dependency policy

Filament availability is the primary risk. Use this resolution strategy once
CMake support is added:

1. Prefer a conda-forge/Pixi or system package-manager Filament installation.
2. Allow the pinned Linux x86_64 CMake fetch path only when explicitly
   requested. Extend or replace it before relying on it for other platforms.
3. Never silently fetch Filament as a side effect of `DART_BUILD_GUI=ON`.
4. Require the matching Filament host tools needed to build material packages.

Proposed options:

```cmake
DART_BUILD_GUI_FILAMENT=OFF
DART_USE_SYSTEM_FILAMENT=ON
DART_FETCH_FILAMENT=OFF
DART_FILAMENT_VERSION=<pinned-version>
```

`DART_BUILD_GUI_RAYLIB` remains a legacy experiment until the Filament path is
validated, then should be removed with the Raylib example in the same major
release that removes the old experiment.
