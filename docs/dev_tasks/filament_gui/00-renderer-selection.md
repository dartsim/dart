# Filament GUI Replacement - Renderer Selection

## Decision

Use Filament as the renderer candidate for the next `dart::gui` experiment, with
GLFW for native windows/input and Dear ImGui for internal debug controls.

This is a single-renderer direction. The DART-owned scene layer is a
maintainability boundary, not a promise to support multiple backends.

## Why Filament

Filament is the best fit among the investigated options because it combines:

- real-time PBR materials and physically based lighting
- directional, point, and spot shadows, including cascaded and soft-shadow
  options
- screen-space contact shadows, ambient occlusion, tone mapping, and
  anti-aliasing
- glTF-oriented asset tooling and a standalone material compiler
- embeddable native C++ API suitable for a DART-owned viewer
- a scope that can stay focused on built-in visualization and debugging

The main risk is packaging, not rendering capability. Filament is not currently
available through the normal DART Pixi dependency set, so promotion depends on
either conda-forge packaging, a compatible system package story, or an explicit
pinned CMake fetch/build path.

## Candidate assessment

| Candidate              | Assessment                                                                                  | Outcome                                      |
| ---------------------- | ------------------------------------------------------------------------------------------- | -------------------------------------------- |
| Filament               | Strong real-time rendering quality, PBR, shadows, glTF tooling, embeddable C++ API.         | Preferred renderer candidate.                |
| OpenSceneGraph         | Existing path, mature scene graph, but aging rendering model and public API leakage.        | Keep as legacy until replacement is ready.   |
| VulkanSceneGraph       | Modern lower-level direction, but raises integration and maintenance cost for DART.         | Not the first replacement candidate.         |
| Raylib                 | Excellent lightweight smoke/demo stack, but not enough visual-quality headroom.             | Keep only as legacy experiment for now.      |
| GLFW                   | Window/input library, not a renderer.                                                       | Use with Filament.                           |
| Dear ImGui             | Debug UI library, not a renderer.                                                           | Use privately for controls/panels.           |
| Viser                  | Useful remote/browser visualization model, but not a native built-in DART GUI foundation.   | Consider separately for remote workflows.    |
| Rerun                  | Useful logging/time-series visualization model, but not a direct interactive GUI backend.   | Consider separately for telemetry workflows. |
| Zero-dependency custom | Maximum control, but would make DART own rendering quality, platform, and asset complexity. | Reject for this scope.                       |

## Required mix

The first serious experiment should therefore be:

```text
Filament renderer + GLFW window/input + private Dear ImGui debug controls
```

Rerun, Viser, or similar tools can still be useful adjacent workflows, but they
should not be treated as replacements for the maintained in-process `dart::gui`
viewer.
