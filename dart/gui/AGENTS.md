# dart/gui/

Agent guidelines for the GUI/rendering module.

## Overview

3D visualization using OpenSceneGraph (OSG) and Dear ImGui.

## Key Components

| Component         | File                    | Purpose                       |
| ----------------- | ----------------------- | ----------------------------- |
| Viewer            | `Viewer.hpp`            | Main window, camera, lighting |
| ImGuiViewer       | `ImGuiViewer.hpp`       | Viewer + ImGui widgets        |
| WorldNode         | `WorldNode.hpp`         | Bridge World → OSG scene      |
| RealTimeWorldNode | `RealTimeWorldNode.hpp` | Real-time simulation loop     |
| DragAndDrop       | `DragAndDrop.hpp`       | Interactive manipulation      |
| InteractiveFrame  | `InteractiveFrame.hpp`  | 3D manipulator handles        |

## Subdirectories

- `render/` - Shape-specific renderers (BoxShapeNode, MeshShapeNode, etc.)
- `glut/` - Legacy GLUT-based viewer (deprecated)

## Code Patterns

- Override `customPreStep()`/`customPostStep()` for control hooks
- Use `viewer.enableDragAndDrop(bodyNode)` for IK manipulation
- ImGui widgets: inherit `ImGuiWidget`, implement `render()`

## Testing

Unit tests: `tests/unit/gui/`
Examples: `examples/imgui/`, `examples/drag_and_drop/`

## See Also

- @docs/onboarding/gui-rendering.md - OSG integration details
- @examples/imgui/README.md - ImGui usage example
