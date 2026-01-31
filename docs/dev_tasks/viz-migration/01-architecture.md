# Visualization Migration — Architecture

## Problem Statement

`dart::gui` is tightly coupled to OpenSceneGraph (OSG). Its public headers expose OSG types
(`dart::gui::Viewer` inherits `osgViewer::Viewer`, `WorldNode` inherits `osg::Group`). OSG is
effectively unmaintained (last release Jan 2020, last commit Dec 2022) and broken on macOS 10.15+.

## Solution: `dart::gui` — Renderer-Agnostic Visualization

### Three-Layer Architecture

```
┌───────────────────────────────────────────────────┐
│  Layer 3: Backend Implementations (pluggable)     │
│  ┌──────────────┐  ┌───────────┐  ┌───────────┐  │
│  │ Raylib       │  │ OSG       │  │ Future:   │  │
│  │ (default)    │  │ (legacy)  │  │ VSG/Web   │  │
│  └──────┬───────┘  └─────┬─────┘  └─────┬─────┘  │
├─────────┴─────────────────┴──────────────┴────────┤
│  Layer 2: dart::gui — Abstract Scene API          │
│  • Scene (pure data: nodes, materials, lights)    │
│  • SceneViewer (lifecycle, camera, input events)  │
│  • SceneExtractor (World → Scene traversal)       │
│  • NO graphics dependencies in this layer         │
├───────────────────────────────────────────────────┤
│  Layer 1: DART Physics Engine Core                │
│  • dart::simulation::World                        │
│  • dart::dynamics (Skeletons, BodyNodes, Shapes)  │
└───────────────────────────────────────────────────┘
```

### Key Design Principles

1. **No backend leakage**: `dart/viz/*.hpp` headers expose zero Raylib/OSG/OpenGL types
2. **Scene as pure data**: `dart::viz::Scene` is a snapshot of renderables — no GPU resources
3. **Viewer owns the loop**: Backends implement `ViewerBackend` to own window/context/render loop
4. **Extension via layers**: Custom debug drawing and UI through DART-owned abstractions
5. **Small, opinionated API**: Default usage covers common workflows with minimal boilerplate

### Module Structure

```
dart/gui/
├── CMakeLists.txt          # Core module (no GPU deps)
├── export.hpp              # DLL export macros (shared with OSG code)
├── fwd.hpp                 # Forward declarations (appended with new types)
├── scene.hpp               # Scene data (nodes, materials, lights, camera)
├── input_event.hpp         # Backend-agnostic input events
├── viewer_backend.hpp      # Abstract backend interface
├── scene_viewer.hpp        # Main viewer class (delegates to backend)
├── scene_extractor.hpp     # World → Scene conversion
├── raylib/                 # Raylib backend (separate library)
│   ├── CMakeLists.txt
│   ├── raylib_backend.hpp
│   └── raylib_backend.cpp
└── [existing OSG code]     # viewer.hpp, world_node.hpp, etc. (unchanged)
```

### Scene Data Model

The scene is a flat list of renderable nodes (no tree traversal needed by backends):

```cpp
// dart::viz::SceneNode — one renderable thing
struct SceneNode {
  uint64_t id;                    // Stable identifier for picking
  Eigen::Isometry3d transform;   // World-space pose
  ShapeData shape;                // Variant: box, sphere, cylinder, mesh, etc.
  Material material;              // Color, alpha, wireframe flag
  bool visible = true;
};

// dart::viz::Scene — the full snapshot
struct Scene {
  std::vector<SceneNode> nodes;
  std::vector<DebugPrimitive> debug;  // Lines, points, arrows
  Camera camera;
  std::vector<Light> lights;
};
```

### Viewer Lifecycle

```cpp
auto viewer = dart::gui::SceneViewer(
    std::make_unique<dart::gui::RaylibBackend>(), config);
viewer.setWorld(world);
viewer.run();  // Blocking; or viewer.frame() in a loop
```

Internally: `SceneViewer` holds a `SceneExtractor` and a `ViewerBackend`. Each frame:

1. `SceneExtractor` traverses `World` → populates `Scene`
2. `ViewerBackend::render(scene)` draws the scene
3. `ViewerBackend` collects input → `SceneViewer` dispatches events

### Backend Interface

```cpp
class ViewerBackend {
public:
  virtual ~ViewerBackend() = default;
  virtual bool initialize(const ViewerConfig& config) = 0;
  virtual bool shouldClose() const = 0;
  virtual void beginFrame() = 0;
  virtual void render(const Scene& scene) = 0;
  virtual void endFrame() = 0;
  virtual void shutdown() = 0;
  virtual std::vector<InputEvent> pollEvents() = 0;
};
```

### Why Raylib as Default

- Zero external dependencies (everything vendored)
- WebAssembly support (DART demos in browser)
- Excellent debug drawing primitives (DrawLine3D, DrawCapsuleWires, etc.)
- 1-3 MB binary vs 20-50 MB for OSG
- Trivial CMake integration (FetchContent)
- Hardware instancing for many-body scenes

### Relationship to Existing Code

- `dart::gui` (OSG): Existing OSG-based viewer code remains unchanged in `dart/gui/viewer.hpp`, `dart/gui/world_node.hpp`, etc.
- New scene visualization code: Added to `dart/gui/` alongside OSG code (scene.hpp, scene_viewer.hpp, scene_extractor.hpp, etc.)
- `dart/gui/raylib/`: Raylib backend implementation (separate library target)
- `docs/dev_tasks/raylib/`: Original exploration docs. This epic evolves that plan.
- `examples/raylib/`: Updated to use new `dart::gui::SceneViewer` API.
