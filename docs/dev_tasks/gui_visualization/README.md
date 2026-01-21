# GUI/Visualization: VSG Migration Plan

> **Created**: 2026-01-20
> **Updated**: 2026-01-20
> **Status**: Phase 1-4 COMPLETE - VSG foundation + ImGui integration
> **Decision**: VulkanSceneGraph (VSG) for all visualization needs

## Background

DART currently uses **OpenSceneGraph (OSG)** for visualization (`dart/gui/`), but:

- OSG is **legacy and no longer actively maintained**
- Need modern, future-proof replacement
- **Decision**: VSG - same author as OSG, Vulkan-based, proper scene graph

## Why VSG

| Aspect        | VSG Advantage                                             |
| ------------- | --------------------------------------------------------- |
| Scene Graph   | ✅ Native hierarchical transforms (critical for robotics) |
| OSG Migration | ✅ Same author, similar concepts, natural migration path  |
| Performance   | ✅ Vulkan-based, multi-threaded, future-proof             |
| Primitives    | ✅ Node-based geometry, similar to OSG patterns           |
| Shaders       | ✅ Modern Vulkan SPIR-V pipeline                          |

---

## Strategy: Building Blocks Approach

Build reusable VSG components that:

1. **Immediately useful** for collision detector visualization
2. **Reusable** for future full OSG→VSG migration
3. **Minimal scope** - don't over-engineer

### What We're NOT Doing (Yet)

- Full dart/gui/ replacement
- WorldNode/ShapeFrameNode porting
- Simulation visualization
- URDF model loading

### What We ARE Doing

- Core VSG setup (window, viewer, camera)
- Shape geometry builders (reusable)
- Transform utilities (Eigen ↔ VSG)
- Debug visualization (points, lines, arrows)
- Simple collision scene viewer

---

## Phase 1: VSG Foundation (Building Blocks)

These components will be reusable for any future VSG work.

### 1.1 Project Setup

```
dart/gui/vsg/                    # New VSG module
├── CMakeLists.txt               # VSG dependency, optional build
├── export.hpp                   # DLL export macros
├── fwd.hpp                      # Forward declarations
└── ...
```

**Tasks**:

- [x] Add VSG as optional dependency in pixi.toml / CMake
- [x] Create dart/gui/vsg/ module structure
- [x] Verify VSG builds on Linux (CI later)

### 1.2 Geometry Builders (Reusable)

Convert collision shapes to VSG geometry. These map directly to future ShapeNode replacements.

```cpp
namespace dart::gui::vsg {

// Shape → VSG geometry conversion (reusable building blocks)
::vsg::ref_ptr<::vsg::Node> createSphere(double radius, const Options& opts = {});
::vsg::ref_ptr<::vsg::Node> createBox(const Eigen::Vector3d& size, const Options& opts = {});
::vsg::ref_ptr<::vsg::Node> createCapsule(double radius, double height, const Options& opts = {});
::vsg::ref_ptr<::vsg::Node> createCylinder(double radius, double height, const Options& opts = {});
::vsg::ref_ptr<::vsg::Node> createPlane(const Eigen::Vector3d& normal, double offset, const Options& opts = {});
::vsg::ref_ptr<::vsg::Node> createMesh(const std::vector<Eigen::Vector3d>& vertices,
                                        const std::vector<std::array<int,3>>& triangles,
                                        const Options& opts = {});

// From experimental collision shapes
::vsg::ref_ptr<::vsg::Node> createFromShape(const collision::experimental::Shape& shape,
                                             const Options& opts = {});

} // namespace dart::gui::vsg
```

**Files**:

- `geometry_builders.hpp/.cpp` - Shape creation utilities

**Tasks**:

- [ ] Implement sphere geometry builder
- [ ] Implement box geometry builder
- [ ] Implement capsule geometry builder
- [ ] Implement cylinder geometry builder
- [ ] Implement plane geometry builder
- [ ] Implement mesh geometry builder
- [ ] Add `createFromShape()` dispatcher for experimental::Shape

### 1.3 Transform Utilities (Reusable)

```cpp
namespace dart::gui::vsg {

// Eigen ↔ VSG conversions
::vsg::dmat4 toVsg(const Eigen::Isometry3d& transform);
::vsg::dvec3 toVsg(const Eigen::Vector3d& vec);
::vsg::dvec4 toVsg(const Eigen::Vector4d& vec);

Eigen::Isometry3d toEigen(const ::vsg::dmat4& mat);
Eigen::Vector3d toEigen(const ::vsg::dvec3& vec);

// Create transform node
::vsg::ref_ptr<::vsg::MatrixTransform> createTransform(const Eigen::Isometry3d& tf);

} // namespace dart::gui::vsg
```

**Files**:

- `conversions.hpp/.cpp` - Type conversions

**Tasks**:

- [x] Implement Eigen → VSG matrix conversion
- [x] Implement Eigen → VSG vector conversions
- [x] Implement VSG → Eigen conversions
- [x] Add createTransform() helper

### 1.4 Color/Material Utilities (Reusable)

```cpp
namespace dart::gui::vsg {

struct MaterialOptions {
  Eigen::Vector4d color = {0.7, 0.7, 0.7, 1.0};
  bool wireframe = false;
  double shininess = 32.0;
};

// Create basic material/shader setup
::vsg::ref_ptr<::vsg::StateGroup> createMaterial(const MaterialOptions& opts);

// Preset colors for debug visualization
namespace colors {
  constexpr Eigen::Vector4d Red    = {1.0, 0.2, 0.2, 1.0};
  constexpr Eigen::Vector4d Green  = {0.2, 1.0, 0.2, 1.0};
  constexpr Eigen::Vector4d Blue   = {0.2, 0.2, 1.0, 1.0};
  constexpr Eigen::Vector4d Yellow = {1.0, 1.0, 0.2, 1.0};
  constexpr Eigen::Vector4d Cyan   = {0.2, 1.0, 1.0, 1.0};
  constexpr Eigen::Vector4d Orange = {1.0, 0.5, 0.2, 1.0};
  constexpr Eigen::Vector4d White  = {1.0, 1.0, 1.0, 1.0};
  constexpr Eigen::Vector4d Gray   = {0.5, 0.5, 0.5, 1.0};
}

} // namespace dart::gui::vsg
```

**Files**:

- `materials.hpp/.cpp` - Material/color utilities

**Tasks**:

- [ ] Implement basic material creation
- [ ] Add preset colors
- [ ] Add wireframe support

### 1.5 Debug Visualization (Reusable)

```cpp
namespace dart::gui::vsg {

// Debug primitives for visualization
::vsg::ref_ptr<::vsg::Node> createPoint(const Eigen::Vector3d& pos,
                                         double size = 0.02,
                                         const Eigen::Vector4d& color = colors::Red);

::vsg::ref_ptr<::vsg::Node> createLine(const Eigen::Vector3d& start,
                                        const Eigen::Vector3d& end,
                                        const Eigen::Vector4d& color = colors::White);

::vsg::ref_ptr<::vsg::Node> createArrow(const Eigen::Vector3d& start,
                                         const Eigen::Vector3d& direction,
                                         double length = 0.1,
                                         const Eigen::Vector4d& color = colors::Blue);

// Batch creation for performance
::vsg::ref_ptr<::vsg::Node> createPoints(const std::vector<Eigen::Vector3d>& positions,
                                          double size = 0.02,
                                          const Eigen::Vector4d& color = colors::Red);

::vsg::ref_ptr<::vsg::Node> createArrows(const std::vector<Eigen::Vector3d>& starts,
                                          const std::vector<Eigen::Vector3d>& directions,
                                          double length = 0.1,
                                          const Eigen::Vector4d& color = colors::Blue);

} // namespace dart::gui::vsg
```

**Files**:

- `debug_draw.hpp/.cpp` - Debug visualization primitives

**Tasks**:

- [ ] Implement point rendering
- [ ] Implement line rendering
- [ ] Implement arrow rendering (line + cone tip)
- [ ] Implement batch versions

---

## Phase 2: Simple Viewer (Building Block)

Basic viewer that can be reused/extended later.

### 2.1 Simple Viewer

```cpp
namespace dart::gui::vsg {

class SimpleViewer {
public:
  SimpleViewer(int width = 800, int height = 600, const std::string& title = "DART VSG");
  ~SimpleViewer();

  // Scene management
  void setScene(::vsg::ref_ptr<::vsg::Node> scene);
  ::vsg::ref_ptr<::vsg::Group> getRoot();

  // Add/remove nodes
  void addNode(::vsg::ref_ptr<::vsg::Node> node);
  void removeNode(::vsg::ref_ptr<::vsg::Node> node);
  void clear();

  // Camera
  void lookAt(const Eigen::Vector3d& eye, const Eigen::Vector3d& center, const Eigen::Vector3d& up);
  void resetCamera();

  // Render
  void frame();           // Render single frame
  void run();             // Run until window closed
  bool shouldClose();

private:
  // VSG internals
  ::vsg::ref_ptr<::vsg::Viewer> mViewer;
  ::vsg::ref_ptr<::vsg::Group> mRoot;
  ::vsg::ref_ptr<::vsg::Window> mWindow;
  // ... camera, etc.
};

} // namespace dart::gui::vsg
```

**Files**:

- `SimpleViewer.hpp/.cpp` - Basic VSG viewer

**Tasks**:

- [ ] Implement window creation
- [ ] Implement basic camera (orbit controls)
- [ ] Implement scene graph root
- [ ] Implement frame() and run()
- [ ] Add grid/axes helpers

---

## Phase 3: Collision Visualization (Specific Use Case)

Minimal additions for collision module visualization.

### 3.1 Collision Scene Builder

```cpp
namespace dart::gui::vsg {

// Visualize collision world state
class CollisionSceneBuilder {
public:
  // Add collision objects
  void addObject(const collision::experimental::CollisionObject& obj,
                 const Eigen::Vector4d& color = colors::Gray);

  // Add collision results
  void addContacts(const collision::experimental::CollisionResult& result,
                   double normalLength = 0.1);

  // Add CCD results
  void addSphereCast(const Eigen::Vector3d& start,
                     const Eigen::Vector3d& end,
                     double radius,
                     const collision::experimental::CcdResult* hit = nullptr);

  // Build scene graph
  ::vsg::ref_ptr<::vsg::Node> build();

  // Clear for next frame
  void clear();

private:
  std::vector<::vsg::ref_ptr<::vsg::Node>> mNodes;
};

} // namespace dart::gui::vsg
```

**Files**:

- `CollisionSceneBuilder.hpp/.cpp` - Collision-specific visualization

**Tasks**:

- [ ] Implement addObject() using geometry builders
- [ ] Implement addContacts() with points + normals
- [ ] Implement addSphereCast() visualization
- [ ] Implement build() to compose scene graph

### 3.2 Example/Tool

Simple executable to test collision visualization:

```
tools/collision_viz/
├── CMakeLists.txt
└── main.cpp
```

```cpp
// Example usage
int main() {
  using namespace dart::collision::experimental;
  using namespace dart::gui::vsg;

  // Create collision world
  CollisionWorld world;
  auto box = world.createObject(std::make_unique<BoxShape>(Eigen::Vector3d(1,1,1)));
  auto sphere = world.createObject(std::make_unique<SphereShape>(0.5),
                                    Eigen::Translation3d(2, 0, 0) * Eigen::Isometry3d::Identity());

  // Collide
  CollisionResult result;
  world.collide(CollisionOption(), result);

  // Visualize
  SimpleViewer viewer;
  CollisionSceneBuilder builder;
  builder.addObject(box, colors::Blue);
  builder.addObject(sphere, colors::Green);
  builder.addContacts(result);
  viewer.setScene(builder.build());
  viewer.run();

  return 0;
}
```

**Tasks**:

- [ ] Create tools/collision_viz/ structure
- [ ] Implement example showing shapes
- [ ] Implement example showing contacts
- [ ] Implement example showing CCD (sphere-cast)

---

## File Structure

```
dart/gui/vsg/
├── CMakeLists.txt
├── export.hpp
├── fwd.hpp
├── conversions.hpp              # Eigen ↔ VSG
├── conversions.cpp
├── materials.hpp                # Colors, materials
├── materials.cpp
├── geometry_builders.hpp        # Shape → VSG geometry
├── geometry_builders.cpp
├── debug_draw.hpp               # Points, lines, arrows
├── debug_draw.cpp
├── SimpleViewer.hpp             # Basic viewer
├── SimpleViewer.cpp
├── CollisionSceneBuilder.hpp    # Collision-specific
└── CollisionSceneBuilder.cpp

tools/collision_viz/
├── CMakeLists.txt
└── main.cpp
```

---

## Mapping to Future OSG Migration

| Building Block       | Future Use (dart/gui replacement)  |
| -------------------- | ---------------------------------- |
| `geometry_builders`  | → `render/*ShapeNode` replacements |
| `conversions`        | → Used everywhere                  |
| `materials`          | → Material system                  |
| `debug_draw`         | → Debug visualization layer        |
| `SimpleViewer`       | → Base for `Viewer` replacement    |
| Scene graph patterns | → `WorldNode`, `ShapeFrameNode`    |

---

## Dependencies

### Required

- **VulkanSceneGraph (VSG)** - core library
- **Vulkan SDK** - graphics API (runtime)

### Build Integration

```cmake
# CMake
find_package(vsg REQUIRED)
target_link_libraries(dart-gui-vsg PUBLIC vsg::vsg)
```

```toml
# pixi.toml (if available via conda-forge)
[dependencies]
vulkanscenegraph = ">=1.0"
```

### Optional (for later phases)

- vsgImGui - ImGui integration
- vsgXchange - model loading (GLTF, OBJ)

---

## Success Criteria

### Phase 1 Complete When:

- [x] VSG builds in DART project
- [x] All geometry builders work (sphere, box, capsule, cylinder, plane, mesh)
- [x] Transform conversions work
- [x] Debug primitives work (points, lines, arrows)

### Phase 2 Complete When:

- [x] SimpleViewer displays shapes
- [x] Camera orbit controls work
- [x] Can render collision world objects

### Phase 3 Complete When:

- [x] CollisionSceneBuilder visualizes collision results
- [x] Example tool works end-to-end
- [x] Can visualize CCD (sphere-cast, capsule-cast)

---

## Phase 4: ImGui Integration (COMPLETE)

Added interactive controls using vsgImGui.

### 4.1 vsgImGui FetchContent

vsgImGui is not on conda-forge, so we use CMake FetchContent.

**Key Challenges Solved**:

- Target conflict: vsgImGui creates `docs`, `clobber`, `uninstall` targets that conflict with DART
- Solution: Create stub targets before `add_subdirectory(vsgImGui)`
- C++20 enforcement: vsgImGui sets C++17 but DART uses C++20
- Solution: Explicit `target_compile_features(dart-gui-vsg PUBLIC cxx_std_20)`

### 4.2 ImGuiViewer Class

New viewer class extending SimpleViewer with ImGui support.

**Files**:

- `ImGuiViewer.hpp/.cpp` - ImGui-enabled VSG viewer

**Features**:

- `setImGuiCallback(std::function<void()>)` - Register custom ImGui widgets
- `computePickingRay(screenX, screenY, origin, direction)` - Screen-to-world ray
- `getWindowSize(width, height)` - Current window dimensions

### 4.3 collision_sandbox Example

Interactive demo showing all collision query types.

**Location**: `examples/collision_sandbox/`

**Demo Modes**:

| Mode      | Description                          |
| --------- | ------------------------------------ |
| Shapes    | Basic primitives (box, sphere, etc.) |
| Contacts  | Collision detection with contacts    |
| Filtering | Collision group/mask filtering       |
| Distance  | Distance between objects             |
| Raycast   | Ray intersection queries             |
| CCD       | Continuous collision (sphere-cast)   |
| Picking   | Mouse raycast picking                |

**Controls**:

- Radio buttons to switch demo modes
- AABB toggle checkbox
- Mode-specific sliders (sphere offset, ray angle)
- Middle-click for mouse picking (in Picking mode)

### Phase 4 Complete When:

- [x] vsgImGui integrated via FetchContent
- [x] ImGuiViewer class with callback support
- [x] collision_sandbox example with all demo modes
- [x] Mouse raycast picking with screen-to-world conversion
- [x] Real collision queries (raycast, sphereCast) instead of mock data

---

## Resources

### VulkanSceneGraph (VSG)

- Website: https://vsg-dev.github.io/VulkanSceneGraph/
- GitHub: https://github.com/vsg-dev/VulkanSceneGraph (1.7k stars)
- Tutorial: https://github.com/vsg-dev/vsgTutorial
- Examples: https://github.com/vsg-dev/vsgExamples
- API Docs: https://vsg-dev.github.io/VulkanSceneGraph/docs/

### Key VSG Concepts

- `vsg::Node` - Base scene graph node
- `vsg::Group` - Container for children
- `vsg::MatrixTransform` - Transform node
- `vsg::StateGroup` - Material/shader state
- `vsg::Geometry` - Renderable geometry
- `vsg::Viewer` - Window + render loop
- `vsg::Builder` - Geometry creation helpers

---

## Decision Log

| Date       | Decision                 | Rationale                                                         |
| ---------- | ------------------------ | ----------------------------------------------------------------- |
| 2026-01-20 | Research complete        | Evaluated multiple options                                        |
| 2026-01-20 | **Choose VSG only**      | Scene graph essential; natural OSG successor; Vulkan future-proof |
| 2026-01-20 | Building blocks approach | Learn VSG incrementally; create reusable components               |
| 2026-01-20 | Collision viz first      | Immediate value; minimal scope; tests building blocks             |
