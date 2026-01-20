# GUI/Visualization Library Migration Plan

> **Created**: 2026-01-20
> **Status**: Research Complete, Awaiting Decision
> **Context**: OSG replacement for DART visualization

## Background

DART currently uses **OpenSceneGraph (OSG)** for visualization (`dart/gui/`), but:
- OSG is **legacy and no longer actively maintained**
- Need modern, future-proof replacement
- Two potential paths: simple (Raylib) vs full-featured (VSG)

## Current DART GUI Architecture

From `dart/gui/`:

| Component | File | Purpose |
|-----------|------|---------|
| Viewer | `Viewer.hpp` | Main window, camera, lighting, headless support |
| ImGuiViewer | `ImGuiViewer.hpp` | Viewer + Dear ImGui widgets |
| WorldNode | `WorldNode.hpp` | Bridge DART World → OSG scene graph |
| ShapeFrameNode | `ShapeFrameNode.hpp` | Per-body-node rendering |
| ShapeNode (abstract) | `render/ShapeNode.hpp` | Base for shape renderers |
| Shape renderers | `render/*ShapeNode.hpp` | Box, Capsule, Cylinder, Sphere, Mesh, etc. |
| DragAndDrop | `DragAndDrop.hpp` | Interactive manipulation |
| InteractiveFrame | `InteractiveFrame.hpp` | 3D gizmo handles |
| TrackballManipulator | `TrackballManipulator.hpp` | Orbit camera control |

**Key Requirements for Replacement**:
1. Scene graph (hierarchical transforms) - **CRITICAL** for robotics
2. ImGui integration - existing widgets depend on it
3. All primitive shapes rendering
4. Selection/picking
5. Shadows
6. Custom shaders
7. Headless rendering
8. Model loading (meshes for URDF)

---

## Candidates Evaluated

### 1. Raylib

**What**: Simple C game development library with 3D support.

| Aspect | Assessment |
|--------|------------|
| Scene Graph | ❌ **NOT BUILT-IN** - must implement manually |
| Primitives | ✅ `DrawSphere`, `DrawCube`, `DrawCapsule`, `DrawCylinder` directly |
| ImGui | ✅ rlImGui well-supported |
| Shadows | ⚠️ Basic, requires custom implementation |
| Picking | ⚠️ `GetRayCollision*` helpers, manual scene-level |
| Shaders | ✅ Custom GLSL supported |
| Models | ✅ GLTF, OBJ, IQM |
| Dependencies | ✅ Self-contained, bundles everything |
| Build | ✅ Simple CMake |
| Learning Curve | ✅ Very low, excellent docs |
| Performance | ✅ Good for real-time |
| Maturity | ✅ 10+ years, 28k+ GitHub stars |

**Pros**:
- Dead simple API
- All primitives directly available
- Self-contained (no external deps)
- Great for quick prototyping

**Cons**:
- **No scene graph** - fundamental gap for robotics
- Would need significant custom code for hierarchical transforms
- Different paradigm from OSG

### 2. VulkanSceneGraph (VSG)

**What**: Modern Vulkan-based scene graph by the same author as OSG.

| Aspect | Assessment |
|--------|------------|
| Scene Graph | ✅ **CORE FEATURE** - native hierarchical transforms |
| Primitives | ✅ Node-based, similar to OSG |
| ImGui | ⚠️ vsgImGui addon available |
| Shadows | ✅ Modern shadow techniques built-in |
| Picking | ✅ Intersection visitors |
| Shaders | ✅ Vulkan SPIR-V, modern pipeline |
| Models | ✅ vsgXchange for GLTF, OBJ, etc. |
| Dependencies | ⚠️ Vulkan SDK required |
| Build | ⚠️ CMake, needs Vulkan setup |
| Learning Curve | ⚠️ Medium (but easier from OSG) |
| Performance | ✅ Excellent (Vulkan, multi-threaded) |
| Maturity | ⚠️ Newer (1.7k stars) but active |

**Pros**:
- Same author as OSG - natural migration path
- Proper scene graph for robotics hierarchies
- Vulkan performance (future-proof)
- Similar concepts to OSG (easier migration)

**Cons**:
- Vulkan SDK dependency
- Steeper learning curve than raylib
- Smaller community than OSG

### 3. Polyscope (For Quick Debugging Only)

**What**: C++/Python viewer for 3D data (meshes, point clouds).

| Aspect | Assessment |
|--------|------------|
| API | ✅ Minimal: `init()`, `registerSurfaceMesh()`, `show()` |
| Vectors | ✅ Built-in vector field visualization |
| Points | ✅ Built-in point cloud support |
| Render Loop | ✅ Not needed - library handles it |
| License | ✅ MIT |

**Best for**: Quick visual debugging of collision results (contacts, normals).
**Not suitable for**: Full DART GUI replacement.

### 4. Rerun (For Logging/Debugging Only)

**What**: Logging + visualization tool for robotics/AI data.

| Aspect | Assessment |
|--------|------------|
| Primitives | ✅ Boxes3D, Capsules3D, Cylinders3D, Ellipsoids3D, Mesh3D |
| Vectors | ✅ Arrows3D |
| Points | ✅ Points3D |
| Dependencies | ⚠️ Apache Arrow (heavy) |
| Viewer | ⚠️ Separate app required |

**Best for**: Time-series debugging, multimodal logging.
**Not suitable for**: Full DART GUI replacement (heavyweight).

---

## Head-to-Head: Raylib vs VSG

| Criterion | Raylib | VSG | Winner |
|-----------|--------|-----|--------|
| Scene Graph | ❌ Manual | ✅ Native | **VSG** |
| OSG Migration Ease | ❌ Different paradigm | ✅ Same author | **VSG** |
| Primitive Rendering | ✅ Direct | ✅ Node-based | Tie |
| ImGui Integration | ✅ rlImGui | ⚠️ vsgImGui | Raylib |
| Build Simplicity | ✅ Self-contained | ⚠️ Vulkan SDK | Raylib |
| Learning (from scratch) | ✅ Low | ⚠️ Medium | Raylib |
| Learning (from OSG) | ⚠️ High | ✅ Low | **VSG** |
| Performance | ✅ Good | ✅ Excellent | **VSG** |
| Future-Proofing | ⚠️ OpenGL | ✅ Vulkan | **VSG** |
| Robotics Suitability | ⚠️ Needs custom work | ✅ Built for this | **VSG** |

---

## Recommendation

### Two-Track Approach

#### Track 1: Quick Debug Tool (Raylib)
**Purpose**: Standalone collision visualization for `dart/collision/experimental`
**Scope**: 
- Render shapes (box, sphere, capsule, cylinder, mesh)
- Show contact points and normals
- Simple camera controls
- No DART integration needed

**Benefits**:
- Quick to implement
- Learn raylib API
- Useful immediately for collision debugging
- Low risk, isolated from main codebase

#### Track 2: Full GUI Replacement (VSG)
**Purpose**: Replace OSG in `dart/gui/`
**Scope**:
- Port WorldNode, ShapeFrameNode, ShapeNode hierarchy
- Port ImGui integration via vsgImGui
- Port all shape renderers
- Port DragAndDrop, InteractiveFrame
- Maintain API compatibility where possible

**Benefits**:
- Proper scene graph for robotics
- Vulkan performance
- Future-proof
- Natural OSG migration

### Decision Matrix

| If your priority is... | Choose |
|------------------------|--------|
| Quick collision debugging NOW | Raylib |
| Long-term DART visualization | VSG |
| Both (recommended) | Raylib first, then VSG |

---

## Next Steps

### Immediate (Collision Debug Tool)
- [ ] Create `tools/collision_debug/` with raylib
- [ ] Implement shape rendering (primitives + mesh)
- [ ] Add contact point/normal visualization
- [ ] Test with experimental collision module

### Future (OSG Replacement)
- [ ] Prototype VSG integration in `dart/gui/experimental/`
- [ ] Port basic Viewer functionality
- [ ] Port ImGui via vsgImGui
- [ ] Port shape renderers
- [ ] Performance comparison with OSG
- [ ] Full migration plan

---

## Resources

### Raylib
- Website: https://www.raylib.com/
- GitHub: https://github.com/raysan5/raylib (28k+ stars)
- Cheatsheet: https://www.raylib.com/cheatsheet/cheatsheet.html
- rlImGui: https://github.com/raylib-extras/rlImGui

### VulkanSceneGraph (VSG)
- Website: https://vsg-dev.github.io/VulkanSceneGraph/
- GitHub: https://github.com/vsg-dev/VulkanSceneGraph (1.7k stars)
- Tutorial: https://github.com/vsg-dev/vsgTutorial
- Examples: https://github.com/vsg-dev/vsgExamples
- vsgImGui: https://github.com/vsg-dev/vsgImGui

### Polyscope (Debug Only)
- Website: https://polyscope.run/
- GitHub: https://github.com/nmwsharp/polyscope (2.1k stars)

### Rerun (Debug Only)
- Website: https://rerun.io/
- Docs: https://rerun.io/docs/getting-started/quick-start

---

## Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-01-20 | Research complete | Evaluated Raylib, VSG, Polyscope, Rerun |
| 2026-01-20 | Two-track approach recommended | Raylib for quick debug, VSG for full replacement |
| | Awaiting decision | Need to decide which track to pursue first |
