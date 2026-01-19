# OpenSceneGraph (OSG) GUI Integration Layer Analysis

## Overview

The DART GUI uses OpenSceneGraph (OSG) as its rendering backend to visualize physics simulations. This document provides a comprehensive analysis of the OSG integration layer architecture, rendering pipeline, and event handling systems.

---

## Core Architecture Components

### 1. Viewer (`dart/gui/Viewer.hpp/cpp`)

**Purpose**: Main visualization window that manages the rendering loop and coordinates all visual components.

**Key Responsibilities**:

- Inherits from `osgViewer::Viewer` (OSG's core viewer class)
- Manages multiple `WorldNode` instances (one per DART World)
- Coordinates camera control, lighting, and event handling
- Supports screen capture and recording capabilities
- Manages drag-and-drop interactions

**Core Features**:

1. **Scene Graph Management**:
   - Root group (`mRootGroup`): Top-level scene graph node
   - Light group (`mLightGroup`): Contains light sources
   - Supports multiple world nodes as children

2. **Camera Modes**:
   - `RGBA`: Standard color rendering
   - `DEPTH`: Depth buffer visualization (not compatible with ImGui widgets)
   - Camera mode callback system for dynamic switching

3. **Lighting System**:
   - Headlights: Camera-attached lights
   - Two additional light sources (`mLight1`, `mLight2`)
   - Positioned based on upward direction vector
   - Toggleable headlights vs scene lights

4. **Simulation Control**:
   - Start/stop simulation via `simulate(bool)`
   - Simulation gating via `allowSimulation(bool)`
   - Active world node tracking

5. **ViewerAttachment System**:
   - Base class for custom attachments to the viewer
   - Automatic refresh callbacks via `ViewerAttachmentCallback`
   - Used for extending viewer functionality

6. **Drag and Drop Support**:
   - Multiple DnD types: `SimpleFrameDnD`, `SimpleFrameShapeDnD`, `InteractiveFrameDnD`, `BodyNodeDnD`
   - Centralized management of drag-and-drop objects
   - Integration with Inverse Kinematics for body node manipulation

**Key Member Variables**:

```cpp
::osg::ref_ptr<::osg::Group> mRootGroup;           // Scene root
::osg::ref_ptr<DefaultEventHandler> mDefaultEventHandler; // Event handler
std::map<::osg::ref_ptr<WorldNode>, bool> mWorldNodes;   // Active worlds
std::unordered_set<ViewerAttachment*> mAttachments;      // Custom attachments
bool mSimulating;                                   // Simulation state
bool mAllowSimulation;                             // Simulation permission
```

---

### 2. WorldNode (`dart/gui/WorldNode.hpp/cpp`)

**Purpose**: Encapsulates a DART `simulation::World` for OSG rendering.

**Key Responsibilities**:

- Manages the visual representation of a DART physics world
- Maintains scene graph of all entities and frames in the world
- Handles simulation stepping and refresh cycles
- Supports shadowing with configurable techniques

**Architecture**:

1. **Dual Group System**:

   ```cpp
   ::osg::ref_ptr<::osg::Group> mNormalGroup;          // Non-shadowed objects
   ::osg::ref_ptr<::osgShadow::ShadowedScene> mShadowedGroup; // Shadowed objects
   ```

   - Objects are placed in either group based on their shadow settings
   - Allows selective shadow rendering for performance

2. **Refresh Cycle**:

   ```
   refresh() →
     ├─ customPreRefresh()
     ├─ clearChildUtilizationFlags()
     ├─ [Optional] Simulation Steps
     │   ├─ customPreStep()
     │   ├─ mWorld->step()
     │   └─ customPostStep()
     ├─ refreshSkeletons()
     ├─ refreshSimpleFrames()
     ├─ clearUnusedNodes()
     └─ customPostRefresh()
   ```

3. **Frame Hierarchy Management**:
   - Maintains map: `Frame* → ShapeFrameNode*`
   - Recursively processes frame trees (skeletons and simple frames)
   - Utilization tracking for garbage collection of unused nodes

4. **Customization Hooks**:
   - `customPreRefresh()` / `customPostRefresh()`: Per-cycle callbacks
   - `customPreStep()` / `customPostStep()`: Per-simulation-step callbacks
   - `setupViewer()`: Called when added to a viewer

**Node Lifecycle**:

```
Frame Discovery → ShapeFrameNode Creation → Refresh → Utilization Check → Cleanup
```

---

### 3. RealTimeWorldNode (`dart/gui/RealTimeWorldNode.hpp/cpp`)

**Purpose**: Specialized `WorldNode` that attempts real-time simulation playback.

**Key Features**:

1. **Real-Time Factor (RTF) Control**:
   - Target RTF: Desired speed multiplier (1.0 = real-time)
   - Actual RTF tracking: Monitors achieved performance
   - Performance statistics: Min/max RTF achieved

2. **Adaptive Stepping**:

   ```cpp
   while (mRefreshTimer.time_s() < mTargetRealTimeLapse) {
       const double nextSimTimeLapse = mWorld->getTime() - startSimTime + simTimeStep;
       if (nextSimTimeLapse <= mTargetSimTimeLapse) {
           mWorld->step();
       }
   }
   ```

   - Takes as many steps as possible within the refresh period
   - Respects the target sim time lapse
   - Automatically adjusts to computational load

3. **Timer Management**:
   - `::osg::Timer mRefreshTimer`: Tracks elapsed real time
   - `mTargetRealTimeLapse`: 1.0 / target frequency
   - `mTargetSimTimeLapse`: RTF × real time lapse
   - Reset timer on pause/unpause

**Use Case**: Ideal for real-time interactive simulations where smooth playback is desired.

---

### 4. ShapeFrameNode (`dart/gui/ShapeFrameNode.hpp/cpp`)

**Purpose**: Represents a DART `ShapeFrame` (a frame with an attached visual shape) in the OSG scene graph.

**Key Responsibilities**:

- Inherits from `osg::MatrixTransform` (allows transformation)
- Manages the render node for the actual shape geometry
- Handles shape changes and updates

**Architecture**:

1. **Transformation Management**:

   ```cpp
   setMatrix(eigToOsgMatrix(mShapeFrame->getWorldTransform()));
   ```

   - Updates world transform from DART to OSG coordinates

2. **Shape Node Management**:
   - Holds pointer to `render::ShapeNode*`
   - Creates appropriate shape node based on shape type
   - Replaces shape node when shape changes

3. **Utilization Tracking**:
   - Marks nodes as utilized during refresh
   - Parent `WorldNode` clears unused nodes
   - Prevents memory leaks from deleted frames

4. **Visibility Control**:
   - Only renders if shape exists and has visual aspect
   - Respects visual aspect properties

**Shape Type Dispatch**:
The `createShapeNode()` method creates specialized nodes for each shape type:

- `SphereShapeNode`, `BoxShapeNode`, `EllipsoidShapeNode`
- `CylinderShapeNode`, `CapsuleShapeNode`, `ConeShapeNode`, `PyramidShapeNode`
- `PlaneShapeNode`, `MultiSphereShapeNode`
- `MeshShapeNode`, `SoftMeshShapeNode`
- `LineSegmentShapeNode`, `PointCloudShapeNode`
- `VoxelGridShapeNode` (if OCTOMAP enabled)
- `HeightmapShapeNode`
- `WarningShapeNode` (fallback for unknown types)

---

## Rendering Pipeline

### Shape Rendering Hierarchy

```
render/
├── ShapeNode (abstract base)          - Base class for all shape renderers
├── BoxShapeNode                       - Box primitives
├── SphereShapeNode                    - Sphere primitives
├── CylinderShapeNode                  - Cylinder primitives
├── CapsuleShapeNode                   - Capsule primitives
├── ConeShapeNode                      - Cone primitives
├── PyramidShapeNode                   - Pyramid primitives
├── EllipsoidShapeNode                 - Ellipsoid primitives
├── PlaneShapeNode                     - Infinite planes
├── MultiSphereShapeNode               - Multi-sphere convex hulls
├── MeshShapeNode                      - Triangle meshes (via Assimp)
├── SoftMeshShapeNode                  - Deformable meshes
├── LineSegmentShapeNode               - Line segments
├── PointCloudShapeNode                - Point clouds
├── VoxelGridShapeNode                 - Voxel grids (Octomap)
├── HeightmapShapeNode                 - Heightmap terrain
└── WarningShapeNode                   - Fallback for unsupported shapes
```

### render::ShapeNode Base Class (`dart/gui/render/ShapeNode.hpp/cpp`)

**Purpose**: Abstract base class for all shape-specific rendering nodes.

**Core Data**:

```cpp
const std::shared_ptr<dart::dynamics::Shape> mShape;     // Shape data
dart::dynamics::ShapeFrame* mShapeFrame;                 // Parent frame
dart::dynamics::VisualAspect* mVisualAspect;            // Visual properties
ShapeFrameNode* mParentShapeFrameNode;                  // OSG parent
::osg::Node* mNode;                                     // OSG scene node
bool mUtilized;                                         // GC tracking
```

**Interface**:

- `refresh()`: Pure virtual - update geometry/appearance
- `getShape()`: Access to DART shape
- `getVisualAspect()`: Access to visual properties (color, material, etc.)

**Typical Subclass Pattern**:

1. Inherit from both `ShapeNode` and appropriate OSG node type
2. Extract shape-specific data in constructor
3. Create OSG geometry in `refresh()` or helper method
4. Apply visual aspect properties (color, material, wireframe)

---

### Example: BoxShapeNode

```cpp
class BoxShapeNode : public ShapeNode, public ::osg::Group {
  // Constructor extracts box dimensions
  // refresh() updates geometry if dimensions changed
  // extractData() creates OSG box geometry
};
```

---

### Example: MeshShapeNode

```cpp
class MeshShapeNode : public ShapeNode, public ::osg::MatrixTransform {
  // Uses Assimp (aiNode) for mesh loading
  // Supports materials and textures
  // Handles mesh scale transformations
};
```

---

## Event Handling System

### 1. DefaultEventHandler (`dart/gui/DefaultEventHandler.hpp/cpp`)

**Purpose**: Central event processor for mouse and keyboard input.

**Key Responsibilities**:

- Inherits from `osgGA::GUIEventHandler`
- Processes mouse and keyboard events
- Performs object picking (raycasting)
- Dispatches events to registered `MouseEventHandler` instances
- Implements Observer pattern for handler lifecycle

**Event Types**:

1. **Mouse Button Events**:

   ```cpp
   enum MouseButtonEvent {
       BUTTON_PUSH,        // Initial click
       BUTTON_DRAG,        // Dragging with button held
       BUTTON_RELEASE,     // Button released
       BUTTON_NOTHING      // No event
   };
   ```

2. **Mouse Buttons**:
   - `LEFT_MOUSE`: Typically for interaction/selection
   - `RIGHT_MOUSE`: Typically for camera rotation
   - `MIDDLE_MOUSE`: Typically for camera pan

3. **Keyboard Shortcuts**:
   - `Spacebar`: Toggle simulation on/off
   - `Ctrl+H`: Toggle headlights

**Picking System** (`PickInfo`):

```cpp
struct PickInfo {
    dart::dynamics::ShapeFrame* frame;    // Picked frame
    std::shared_ptr<dart::dynamics::Shape> shape; // Picked shape
    Eigen::Vector3d position;             // World-space intersection point
    Eigen::Vector3d normal;               // Surface normal at intersection
};
```

**Picking Process**:

1. Compute line segment intersection using OSG utilities
2. Extract `render::ShapeNode` from hit drawable
3. Retrieve associated DART shape and frame
4. Convert intersection data to Eigen types

**Cursor Delta Calculation**:

- `getDeltaCursor()`: Projects cursor movement into 3D space
- **Constraint Types**:
  - `UNCONSTRAINED`: Project onto plane parallel to camera
  - `LINE_CONSTRAINT`: Constrain to a line
  - `PLANE_CONSTRAINT`: Constrain to a plane

**Pick Suppression**:

- Can suppress picks for specific button/event combinations
- Useful for implementing custom behaviors
- Per-button, per-event control

**MouseEventHandler Registry**:

- Add handlers via `addMouseEventHandler()`
- Automatic cleanup via Observer pattern
- Handlers receive `update()` calls on each event

---

### 2. MouseEventHandler (`dart/gui/MouseEventHandler.hpp`)

**Purpose**: Abstract base class for custom mouse event behaviors.

**Interface**:

```cpp
class MouseEventHandler {
public:
    virtual void update() = 0;  // Called on every mouse event
protected:
    DefaultEventHandler* mEventHandler; // Access to event state
};
```

**Typical Usage Pattern**:

```cpp
class MyHandler : public MouseEventHandler {
    void update() override {
        auto button = mEventHandler->getButtonEvent(LEFT_MOUSE);
        if (button == BUTTON_PUSH) {
            auto picks = mEventHandler->getButtonPicks(LEFT_MOUSE, BUTTON_PUSH);
            // Handle picks...
        }
    }
};
```

**Built-in Handlers**:

- `DragAndDrop`: Base class for drag-and-drop interactions
- `SimpleFrameDnD`: Drag simple frames
- `SimpleFrameShapeDnD`: Drag shapes within frames
- `InteractiveFrameDnD`: Drag interactive frames
- `BodyNodeDnD`: Drag body nodes with IK

---

### 3. TrackballManipulator (`dart/gui/TrackballManipulator.hpp/cpp`)

**Purpose**: Camera controller for interactive viewpoint manipulation.

**Key Features**:

- Inherits from `osgGA::OrbitManipulator`
- Customizes mouse button behaviors:
  - **Left Mouse**: Disabled for interaction (not camera control)
  - **Right Mouse**: Orbit rotation (delegated to OrbitManipulator's left button)
  - **Middle Mouse**: Pan (inherited from OrbitManipulator)
  - **Scroll Wheel**: Zoom (inherited from OrbitManipulator)

**Configuration**:

```cpp
setVerticalAxisFixed(false);  // Allow free rotation
setAllowThrow(false);         // Disable momentum-based camera motion
```

**Rationale**:

- Left mouse reserved for object selection/manipulation
- Right mouse provides intuitive camera rotation
- Standard trackball interaction model

---

## Data Flow Architecture

### Complete Rendering Cycle

```
1. Application Loop
   └─> Viewer::frame()
       └─> Viewer::updateViewer()
           ├─> updateDragAndDrops()
           └─> Scene Graph Update Callback
               └─> WorldNode::refresh()
                   ├─> clearChildUtilizationFlags()
                   ├─> [Optional] World Simulation Steps
                   ├─> refreshSkeletons()
                   │   └─> For each Skeleton:
                   │       └─> refreshBaseFrameNode()
                   │           └─> Recursive Frame Processing
                   │               └─> refreshShapeFrameNode()
                   │                   ├─> Get or Create ShapeFrameNode
                   │                   └─> ShapeFrameNode::refresh()
                   │                       ├─> Update Transform
                   │                       └─> refreshShapeNode()
                   │                           └─> render::ShapeNode::refresh()
                   │                               └─> Update Geometry
                   ├─> refreshSimpleFrames()
                   │   └─> Similar to Skeletons
                   └─> clearUnusedNodes()
                       └─> Remove unreferenced ShapeFrameNodes

2. OSG Rendering
   └─> Camera Traversal
       ├─> Cull Pass: Determine visible objects
       ├─> Draw Pass: Render geometry
       └─> [Optional] Shadow Pass

3. Event Processing
   └─> DefaultEventHandler::handle()
       ├─> Update cursor position
       ├─> Process keyboard events
       ├─> Process mouse events
       │   ├─> Perform picking
       │   ├─> Update button states
       │   └─> triggerMouseEventHandlers()
       │       └─> For each MouseEventHandler:
       │           └─> handler->update()
       └─> Update drag-and-drop operations
```

---

## Shadow Rendering

### Shadow Technique Support

**Configuration**:

```cpp
WorldNode::setShadowTechnique(osg::ref_ptr<osgShadow::ShadowTechnique>)
```

**Default Technique** (`createDefaultShadowTechnique`):

- Uses `osgShadow::ShadowMap`
- 4096×4096 shadow texture resolution
- Uses Light #1 (highest positioned light)

**Shadow Groups**:

- `mNormalGroup`: Objects without shadows
- `mShadowedGroup`: Objects with shadows (osgShadow::ShadowedScene)
- Objects automatically placed based on `VisualAspect::getShadowed()`

**Traversal Masks**:

- `ReceivesShadowTraversalMask = 0x2`: Objects that receive shadows
- `CastsShadowTraversalMask = 0x1`: Objects that cast shadows

---

## Screen Capture and Recording

### Screen Capture

**Single Frame Capture**:

```cpp
viewer->captureScreen("screenshot.png");
```

**Implementation**:

- Uses `SaveScreen` callback attached to camera
- Reads pixels via `glReadPixels` (GL_RGB format)
- Saves via `osgDB::writeImageFile`

### Screen Recording

**Continuous Recording**:

```cpp
viewer->record("output_dir", "frame_prefix", restart=false, digits=6);
viewer->pauseRecording();
```

**Features**:

- Automatic frame numbering with zero-padding
- Configurable prefix and number of digits
- Resume capability (restart=false)
- PNG output format

---

## Headless Rendering

### Overview

DART supports headless rendering for CI pipelines, batch frame capture, and testing without a display window. This uses OSG's pbuffer backend to render to an offscreen buffer.

### Key API

```cpp
// Configure headless mode
ViewerConfig config = ViewerConfig::headless(1280, 720);

// Create viewer with config
auto viewer = Viewer::create(config);

// Check if headless
if (viewer->isHeadless()) {
  // Capture frames to files
  viewer->captureScreen("frame_000001.png");

  // Or capture to memory buffer (raw RGBA pixels)
  int width, height;
  std::vector<unsigned char> pixels = viewer->captureBuffer(&width, &height);
}
```

### ViewerConfig Options

| Field                 | Type            | Description                              |
| --------------------- | --------------- | ---------------------------------------- |
| `mode`                | `RenderingMode` | `Window` (default) or `Headless`         |
| `width`               | `int`           | Viewport width (default: 1024)           |
| `height`              | `int`           | Viewport height (default: 768)           |
| `useSoftwareRenderer` | `bool`          | OSMesa placeholder (not yet implemented) |

### CI Integration

Headless mode requires either a display server or virtual framebuffer:

```bash
# Using xvfb-run wrapper (recommended for CI)
xvfb-run --auto-servernum ./my_headless_app --headless --frames 100

# Manual Xvfb setup
Xvfb :99 -screen 0 1280x720x24 &
export DISPLAY=:99
./my_headless_app --headless --frames 100
```

### Example: rigid_cubes

The `rigid_cubes` example demonstrates headless mode with CLI flags:

```bash
./rigid_cubes --headless --frames 100 --out ./output/ --width 1920 --height 1080
```

See [`examples/rigid_cubes/README.md`](../../examples/rigid_cubes/README.md) for full documentation including video creation with ffmpeg.

### Files

- `dart/gui/ViewerConfig.hpp` - `RenderingMode` enum and `ViewerConfig` struct
- `dart/gui/Viewer.hpp` - `Viewer(ViewerConfig)` constructor, `isHeadless()`, `captureBuffer()`
- `.github/workflows/ci_ubuntu.yml` - `headless-rendering` CI job

---

## Camera System

### Camera Modes

1. **RGBA Mode** (Default):
   - Standard color rendering
   - Compatible with all features including ImGui

2. **DEPTH Mode**:
   - Renders depth buffer visualization
   - Not compatible with ImGui widgets
   - Useful for debugging

**Mode Switching**:

```cpp
viewer->setCameraMode(CameraMode::DEPTH);
CameraMode mode = viewer->getCameraMode();
```

### Field of View Control

```cpp
viewer->setVerticalFieldOfView(60.0);  // degrees
double fov = viewer->getVerticalFieldOfView();
```

**Constraints**:

- Only works with perspective projection
- Returns 0.0 for orthographic cameras

---

## Lighting System

### Light Configuration

**Two-Light Setup**:

- **Light #1** (`mLight1`, `mLightSource1`): Primary scene light
- **Light #2** (`mLight2`, `mLightSource2`): Fill light

**Positioning**:

```cpp
mLight1->setPosition(mUpwards + mOver);  // Upper-side light
mLight2->setPosition(mUpwards - mOver);  // Upper-opposite light
```

**Headlights Mode**:

- Headlights: Camera-attached lights
- Scene lights: Fixed-position lights
- Toggle via `switchHeadlights(bool)`

**Light Properties**:

```cpp
// Headlights ON
mLight->setAmbient(0.1, 0.1, 0.1);
mLight->setDiffuse(0.8, 0.8, 0.8);
mLight->setSpecular(1.0, 1.0, 1.0);

// Headlights OFF (Scene Lights Active)
mLight1->setDiffuse(0.7, 0.7, 0.7);
mLight2->setDiffuse(0.3, 0.3, 0.3);
```

### Upward Direction

```cpp
viewer->setUpwardsDirection(Eigen::Vector3d(0, 0, 1));  // Z-up (default)
```

- Affects light positioning
- Influences camera manipulator

---

## Utilization Tracking and Garbage Collection

### Purpose

Prevent memory leaks from deleted DART entities while avoiding dangling pointers.

### Mechanism

1. **Flag Clearing**:

   ```cpp
   WorldNode::clearChildUtilizationFlags()
   ```

   - Called at start of refresh cycle
   - Sets `mUtilized = false` on all ShapeFrameNodes

2. **Utilization Marking**:

   ```cpp
   ShapeFrameNode::refresh()
   ```

   - Sets `mUtilized = true` when refreshed
   - Indicates frame still exists in DART world

3. **Garbage Collection**:

   ```cpp
   WorldNode::clearUnusedNodes()
   ```

   - Removes nodes where `mUtilized == false`
   - Indicates DART entity was deleted

### Benefits

- Automatic cleanup of stale visual nodes
- No manual tracking required
- Handles dynamic entity creation/destruction

---

## Integration Points with DART Core

### DART → OSG Data Flow

1. **World State**:
   - `simulation::World` → `WorldNode`
   - World time, time step, simulation state

2. **Frame Hierarchy**:
   - `dynamics::Frame` → `ShapeFrameNode`
   - World transforms, frame trees

3. **Shape Geometry**:
   - `dynamics::Shape` → `render::ShapeNode`
   - Geometry data (dimensions, vertices, etc.)

4. **Visual Properties**:
   - `dynamics::VisualAspect` → Material/Color settings
   - Color, transparency, wireframe, shadowing

5. **Entity Structure**:
   - `dynamics::Skeleton` → Frame tree processing
   - `dynamics::SimpleFrame` → Direct frame rendering

### OSG → DART Data Flow (Limited)

1. **User Interactions**:
   - Pick events → DART entity selection
   - Drag-and-drop → Entity pose updates, IK solving

2. **Camera State** (Read-only):
   - Cursor position → 3D ray calculations
   - View matrix → Constraint plane calculations

---

## Performance Considerations

### Optimization Strategies

1. **Utilization Tracking**:
   - Short-circuit refresh if already utilized
   - Avoids redundant geometry updates

2. **Shadow Grouping**:
   - Separate shadowed/non-shadowed objects
   - Reduces shadow computation overhead

3. **Shape Node Reuse**:
   - Checks if shape instance changed before recreating node
   - Only refresh existing node if possible

4. **Deferred Deletion**:
   - Node cleanup after refresh cycle
   - Prevents issues with mid-update deletions

### Real-Time Simulation

**RealTimeWorldNode** provides:

- Adaptive stepping based on computational load
- RTF monitoring for performance feedback
- Smooth playback even if simulation is expensive

---

## Extension Points

### Custom Shape Rendering

To add a new shape type:

1. Create `MyShapeNode : public ShapeNode, public ::osg::Node`
2. Implement `refresh()` to update geometry
3. Add case to `ShapeFrameNode::createShapeNode()`

### Custom Event Handlers

To add custom interactions:

1. Inherit from `MouseEventHandler`
2. Implement `update()` method
3. Register with `viewer->getDefaultEventHandler()->addMouseEventHandler()`

### Custom World Behaviors

To customize simulation updates:

1. Inherit from `WorldNode` or `RealTimeWorldNode`
2. Override `customPreRefresh()`, `customPostRefresh()`, etc.
3. Add custom logic before/after standard processing

### Viewer Attachments

To add persistent visual elements:

1. Inherit from `ViewerAttachment`
2. Implement `refresh()` for per-frame updates
3. Add to viewer: `viewer->addAttachment(myAttachment)`

Common attachments you can reuse without writing custom logic:

- `GridVisual` renders reference grid planes that can be toggled or recolored.
- `SupportPolygonVisual` tracks a skeleton's support polygon, centroid, and COM.
- `PolyhedronVisual` converts an arbitrary set of vertices into a convex hull and draws it with a fill and wireframe overlay—handy for visualizing custom polyhedra directly from their V-representation.

---

## File Structure Summary

### Core Components

- `dart/gui/Viewer.hpp/cpp`
  - Main viewer class
- `dart/gui/WorldNode.hpp/cpp`
  - World visualization
- `dart/gui/RealTimeWorldNode.hpp/cpp`
  - Real-time simulation support
- `dart/gui/ShapeFrameNode.hpp/cpp`
  - Frame-to-node mapping

### Event Handling

- `dart/gui/DefaultEventHandler.hpp/cpp`
  - Central event processor
- `dart/gui/MouseEventHandler.hpp`
  - Custom handler base class
- `dart/gui/TrackballManipulator.hpp/cpp`
  - Camera controller

### Rendering

- `dart/gui/render/ShapeNode.hpp/cpp`
  - Base shape renderer
- `dart/gui/render/BoxShapeNode.hpp/cpp`
  - Box geometry rendering
- `dart/gui/render/SphereShapeNode.hpp/cpp`
  - Sphere geometry rendering
- `dart/gui/render/CylinderShapeNode.hpp/cpp`
  - Cylinder geometry rendering
- `dart/gui/render/CapsuleShapeNode.hpp/cpp`
  - Capsule geometry rendering
- `dart/gui/render/ConeShapeNode.hpp/cpp`
  - Cone geometry rendering
- `dart/gui/render/PyramidShapeNode.hpp/cpp`
  - Pyramid geometry rendering
- `dart/gui/render/EllipsoidShapeNode.hpp/cpp`
  - Ellipsoid geometry rendering
- `dart/gui/render/PlaneShapeNode.hpp/cpp`
  - Plane geometry rendering
- `dart/gui/render/MultiSphereShapeNode.hpp/cpp`
  - Multi-sphere convex hull rendering
- `dart/gui/render/MeshShapeNode.hpp/cpp`
  - Mesh geometry rendering (using Assimp)
- `dart/gui/render/SoftMeshShapeNode.hpp/cpp`
  - Deformable mesh rendering
- `dart/gui/render/LineSegmentShapeNode.hpp/cpp`
  - Line segment rendering
- `dart/gui/render/PointCloudShapeNode.hpp/cpp`
  - Point cloud rendering
- `dart/gui/render/VoxelGridShapeNode.hpp/cpp`
  - Voxel grid rendering (requires OCTOMAP)
- `dart/gui/render/HeightmapShapeNode.hpp`
  - Heightmap terrain rendering
- `dart/gui/render/WarningShapeNode.hpp/cpp`
  - Fallback renderer for unsupported shapes

---

## Key Design Patterns

### 1. Observer Pattern

- `DefaultEventHandler` observes `MouseEventHandler` instances
- Automatic cleanup when handlers are destroyed
- Subject/Observer from DART common library

### 2. Visitor Pattern (via OSG)

- Node callbacks for scene graph traversal
- Update callbacks for per-frame logic
- Draw callbacks for screen capture

### 3. Factory Pattern

- `ShapeFrameNode::createShapeNode()` creates appropriate shape nodes
- Type-based dispatch for different shape types
- Extensible for new shape types

### 4. Utilization Tracking Pattern

- Mark-and-sweep garbage collection
- Prevents dangling pointers to deleted DART objects
- Three-phase: clear flags → mark used → sweep unused

### 5. Dual Scene Graph Pattern

- Separate groups for shadowed/non-shadowed objects
- Performance optimization
- Dynamic object migration between groups

---

## Common Workflows

### Adding a World to the Viewer

```cpp
// Create viewer
dart::gui::Viewer viewer;

// Create world node
auto worldNode = new dart::gui::WorldNode(myWorld);

// Configure shadows (optional)
auto shadowTechnique = dart::gui::WorldNode::createDefaultShadowTechnique(&viewer);
worldNode->setShadowTechnique(shadowTechnique);

// Add to viewer
viewer.addWorldNode(worldNode);

// Start simulation
viewer.simulate(true);

// Run viewer
viewer.run();
```

### Implementing Custom Mouse Interaction

```cpp
class MyInteractionHandler : public dart::gui::MouseEventHandler {
public:
    void update() override {
        // Check for left mouse button push
        if (mEventHandler->getButtonEvent(dart::gui::LEFT_MOUSE)
            == dart::gui::BUTTON_PUSH) {

            // Get picked objects
            const auto& picks = mEventHandler->getButtonPicks(
                dart::gui::LEFT_MOUSE,
                dart::gui::BUTTON_PUSH);

            // Handle picks
            for (const auto& pick : picks) {
                // Do something with pick.frame, pick.shape, pick.position
            }
        }
    }
};

// Register handler
auto handler = new MyInteractionHandler();
viewer.getDefaultEventHandler()->addMouseEventHandler(handler);
```

### Creating Custom Shape Rendering

```cpp
class CustomShapeNode : public dart::gui::render::ShapeNode,
                        public ::osg::Geode {
public:
    CustomShapeNode(std::shared_ptr<MyCustomShape> shape,
                    dart::gui::ShapeFrameNode* parent)
        : ShapeNode(shape, parent, this),
          mCustomShape(shape) {
        refresh();
    }

    void refresh() override {
        // Create OSG geometry from custom shape data
        auto geometry = new ::osg::Geometry();
        // ... setup vertices, normals, colors, etc.

        // Apply visual aspect (color, material, etc.)
        auto visual = getVisualAspect();
        // ... apply properties

        // Update geode
        removeDrawables(0, getNumDrawables());
        addDrawable(geometry);
    }

private:
    std::shared_ptr<MyCustomShape> mCustomShape;
};

// Register in ShapeFrameNode::createShapeNode()
if (MyCustomShape::getStaticType() == shapeType) {
    std::shared_ptr<MyCustomShape> cs =
        std::dynamic_pointer_cast<MyCustomShape>(shape);
    if (cs)
        mRenderShapeNode = new CustomShapeNode(cs, this);
}
```

---

## Debugging Tips

### Visualizing the Scene Graph

OSG provides built-in tools for scene graph inspection:

```cpp
// Print scene graph structure
osgDB::writeNodeFile(*viewer.getSceneData(), "scene_graph.osg");
```

### Checking Utilization

Add debug output to track node lifecycle:

```cpp
void WorldNode::clearUnusedNodes() {
    for (auto& node_pair : mFrameToNode) {
        if (!node_pair.second->wasUtilized()) {
            std::cout << "Removing unused node: "
                      << node_pair.first->getName() << std::endl;
        }
    }
    // ... cleanup code
}
```

### Monitoring RTF Performance

```cpp
auto rtWorldNode = dynamic_cast<RealTimeWorldNode*>(worldNode);
if (rtWorldNode) {
    std::cout << "Last RTF: " << rtWorldNode->getLastRealTimeFactor() << std::endl;
    std::cout << "Min RTF: " << rtWorldNode->getLowestRealTimeFactor() << std::endl;
    std::cout << "Max RTF: " << rtWorldNode->getHighestRealTimeFactor() << std::endl;
}
```

### Pick Debugging

```cpp
void MyHandler::update() override {
    const auto& picks = mEventHandler->getMovePicks();
    for (const auto& pick : picks) {
        std::cout << "Hovering over: " << pick.frame->getName()
                  << " at " << pick.position.transpose() << std::endl;
    }
}
```

---

## Limitations and Known Issues

### ImGui + Depth Mode Incompatibility

- Depth rendering mode doesn't work with ImGui widgets
- Use RGBA mode when ImGui UI is present
- Documented in `CameraMode::DEPTH` enum

### ImGui Context Isolation

- Each ImGui handler should keep its own ImGui context.
- Sharing a global context across multiple viewers can overwrite IO/draw data and lead to missing or mis-sized UI.

### Shadow Performance

- Shadow rendering can be computationally expensive
- Use shadow suppression for objects that don't need shadows
- Consider disabling shadows on less important objects

### Real-Time Constraints

- `RealTimeWorldNode` cannot guarantee real-time if simulation is too expensive
- Monitor RTF to detect performance issues
- Consider reducing simulation complexity if RTF < target

### OSG Namespace Conflict

- DART's `dart::gui` namespace conflicts with root `::osg`
- Always use `::osg::` for OSG types (note the leading `::`)
- Custom META macros to work around namespace issues

---

## Conclusion

The DART OSG GUI integration layer provides a robust and extensible framework for visualizing physics simulations. Key strengths include:

1. **Clean Separation**: Clear boundary between DART simulation and OSG rendering
2. **Automatic Management**: Utilization tracking prevents memory leaks
3. **Extensibility**: Well-defined extension points for custom shapes and interactions
4. **Real-Time Support**: Adaptive simulation for smooth playback
5. **Rich Features**: Shadows, picking, screen capture, drag-and-drop

The architecture follows good software engineering practices with clear responsibilities, minimal coupling, and strong cohesion within each component. The use of OSG's scene graph provides excellent performance through automatic culling and efficient rendering.
