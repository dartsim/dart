# DART GUI - Developer Onboarding Guide

## 1. Overview

**DART** (Dynamic Animation and Robotics Toolkit) is a comprehensive C++ physics engine for robotics simulation, kinematics, dynamics, and control. The **dart_gui** component provides 3D visualization and interactive manipulation capabilities built on OpenSceneGraph (OSG) and Dear ImGui.

### Documentation Structure

This onboarding guide is organized into several focused documents:

- **[README.md](README.md)** (this file) - Architecture overview and common workflows
- **[building.md](building.md)** - Step-by-step build instructions for all platforms
- **[contributing.md](contributing.md)** - Comprehensive contribution workflow and guidelines
- **[code-style.md](code-style.md)** - Code style conventions for C++, Python, and CMake
- **[architecture.md](architecture.md)** - Deep dive into DART's simulation core
- **[dynamics.md](dynamics.md)** - Articulated body system and kinematics
- **[constraints.md](constraints.md)** - Constraint resolution and collision response
- **[gui-rendering.md](gui-rendering.md)** - OpenSceneGraph integration details
- **[python-bindings.md](python-bindings.md)** - pybind11 bindings architecture
- **[build-system.md](build-system.md)** - CMake internals and dependency analysis

### Purpose and Problem Solved

DART addresses the need for:
- **High-fidelity physics simulation** for articulated rigid and soft body systems
- **Efficient algorithms** - O(n) complexity for forward/inverse dynamics using Featherstone algorithms
- **Interactive 3D visualization** for robotics research and development
- **Python integration** for rapid prototyping and machine learning workflows
- **Research-grade tooling** maintained by the robotics community and Meta employee Jeongseok Lee

### Key Features

- **Advanced Dynamics**: Articulated body simulation with 10+ joint types, contact resolution, constraint solving
- **Multiple Collision Backends**: FCL (default), Bullet, DART native, ODE
- **3D Visualization**: OpenSceneGraph-based rendering with shadows, materials, and real-time updates
- **Interactive Manipulation**: Drag-and-drop, inverse kinematics, interactive frames with visual handles
- **ImGui Integration**: Modern immediate-mode GUI for controls, debugging, and custom widgets
- **Python Bindings**: Complete API coverage via pybind11 with NumPy integration
- **File Format Support**: URDF, SDF, SKEL, MJCF for robot model loading
- **Cross-Platform**: Linux, macOS (Intel/ARM), Windows

### Technologies Used

| Category | Technology | Purpose |
|----------|-----------|---------|
| **Core Language** | C++17 | Main implementation |
| **Build System** | CMake 3.22.1+ | Cross-platform builds |
| **Python Bindings** | pybind11 2.13.6 | Python API |
| **Linear Algebra** | Eigen 3.4.0+ | Math operations |
| **Collision Detection** | FCL 0.7.0+ | Primary collision backend |
| **3D Rendering** | OpenSceneGraph 3.0.0+ | Visualization |
| **GUI Framework** | Dear ImGui 1.91.9 | Immediate-mode UI |
| **Model Loading** | assimp 5.4.3+ | 3D asset import |
| **Environment** | pixi/conda-forge | Dependency management |
| **Graphics API** | OpenGL 2+ | Rendering backend |

---

## 2. High-Level Architecture Diagram

```mermaid
graph TB
    subgraph "Application Layer"
        APP[Application Code<br/>C++ or Python]
    end

    subgraph "GUI Layer"
        VIEWER[Viewer/ImGuiViewer]
        IMGUI[ImGui Integration]
        WORLDNODE[WorldNode/RealTimeWorldNode]
        DND[DragAndDrop System]
        INTERACTIVE[InteractiveFrame]
    end

    subgraph "Rendering Layer"
        OSG[OpenSceneGraph]
        SHAPENODE[ShapeFrameNode]
        RENDERNODES[Shape Render Nodes<br/>Box, Sphere, Mesh, etc.]
        SHADOWS[Shadow Rendering]
    end

    subgraph "Simulation Core"
        WORLD[World]
        SKELETON[Skeleton]
        COLLISION[Collision Detection]
        CONSTRAINT[Constraint Solver]
        INTEGRATION[Integration]
    end

    subgraph "Dynamics Layer"
        BODYNODE[BodyNode]
        JOINT[Joint System]
        SHAPES[Shape System]
        IK[Inverse Kinematics]
    end

    subgraph "Foundation"
        MATH[Math Utilities]
        COMMON[Common/Aspect]
        LCP[LCP Solver]
    end

    APP --> VIEWER
    APP --> WORLD

    VIEWER --> IMGUI
    VIEWER --> WORLDNODE
    VIEWER --> DND
    VIEWER --> INTERACTIVE

    IMGUI --> OSG
    WORLDNODE --> OSG
    WORLDNODE --> WORLD

    OSG --> SHAPENODE
    OSG --> SHADOWS
    SHAPENODE --> RENDERNODES

    DND --> BODYNODE
    DND --> IK
    INTERACTIVE --> SHAPES

    WORLD --> SKELETON
    WORLD --> COLLISION
    WORLD --> CONSTRAINT
    WORLD --> INTEGRATION

    SKELETON --> BODYNODE
    BODYNODE --> JOINT
    BODYNODE --> SHAPES
    BODYNODE --> IK

    COLLISION --> SHAPES
    CONSTRAINT --> LCP

    JOINT --> MATH
    SHAPES --> MATH
    IK --> MATH
    COLLISION --> COMMON
    SKELETON --> COMMON

    click VIEWER "dart/gui/osg/Viewer.hpp" "Open Viewer"
    click IMGUI "dart/gui/osg/ImGuiViewer.hpp" "Open ImGuiViewer"
    click WORLDNODE "dart/gui/osg/WorldNode.hpp" "Open WorldNode"
    click DND "dart/gui/osg/DragAndDrop.hpp" "Open DragAndDrop"
    click INTERACTIVE "dart/gui/osg/InteractiveFrame.hpp" "Open InteractiveFrame"
    click WORLD "dart/simulation/World.hpp" "Open World"
    click SKELETON "dart/dynamics/Skeleton.hpp" "Open Skeleton"
    click BODYNODE "dart/dynamics/BodyNode.hpp" "Open BodyNode"
    click JOINT "dart/dynamics/Joint.hpp" "Open Joint"
    click COLLISION "dart/collision/CollisionDetector.hpp" "Open CollisionDetector"
    click CONSTRAINT "dart/constraint/ConstraintSolver.hpp" "Open ConstraintSolver"
```

### Component Explanations

**Application Layer:**
- Entry point for user code (examples, tutorials, custom applications)
- Can be written in C++ or Python (via dartpy bindings)

**GUI Layer:**
- **Viewer**: Main window manager for 3D visualization with camera control and event handling
- **ImGuiViewer**: Extended viewer with Dear ImGui widget support for interactive UI
- **WorldNode/RealTimeWorldNode**: Bridge between DART World and OSG scene graph, handles real-time simulation
- **DragAndDrop**: Interactive manipulation system with constraint support
- **InteractiveFrame**: Visual 3D manipulator with translation/rotation handles

**Rendering Layer:**
- **OpenSceneGraph (OSG)**: Scene graph management and OpenGL rendering
- **ShapeFrameNode**: Converts DART frames to OSG nodes
- **Shape Render Nodes**: Specialized renderers for 16+ shape types (box, sphere, mesh, etc.)
- **Shadow Rendering**: Configurable shadow techniques

**Simulation Core:**
- **World**: Top-level simulation container with time-stepping
- **Skeleton**: Articulated body system (robot/character)
- **Collision Detection**: Multi-backend support (FCL, Bullet, ODE)
- **Constraint Solver**: LCP-based constraint resolution
- **Integration**: Time-stepping schemes (Euler, Semi-Implicit Euler, RK4)

**Dynamics Layer:**
- **BodyNode**: Individual rigid body with mass, inertia, shapes
- **Joint System**: 10+ joint types (revolute, prismatic, free, ball, etc.)
- **Shape System**: Collision/visual geometry (box, sphere, mesh, etc.)
- **Inverse Kinematics**: IK solvers for end-effector control

**Foundation:**
- **Math Utilities**: Eigen-based linear algebra, transformations, geometry
- **Common/Aspect**: Design patterns (Factory, Observer, Aspect-oriented)
- **LCP Solver**: Lemke's algorithm for constraint solving

---

## 3. Component Breakdown

### Component: Viewer (GUI Entry Point)

**File**: [`Viewer.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.hpp) | [`Viewer.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp)

**Purpose**: Main 3D visualization window that integrates OpenSceneGraph rendering with DART simulation. Manages camera, lighting, shadows, event handling, and world node registration.

**Key Elements**:
- [`Viewer::Viewer()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L86) - Constructor that sets up OSG viewer with default camera and lighting
- [`Viewer::addWorldNode()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L122) - Registers a WorldNode for rendering
- [`Viewer::enableDragAndDrop()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L189) - Activates interactive manipulation for frames/bodies
- [`Viewer::run()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L462) - Main rendering loop
- [`Viewer::setupDefaultLights()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L292) - Configures scene lighting
- [`Viewer::captureScreen()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L373) - Screenshot functionality

**Depends On**:
- **Internal**: OpenSceneGraph scene graph, camera manipulators, event handlers
- **External**: OSG (OpenSceneGraph library), OpenGL

---

### Component: ImGuiViewer (Enhanced Viewer)

**File**: [`ImGuiViewer.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiViewer.hpp) | [`ImGuiViewer.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiViewer.cpp)

**Purpose**: Extended viewer with Dear ImGui integration for modern UI widgets and controls. Provides immediate-mode GUI capabilities for debugging, controls, and custom interfaces.

**Key Elements**:
- [`ImGuiViewer::ImGuiViewer()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiViewer.cpp#L46) - Initializes ImGui handler and default widgets
- [`ImGuiHandler`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiHandler.hpp) - Bridges OSG events to ImGui input system
- [`ImGuiWidget`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiWidget.hpp) - Base class for custom widgets
- [`AboutWidget`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiViewer.cpp#L49) - Default about dialog

**Depends On**:
- **Internal**: Viewer base class, ImGuiHandler, ImGuiWidget system
- **External**: Dear ImGui library, OpenGL2 backend

---

### Component: WorldNode (Simulation-Rendering Bridge)

**File**: [`WorldNode.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.hpp) | [`WorldNode.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.cpp)

**Purpose**: Encapsulates a DART World for OSG rendering. Manages skeleton visualization, shape nodes, shadow groups, and synchronization between simulation and rendering state.

**Key Elements**:
- [`WorldNode::WorldNode()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.cpp#L59) - Creates OSG node wrapping DART World
- [`WorldNode::refresh()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.cpp#L138) - Updates visual state from simulation
- [`WorldNode::customPreRefresh()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.hpp#L81) - Hook for custom update logic
- [`WorldNode::setShadowTechnique()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/WorldNode.cpp#L218) - Configures shadow rendering
- Dual scene graph: Normal group and shadow group for optimized shadow rendering

**Depends On**:
- **Internal**: DART World, Skeleton, ShapeFrameNode, shadow utilities
- **External**: OSG scene graph nodes

---

### Component: RealTimeWorldNode (Simulation Loop)

**File**: [`RealTimeWorldNode.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.hpp) | [`RealTimeWorldNode.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.cpp)

**Purpose**: Real-time simulation with adaptive time stepping. Maintains target real-time factor (RTF) and provides hooks for custom logic before/after each simulation step.

**Key Elements**:
- [`RealTimeWorldNode::refresh()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.cpp#L103) - Advances simulation and updates visuals
- [`RealTimeWorldNode::customPreStep()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.hpp#L69) - Pre-step hook for control
- [`RealTimeWorldNode::customPostStep()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.hpp#L74) - Post-step hook for logging
- Adaptive stepping: Multiple sub-steps per frame to maintain RTF

**Depends On**:
- **Internal**: WorldNode base class, DART World time-stepping
- **External**: System clock for timing

---

### Component: ShapeFrameNode (Frame Visualization)

**File**: [`ShapeFrameNode.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ShapeFrameNode.hpp) | [`ShapeFrameNode.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ShapeFrameNode.cpp)

**Purpose**: Bridges DART frames to OSG scene graph. Manages transformation updates, shape rendering, visual properties (color, transparency), and lifecycle.

**Key Elements**:
- [`ShapeFrameNode::ShapeFrameNode()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ShapeFrameNode.cpp#L56) - Creates OSG node for a DART frame
- [`ShapeFrameNode::refresh()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ShapeFrameNode.cpp#L109) - Updates transformations and visual properties
- [`ShapeFrameNode::createShapeNode()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ShapeFrameNode.cpp#L190) - Factory for shape-specific renderers
- Utilization tracking for automatic garbage collection

**Depends On**:
- **Internal**: DART Frame, Shape, VisualAspect, shape render nodes
- **External**: OSG transformation nodes

---

### Component: Shape Render Nodes

**Directory**: [`/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/)

**Purpose**: Specialized renderers for 16+ DART shape types. Each renderer converts DART shape geometry to OSG drawable geometry with proper materials and textures.

**Key Shape Nodes**:
- [`BoxShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/BoxShapeNode.hpp) - Box primitives
- [`SphereShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/SphereShapeNode.hpp) - Sphere primitives
- [`CylinderShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/CylinderShapeNode.hpp) - Cylinder primitives
- [`CapsuleShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/CapsuleShapeNode.hpp) - Capsule primitives
- [`MeshShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/MeshShapeNode.hpp) - Arbitrary mesh geometry
- [`SoftMeshShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/SoftMeshShapeNode.hpp) - Deformable soft bodies
- [`PointCloudShapeNode`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/render/PointCloudShapeNode.hpp) - Point cloud visualization

**Depends On**:
- **Internal**: DART Shape classes, VisualAspect properties
- **External**: OSG geometry, materials, textures

---

### Component: DragAndDrop System

**File**: [`DragAndDrop.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp) | [`DragAndDrop.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.cpp)

**Purpose**: Comprehensive drag-and-drop framework for interactive manipulation with constraint support, rotation modes, and specialized handlers for different entity types.

**Key Elements**:
- [`DragAndDrop`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp#L61) - Abstract base class
- [`SimpleFrameDnD`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp#L178) - Drag SimpleFrame objects
- [`InteractiveFrameDnD`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp#L236) - Drag interactive frame tools
- [`BodyNodeDnD`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp#L266) - Drag robot bodies with IK
- Constraint types: `UNCONSTRAINED`, `LINE_CONSTRAINT`, `PLANE_CONSTRAINT`
- Rotation modes: `HOLD_MODKEY`, `ALWAYS_ON`, `ALWAYS_OFF`

**Depends On**:
- **Internal**: DART frames, BodyNode, IK module, picking system
- **External**: OSG event adapters for mouse input

---

### Component: InteractiveFrame

**File**: [`InteractiveFrame.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/InteractiveFrame.hpp) | [`InteractiveFrame.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/InteractiveFrame.cpp)

**Purpose**: 3D manipulator widget with visual handles for translation and rotation. Creates arrows, rings, and planes for intuitive 3D object manipulation.

**Key Elements**:
- [`InteractiveFrame`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/InteractiveFrame.hpp#L109) - Composite frame with 9 manipulation tools
- [`InteractiveTool`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/InteractiveFrame.hpp#L49) - Individual tool (arrow, ring, plane)
- Tool types: `LINEAR` (translation arrows), `ANGULAR` (rotation rings), `PLANAR` (2D planes)
- Color-coded axes: X=red, Y=green, Z=blue
- [`resizeStandardVisuals()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/InteractiveFrame.cpp#L570) - Customize tool size/thickness

**Depends On**:
- **Internal**: DART SimpleFrame, Shape system
- **External**: None (pure DART component)

---

### Component: World (Simulation Core)

**File**: [`World.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.hpp) | [`World.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp)

**Purpose**: Top-level simulation container that manages skeletons, simple frames, time-stepping, and coordinates collision detection and constraint solving.

**Key Elements**:
- [`World::step()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L356) - Main simulation loop
- [`World::addSkeleton()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L196) - Register articulated bodies
- [`World::addSimpleFrame()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L241) - Register non-simulated frames
- [`World::setGravity()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L489) - Configure gravity vector
- [`World::setTimeStep()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L505) - Set integration time step
- Recording support for playback and analysis

**Depends On**:
- **Internal**: Skeleton, SimpleFrame, CollisionDetector, ConstraintSolver, Integrator
- **External**: Eigen for math operations

---

### Component: Skeleton (Articulated Body System)

**File**: [`Skeleton.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.hpp) | [`Skeleton.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp)

**Purpose**: Represents articulated rigid body systems (robots, characters) as a tree of BodyNodes connected by Joints. Manages kinematics, dynamics, and state.

**Key Elements**:
- [`Skeleton::create()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp#L148) - Factory for creating skeletons
- [`Skeleton::createJointAndBodyNodePair<>()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp#L521) - Add body to tree
- [`Skeleton::getMassMatrix()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp#L1567) - Compute mass matrix (O(n²))
- [`Skeleton::getCoriolisAndGravityForces()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp#L1624) - Compute C(q,q̇) + g(q) (O(n))
- Configuration space: generalized positions (q) and velocities (q̇)
- Support for kinematic trees and closed-loop structures

**Depends On**:
- **Internal**: BodyNode, Joint, IK module, algorithms (ABA, RNEA, CRBA)
- **External**: Eigen for linear algebra

---

### Component: BodyNode (Rigid Body)

**File**: [`BodyNode.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.hpp) | [`BodyNode.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp)

**Purpose**: Individual rigid body in an articulated system. Manages mass properties, inertia, shapes (collision/visual), and local transformations.

**Key Elements**:
- [`BodyNode::createShapeNodeWith<>()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp#L357) - Add shape with aspects
- [`BodyNode::setMass()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp#L545) - Configure mass
- [`BodyNode::setInertia()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp#L580) - Configure inertia tensor
- [`BodyNode::getWorldTransform()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp#L755) - Get global pose
- [`BodyNode::getJacobian()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.cpp#L1143) - Compute Jacobian

**Depends On**:
- **Internal**: Joint (parent), Shape, Inertia, Aspect system
- **External**: Eigen for transformations

---

### Component: Joint System

**File**: [`Joint.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Joint.hpp) and specialized joint files

**Purpose**: Connects BodyNodes and defines their relative motion constraints. Provides 10+ joint types for various kinematic configurations.

**Key Joint Types**:
- [`RevoluteJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/RevoluteJoint.hpp) - 1-DOF rotation (1D)
- [`PrismaticJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/PrismaticJoint.hpp) - 1-DOF translation (1D)
- [`FreeJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/FreeJoint.hpp) - 6-DOF unconstrained (6D)
- [`BallJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BallJoint.hpp) - 3-DOF rotation (3D)
- [`WeldJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/WeldJoint.hpp) - 0-DOF fixed (0D)
- [`UniversalJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/UniversalJoint.hpp) - 2-DOF rotation (2D)
- [`EulerJoint`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/EulerJoint.hpp) - 3-DOF Euler angles (3D)

**Key Features**:
- Position/velocity limits
- Damping and spring forces
- Coulomb friction
- Servo control

**Depends On**:
- **Internal**: BodyNode, math utilities
- **External**: Eigen for transformations

---

### Component: Collision Detection

**File**: [`CollisionDetector.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/CollisionDetector.hpp) | [`CollisionGroup.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/CollisionGroup.hpp)

**Purpose**: Pluggable collision detection system supporting multiple backends. Detects collisions between shapes and generates contact points.

**Key Backends**:
- [`FCLCollisionDetector`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/fcl/FCLCollisionDetector.hpp) - Default, uses FCL library
- [`BulletCollisionDetector`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/bullet/BulletCollisionDetector.hpp) - Uses Bullet physics
- [`DARTCollisionDetector`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/dart/DARTCollisionDetector.hpp) - Native implementation
- [`OdeCollisionDetector`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/ode/OdeCollisionDetector.hpp) - Uses ODE library

**Key Elements**:
- [`CollisionDetector::detectCollision()`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/CollisionDetector.cpp#L84) - Run collision detection
- [`CollisionGroup`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/CollisionGroup.hpp) - Groups shapes for broad-phase optimization
- [`Contact`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/Contact.hpp) - Contact point data structure

**Depends On**:
- **Internal**: Shape, CollisionObject
- **External**: FCL, Bullet, or ODE depending on backend

---

### Component: Constraint Solver

**File**: [`ConstraintSolver.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintSolver.hpp) | [`ConstraintSolver.cpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintSolver.cpp)

**Purpose**: Resolves constraints (contacts, joint limits, motors) using LCP-based formulation. Computes constraint impulses to satisfy constraint equations.

**Key Elements**:
- [`ConstraintSolver::solve()`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintSolver.cpp#L159) - Main constraint solving loop
- [`ConstraintSolver::addConstraint()`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintSolver.cpp#L86) - Register constraint
- Constraint types: Contact, JointLimit, Motor, Servo, Mimic, custom
- LCP solvers: Dantzig (primary), PGS (fallback)
- Skeleton grouping for independent constraint solving

**Depends On**:
- **Internal**: Constraint classes, BoxedLcpSolver, Skeleton
- **External**: Eigen for matrix operations

---

### Component: Python Bindings (dartpy)

**Directory**: [`/home/jeongseok/dev/dartsim/dart_gui/python/`](/home/jeongseok/dev/dartsim/dart_gui/python/)

**Purpose**: Complete Python API for DART using pybind11. Enables rapid prototyping, machine learning integration, and scripting.

**Key Modules**:
- `dartpy.math` - Mathematical utilities and Eigen↔NumPy conversion
- `dartpy.dynamics` - Skeletons, BodyNodes, Joints, IK
- `dartpy.collision` - Collision detection backends
- `dartpy.constraint` - Constraint solving
- `dartpy.simulation` - World simulation
- `dartpy.gui.osg` - 3D visualization with OSG and ImGui
- `dartpy.utils` - File parsers (URDF, SDF, SKEL, MJCF)

**Key Files**:
- [`pyproject.toml`](/home/jeongseok/dev/dartsim/dart_gui/pyproject.toml) - Python package configuration
- [`setup.py`](/home/jeongseok/dev/dartsim/dart_gui/setup.py) - Build script using pybind11
- [`python/dartpy/`](/home/jeongseok/dev/dartsim/dart_gui/python/dartpy/) - Python bindings source
- Type stubs (`.pyi` files) for IDE support

**Depends On**:
- **Internal**: All DART C++ modules
- **External**: pybind11, NumPy

---

## 4. Data Flow & Call Flow Examples

### Example Flow 1: Creating and Visualizing a Simple Simulation

**Description**: User creates a falling box simulation, sets up 3D visualization with OSG, and runs the interactive viewer loop.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant User as User Code
    participant Skeleton as Skeleton
    participant World as World
    participant WorldNode as WorldNode
    participant Viewer as Viewer
    participant OSG as OpenSceneGraph

    User->>Skeleton: create()
    User->>Skeleton: createJointAndBodyNodePair<FreeJoint>()
    User->>Skeleton: createShapeNodeWith<>()
    User->>World: create()
    User->>World: addSkeleton(skeleton)
    User->>WorldNode: new RealTimeWorldNode(world)
    User->>Viewer: create Viewer
    User->>Viewer: addWorldNode(node)
    User->>Viewer: setUpViewInWindow()
    User->>Viewer: run()

    loop Rendering Loop
        Viewer->>WorldNode: refresh()
        WorldNode->>World: step()
        World->>Skeleton: computeForwardKinematics()
        World->>Skeleton: computeForwardDynamics()
        World->>World: integrate()
        WorldNode->>OSG: update scene graph
        OSG->>Viewer: render frame
    end
</mermaid>

**Key Files**:
- Example: [`/home/jeongseok/dev/dartsim/dart_gui/examples/hello_world/main.cpp`](/home/jeongseok/dev/dartsim/dart_gui/examples/hello_world/main.cpp)
- [`World::step()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L356)
- [`RealTimeWorldNode::refresh()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/RealTimeWorldNode.cpp#L103)
- [`Viewer::run()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L462)

---

### Example Flow 2: Interactive Drag-and-Drop with IK

**Description**: User enables drag-and-drop for a robot body node, clicks and drags it, triggering inverse kinematics to reposition the robot while maintaining constraints.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant User as User
    participant Viewer as Viewer
    participant DnD as BodyNodeDnD
    participant IK as IK Module
    participant Skeleton as Skeleton
    participant WorldNode as WorldNode

    User->>Viewer: enableDragAndDrop(bodyNode)
    Viewer->>DnD: create BodyNodeDnD handler

    User->>Viewer: Mouse Click on BodyNode
    Viewer->>DnD: handlePick(pickInfo)
    DnD->>DnD: store picked body + offset

    User->>Viewer: Mouse Drag
    Viewer->>DnD: handleMove(mouse position)
    DnD->>IK: setTarget(new position)
    DnD->>IK: solve()
    IK->>Skeleton: compute joint angles
    Skeleton->>Skeleton: setPositions(q_new)

    loop Each Frame
        WorldNode->>Skeleton: computeForwardKinematics()
        WorldNode->>WorldNode: refresh visuals
    end

    User->>Viewer: Mouse Release
    Viewer->>DnD: handleRelease()
    DnD->>DnD: cleanup drag state
</mermaid>

**Key Files**:
- Example: [`/home/jeongseok/dev/dartsim/dart_gui/examples/atlas_puppet/main.cpp`](/home/jeongseok/dev/dartsim/dart_gui/examples/atlas_puppet/main.cpp)
- [`BodyNodeDnD`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/DragAndDrop.hpp#L266)
- [`IK::solve()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/IK.cpp#L142)
- [`Viewer::enableDragAndDrop()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/Viewer.cpp#L189)

---

### Example Flow 3: World Simulation Step (Physics Pipeline)

**Description**: Internal process of a single World::step() call, showing collision detection, constraint solving, and integration.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant World as World
    participant Skeleton as Skeleton
    participant CollisionDet as CollisionDetector
    participant ConstraintS as ConstraintSolver
    participant LCP as LcpSolver
    participant Integrator as Integrator

    World->>Skeleton: computeForwardKinematics()
    Note over Skeleton: Update body transforms

    World->>Skeleton: computeForwardDynamics()
    Note over Skeleton: ABA: q̈ = M⁻¹(τ - C - g)

    World->>CollisionDet: detectCollision()
    CollisionDet-->>World: contact points

    World->>ConstraintS: addConstraint(contacts)
    World->>ConstraintS: addConstraint(jointLimits)

    World->>ConstraintS: solve()
    ConstraintS->>ConstraintS: buildConstraintMatrix()
    ConstraintS->>LCP: solve(A, b)
    LCP-->>ConstraintS: impulses
    ConstraintS->>Skeleton: applyConstraintImpulses()

    World->>Integrator: integrate(q, q̇, q̈, dt)
    Note over Integrator: Semi-Implicit Euler:<br/>q̇ₙ₊₁ = q̇ₙ + q̈ₙ·dt<br/>qₙ₊₁ = qₙ + q̇ₙ₊₁·dt
    Integrator-->>World: new state (q, q̇)

    World->>World: time += dt
</mermaid>

**Key Files**:
- [`World::step()`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.cpp#L356)
- [`Skeleton::computeForwardDynamics()`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.cpp#L2154)
- [`ConstraintSolver::solve()`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintSolver.cpp#L159)
- [`SemiImplicitEulerIntegrator`](/home/jeongseok/dev/dartsim/dart_gui/dart/integration/SemiImplicitEulerIntegrator.cpp)

---

### Example Flow 4: ImGui Widget Rendering

**Description**: How ImGui widgets are integrated into the OSG rendering loop using camera callbacks.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant OSG as OSG Renderer
    participant Camera as OSG Camera
    participant ImGuiH as ImGuiHandler
    participant Widget as ImGuiWidget
    participant ImGui as Dear ImGui

    OSG->>Camera: pre-draw callback
    Camera->>ImGuiH: newFrame()
    ImGuiH->>ImGui: ImGui_ImplOpenGL2_NewFrame()
    ImGuiH->>ImGui: ImGui::NewFrame()
    Note over ImGui: Start new frame,<br/>process input state

    OSG->>OSG: render 3D scene

    OSG->>Camera: post-draw callback
    Camera->>ImGuiH: render()

    loop For each widget
        ImGuiH->>Widget: widget->render()
        Widget->>ImGui: ImGui::Begin()
        Widget->>ImGui: ImGui::Button(), etc.
        Widget->>ImGui: ImGui::End()
    end

    ImGuiH->>ImGui: ImGui::Render()
    ImGuiH->>ImGui: ImGui_ImplOpenGL2_RenderDrawData()
    Note over ImGui: Draw UI on top of 3D scene

    OSG->>OSG: swap buffers
</mermaid>

**Key Files**:
- [`ImGuiHandler::newFrame()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiHandler.cpp#L135)
- [`ImGuiHandler::render()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiHandler.cpp#L207)
- [`ImGuiWidget::render()`](/home/jeongseok/dev/dartsim/dart_gui/dart/gui/osg/ImGuiWidget.hpp#L49)

---

### Example Flow 5: Loading URDF and Creating Simulation

**Description**: Loading a robot model from URDF file and adding it to simulation world.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant User as User Code
    participant URDFLoader as DartLoader
    participant Skeleton as Skeleton
    participant Parser as URDF Parser
    participant World as World

    User->>URDFLoader: DartLoader()
    User->>URDFLoader: parseSkeleton("robot.urdf")
    URDFLoader->>Parser: parseURDF()
    Parser->>Parser: parse XML
    Parser->>Skeleton: create()

    loop For each link
        Parser->>Skeleton: createJointAndBodyNodePair<>()
        Parser->>Skeleton: addShape()
        Parser->>Skeleton: setMass/Inertia()
    end

    URDFLoader-->>User: skeleton
    User->>World: addSkeleton(skeleton)
    User->>World: step()
</mermaid>

**Key Files**:
- [`DartLoader`](/home/jeongseok/dev/dartsim/dart_gui/dart/utils/urdf/DartLoader.hpp)
- [`DartLoader::parseSkeleton()`](/home/jeongseok/dev/dartsim/dart_gui/dart/utils/urdf/DartLoader.cpp)
- Example: [`/home/jeongseok/dev/dartsim/dart_gui/examples/atlas_puppet/main.cpp`](/home/jeongseok/dev/dartsim/dart_gui/examples/atlas_puppet/main.cpp)

---

### Example Flow 6: Python API Usage

**Description**: Using the Python bindings (dartpy) to create and simulate a robot.

**Sequence Diagram**:

```mermaid
sequenceDiagram
    participant Python as Python Script
    participant dartpy as dartpy Module
    participant DART as DART C++
    participant NumPy as NumPy

    Python->>dartpy: import dartpy
    Python->>dartpy: world = dartpy.simulation.World()
    dartpy->>DART: World::create()

    Python->>dartpy: skel = dartpy.dynamics.Skeleton.create()
    dartpy->>DART: Skeleton::create()

    Python->>dartpy: world.addSkeleton(skel)
    dartpy->>DART: World::addSkeleton()

    loop Simulation
        Python->>dartpy: world.step()
        dartpy->>DART: World::step()
        Python->>dartpy: q = skel.getPositions()
        dartpy->>DART: Skeleton::getPositions()
        DART->>NumPy: convert Eigen to ndarray
        NumPy-->>Python: positions array
    end
</mermaid>

**Key Files**:
- Example: [`/home/jeongseok/dev/dartsim/dart_gui/examples/hello_world/main.py`](/home/jeongseok/dev/dartsim/dart_gui/examples/hello_world/main.py)
- [`python/dartpy/`](/home/jeongseok/dev/dartsim/dart_gui/python/dartpy/) - Bindings implementation
- [`setup.py`](/home/jeongseok/dev/dartsim/dart_gui/setup.py) - Build configuration

---

## 5. Data Models (Entities)

### Entity: Skeleton

**File**: [`Skeleton.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Skeleton.hpp)

**Purpose**: Articulated body system representing a robot or character

**Key Fields**:
- `mBodyNodes: std::vector<BodyNode*>` - Tree of rigid bodies
- `mJoints: std::vector<Joint*>` - Joint connections
- `mDofs: std::vector<DegreeOfFreedom*>` - Generalized coordinates
- `q: Eigen::VectorXd` - Generalized positions
- `dq: Eigen::VectorXd` - Generalized velocities
- `ddq: Eigen::VectorXd` - Generalized accelerations

**Relations**:
- Contains: BodyNode tree, Joints, DOFs
- Used by: World, IK module, ConstraintSolver

**Notes**:
- Tree structure with efficient O(n) algorithms (ABA, RNEA)
- Supports closed-loop structures via constraints
- Configuration space uses Lie group representations for joints

---

### Entity: BodyNode

**File**: [`BodyNode.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/BodyNode.hpp)

**Purpose**: Individual rigid body in an articulated system

**Key Fields**:
- `mMass: double` - Body mass
- `mInertia: Inertia` - Inertia tensor and COM offset
- `mParentJoint: Joint*` - Incoming joint from parent
- `mChildBodyNodes: std::vector<BodyNode*>` - Children in tree
- `mShapeNodes: std::vector<ShapeNode*>` - Collision/visual shapes
- `mT: Eigen::Isometry3d` - World transformation

**Relations**:
- Parent: Joint (incoming)
- Children: BodyNode (outgoing)
- Contains: ShapeNode collection
- Part of: Skeleton

**Notes**:
- Mass properties required for dynamics
- Multiple shapes per body supported
- Transformations updated via forward kinematics

---

### Entity: Joint

**File**: [`Joint.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Joint.hpp)

**Purpose**: Kinematic constraint connecting two BodyNodes

**Key Fields**:
- `mNumDofs: size_t` - Degrees of freedom (0-6)
- `mPositions: Eigen::VectorXd` - Joint positions (q)
- `mVelocities: Eigen::VectorXd` - Joint velocities (q̇)
- `mAccelerations: Eigen::VectorXd` - Joint accelerations (q̈)
- `mPositionLowerLimits: Eigen::VectorXd` - Lower position bounds
- `mPositionUpperLimits: Eigen::VectorXd` - Upper position bounds
- `mDampingCoefficients: Eigen::VectorXd` - Damping forces
- `mSpringStiffnesses: Eigen::VectorXd` - Spring forces

**Relations**:
- Connects: Parent BodyNode → Child BodyNode
- Owned by: Skeleton

**Notes**:
- 10+ specialized joint types (Revolute, Prismatic, Free, etc.)
- Supports position/velocity limits, damping, springs
- Provides relative transformation between bodies

---

### Entity: Shape

**File**: [`Shape.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Shape.hpp)

**Purpose**: Geometric representation for collision detection and visualization

**Key Shape Types**:
- `BoxShape` - Rectangular box
- `SphereShape` - Sphere
- `CylinderShape` - Cylinder
- `CapsuleShape` - Capsule (cylinder + hemispheres)
- `MeshShape` - Arbitrary triangle mesh
- `SoftMeshShape` - Deformable mesh
- `PointCloudShape` - Point cloud
- `VoxelGridShape` - Voxel grid

**Key Fields**:
- `mVolume: double` - Shape volume
- `mBoundingBox: BoundingBox` - Axis-aligned bounding box

**Relations**:
- Owned by: ShapeNode
- Used by: CollisionDetector, Renderer

**Notes**:
- Shapes can have multiple aspects (Visual, Collision, Dynamics)
- Inertia computed from shape geometry
- Material properties for rendering

---

### Entity: World

**File**: [`World.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/simulation/World.hpp)

**Purpose**: Top-level simulation container

**Key Fields**:
- `mSkeletons: std::vector<Skeleton*>` - Articulated bodies
- `mSimpleFrames: std::vector<SimpleFrame*>` - Non-simulated frames
- `mTime: double` - Current simulation time
- `mTimeStep: double` - Integration time step (default: 0.001s)
- `mGravity: Eigen::Vector3d` - Gravity vector (default: [0,0,-9.81])
- `mCollisionDetector: CollisionDetector*` - Collision system
- `mConstraintSolver: ConstraintSolver*` - Constraint system
- `mIntegrator: Integrator*` - Time integration scheme

**Relations**:
- Contains: Skeletons, SimpleFrames
- Uses: CollisionDetector, ConstraintSolver, Integrator
- Rendered by: WorldNode

**Notes**:
- Manages simulation time and stepping
- Coordinates collision detection and constraint solving
- Supports recording for playback

---

### Entity: Contact

**File**: [`Contact.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/collision/Contact.hpp)

**Purpose**: Contact point data from collision detection

**Key Fields**:
- `point: Eigen::Vector3d` - Contact point in world coordinates
- `normal: Eigen::Vector3d` - Contact normal (from object A to B)
- `penetrationDepth: double` - Overlap distance
- `collisionObject1: CollisionObject*` - First colliding object
- `collisionObject2: CollisionObject*` - Second colliding object

**Relations**:
- Generated by: CollisionDetector
- Used by: ConstraintSolver (creates contact constraints)

**Notes**:
- Multiple contacts per collision pair
- Normal points from object A to object B
- Penetration depth used for impulse calculation

---

### Entity: Constraint

**File**: [`ConstraintBase.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/constraint/ConstraintBase.hpp)

**Purpose**: Abstract base for constraint types

**Constraint Types**:
- `ContactConstraint` - Contact between bodies
- `JointLimitConstraint` - Joint position/velocity limits
- `ServoMotorConstraint` - Joint servo control
- `MimicMotorConstraint` - Mimic joint behavior
- Custom constraints

**Key Fields**:
- `mDim: size_t` - Constraint dimensionality
- `mActive: bool` - Whether constraint is active

**Relations**:
- Managed by: ConstraintSolver
- Acts on: Skeleton DOFs

**Notes**:
- Formulated as LCP (Linear Complementarity Problem)
- Computes impulses to satisfy constraint equations

---

### Entity: Frame

**File**: [`Frame.hpp`](/home/jeongseok/dev/dartsim/dart_gui/dart/dynamics/Frame.hpp)

**Purpose**: Coordinate frame in the simulation world

**Types**:
- `BodyNode` - Frame attached to rigid body
- `SimpleFrame` - User-defined static or movable frame
- `InteractiveFrame` - Frame with 3D manipulation handles

**Key Fields**:
- `mParentFrame: Frame*` - Parent in frame hierarchy
- `mRelativeTransform: Eigen::Isometry3d` - Transform relative to parent
- `mWorldTransform: Eigen::Isometry3d` - Cached world transform

**Relations**:
- Hierarchy: Parent-child frame tree
- Can contain: ShapeNodes for visualization

**Notes**:
- All entities inherit from Frame
- World coordinate frame is root of hierarchy
- Transformations propagate down the tree

---

## 6. Build and Development Guide

### Prerequisites

**Required**:
- CMake ≥ 3.22.1
- C++17 compiler (GCC, Clang, MSVC)
- Eigen ≥ 3.4.0
- FCL ≥ 0.7.0
- assimp ≥ 5.4.3
- OpenSceneGraph ≥ 3.0.0
- ImGui ≥ 1.91.9 or fetched automatically

**Optional**:
- Python ≥ 3.7 (for dartpy bindings)
- Bullet, ODE (alternative collision backends)
- IPOPT, NLopt (optimization)
- urdfdom (URDF parsing)

### Quick Start with pixi

[`pixi`](/home/jeongseok/dev/dartsim/dart_gui/pixi.toml) provides a reproducible development environment:

```bash
# Install pixi (if not already installed)
curl -fsSL https://pixi.sh/install.sh | bash

# Configure and build
pixi run config
pixi run build

# Run tests
pixi run test

# Run examples
pixi run hello-world
pixi run atlas-puppet

# Build Python bindings
pixi run build-py-dev
pixi run test-py
```

### Manual CMake Build

```bash
# Create build directory
mkdir build && cd build

# Configure
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DDART_BUILD_GUI_OSG=ON \
  -DDART_BUILD_DARTPY=ON

# Build
cmake --build . -j$(nproc)

# Install
sudo cmake --install .
```

### Common CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `DART_BUILD_GUI_OSG` | ON | Build OSG-based GUI |
| `DART_BUILD_DARTPY` | OFF | Build Python bindings |
| `DART_USE_SYSTEM_IMGUI` | OFF | Use system ImGui vs fetched |
| `DART_CODECOV` | OFF | Enable code coverage |
| `DART_BUILD_EXTRAS` | ON | Build examples/tutorials |

### Running Examples

**C++ Examples** ([`/home/jeongseok/dev/dartsim/dart_gui/examples/`](/home/jeongseok/dev/dartsim/dart_gui/examples/)):
```bash
# Using pixi
pixi run hello-world
pixi run drag-and-drop
pixi run atlas-puppet

# Manual
./build/bin/hello_world
./build/bin/drag_and_drop
./build/bin/atlas_puppet
```

**Python Examples**:
```bash
# Using pixi
pixi run hello-world-py
pixi run operational-space-control-py

# Manual
python examples/hello_world/main.py
python examples/operational_space_control/main.py
```

### Development Workflow

1. **Make code changes**
2. **Run formatter**: `pixi run format`
3. **Check formatting**: `pixi run check-format`
4. **Build**: `pixi run build`
5. **Run tests**: `pixi run test`
6. **Build docs**: `pixi run docs-build`

### Project Structure

```
dart_gui/
├── dart/                      # C++ library source
│   ├── collision/            # Collision detection backends
│   ├── common/               # Common utilities and patterns
│   ├── constraint/           # Constraint solver
│   ├── dynamics/             # Kinematics and dynamics
│   ├── external/             # Bundled dependencies (ImGui, etc.)
│   ├── gui/osg/              # OpenSceneGraph visualization
│   ├── integration/          # Time integrators
│   ├── lcpsolver/            # LCP solver
│   ├── math/                 # Mathematical utilities
│   ├── optimizer/            # Optimization algorithms
│   ├── simulation/           # World simulation
│   └── utils/                # File parsers (URDF, SDF, etc.)
├── python/                   # Python bindings (dartpy)
├── examples/                 # C++ and Python examples
├── tutorials/                # Tutorial code
├── tests/                    # Unit tests
├── docs/                     # Documentation
├── cmake/                    # CMake modules
├── CMakeLists.txt            # Root CMake file
├── pixi.toml                 # pixi configuration
└── pyproject.toml            # Python package configuration
```

### Key Entry Points

**C++ API**:
```cpp
#include <dart/dart.hpp>              // Core DART
#include <dart/gui/osg/osg.hpp>       // OSG GUI
#include <dart/utils/urdf/urdf.hpp>   // URDF parsing
```

**Python API**:
```python
import dartpy as dart
from dartpy.gui.osg import Viewer, RealTimeWorldNode
from dartpy.utils import DartLoader
```

---

## 7. Common Workflows and Patterns

### Pattern 1: Creating a Basic Simulation

```cpp
// Create skeletons
auto robot = dynamics::Skeleton::create("robot");
// ... add bodies, joints, shapes

// Create world
auto world = simulation::World::create();
world->addSkeleton(robot);
world->setGravity(Eigen::Vector3d(0, 0, -9.81));
world->setTimeStep(0.001);

// Simulation loop
while (running) {
  world->step();
  // Access state: robot->getPositions(), getVelocities()
}
```

### Pattern 2: Creating Visualization

```cpp
// Create viewer
gui::osg::Viewer viewer;
viewer.setUpViewInWindow(0, 0, 1280, 720);

// Wrap world in real-time node
osg::ref_ptr<gui::osg::RealTimeWorldNode> node
  = new gui::osg::RealTimeWorldNode(world);

// Add to viewer
viewer.addWorldNode(node);

// Configure camera
viewer.getCameraManipulator()->setHomePosition(
  osg::Vec3(5, 5, 3), osg::Vec3(0, 0, 0), osg::Vec3(0, 0, 1));
viewer.setCameraManipulator(viewer.getCameraManipulator());

// Run viewer loop (handles simulation + rendering)
viewer.run();
```

### Pattern 3: Custom Simulation Hooks

```cpp
class MyWorldNode : public gui::osg::RealTimeWorldNode {
public:
  MyWorldNode(const simulation::WorldPtr& world)
    : gui::osg::RealTimeWorldNode(world) {}

  void customPreStep() override {
    // Apply control forces before physics step
    auto robot = getWorld()->getSkeleton("robot");
    robot->setForces(computeControlTorques());
  }

  void customPostStep() override {
    // Log data after physics step
    logState(getWorld()->getTime());
  }
};

// Use custom node
osg::ref_ptr<MyWorldNode> node = new MyWorldNode(world);
viewer.addWorldNode(node);
```

### Pattern 4: Interactive Manipulation

```cpp
// Enable drag-and-drop for a frame
gui::osg::InteractiveFramePtr manipulator
  = new gui::osg::InteractiveFrame(Frame::World(), "tool", tf);
world->addSimpleFrame(manipulator);
viewer.enableDragAndDrop(manipulator.get());

// Enable drag-and-drop for robot bodies with IK
for (auto bodyNode : robot->getBodyNodes()) {
  viewer.enableDragAndDrop(bodyNode);
}
```

### Pattern 5: Loading Robot Models

```cpp
// URDF
dart::utils::DartLoader loader;
auto robot = loader.parseSkeleton("path/to/robot.urdf");
world->addSkeleton(robot);

// SDF
auto robot = dart::utils::SdfParser::readSkeleton("path/to/model.sdf");

// SKEL (DART native format)
auto robot = dart::utils::SkelParser::readSkeleton("path/to/skel.skel");
```

### Pattern 6: Custom ImGui Widgets

```cpp
class MyWidget : public gui::osg::ImGuiWidget {
public:
  void render() override {
    ImGui::Begin("My Widget", &mIsVisible);

    if (ImGui::Button("Reset Simulation")) {
      mWorld->reset();
    }

    ImGui::Text("Time: %.2f", mWorld->getTime());
    ImGui::End();
  }

private:
  simulation::WorldPtr mWorld;
};

// Add to ImGuiViewer
auto widget = std::make_shared<MyWidget>();
viewer.getImGuiHandler()->addWidget(widget, true);
```

---

## 8. Additional Resources

### Documentation
- **Main Website**: https://dartsim.github.io/
- **API Documentation**: Built with Doxygen (run `pixi run api-docs-cpp`)
- **Tutorials**: [`/home/jeongseok/dev/dartsim/dart_gui/tutorials/`](/home/jeongseok/dev/dartsim/dart_gui/tutorials/)
- **Examples**: [`/home/jeongseok/dev/dartsim/dart_gui/examples/`](/home/jeongseok/dev/dartsim/dart_gui/examples/)

### Community
- **GitHub**: https://github.com/dartsim/dart
- **Maintained by**: Jeongseok Lee (Meta employee) and robotics community

### Related Analysis Documents
This repository contains additional detailed analysis documents:
- [`architecture.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/architecture.md) - Core DART architecture deep dive
- [`gui-rendering.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/gui-rendering.md) - OpenSceneGraph integration details
- [`python-bindings.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/python-bindings.md) - Python bindings (dartpy) reference
- [`build-system.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/build-system.md) - Build system and dependencies
- [`dynamics.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/dynamics.md) - Dynamics system exploration
- [`constraints.md`](/home/jeongseok/dev/dartsim/dart_gui/docs/onboarding/constraints.md) - Constraint solver analysis

### Key Design Patterns Used in DART
- **Factory Pattern**: Skeleton::create(), Joint factories
- **Strategy Pattern**: Pluggable collision backends, integrators, solvers
- **Observer Pattern**: Event handlers, callbacks
- **Composite Pattern**: Frame hierarchy, BodyNode tree
- **Aspect Pattern**: Runtime extensibility for entities
- **Template Method Pattern**: WorldNode customization hooks
- **Singleton Pattern**: Global registries
- **RAII Pattern**: Smart pointers, resource management

---

## Summary

**DART GUI** is a mature, research-grade robotics simulation and visualization library with:

✅ **O(n) efficient dynamics** via Featherstone algorithms
✅ **Interactive 3D visualization** with OSG + ImGui
✅ **Multiple collision backends** (FCL, Bullet, ODE)
✅ **Python integration** for ML/research workflows
✅ **Extensive file format support** (URDF, SDF, MJCF, SKEL)
✅ **Cross-platform** with reproducible builds via pixi

The codebase demonstrates excellent software engineering practices with clear layering, design patterns, and extensibility. The GUI components provide rich interactivity for robotics research and development.

---

**Generated**: 2025-10-19
**Project Version**: 7.0.0
**Maintainer**: Jeongseok Lee (Meta)
