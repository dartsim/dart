# DART Build System Analysis

**Project:** DART (Dynamic Animation and Robotics Toolkit)
**Version:** 7.0.0
**Analysis Date:** 2025-10-19
**Root Directory:** `<project_root>`

---

## Table of Contents
1. [Build System Overview](#build-system-overview)
2. [CMake Structure](#cmake-structure)
3. [External Dependencies](#external-dependencies)
4. [Bundled Dependencies](#bundled-dependencies)
5. [Component Architecture](#component-architecture)
6. [Build Targets](#build-targets)
7. [Environment Management](#environment-management)
8. [ROS Integration](#ros-integration)

---

## Build System Overview

### Build Tool Chain
- **CMake Version:** ≥ 3.22.1
- **Build Generator:** Ninja (via pixi) / CMake default
- **C++ Standard:** C++17
- **Package Manager:** pixi (conda-forge based)
- **Python:** Python 3 (for dartpy bindings)

### Build Configurations
- **Release** (default): `-O3 -DNDEBUG`
- **Debug**: `-g -fno-omit-frame-pointer -fno-inline-functions`
- **RelWithDebInfo**: Combines Release + Debug flags
- **Profile**: Debug + profiling (`-pg`)

### Key Build Options
```cmake
DART_BUILD_GUI_OSG          = ON   # Build OpenSceneGraph GUI
DART_BUILD_DARTPY           = OFF  # Build Python bindings
DART_BUILD_PROFILE          = OFF  # Enable profiling support
DART_ENABLE_SIMD            = OFF  # Enable SIMD instructions
DART_USE_SYSTEM_IMGUI       = OFF  # Use system ImGui vs fetch from GitHub
DART_USE_SYSTEM_GOOGLEBENCHMARK = OFF
DART_USE_SYSTEM_GOOGLETEST  = OFF
DART_USE_SYSTEM_TRACY       = OFF
BUILD_SHARED_LIBS           = ON   # Build shared libraries (Linux/macOS)
```

---

## CMake Structure

### Primary CMakeLists Files

#### 1. Root CMakeLists.txt
**File:** `CMakeLists.txt`

**Responsibilities:**
- Project configuration and version extraction from `package.xml`
- Compiler flags setup (GCC, Clang, MSVC)
- Build type configuration
- Code coverage setup (`DART_CODECOV`)
- Documentation generation (Doxygen)
- Code formatting targets (`format`, `check-format`)
- Installation rules
- Component exports and CMake config generation

**Key Features:**
- Extracts version (7.0.0) from `package.xml` using regex
- Sets up custom targets: `tests`, `examples`, `tutorials`, `ALL`
- Generates pkg-config file (`dart.pc`)
- Supports both absolute and relative install paths

#### 2. dart/CMakeLists.txt
**File:** `dart/CMakeLists.txt`

**Responsibilities:**
- Defines core DART library
- Manages subdirectory structure
- Links core dependencies (Eigen3, FCL, assimp, fmt, spdlog)
- Configures SIMD instructions
- Sets active log levels
- Generates `dart/config.hpp`

**Component Hierarchy:**
```
dart/
├── external/       # Bundled dependencies
├── common/         # Common utilities
├── math/           # Math utilities
├── integration/    # Numerical integration
├── lcpsolver/      # LCP solver
├── optimizer/      # Optimization algorithms
├── dynamics/       # Dynamics engine
├── collision/      # Collision detection
├── constraint/     # Constraint handling
├── simulation/     # Simulation framework
├── utils/          # Utility functions
└── gui/            # GUI components
    └── osg/        # OpenSceneGraph GUI
```

#### 3. dart/gui/osg/CMakeLists.txt
**File:** `dart/gui/osg/CMakeLists.txt`

**Responsibilities:**
- Builds `dart-gui-osg` library
- Integrates OpenSceneGraph
- Integrates ImGui (system or fetched)
- Manages render subdirectory
- Generates component headers (`All.hpp`, `osg.hpp`)

**Dependencies:**
- `dart-utils` (dependent target)
- OpenSceneGraph 3.0.0+ (external)
- ImGui 1.80+ (external or fetched)
- OpenGL (for ImGui backend)

---

## External Dependencies

### Required Core Dependencies

#### 1. Eigen3 (Linear Algebra)
- **Version:** ≥ 3.4.0
- **Purpose:** Matrix operations, linear algebra
- **CMake Module:** `cmake/DARTFindEigen3.cmake`
- **Find Package:** `Eigen3` (CONFIG mode)

#### 2. FCL (Flexible Collision Library)
- **Version:** ≥ 0.7.0, < 0.8
- **Purpose:** Collision detection
- **CMake Module:** `cmake/DARTFindfcl.cmake`
- **ROS Dependency:** `libfcl-dev`

#### 3. assimp (Asset Importer)
- **Version:** ≥ 5.4.3, < 6
- **Purpose:** 3D model loading (meshes, skeletons)
- **CMake Module:** `cmake/DARTFindassimp.cmake`
- **Special Checks:** Constructor/destructor availability for `aiScene` and `aiMaterial`

#### 4. fmt (Formatting Library)
- **Version:** ≥ 11.1.4, < 12
- **Purpose:** String formatting
- **CMake Module:** `cmake/DARTFindfmt.cmake`
- **Targets:** `fmt::fmt` or `fmt::fmt-header-only`

#### 5. octomap (Octree-based 3D mapping)
- **Version:** ≥ 1.10.0, < 2
- **Purpose:** VoxelGridShape support
- **CMake Module:** `cmake/DARTFindoctomap.cmake`
- **Optional:** Warning if not found

### Optional Dependencies

#### 6. spdlog (Logging)
- **Version:** ≥ 1.15.3, < 2
- **Purpose:** Fast C++ logging library
- **CMake Module:** `cmake/DARTFindspdlog.cmake`
- **Targets:** `spdlog::spdlog` or `spdlog::spdlog_header_only`
- **Skip Option:** `DART_SKIP_spdlog`

### GUI-Specific Dependencies

#### 7. OpenSceneGraph (OSG)
- **Version:** ≥ 3.0.0, < 4
- **Recommended:** ≥ 3.7.0 (for macOS 10.15+ compatibility)
- **Purpose:** 3D visualization and rendering
- **CMake Module:** `cmake/DARTFindOpenSceneGraph.cmake`
- **Components Required:**
  - `osg`
  - `osgViewer`
  - `osgManipulator`
  - `osgGA`
  - `osgDB`
  - `osgShadow`
  - `osgUtil`
- **Target:** `osg::osg` (INTERFACE IMPORTED)
- **Platform Notes:**
  - macOS: Manual build required on osx-64 and osx-arm64 (git SHA: `2e4ae2ea94595995c1fc56860051410b0c0be605`)
  - Linux: Available via conda-forge
  - Windows: Available via conda-forge

#### 8. ImGui (Immediate Mode GUI)
- **Version:** ≥ 1.91.9, < 2 (system), v1.84.2 (fetched)
- **Purpose:** In-scene GUI widgets and overlays
- **CMake Module:** `cmake/DARTFindimgui.cmake`
- **Fetch Module:** `dart/external/imgui/CMakeLists.txt`
- **Backend:** OpenGL2 (for OSG compatibility)
- **Target:** `imgui::imgui`
- **Modes:**
  - **System Mode** (`DART_USE_SYSTEM_IMGUI=ON`): Links against system package
  - **Fetch Mode** (`DART_USE_SYSTEM_IMGUI=OFF`): Downloads from GitHub
    - Repository: `https://github.com/ocornut/imgui.git`
    - Tag: `v1.84.2`
    - Not installed with DART (local build only)

#### 9. OpenGL
- **Purpose:** Graphics rendering (required by ImGui backend)
- **Target:** `OpenGL::GL`
- **Platform:** All (system-provided)

### Optimizer Dependencies (Optional Components)

#### 10. IPOPT
- **Version:** ≥ 3.14.17, < 4
- **Purpose:** Interior Point Optimizer
- **Component:** `dart-optimizer-ipopt`
- **CMake Module:** `cmake/DARTFindIPOPT.cmake`

#### 11. NLopt
- **Version:** ≥ 2.10.0, < 3
- **Purpose:** Nonlinear optimization library
- **Component:** `dart-optimizer-nlopt`
- **CMake Module:** `cmake/DARTFindNLOPT.cmake`

### Collision Engine Dependencies (Optional Components)

#### 12. Bullet Physics
- **Version:** ≥ 3.25, < 4
- **Purpose:** Alternative collision detection
- **Component:** `dart-collision-bullet`
- **CMake Module:** `cmake/DARTFindBullet.cmake`

### Utility Dependencies

#### 13. tinyxml2
- **Version:** ≥ 11.0.0, < 12
- **Purpose:** XML parsing (for SDF)
- **Component:** `dart-utils`
- **CMake Module:** `cmake/DARTFindtinyxml2.cmake`

#### 14. libsdformat
- **Version:** ≥ 16.0.0, < 17
- **Purpose:** Official SDFormat parser used to canonicalize files (version conversion, `<include>` resolution, URI normalization) before DART walks the DOM.
- **Component:** `dart-utils`
- **CMake Module:** `cmake/DARTFindsdformat.cmake`
- **Notes:** Required for all SDF parsing. DART no longer ships a fallback XML code path, so builds without libsdformat cannot load `.sdf`/`.world` assets.

#### 15. urdfdom
- **Version:** ≥ 4.0.1, < 5
- **Purpose:** URDF parsing
- **Component:** `dart-utils-urdf`
- **CMake Module:** `cmake/DARTFindurdfdom.cmake`
- **ROS Dependency:** `liburdfdom-dev`

### Build/Test Dependencies (Build-time only)

#### 16. Google Test (gtest)
- **Version:** ≥ 1.17.0, < 2
- **Purpose:** Unit testing framework
- **Option:** `DART_USE_SYSTEM_GOOGLETEST`

#### 17. Google Benchmark
- **Version:** ≥ 1.9.3, < 2
- **Purpose:** Performance benchmarking
- **Option:** `DART_USE_SYSTEM_GOOGLEBENCHMARK`

#### 18. Tracy Profiler
- **Version:** ≥ 0.11.1, < 0.12
- **Purpose:** Frame profiling
- **Option:** `DART_USE_SYSTEM_TRACY`
- **Components:**
  - `tracy-profiler-client` (runtime)
  - `tracy-profiler-gui` (build-time)

### Additional Platform Dependencies

#### Linux (linux-64)
- **lcov:** ≥ 1.16, < 2 (coverage reports)

#### macOS
- **Cocoa Framework** (linked with ImGui)
- **Git:** ≥ 2.40.0 (for OSG build)

---

## Bundled Dependencies

### Overview
DART includes several dependencies as part of the source tree under `dart/external/`.

### 1. ODE LCP Solver
**Location:** `dart/external/odelcpsolver/`

**Purpose:** Linear Complementarity Problem (LCP) solver from Open Dynamics Engine

**Target:** `dart-external-odelcpsolver`

**Source Files:**
- `lcp.cpp/h` - LCP solver implementation
- `matrix.cpp/h` - Matrix operations
- `fastldlt.cpp` - Fast LDLT decomposition
- `fastlsolve.cpp` - Fast linear solver
- `fastltsolve.cpp` - Fast linear transpose solver
- `fastdot.cpp` - Fast dot product
- `error.cpp/h` - Error handling
- `misc.cpp/h` - Miscellaneous utilities

**Linked By:** Core `dart` library

### 2. ImGui (Conditionally Bundled)
**Location:** `dart/external/imgui/`

**Condition:** `DART_USE_SYSTEM_IMGUI=OFF` (default)

**Purpose:** Dear ImGui - Immediate Mode GUI library

**Target:** `dart-external-imgui`

**Fetch Method:** CMake FetchContent
- **Repository:** `https://github.com/ocornut/imgui.git`
- **Tag:** `v1.84.2`
- **Minimum Version:** 1.80
- **Shallow Clone:** Yes

**Source Files (Fetched):**
- Core: `imgui.cpp`, `imgui_draw.cpp`, `imgui_tables.cpp`, `imgui_widgets.cpp`
- Headers: `imgui.h`, `imgui_internal.h`, `imconfig.h`, `imstb_*.h`
- Backend: `imgui_impl_opengl2.cpp/h` (OpenGL2 backend for OSG)

**Dependencies:**
- OpenGL (via `OpenGL::GL`)
- Cocoa framework (macOS only)

**Installation:** NOT installed (local build only)

**Linked By:** `dart-gui-osg` library

### 3. ConvHull 3D
**Location:** `dart/external/convhull_3d/`

**Purpose:** 3D convex hull computation

**Source Files:**
- `convhull_3d.h` - Convex hull algorithm
- `safe_convhull_3d.h` - Safe wrapper

### 4. IKFast
**Location:** `dart/external/ikfast/`

**Purpose:** IKFast solver interface

**Source Files:**
- `ikfast.h` - IKFast header

---

## Component Architecture

### Component Hierarchy

```
Component Dependency Tree:
└── dart (core)
    ├── external-odelcpsolver
    ├── Eigen3
    ├── fcl
    ├── assimp
    ├── fmt
    ├── spdlog (optional)
    └── octomap (optional)

    ├── optimizer-ipopt
    │   └── depends: dart, ipopt
    │
    ├── optimizer-nlopt
    │   └── depends: dart, nlopt
    │
    ├── collision-bullet
    │   └── depends: dart, bullet
    │
    ├── utils
    │   └── depends: dart, tinyxml2, libsdformat
    │
    ├── utils-urdf
    │   └── depends: utils, urdfdom
    │
    └── gui-osg
        └── depends: utils, OpenSceneGraph, ImGui, OpenGL
```

### Component Targets

| Component | Library Target | Dependencies |
|-----------|---------------|--------------|
| `dart` | `dart` | `dart-external-odelcpsolver`, `Eigen3::Eigen`, `fcl`, `assimp`, `fmt::fmt` |
| `optimizer-ipopt` | `dart-optimizer-ipopt` | `dart`, `ipopt` |
| `optimizer-nlopt` | `dart-optimizer-nlopt` | `dart`, `nlopt` |
| `collision-bullet` | `dart-collision-bullet` | `dart`, `bullet` |
| `utils` | `dart-utils` | `dart`, `tinyxml2`, `libsdformat` |
| `utils-urdf` | `dart-utils-urdf` | `dart-utils`, `urdfdom` |
| `gui-osg` | `dart-gui-osg` | `dart-utils`, `osg::osg`, `imgui::imgui` |
| `external-imgui` | `dart-external-imgui` | `OpenGL::GL` |

### Source Directory Structure

```
dart/
├── common/          # Common utilities and definitions
├── math/            # Mathematical utilities (vectors, matrices, transforms)
├── integration/     # Numerical integration methods
├── lcpsolver/       # Linear Complementarity Problem solver
├── optimizer/       # Optimization framework
│   ├── ipopt/      # IPOPT optimizer plugin
│   └── nlopt/      # NLopt optimizer plugin
├── dynamics/        # Dynamics engine (bodies, joints, skeletons)
├── collision/       # Collision detection framework
│   ├── dart/       # Native collision engine
│   ├── fcl/        # FCL collision engine
│   └── bullet/     # Bullet collision engine
├── constraint/      # Constraint solver
├── simulation/      # Simulation world and framework
├── utils/           # Utility functions
│   ├── sdf/        # SDF file parser
│   └── urdf/       # URDF file parser
└── gui/             # GUI components
    └── osg/         # OpenSceneGraph GUI
        ├── render/  # Rendering utilities
        └── detail/  # Implementation details
```

---

## Build Targets

### Library Targets

#### Core Libraries
- **`dart`** - Main DART library
  - Type: Shared/Static (platform-dependent)
  - Output: `libdart.so` / `libdart.a` / `dart.lib`
  - Install: Yes

- **`dart-external-odelcpsolver`** - Bundled ODE LCP solver
  - Output: `libdart-external-odelcpsolver.so`
  - Install: Yes

- **`dart-external-imgui`** - Bundled ImGui (if `DART_USE_SYSTEM_IMGUI=OFF`)
  - Output: `libdart-external-imgui.so`
  - Install: No (local build only)

#### Component Libraries
- **`dart-optimizer-ipopt`** - IPOPT optimizer plugin
- **`dart-optimizer-nlopt`** - NLopt optimizer plugin
- **`dart-collision-bullet`** - Bullet collision plugin
- **`dart-utils`** - Utility functions
- **`dart-utils-urdf`** - URDF parser
- **`dart-gui-osg`** - OpenSceneGraph GUI

### Python Bindings Target
- **`dartpy`** - Python bindings
  - Condition: `DART_BUILD_DARTPY=ON`
  - Build Type: Python extension module
  - Dependencies: `pybind11`, core DART libraries

### Custom Targets

#### Development Targets
- **`format`** - Auto-format C++ code with clang-format-14
- **`check-format`** - Check C++ code formatting
- **`examples`** - Build all examples
- **`tutorials`** - Build all tutorials
- **`tests`** - Build all unit tests
- **`ALL`** - Build everything (dartpy, tests, examples, tutorials)

#### Test Targets
- **`tests_and_run`** - Build and run tests
- **`pytest`** - Run Python tests

#### Documentation Targets
- **`docs`** - Generate Doxygen API documentation
- **`docs_forced`** - Force regenerate documentation
- **`view_docs`** - Open documentation in browser

#### Coverage Targets (when `DART_CODECOV=ON`)
- **`coverage`** - Generate coverage report
- **`coverage_html`** - Generate HTML coverage report
- **`coverage_view`** - View coverage report in browser

### Example Executables
Examples are built in `build/.../bin/`:
- `hello_world` - Basic DART usage
- `atlas_puppet` - Atlas robot control
- `atlas_simbicon` - Atlas robot with Simbicon controller
- Various other examples...

### Tutorial Executables
Tutorials are built in `build/.../bin/`:
- `tutorial_biped` / `tutorial_biped_finished`
- `tutorial_collisions` / `tutorial_collisions_finished`
- `tutorial_dominoes` / `tutorial_dominoes_finished`
- `tutorial_multi_pendulum` / `tutorial_multi_pendulum_finished`

### Benchmark Executables (Performance Testing)
- `BM_INTEGRATION_empty`
- `BM_INTEGRATION_boxes`
- `BM_INTEGRATION_kinematics`

---

## Environment Management

### Pixi Configuration
**File:** `pixi.toml`

**Purpose:** Cross-platform environment and dependency management using conda-forge packages.

### Platforms Supported
- `linux-64` - Linux x86_64
- `osx-64` - macOS x86_64 (Intel)
- `osx-arm64` - macOS ARM64 (Apple Silicon)
- `win-64` - Windows x86_64

### Pixi Tasks

#### Configuration Tasks
```bash
pixi run config          # Configure CMake (Release)
pixi run config-debug    # Configure CMake (Debug)
pixi run config-py       # Configure with Python bindings
pixi run config-coverage # Configure with coverage
pixi run config-install  # Configure for installation
```

#### Build Tasks
```bash
pixi run build           # Build DART (Release)
pixi run build-debug     # Build DART (Debug)
pixi run build-tests     # Build unit tests
pixi run build-py-dev    # Build Python bindings
pixi run build-coverage  # Build with coverage
```

#### Test Tasks
```bash
pixi run test            # Run C++ tests
pixi run test-py         # Run Python tests
pixi run test-all        # Run all tests
```

#### Linting Tasks
```bash
pixi run lint            # Format C++ and Python code
pixi run lint-cpp        # Format C++ code only
pixi run lint-py         # Format Python code only
pixi run check-lint      # Check formatting (CI)
pixi run check-lint-cpp  # Check C++ formatting
pixi run check-lint-py   # Check Python formatting
```

#### Example/Tutorial Tasks
```bash
pixi run ex-hello-world  # Run hello_world example
pixi run ex-atlas-puppet # Run atlas_puppet example
pixi run ex-atlas-simbicon # Run atlas_simbicon example

pixi run tu-biped        # Run biped tutorial
pixi run tu-collisions   # Run collisions tutorial
pixi run tu-dominoes     # Run dominoes tutorial
```

#### Python Example Tasks
```bash
pixi run py-ex-hello-world      # Python hello world
pixi run py-ex-rigid-cubes      # Python rigid cubes
pixi run py-ex-drag-and-drop    # Python drag and drop
pixi run py-ex-operational-space-control
```

#### Documentation Tasks
```bash
pixi run docs-build      # Build user documentation
pixi run docs-serve      # Serve documentation (port 8000)
pixi run api-docs-cpp    # Build C++ API docs
pixi run api-docs-py     # Build Python API docs
```

#### Profiling Tasks
```bash
pixi run tracy           # Launch Tracy profiler GUI
```

#### Utility Tasks
```bash
pixi run clean           # Clean build artifacts
pixi run install         # Install DART to conda prefix
pixi run generate-stubs  # Generate Python stub files
```

### Environment Variables

#### CMake Configuration
- `CMAKE_INSTALL_PREFIX` - `$CONDA_PREFIX`
- `CMAKE_BUILD_TYPE` - `Release` / `Debug`
- `CMAKE_PREFIX_PATH` - `$CONDA_PREFIX`

#### Build Options
- `DART_BUILD_DARTPY` - `ON` (for Python tasks)
- `DART_BUILD_PROFILE` - `ON`
- `DART_USE_SYSTEM_GOOGLEBENCHMARK` - `ON`
- `DART_USE_SYSTEM_GOOGLETEST` - `ON`
- `DART_USE_SYSTEM_IMGUI` - `ON`
- `DART_USE_SYSTEM_TRACY` - `ON`
- `DART_VERBOSE` - `OFF` (configurable)

#### Python Environment
- `PYTHONPATH` - `build/$PIXI_ENVIRONMENT_NAME/cpp/$BUILD_TYPE/python/dartpy`

### Build Output Structure
```
build/
└── $PIXI_ENVIRONMENT_NAME/
    └── cpp/
        ├── Release/         # Release build
        │   ├── lib/         # Libraries
        │   ├── bin/         # Executables
        │   └── python/      # Python bindings
        │       └── dartpy/
        └── Debug/           # Debug build
            ├── lib/
            ├── bin/
            └── python/
                └── dartpy/
```

### Special Features

#### Gazebo Integration Feature
**Environment:** `pixi run -e gazebo <task>`

**Purpose:** Test DART integration with Gazebo Physics

**Tasks:**
- `download-gz` - Download gz-physics source
- `patch-gz` - Patch DART version requirement
- `config-gz` - Configure gz-physics build
- `build-gz` - Build gz-physics with dartsim plugin
- `test-gz` - Verify DART integration

**Dependencies:**
- `libgz-cmake4`, `libgz-plugin3`, `libgz-math8`
- `libgz-common6`, `libgz-utils2`, `libsdformat15`

---

## ROS Integration

### Package Manifest
**File:** `package.xml`

**Format:** Catkin package.xml (format 2)

### Package Information
- **Name:** `dartsim`
- **Version:** 7.0.0
- **Build Type:** `cmake`
- **Maintainer:** Michael X. Grey <grey@openrobotics.org>
- **Author:** Jeongseok Lee <jslee02@gmail.com>, C. Karen Liu

### ROS Dependencies

#### Build Dependencies
```xml
<build_depend>pkg-config</build_depend>
```

#### Runtime Dependencies
```xml
<depend>assimp</depend>
<depend>bullet</depend>
<depend>eigen</depend>
<depend>libfcl-dev</depend>
<depend>liburdfdom-dev</depend>
<depend>tinyxml2</depend>
<depend>sdformat</depend>
```
