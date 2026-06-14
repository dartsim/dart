# DART Build System Analysis

**Project:** DART (Dynamic Animation and Robotics Toolkit)
**Version:** 7.0.0
**Analysis Date:** 2025-10-19
**Root Directory:** `<project_root>`

---

## Quickstart (Read This First)

- Day-to-day `pixi run …` workflows: see [building.md](building.md).
- Gazebo / gz-physics integration: jump to [Gazebo Integration Feature](#gazebo-integration-feature). Compatibility gate: `DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> pixi run -e gazebo test-gz`.
- CI workflow overview + `gh` monitoring commands: see [ci-cd.md](ci-cd.md).

<details>
<summary>Deep dive (optional): build system analysis</summary>

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

- **CMake Version:** ≥ 4.2.3
- **Build Generator:** Ninja (via pixi) / CMake default
- **C++ Standard:** C++23
- **Package Manager:** pixi (conda-forge based)
- **Python:** Python 3.14+ (for dartpy bindings)

### Build Configurations

- **Release** (default): `-O3 -DNDEBUG`
- **Debug**: `-g -fno-omit-frame-pointer -fno-inline-functions`
- **RelWithDebInfo**: Combines Release + Debug flags
- **Profile**: Debug + profiling (`-pg`)

### Key Build Options

```cmake
DART_BUILD_GUI          = ON   # Filament-backed GUI; Linux x86_64 default
DART_USE_SYSTEM_FILAMENT = ON  # Use installed Filament or Filament_ROOT
DART_FETCH_FILAMENT     = ON   # Fetch pinned Filament archive fallback
DART_BUILD_DARTPY           = OFF  # Build Python bindings
DART_BUILD_PROFILE          = OFF  # Enable profiling support
DART_PROFILE_TRACY          = OFF  # Enable Tracy profiler backend (developer-only)
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
├── common/         # Common utilities
├── math/           # Math utilities (includes math/optimization/)
├── lcpsolver/      # LCP solver
├── optimizer/      # Deprecated alias headers that forward to math/optimization
├── dynamics/       # Dynamics engine
├── collision/      # Collision detection (FCL + optional Bullet/ODE)
├── constraint/     # Constraint handling
├── simulation/     # Simulation framework + time stepping
├── utils/          # Parsers and helpers (URDF/SDF)
└── gui/            # Filament-backed GUI components
    └── detail/
```

#### 3. dart/gui/CMakeLists.txt

**File:** `dart/gui/CMakeLists.txt`

**Responsibilities:**

- Builds `dart-gui` library
- Integrates the Filament-backed GUI implementation
- Integrates GLFW3 and ImGui
- Generates component headers (`all.hpp`, `gui.hpp`) and installs promoted
  `dart/gui/*.hpp` scene/viewer descriptor headers

**Dependencies:**

- private GUI implementation sources
- Filament 1.71.3, GLFW3, ImGui, PNG, and JPEG for the private backend

---

## External Dependencies

### Required Core Dependencies

#### 1. Eigen3 (Linear Algebra)

- **Version:** ≥ 3.4.0
- **Purpose:** Matrix operations, linear algebra
- **CMake Module:** `cmake/DARTFindEigen3.cmake`
- **Find Package:** `Eigen3` (CONFIG mode)

#### 2. assimp (Asset Importer)

- **Version:** ≥ 5.4.3, < 6
- **Purpose:** 3D model loading (meshes, skeletons)
- **CMake Module:** `cmake/DARTFindassimp.cmake`
- **Special Checks:** Constructor/destructor availability for `aiScene` and `aiMaterial`

#### 3. fmt (Formatting Library)

- **Version:** ≥ 11.1.4, < 12
- **Purpose:** String formatting
- **CMake Module:** `cmake/DARTFindfmt.cmake`
- **Targets:** `fmt::fmt` or `fmt::fmt-header-only`

### Optional Dependencies

#### 4. spdlog (Logging)

- **Version:** ≥ 1.15.3, < 2
- **Purpose:** Fast C++ logging library
- **CMake Module:** `cmake/DARTFindspdlog.cmake`
- **Targets:** `spdlog::spdlog` or `spdlog::spdlog_header_only`
- **Skip Option:** `DART_SKIP_spdlog`

#### 5. FCL (Flexible Collision Library)

- **Version:** ≥ 0.7.0, < 0.8
- **Purpose:** Reference collision tests and benchmarks only
- **CMake Module:** `cmake/DARTFindfcl.cmake`
- **ROS Dependency:** `libfcl-dev`
- **Scope:** Not discovered or linked by core DART libraries unless reference
  collision tests or benchmarks are enabled

#### 6. octomap (Octree-based 3D mapping)

- **Version:** ≥ 1.10.0, < 2
- **Purpose:** Test and benchmark comparisons for native occupancy-grid
  behavior
- **CMake Module:** `cmake/DARTFindoctomap.cmake`
- **Scope:** Not discovered or linked by core DART libraries

### GUI-Specific Dependencies

#### DART GUI Stack

- **Version:** Filament 1.71.3 for the pinned Linux x86_64 fetch.
- **Purpose:** Maintained built-in 3D visualization and interaction renderer.
- **Windowing/UI:** GLFW3 and Dear ImGui are private backend dependencies.
- **Options:**
  - `DART_BUILD_GUI=ON` builds `dart-gui`, the `dartsim` executable, and the
    Filament smoke-test target. It defaults to `ON` only on Linux x86_64, where
    the pinned Filament archive is supported.
  - `DART_FETCH_FILAMENT=ON` fetches the pinned Filament archive for supported
    platforms.
  - `DART_USE_SYSTEM_FILAMENT=ON` discovers an installed package or `Filament_ROOT`.

OpenSceneGraph and Raylib are no longer maintained renderer options in the
active build.

#### ImGui (Immediate Mode GUI)

- **Version:** ≥ 1.91.9, < 2 (system), v1.84.2 (fetched)
- **Purpose:** In-scene GUI widgets and overlays
- **CMake Module:** `cmake/DARTFindimgui.cmake`
- **Fetch Module:** `dart/external/imgui/CMakeLists.txt`
- **Backend:** Private DART GUI overlay integration
- **Target:** `imgui::imgui`
- **Modes:**
  - **System Mode** (`DART_USE_SYSTEM_IMGUI=ON`): Links against system package
  - **Fetch Mode** (`DART_USE_SYSTEM_IMGUI=OFF`): Downloads from GitHub
    - Repository: `https://github.com/ocornut/imgui.git`
    - Tag: `v1.84.2`
    - Not installed with DART (local build only)

#### Filament

- **Purpose:** DART's maintained built-in GUI renderer.
- **Options:**
  - **Enable**: `DART_BUILD_GUI=ON`
  - **System Mode** (`DART_USE_SYSTEM_FILAMENT=ON`): Finds an installed
    Filament tree, typically through `Filament_ROOT`
  - **Fetch Mode** (`DART_FETCH_FILAMENT=ON`): Explicitly fetches the pinned
    Filament archive for supported platforms, including official dartpy wheels
- **Public build flag:** Keep `DART_BUILD_GUI` as the single public option for
  the GUI surface. Filament is the maintained backend, so do not add
  backend-specific public toggles such as `DART_BUILD_GUI_FILAMENT`; use
  dependency-selection options such as `DART_USE_SYSTEM_FILAMENT` and
  `DART_FETCH_FILAMENT` for packaged versus fetched Filament.
- **Migration:** The full replacement plan lives in
  [gui-rendering.md](gui-rendering.md). New public GUI APIs should describe
  DART concepts and keep Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal,
  OSG, and Raylib types private.

#### OpenGL

- **Purpose:** Platform graphics support used by Filament on OpenGL-capable
  systems.
- **Target:** `OpenGL::GL`
- **Platform:** All (system-provided)

### Collision Engine Dependencies (Optional Components)

#### 10. Bullet Physics

- **Version:** ≥ 3.25, < 4
- **Purpose:** Alternative collision detection
- **Option:** `DART_BUILD_COLLISION_BULLET`
- **Integration:** Compiled directly into the core `dart` target (no separate component)
- **CMake Module:** `cmake/DARTFindBullet.cmake`
- **Failure mode:** Configuration aborts with `FATAL_ERROR` if the option is `ON` and Bullet is missing.
- **Disable:** There is no `DART_SKIP_Bullet`; set `DART_BUILD_COLLISION_BULLET=OFF` to omit the backend entirely.

#### 11. Open Dynamics Engine (ODE)

- **Version:** ≥ 0.13, < 1
- **Purpose:** Alternative collision detection
- **Option:** `DART_BUILD_COLLISION_ODE`
- **Integration:** Compiled directly into the core `dart` target (no separate component)
- **CMake Module:** `cmake/DARTFindODE.cmake`
- **Failure mode:** Configuration aborts with `FATAL_ERROR` if the option is `ON` and ODE is missing.
- **Disable:** There is no `DART_SKIP_ODE`; set `DART_BUILD_COLLISION_ODE=OFF` to omit the backend entirely.

### Utility Dependencies

#### 12. tinyxml2

- **Version:** ≥ 11.0.0, < 12
- **Purpose:** XML parsing (for SDF)
- **Component:** `dart-utils`
- **CMake Module:** `cmake/DARTFindtinyxml2.cmake`

#### 13. libsdformat

- **Version:** ≥ 16.0.0, < 17
- **Purpose:** Official SDFormat parser used to canonicalize files (version conversion, `<include>` resolution, URI normalization) before DART walks the DOM.
- **Component:** `dart-utils`
- **CMake Module:** `cmake/DARTFindsdformat.cmake`
- **Notes:** Required for all SDF parsing. DART no longer ships a fallback XML code path, so builds without libsdformat cannot load `.sdf`/`.world` assets.

#### 14. urdfdom

- **Version:** ≥ 4.0.1, < 5
- **Purpose:** URDF parsing
- **Component:** `dart-utils-urdf`
- **CMake Module:** `cmake/DARTFindurdfdom.cmake`
- **ROS Dependency:** `liburdfdom-dev`

### Build/Test Dependencies (Build-time only)

#### 15. Google Test (gtest)

- **Version:** ≥ 1.17.0, < 2
- **Purpose:** Unit testing framework
- **Option:** `DART_USE_SYSTEM_GOOGLETEST`

#### 16. Google Benchmark

- **Version:** ≥ 1.9.3, < 2
- **Purpose:** Performance benchmarking
- **Option:** `DART_USE_SYSTEM_GOOGLEBENCHMARK`

#### 17. Tracy Profiler

- **Purpose:** Optional local developer frame profiling
- **Backend option:** `DART_PROFILE_TRACY`
- **Option:** `DART_USE_SYSTEM_TRACY`
- **Default:** `OFF` for CMake, Pixi development, and package builds.
  `pixi run tracy-build` opts into `DART_PROFILE_TRACY=ON` only for the
  developer profiling build and keeps `DART_USE_SYSTEM_TRACY=OFF`, so Tracy is
  not installed through package-manager dependencies. `pixi run tracy` launches
  the fetched GUI client. The launcher defaults `TRACY_DPI_SCALE=1` to avoid
  oversized Linux DPI auto-scaling; set `TRACY_DPI_SCALE` explicitly to override
  it.

### Additional Platform Dependencies

#### Linux (linux-64)

- **lcov:** ≥ 1.16, < 2 (coverage reports)

#### macOS

- **Cocoa Framework** (transitively required by windowing/rendering packages)

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
- Backend integration: private DART GUI overlay code

**Dependencies:**

- OpenGL (via `OpenGL::GL`)
- Cocoa framework (macOS only)

**Installation:** NOT installed (local build only)

**Linked By:** `dart-gui` library

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

**Notes:**

- The actual solver binaries are supplied by users (generated with OpenRAVE's
  IkFast tooling). See
  `docs/readthedocs/shared/inverse_kinematics/ikfast.rst` for the current
  support policy and integration guide.

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
    ├── collision-fcl (native-backed compatibility facade; no FCL dependency)
    ├── collision-bullet (native-backed compatibility facade; no Bullet dependency)
    └── collision-ode (native-backed compatibility facade; no ODE collision dependency)

Reference test/benchmark targets, when enabled:
    ├── dart-test-reference-fcl (optional; reference tests/benchmarks only)
    ├── dart-test-reference-bullet (optional; reference tests/benchmarks only)
    ├── dart-test-reference-ode (optional; reference tests/benchmarks only)
    └── octomap (optional; occupancy-grid tests/benchmarks only)

    ├── utils
    │   └── depends: dart, tinyxml2, libsdformat
    │
    ├── utils-urdf
    │   └── depends: utils, urdfdom
    │
    ├── io
    │   └── depends: utils (+ utils-urdf when available)
    │
    └── gui
        └── depends: private Filament, GLFW3, ImGui implementation
```

### Component Targets

| Component        | Library Target        | Dependencies                                                                                                                        |
| ---------------- | --------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `dart`           | `dart`                | `dart-external-odelcpsolver`, `Eigen3::Eigen`, `fcl`, `assimp`, `fmt::fmt` (plus Bullet/ODE when the collision options are enabled) |
| `utils`          | `dart-utils`          | `dart`, `tinyxml2`, `libsdformat`                                                                                                   |
| `utils-urdf`     | `dart-utils-urdf`     | `dart-utils`, `urdfdom`                                                                                                             |
| `io`             | `dart-io`             | `dart-utils` (plus `dart-utils-urdf` when available)                                                                                |
| `gui`            | `dart-gui`            | private GUI implementation, Filament, GLFW3, ImGui, PNG, JPEG                                                                       |
| `external-imgui` | `dart-external-imgui` | `OpenGL::GL`                                                                                                                        |

> Bullet and ODE no longer create standalone `dart-collision-*` components. When `DART_BUILD_COLLISION_BULLET` or `DART_BUILD_COLLISION_ODE` is `ON`, their sources and link dependencies are baked directly into the `dart` target.

> Advanced optimizer targets (IPOPT, NLopt, pagmo, SNOPT) were moved to [dart-optimization](https://github.com/dartsim/dart-optimization). The `dart/optimizer` directory that remains in this repo only ships deprecated headers that forward to `dart/math/optimization`.

### Optional Component Output Directories

Optional component libraries declared outside the core `dart/` subtree may not
inherit the output-directory properties used by the primary DART libraries. If
an in-tree example, test, or exported component links a target as
`-ldart-<component>`, set `ARCHIVE_OUTPUT_DIRECTORY` and
`LIBRARY_OUTPUT_DIRECTORY` to `${DART_BINARY_DIR}/lib`, and set
`RUNTIME_OUTPUT_DIRECTORY` to `${DART_BINARY_DIR}/bin` when the target can
produce runtime artifacts.

Validate new optional components from a clean build by building the executable
or example that links them. Incremental local builds can hide this class of
failure when a stale library already exists in the shared output directory.

### Source Directory Structure

```
dart/
├── common/          # Common utilities and definitions
├── math/            # Mathematical utilities (vectors, matrices, transforms)
│   └── optimization/       # Gradient-descent + IK helpers
├── optimizer/       # Deprecated alias headers (forwards to math/optimization)
├── lcpsolver/       # Linear Complementarity Problem solver
├── dynamics/        # Dynamics engine (bodies, joints, skeletons)
├── collision/       # Collision detection framework
│   ├── dart/       # Native collision engine
│   ├── fcl/        # FCL collision engine
│   ├── bullet/     # Bullet collision engine (optional)
│   └── ode/        # ODE collision engine (optional)
├── constraint/      # Constraint solver
├── simulation/      # Simulation world, integration, and time stepping
├── io/              # Unified skeleton loading (readSkeleton)
├── utils/           # Utility functions
│   ├── sdf/        # SDF file parser
│   └── urdf/       # URDF file parser
└── gui/             # Filament-backed GUI components
    └── detail/
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

- **`dart-utils`** - Utility functions
- **`dart-utils-urdf`** - URDF parser
- **`dart-io`** - Unified model loading (`dart::io`)
- **`dart-gui`** - Filament-backed GUI

### Python Bindings Target

- **`dartpy`** - Python bindings
  - Condition: `DART_BUILD_DARTPY=ON`
  - Build Type: Python extension module
  - Dependencies: `nanobind`, core DART libraries

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

- `dartsim` - Application-level simulator/viewer and migrated visual scenes
- `dart-demos` - Runtime-switchable DART 7 World demo catalog

Examples that depend on optional component libraries must guard on the required
CMake targets before registering the executable. For example, a GUI example
that also links `dart-simulation` should return early unless both
`dart-gui` and `dart-simulation` exist; otherwise CI
configurations that build examples with a skipped prerequisite target can
register an executable that later fails to link. Validate this class of change
with `pixi run build-examples ON Release`.

### Tutorial Executables

The legacy OpenSceneGraph tutorial executables were removed. The
`DART_BUILD_TUTORIALS` option remains accepted for compatibility but no longer
adds those targets.

### Benchmark Executables (Performance Testing)

- Benchmarks are built in `build/.../bin/` and can be run via pixi:
  - `pixi run bm boxes`
  - `pixi run bm kinematics`
  - `pixi run bm lcp_compare`
  - `pixi run bm --pixi-help` for more aliases/targets

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
pixi run -e cuda test-all # Run CUDA-enabled full validation on Linux CUDA hosts
```

#### Linting Tasks

```bash
pixi run lint             # Format code + docs (C++, simulation, Python, YAML, TOML, MD) and lint RST
pixi run lint-cpp         # Format C++ code only
pixi run lint-simulation  # Format simulation C++ sources
pixi run lint-py          # Format Python code only
pixi run lint-yaml        # Format YAML files
pixi run lint-toml        # Format TOML files
pixi run lint-md          # Format Markdown files
pixi run lint-rst         # Lint reStructuredText files
pixi run check-lint       # Check formatting/linting (CI)
pixi run check-lint-cpp   # Check C++ formatting
pixi run check-lint-simulation # Check simulation formatting
pixi run check-lint-py    # Check Python formatting
pixi run check-lint-yaml  # Check YAML formatting
pixi run check-lint-rst   # Check reStructuredText files
```

#### Example/Tutorial Tasks

```bash
pixi run demos -- --scene rigid_body # Run a DART 7 World demo scene

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
pixi run tracy-build     # Build the Tracy GUI client from fetched source
pixi run tracy           # Build and launch the Tracy GUI client
TRACY_DPI_SCALE=1.5 pixi run tracy
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
- `DART_PROFILE_TRACY` - `OFF` by default; `pixi run tracy-build` opts in for
  the developer profiling build
- `DART_USE_SYSTEM_TRACY` - `OFF`
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

</details>

### Special Features

#### Gazebo Integration Feature

**Environment:** `pixi run -e gazebo <task>`

**Purpose:** Test DART integration with Gazebo Physics

**Context (why this exists):** This workflow is the compatibility gate for a pinned gz-physics checkout (`gz-physics9_9.0.0`) built against the DART installed from this repository. When updating DART (e.g., DART 7), keep downstream compatibility by fixing issues in DART (or upstream gz-physics) rather than carrying local gz-physics source patches here. DART 7's generated package-version file satisfies the pinned DART 6.10+ CMake requirement, so the local workflow validates that requirement instead of editing downstream sources.

**Tasks:**

- `download-gz` - Download gz-physics source
- `patch-gz` - Validate DART version requirement compatibility
- `config-gz` - Configure gz-physics build
- `build-gz` - Build gz-physics with dartsim plugin
- `test-gz` - Verify DART integration

**Start here next time:**

- [`pixi.toml`](../../pixi.toml) (`[feature.gazebo]`) - task chain + env overrides
- [`scripts/patch_gz_physics.py`](../../scripts/patch_gz_physics.py) - validates the pinned DART version requirement without mutating gz-physics sources
- [`cmake/gz_physics_force_vendor_gtest.cmake`](../../cmake/gz_physics_force_vendor_gtest.cmake) - ensures gz-physics uses its vendored GoogleTest headers
- [`.github/workflows/ci_gz_physics.yml`](../../.github/workflows/ci_gz_physics.yml) - CI entry point for this workflow

**Required compatibility-gate command:**

```bash
N=${DART_SAFE_JOBS:-$(python scripts/parallel_jobs.py)}
DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
```

This command downloads the pinned `gz-physics9_9.0.0` branch, validates the
DART version requirement through `scripts/patch_gz_physics.py`, configures with
tests enabled, builds the DART plugin and selected gz-physics tests, runs
`ctest --tests-regex "^(UNIT_|check_UNIT_|COMMON_TEST_.*dartsim)"`, and checks
that the generated `libgz-physics-dartsim-plugin` links to DART libraries.

**Fast iteration loop (Suggested (Unverified)):**

1. Run `pixi run lint` before committing changes.
2. Configure gz-physics with `DART_PARALLEL_JOBS=<N> pixi run -e gazebo config-gz`.
3. Build the failing target(s) with `DART_PARALLEL_JOBS=<N> pixi run -e gazebo ninja -C .deps/gz-physics/build -j <N> <target>`.
4. Run the full suite with `DART_PARALLEL_JOBS=<N> pixi run -e gazebo test-gz`.

**What to look for (success signal):**

- The selected `UNIT_`, `check_UNIT_`, and `COMMON_TEST_.*dartsim` ctest targets
  pass with `CTEST_OUTPUT_ON_FAILURE=1`.
- The plugin linkage check prints a DART library from `ldd` or `otool -L`.
- `test-gz` prints: `✓ DART plugin built successfully with DART integration!`

For PR or release evidence, paste either the `CI gz-physics` run URL or a short
local transcript summary with the pinned branch, command, selected ctest regex,
plugin link-check result, and final success line. If a gz-physics-sensitive
change skips this gate, state the deferral reason in the PR description.

**Gotchas (common failures and how to respond):**

- **Missing DART components at configure time.** Example errors:
  - `Could NOT find DART (missing: collision-bullet collision-ode) (Required is at least version "7.0")`
  - `... but it set DART_FOUND to FALSE ...`
  - **Resolution:** The downstream is requesting legacy components. In DART 7,
    Bullet/ODE backends are part of the core `dart` component;
    `collision-bullet` / `collision-ode` exist only as deprecated compatibility
    components while clean-break gates are being closed. Prefer to update
    downstream to depend on `dart`; keep this workflow passing on the DART 6.17
    support lane for existing consumers.
- **No local gz-physics source patches.** Keep `scripts/patch_gz_physics.py`
  limited to validating the DART version requirement; otherwise this workflow
  stops catching real compatibility breaks.
- **gtest header mismatches.** Symptom: link errors like `undefined reference to testing::internal::MakeAndRegisterTestInfo(std::string, ...)` when building gz-physics tests. The `config-gz` task passes `-DCMAKE_PROJECT_TOP_LEVEL_INCLUDES:FILEPATH=$PWD/cmake/gz_physics_force_vendor_gtest.cmake` to ensure gz-physics compiles against the vendored headers that match its vendored gtest library; keep that behavior.
- **Deprecation noise is expected.** When gz-physics links the deprecated compatibility targets, CMake may emit deprecation warnings; these are intentional and should be treated as migration pressure for downstreams.
- **Type-name API compatibility.** `CollisionDetector::getType()` and `BoxedLcpSolver::getType()` return `const std::string&` for gz-physics compatibility; prefer `getTypeView()` in DART code, and keep `test-gz` green before changing signatures.
- **Constraint solver type compatibility.** gz-physics does `dynamic_cast<BoxedLcpConstraintSolver*>` on the solver returned by `World::getConstraintSolver()`. If `World` creates a base `ConstraintSolver` instead of `BoxedLcpConstraintSolver`, the cast fails and gz-physics tests break (`COMMON_TEST_world_features_dartsim`, `UNIT_WorldFeatures_TEST`). Always use `BoxedLcpConstraintSolver` (deprecated but required) when constructing the solver in `createConstraintSolver()` in `world.cpp`.
- **The gz-physics checkout is ephemeral.** `download-gz` clones into
  `.deps/gz-physics`; those files are expected and should remain untracked.

**Dependencies:**

- Managed by pixi in `pixi.toml` under `[feature.gazebo.dependencies]` (avoid duplicating versioned package names here).

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
