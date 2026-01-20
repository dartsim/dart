# Building DART from Source

This guide describes how to build DART from source, including both the C++ library and Python bindings (dartpy).

## Start here next time

- Local test workflow: [testing.md](testing.md)
- CI monitoring and expectations: [ci-cd.md](ci-cd.md)
- Gazebo / gz-physics integration: [build-system.md](build-system.md#gazebo-integration-feature)

## Supported Environments

DART is supported on the following operating systems and compilers:

| Operating System      | Compiler           |
| --------------------- | ------------------ |
| Ubuntu 22.04 or later | GCC 11.2 or later  |
| Windows 2022 or later | Visual Studio 2022 |
| macOS 13 or later     | Clang 13 or later  |

> **Note:** DART requires C++20. See [Compatibility Policy](compatibility-policy.md) for details on how platform requirements are determined.

## Prerequisites

Before you can build DART, you'll need to install the required and optional dependencies. The required dependencies are the minimum set needed to build DART, while the optional dependencies enable additional features.

> **Note:** The dependencies and installation steps are subject to change. Please report any issues you encounter and contribute to keeping the instructions up-to-date.

### Ubuntu

Install required dependencies using `apt`:

```bash
sudo apt install \
  build-essential cmake pkg-config git libassimp-dev \
  libeigen3-dev libfcl-dev libfmt-dev \
  libsdformat15 libgz-math8 libgz-utils2
```

> **Note:** Replace the `libsdformat15`/`libgz-*` package names with the versions available on your distribution. These packages are published via [packages.osrfoundation.org](https://packages.osrfoundation.org).

Install optional dependencies:

```bash
sudo apt install \
  coinor-libipopt-dev libbullet-dev \
  libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev \
  libopenscenegraph-dev libnlopt-cxx-dev liboctomap-dev libode-dev \
  libspdlog-dev libyaml-cpp-dev ocl-icd-opencl-dev opencl-headers \
  opencl-clhpp-headers
```

### macOS

Add the OSRF tap and install required dependencies using `brew`:

```bash
brew tap osrf/simulation
brew install assimp cmake eigen fmt fcl osrf/simulation/sdformat13
```

> **Note:** Replace `sdformat13` with the latest formula published in the `osrf/simulation` tap.

Install optional dependencies:

```bash
brew install bullet ipopt nlopt octomap ode \
  open-scene-graph --HEAD \
  spdlog tinyxml2 urdfdom yaml-cpp
```

### Windows

Install required dependencies using `vcpkg`:

```bash
vcpkg install --triplet x64-windows assimp eigen3 fcl fmt sdformat spdlog
```

Install optional dependencies:

```bash
vcpkg install --triplet x64-windows \
  assimp eigen3 fcl fmt spdlog bullet3 glfw3 nlopt ode \
  opencl opengl osg pagmo2 nanobind tinyxml2 urdfdom yaml-cpp
```

### Arch Linux (experimental)

Install required dependencies using `yay`:

```bash
yay -S assimp cmake eigen fcl fmt sdformat
```

Install optional dependencies:

```bash
yay -S \
  bullet coin-or-ipopt nlopt octomap ode opencl-clhpp \
  opencl-headers opencl-icd-loader openscenegraph pagmo spdlog tinyxml2 \
  urdfdom nanobind
```

### FreeBSD (experimental)

TODO

## Dependency Reference

For the complete and up-to-date list of dependencies with version requirements, refer to:

- [`CMakeLists.txt`](../../CMakeLists.txt) - Authoritative source for CMake dependencies and version requirements
- [`pixi.toml`](../../pixi.toml) - Managed dependencies for reproducible builds

## Recommended pixi Workflow

We ship a [pixi](https://pixi.sh) environment for contributors. Pixi installs every required dependency (CMake, Ninja, compilers, Python, optional libraries) and exposes reproducible tasks so you do not have to manage toolchains manually.

1. [Install pixi](https://pixi.sh/latest/#installation) and run `pixi install` once to create the environment.
2. Configure a build:

   ```bash
   pixi run config                # Release (default)
   pixi run config-debug          # Debug
   ```

   `config` accepts args for common toggles:

   ```bash
   pixi run config OFF            # Suggested (Unverified): disable dartpy
   pixi run config ON Debug       # Suggested (Unverified): enable dartpy + Debug build
   ```

   You can also override booleans via environment variables instead of editing `pixi.toml` (Suggested (Unverified)):

   ```bash
   DART_BUILD_DARTPY_OVERRIDE=OFF pixi run config
   DART_BUILD_GUI_OVERRIDE=OFF pixi run config
   DART_BUILD_GUI_RAYLIB_OVERRIDE=ON DART_USE_SYSTEM_RAYLIB_OVERRIDE=OFF pixi run config
   ```

   Note: Official dartpy wheels include OSG/GUI; keep `DART_BUILD_GUI` enabled when validating dartpy changes.

3. Build and test:

   ```bash
   pixi run build                 # cmake --build … --target all
   pixi run build-tests           # builds the C++ test targets
   pixi run test                  # ctest -LE simulation-experimental
   pixi run test-all              # helper script that runs lint + build + tests
   ```

   Parallelism knobs (optional):
   - `DART_PARALLEL_JOBS` - used by helper scripts and the Gazebo workflow
   - `CMAKE_BUILD_PARALLEL_LEVEL` - controls `cmake --build`
   - `CTEST_PARALLEL_LEVEL` - controls `ctest`

   If you want to cap parallelism for local runs, pick a value around two-thirds of logical cores to keep the machine responsive, then set `DART_PARALLEL_JOBS` (and `CTEST_PARALLEL_LEVEL` for ctest). Suggested (Unverified):

   ```bash
   # Linux
   N=$(( ( $(nproc) * 2 ) / 3 ))
   # macOS
   N=$(( ( $(sysctl -n hw.ncpu) * 2 ) / 3 ))
   # Windows (PowerShell)
   $env:N=[math]::Floor([Environment]::ProcessorCount*2/3)
   ```

   Suggested (Unverified):

   ```bash
   pixi run lint
   cmake --build <BUILD_DIR> --target <TARGET>
   ```

   Example:

   ```bash
   pixi run lint
   cmake --build build/default/cpp/Release --target UNIT_gui_MeshShapeNodeMaterialUpdates
   ```

   Suggested (Unverified):

   ```bash
   DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> pixi run test
   DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> pixi run test-all
   ```

   Note: `pixi run test-all` runs `pixi run lint` (auto-fixing) internally; check `git status` afterwards before committing.

4. (Optional) Gazebo / gz-physics integration test:

   Suggested (Unverified):

   ```bash
   DART_PARALLEL_JOBS=<N> CTEST_PARALLEL_LEVEL=<N> pixi run -e gazebo test-gz
   ```

   This runs the gz-physics integration workflow (task chain, patch policy, and common failure modes are documented in [build-system.md](build-system.md#gazebo-integration-feature)).

Pixi automatically detects whether optional Ninja targets (for example `pytest` or GUI tutorials) were generated. If a target is missing (because its corresponding `DART_BUILD_*` option is `OFF`), the helper scripts skip it instead of hard failing, which mirrors the CI workflow. You can still use the “manual” CMake flow described below, but pixi is the fastest path to a working development environment on every platform.

## Clone the Repository

1. Clone the DART repository:

   ```bash
   git clone https://github.com/dartsim/dart.git
   ```

2. (Optional) Checkout a specific version:

   ```bash
   git checkout -b <branch_or_tag_or_commit>
   ```

> **Note:** The DART repository is actively maintained. For the latest information, refer to the [DART GitHub repository](https://github.com/dartsim/dart).

## Build Configuration

DART uses CMake as its build system. CMake generates build files for various build systems, including Makefiles, Visual Studio projects, and Xcode projects. For more information, see the [CMake generators documentation](https://cmake.org/cmake/help/latest/manual/cmake-generators.7.html).

### Basic Configuration

1. Create a build directory:

   ```bash
   mkdir build
   cd build
   ```

2. Run CMake:

   ```bash
   cmake ..
   ```

### CMake Options

For all available CMake configuration options and their defaults, refer to [`CMakeLists.txt`](../../CMakeLists.txt). Common options include:

- `CMAKE_BUILD_TYPE` - Build configuration (Release, Debug, etc.). Only applies to single-config generators (e.g., Ninja, Unix Makefiles). Multi-config generators (Visual Studio, Xcode) expose the configuration inside the IDE or via `cmake --build` `--config`.
- `DART_BUILD_DARTPY` - Enable Python bindings
- `DART_BUILD_GUI` - Enable OpenSceneGraph GUI
- `DART_BUILD_GUI_RAYLIB` - Enable experimental Raylib integration (builds `raylib` example)
- `DART_BUILD_GUI_VSG` - Enable VulkanSceneGraph visualization (requires Vulkan SDK; used by `collision_viz` example)
- `DART_BUILD_TESTS` - Build C++ tests (wraps the standard `BUILD_TESTING` option)
- `DART_BUILD_EXAMPLES` - Build the GUI-based example targets (defaults to `ON`; automatically skip when disabled or when `DART_BUILD_GUI=OFF`)
- `DART_BUILD_TUTORIALS` - Build the GUI-based tutorial targets (defaults to `ON`; automatically skip when disabled or when `DART_BUILD_GUI=OFF`)

### Platform-Specific Configuration Examples

```bash
# Unix Makefiles
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release

# Visual Studio 2017 (multi-config; pick configuration at build-time)
cmake .. -G "Visual Studio 15 2017" -A x64

# Xcode (multi-config; pick configuration at build-time)
cmake .. -G "Xcode"
```

## Building from Command Line

You can build DART from the command line regardless of the configured generator:

1. Navigate to the build directory:

   ```bash
   cd build
   ```

2. Run the build command:

   ```bash
   cmake --build . [--target <target>] [-j<num_cores>]
   ```

### Build Targets

DART provides several CMake targets:

- **ALL**: Builds all targets, including tests, examples, and tutorials
- **all**: Builds core targets only (no tests/examples/tutorials)
- **tests**: Builds all tests
- **test**: Runs tests (requires building tests first)
- **tests_and_run**: Builds and runs tests
- **examples**: Builds all examples
- **tutorials**: Builds all tutorials
- **benchmarks**: Builds all benchmarks
- **view_docs**: Builds documentation and opens in browser
- **install**: Installs the project
- **dartpy**: Builds Python bindings (prefer using pip instead)
- **pytest**: Runs Python tests
- **coverage**: Runs tests and generates coverage report
- **coverage_html**: Generates HTML coverage report
- **coverage_view**: Generates and opens HTML coverage report

> **Note:** This list may not be exhaustive. Refer to the main `CMakeLists.txt` for the complete list.

## Building from IDEs

If you configured the build for IDEs (Visual Studio, Xcode, etc.), you can build DART directly from the IDE. Refer to your IDE's documentation for specific instructions.

## Building dartpy (Python Bindings)

### Using pip (Recommended)

The easiest way to install dartpy is via pip:

```bash
pip install dartpy -U
```

### Building from Source

To build dartpy from source, enable the Python bindings when configuring CMake:

```bash
cmake -DDART_BUILD_DARTPY=ON ..
```

Then build using the standard CMake commands or the `dartpy` target:

```bash
cmake --build . --target dartpy
```

For more details on Python bindings architecture, see [python-bindings.md](python-bindings.md).

## Signals to Look For

| Command           | Success                                    | Failure                            |
| ----------------- | ------------------------------------------ | ---------------------------------- |
| `pixi run config` | "Configuring done" with no CMake errors    | CMake error messages in output     |
| `pixi run build`  | "Build finished" or exit code 0            | Compiler errors, linker errors     |
| `pixi run test`   | "100% tests passed" or all tests green     | Test failure summary with count    |
| `cmake --build .` | No errors, successful compilation messages | Error messages with file:line info |

## See Also

- [build-system.md](build-system.md) - Deep dive into DART's CMake architecture
- [architecture.md](architecture.md) - Core simulation engine architecture
- [CONTRIBUTING.md](../../CONTRIBUTING.md) - Contribution guidelines and code style
