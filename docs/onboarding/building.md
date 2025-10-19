# Building DART from Source

This guide describes how to build DART from source, including both the C++ library and Python bindings (dartpy).

## Supported Environments

DART is supported on the following operating systems and compilers:

| Operating System      | Compiler              |
|-----------------------|-----------------------|
| Ubuntu 22.04 or later | GCC 11.2 or later     |
| Windows 2022 or later | Visual Studio 2022    |
| macOS 13 or later     | Clang 13 or later     |

## Prerequisites

Before you can build DART, you'll need to install the required and optional dependencies. The required dependencies are the minimum set needed to build DART, while the optional dependencies enable additional features.

> **Note:** The dependencies and installation steps are subject to change. Please report any issues you encounter and contribute to keeping the instructions up-to-date.

### Ubuntu

Install required dependencies using `apt`:

```bash
sudo apt install \
  build-essential cmake pkg-config git libassimp-dev \
  libeigen3-dev libfcl-dev libfmt-dev
```

Install optional dependencies:

```bash
sudo apt install \
  coinor-libipopt-dev freeglut3-dev libxi-dev libxmu-dev libbullet-dev \
  libtinyxml2-dev liburdfdom-dev liburdfdom-headers-dev \
  libopenscenegraph-dev libnlopt-cxx-dev liboctomap-dev libode-dev \
  libspdlog-dev libyaml-cpp-dev ocl-icd-opencl-dev opencl-headers \
  opencl-clhpp-headers
```

### macOS

Install required dependencies using `brew`:

```bash
brew install assimp cmake eigen fmt fcl
```

Install optional dependencies:

```bash
brew install bullet freeglut ipopt nlopt octomap ode \
  open-scene-graph --HEAD \
  spdlog tinyxml2 urdfdom yaml-cpp
```

### Windows

Install required dependencies using `vcpkg`:

```bash
vcpkg install --triplet x64-windows assimp eigen3 fcl fmt spdlog
```

Install optional dependencies:

```bash
vcpkg install --triplet x64-windows \
  assimp eigen3 fcl fmt spdlog bullet3 freeglut glfw3 nlopt ode \
  opencl opengl osg pagmo2 pybind11 tinyxml2 urdfdom yaml-cpp
```

### Arch Linux (experimental)

Install required dependencies using `yay`:

```bash
yay -S assimp cmake eigen fcl fmt
```

Install optional dependencies:

```bash
yay -S \
  bullet coin-or-ipopt freeglut nlopt octomap ode opencl-clhpp \
  opencl-headers opencl-icd-loader openscenegraph pagmo spdlog tinyxml2 \
  urdfdom pybind11
```

### FreeBSD (experimental)

TODO

## Dependency Reference

Summary of dependencies required to build DART:

| Dependency     | Required | Type    | Min. Version | Notes |
|----------------|----------|---------|--------------|-------|
| CMake          | Yes      | Build   | 3.22.1       |       |
| Assimp         | Yes      | Runtime | 5.2.2        |       |
| Eigen          | Yes      | Runtime | 3.4.0        |       |
| FCL            | Yes      | Runtime | 0.7.0        |       |
| fmt            | Yes      | Runtime | 8.1.1        |       |
| Bullet         | No       | Runtime | 3.06         |       |
| Ipopt          | No       | Runtime | 3.11.9       |       |
| Octomap        | No       | Runtime | 1.9.7        |       |
| ODE            | No       | Runtime | 0.16.2       |       |
| Pagmo          | No       | Runtime | 2.18.0       |       |
| spdlog         | No       | Runtime | 1.9.2        |       |
| tinyxml2       | No       | Runtime | 9.0.0        |       |
| urdfdom        | No       | Runtime | 3.0.1        |       |
| OpenSceneGraph | No       | Runtime | 3.6.5        |       |

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

DART provides several CMake options to customize the build:

| Option                    | Default Value | Description                              |
|---------------------------|---------------|------------------------------------------|
| CMAKE_BUILD_TYPE          | Release       | Specifies the build type                 |
| DART_ENABLE_SIMD          | ON            | Enables use of SIMD instructions         |
| DART_BUILD_DARTPY         | OFF           | Build Python bindings                    |
| DART_BUILD_GUI_OSG        | ON            | Build OpenSceneGraph GUI                 |

> **Note:** This list may not be exhaustive. Refer to the main `CMakeLists.txt` file for the complete list of options.

### Platform-Specific Configuration Examples

```bash
# Unix Makefiles
cmake .. -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release

# Visual Studio 2017
cmake .. -G "Visual Studio 15 2017" -A x64 -DCMAKE_BUILD_TYPE=Release

# Xcode
cmake .. -G "Xcode" -DCMAKE_BUILD_TYPE=Release
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

## See Also

- [build-system.md](build-system.md) - Deep dive into DART's CMake architecture
- [architecture.md](architecture.md) - Core simulation engine architecture
- [CONTRIBUTING.md](../../CONTRIBUTING.md) - Contribution guidelines and code style
