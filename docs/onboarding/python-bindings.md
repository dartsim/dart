# DART Python Bindings (dartpy)

## Design Decisions

### Why nanobind?

**Choice**: nanobind for C++/Python bindings

**Rationale**:

- Header-only library with a CMake-first workflow
- Eigen ↔ NumPy integration via `nanobind/eigen/*` (used throughout `python/dartpy/`)
- Modern C++17+ support (DART is built as C++20)
- Designed for high-performance bindings

### Why scikit-build-core?

**Choice**: scikit-build-core (not setuptools or traditional setup.py)

**Rationale**:

- Modern PEP 517/518 compliant build system
- Native CMake integration (DART is already CMake-based)
- Supports editable installs (`pip install -e .`)
- Cleaner than deprecated setup.py approach
- Better wheel building control

**Configuration**: See `pyproject.toml` for build settings and version extraction from `package.xml`

### Why pixi for wheel building?

**Choice**: pixi-based wheel building (not cibuildwheel + Docker)

**Rationale**:

- **No Docker maintenance**: No custom container images to maintain
- **Local reproducibility**: Same commands work locally and in CI
- **Unified tooling**: pixi handles both development and distribution
- **Cross-platform**: Consistent environment via conda-forge

**Result**: Simpler, faster builds with better developer experience

## Architecture

### Module Structure

dartpy now flattens most symbols onto the top-level package to avoid deep
namespaces:

```
dartpy/
├── io            # File parsers (URDF, SDF, SKEL, MJCF)
└── gui           # 3D visualization (OpenSceneGraph + ImGui)
```

- Core classes/functions (dynamics, collision, math, simulation, constraint,
  optimizer) are promoted onto `dartpy` directly.
- Legacy submodules remain importable in DART 7.x but will be removed in DART
  8.0. Toggle deprecation handling with `DARTPY_WARN_ON_LEGACY_MODULES` or
  `DARTPY_ENABLE_LEGACY_MODULES`.

**Source**: See `python/dartpy/` directory for module implementations

### Eigen ↔ NumPy Integration

**Key Design**: nanobind's Eigen support enables seamless conversion

**Implementation**: See `python/dartpy/math/geometry.cpp` for representative usage (`#include <nanobind/eigen/dense.h>`).

**Features**:

- Zero-copy conversions where possible
- Automatic bidirectional conversion
- Support for Eigen::Isometry3 (4x4 transforms)
- Quaternion support

**Usage**:

```python
import dartpy as dart
import numpy as np

# NumPy arrays automatically convert to Eigen types
skel.set_positions(np.array([0.1, 0.2, 0.3]))

# Eigen types automatically convert to NumPy arrays
positions = skel.get_positions()  # Returns ndarray
```

### OSG Bindings Design

GUI bindings are built only when `DART_BUILD_GUI=ON`. The build wires this up by
conditionally appending the GUI sources in `python/dartpy/CMakeLists.txt`.

Project policy: official dartpy wheels build with GUI enabled, so `dartpy.gui`
is expected to be available in release artifacts and CI. For local headless-only
builds you can disable GUI, but some examples/tutorials will not run.

## Pythonic Naming Transition

- All camelCase bindings now receive snake_case aliases at import time (runtime shim lives in `python/dartpy/_naming.py`)
- camelCase still works but emits a one-time `DeprecationWarning` per symbol by default; set `DARTPY_WARN_ON_CAMELCASE=0` to silence
- Turn the shim off entirely with `DARTPY_ENABLE_SNAKE_CASE=0` (useful for bisecting)
- Prefer snake_case in new code; ship a codemod/release note alongside the next major to help users update usages

## Installation Methods

### For End Users

```bash
# PyPI wheels (pre-built, includes OSG)
pip install dartpy

# conda-forge
conda install dartpy -c conda-forge
```

### For Developers

**See**: `docs/readthedocs/dartpy/developer_guide/build.rst` for detailed instructions

**Quick options**:

- **Editable install**: `pip install -e . -v --no-build-isolation`
- **Using pixi**: `pixi run build-py-dev`
- **Traditional CMake**: Configure with `-DDART_BUILD_DARTPY=ON`, then build and install

## PyPI Publishing

### Build Infrastructure

Wheels are built using **pixi** environments defined in `pixi.toml`:

- Features: `py312-wheel`, `py313-wheel`
- Platform-specific build tasks
- OSG enabled on all platforms

### Publishing Workflow

```bash
# Build
pixi run -e py312-wheel wheel-build
pixi run -e py313-wheel wheel-build

# Repair (Linux only - run auditwheel)
pixi run -e py312-wheel wheel-repair
pixi run -e py313-wheel wheel-repair

# Verify and test
pixi run -e py312-wheel wheel-verify wheel-test
pixi run -e py313-wheel wheel-verify wheel-test

# Publish
pixi run -e py312-wheel wheel-upload
pixi run -e py313-wheel wheel-upload
```

### Version Management

**Source of truth**: `package.xml`

```xml
<version>7.0.0</version>          <!-- Release -->
<version>7.0.0.dev0</version>     <!-- Development -->
```

**Extraction**: `pyproject.toml` regex automatically extracts version during build

**Publishing policy** (to avoid PyPI 10GiB limit):

- **Release versions** (`7.0.0`) → Production PyPI
- **Dev versions** (`7.0.0.dev0`) → Test PyPI only (use `wheel-upload-test`)
- Manual cleanup: Delete old versions via PyPI web interface when needed

### CI/CD

**Workflow**: `.github/workflows/publish_dartpy.yml`

- **Every commit**: Build → Repair → Verify → Test → Upload artifacts
- **Version tags** (`v*.*.*`): All above + Publish to PyPI

## Key Patterns

### Pattern 1: Basic Simulation

```python
import dartpy as dart

world = dart.simulation.World()
parser = dart.utils.UrdfParser()
robot = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.addSkeleton(robot)

for _ in range(100):
    world.step()
```

### Pattern 2: Visualization with Custom Logic

```python
class MyWorldNode(dart.gui.RealTimeWorldNode):
    def customPreStep(self):
        # Apply controls before physics step
        pass

    def customPostStep(self):
        # Log data after physics step
        pass

world = dart.simulation.World()
node = MyWorldNode(world)

viewer = dart.gui.Viewer()
viewer.addWorldNode(node)
viewer.run()
```

### Pattern 3: Jacobian Computation

```python
import numpy as np

ee = robot.getBodyNode("end_effector")
J = ee.getLinearJacobian()
forces = np.matmul(J.T, target_force)
robot.setForces(forces)
```

## Testing

**Scripts**:

- `scripts/test_wheel.py` - Test wheel in isolated environment
- `scripts/verify_version.py` - Verify version consistency

**Examples**: See `python/examples/` for comprehensive usage examples

## Type Stubs

**Location**: `python/stubs/dartpy/`

**Purpose**: IDE autocomplete and type checking

**Generation**: Can be regenerated with `pixi run generate-stubs`

## References

- **Package configuration**: `pyproject.toml`
- **Build system**: `python/dartpy/CMakeLists.txt`
- **pixi configuration**: `pixi.toml` (features: `py312-wheel`, `py313-wheel`)
- **Examples**: `python/examples/`
- **Developer guide**: `docs/readthedocs/dartpy/developer_guide/build.rst`
