# DART Python Bindings (dartpy)

## Design Decisions

### Why pybind11?

**Choice**: pybind11 for C++/Python bindings

**Rationale**:
- Header-only library, no external dependencies
- Seamless Eigen ↔ NumPy integration with custom type casters
- Automatic reference counting and lifetime management
- Modern C++17 features support
- Excellent performance (zero-copy when possible)

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
- **No Docker maintenance**: No custom manylinux images to maintain
- **Local reproducibility**: Same commands work locally and in CI
- **Unified tooling**: pixi handles both development and distribution
- **Cross-platform**: Consistent environment via conda-forge

**Result**: Simpler, faster builds with better developer experience

## Architecture

### Module Structure

dartpy organizes DART's C++ API into Python modules:

```
dartpy/
├── math          # Eigen integration, geometry utilities
├── dynamics      # Skeletons, BodyNodes, Joints
├── collision     # Collision detection backends
├── constraint    # Constraint solving
├── simulation    # World simulation
├── utils         # File parsers (URDF, SDF, SKEL, MJCF)
└── gui.osg       # 3D visualization (OpenSceneGraph + ImGui)
```

**Source**: See `python/dartpy/` directory for module implementations

### Eigen ↔ NumPy Integration

**Key Design**: Custom pybind11 type casters enable seamless conversion

**Implementation**: `python/dartpy/eigen_geometry_pybind.h`

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
skel.setPositions(np.array([0.1, 0.2, 0.3]))

# Eigen types automatically convert to NumPy arrays
positions = skel.getPositions()  # Returns ndarray
```

### OSG Bindings Design

**Issue**: `dartpy.gui.osg` was not available in wheels despite `DART_BUILD_GUI_OSG=ON`

**Root Cause**: Python bindings checked for undefined `HAVE_DART_GUI_OSG` preprocessor macro

**Solution**:
1. Use `DART_BUILD_GUI_OSG` directly in `python/dartpy/gui/module.cpp`:
   ```cpp
   #if DART_BUILD_GUI_OSG
     // Bind OSG module
   #endif
   ```

2. Pass as compile definition in `python/dartpy/CMakeLists.txt`:
   ```cmake
   if(DART_BUILD_GUI_OSG)
     target_compile_definitions(${pybind_module} PRIVATE DART_BUILD_GUI_OSG=1)
   endif()
   ```

**Result**: OSG now works on all platforms where OpenSceneGraph is available (Linux, macOS, Windows via conda-forge)

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

# Repair (Linux only - convert to manylinux)
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
loader = dart.utils.DartLoader()
robot = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.addSkeleton(robot)

for _ in range(100):
    world.step()
```

### Pattern 2: Visualization with Custom Logic
```python
class MyWorldNode(dart.gui.osg.RealTimeWorldNode):
    def customPreStep(self):
        # Apply controls before physics step
        pass

    def customPostStep(self):
        # Log data after physics step
        pass

world = dart.simulation.World()
node = MyWorldNode(world)

viewer = dart.gui.osg.Viewer()
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

## MeshShape and TriMesh Bindings

**Design Decision:** Python bindings expose only `dart::math::TriMesh<double>` for mesh operations, not Assimp's `aiScene*` types. This ensures Python users work with clean, format-agnostic mesh data. The MeshShape bindings provide TriMesh-based constructors and `getTriMesh()` accessor. Deprecated aiScene-based constructors were intentionally not exposed in Python bindings (breaking change allowed for dartpy). See `python/dartpy/math/TriMesh.cpp` and `python/dartpy/dynamics/Shape.cpp` for bindings implementation.

## References

- **Package configuration**: `pyproject.toml`
- **Build system**: `python/dartpy/CMakeLists.txt`
- **pixi configuration**: `pixi.toml` (features: `py312-wheel`, `py313-wheel`)
- **Examples**: `python/examples/`
- **Developer guide**: `docs/readthedocs/dartpy/developer_guide/build.rst`
