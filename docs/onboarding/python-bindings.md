# DART Python Bindings (dartpy)

## Start here next time

- Local build/test workflow: [building.md](building.md) and [testing.md](testing.md)
- CI monitoring and expectations: [ci-cd.md](ci-cd.md)
- Dartpy developer build guide: [docs/readthedocs/dartpy/developer_guide/build.rst](../readthedocs/dartpy/developer_guide/build.rst)

## Design Decisions

### Why nanobind?

**Choice**: nanobind for C++/Python bindings

**Rationale**:

- Header-only library with a CMake-first workflow
- Eigen ↔ NumPy integration via `nanobind/eigen/*` (used throughout `python/dartpy/`)
- Modern C++ support (DART is built as C++23)
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
├── simulation    # DART 7 World and simulation object model
├── io            # File parsers (URDF, SDF, SKEL, MJCF)
└── gui           # Filament-backed 3D visualization descriptors and helpers
```

- Core classes/functions (dynamics, collision, math, simulation, constraint,
  optimizer) are promoted onto `dartpy` directly.
- Legacy submodules remain available on `release-6.*` support branches. Main keeps
  migration warnings only while the DART 7 clean-break gates are being closed;
  the DART 7 public contract should not carry the legacy dartpy 6 API surface.
  Toggle deprecation handling with `DARTPY_WARN_ON_LEGACY_MODULES` or
  `DARTPY_ENABLE_LEGACY_MODULES` while that temporary surface exists.

**Source**: See `python/dartpy/` directory for module implementations

### DART 7 World Bindings And Transition

The ECS-backed world is the DART 7 simulation API target. Python has already
promoted the common path: `dartpy.simulation.World` is the canonical owner,
`dartpy.World` is the same class object after `import dartpy as dart`, generated
stubs no longer publish `dartpy.simulation_experimental`, and the classic
DART 6 world is quarantined as `dartpy.gui.RenderWorld` for rendering plumbing.
The Python `dartpy.io` parser surface no longer exposes whole-world
`read_world` / `parse_world` entrypoints; load skeletons and add them to the
DART 7 world instead. Keep that shape guarded with
`pixi run check-dartpy-import-layout`, stub generation, Python tests, and wheel
import smokes.

The C++ surface now uses the final `dart::simulation` namespace and
`dart-simulation` target. Bindings for the promoted Python module should
continue to expose only public wrapper types and must not expose EnTT, `comps`,
or other ECS internals. The DART 7 path is to keep release-6 branches as the
parity reference and avoid carrying the DART 6 world beside the promoted API.

Durable API-shape rationale for this surface lives in
[simulation_python_api.md](../design/simulation_python_api.md).

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

### Binding Conventions

- Prefer shared numeric conversion helpers for Python inputs (sequences and NumPy) so bindings behave consistently across modules.
- Release the GIL around long-running, non-callback C++ calls (e.g., stepping and collision queries) using `nb::call_guard<nb::gil_scoped_release>()` where safe.
- Convert Python arguments to C++ types before releasing the GIL; nanobind casting and NumPy access require holding the GIL.
- When the C++ API stores raw pointers, add explicit keep-alive/shared-ownership patterns to prevent lifetime bugs in Python.
- Treat dartpy as the public user-facing API, not a mirror of all C++ internals.
  Do not include `dart/**/detail/**`, `dart/**/internal/**`, or
  `dart/simulation/comps/**` headers.
- Do not bind `dart::...::detail` or `dart::...::internal` types, aliases, or
  data members. A public C++ alias whose underlying type is in `detail` still
  needs an allowlist entry until the C++ API provides a public wrapper.
- Do not expose deprecated compatibility shims unless Python users need a
  migration window; add a deprecation warning, test, stub/doc update, and
  removal condition for each shim.
- Prefer explicit public accessors or Pythonic value wrappers when C++ stores
  data in an internal base class.
- Keep `scripts/check_api_boundaries_allowlist.txt` entries temporary: each
  entry needs a replacement, `remove_by` condition, tracking phase or issue, and
  reason. See [api-boundaries.md](api-boundaries.md).

### GUI Bindings Design

GUI bindings are built only when `DART_BUILD_GUI=ON`. The build wires this up by
conditionally appending the GUI sources in `python/dartpy/CMakeLists.txt`.

Project policy: official dartpy wheels build with GUI enabled, so `dartpy.gui`
is expected to be available in release artifacts and CI. For local headless-only
builds you can disable GUI, but some examples/tutorials will not run.

The maintained `dartpy.gui` surface is built around DART-owned concepts such as
scene descriptors, renderable descriptors, picking, camera helpers, debug
draws, screenshot options, and viewer lifecycle state. Filament, GLFW, Dear
ImGui, OpenGL, Vulkan, Metal, OSG, and Raylib implementation types stay out of
public Python-facing contracts. See [gui-rendering.md](gui-rendering.md) for
the promoted GUI boundary.

## Fast Iteration Loop

Smallest repeatable local loop for dartpy binding changes.

Suggested (Unverified):

```bash
pixi run lint
pixi run build-py-dev
```

Signals to look for:

- Lint completes without errors; auto-formatting changes are reviewed before committing.
- The dartpy build finishes without nanobind compile errors.

## Gotchas

- `ImGuiKey` can be a typedef to `int` in some ImGui releases; nanobind `enum_` requires an enum type, so bind a shim enum and cast back to `ImGuiKey` for ImGui calls.

## Pythonic Naming Transition

- All camelCase bindings now receive snake_case aliases at import time (runtime shim lives in `python/dartpy/_naming.py`)
- camelCase still works but emits a one-time `DeprecationWarning` per symbol by default; set `DARTPY_WARN_ON_CAMELCASE=0` to silence
- Turn the shim off entirely with `DARTPY_ENABLE_SNAKE_CASE=0` (useful for bisecting)
- Prefer snake_case in new code; ship a codemod/release note alongside the next major to help users update usages

## Installation Methods

### For End Users

```bash
# PyPI wheels (pre-built, includes the Filament-backed GUI)
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
- Filament-backed GUI enabled on all official wheel platforms through
  `DART_BUILD_GUI=ON`

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

Run Linux wheel import tests after `wheel-repair`, not against the raw
`linux_x86_64` wheel produced by `wheel-build`. The raw wheel can still depend
on shared libraries from the build environment, while the repaired
`manylinux_*` wheel is the artifact that should import in a fresh virtual
environment and prove `dartpy.gui` is present.

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

world = dart.World()
parser = dart.io.UrdfParser()
robot = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
multibody = dart.add_skeleton(world, robot)

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

world = dart.World()
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

**Examples**: See `python/examples/README.md` for the category index and
`python/examples/` for the full set of examples.

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
- **Examples**: `python/examples/README.md` (index) and `python/examples/`
- **Developer guide**: `docs/readthedocs/dartpy/developer_guide/build.rst`
