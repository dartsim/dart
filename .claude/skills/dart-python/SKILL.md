---
name: dart-python
description: DART Python bindings (dartpy) - nanobind, wheel building, API patterns
---

# DART Python Bindings (dartpy)

Load this skill when working with Python bindings or dartpy.

## Quick Start

```python
import dartpy as dart
import numpy as np

world = dart.World()
parser = dart.io.UrdfParser()
robot = parser.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.addSkeleton(robot)

for _ in range(100):
    world.step()
    print(robot.getPositions())
```

## Full Documentation

For complete Python bindings guide: `docs/onboarding/python-bindings.md`

## Module Structure

```
dartpy/
├── io            # File parsers (URDF, SDF, SKEL, MJCF)
└── gui           # 3D visualization (OSG + ImGui)
```

Core classes promoted to top-level `dartpy`:
- `World`, `Skeleton`, `BodyNode`, `Joint`
- Collision, simulation, math utilities

## Development Commands

```bash
# Build for development
pixi run build-py-dev

# Lint before committing
pixi run lint

# Generate type stubs
pixi run generate-stubs
```

## Key Patterns

### Visualization with Custom Logic

```python
class MyWorldNode(dart.gui.RealTimeWorldNode):
    def customPreStep(self):
        # Apply controls before physics step
        pass

    def customPostStep(self):
        # Log data after physics step
        pass

viewer = dart.gui.Viewer()
viewer.addWorldNode(MyWorldNode(world))
viewer.run()
```

### NumPy Integration

```python
# NumPy arrays automatically convert to Eigen types
skel.set_positions(np.array([0.1, 0.2, 0.3]))

# Eigen types automatically convert to NumPy arrays
positions = skel.get_positions()  # Returns ndarray

# Jacobian computation
ee = robot.getBodyNode("end_effector")
J = ee.getLinearJacobian()
forces = np.matmul(J.T, target_force)
```

## Naming Convention

- **snake_case** is preferred (e.g., `get_positions()`)
- **camelCase** still works but emits `DeprecationWarning`
- Silence warnings: `DARTPY_WARN_ON_CAMELCASE=0`

## Wheel Building

```bash
# Build wheels
pixi run -e py312-wheel wheel-build
pixi run -e py313-wheel wheel-build

# Linux: repair with auditwheel
pixi run -e py312-wheel wheel-repair

# Verify and test
pixi run -e py312-wheel wheel-verify wheel-test
```

## Version Management

Source of truth: `package.xml`

```xml
<version>7.0.0</version>          <!-- Release -->
<version>7.0.0.dev0</version>     <!-- Development -->
```

## Key Files

- Package config: `pyproject.toml`
- Build system: `python/dartpy/CMakeLists.txt`
- Type stubs: `python/stubs/dartpy/`
- Examples: `python/examples/`

## Gotchas

- GUI bindings require `DART_BUILD_GUI=ON`
- GIL released for long-running C++ calls
- Convert Python args to C++ before releasing GIL
- MeshShape uses `TriMesh<double>`, not aiScene
