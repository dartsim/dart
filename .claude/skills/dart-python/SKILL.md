---
name: dart-python
description: DART Python bindings (dartpy) - nanobind, wheel building, API patterns
---

# DART Python Bindings (dartpy)

Load this skill when working with Python bindings or dartpy.

## Quick Start

```python
import dartpy as dart

world = dart.World()
skel = dart.io.read_skeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.add_skeleton(skel)

for _ in range(100):
    world.step()
```

## Full Documentation

For complete Python bindings guide: `docs/onboarding/python-bindings.md`

For module-specific details: `python/AGENTS.md`

## Quick Commands

```bash
pixi run build-py-dev    # Build for development
pixi run test-py         # Run Python tests
pixi run generate-stubs  # Generate type stubs
```

## Wheel Building

```bash
pixi run -e py312-wheel wheel-build
pixi run -e py312-wheel wheel-repair  # Linux only
pixi run -e py312-wheel wheel-test
```

## Key Patterns

- **snake_case** preferred (camelCase emits DeprecationWarning)
- NumPy arrays auto-convert to Eigen types
- GUI requires `DART_BUILD_GUI=ON`

## Key Files

- Package config: `pyproject.toml`
- Build system: `python/dartpy/CMakeLists.txt`
- Type stubs: `python/stubs/dartpy/`
