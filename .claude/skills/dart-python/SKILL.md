---
name: dart-python
description: "DART Python: dartpy bindings, nanobind, wheels, and API patterns"
---

# DART Python Bindings (dartpy)

Load this skill when working with Python bindings or dartpy.

When a binding exposes or changes 3D structure or behavior, also load
`dart-verify-sim`: a focused Python text oracle plus an assessed claim-tied
capture, or a recorded exception when rendering is unavailable.

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
pixi run -e py314-wheel wheel-build
pixi run -e py314-wheel wheel-repair  # Linux only
pixi run -e py314-wheel wheel-test
```

## Key Patterns

- **snake_case** preferred (camelCase emits DeprecationWarning)
- NumPy arrays auto-convert to Eigen types
- GUI requires `DART_BUILD_GUI=ON`

## Key Files

- Package config: `pyproject.toml`
- Build system: `python/dartpy/CMakeLists.txt`
- Type stubs: `python/stubs/dartpy/`
