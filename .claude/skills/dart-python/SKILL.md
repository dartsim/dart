---
name: dart-python
description: "DART Python: dartpy bindings, pybind11, wheels, and API patterns"
---

# DART Python Bindings (dartpy)

Load this skill when working with Python bindings or dartpy.

When a binding exposes or changes model/scene loading, dynamics,
collision/contact/constraints, simulation stepping, GUI/OSG output, or a visual example, also
load `dart-verify-sim`. Pair a focused Python text/behavior oracle with an
assessed, claim-tied OSG capture; document a visual exception when capture is
unavailable or not applicable.

## Quick Start

```python
import dartpy as dart

world = dart.simulation.World()
loader = dart.utils.DartLoader()
skel = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf")
world.addSkeleton(skel)

for _ in range(100):
    world.step()
```

## Full Documentation

For complete Python bindings guide: `docs/onboarding/python-bindings.md`

For current examples and test patterns: `python/examples/` and `python/tests/`

## Quick Commands

```bash
pixi run build-py-dev    # Build for development
pixi run test-py         # Run Python tests
```

## Wheel Building

This release branch does not define local Pixi wheel tasks. Before changing
wheel packaging, inspect the current CI/package workflow and add or document
release-branch wheel tasks in the same PR that introduces them.

## Key Patterns

- DART 6.20 uses pybind11 under `python/dartpy/`; do not import DART 7
  nanobind guidance.
- Follow the existing DART 6 camelCase binding names used in `python/examples`
  and `python/tests`.
- NumPy arrays auto-convert to Eigen types
- GUI requires `DART_BUILD_GUI=ON`

## Key Files

- Package config: `pyproject.toml`
- Build system: `python/dartpy/CMakeLists.txt`
