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

This development branch ships Pixi-managed wheel tasks. The core tasks are
`wheel-build-core`, `wheel-repair-linux-core`, `wheel-repair-macos-core`,
`wheel-repair-windows-core`, `wheel-verify-core`, and `wheel-test-core`;
the per-Python environments `py312-wheel` through `py314-wheel` expose
`wheel-build`, `wheel-repair`, `wheel-verify`, and `wheel-test` on top of
them. CI runs them in `.github/workflows/publish_dartpy.yml` (for example
`pixi run -e py314-wheel wheel-build`). See
`docs/onboarding/python-bindings.md` for the wheel workflow overview.

cp315 wheels cover Linux x86_64 and aarch64 through
`python3 scripts/wheel_linux.py cp315` using Docker's manylinux toolchain.
macOS arm64 and Windows x64 cp315 wheels follow once conda-forge's bullet-cpp
has Python 3.15 builds
([conda-forge/bullet-feedstock#70](https://github.com/conda-forge/bullet-feedstock/pull/70)).

## Key Patterns

- DART 6 uses pybind11 under `python/dartpy/`; do not import nanobind
  guidance.
- Follow the existing DART 6 camelCase binding names used in `python/examples`
  and `python/tests`.
- NumPy arrays auto-convert to Eigen types
- dartpy requires the OSG GUI library; keep `DART_BUILD_GUI_OSG=ON` (the default)

## Key Files

- Package config: `pyproject.toml`
- Build system: `python/dartpy/CMakeLists.txt`
