---
name: dart-python
description: "DART Python: dartpy bindings, nanobind, wheels, and API patterns"
---

# DART Python Bindings (dartpy)

Load this skill when working with Python bindings or dartpy.

When a binding exposes or changes model/scene loading, dynamics,
collision/contact/constraints, simulation stepping, GUI/Filament output, or a visual example, also
load `dart-verify-sim`. Pair a focused Python text/behavior oracle with an
assessed, claim-tied Filament capture; document a visual exception when capture is
unavailable or not applicable.

## Quick Start

```python
import dartpy as dart

world = dart.simulation.World()
loader = dart.utils.DartLoader()
skel = loader.parseSkeleton("dart://sample/urdf/six_axis_arm/six_axis_arm.urdf")
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
pixi run test-py         # Run Python tests that need no graphics device
pixi run test-gui        # Run dartpy.gui rendering and integration tests
```

## Wheel Building

This development branch ships Pixi-managed wheel tasks. The core tasks are
`wheel-build-core`, `wheel-repair-linux-core`, `wheel-repair-macos-core`,
`wheel-repair-windows-core`, `wheel-verify-core`, and `wheel-test-core`;
the `py312-wheel` environment exposes `wheel-build`, `wheel-repair`,
`wheel-verify`, and `wheel-test` on top of them, and `py313-wheel` exposes
`wheel-test-abi3`. Every wheel is `cp312-abi3`, built by scikit-build-core
from `pyproject.toml`. CI runs them in
`.github/workflows/publish_dartpy.yml` (for example
`pixi run -e py312-wheel wheel-build`). See
`docs/onboarding/python-bindings.md` for the wheel workflow overview.

## Key Patterns

- nanobind is dartpy's only binder on `main`, under `python/dartpy/`.
  Use the DART 6.20 compatibility notes in the owner guide.
- Until the PLAN-624 rename lands, follow the current binding names in
  `python/examples` and `python/tests`; bind new DART 7 APIs directly in the
  DART 7 Python style (snake_case, properties, keyword-only options) from the
  PLAN-624 design doc.
- NumPy arrays auto-convert to Eigen types
- `dartpy.gui` always exists: dartpy requires `DART_BUILD_IO` and
  `DART_BUILD_GUI`. GUI callers use the Filament-backed `dart.gui.Viewer` and
  related neutral classes; import and viewer construction need no display.

## Key Files

- Package config: `pyproject.toml` (scikit-build-core) and the PEP 517
  wrapper `scripts/pep517/dartpy_backend.py`
- Package files: `python/dartpy/package/` (`__init__.py`, `py.typed`,
  `experimental/`)
- Binding dependencies: `python/CMakeLists.txt`
- Binding build system: `python/dartpy/CMakeLists.txt`
