---
name: dart-python
description: "DART Python: dartpy bindings, pybind11, wheels, and API patterns"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-python/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

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

This release branch ships Pixi-managed wheel tasks. The core tasks are
`wheel-build-core`, `wheel-repair-linux-core`, `wheel-repair-macos-core`,
`wheel-repair-windows-core`, `wheel-verify-core`, and `wheel-test-core`;
the per-Python environments `py310-wheel` through `py313-wheel` expose
`wheel-build`, `wheel-repair`, `wheel-verify`, and `wheel-test` on top of
them. CI runs them in `.github/workflows/publish_dartpy.yml` (for example
`pixi run -e py313-wheel wheel-build`). See
`docs/onboarding/python-bindings.md` for the wheel workflow overview.

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
