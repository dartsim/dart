# python/

Agent guidelines for Python bindings (dartpy).

## Overview

Complete Python API via nanobind with NumPy integration.

## Directory Structure

| Directory    | Purpose                       |
| ------------ | ----------------------------- |
| `dartpy/`    | nanobind binding source files |
| `examples/`  | Python example scripts        |
| `tutorials/` | Python tutorials              |
| `tests/`     | Python test suite (pytest)    |

## Module Structure

```python
import dartpy as dart

# Core modules (snake_case API)
dart.World()
dart.Skeleton("name")
dart.io.read_skeleton()  # Unified loader
dart.gui                 # Backend-hidden GUI descriptors and helpers
```

## Code Patterns

- Eigen ↔ NumPy automatic conversion
- Use `dart.io.read_skeleton()` for model loading
- Construct directly: `dart.Skeleton()` / `dart.Skeleton("name")` and `dart.World()` (value types; dartpy exposes no `create()` factories, and new C++ `World::create()` usages are gated by `pixi run check-dart7-world-promotion-blockers`)
- Keep dartpy GUI APIs aligned with DART-owned C++ concepts.
  `dartpy.gui` exposes the constrained descriptor, debug, picking, and run-loop
  bridge.
- Do not expose Filament, GLFW, Dear ImGui, OpenGL, Vulkan, Metal, OSG, or
  Raylib implementation types through new Python-facing contracts.

## Building

```bash
pixi run build           # Build C++ and Python
pixi run test-py         # Run Python tests
```

## Testing

- Use pytest for all Python tests
- Run: `pixi run test-py` or `pytest python/tests/`
- Test files: `test_<module>.py`

## See Also

- @docs/onboarding/python-bindings.md - Binding architecture
- @docs/onboarding/gui-rendering.md - Filament GUI architecture and workflow
- @python/examples/README.md - Example index
- @pyproject.toml - Python package configuration
