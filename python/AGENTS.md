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
dart.Skeleton.create()
dart.io.read_skeleton()  # Unified loader
dart.gui.Viewer()        # Visualization
```

## Code Patterns

- Eigen ↔ NumPy automatic conversion
- Use `dart.io.read_skeleton()` for model loading
- Factory methods: `Skeleton.create()`, `World.create()`

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
- @python/examples/README.md - Example index
- @pyproject.toml - Python package configuration
