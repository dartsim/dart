# dartpy Packaging Refactor

**Status**: ✅ Complete
**Completed**: 2025-01-19

## Overview

Modernized dartpy packaging to support PyPI/conda-forge distribution with local verification tools.

## Key Changes

### Build System
- **Fixed CMake install**: Uncommented and fixed install rule in `python/dartpy/CMakeLists.txt`
- **Conditional builds**: Use `DART_BUILD_WHEELS` flag for wheel-specific behavior
- **GUI exclusion**: GUI OSG bindings excluded from wheel builds (filtered in CMakeLists.txt)
- **Python 3.13 fix**: Migrated from deprecated `distutils` to `sysconfig`

### Configuration
- **pyproject.toml**: Cleaned up redundant CMake args, added wheel-specific settings
- **CMakeLists.txt**: Single source of truth, handles both wheel and traditional builds
- **Conditional pagmo**: Skipped in wheel builds to reduce size

### Tools Added
- `scripts/verify_wheel.py` - Verify wheel contents
- `scripts/test_installation.py` - Test dartpy installation
- Pixi tasks: `build-wheel`, `verify-wheel`, `test-installation`

## Installation Methods

```bash
# Traditional CMake
cmake -DDART_BUILD_DARTPY=ON .. && make dartpy && cmake --install .

# Pip editable
pip install -e . -v --no-build-isolation

# Wheel build
pixi run build-wheel
pixi run verify-wheel
```

## Architecture Decisions

1. **CMakeLists.txt as single source**: Minimizes duplication, supports conda-forge
2. **scikit-build-core backend**: Modern standard for scientific Python packages
3. **Minimal wheels**: Exclude GUI/pagmo for smaller, faster builds
4. **DART_BUILD_WHEELS flag**: Controls conditional behavior cleanly

## Files Modified

**Core**:
- `python/dartpy/CMakeLists.txt` - Install rules, GUI exclusion, Python 3.13 fix
- `dart/optimizer/CMakeLists.txt` - Conditional pagmo
- `pyproject.toml` - Cleaned configuration
- `pixi.toml` - Added wheel tasks

**Documentation**:
- `docs/readthedocs/dartpy/developer_guide/build.rst` - Complete rewrite
- `docs/onboarding/python-bindings.md` - Updated with new build info
- `docs/dev_tasks/README.md` - Task tracking guidelines

**Tools**:
- `scripts/verify_wheel.py` - New
- `scripts/test_installation.py` - New

## Validation

✅ All builds pass:
- Traditional CMake install works
- Wheel builds successfully (37MB, contains dartpy extension + 884 files)
- All pixi tasks functional
- No validation errors

## Lessons Learned

- Always test both wheel and traditional builds with conditional logic
- Keep CMakeLists.txt as single source of truth to avoid duplication
- Filter GUI files at CMake level when building wheels
- Use modern Python standards (`sysconfig` not `distutils`)
