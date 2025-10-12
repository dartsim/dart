# Cleanup Summary: Stub File Approach → Compiled Module Approach

## Date: 2025-10-13

## Decision: Option A - Keep Working Compiled Module Approach

After extensive testing, we determined that using stub files (.pyi) for documentation generation is **not robust or maintainable**. We've reverted to the working compiled module approach.

## Changes Made

### Files Deleted ❌
1. `/scripts/prepare_stubs_for_docs.py` - Custom stub file post-processor (deleted)

### Files Modified ✅
1. **`/docs/readthedocs/conf.py`**
   - Removed sphinx-autoapi configuration
   - Restored PYTHONPATH-based compiled module loading
   - Added clear messaging about local vs RTD builds

2. **`/pixi.toml`**
   - Removed `prepare-stubs-for-docs` task
   - Restored `docs-build` task with PYTHONPATH and `build-py-dev` dependency
   - Removed `sphinx-autoapi` dependency

3. **`/docs/readthedocs/requirements.txt`**
   - Removed `sphinx-autoapi` dependency

4. **`/docs/readthedocs/dartpy/python_api_reference.rst`**
   - Restored proper module toctree
   - Removed autoapi references

5. **`/docs/readthedocs/README.md`**
   - Updated to explain compiled module approach
   - Clear documentation of local vs RTD behavior
   - Future path: PyPI wheels

### Files Restored ✅
- `/docs/readthedocs/dartpy/modules/*.rst` - All 9 module RST files copied from `/docs/python_api/modules/`

### Directories Cleaned ✅
- `/docs/readthedocs/_stubs_fixed/` - Removed generated fixed stub files

## Current State ✅

### Local Builds (Working Perfectly!)
```bash
pixi run docs-build && pixi run docs-serve
```

**Result:**
- ✅ Full Python API documentation
- ✅ All classes, methods, type hints documented
- ✅ Uses compiled dartpy module via PYTHONPATH
- ✅ Clean build with no errors

### Read the Docs
- ⚠️ Python API pages will be empty (module not available)
- ✅ Documentation structure works correctly
- ✅ Navigation and other pages work fine
- 🎯 **Temporary limitation** until dartpy wheels are on PyPI

## Why Stub Files Didn't Work

1. **pybind11-stubgen bugs**
   - Generates duplicate parameter names: `def func(std: ..., std: ..., std: ...)`
   - Uses reserved keywords as params: `from:`, `to:`
   - Outputs invalid C++ type names

2. **Fundamental incompatibility**
   - Stub files (.pyi) are for type checkers, not runtime
   - Cannot be imported by Python
   - Contain shorthand syntax that doesn't parse as valid Python

3. **Fragile post-processing**
   - Requires complex custom parsing code
   - New edge cases emerge constantly
   - High maintenance burden

4. **Not industry standard**
   - NumPy, PyTorch, TensorFlow don't do this
   - Standard approach: Publish wheels, install in RTD

## Future Path 🎯

### Recommended: PyPI Wheels for Read the Docs

**Steps:**
1. Set up `cibuildwheel` in CI (1-2 days)
2. Publish dartpy wheels to PyPI (1 day)
3. Update `.readthedocs.yml` to install from PyPI (5 minutes)

**Result:** Full Python API documentation everywhere!

## Documentation

- ✅ `/docs/DOCUMENTATION_APPROACH_ANALYSIS.md` - Complete analysis of all approaches
- ✅ `/docs/readthedocs/README.md` - Updated with current approach
- ✅ This file - Cleanup summary

## Test Results ✅

```bash
$ pixi run docs-build
✓ Using compiled dartpy from PYTHONPATH
  Full Python API documentation will be generated
...
build succeeded.

The HTML pages are in _build/html.
```

**Build Status:** ✅ Success
**Warnings:** 0 errors, minor deprecation warnings (unrelated)
**API Documentation:** ✅ Complete with all modules

## Files to Keep

### For IDE Support
- `/scripts/generate_stubs.py` - Generates .pyi files for IDEs/type checkers
- `pixi run generate-stubs` - Task to generate IDE stub files

Stub files in `/python/stubs/` are still useful for:
- IDE autocompletion (PyCharm, VS Code)
- Type checking (mypy, pyright)
- Local development

## Conclusion

✅ **Working solution restored**
- Local docs: Perfect with compiled module
- Read the Docs: Acceptable temporary state
- Future: PyPI wheels for complete RTD docs

❌ **Stub file approach abandoned**
- Too fragile and unmaintainable
- Not the right tool for documentation
- Focus efforts on PyPI publishing instead
