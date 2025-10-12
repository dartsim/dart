# Python API Documentation Approach Analysis

## Problem Statement

We want to generate Sphinx documentation for dartpy (Python bindings) that works on **both local builds and Read the Docs** without requiring the compiled C++ extension module.

## Approaches Evaluated

### ❌ Approach 1: Use Stub Files (.pyi) with sphinx-autoapi

**What we tried:**
1. Generate `.pyi` stub files with `pybind11-stubgen`
2. Fix syntax errors (reserved keywords, duplicate parameters, unmatched parentheses)
3. Convert `.pyi` to `.py` for sphinx-autoapi compatibility
4. Use sphinx-autoapi to generate documentation

**Why it doesn't work:**
- **pybind11-stubgen has known bugs**: Generates duplicate parameter names (`std: ..., std: ..., std: ...`)
- **Invalid syntax**: Reserved keywords as parameter names (`from:`, `to:`)
- **Complex type hints**: References to types that don't exist as importable modules
- **Fragile post-processing**: Custom parsing code breaks easily with new edge cases
- **Maintenance burden**: Every pybind11-stubgen update may introduce new issues

**Status**: ⚠️ **NOT RECOMMENDED** - Too fragile and unmaintainable

---

### ✅ Approach 2: Use Compiled Module Locally (Current Working Solution)

**How it works:**
1. Local builds: Use compiled dartpy module via PYTHONPATH
2. Sphinx autodoc imports and introspects the actual module
3. Read the Docs: Accept empty API pages until dartpy is on PyPI

**Advantages:**
- ✅ **Robust**: Uses actual compiled module, no parsing issues
- ✅ **Complete**: Full API with docstrings, signatures, type hints
- ✅ **Maintainable**: No custom code to maintain
- ✅ **Standard approach**: This is how most Python C extensions document

**Disadvantages:**
- ⚠️ Read the Docs won't have Python API documentation
- ⚠️ Requires building dartpy before documentation

**Status**: ✅ **CURRENTLY WORKING** - Best solution until RTD solution is ready

---

### 🎯 Approach 3: Pre-built Wheels for Read the Docs (RECOMMENDED FUTURE)

**How it would work:**
1. Build dartpy wheels in CI for multiple platforms (Linux, macOS, Windows)
2. Publish wheels to PyPI or GitHub Releases
3. Configure Read the Docs to install from PyPI: `pip install dartpy`
4. Use same Sphinx autodoc approach as local builds

**Advantages:**
- ✅ **Full API docs on RTD**: Same quality as local builds
- ✅ **Robust**: Uses actual compiled module
- ✅ **Standard**: This is the industry-standard approach
- ✅ **Consistent**: Same docs locally and on RTD

**Implementation effort:**
1. Set up cibuildwheel in CI (1-2 days)
2. Configure PyPI publishing (1 day)
3. Update `.readthedocs.yml` to install dartpy (5 minutes)

**Status**: 🎯 **RECOMMENDED** - The proper long-term solution

---

### 🤔 Approach 4: Manual RST Documentation (Alternative)

**How it works:**
- Manually write `.rst` files documenting each class/function
- Include code examples and usage patterns
- No automated API generation

**Advantages:**
- ✅ Works on both local and RTD
- ✅ Can include better examples and explanations
- ✅ No dependency on compiled module

**Disadvantages:**
- ❌ **High maintenance**: Must manually update for API changes
- ❌ **Incomplete**: Easy to miss functions or get signatures wrong
- ❌ **Slow**: Takes significant time to write comprehensive docs

**Status**: ⚠️ **NOT RECOMMENDED** - Too much manual work

---

## Recommendation

### Short-term (Current State)
**Keep Approach 2: Use compiled module locally**

```bash
# Local documentation (full API)
pixi run docs-build && pixi run docs-serve

# Read the Docs (empty API pages, but structure works)
# Just commit and push - RTD will build
```

**Pros:**
- Already working perfectly locally
- No additional maintenance burden
- RTD builds won't fail, just have empty API pages

### Long-term (Within 1-2 months)
**Implement Approach 3: Pre-built wheels**

**Steps:**
1. Add cibuildwheel to CI workflow
2. Publish dartpy to PyPI
3. Update `.readthedocs.yml`:
   ```yaml
   python:
     install:
       - method: pip
         path: .
         extra_requirements:
           - docs
       - requirements: docs/requirements.txt
   ```

**Result:** Full Python API documentation on Read the Docs!

---

## Why Not Stub Files?

The stub file approach is fundamentally flawed for documentation because:

1. **Stub files are for type checkers, not runtime**
   - They can contain shorthand syntax that doesn't parse as valid Python
   - They're never meant to be imported or executed

2. **pybind11-stubgen is imperfect**
   - It generates invalid Python syntax
   - It has bugs (duplicate parameters, unmatched parentheses)
   - It outputs C++ type names that don't exist in Python

3. **Post-processing is fragile**
   - We'd need to maintain custom parsing code
   - New pybind11-stubgen versions = new bugs to fix
   - Edge cases will keep appearing

4. **Industry doesn't do this**
   - NumPy, PyTorch, TensorFlow all use the compiled module approach
   - Standard solution: Publish wheels, install in RTD

---

## Decision

✅ **Use Approach 2 now** (compiled module locally)
🎯 **Plan Approach 3 next** (pre-built wheels for RTD)
❌ **Abandon stub file approach** (too fragile)

---

## Files to Clean Up

If we abandon the stub file approach:

```bash
# Remove these files:
rm scripts/prepare_stubs_for_docs.py
rm scripts/convert_stubs_to_modules.py  # if exists

# Update pixi.toml:
# - Remove prepare-stubs-for-docs task
# - Update docs-build to use compiled module (already done)

# Update conf.py:
# - Remove autoapi configuration
# - Keep simple PYTHONPATH setup (already done)
```

## Summary

The **compiled module approach is the right solution**. Stub files are the wrong tool for this job. Focus energy on publishing dartpy wheels to PyPI instead of trying to make stub files work for documentation.
