# Resume: Test Coverage Audit

> **For AI Agents**: Read this file first when resuming work on the test coverage audit task.

## Branch

```bash
git checkout task/test_coverage
git pull origin task/test_coverage
git log -5 --oneline
```

## Current State (as of Jan 2026)

**Phases completed:**
- ✅ Phase 0: Initial assessment
- ✅ Phase 1: Enabled 3 disabled tests (1 remains for Phase 2 experimental)
- ✅ Phase 2: Fixed coverage infrastructure (lcov + codecov.yml)
- ✅ Phase 3: Created coverage gap analysis (`02-coverage-gaps.md`)
- 🔄 Phase 4: Test implementation (in progress)

**Tests added so far:**
- `tests/unit/common/test_NameManager.cpp` (20 tests)

## Immediate Next Step

**Add more quick-win tests from the prioritized list.**

Pick ONE of these and implement:

| Priority | Class | File to Create | Estimated Tests |
|----------|-------|----------------|-----------------|
| 1 | `Singleton` | `tests/unit/common/test_Singleton.cpp` | 5-8 |
| 2 | `Signal/Observer` | `tests/unit/common/test_Signal.cpp` | 10-15 |
| 3 | `CollisionFilter` | Expand `tests/unit/collision/test_CollisionFilter.cpp` | +5-10 |

## How to Add a New Test

1. **Create test file** following existing patterns:
   ```cpp
   // tests/unit/common/test_NewClass.cpp
   #include <dart/common/NewClass.hpp>
   #include <gtest/gtest.h>
   // ... tests using TEST() or TEST_F() macros
   ```

2. **Add to CMakeLists.txt** (`tests/unit/CMakeLists.txt`):
   - For common module: Add to `common_unit_sources` list (around line 40-56)
   - For other modules: Use `dart_add_test()` macro

3. **Build and run**:
   ```bash
   pixi run build
   cmake --build build/default/cpp/Release --target UNIT_common_NewClass
   ./build/default/cpp/Release/bin/UNIT_common_NewClass
   ```

4. **Verify all tests pass**:
   ```bash
   pixi run test-unit
   ```

## Key Files to Know

| File | Purpose |
|------|---------|
| `codecov.yml` | Coverage configuration (components, exclusions, targets) |
| `pixi.toml:789` | Coverage report generation (lcov filters) |
| `tests/unit/CMakeLists.txt` | Test registration |
| `tests/helpers/GTestUtils.hpp` | Eigen comparison macros |
| `02-coverage-gaps.md` | Detailed gap analysis with priorities |

## Important Context

1. **Coverage.info filter**: We added `*/.pixi/*` to lcov `--remove` to exclude third-party headers. Without this, coverage was polluted with Eigen/assimp/etc.

2. **codecov.yml changes**:
   - Added per-module components for visibility
   - Added gcov `branch_detection` to reduce macro noise
   - Lowered project target to 80% (85% was unrealistic near-term)
   - Patch target remains 85% for new code

3. **Test file naming**: `test_<ClassName>.cpp` for unit tests

4. **CMake pattern for common tests**: Add to `common_unit_sources` list, NOT via `dart_add_test()`

5. **NameManager quirk**: `changeObjectName()` returns `newName` (not empty string) when object not found

## Module Coverage Ratios (pre-improvement)

| Module | Ratio | Notes |
|--------|-------|-------|
| common | 0.15 | 37 untested headers - HIGH priority |
| utils | 0.15 | Parser support mostly tested |
| dynamics | 0.30 | Largest module |
| collision | 0.30 | Multiple backends |
| constraint | 0.34 | Solver edge cases need work |

## Commits on This Branch

```
11905d689e8 docs: update test coverage audit status and resume instructions
92d245c165a test: add NameManager unit tests and coverage gap analysis
55c26ac7827 test: improve coverage infrastructure and enable valid disabled tests
```

## Verification Before Committing

```bash
pixi run lint          # Format code
pixi run build         # Ensure it compiles
pixi run test-unit     # Run unit tests (or specific test)
git add <files>
git commit -m "test: add <ClassName> unit tests"
git push origin task/test_coverage
```

## What NOT To Do

- ❌ Don't add trivial getter/setter tests (low ROI)
- ❌ Don't test private implementation details
- ❌ Don't delete failing tests to make coverage pass
- ❌ Don't add tests for `dart/gui/` (excluded, requires display)
- ❌ Don't add tests for `dart/simulation/experimental/` (unstable API)
