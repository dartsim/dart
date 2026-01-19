# Test Coverage Audit — Dev Task

## Current Status

- [x] Phase 0: Initial assessment and branch setup
- [x] Phase 1: Enable valid disabled tests (3 of 4 enabled)
- [x] Phase 2: Fix coverage infrastructure (lcov filters, codecov.yml)
- [x] Phase 3: Identify and prioritize coverage gaps (see `02-coverage-gaps.md`)
- [ ] Phase 4: Implement high-value tests (in progress)
- [ ] Phase 5: Establish ongoing coverage standards

**Branch**: `task/test_coverage`

## Goal

Improve DART's test coverage from ~67% to 80%+ with meaningful, maintainable tests that validate behavior and contracts—not just line counts.

## What's Been Done

### Phase 1: Enabled Disabled Tests

| Test | Status |
|------|--------|
| `ClonedWorldsStayDeterministic` | ✅ Enabled |
| `ContactsReportNonZeroForceWithLargeTimeStep` | ✅ Enabled |
| `CoMJacobianSignConsistency` | ✅ Enabled |
| `HierarchicalTransforms` | ⏳ Requires Phase 2 experimental |

### Phase 2: Infrastructure Fixes

1. **Fixed lcov filter** (`pixi.toml:802`): Added `'*/.pixi/*'` to exclude third-party headers
2. **Updated codecov.yml**:
   - Added per-module components for visibility
   - Added gcov `branch_detection` to reduce macro/exception noise
   - Added exclusions: `examples/**`, `tutorials/**`, `**/ikfast*.cpp`
   - Set realistic targets: 80% project, 85% patch

### Phase 3: Coverage Gap Analysis

See `02-coverage-gaps.md` for detailed analysis. Summary:

| Module | Test Ratio | Priority |
|--------|-----------|----------|
| common | 0.15 | **High** (37 untested headers) |
| utils | 0.15 | Medium |
| dynamics | 0.30 | High |
| collision | 0.30 | High |
| constraint | 0.34 | High |

### Phase 4: Tests Added (In Progress)

| Test File | Tests | Coverage |
|-----------|-------|----------|
| `test_NameManager.cpp` | 20 | `dart::common::NameManager` |

## What's Next

**Continue Phase 4**: Add quick-win tests from the prioritized list in `02-coverage-gaps.md`.

Top candidates:
1. `test_Singleton.cpp` - Template singleton pattern
2. `test_Signal.cpp` - Event system (Observer/Subject)
3. Expand `test_CollisionFilter.cpp` - Already partially tested

## Test Suite Structure

| Directory | Count | Purpose |
|-----------|-------|---------|
| `tests/unit/` | 120+ | Isolated component testing |
| `tests/integration/` | 68 | Cross-module testing |
| `tests/benchmark/` | 19 | Performance measurement |
| `tests/helpers/` | 3 | Shared test utilities |

## Key Decisions

| Decision | Rationale |
|----------|-----------|
| 80% target (not 85%) | More realistic near-term; 85% is stretch goal |
| Focus on behavioral tests | Line coverage is a proxy, not the goal |
| Exclude GUI from metrics | Cannot test headless reliably |
| Per-module thresholds | Different modules have different testability |

## Success Criteria

| Metric | Current | Target | Stretch |
|--------|---------|--------|---------|
| Overall line coverage | ~67% | 80% | 85% |
| Core module coverage | ~69% | 82% | 87% |
| CI runtime | ~10min | <12min | <10min |
| Flaky test rate | Unknown | <1% | 0% |

## Non-Goals

- 100% coverage (diminishing returns)
- Testing GUI code in CI (requires display)
- Testing experimental APIs (unstable)
- Trivial getter/setter tests

## Files in This Task

| File | Purpose |
|------|---------|
| `README.md` | This file - task overview |
| `RESUME.md` | Instructions for resuming work |
| `02-coverage-gaps.md` | Detailed gap analysis |
| `DISABLED_TESTS.md` | Analysis of disabled tests |

## Related Files (Outside This Directory)

| File | Purpose |
|------|---------|
| `codecov.yml` | Coverage configuration |
| `pixi.toml:789-810` | Coverage report generation |
| `tests/unit/CMakeLists.txt` | Test registration |
| `tests/helpers/` | Shared test utilities |
