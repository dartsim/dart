# Dantzig LCP Solver Modernization Plan

**Status:** Work In Progress
**Last Updated:** 2025-10-14
**Assignee:** TBD

## Context & Resume Prompt

```
I am working on modernizing the Dantzig LCP solver in the DART project located at dart/lcpsolver/dantzig/.
This code was originally copied from dart/external/odelcpsolver/. The goal is to modernize it while maintaining
correctness and improving performance. Please continue with the next pending task from the plan document at
docs/readthedocs/dart/developer_guide/wip/dantzig_lcp_modernization.md.
```

## Overview

The Dantzig LCP solver in `dart/lcpsolver/dantzig/` is a critical component for constraint solving in DART's
physics engine. It was originally copied from the Open Dynamics Engine (ODE) implementation at
`dart/external/odelcpsolver/`. This plan outlines a comprehensive modernization effort to improve code quality,
performance, and maintainability while ensuring correctness through rigorous testing.

## Goals

1. **Establish Baseline**: Move ODE reference implementation to a baseline testing directory
2. **Validate Correctness**: Create comprehensive tests to verify behavior matches ODE implementation
3. **Performance Benchmarking**: Establish performance baselines and track improvements
4. **Namespace Migration**: Remove ODE namespace, adopt dart::lcpsolver namespace
5. **Modernization (Phased)**:
   - Phase A: STL libraries (vectors, algorithms, etc.) - no performance degradation
   - Phase B: Eigen library integration for matrix operations
   - Phase C: DrJit C++ library for CPU architecture-agnostic SIMD operations
6. **Code Style**: Update to match DART codebase conventions

## Current Status

### Completed Tasks ✅
- [x] Copied dart/external/odelcpsolver to dart/lcpsolver/dantzig/
- [x] Created tests/unit/lcpsolver/ directory structure
- [x] Added pixi run test-lcpsolver command
- [x] Moved test_Lemke.cpp to unit tests
- [x] Create comprehensive plan document (this document)
- [x] Copied dart/external/odelcpsolver/ to tests/baseline/odelcpsolver/
- [x] Created tests/baseline/CMakeLists.txt for baseline ODE library
- [x] Updated tests/CMakeLists.txt to include baseline subdirectory
- [x] Created tests/benchmark/lcpsolver/ infrastructure:
  - CMakeLists.txt for benchmark builds
  - LCPTestProblems.hpp (shared test cases for 1D-48D problems)
  - bm_lcpsolver.cpp (combined benchmark for Dantzig vs ODE baseline comparison)
- [x] Created comprehensive unit test test_DantzigVsODE.cpp:
  - Tests all problem sizes (1D, 2D, 4D, 6D, 12D, 24D, 48D)
  - Verifies complementarity conditions
  - Compares solutions between Dantzig and ODE baseline
  - Currently passing: 1D, 2D, 4D (3/7 tests)
- [x] Updated tests/benchmark/CMakeLists.txt to include lcpsolver subdirectory
- [x] Fixed baseline library includes to use relative paths
- [x] Added pixi task `bm-lcpsolver` for running combined benchmarks
- [x] Baseline ODE library successfully builds and links

### In Progress 🚧
- [ ] Refine test cases for better LCP problem conditioning
- [ ] Investigate and fix remaining test failures (6D, 12D, 24D, 48D)

### Pending Tasks 📋
- [ ] Remove ODE namespace from dart/lcpsolver/dantzig/
- [ ] Phase A: STL modernization
- [ ] Phase B: Eigen integration
- [ ] Phase C: DrJit SIMD integration (future)
- [ ] Code style updates

## Detailed Implementation Plan

### 1. Establish Baseline (Week 1)

#### 1.1 Move ODE Reference Implementation
**Status:** Pending

**Tasks:**
- Move `dart/external/odelcpsolver/` → `tests/baseline/odelcpsolver/`
- Create `tests/baseline/CMakeLists.txt`
- Build baseline ODE LCP solver as a separate library for testing
- Update any references in the codebase

**Files to modify:**
- `tests/baseline/CMakeLists.txt` (create)
- `tests/CMakeLists.txt` (add subdirectory)
- Update any includes/references

**Success criteria:**
- Baseline ODE solver builds successfully
- Can be linked from test files
- No impact on main DART build

#### 1.2 Create Benchmark Infrastructure
**Status:** Pending

**Tasks:**
- Create `tests/benchmark/lcpsolver/` directory
- Add benchmark comparing dart::lcpsolver::dantzig vs ODE baseline
- Test cases: various problem sizes (1D, 2D, 4D, 6D, 12D, 24D, 48D)
- Measure: solve time, iteration count, solution accuracy
- Add to pixi commands: `pixi run benchmark-lcpsolver`

**Files to create:**
- `tests/benchmark/lcpsolver/CMakeLists.txt`
- `tests/benchmark/lcpsolver/bm_dantzig_solver.cpp`
- `tests/benchmark/lcpsolver/bm_ode_baseline.cpp`
- `tests/benchmark/lcpsolver/test_problems.hpp` (shared test cases)

**Success criteria:**
- Benchmarks run successfully
- Can compare performance between implementations
- Baseline performance metrics documented

### 2. Validation & Testing (Week 1-2)

#### 2.1 Comprehensive Unit Tests
**Status:** Pending

**Tasks:**
- Create unit tests that compare dart::lcpsolver::dantzig output with ODE baseline
- Test cases covering:
  - Small problems (1D-6D)
  - Medium problems (12D-24D)
  - Large problems (48D+)
  - Edge cases (singular matrices, infeasible problems)
  - Typical robotics scenarios
- Validate:
  - Solution correctness (residual < epsilon)
  - Iteration count similar
  - Complementarity conditions satisfied

**Files to create:**
- `tests/unit/lcpsolver/test_DantzigSolver.cpp`
- `tests/unit/lcpsolver/test_DantzigVsODE.cpp`
- `tests/unit/lcpsolver/LCPTestProblems.hpp`

**Success criteria:**
- 100% test pass rate
- Solutions match ODE baseline within tolerance
- Edge cases handled correctly

### 3. Namespace Migration (Week 2)

#### 3.1 Remove ODE Namespace
**Status:** Pending

**Tasks:**
- Replace ODE-specific types with DART/standard types:
  - `dReal` → `double` or `dart::lcpsolver::Real`
  - `dxSolveL*` → `dart::lcpsolver::solveL*`
  - Remove ODE_API macros
- Update namespace from global/ODE to `dart::lcpsolver::dantzig`
- Update all includes and forward declarations

**Files to modify:**
- `dart/lcpsolver/dantzig/*.h`
- `dart/lcpsolver/dantzig/*.cpp`
- Any files that include dantzig headers

**Success criteria:**
- No ODE-specific names remain
- All code in dart::lcpsolver namespace
- Tests still pass
- No performance regression

### 4. Phase A: STL Modernization (Week 3)

**Status:** Pending

**Principles:**
- Only replace raw arrays/pointers where STL improves safety/clarity
- No performance degradation (verify with benchmarks)
- Prefer std::array for fixed-size, std::vector for dynamic

**Tasks:**
- Replace C-style arrays with std::array or std::vector where appropriate
- Use std::algorithm functions (std::fill, std::copy, std::transform, etc.)
- Replace manual memory management with RAII
- Use std::unique_ptr/std::shared_ptr where appropriate
- Modern for loops (range-based) where clearer

**Files to modify:**
- All files in `dart/lcpsolver/dantzig/`

**Benchmarking:**
- Run benchmarks before and after each change
- Document any performance changes
- Rollback if >5% performance degradation

**Success criteria:**
- Modern C++ idioms used throughout
- No raw new/delete for memory management
- Tests pass
- Performance ≥95% of baseline

### 5. Phase B: Eigen Integration (Week 4)

**Status:** Pending

**Tasks:**
- Replace raw matrix operations with Eigen
- Use Eigen::VectorXd, Eigen::MatrixXd
- Leverage Eigen's optimized linear algebra operations
- Use Eigen's expression templates for efficiency

**Files to modify:**
- `dart/lcpsolver/dantzig/lcp.cpp`
- `dart/lcpsolver/dantzig/matrix.cpp`
- `dart/lcpsolver/dantzig/fastldlt.cpp`
- Related headers

**Benchmarking:**
- Compare performance before/after Eigen integration
- Eigen's SIMD optimizations should improve performance

**Success criteria:**
- All matrix operations use Eigen
- Tests pass
- Performance ≥100% of baseline (hopefully better)

### 6. Phase C: DrJit SIMD (Future - Week 5+)

**Status:** Future Work

**Research needed:**
- DrJit compatibility with DART's build system
- Performance gains vs complexity
- Maintenance burden

**Tasks:**
- Evaluate DrJit library integration
- Proof of concept on critical paths
- Performance comparison
- Decision: integrate or defer

### 7. Code Style Updates (Throughout)

**Status:** Ongoing

**Guidelines:**
- Follow DART naming conventions:
  - Functions: camelCase
  - Classes: PascalCase
  - Private members: mMemberName
  - Constants: kConstantName
- Use DART's formatting (run clang-format)
- Add Doxygen comments
- Use const correctness
- Prefer auto where type is obvious

**Files to modify:**
- All files in `dart/lcpsolver/dantzig/`

**Success criteria:**
- Code passes DART's linter/formatter
- Consistent with DART codebase style
- Well-documented with Doxygen comments

## Testing Strategy

### Unit Tests
- Location: `tests/unit/lcpsolver/`
- Coverage: >90% line coverage
- Validation: Compare against ODE baseline
- Edge cases: Singular, infeasible, degenerate problems

### Benchmark Tests
- Location: `tests/benchmark/lcpsolver/`
- Track: Time, iterations, solution quality
- Compare: dart::lcpsolver vs ODE baseline
- Sizes: 1D, 2D, 4D, 6D, 12D, 24D, 48D

### Integration Tests
- Use in realistic robot simulation scenarios
- Verify no regression in existing DART examples
- Performance in end-to-end workflows

## Performance Metrics

### Baseline (ODE Implementation)
To be measured and documented in Phase 1.

### Target Performance
- Phase A (STL): ≥95% of baseline
- Phase B (Eigen): ≥100% of baseline (aiming for improvement)
- Phase C (DrJit): ≥110% of baseline (goal)

## Risk Management

### Risks & Mitigation

1. **Performance Regression**
   - Mitigation: Continuous benchmarking, rollback threshold
   - Each change validated before proceeding

2. **Correctness Issues**
   - Mitigation: Comprehensive unit tests vs ODE baseline
   - Numerical tolerance validation

3. **Integration Complexity**
   - Mitigation: Phased approach, incremental changes
   - Maintain backward compatibility during transition

4. **Time/Scope Creep**
   - Mitigation: Clear phase boundaries
   - Phase C (DrJit) is optional/future work

## Documentation Updates

As implementation progresses, update:
- This plan document (keep in sync!)
- API documentation (Doxygen comments)
- User-facing documentation if API changes
- Performance benchmarks and results
- Migration guide if needed

## Review & Sign-off

Each phase requires:
1. Code review
2. Test coverage verification
3. Performance benchmarking
4. Documentation updates
5. Sign-off before next phase

## Timeline

- **Week 1**: Baseline setup, initial benchmarks
- **Week 2**: Validation tests, namespace migration
- **Week 3**: Phase A (STL modernization)
- **Week 4**: Phase B (Eigen integration)
- **Week 5+**: Phase C evaluation (optional/future)

**Note**: Timeline is flexible based on complexity and performance validation.

## References

- Original ODE LCP solver: `dart/external/odelcpsolver/`
- DART coding standards: `CONTRIBUTING.md`
- Dantzig LCP algorithm papers/references: TBD
- DrJit library: https://github.com/mitsuba-renderer/drjit

---

**Important**: This document should be kept in sync with implementation progress.
Update status indicators and add notes as work proceeds.
