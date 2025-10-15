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
  - tests/common/lcpsolver/LCPTestProblems.hpp (shared test utilities for 1D-48D problems)
  - bm_lcpsolver.cpp (combined benchmark for Dantzig vs ODE baseline comparison)
- [x] Created comprehensive unit test test_DantzigVsODE.cpp:
  - Tests all problem sizes (1D, 2D, 4D, 6D, 12D, 24D, 48D)
  - Verifies complementarity conditions
  - Compares solutions between Dantzig and ODE baseline
  - All 7 tests passing (100%)
- [x] Created test_LCPTestProblems.cpp to validate test problem quality:
  - Checks symmetry, positive definiteness, conditioning
  - Includes random problem generators with fixed seeds
  - Separates well-formed and ill-formed test problems
- [x] Updated tests/benchmark/CMakeLists.txt to include lcpsolver subdirectory
- [x] Fixed baseline library includes using forward declarations
- [x] Added pixi task `bm-lcpsolver` for running combined benchmarks
- [x] Baseline ODE library successfully builds and links
- [x] Benchmark successfully compares Dantzig vs ODE baseline on identical problems
- [x] Confirmed both implementations are functionally identical (produce same results)
- [x] Created tests/common/ directory for shared test utilities
- [x] Namespace migration to dart::lcpsolver with C++17 nested namespace syntax
- [x] Added template function signatures (SolveLCP<Scalar>) with backward compatibility (dSolveLCP)
- [x] Enhanced benchmarking with F32/F64 precision comparisons and perfect vertical alignment
- [x] Phase 3.2.1: ScalarTraits<T> foundation added to common.h
- [x] Phase A (Partial): STL modernization
  - error.cpp: C++ headers (<cstdarg>, <cstdio>), std::array, dart::common::Logging integration
  - misc.cpp: Modern loop variables (for (int i...) with ++i)
  - matrix.cpp: std::fill for _dSetZero/_dSetValue, <algorithm> header

### In Progress 🚧
- [ ] Phase A: STL modernization (ongoing - 3 files completed, more remaining)

### Pending Tasks 📋
- [ ] Phase 3.2: Complete full template implementation (deferred - see status below)
- [ ] Phase A: STL modernization
- [ ] Phase B: Eigen integration
- [ ] Phase C: DrJit SIMD integration (future)
- [ ] Code style updates

## Detailed Implementation Plan

### Phase 3.2: Full Template Implementation (DEFERRED - Complexity)

**Status**: Phase 3.2.1 completed (ScalarTraits foundation), remaining steps deferred due to code complexity.

**Current Implementation**: The template API (`SolveLCP<Scalar>`) exists but uses type conversion internally:
- `SolveLCP<float>` converts float → double → float (lines 1514-1570 in lcp.cpp)
- `SolveLCP<double>` calls `dSolveLCP` directly (no conversion)

**Why Deferred**:
- Main solver implementation: 1,599 lines (lcp.cpp)
- Matrix operations: 577 lines (matrix.cpp)
- Complex internal state machine with dLCP class
- Fast LDLT optimizations with hand-tuned assembly-like code
- Risk of introducing bugs outweighs current benefits

**What Was Completed** (Phase 3.2.1):
- ✅ `ScalarTraits<T>` template added to `common.h`
- ✅ Specializations for float (ε=1e-7f) and double (ε=1e-14)
- ✅ Math function wrappers (sqrt, abs, recip, sin, cos)
- ✅ Foundation for future template work
- ✅ Tests pass, no regressions

**Future Work**:
Full native template support (no type conversions) would require:
1. Templatizing dLCP class (~500 lines of state management)
2. Templatizing all matrix operations (fastldlt, fastlsolve, fastltsolve, fastdot)
3. Updating 14+ source files with interconnected dependencies
4. Extensive testing to ensure numerical stability
5. Estimated effort: 2-3 weeks for safe, incremental implementation

**Recommendation**:
Defer until there's a compelling use case (e.g., performance critical path identified through profiling, embedded systems requiring float precision). Current API provides type flexibility, internal conversion overhead is minimal for typical problem sizes.

**Goal**: Completely templatize the Dantzig LCP solver to natively support both `float` and `double` precision without internal type conversions.

**Original Issue**: The existing implementation converts `float` → `double` → `float`, which defeats the purpose of templating and wastes performance.

**Target Architecture**:
```cpp
template <typename Scalar>
bool SolveLCP(
    int n,
    Scalar* A,
    Scalar* x,
    Scalar* b,
    Scalar* w,
    int nub,
    Scalar* lo,
    Scalar* hi,
    int* findex,
    bool earlyTermination);
```

#### Phase 3.2.1: Type Abstraction Layer (Step 1)

**Objective**: Create a type abstraction that replaces `dReal` with template parameter throughout the codebase.

**Files to modify**:
- `dart/lcpsolver/dantzig/common.h` - Add type traits and constants

**Tasks**:
1. Create template-friendly type traits:
```cpp
namespace dart::lcpsolver {

// Type traits for scalar types
template <typename Scalar>
struct ScalarTraits {
  static constexpr Scalar epsilon();
  static constexpr Scalar infinity();
  static Scalar sqrt(Scalar x);
  static Scalar abs(Scalar x);
  static Scalar recip(Scalar x);
};

// Specializations for float and double
template <>
struct ScalarTraits<float> {
  static constexpr float epsilon() { return 1e-7f; }
  static constexpr float infinity() { return std::numeric_limits<float>::infinity(); }
  static float sqrt(float x) { return std::sqrt(x); }
  static float abs(float x) { return std::fabs(x); }
  static float recip(float x) { return 1.0f / x; }
};

template <>
struct ScalarTraits<double> {
  static constexpr double epsilon() { return 1e-14; }
  static constexpr double infinity() { return std::numeric_limits<double>::infinity(); }
  static double sqrt(double x) { return std::sqrt(x); }
  static double abs(double x) { return std::fabs(x); }
  static double recip(double x) { return 1.0 / x; }
};

} // namespace dart::lcpsolver
```

2. Update `common.h` macros to use ScalarTraits:
```cpp
#define dInfinity ScalarTraits<Scalar>::infinity()
#define dRecip(x) ScalarTraits<Scalar>::recip(x)
// etc.
```

**Success Criteria**:
- Type traits compile for both float and double
- No changes to algorithm logic yet
- Tests still pass with double precision

#### Phase 3.2.2: Templatize Core Math Functions (Step 2)

**Objective**: Convert matrix operations and core math functions to templates.

**Files to modify**:
- `dart/lcpsolver/dantzig/matrix.h` - Function declarations
- `dart/lcpsolver/dantzig/matrix.cpp` - Function implementations

**Tasks**:
1. Templatize matrix functions:
```cpp
// Before:
void dFactorLDLT(dReal* A, dReal* d, int n, int nskip);

// After:
template <typename Scalar>
void FactorLDLT(Scalar* A, Scalar* d, int n, int nskip);
```

2. Move implementations to header (template implementations must be in headers)
3. Update all callers to use template versions

**Functions to templatize**:
- `dFactorLDLT` → `FactorLDLT<Scalar>`
- `dSolveLDLT` → `SolveLDLT<Scalar>`
- `dSolveL1` → `SolveL1<Scalar>`
- `dSolveL1T` → `SolveL1T<Scalar>`
- `dLDLTRemove` → `LDLTRemove<Scalar>`
- `dSetZero` → `SetZero<Scalar>`
- `dDot` → `Dot<Scalar>`
- `dMultiply*` → `Multiply*<Scalar>`

**Implementation Strategy**:
- Create template versions in new header: `matrix_template.hpp`
- Keep old implementations temporarily for backward compatibility
- Gradually migrate callers
- Remove old implementations after migration complete

**Success Criteria**:
- All matrix functions are templated
- Tests pass for both float and double
- Benchmark shows no performance regression

#### Phase 3.2.3: Templatize Fast LDLT Operations (Step 3)

**Objective**: Convert fast*.cpp optimized routines to templates.

**Files to modify**:
- `dart/lcpsolver/dantzig/fastldlt.cpp` → `fastldlt_template.hpp`
- `dart/lcpsolver/dantzig/fastlsolve.cpp` → `fastlsolve_template.hpp`
- `dart/lcpsolver/dantzig/fastltsolve.cpp` → `fastltsolve_template.hpp`
- `dart/lcpsolver/dantzig/fastdot.cpp` → `fastdot_template.hpp`

**Tasks**:
1. Move functions to headers with template parameter
2. Replace `dReal` with `Scalar` template parameter
3. Update internal function calls to use template versions
4. Test both float and double precision

**Functions to templatize**:
- `_dFactorLDLT` → `_FactorLDLT<Scalar>`
- `_dSolveL1` → `_SolveL1<Scalar>`
- `_dSolveL1T` → `_SolveL1T<Scalar>`
- Fast dot product implementations

**Success Criteria**:
- Fast operations work with both float and double
- Performance matches non-template baseline
- No code duplication

#### Phase 3.2.4: Templatize Main LCP Solver (Step 4)

**Objective**: Convert the main dLCP class and SolveLCP function to templates.

**Files to modify**:
- `dart/lcpsolver/dantzig/lcp.h` - Update declarations
- `dart/lcpsolver/dantzig/lcp.cpp` → move to `lcp_template.hpp`

**Tasks**:
1. Templatize dLCP class:
```cpp
template <typename Scalar>
struct LCP {
  const int m_n;
  const int m_nskip;
  int m_nub;
  int m_nC, m_nN;
  Scalar* const m_A;
  Scalar* const m_x;
  Scalar* const m_b;
  // ... etc

  void transfer_i_to_C(int i);
  void transfer_i_to_N(int i);
  // ... etc
};
```

2. Templatize main solver:
```cpp
template <typename Scalar>
bool SolveLCP(
    int n,
    Scalar* A,
    Scalar* x,
    Scalar* b,
    Scalar* w,
    int nub,
    Scalar* lo,
    Scalar* hi,
    int* findex,
    bool earlyTermination);
```

3. Move implementation to header file (required for templates)
4. Provide explicit instantiations in .cpp file for faster compilation:
```cpp
// lcp.cpp
#include "lcp_template.hpp"

namespace dart::lcpsolver {

// Explicit instantiations
template bool SolveLCP<float>(...);
template bool SolveLCP<double>(...);

} // namespace dart::lcpsolver
```

5. Update backward compatibility wrapper:
```cpp
// Keep for backward compatibility
inline bool dSolveLCP(
    int n, dReal* A, dReal* x, dReal* b, dReal* w,
    int nub, dReal* lo, dReal* hi, int* findex,
    bool earlyTermination = false)
{
  return SolveLCP<dReal>(n, A, x, b, w, nub, lo, hi, findex, earlyTermination);
}
```

**Success Criteria**:
- Main solver fully templated
- Both float and double work natively (no conversions)
- Tests pass for both types
- Benchmarks show appropriate performance characteristics

#### Phase 3.2.5: Update Utilities and Helpers (Step 5)

**Objective**: Templatize remaining utility functions.

**Files to modify**:
- `dart/lcpsolver/dantzig/misc.h` - Test and utility functions
- `dart/lcpsolver/dantzig/misc.cpp` - Implementations

**Tasks**:
1. Templatize test functions:
```cpp
template <typename Scalar>
int TestSolveLCP();
```

2. Templatize matrix test utilities:
```cpp
template <typename Scalar>
void MakeRandomMatrix(Scalar* A, int n, int m, Scalar range);
```

3. Update error handling to work with templates

**Success Criteria**:
- All utilities are templated
- Test suite works with both float and double
- No hard-coded type assumptions

#### Phase 3.2.6: Update Benchmarks (Step 6)

**Objective**: Remove type conversion overhead from benchmarks to get true performance comparison.

**Files to modify**:
- `tests/benchmark/lcpsolver/bm_lcpsolver.cpp`

**Tasks**:
1. Update F32 benchmark to use native float throughout:
```cpp
static void BM_Dantzig_F32_Solver(
    benchmark::State& state, dart::test::LCPProblem problem)
{
  // Convert problem data to float once
  std::vector<float> A_f(problem.dimension * problem.dimension);
  std::vector<float> b_f(problem.dimension);
  std::vector<float> x_f(problem.dimension, 0.0f);
  std::vector<float> w_f(problem.dimension, 0.0f);
  std::vector<float> lo_f(problem.dimension, -1e10f);
  std::vector<float> hi_f(problem.dimension, 1e10f);

  // Convert once
  for (int i = 0; i < problem.dimension; ++i) {
    for (int j = 0; j < problem.dimension; ++j) {
      A_f[i * problem.dimension + j] = static_cast<float>(problem.A(i, j));
    }
    b_f[i] = static_cast<float>(problem.b(i));
  }

  for (auto _ : state) {
    // Reset solution vectors
    std::fill(x_f.begin(), x_f.end(), 0.0f);
    std::fill(w_f.begin(), w_f.end(), 0.0f);

    // Copy A back (solver modifies it)
    std::vector<float> A_copy = A_f;

    // Solve LCP using native float (no conversions inside!)
    bool success = dart::lcpsolver::SolveLCP<float>(
        problem.dimension,
        A_copy.data(),
        x_f.data(),
        b_f.data(),
        w_f.data(),
        0,
        lo_f.data(),
        hi_f.data(),
        nullptr,
        false);

    benchmark::DoNotOptimize(success);
    benchmark::DoNotOptimize(x_f.data());
  }

  state.SetLabel("Dantzig-F32/" + problem.name);
}
```

2. Verify performance characteristics:
   - F32 should be faster due to smaller memory footprint and SIMD
   - Or possibly slower due to precision issues requiring more iterations
   - Document actual results

**Success Criteria**:
- Benchmarks use native types throughout
- No hidden conversions
- True performance comparison documented

#### Phase 3.2.7: Update Tests (Step 7)

**Objective**: Ensure test suite validates both float and double implementations.

**Files to modify**:
- `tests/unit/lcpsolver/test_DantzigVsODE.cpp`

**Tasks**:
1. Add parameterized tests for both types:
```cpp
template <typename Scalar>
class DantzigSolverTest : public ::testing::Test {
protected:
  void TestProblem(const LCPProblem& problem) {
    // Test logic here
  }
};

using ScalarTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(DantzigSolverTest, ScalarTypes);

TYPED_TEST(DantzigSolverTest, Problem1D) {
  this->TestProblem(getProblem1D());
}
```

2. Verify numerical accuracy for both types:
   - Float: looser tolerance (1e-5)
   - Double: stricter tolerance (1e-10)

**Success Criteria**:
- Tests pass for both float and double
- Appropriate tolerances for each type
- Coverage of edge cases for both types

#### Phase 3.2.8: Documentation and Cleanup (Step 8)

**Objective**: Document the templated API and clean up old code.

**Tasks**:
1. Update API documentation in headers
2. Add examples of using both float and double
3. Remove old non-template implementations
4. Update CMakeLists.txt if needed
5. Update this plan document with results

**Success Criteria**:
- API is well documented
- No dead code remains
- Clean separation of concerns
- Easy to use for clients

#### Testing Strategy for Phase 3.2

**For each step**:
1. Run unit tests: `pixi run test-lcpsolver`
2. Run benchmarks: `pixi run bm-lcpsolver`
3. Compare results:
   - Correctness: Solutions should match baseline within tolerance
   - Performance: Document any changes
4. Document findings before proceeding to next step

**Performance Expectations**:
- **Float (F32)**:
  - Memory: ~50% of F64 (due to smaller data)
  - Speed: Could be faster (SIMD, cache) or slower (precision issues)
  - Accuracy: Lower precision, may need more iterations

- **Double (F64)**:
  - Memory: Baseline reference
  - Speed: Should match current performance
  - Accuracy: Higher precision, fewer iterations

**Rollback Criteria**:
- If any step causes >10% performance regression for F64
- If tests fail and cannot be fixed within reasonable effort
- If complexity becomes unmanageable

#### File Organization After Phase 3.2

**New structure**:
```
dart/lcpsolver/dantzig/
├── common.h              # Type traits, constants
├── lcp.h                 # Public API declarations
├── lcp.cpp               # Explicit template instantiations
├── lcp_template.hpp      # Template implementations
├── matrix_template.hpp   # Matrix operations (templates)
├── fastldlt_template.hpp # Fast LDLT operations
├── fastlsolve_template.hpp
├── fastltsolve_template.hpp
├── fastdot_template.hpp
├── misc_template.hpp     # Utility functions
├── error.h               # Error handling
└── error.cpp             # Error implementation
```

**Backward compatibility**:
- Keep `dReal` as `typedef double dReal;`
- Keep `dSolveLCP` as inline wrapper to `SolveLCP<double>`
- Existing code continues to work unchanged

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
