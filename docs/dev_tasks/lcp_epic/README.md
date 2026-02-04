# LCP Epic: Dynamics-Agnostic LCP API + Solvers

## Status: Phase 4 In Progress

| Phase   | Description                           | Status                      |
| ------- | ------------------------------------- | --------------------------- |
| Phase 1 | Solver Selection API + Demo Stability | :white_check_mark: PR #2464 |
| Phase 2 | Test Infrastructure + Problem Factory | :white_check_mark: Complete |
| Phase 3 | Python Bindings                       | :white_check_mark: Complete |
| Phase 4 | New Solvers (APGD, TGS)               | :construction: PR #2465     |

---

## Phase 1: Foundation (COMPLETE ✅)

**PR #2464**: WorldConfig Solver Selection + Demo Stability

- Added `LcpSolverType` enum with Dantzig/Pgs/Lemke
- Added primary/secondary solver config to `WorldConfig`
- Fixed demo crashes with bounds checking and exception handling
- Unit test: `tests/unit/simulation/test_world_config.cpp`

---

## Phase 2: Test Infrastructure (CURRENT FOCUS)

### Goals

Following PR #2462 patterns:

- [x] Create `tests/common/lcpsolver/lcp_problem_factory.hpp` - Unified problem generation
- [x] `tests/unit/math/lcp/test_all_solvers_smoke.cpp` - Every solver basic test
- [ ] Extend `test_LcpEdgeCases.cpp` - n=1, singular matrices (optional)
- [ ] Update `lcp_test_fixtures.hpp` to use factory (optional refactor)

### 2.1 LCP Problem Factory (DONE ✅)

**New file**: `tests/common/lcpsolver/lcp_problem_factory.hpp`

**Categories implemented**:

- Standard, Boxed, BoxedFriction
- Edge cases: empty(), trivial1d(), trivial1dAtLowerBound()
- Well-conditioned: standard2dSpd(), boxed2dActiveUpper()
- Ill-conditioned: illConditioned3d()
- Contact scenarios: singleContactFriction()

### 2.2 All Solvers Smoke Test (DONE ✅)

**New file**: `tests/unit/math/lcp/test_all_solvers_smoke.cpp`

Tests all 19 LCP solvers with 5 test cases:

- EmptyProblemSucceeds
- Trivial1dDoesNotCrash
- Standard2dDoesNotCrash
- BoxedProblemHandledCorrectly
- FrictionProblemDoesNotCrash

### Acceptance Criteria

- [x] Every solver in `dart/math/lcp/` has at least one test
- [x] Edge cases documented and tested
- [x] `pixi run test-all` passes (143/143)

---

## Phase 3: Python Bindings (COMPLETE ✅)

### Completed

- [x] `LcpSolverType` enum bound to Python
- [x] `CollisionDetectorType` enum bound to Python
- [x] `WorldConfig` struct with solver selection fields
- [x] `World.create(config)` factory method
- [x] Python tests in `python/tests/unit/simulation/test_world.py`

### Deferred to Phase 4

- Additional demo scenarios using Problem Factory
- Convergence visualization for iterative solvers
- Refactor demo to use shared Problem Factory
- Performance history export (CSV)

---

## Phase 4: New Solvers (IN PROGRESS)

### 4.1 Implemented Solvers

| Priority | Solver                      | Source                     | Status                  |
| -------- | --------------------------- | -------------------------- | ----------------------- |
| **P1**   | APGD (Nesterov-Accelerated) | Chrono, Mazhar ToG 2015    | :white_check_mark: Done |
| **P1**   | TGS (Temporal Gauss-Seidel) | PhysX/Isaac Gym            | :white_check_mark: Done |
| **P2**   | ADMM with Adaptive rho      | Pinocchio, Carpentier 2024 | :white_check_mark: Done |
| **P2**   | SAP (Semi-Analytic Primal)  | Drake v1.5+, Castro 2022   | :white_check_mark: Done |

### 4.2 Future Solver Candidates

| Priority | Solver                      | Source        | Use Case                     | Complexity |
| -------- | --------------------------- | ------------- | ---------------------------- | ---------- |
| **P3**   | IPC (Incremental Potential) | SIGGRAPH 2020 | Guaranteed intersection-free | High       |

### 4.3 Implemented P3 Solvers

| Priority | Solver                   | Source   | Status                  |
| -------- | ------------------------ | -------- | ----------------------- |
| **P3**   | Boxed Semi-Smooth Newton | Research | :white_check_mark: Done |

### 4.4 Runtime Solver Switching

```cpp
// TODO: World::setLcpSolver() for post-creation changes
// Requires ConstraintSolver hot-swap capability
```

### 4.5 Benchmark Infrastructure

- [x] Performance profiles (Dolan-More) - `scripts/lcp_performance_profile.py`
- [ ] CI regression detection
- [ ] Problem Factory scaling tests

#### Performance Profile Results

Run with: `pixi run python scripts/lcp_performance_profile.py --run`

Results saved to `docs/background/lcp/figures/`:

- `performance_profile_standard.csv` - Standard LCP problems
- `performance_profile_boxed.csv` - Boxed constraint problems
- `performance_profile_frictionindex.csv` - Friction contact problems

**Key findings** (n=12,24,48,96):

| Category      | Top Performers                  |
| ------------- | ------------------------------- |
| Standard      | Direct, PGS, TGS, SAP           |
| Boxed         | TGS, PGS, Jacobi, SymmetricPsor |
| FrictionIndex | PGS, TGS, SAP, ADMM             |

### 4.6 Documentation Expansion

- Expand `docs/background/lcp/` as educational resource
- Add pseudocode for future solvers
- "Choosing a solver" decision guide

---

## Validation Commands

```bash
# Phase 1 validation
pixi run lint                    # Format
pixi run build                   # Build
pixi run test-unit               # Unit tests

# Demo manual testing
./build/default/cpp/Release/bin/lcp_solvers
# 1. Click through all examples rapidly
# 2. Click "Run All" on each
# 3. Switch solver while running

# Phase 2 validation
pixi run test-all                # Full test suite
```

---

## References

- `dart/math/lcp/` - LCP solver implementations (24 solvers)
- `docs/background/lcp/` - Theory and algorithm documentation
- `examples/lcp_solvers/` - Interactive demo
- `tests/unit/math/lcp/` - Existing unit tests
- `tests/benchmark/lcpsolver/` - Performance benchmarks
- `tests/common/lcpsolver/` - Shared test fixtures

---

**Last Updated**: 2025-01-18
