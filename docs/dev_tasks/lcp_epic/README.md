# LCP Epic: Dynamics-Agnostic LCP API + Solvers

## Status: Phase 1 In Progress

| Phase       | Description                           | Status                 |
| ----------- | ------------------------------------- | ---------------------- |
| **Phase 1** | Solver Selection API + Demo Stability | :construction: Current |
| Phase 2     | Test Infrastructure + Problem Factory | :hourglass: Pending    |
| Phase 3     | Demo Enhancements + Python Bindings   | :clipboard: Planned    |
| Phase 4     | Future Solvers, Benchmarks, Docs      | :clipboard: Stub       |

---

## Phase 1: Foundation (CURRENT FOCUS)

### Goals

1. Add `LcpSolverType` enum and primary/secondary config to `WorldConfig`
2. Fix demo crashes when switching scenes in `examples/lcp_solvers/`

### Non-Goals (deferred)

- Runtime solver switching (`World::setLcpSolver()`) -> Phase 4
- Python bindings for new enum -> Phase 3
- Additional solver types beyond Dantzig/Pgs/Lemke -> Future

---

### 1.1 WorldConfig Solver Selection

**Files**:

| File                        | Change                                             |
| --------------------------- | -------------------------------------------------- |
| `dart/simulation/World.hpp` | Add `LcpSolverType` enum, primary/secondary fields |
| `dart/simulation/World.cpp` | Create solver based on config                      |

**API**:

```cpp
enum class LcpSolverType { Dantzig, Pgs, Lemke };

struct WorldConfig {
  std::string name = "world";
  CollisionDetectorType collisionDetector = CollisionDetectorType::Fcl;
  LcpSolverType primaryLcpSolver = LcpSolverType::Dantzig;
  std::optional<LcpSolverType> secondaryLcpSolver = LcpSolverType::Pgs;
  // TODO: Revisit primary/secondary architecture in future
};
```

**Acceptance Criteria**:

- [ ] Default world uses Dantzig + PGS (unchanged behavior)
- [ ] Custom config works: `{.primaryLcpSolver = Pgs, .secondaryLcpSolver = std::nullopt}`
- [ ] All existing tests pass

---

### 1.2 Demo Stability (`examples/lcp_solvers/`)

**Root Cause**: Crashes when switching scenes (state management)

**Fixes**:

1. Clear all state on example switch
2. Guard problem dimension mismatches
3. Add exception handling around solver calls
4. Validate results before display

**Acceptance Criteria**:

- [ ] Switch all 10 examples without crash
- [ ] "Run All" on each example without crash
- [ ] ASAN clean

---

## Phase 2: Test Infrastructure (NEXT)

### Goals

Following PR #2462 patterns:

- Create `tests/common/lcpsolver/LcpProblemFactory.hpp` - Unified problem generation
- `tests/unit/math/lcp/test_AllSolversSmoke.cpp` - Every solver basic test
- `tests/unit/simulation/test_WorldLcpConfig.cpp` - Config selection tests
- Extend `test_LcpEdgeCases.cpp` - n=0, n=1, singular matrices

### 2.1 LCP Problem Factory

**New file**: `tests/common/lcpsolver/LcpProblemFactory.hpp`

**Categories**:

- Standard, Boxed, BoxedFriction
- WellConditioned, IllConditioned
- SingleContact, MultiContact, ChainStructure, StackStructure
- Edge cases: Empty, Trivial, Degenerate, ZeroRows

**Refactor**:

- Update `LcpTestFixtures.hpp` to use factory
- Update benchmark `bm_lcp_compare.cpp` to use factory

### Acceptance Criteria

- [ ] Every solver in `dart/math/lcp/` has at least one test
- [ ] Edge cases documented and tested
- [ ] `pixi run test-all` passes

---

## Phase 3: Demo + Python (PLANNED)

### Goals

- Additional demo scenarios using Problem Factory
- Convergence visualization for iterative solvers
- `dartpy` bindings for `LcpSolverType`
- Refactor demo to use shared Problem Factory

### Lower Priority Items

- Performance history export (CSV)
- Solver comparison overlay mode

---

## Phase 4: Future Work (STUB)

### 4.1 Future Solver Candidates

Based on research, prioritized by impact/effort:

| Priority | Solver                      | Source                     | Use Case                     | Complexity |
| -------- | --------------------------- | -------------------------- | ---------------------------- | ---------- |
| **P1**   | APGD (Nesterov-Accelerated) | Chrono, Mazhar ToG 2015    | 10-100x faster than PGS      | Low        |
| **P1**   | TGS (Temporal Gauss-Seidel) | PhysX/Isaac Gym            | GPU real-time                | Medium     |
| **P2**   | ADMM with Adaptive rho      | Pinocchio, Carpentier 2024 | Rigid+compliant              | Medium     |
| **P2**   | SAP (Semi-Analytic Primal)  | Drake v1.5+, Castro 2022   | Global convergence           | Medium     |
| **P3**   | IPC (Incremental Potential) | SIGGRAPH 2020              | Guaranteed intersection-free | High       |
| **P3**   | Semi-Smooth Newton          | Research                   | High accuracy                | Medium     |

### 4.2 Runtime Solver Switching

```cpp
// TODO: World::setLcpSolver() for post-creation changes
// Requires ConstraintSolver hot-swap capability
```

### 4.3 Benchmark Infrastructure

- Performance profiles (Dolan-More)
- CI regression detection
- Problem Factory scaling tests

### 4.4 Documentation Expansion

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

- `dart/math/lcp/` - LCP solver implementations (19+ solvers)
- `docs/background/lcp/` - Theory and algorithm documentation
- `examples/lcp_solvers/` - Interactive demo
- `tests/unit/math/lcp/` - Existing unit tests
- `tests/benchmark/lcpsolver/` - Performance benchmarks
- `tests/common/lcpsolver/` - Shared test fixtures

---

**Last Updated**: 2025-01-18
