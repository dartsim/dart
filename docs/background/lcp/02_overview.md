# LCP Solvers in DART

**Navigation**: [← Problem Statement](01_problem-statement.md) | [Pivoting Methods →](03_pivoting-methods.md)

## Implementation Status

This section tracks which LCP solvers are currently implemented in DART (`dart/math/lcp/`).

| Category           | Method                     | Status             | Location                                            | Notes                                   |
| ------------------ | -------------------------- | ------------------ | --------------------------------------------------- | --------------------------------------- |
| **Pivoting**       | Dantzig Principal Pivoting | ✅ Implemented     | `pivoting/DantzigSolver.hpp`                        | BLCP solver with friction index support |
| **Pivoting**       | Lemke Complementary Pivot  | ✅ Implemented     | `pivoting/LemkeSolver.hpp`                          | Standard LCP solver                     |
| **Pivoting**       | Baraff Incremental         | ❌ Not implemented | -                                                   | Planned                                 |
| **Projection**     | PGS (Gauss-Seidel)         | ✅ Implemented     | `projection/PgsSolver.hpp`                          | Boxed LCP + friction index (iterative)  |
| **Projection**     | PSOR (Over-Relaxation)     | ✅ Implemented     | `projection/PgsSolver.hpp`                          | Set `LcpOptions::relaxation`            |
| **Projection**     | Jacobi (Projected)         | ✅ Implemented     | `projection/JacobiSolver.hpp`                       | Parallel-friendly baseline              |
| **Projection**     | Blocked Gauss-Seidel       | ✅ Implemented     | `projection/BgsSolver.hpp`                          | For contact problems                    |
| **Projection**     | NNCG (Conjugate Gradient)  | ✅ Implemented     | `projection/NncgSolver.hpp`                         | Better convergence than PGS             |
| **Projection**     | Subspace Minimization      | ✅ Implemented     | `projection/SubspaceMinimizationSolver.hpp`         | Hybrid PGS approach                     |
| **Newton**         | Minimum Map Newton         | ✅ Implemented     | `newton/MinimumMapNewtonSolver.hpp`                 | Standard LCP (boxed/findex fallback)    |
| **Newton**         | Fischer-Burmeister Newton  | ✅ Implemented     | `newton/FischerBurmeisterNewtonSolver.hpp`          | Standard LCP (boxed/findex fallback)    |
| **Newton**         | Penalized FB Newton        | ✅ Implemented     | `newton/PenalizedFischerBurmeisterNewtonSolver.hpp` | Extension of FB                         |
| **Interior Point** | Interior Point Method      | ❌ Not implemented | -                                                   | Very robust                             |
| **Staggering**     | Staggering Method          | ❌ Not implemented | -                                                   | For coupled problems                    |

**Legend**: ✅ Implemented | 🚧 In Progress | ❌ Not Implemented | 📋 Planned

### Unified API: `LcpProblem` + `LcpSolver`

- `LcpProblem` bundles boxed LCP data `(A, b, lo, hi, findex)` so solvers share
  one interface and handle friction index coupling consistently.
- All solvers implement
  `LcpSolver::solve(const LcpProblem&, Eigen::VectorXd&, const LcpOptions&)`.
- `constraint::ConstraintSolver` now builds an `LcpProblem` and calls
  `math::DantzigSolver` (primary) with an optional `math::PgsSolver` fallback.
- Solvers validate basic invariants (e.g., `lo <= hi`, `findex` in range, no
  NaN bounds) and treat empty problems as a trivial success.

### Repository Layout

```text
dart/math/lcp/
├── LcpTypes.hpp/cpp            # LcpProblem, LcpOptions, LcpResult, status codes
├── LcpSolver.hpp/cpp           # LcpSolver interface
├── LcpValidation.hpp           # Shared residual/KKT validation utilities
├── All.hpp                     # Convenience umbrella header
│
├── pivoting/
│   ├── DantzigSolver.hpp/cpp   # Boxed LCP + findex (pivoting, ODE-derived)
│   ├── LemkeSolver.hpp/cpp     # Standard LCP (boxed/findex delegates)
│   └── dantzig/                # Low-level ODE Dantzig implementation
│
├── projection/
│   ├── BgsSolver.hpp/cpp       # Blocked Gauss-Seidel
│   ├── JacobiSolver.hpp/cpp    # Projected Jacobi
│   ├── NncgSolver.hpp/cpp      # NNCG acceleration of PGS
│   ├── PgsSolver.hpp/cpp       # Boxed LCP + findex (iterative)
│   └── SubspaceMinimizationSolver.hpp/cpp  # PGS-SM hybrid
│
├── newton/                     # Minimum map, FB, penalized FB Newton
└── other/                      # Future solver families
```

See [Problem Statement](01_problem-statement.md) for the `w = Ax - b`
convention and the friction-index (`findex`) interpretation.

### Adding a Solver

1. Implement
   `dart::math::LcpSolver::solve(const LcpProblem&, Eigen::VectorXd&, const LcpOptions&)`.
2. Keep solver-specific parameters in a struct and reference it via
   `LcpOptions::customOptions` (see `PgsSolver::Parameters`).
3. Add unit tests under `tests/unit/math/lcp` (optionally benchmarks under
   `tests/benchmark/lcpsolver`).
4. Add the header to `dart/math/lcp/All.hpp` if it should be part of the
   public API.

### Usage Examples

See [Pivoting Methods](03_pivoting-methods.md) and
[Projection Methods](04_projection-methods.md) for `LcpProblem` setup and
solver usage examples.

### Currently Implemented Solvers

#### 1. Dantzig Principal Pivoting Method (`pivoting/DantzigSolver.hpp`)

- **Type**: Principal pivoting method for BLCP (Boxed Linear Complementarity
  Problem)
- **Algorithm**: Dantzig-Cottle principal pivoting
- **Source**: Derived from Open Dynamics Engine (ODE)
- **Named after**: George B. Dantzig (pioneer of linear programming and simplex method)
- **Features**:
  - Supports bounded variables (lo, hi)
  - Handles friction with `findex` parameter
  - Supports unbounded variables via ±∞ bounds
  - Early termination option
- **Use Case**: General BLCP problems with bounds, friction constraints

#### 2. Lemke Complementary Pivot Method (`pivoting/LemkeSolver.hpp`)

- **Type**: Complementary pivoting method for standard LCP
- **Algorithm**: Lemke's algorithm with artificial variable
- **Named after**: Carlton E. Lemke (developed complementary pivot theory)
- **Features**:
  - Standard LCP formulation: Mx = q + w, x >= 0, w >= 0, x^T w = 0
  - Validates solutions against LCP conditions
- **Use Case**: Standard LCP problems without bounds

#### 3. Jacobi (Projected) (`projection/JacobiSolver.hpp`)

- **Type**: Iterative projection method for boxed LCP
- **Algorithm**: Jacobi updates with projection onto `[lo, hi]`
- **Features**:
  - Uses the previous iterate for all updates (parallel-friendly)
  - Supports bounds and `findex` friction coupling
  - Optional damping via `LcpOptions::relaxation`
- **Use Case**: Parallel/GPU baselines and coarse approximations

#### 4. Projected Gauss-Seidel (PGS) (`projection/PgsSolver.hpp`)

- **Type**: Iterative projection method for boxed LCP
- **Algorithm**: Gauss-Seidel with projection onto `[lo, hi]` and friction index
  coupling
- **Features**:
  - Handles bounds and `findex` friction pyramids
  - Supports warm starts, optional relaxation (PSOR), and customizable sweep
    order
  - Early-out when the primary pivoting solver fails in ConstraintSolver
- **Use Case**: Real-time fallback for constraint solving where approximate
  solutions are acceptable

#### 5. Blocked Gauss-Seidel (BGS) (`projection/BgsSolver.hpp`)

- **Type**: Blocked projection method for boxed LCP
- **Algorithm**: Block Gauss-Seidel with per-block Dantzig solves
- **Features**:
  - Groups variables via `findex` by default (contact-style blocks)
  - Optional explicit block sizes via solver parameters
  - Shares bounds and friction index handling with PGS
- **Use Case**: Contact problems where per-contact blocks improve convergence

#### 6. NNCG (Nonsmooth Nonlinear Conjugate Gradient) (`projection/NncgSolver.hpp`)

- **Type**: Projection method with conjugate gradient acceleration
- **Algorithm**: NNCG using PGS sweeps as the nonlinear projection map
- **Features**:
  - Boxed LCP support (same bounds and `findex` handling as PGS)
  - Configurable restart interval and threshold
  - PGS-based warm start and projection
- **Use Case**: Large-scale problems needing faster convergence than PGS

#### 7. Subspace Minimization (PGS-SM) (`projection/SubspaceMinimizationSolver.hpp`)

- **Type**: Two-phase projection method for boxed LCP
- **Algorithm**: PGS for active set estimation + reduced solve on free set
- **Features**:
  - Uses PGS sweeps to estimate active constraints
  - Solves a reduced system for interior variables each iteration
  - Works with bounds and friction index coupling
- **Use Case**: Medium-scale problems where PGS converges slowly

#### 8. Minimum Map Newton (`newton/MinimumMapNewtonSolver.hpp`)

- **Type**: Newton method using the minimum map reformulation
- **Algorithm**: Active/free set Newton on `H(x) = min(x, Ax - b)`
- **Features**:
  - Standard LCP only (`lo = 0`, `hi = +inf`, `findex = -1`)
  - Boxed/findex problems delegate to the boxed-capable pivoting solver
- **Use Case**: High-accuracy solves for standard LCPs

#### 9. Fischer-Burmeister Newton (`newton/FischerBurmeisterNewtonSolver.hpp`)

- **Type**: Newton method using the Fischer-Burmeister function
- **Algorithm**: Smooth FB reformulation with line search
- **Features**:
  - Standard LCP only (`lo = 0`, `hi = +inf`, `findex = -1`)
  - Boxed/findex problems delegate to the boxed-capable pivoting solver
- **Use Case**: High-accuracy solves for standard LCPs

#### 10. Penalized Fischer-Burmeister Newton (`newton/PenalizedFischerBurmeisterNewtonSolver.hpp`)

- **Type**: Newton method using a penalized Fischer-Burmeister function
- **Algorithm**: FB reformulation with penalty term and line search
- **Features**:
  - Standard LCP only (`lo = 0`, `hi = +inf`, `findex = -1`)
  - Boxed/findex problems delegate to the boxed-capable pivoting solver
  - Penalty parameter (`lambda`) to tune convergence behavior
- **Use Case**: High-accuracy solves for standard LCPs with tunable penalty

## Introduction

Linear Complementarity Problems (LCPs) are fundamental to physics-based simulation, particularly in:

- Contact mechanics
- Rigid body dynamics
- Fluid simulation
- Constraint solving
- Optimization problems

### LCP Definition

The standard LCP is defined as:

```
Find x such that:
  Ax - b >= 0
  x >= 0
  x^T(Ax - b) = 0
```

where:

- `A` is an n×n matrix
- `b` is an n-dimensional vector
- `x` is the n-dimensional solution vector

The complementarity condition `x^T(Ax - b) = 0` means that for each index i, either `x_i = 0` or `(Ax - b)_i = 0`.

### Problem Variants

#### BLCP (Boxed LCP)

LCP with bounded variables:

```
Find x such that:
  l <= x <= u
  Ax - b complements x
```

where `l` and `u` are lower and upper bounds.

#### MLCP (Mixed LCP)

Combination of equality constraints and complementarity conditions.

## Solver Categories Overview

LCP solvers can be categorized into several main families:

### 1. [Pivoting Methods](03_pivoting-methods.md)

- **Direct** and **Incremental Pivoting**
- Exact solutions (if exists)
- Time: O(n^4), Storage: O(n^2)
- Best for: Small-medium problems, poorly conditioned matrices

### 2. [Projection/Sweeping Methods](04_projection-methods.md)

- **Jacobi**, **PGS**, **PSOR**, **Blocked Gauss-Seidel**, **NNCG**, **PGS-SM**
- Iterative with linear convergence
- Time: O(n) per iteration, Storage: O(n)
- Best for: Real-time simulation, interactive applications

### 3. [Newton Methods](05_newton-methods.md)

- **Minimum Map**, **Fischer-Burmeister**, **Penalized FB**
- Superlinear to quadratic convergence
- Time: O(n^3) or O(n) per iteration
- Best for: High accuracy, off-line simulation

### 4. [Other Methods](06_other-methods.md)

- **Interior Point**, **Staggering**, **Specialized Methods**
- Problem-specific approaches
- Various convergence properties

## Quick Selection Guide

| Use Case             | Recommended Method       | Reason                  |
| -------------------- | ------------------------ | ----------------------- |
| Real-time simulation | PGS, PSOR, BGS           | Fast O(n) iterations    |
| High accuracy        | Newton, Pivoting         | Superlinear convergence |
| Large-scale          | NNCG, PGS                | Scalable, matrix-free   |
| Poorly conditioned   | Pivoting, Interior Point | Numerically robust      |
| Contact mechanics    | BGS, Dantzig             | Natural block structure |
| Parallel computing   | Jacobi, Red-Black GS     | Embarrassingly parallel |

See [LCP Selection Guide](07_selection-guide.md) for detailed recommendations.

## Documentation Structure

- **[01_problem-statement.md](01_problem-statement.md)** - LCP definition, variants, and applications
- **[02_overview.md](02_overview.md)** (this file) - Implementation status and introduction
- **[03_pivoting-methods.md](03_pivoting-methods.md)** - Pivoting methods details
- **[04_projection-methods.md](04_projection-methods.md)** - Projection/sweeping methods details
- **[05_newton-methods.md](05_newton-methods.md)** - Newton methods details
- **[06_other-methods.md](06_other-methods.md)** - Interior Point, Staggering, and specialized methods
- **[07_selection-guide.md](07_selection-guide.md)** - Practical selection guidelines

## Numerical Properties Comparison

### Convergence Rates

| Method Category | Convergence Rate         | Iterations Needed |
| --------------- | ------------------------ | ----------------- |
| Pivoting        | Exact (finite)           | 1 (worst O(2^n))  |
| PGS/PSOR        | Linear                   | 50-500            |
| Blocked Methods | Linear                   | 50-500            |
| NNCG            | Linear to Superlinear    | 20-200            |
| Interior Point  | Superlinear              | 10-50             |
| Newton          | Superlinear to Quadratic | 5-20              |

### Computational Cost per Iteration

| Method         | Time               | Storage        | Notes                          |
| -------------- | ------------------ | -------------- | ------------------------------ | ---- | ---------------------- |
| Pivoting       | O(n^3)             | O(n^2)         | With incremental factorization |
| PGS/PSOR       | O(nk)\*            | O(n)           | k = max non-zeros per row      |
| BGS            | O(n·b^3)           | O(n)           | b = block size                 |
| NNCG           | O(n)               | O(n)           | Same as PGS                    |
| PGS-SM         | O(nk) + O(         | A              | ^3)                            | O(n) | Reduced subspace solve |
| Interior Point | O(n^3) or O(n)\*\* | O(n^2) or O(n) | Depends on solver              |
| Newton         | O(n^3) or O(n)\*\* | O(n^2) or O(n) | Depends on solver              |

\* k is typically small (< 10) for sparse problems
\*\* Direct solver vs iterative solver

### Matrix Requirements

| Method          | Requirements                                        |
| --------------- | --------------------------------------------------- |
| Dantzig/Lemke   | None (general LCP)                                  |
| Baraff Pivoting | Symmetric PD or PSD                                 |
| PGS/PSOR        | Non-zero diagonal (symmetric for convergence proof) |
| Newton Methods  | None (but active set should be non-singular)        |
| Interior Point  | None                                                |

## Implementation Roadmap

### Phase 1: Core Iterative Methods (High Priority)

- [x] Projected Gauss-Seidel (PGS) — `dart::math::PgsSolver`
- [x] Projected SOR (PSOR) — `dart::math::PgsSolver` via `LcpOptions::relaxation`
- [x] Projected Jacobi — `dart::math::JacobiSolver`
- [x] Basic termination criteria and merit functions (`dart/math/lcp/LcpValidation.hpp`)

### Phase 2: Blocked Methods (Medium Priority)

- [x] Blocked Gauss-Seidel (BGS)
- [ ] Per-contact block structure
- [ ] Direct 2D/3D sub-solvers

### Phase 3: Advanced Iterative (Medium Priority)

- [x] Nonsmooth Nonlinear Conjugate Gradient (NNCG)
- [x] Subspace Minimization (PGS-SM)
- [ ] Staggering methods

### Phase 4: Newton Methods (Low Priority)

- [x] Minimum Map Newton
- [x] Fischer-Burmeister Newton
- [x] Penalized Fischer-Burmeister Newton
- [x] Projected line search
- [ ] Nonsmooth gradient descent (warm start)

### Phase 5: Additional Methods (Future)

- [ ] Interior Point method
- [ ] Baraff incremental pivoting
- [ ] Specialized methods (shock propagation, etc.)

## References

### Key Papers and Books

1. **Cottle, R. W., Pang, J.-S., & Stone, R. E.** (1992). "The Linear Complementarity Problem"
2. **Baraff, D.** (1994). "Fast Contact Force Computation for Nonpenetrating Rigid Bodies"
3. **Erleben, K., et al.** (2017). "Numerical Methods for Linear Complementarity Problems in Physics-Based Animation"
4. **Fischer, A.** (1992). "A special Newton-type optimization method"
5. **Nocedal, J., & Wright, S.** (1999). "Numerical Optimization"

### Software Implementations

- **PATH**: Fischer-Burmeister with non-monotone line search
- **Num4LCP**: Research library with multiple solvers
- **Bullet Physics**: PGS and NNCG implementations
- **ODE**: Dantzig solver (basis for DART's implementation)
- **MOSEK, CPLEX**: Commercial QP solvers for symmetric PD matrices

## Contributing

When implementing new LCP solvers:

1. Update the implementation status table in this document
2. Add comprehensive documentation to the appropriate detailed page
3. Include algorithm pseudocode
4. Document convergence properties and use cases
5. Add unit tests with known solutions
6. Benchmark against existing solvers
7. Update the selection guide with practical recommendations

## Implementation References

- **Pivoting**: Active/free set formulation, 2D/3D direct solvers, and incremental Baraff pivoting (see `03_pivoting-methods.md`).
- **Projection**: Splitting derivations, PSOR update, BLCP projection, BGS block structure, PGS-SM, and NNCG pseudocode (see `04_projection-methods.md`).
- **Other**: Staggering details and friction QP notes (see `06_other-methods.md`).

---

**Last Updated**: 2025-11-22
