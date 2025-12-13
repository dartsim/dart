# LCP Solvers in DART

This directory contains the Linear Complementarity Problem (LCP) solver
infrastructure for DART.

## Directory Structure

```
dart/math/lcp/
├── LcpTypes.hpp/cpp            # LcpProblem, LcpOptions, LcpResult, status codes
├── LcpSolver.hpp/cpp           # LcpSolver interface
├── LcpValidation.hpp           # Shared residual/KKT validation utilities
├── All.hpp                     # Convenience umbrella header
│
├── pivoting/
│   ├── DantzigSolver.hpp/cpp   # Boxed LCP + findex (pivoting, ODE-derived)
│   ├── LemkeSolver.hpp/cpp     # Standard LCP only (no box bounds)
│   └── dantzig/                # Low-level ODE Dantzig implementation
│       ├── Lcp.hpp
│       └── ...
│
├── projection/
│   ├── PgsSolver.hpp/cpp       # Boxed LCP + findex (iterative)
│   └── ...
│
├── newton/                     # Future Newton methods
└── other/                      # Future solver families
```

## Problem Convention

DART uses the convention:

```
w = Ax - b
lo <= x <= hi
(x, w) satisfy boxed complementarity (KKT) conditions
```

Many references write `w = Ax + q`; the mapping is `b = -q`.

## Friction Index (`findex`)

For contact-style friction coupling, a row `i` can reference a "normal"
variable `j = findex[i] >= 0`. The bounds are interpreted as:

```
hiEff[i] = |hi[i] * x[j]|
loEff[i] = -hiEff[i]
```

To encode `|x[i]| <= mu * x[j]`, store `lo[i] = -mu`, `hi[i] = +mu`,
`findex[i] = j`.

## Basic Usage

```cpp
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>

dart::math::DantzigSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.validateSolution = true;

dart::math::LcpProblem problem(A, b, lo, hi, findex);
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());
dart::math::LcpResult result = solver.solve(problem, x, options);
```

## Testing and Benchmarks

- Unit tests: `tests/unit/math/lcp`
- Benchmarks: `tests/benchmark/lcpsolver`

## Adding a New Solver

1. Add headers/sources under `pivoting/`, `projection/`, `newton/`, or `other/`.
2. Implement `dart::math::LcpSolver::solve(const LcpProblem&, Eigen::VectorXd&, const LcpOptions&)`.
3. Add unit tests under `tests/unit/math/lcp`.
4. Optionally add a benchmark case under `tests/benchmark/lcpsolver`.
`options.customOptions` with a `PgsSolver::Parameters` instance.

## Planned Solvers

### Projection Methods (projection/)

- **PSORSolver**: Projected SOR with relaxation parameter
- **BGSSolver**: Blocked Gauss-Seidel for contact problems
- **NNCGSolver**: Nonsmooth Nonlinear Conjugate Gradient

### Newton Methods (newton/)

- **MinimumMapSolver**: Minimum map Newton method
- **FischerBurmeisterSolver**: Fischer-Burmeister Newton method

### Other Methods (other/)

- **InteriorPointSolver**: Interior point method
- **StaggeringSolver**: Staggering for coupled sub-problems

## Migration Guide

### From Legacy Lemke to Modern API

**Before (v1)** (deprecated):

```cpp
// Legacy free-function API removed; use the class-based solver instead.
```

**After (v2)**:

```cpp
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

auto solver = std::make_shared<dart::math::LemkeSolver>();
Eigen::VectorXd z(n);
auto result = solver->solve(M, q, z);
if (result.succeeded() && result.validated) {
    // Use solution...
}
```

### From Legacy Dantzig to Modern API

The Dantzig principal pivoting solver currently only has a template function API.

```cpp
// Current (v1) - still recommended for BLCP
#include <dart/math/lcp/pivoting/dantzig/Lcp.hpp>
bool success = dart::math::SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);

// Future (v2) - planned
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>
auto solver = std::make_shared<dart::math::DantzigSolver>();
auto result = solver->solve(A, b, x, options);
```

## Adding New Solvers

To add a new solver:

1. **Choose the category**: pivoting, projection, newton, or other
2. **Create header and source files** in the appropriate subdirectory
3. **Inherit from LcpSolver** base class
4. **Implement required virtual methods**:
   - `solve(A, b, x, options)`
   - `getName()`
   - `getCategory()`
5. **Add to All.hpp** for convenience
6. **Write unit tests** in `tests/unit/math/`
7. **Document** algorithm, properties, and usage

### Example Template

```cpp
// pivoting/MyNewSolver.hpp
#pragma once

#include "dart/math/lcp/Solver.hpp"

namespace dart::math::pivoting {

class MyNewSolver : public LcpSolver {
public:
    MyNewSolver();
    ~MyNewSolver() override = default;

    LcpResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        Eigen::VectorXd& x,
        const LcpOptions& options) override;

    std::string getName() const override { return "MyNewSolver"; }
    std::string getCategory() const override { return "Pivoting"; }
};

} // namespace dart::math::pivoting
```

## Testing

All solvers should have comprehensive tests:

```cpp
#include <gtest/gtest.h>
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

TEST(LemkeSolver, SimpleExample) {
    auto solver = std::make_shared<dart::math::LemkeSolver>();

    // Set up problem...
    Eigen::MatrixXd A(2, 2);
    Eigen::VectorXd b(2);
    Eigen::VectorXd x(2);

    // Solve
    auto result = solver->solve(A, b, x);

    // Verify
    EXPECT_TRUE(result.succeeded());
    EXPECT_LT(result.complementarity, 1e-6);
}
```

## References

See comprehensive documentation in `/docs/background/lcp/`:

- `01_problem-statement.md` - LCP formulation and applications
- `02_overview.md` - Implementation status and overview
- `03_pivoting-methods.md` - Pivoting algorithms
- `04_projection-methods.md` - Projection/sweeping algorithms
- `05_newton-methods.md` - Newton-based algorithms
- `06_other-methods.md` - Interior point and specialized methods
- `07_selection-guide.md` - Practical selection guidelines

## Backward Compatibility

All legacy APIs remain fully functional:

- `Lemke(M, q, z)` and `validate(M, z, q)` functions
- `SolveLCP<Scalar>()` template function

New code should prefer the modern API (v2) for consistency and future extensibility.
