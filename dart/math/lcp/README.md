# LCP Solvers in DART

This directory contains the Linear Complementarity Problem (LCP) solver
infrastructure for DART.

## Directory Structure

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
│   └── PgsSolver.hpp/cpp       # Boxed LCP + findex (iterative)
│
├── newton/                     # Future Newton methods
└── other/                      # Future solver families
```

## Problem Convention

DART uses:

```text
w = Ax - b
lo <= x <= hi
boxed complementarity (KKT) conditions
```

Many references write `w = Ax + q`; the mapping is `b = -q`.

## Friction Index (`findex`)

For contact-style friction coupling, a row `i` can reference a "normal"
variable `j = findex[i] >= 0`. The bounds are interpreted as:

```text
hiEff[i] = |hi[i] * x[j]|
loEff[i] = -hiEff[i]
```

To encode `|x[i]| <= mu * x[j]`, store `lo[i] = -mu`, `hi[i] = +mu`,
`findex[i] = j`.

## Usage

```cpp
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>

dart::math::LcpProblem problem(A, b, lo, hi, findex);
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());

dart::math::DantzigSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.validateSolution = true;

dart::math::LcpResult result = solver.solve(problem, x, options);
```

`dart::math::PgsSolver` uses the same `LcpProblem` interface and supports
PSOR-style relaxation via `LcpOptions::relaxation`.

`dart::math::LemkeSolver` implements the standard LCP Lemke algorithm and
delegates boxed/findex problems to `dart::math::DantzigSolver`. For the common
form `w = Mz + q`, construct `LcpProblem` with `b = -q`.

## Testing, Benchmarks, and References

- Unit tests: `tests/unit/math/lcp`
- Benchmarks: `tests/benchmark/lcpsolver`
- Background docs: `docs/background/lcp`

## Adding a New Solver

1. Add headers/sources under `pivoting/`, `projection/`, `newton/`, or `other/`.
2. Implement
   `dart::math::LcpSolver::solve(const LcpProblem&, Eigen::VectorXd&, const LcpOptions&)`.
3. Add unit tests under `tests/unit/math/lcp` (and optionally a benchmark).
4. Add your solver header to `dart/math/lcp/All.hpp` if appropriate.
