# LCP Solver Architecture

This directory contains the Linear Complementarity Problem (LCP) solver infrastructure for DART.

## Directory Structure

```
lcp/
├── Types.hpp/cpp               # Common types (LCPOptions, LCPResult, SolverStatus)
├── Solver.hpp/cpp              # Base LCPSolver interface class
├── All.hpp                     # Convenience header including all solvers
│
├── Lemke.hpp/cpp               # Legacy Lemke solver (backward compatibility)
├── ODELCPSolver.hpp/cpp        # Legacy ODE-based solver (backward compatibility)
│
├── dantzig/                    # Dantzig principal pivoting method (from ODE)
│   ├── Lcp.hpp/cpp
│   ├── Matrix.hpp
│   └── ...
│
├── pivoting/                   # Pivoting methods
│   └── LemkeSolver.hpp/cpp     # Modern Lemke wrapper
│
├── projection/                 # Projection/sweeping methods (future)
│   ├── PGSSolver.hpp/cpp       # Projected Gauss-Seidel (planned)
│   ├── PSORSolver.hpp/cpp      # Projected SOR (planned)
│   └── ...
│
├── newton/                     # Newton methods (future)
│   ├── MinimumMapSolver.hpp    # Minimum map Newton (planned)
│   └── ...
│
└── other/                      # Other methods (future)
    └── ...
```

## Architecture

### Version 1 (Legacy API - Backward Compatibility)

The legacy API is preserved for backward compatibility:

```cpp
// Legacy Lemke solver
#include <dart/math/lcp/Lemke.hpp>
int exitCode = dart::math::Lemke(M, q, &z);
bool valid = dart::math::validate(M, z, q);

// Legacy Dantzig solver
#include <dart/math/lcp/dantzig/Lcp.hpp>
bool success = dart::math::SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);

// Legacy ODE solver
#include <dart/math/lcp/ODELCPSolver.hpp>
dart::math::ODELCPSolver solver;
solver.Solve(A, b, &x, numContacts, mu, numDir, true);
```

### Version 2 (Modern API - Recommended for New Code)

The new API provides a consistent interface across all solvers:

```cpp
#include <dart/math/lcp/Types.hpp>
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

// Create solver
auto solver = std::make_shared<dart::math::LemkeSolver>();

// Configure options
dart::math::LCPOptions options;
options.validateSolution = true;
options.absoluteTolerance = 1e-8;

// Solve
Eigen::VectorXd x(n);
dart::math::LCPResult result = solver->solve(A, b, x, options);

// Check result
if (result.succeeded()) {
    std::cout << "Solution found in " << result.iterations << " iterations\n";
    std::cout << "Residual: " << result.residual << "\n";
    std::cout << "Complementarity error: " << result.complementarity << "\n";
} else {
    std::cerr << "Solver failed: " << dart::math::toString(result.status) << "\n";
    std::cerr << result.message << "\n";
}
```

## Common Types

### SolverStatus

Exit status codes for solvers:

```cpp
enum class SolverStatus {
    Success,           // Solution found successfully
    Failed,            // Failed to find a solution
    MaxIterations,     // Maximum iterations reached
    NumericalError,    // Numerical issues (NaN, Inf, etc.)
    InvalidProblem,    // Invalid problem formulation
    Degenerate,        // Solution is degenerate
    NotSolved          // Not yet solved
};
```

### LCPResult

Result structure returned by all solvers:

```cpp
struct LCPResult {
    SolverStatus status;      // Exit status
    int iterations;           // Number of iterations performed
    double residual;          // Final residual norm
    double complementarity;   // Complementarity error: ||x * (Ax + b)||
    bool validated;           // Whether solution was validated
    std::string message;      // Additional information

    bool succeeded() const;   // Check if solution was successful
};
```

### LCPOptions

Configuration options for solvers:

```cpp
struct LCPOptions {
    int maxIterations;                  // Maximum iterations (0 = solver default)
    double absoluteTolerance;           // Absolute convergence tolerance
    double relativeTolerance;           // Relative convergence tolerance
    double complementarityTolerance;    // Complementarity tolerance
    bool validateSolution;              // Validate solution after solving
    double relaxation;                  // Relaxation parameter (for SOR)
    bool warmStart;                     // Use warm starting
    bool earlyTermination;              // Early termination (for pivoting)
    void* customOptions;                // Solver-specific options

    // Factory methods
    static LCPOptions withRelaxation(double relaxation, int maxIter = 100);
    static LCPOptions highAccuracy();
    static LCPOptions realTime();
};
```

## Base Solver Interface

All modern solvers inherit from `LCPSolver`:

```cpp
class LCPSolver {
public:
    virtual ~LCPSolver() = default;

    // Solve with default options
    virtual LCPResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        Eigen::VectorXd& x);

    // Solve with custom options
    virtual LCPResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        Eigen::VectorXd& x,
        const LCPOptions& options) = 0;

    virtual std::string getName() const = 0;
    virtual std::string getCategory() const = 0;

    virtual LCPOptions getDefaultOptions() const;
    virtual void setDefaultOptions(const LCPOptions& options);

protected:
    LCPOptions defaultOptions_;
};
```

## Implemented Solvers

### Pivoting Methods

#### LemkeSolver (dart::math::LemkeSolver)

Lemke's complementary pivot algorithm for standard LCP.

**Properties**:

- Algorithm: Lemke's complementary pivoting with artificial variable
- Time: O(n^4) worst case
- Space: O(n^2)
- Convergence: Exact solution (finite termination)
- Matrix Requirements: None (general LCP)
- Named after: Carlton E. Lemke

**Usage**:

```cpp
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

auto solver = std::make_shared<dart::math::LemkeSolver>();
LcpResult result = solver->solve(A, b, x);
```

#### Dantzig Principal Pivoting (dart::math::SolveLCP template function)

Dantzig-Cottle principal pivoting method for BLCP (Boxed LCP) with friction support.

**Properties**:

- Algorithm: Principal pivoting method
- Time: O(n^4) worst case
- Space: O(n^2)
- Supports: Bounded variables, friction cones
- Source: Open Dynamics Engine (ODE)
- Named after: George B. Dantzig

**Usage**:

```cpp
#include <dart/math/lcp/dantzig/Lcp.hpp>

bool success = dart::math::SolveLCP<double>(
    n, A, x, b, w, nub, lo, hi, findex, earlyTermination);
```

### Utility Classes

#### ODELCPSolver (dart::math::ODELCPSolver)

Convenience wrapper for contact mechanics problems. Uses Dantzig principal pivoting internally.

**Usage**:

```cpp
#include <dart/math/lcp/ODELCPSolver.hpp>

dart::math::ODELCPSolver solver;
bool success = solver.Solve(A, b, &x, numContacts, mu, numDir, true);
```

## Planned Solvers

### Projection Methods (projection/)

- **PGSSolver**: Projected Gauss-Seidel (O(n) per iteration)
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

**Before (v1)**:

```cpp
#include <dart/math/lcp/Lemke.hpp>

Eigen::VectorXd z(n);
int exitCode = dart::math::Lemke(M, q, &z);
if (exitCode == 0) {
    bool valid = dart::math::validate(M, z, q);
    // Use solution...
}
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
#include <dart/math/lcp/dantzig/Lcp.hpp>
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
3. **Inherit from LCPSolver** base class
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

class MyNewSolver : public LCPSolver {
public:
    MyNewSolver();
    ~MyNewSolver() override = default;

    LCPResult solve(
        const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b,
        Eigen::VectorXd& x,
        const LCPOptions& options) override;

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
- `ODELCPSolver` class

New code should prefer the modern API (v2) for consistency and future extensibility.
