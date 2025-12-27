# Other LCP Solver Methods

**Navigation**: [← Newton Methods](05_newton-methods.md) | [Selection Guide →](07_selection-guide.md)

## Overview

This document covers additional LCP solver methods including Interior Point,
Staggering, and specialized methods that don't fit into the main categories of
pivoting, projection, or Newton methods.

Interior point is implemented in DART as `dart::math::InteriorPointSolver`.
Staggering is implemented as `dart::math::StaggeringSolver`.
Blocked Jacobi is implemented as `dart::math::BlockedJacobiSolver`.
MPRGP is implemented as `dart::math::MprgpSolver`.

## 1. Interior Point Method ✅ (Implemented)

### Description

Iterative method based on Kojima mapping that solves a relaxed complementarity problem while following a central path trajectory.

### Problem Reformulation

Instead of `x^T(Ax - b) = 0`, solve:

```
x^T(Ax - b) = mu  (relaxed complementarity)
where mu > 0 is a small parameter
```

### Kojima Mapping

```
F(x, y, mu) = [ Ax - y - b    ]
              [ X*Y*e - mu*e  ]
```

where:

- X = diag(x), Y = diag(y)
- e = vector of ones
- mu = centering parameter

### Algorithm

```
Initialize: x > 0, y > 0
for iter = 1 to max_iter:
  # Compute centering parameter
  mu = sigma * x^T * y / n
  where 0 < sigma < 1

  # Solve Newton equation for Kojima mapping
  [ A    -I  ] [ Δx ] = - [ Ax - y - b    ]
  [ Y     X  ] [ Δy ]     [ X*Y*e - mu*e  ]

  # Line search for step length
  t_x = max{t | x + t*Δx >= (1-alpha)*x}
  t_y = max{t | y + t*Δy >= (1-alpha)*y}
  t = min(t_x, t_y)

  # Update
  x = x + t*Δx
  y = y + t*Δy

  # Reduce mu toward 0
  mu = mu / (iter + 1)
```

### DART Implementation

```cpp
#include <dart/math/lcp/other/InteriorPointSolver.hpp>

using namespace dart::math;

LcpProblem problem(
    A,
    b,
    Eigen::VectorXd::Zero(n),
    Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(n, -1));

Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
InteriorPointSolver solver;
LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 50;
auto result = solver.solve(problem, x, options);
```

> Note: DART's implementation targets standard LCPs and delegates boxed or
> friction-indexed problems to the boxed-capable pivoting solver.

### Properties

- **Time**: O(n³) per iteration (or O(n) with iterative solver)
- **Storage**: O(n²) (or O(n) with iterative solver)
- **Convergence**: Superlinear
- **Matrix Requirements**: None (very general)

### Parameters

- **sigma** (centering): 0 < sigma < 1 (typically 0.1-0.5)
- **alpha** (step control): 0 < alpha < 1 (typically 0.99)

### Advantages/Disadvantages

✅ Very robust - works for general LCPs
✅ Guaranteed interior iterates (x, y > 0)
✅ Theoretically well-founded (central path)
✅ Handles ill-conditioned problems
❌ O(n³) cost per iteration (expensive)
❌ More complex than projection methods
❌ Requires careful parameter tuning

### Use Cases

- Very ill-conditioned problems
- When other methods fail
- Problems requiring guaranteed feasibility
- When computational budget allows

## 2. Staggering Methods ✅ (Implemented)

### Description

Partitions LCP into coupled sub-problems and solves them iteratively until convergence.

### Typical Partition for Contact Problems

```
Variables partitioned into:
  N - Normal forces
  F - Friction forces
  S - Slack variables

Coupled LCPs:
  A_NN * N + A_NF * F + A_NS * S = b_N
  A_FN * N + A_FF * F + A_FS * S = b_F
  A_SN * N + A_SF * F + A_SS * S = b_S
```

### Algorithm

```
Initialize: N, F, S
for iter = 1 to max_iter:
  # Solve for normal forces (keeping F, S fixed)
  N = solve_LCP(A_NN, b_N - A_NF*F - A_NS*S)

  # Solve for friction forces (keeping N, S fixed)
  F = solve_QP(A_FF, b_F - A_FN*N - A_FS*S, bounds=[-mu*N, mu*N])

  # Solve for slack variables (keeping N, F fixed)
  S = solve_LCP(A_SS, b_S - A_SN*N - A_SF*F)

  # Check convergence
  if ||change|| < epsilon:
    break
```

### Semi-Staggering

Only one iteration of staggering, used as warm-start:

```
# Initialize with PGS or zeros
x = initial_guess

# One staggering iteration
N = solve_normal(...)
F = solve_friction(...)

# Use as warm-start for Newton or other method
x_final = newton_method(x=[N, F])
```

### DART Implementation

```cpp
dart::math::StaggeringSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 50;
options.relaxation = 1.0;  // Optional damping
solver.solve(problem, x, options);
```

> Note: DART partitions variables by `findex` (normal indices are `findex < 0`,
> friction indices are `findex >= 0`) and alternates normal and friction sub-solves.

### Properties

- **Time**: Depends on sub-solvers
- **Convergence**: Problem-dependent, no general theory
- **Effectiveness**: Can dramatically improve convergence

### Sub-Solver Choices

- **Normal forces**: 1D LCP (direct) or iterative
- **Friction forces**: QP solver (cone constraints)
- **Slack variables**: Direct elimination or iterative

### Advantages/Disadvantages

✅ Can use different solvers for different blocks
✅ Exploits problem structure
✅ Can accelerate convergence dramatically
❌ No convergence guarantees
❌ May not converge for all problems
❌ Requires problem-specific partitioning

### Implementation notes for contact/friction

- Split variables into normal impulses `x_N`, friction impulses `x_F`, and (optionally) slack `x_S`.
- Normal subproblem often has a symmetric PSD matrix `A_NN` → can be solved by QP or PCG and produces `x_N`.
- Friction subproblem is a BLCP with bounds `|x_F| ≤ μ x_N`; when `A_FF` is symmetric PSD it is equivalent to the QP

  ```
  minimize 0.5 x_F^T A_FF x_F + c_F^T x_F
  subject to x_F ≥ 0,  c_N - e^T x_F ≥ 0
  ```

- Iterate: solve normal → update bounds for friction → solve friction (QP or small LCP) → repeat until fixed point. A single pass can also be used to warm-start Newton or PGS.

### Use Cases

- Contact problems with normal/tangential separation
- Warm-starting Newton methods
- Problems with natural block structure
- When standard methods converge slowly

## 3. Specialized Methods

### 3.1 Shock-Propagation Method ❌ (Not Implemented)

**Description**: Spatial blocking along gravity direction for fast shock wave propagation.

**Algorithm**:

```
# Divide contacts into layers based on height
layers = compute_layers_by_gravity_direction(contacts)

# Solve layer by layer from bottom to top
for layer in layers (bottom to top):
  # Solve all contacts in this layer
  for contact in layer:
    solve_contact_LCP(contact)

  # Forces propagate to next layer
```

**Properties**:

- Domain: Velocity-based BLCP
- Structure: Gravity-aligned layers
- Convergence: Fast for stacking scenarios

**Use Cases**:

- Tower/wall simulations
- Stacking scenarios
- Gravity-dominated scenes
- High-fidelity contact propagation

### 3.2 Modified Proportioning with Reduced-Gradient Projections (MPRGP) ✅

**Description**: QP solver method using proportioning and reduced-gradient projections.

**Application**: Fluid simulation (when LCP reformulated as QP)

**Properties**:

- Requires: A symmetric positive definite
- Convergence: Monotone descent
- Use: Fluid problems

**Algorithm Sketch**:

```
while not converged:
  # Proportioning step
  Compute proportioning direction

  # Reduced-gradient projection
  Project onto feasible set

  # Line search
  Find step length

  # Update
  x = x + t * direction
```

### DART Implementation

```cpp
#include <dart/math/lcp/other/MprgpSolver.hpp>

using namespace dart::math;

LcpProblem problem(
    A,
    b,
    Eigen::VectorXd::Zero(n),
    Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(n, -1));

Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
MprgpSolver solver;
LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 200;
auto result = solver.solve(problem, x, options);
```

> Note: DART's implementation targets standard LCPs with symmetric positive
> definite matrices and delegates boxed or friction-indexed problems to the
> boxed-capable pivoting solver.

## 4. Blocked Jacobi ✅ (Implemented)

### Description

Jacobi splitting applied to blocks (similar to Blocked Gauss-Seidel but with Jacobi update).

### Algorithm

```
# All blocks updated in parallel
for iter = 1 to max_iter:
  parallel for each block i:
    r_i = b_i + sum(A_{ij} * x_j^k, j != i)
    x_i^{k+1} = SolveSubLCP(A_{ii}, r_i)
```

### DART Implementation

```cpp
#include <dart/math/lcp/projection/BlockedJacobiSolver.hpp>

using namespace dart::math;

LcpProblem problem(A, b, lo, hi, findex);
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());
BlockedJacobiSolver solver;
LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 200;
auto result = solver.solve(problem, x, options);
```

> Note: Block partitions follow `findex` by default (contact blocks), or can be
> set explicitly via `BlockedJacobiSolver::Parameters::blockSizes`.
> DART uses `DirectSolver` for standard blocks up to 3 variables and falls back
> to `DantzigSolver` for boxed or larger blocks.

### Properties

- **Parallelization**: Fully parallel (all blocks independent)
- **Convergence**: Slower than BGS, faster than scalar Jacobi
- **Use**: Parallel computing, GPU

## Comparison Summary

| Method            | Convergence | Complexity | Robustness | Parallelization |
| ----------------- | ----------- | ---------- | ---------- | --------------- |
| Interior Point ✅ | Superlinear | O(n³)      | Very High  | No              |
| Staggering ✅     | Variable    | Depends    | Medium     | No              |
| Shock-Propagation | Linear      | O(n)       | Medium     | Limited         |
| MPRGP ✅          | Monotone    | O(n²)      | High       | No              |
| Blocked Jacobi ✅ | Linear      | O(n·b³)    | Medium     | Yes             |

## Implementation Priority

### Implemented in DART

- **Interior Point**: Primal-dual path-following for standard LCPs
- **Staggering**: Normal/friction block solve for contact-style problems
- **Blocked Jacobi**: Parallel block updates with per-block LCP solves
- **MPRGP**: Bound-constrained QP solver for standard SPD LCPs

### Low Priority (Specialized Use Cases)

- **Shock-Propagation**: For gravity-dominated scenarios

### Very Low Priority (Niche Applications)

- Additional specialized QP methods

## When to Use These Methods

### Interior Point

**Use when**:

- All other methods fail to converge
- Very ill-conditioned problems
- Robustness is paramount

**Avoid when**:

- Real-time constraints (too expensive)
- Simpler methods work fine

### Staggering

**Use when**:

- Natural problem partition exists
- Need to warm-start Newton methods
- Standard methods converge slowly

**Avoid when**:

- Problem has no natural structure
- Convergence guarantees needed

### Specialized Methods

**Use when**:

- Problem matches specific structure (e.g., gravity layers)
- Domain-specific optimizations beneficial

**Avoid when**:

- General-purpose solver needed
- Problem doesn't match assumptions

## External Solver Integration

### QP Solvers for Symmetric PD Matrices

When A is symmetric PD, LCP can be reformulated as QP:

```
minimize: 0.5 * x^T * A * x + x^T * b
subject to: x >= 0
```

**Available Solvers**:

- **MOSEK** (Commercial): Very robust, supports large-scale
- **CPLEX** (Commercial): IBM optimizer, industry standard
- **LANCELOT** (Free): Academic use, Fortran-based
- **SQP**: Sequential Quadratic Programming
- **SNOPT**: Sparse Nonlinear Optimizer

**Integration Strategy**:

```cpp
// Wrapper for external QP solver
class QPSolverWrapper {
  bool solve(const MatrixXd& A, const VectorXd& b, VectorXd* x) {
    // Convert to QP format
    // Call external solver
    // Convert solution back
  }
};
```

## Future Work

### Multigrid Methods

Potential for O(n) convergence with multilevel hierarchy:

```
Coarse grid -> Medium grid -> Fine grid
```

**Benefits**:

- Faster convergence than single-level
- Preserve robustness of splitting methods

**Challenges**:

- Complex implementation
- Problem-specific grid structure

### Adaptive Methods

Dynamic method selection based on problem characteristics:

```
if (well_conditioned):
  use PGS
elif (ill_conditioned):
  use Pivoting or Interior Point
elif (small_problem):
  use Newton
```

## References

### Interior Point Methods

1. **Kojima, M., Megiddo, N., Noma, T., & Yoshise, A.** (1991). _A unified approach to interior point algorithms for linear complementarity problems_. Springer Science & Business Media.
   - Kojima mapping and central path
   - Path-following algorithms
   - Theoretical foundations

2. **Wright, S. J.** (1997). _Primal-dual interior-point methods_. SIAM.
   - Comprehensive treatment of interior point methods
   - Chapter 9: Complementarity problems
   - Implementation details

3. **Kojima, M., Megiddo, N., & Noma, T.** (1989). "Homotopy continuation methods for nonlinear complementarity problems". _Mathematics of Operations Research_, 14(3), 404-416.
   - Path-following for NCP/LCP
   - Centering parameter selection
   - Global convergence

4. **Monteiro, R. D., & Pang, J. S.** (1996). "A potential reduction Newton method for constrained equations". _SIAM Journal on Optimization_, 9(3), 729-754.
   - Potential reduction approach
   - Newton systems in interior point
   - Applications to LCP

5. **Ye, Y.** (1997). _Interior point algorithms: theory and analysis_. John Wiley & Sons.
   - Theory of interior point methods
   - Polynomial-time algorithms
   - Linear and nonlinear complementarity

### Staggering Methods

6. **Kaufman, D. M., Sueda, S., James, D. L., & Pai, D. K.** (2008). "Staggered projections for frictional contact in multibody systems". _ACM Transactions on Graphics (TOG)_, 27(5), 1-11.
   - Staggering for normal/tangential forces
   - Convergence analysis
   - Application to cloth and rigid bodies

7. **Otaduy, M. A., & Gross, M.** (2007). "Interactive design and analysis of deformable materials using force density meshes". _ACM Transactions on Graphics (TOG)_, 26(3), 14.
   - Staggering for constraint satisfaction
   - Warm-starting techniques

8. **Tournier, M., Nesme, M., Gilles, B., & Faure, F.** (2015). "Stable constrained dynamics". _ACM Transactions on Graphics (TOG)_, 34(4), 1-10.
   - Semi-staggering for stability
   - Partitioning strategies

### Shock-Propagation Method

9. **Guendelman, E., Bridson, R., & Fedkiw, R.** (2003). "Nonconvex rigid bodies with stacking". _ACM transactions on graphics (TOG)_, 22(3), 871-878.
   - Original shock propagation method
   - Layer-based spatial decomposition
   - Gravity-aligned blocking

10. **Pabst, S., Koch, A., & Straßer, W.** (2010). "Fast and scalable CPU/GPU collision detection for rigid and deformable surfaces". In _Computer Graphics Forum_, 29(5), 1605-1612.
    - Parallel shock propagation
    - Spatial layering on GPU

11. **Tonge, R., Benevolenski, F., & Voroshilov, A.** (2012). "Mass splitting for jitter-free parallel rigid body simulation". _ACM Transactions on Graphics (TOG)_, 31(4), 1-8.
    - Related parallel propagation
    - Domain decomposition for contacts

### MPRGP (Modified Proportioning with Reduced-Gradient Projections)

12. **Dostál, Z.** (2009). _Optimal quadratic programming algorithms: with applications to variational inequalities_. Springer Science & Business Media.
    - MPRGP algorithm
    - Proportioning and projections
    - Applications to contact and fluid problems

13. **Dostál, Z., & Schöberl, J.** (2005). "Minimizing quadratic functions subject to bound constraints with the rate of convergence and finite termination". _Computational Optimization and Applications_, 30(1), 23-43.
    - Convergence analysis of MPRGP
    - Finite termination properties

14. **Dostál, Z., Kozubek, T., Markopoulos, A., & Brzobohatý, T.** (2010). "Scalable FETI algorithms for large 3D multibody contact problems". In _Parallel Processing and Applied Mathematics_, 312-321. Springer.
    - MPRGP for large-scale contact
    - FETI domain decomposition

### Blocked Jacobi

15. **Adams, L. M.** (1982). "Iterative algorithms for large sparse linear systems on parallel computers". _NASA Technical Memorandum_, 83267.
    - Block Jacobi for parallelization
    - Applications to sparse systems

16. **Schwarz, H. A.** (1870). "Über einige Abbildungsaufgaben". _Journal für die reine und angewandte Mathematik_, 70, 105-120.
    - Original Schwarz alternating method
    - Historical foundation for domain decomposition

### External QP Solvers

17. **MOSEK ApS** (2023). "MOSEK Optimization Suite". https://www.mosek.com/
    - Commercial QP/LCP solver
    - Interior point methods
    - Large-scale optimization

18. **IBM ILOG** (2023). "CPLEX Optimizer". https://www.ibm.com/products/ilog-cplex-optimization-studio
    - Commercial optimization suite
    - Simplex and barrier methods
    - Mixed-integer support

19. **Conn, A. R., Gould, N. I., & Toint, P. L.** (2000). _Trust region methods_. SIAM.
    - LANCELOT algorithm (Chapter 17)
    - Large-scale nonlinear optimization
    - Free for academic use

20. **Gill, P. E., Murray, W., & Saunders, M. A.** (2005). "SNOPT: An SQP algorithm for large-scale constrained optimization". _SIAM review_, 47(1), 99-131.
    - Sequential Quadratic Programming
    - Sparse nonlinear optimization
    - Applications to complementarity

### Multigrid Methods (Future Work)

21. **Brandt, A.** (1977). "Multi-level adaptive solutions to boundary-value problems". _Mathematics of computation_, 31(138), 333-390.
    - Original multigrid method
    - V-cycle and W-cycle
    - Potential for O(n) LCP solvers

22. **Hackbusch, W.** (1985). _Multi-grid methods and applications_. Springer Science & Business Media.
    - Comprehensive multigrid theory
    - Applications to various PDEs
    - Adaptation to LCP

23. **Gratton, S., Sartenaer, A., & Toint, P. L.** (2008). "Recursive trust-region methods for multiscale nonlinear optimization". _SIAM Journal on Optimization_, 19(1), 414-444.
    - Multiscale optimization
    - Trust regions on different scales
    - Related to multigrid ideas

### Applications and Comparisons

24. **Erleben, K., Silcowitz-Hansen, M., & Niebe, S.** (2017). "Numerical methods for linear complementarity problems in physics-based animation". _Synthesis Lectures on Computer Graphics and Animation_, 11(2), 1-159.
    - Comprehensive survey including all methods
    - Performance comparisons
    - Implementation recommendations

25. **Tasora, A., Negrut, D., & Anitescu, M.** (2008). "Large-scale parallel multi-body dynamics with frictional contact on the graphical processing unit". _Proceedings of the Institution of Mechanical Engineers, Part K: Journal of Multi-body Dynamics_, 222(4), 315-326.
    - GPU implementations of various methods
    - Parallel Jacobi and staggering
    - Performance analysis

26. **Mazhar, H., Heyn, T., Negrut, D., & Tasora, A.** (2013). "Using Nesterov's method to accelerate multibody dynamics with friction and contact". _ACM Transactions on Graphics (TOG)_, 34(3), 1-14.
    - Acceleration techniques
    - Comparison with standard methods
    - Applications to granular materials

27. **Todorov, E., Erez, T., & Tassa, Y.** (2012). "MuJoCo: A physics engine for model-based control". In _2012 IEEE/RSJ International Conference on Intelligent Robots and Systems_, 5026-5033. IEEE.
    - PGS and Newton-Raphson in MuJoCo
    - Practical solver selection
    - Real-world applications

---

**Navigation**: [← Newton Methods](05_newton-methods.md) | [Selection Guide →](07_selection-guide.md)
