# Other LCP Solver Methods

> **Attribution**: This content is derived from "Contact Handling for Articulated
> Rigid Bodies Using LCP" by Jie Tan, Kristin Siu, and C. Karen Liu.
> The original PDF is preserved at [`docs/lcp.pdf`](../../lcp.pdf).

**Navigation**: [← Newton Methods](05_newton-methods.md) | [Index](../README.md) | [Selection Guide →](07_selection-guide.md)

## Overview

This document covers additional LCP solver methods including Interior Point,
Staggering, and specialized methods that don't fit into the main categories of
pivoting, projection, or Newton methods.

Interior point is implemented in DART as `dart::math::InteriorPointSolver`.
Staggering is implemented as `dart::math::StaggeringSolver`.
Blocked Jacobi is implemented as `dart::math::BlockedJacobiSolver`.
MPRGP is implemented as `dart::math::MprgpSolver`.
Shock propagation is implemented as `dart::math::ShockPropagationSolver`.
ADMM is implemented as `dart::math::AdmmSolver`.
SAP is implemented as `dart::math::SapSolver`.

## 1. Interior Point Method ✅ (Implemented)

### Description

Iterative method based on Kojima mapping that solves a relaxed complementarity problem while following a central path trajectory.

### Problem Reformulation

Instead of $x^T(Ax - b) = 0$, solve:

$$x^T(Ax - b) = \mu \quad \text{(relaxed complementarity)}$$

where $\mu > 0$ is a small parameter.

### Kojima Mapping

$$F(x, y, \mu) = \begin{bmatrix} Ax - y - b \\ XYe - \mu e \end{bmatrix}$$

where:

- $X = \text{diag}(x)$, $Y = \text{diag}(y)$
- $e$ = vector of ones
- $\mu$ = centering parameter

### Algorithm

Initialize: $x > 0$, $y > 0$

For iter $= 1$ to max_iter:

1. Compute centering parameter: $\mu = \sigma \cdot x^T y / n$ where $0 < \sigma < 1$

2. Solve Newton equation for Kojima mapping:
   $$\begin{bmatrix} A & -I \\ Y & X \end{bmatrix} \begin{bmatrix} \Delta x \\ \Delta y \end{bmatrix} = -\begin{bmatrix} Ax - y - b \\ XYe - \mu e \end{bmatrix}$$

3. Line search for step length:
   $$t_x = \max\{t \mid x + t\Delta x \geq (1-\alpha)x\}$$
   $$t_y = \max\{t \mid y + t\Delta y \geq (1-\alpha)y\}$$
   $$t = \min(t_x, t_y)$$

4. Update: $x = x + t\Delta x$, $y = y + t\Delta y$

5. Reduce $\mu$ toward 0: $\mu = \mu / (\text{iter} + 1)$

### DART Implementation

```cpp
#include <dart/math/lcp/other/interior_point_solver.hpp>

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
> Strictly interior standard LCPs first try the shared validated linear-solve
> fast path. This path is accepted only when the candidate is strictly positive
> and passes the LCP solution validator; otherwise the path-following solver
> runs normally.

DART 7 benchmark evidence includes `BM_LcpInteriorPointPathSweep`, which
compares centering parameter $\sigma=0.1/0.3$ and step scales 0.75/0.99 over
well-conditioned dense SPD, banded SPD, mildly ill-conditioned SPD, and
near-singular SPD standard-LCP fixtures. Focused default, SIMD-enabled, and
CUDA-enabled build-tree runs reported 9 rows with `contract_ok=1`; the
CUDA-enabled rows are CPU solver rows in a CUDA-enabled build, not CUDA LCP
kernel execution.
`BM_LcpContactNormalStandardSweep` adds contact-derived standard-LCP evidence
for Interior Point by extracting only the normal rows from DART 7 separated
sphere-ground, coupled vertical-stack, and articulated unified-contact
snapshots. This does not claim native boxed or friction-index support for the
standard-only Interior Point solver.

### Properties

- **Time**: $O(n^3)$ per iteration (or $O(n)$ with iterative solver)
- **Storage**: $O(n^2)$ (or $O(n)$ with iterative solver)
- **Convergence**: Superlinear
- **Matrix Requirements**: None (very general)

### Parameters

- **$\sigma$** (centering): $0 < \sigma < 1$ (typically 0.1-0.5)
- **$\alpha$** (step control): $0 < \alpha < 1$ (typically 0.99)

### Advantages/Disadvantages

✅ Very robust - works for general LCPs
✅ Guaranteed interior iterates ($x, y > 0$)
✅ Theoretically well-founded (central path)
✅ Handles ill-conditioned problems
❌ $O(n^3)$ cost per iteration (expensive)
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

Variables partitioned into:

- $N$ - Normal forces
- $F$ - Friction forces
- $S$ - Slack variables

Coupled LCPs:
$$A_{NN} N + A_{NF} F + A_{NS} S = b_N$$
$$A_{FN} N + A_{FF} F + A_{FS} S = b_F$$
$$A_{SN} N + A_{SF} F + A_{SS} S = b_S$$

### Algorithm

Initialize: $N$, $F$, $S$

For iter $= 1$ to max_iter:

1. Solve for normal forces (keeping $F$, $S$ fixed):
   $$N = \text{solve\_LCP}(A_{NN}, b_N - A_{NF}F - A_{NS}S)$$

2. Solve for friction forces (keeping $N$, $S$ fixed):
   $$F = \text{solve\_QP}(A_{FF}, b_F - A_{FN}N - A_{FS}S, \text{bounds}=[-\mu N, \mu N])$$

3. Solve for slack variables (keeping $N$, $F$ fixed):
   $$S = \text{solve\_LCP}(A_{SS}, b_S - A_{SN}N - A_{SF}F)$$

4. Check convergence: if $\|\text{change}\| < \epsilon$, break.

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
> Non-warm-started strictly interior friction-index rows first try the shared
> validated friction-index exact solve; active friction bounds, warm starts, and
> validator-rejected rows continue through the normal/friction staggering loop.

DART 7 benchmark evidence includes `BM_LcpStaggeringContactPipelineSweep`,
which exercises the normal/friction split on DART-owned contact-pipeline
fixtures rather than only synthetic math packets. The rows cover separated
sphere-ground contacts, coupled vertical sphere stacks, and articulated unified
contacts for ground, rigid-impact, and cross-link-impact cases. Focused default,
SIMD-enabled, and CUDA-enabled build-tree runs reported 9 rows with
`contract_ok=1`, normal-row counters `1/2/3/4/5`, friction-row counters
`2/4/6/8/10`, coupled-contact flags, and backend build-state counters. The
CUDA-enabled rows are CPU Staggering solver rows in a CUDA-enabled build, not
CUDA LCP kernel execution.

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

- Split variables into normal impulses $x_N$, friction impulses $x_F$, and (optionally) slack $x_S$.
- Normal subproblem often has a symmetric PSD matrix $A_{NN}$ → can be solved by QP or PCG and produces $x_N$.
- Friction subproblem is a BLCP with bounds $|x_F| \leq \mu x_N$; when $A_{FF}$ is symmetric PSD it is equivalent to the QP:

$$\min_{x_F} \frac{1}{2} x_F^T A_{FF} x_F + c_F^T x_F \quad \text{subject to } x_F \geq 0, \; c_N - e^T x_F \geq 0$$

- Iterate: solve normal → update bounds for friction → solve friction (QP or small LCP) → repeat until fixed point. A single pass can also be used to warm-start Newton or PGS.

### Use Cases

- Contact problems with normal/tangential separation
- Warm-starting Newton methods
- Problems with natural block structure
- When standard methods converge slowly

## 3. Specialized Methods

### 3.1 Shock-Propagation Method ✅ (Implemented)

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

### DART Implementation

```cpp
#include <dart/math/lcp/other/shock_propagation_solver.hpp>

using namespace dart::math;

LcpProblem problem(A, b, lo, hi, findex);
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());

ShockPropagationSolver solver;
ShockPropagationSolver::Parameters params;
params.blockSizes = {3, 3, 3};  // One block per contact (example)
params.layers = {{0}, {1}, {2}}; // Bottom-to-top ordering

LcpOptions options = solver.getDefaultOptions();
options.customOptions = &params;
solver.solve(problem, x, options);
```

> Note: Layers must cover all blocks. If `layers` is empty, DART uses a single
> layer containing all blocks (equivalent to an ordered block sweep).
> Strictly interior standard LCPs without a warm start first try the shared
> validated linear-solve fast path. Default-option solves take that path before
> block/layer construction; custom block/layer options first run a lightweight
> structure validation so invalid custom layers still fail before the fast path
> is accepted. Accepted exact candidates validate the problem form and final
> solution before the full layered path setup runs. SPD rows prefer an LLT exact
> solve before falling back to the shared linear-solve helper. Boxed LCPs without
> friction-index coupling use the same structure-validation gate, then try the
> shared projected-active-set exact solve before block data construction; the
> shortcut is accepted only when the final boxed solution passes validation.
> Strictly interior friction-index rows up to 192 variables use the shared
> validated friction-index exact solve after lightweight block/layer structure
> validation. Default options and empty custom block/layer options avoid block
> data construction before that exact attempt; non-empty custom partitions still
> build and validate block data first so invalid partitions fail before a fast
> path can accept a solution. Larger or validator-rejected friction-index rows
> continue through the layered block path. Non-warm-started fallback solves
> delay resetting the initial guess until after those exact attempts and block
> validation, avoiding a zero-vector write on accepted exact candidates while
> preserving the layered fallback initialization.
> Small uncoupled fixed-bound blocks first try a local direct linear solve when
> the unconstrained candidate is already feasible; active-bound, singular,
> non-finite, larger, and active local `findex` blocks still use the existing
> Direct/Dantzig fallback path.

DART 7 benchmark evidence includes `BM_LcpShockPropagationLayerSweep`, which
compares single-layer, two-layer, and serial layer schedules over standard
48-row, boxed 24-row, and friction-index 8-contact fixtures using 3-row
blocks. Focused default, SIMD-enabled, and CUDA-enabled build-tree runs reported
9 rows with `contract_ok=1`; the CUDA-enabled rows are CPU solver rows in a
CUDA-enabled build, not CUDA LCP kernel execution.

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
#include <dart/math/lcp/other/mprgp_solver.hpp>

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
> definite matrices. `supportsProblem(problem)` reports only that native SPD
> subset by default, while boxed, friction-indexed, non-symmetric, and
> non-positive-definite packets still delegate to the boxed-capable pivoting
> solver through `solve()`.
> Default, non-warm-started, strictly interior standard LCPs reuse the
> positive-definite factorization check as a validated linear-solve fast path
> before the reduced-gradient loop. Custom-option solves stay on the iterative
> path so parameter stress tests and user tuning remain observable.

DART 7 benchmark evidence includes `BM_LcpMprgpSpdCheckSweep`, which compares
well-conditioned dense SPD, banded SPD, mildly ill-conditioned SPD, and
near-singular SPD standard-LCP fixtures while toggling the positive-definite
factorization check. Focused default, SIMD-enabled, and CUDA-enabled build-tree
runs reported 9 rows with `contract_ok=1`; the CUDA-enabled rows are CPU solver
rows in a CUDA-enabled build, not CUDA LCP kernel execution.
`BM_LcpContactNormalStandardSweep` adds contact-derived standard-LCP evidence
for MPRGP by extracting only the normal rows from DART 7 separated
sphere-ground, coupled vertical-stack, and articulated unified-contact
snapshots. This remains standard normal-contact evidence; boxed or
friction-index contact paths still delegate to boxed-capable solvers.

### 3.3 ADMM ✅

**Description**: Alternating Direction Method of Multipliers solver for boxed
LCPs. DART's implementation alternates between a proximal linear solve, a box
projection, and a dual update:

$$x = (A + (\rho+\mu)I)^{-1}(\rho z - y + b)$$
$$z = \Pi_{[l,u]}(x + y/\rho)$$
$$y = y + \rho(x-z)$$

For friction-index rows, the projection uses effective bounds coupled to the
current normal impulse.
The default initial penalty is $\rho=4$, chosen from current DART-owned
adaptive-rho sweep evidence to reduce active-box iteration counts while keeping
the same adaptive residual-balancing path.
For non-warm-started standard LCPs with default per-solve parameters, ADMM first
tries a validated strict-interior exact solve before allocating iteration
workspace. The fast path prefers an LLT solve on SPD rows, falls back to the
shared linear-solve helper when needed, and is accepted only when the candidate
is strictly positive and passes solution validation. Non-warm-started boxed
LCPs without friction-index coupling then try a projected active-set exact solve:
the unconstrained solution proposes lower/upper/free rows, the free block is
solved exactly, and the shortcut is accepted only if the final boxed solution
passes the shared validator. Small and medium strictly interior friction-index
rows use the shared validated friction-index exact solve; larger
friction-index, warm-started, and explicit custom-option calls stay on the
operator-splitting loop.
The ADMM loop reuses its linear-solve right-hand side and projected-step
workspace across iterations, avoiding repeated per-iteration vector allocation
without changing the operator-splitting updates or convergence tests.
Focused `BM_LcpAdmmRhoSweep` rows compare fixed and adaptive $\rho$ settings
on identical standard, boxed, and friction-index benchmark fixtures; these rows
are CPU solver rows even when emitted by a CUDA-enabled build. Additional
`BM_LcpContactSolverComparisonSweep/Admm/*` rows reuse DART 7 separated
sphere-ground, coupled vertical-stack, and articulated unified-contact
friction-index fixtures so ADMM contact evidence is tracked independently from
the synthetic $\rho$ sweep. ADMM also has solver-specific generated evidence on
16x-coupled mildly ill-conditioned friction-index packets at 16 and 24 contacts;
the focused correctness slice and `BM_LcpMildIllConditioned/ExtremeCoupled*`
rows keep that claim separate from broader all-solver conditioning coverage.

```cpp
#include <dart/math/lcp/other/admm_solver.hpp>

dart::math::AdmmSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 200;
solver.solve(problem, x, options);
```

**Use Cases**:

- Boxed/contact-style problems where operator-splitting robustness is useful
- Approximate solves with adaptive residual balancing
- Benchmark comparisons against projection and pivoting methods

### 3.4 SAP ✅

**Description**: Semi-Analytic Primal inspired regularized solve for boxed
LCPs. DART's implementation minimizes a regularized quadratic objective with
Armijo backtracking:

$$L(x) = \frac{1}{2}x^T A x - b^T x + \text{regularization}(x,l,u)$$

The regularization makes the method useful for compliant-contact-inspired
experiments, but it also means strict LCP complementarity is approximate by
design.
Focused `BM_LcpSapRegularizationSweep` rows compare regularization values
`1e-6`, `1e-5`, and `1e-4` on identical standard, boxed, and friction-index
benchmark fixtures; these rows are CPU solver rows even when emitted by a
CUDA-enabled build. The main `BM_LcpCompare/*/Sap` profile uses the stricter
`1e-6` regularization for standard and friction-index rows, and the solver's
default `1e-4` compliance regularization for boxed rows where the current
comparison tolerance admits the expected approximate complementarity. Additional
`BM_LcpContactSolverComparisonSweep/Sap/*` rows reuse DART 7 separated
sphere-ground, coupled vertical-stack, and articulated unified-contact
friction-index fixtures so SAP contact evidence is tracked independently from
the synthetic regularization sweep. SAP also has
solver-specific generated evidence on 16x-coupled mildly ill-conditioned
friction-index packets at 16 and 24 contacts; the focused correctness slice and
`BM_LcpMildIllConditioned/ExtremeCoupled*` rows keep that claim separate from
broader all-solver conditioning coverage.
For non-warm-started high-level solves, strictly interior standard rows use the
shared validated linear-solve fast path and boxed rows without friction-index
coupling use the shared projected-active-set exact solve. Warm-started,
coupled-friction, and validator-rejected rows stay on SAP's regularized Newton
path.

```cpp
#include <dart/math/lcp/other/sap_solver.hpp>

dart::math::SapSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.maxIterations = 50;
solver.solve(problem, x, options);
```

**Use Cases**:

- Regularized contact-style solves
- DART 7 compliant-contact experiments
- Approximate benchmark baselines where exact complementarity is not required

## 4. Blocked Jacobi ✅ (Implemented)

### Description

Jacobi splitting applied to blocks (similar to Blocked Gauss-Seidel but with Jacobi update).

### Algorithm

```
# All blocks updated in parallel
for iter = 1 to max_iter:
  parallel for each block i:
    r_i = b_i - sum(A_{ij} * x_j^k, j != i)
    x_i^{k+1} = SolveSubLCP(A_{ii}, r_i)
```

### DART Implementation

```cpp
#include <dart/math/lcp/projection/blocked_jacobi_solver.hpp>

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
> to `DantzigSolver` for boxed or larger blocks. Singleton blocks with fixed
> bounds use the scalar projected solve directly, which preserves the Jacobi
> update while avoiding local solver setup overhead. Fixed-bound problems also
> precompute the Jacobi snapshot product once per iteration so independent
> blocks do not repeat dense row products.
> Strictly interior standard LCPs without a warm start first try the shared
> validated linear-solve fast path. Default-option solves can take that path
> before block construction, while explicit custom block partitions are still
> validated before the fast path is accepted. Strictly interior friction-index
> rows use the shared validated friction-index exact solve through the same
> validation gateway, and contact-sized local `findex` blocks try that helper
> before falling back to Dantzig.

DART 7 benchmark evidence includes `BM_LcpBlockPartitionSweep`, which compares
Blocked Jacobi and BGS on standard, boxed, and friction-index fixtures with
full-block, 3-row block, auto `findex` contact-block, and explicit
contact-block partitions. Focused default, SIMD-enabled, and CUDA-enabled
build-tree runs reported 12 rows with `contract_ok=1`, block counts `1/4`,
block sizes `3/12`, friction-index `contact_count=4`, observed solver
iterations `1/4/5/6/10`, and backend build-state counters. The CUDA-enabled
rows are CPU Blocked Jacobi/BGS solver rows in a CUDA-enabled build, not CUDA
LCP kernel execution.

### Properties

- **Parallelization**: Fully parallel (all blocks independent)
- **Convergence**: Slower than BGS, faster than scalar Jacobi
- **Use**: Parallel computing, GPU

## Comparison Summary

| Method               | Convergence | Complexity       | Robustness | Parallelization |
| -------------------- | ----------- | ---------------- | ---------- | --------------- |
| Interior Point ✅    | Superlinear | $O(n^3)$         | Very High  | No              |
| Staggering ✅        | Variable    | Depends          | Medium     | No              |
| Shock-Propagation ✅ | Linear      | $O(n)$           | Medium     | Limited         |
| MPRGP ✅             | Monotone    | $O(n^2)$         | High       | No              |
| ADMM ✅              | Variable    | Linear solve     | Medium     | No              |
| SAP ✅               | Regularized | Newton solve     | Medium     | No              |
| Blocked Jacobi ✅    | Linear      | $O(n \cdot b^3)$ | Medium     | Yes             |

## Implementation Priority

### Implemented in DART

- **Interior Point**: Primal-dual path-following for standard LCPs
- **Staggering**: Normal/friction block solve for contact-style problems
- **Blocked Jacobi**: Parallel block updates with per-block LCP solves
- **MPRGP**: Bound-constrained QP solver for standard SPD LCPs
- **Shock-Propagation**: Layered block solve for gravity-dominated contact
- **ADMM**: Operator-splitting solver for boxed LCPs
- **SAP**: Regularized SAP-inspired boxed LCP solver

### Low Priority (Specialized Use Cases)

- Additional gravity-structured contact methods

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

When $A$ is symmetric PD, LCP can be reformulated as QP:

$$\min_x \frac{1}{2} x^T A x + x^T b \quad \text{subject to } x \geq 0$$

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

Potential for $O(n)$ convergence with multilevel hierarchy:

$$\text{Coarse grid} \to \text{Medium grid} \to \text{Fine grid}$$

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
