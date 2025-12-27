# LCP Solver Selection Guide

**Navigation**: [← Other Methods](06_other-methods.md) | [Overview](02_overview.md)

## Quick Decision Tree

```
START
  |
  ├─ Real-time/Interactive? ──YES──> PGS/PSOR (PgsSolver w/ relaxation)
  |
  ├─ High accuracy needed? ──YES──> Newton Methods or Pivoting
  |                                  (Dantzig/Lemke available now)
  |
  ├─ Large problem (n>1000)? ──YES──> PGS or NNCG
  |
  ├─ Ill-conditioned? ──YES──> Pivoting (Dantzig/Lemke)
  |                             or Interior Point
  |
  ├─ Contact problem? ──YES──> BGS or Dantzig
  |
  └─ Default ──> Start with Dantzig or PGS
                  (Lemke for standard LCP)
```

## Detailed Selection by Use Case

### 1. Real-Time Physics Simulation

**Recommended**: PGS/PSOR (`PgsSolver` with relaxation) > BGS
**Currently Available**: PGS/PSOR ✅, BGS ✅, Dantzig ✅, Lemke ✅

**Rationale**:

- Need O(n) per-iteration cost
- Acceptable approximate solutions
- 50-100 iterations typical
- Predictable performance

**Configuration**:

```
Method: PGS
Max iterations: 50-100
Tolerance: 1e-4 to 1e-6
Warm start: Previous time-step solution
Relaxation: 1.0 (PGS), 1.2-1.5 (PSOR)
Randomize order: Optional (can improve robustness)
Fallback: Dantzig if iterative solve stalls
```

`constraint::ConstraintSolver` already wires this up by running
`math::DantzigSolver` first and falling back to `math::PgsSolver` when needed.

### 2. High-Accuracy Off-Line Simulation

**Recommended**: Newton > Pivoting > Interior Point
**Currently Available**: Minimum Map Newton ✅, Fischer-Burmeister Newton ✅,
Penalized FB Newton ✅ (standard LCP only), Dantzig ✅, Lemke ✅

**Rationale**:

- Need accuracy 1e-8 to 1e-12
- Computational budget allows
- 5-20 iterations typical for Newton
- Exact solutions from pivoting

**Configuration (Newton Methods)**:

```
Method: Minimum Map Newton
Max iterations: 20-50
Tolerance: 1e-8 to 1e-12
Warm start: PGS (10 iterations)
Subsolver: GMRES with tolerance 0.1*||H||
```

For Penalized FB, set `PenalizedFischerBurmeisterNewtonSolver::Parameters::lambda`.

**Current Best**:

```cpp
dart::math::LcpProblem problem(A, b, lo, hi, findex);
dart::math::DantzigSolver solver;
solver.solve(problem, x, solver.getDefaultOptions());

// Lemke for standard LCP (lo=0, hi=inf, findex=-1)
dart::math::LcpProblem standard(M, -q, Eigen::VectorXd::Zero(q.size()),
                                Eigen::VectorXd::Constant(q.size(),
                                                          std::numeric_limits<double>::infinity()),
                                Eigen::VectorXi::Constant(q.size(), -1));
dart::math::LemkeSolver lemke;
lemke.solve(standard, z, lemke.getDefaultOptions());
```

### For Contact Mechanics with Friction

**Recommended**: BGS (available) > Dantzig > PGS
**Currently Available**: BGS ✅, Dantzig ✅, PGS ✅

**Rationale**:

- Natural block structure (per-contact)
- Friction cone constraints
- Normal/tangential coupling

**Configuration**:

```cpp
// Use the boxed LCP API with friction indices
dart::math::LcpProblem problem(A, b, lo, hi, findex);
dart::math::DantzigSolver solver;
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());
solver.solve(problem, x, solver.getDefaultOptions());
```

**BGS (Available)**:

```
Per-contact blocks with:
  - Normal: 1D direct solve
  - Friction: 2D/3D geometric solver
  - Pyramid: 4D small iterative
```

### 4. Large-Scale Problems (n > 1000)

**Recommended**: PGS (available) > NNCG (available) > Newton+Iterative
**Currently Available**: PGS ✅, NNCG ✅, Dantzig ✅, Lemke ✅

**Rationale**:

- O(n) per-iteration essential
- Matrix-free implementations
- Sparse matrix exploitation

**Recommended Configuration**:

```
Method: NNCG
Max iterations: 100-500
Restart: Every 20 iterations
Warm start: Previous solution or zeros
```

**Current Approach**:

- Prefer `math::PgsSolver` for scalability, with Dantzig fallback if it fails
- Decompose by contact groups to keep LCP blocks small
- For extremely large problems, consider external solvers or approximations

### 5. Ill-Conditioned Problems

**Recommended**: Pivoting > Interior Point
**Currently Available**: Dantzig ✅, Lemke ✅

**Rationale**:

- Robust to conditioning
- Handle large mass ratios
- Numerical stability

**Current Solution**:

```cpp
// Dantzig is robust for ill-conditioned problems
dart::math::LcpProblem problem(A, b, lo, hi, findex);
dart::math::DantzigSolver solver;
solver.solve(problem, x, solver.getDefaultOptions());

// Or Lemke
dart::math::LcpProblem standardProblem(
    M,
    q,
    Eigen::VectorXd::Zero(q.size()),
    Eigen::VectorXd::Constant(
        q.size(), std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(q.size(), -1));
dart::math::LemkeSolver lemke;
lemke.solve(standardProblem, z, lemke.getDefaultOptions());
validate(M, z, q);  // Always validate
```

**When Pivoting Fails**:

```
Try:
1. Problem reformulation
2. Regularization (add small diagonal)
3. Scaling/preconditioning
4. External QP solver (if A is symmetric PD)
```

### 6. Parallel/GPU Computing

**Recommended**: Jacobi > Blocked Jacobi > Red-Black GS
**Currently Available**: Jacobi, Blocked Jacobi, Red-Black GS (single-threaded baselines)

**Rationale**:

- Embarrassingly parallel updates
- GPU-friendly operations
- No data dependencies

**Future Configuration**:

```
Method: Jacobi or Red-Black GS
Max iterations: 100-200 (more than PGS)
Parallelization: GPU kernels
Block size: 32-256 threads per block
```

## Method Comparison Matrix

### Computational Cost

| Use Case      | Best Method | Per-Iter Time | Iterations | Total Time | Available Now |
| ------------- | ----------- | ------------- | ---------- | ---------- | ------------- |
| Real-time     | PGS         | O(n)          | 50-100     | O(50n)     | ✅            |
| Real-time     | Dantzig     | O(n³)         | 1          | O(n³)      | ✅            |
| High accuracy | Newton      | O(n³)\*       | 5-20       | O(20n³)\*  | ✅ (standard) |
| High accuracy | Dantzig     | O(n³)         | 1          | O(n³)      | ✅            |
| Contact       | BGS         | O(nb³)        | 50-100     | O(50nb³)   | ✅            |
| Contact       | Dantzig     | -             | -          | -          | ✅            |
| Large-scale   | NNCG        | O(n)          | 20-200     | O(200n)    | ✅            |

\* With iterative subsolver

### Accuracy vs Speed Trade-off

```
Higher Accuracy ─────────────────────────> Lower Accuracy
Slower          ─────────────────────────> Faster

Pivoting ─> Newton ─> Interior Point ─> NNCG ─> BGS ─> PGS ─> Jacobi
(exact)     (1e-10)   (1e-8)           (1e-6)  (1e-4) (1e-3) (1e-2)

✅ Available:  Direct 2D/3D, Dantzig, Lemke, Baraff, PGS/PSOR/Symmetric PSOR, Jacobi, Blocked Jacobi, Red-Black GS, Staggering, BGS, PGS-SM, NNCG, Newton (standard LCP), Interior Point, MPRGP (standard SPD)
```

### Robustness vs Efficiency

```
More Robust ─────────────────────────> Less Robust
Slower      ─────────────────────────> Faster

Pivoting ─> Interior Point ─> Newton ─> BGS ─> PGS ─> Jacobi

✅ Available:  Direct 2D/3D, Dantzig, Lemke, Baraff, PGS/PSOR/Symmetric PSOR, Jacobi, Blocked Jacobi, Red-Black GS, Staggering, BGS, PGS-SM, NNCG, Newton (standard LCP), Interior Point, MPRGP (standard SPD)
```

## Problem Size Guidelines

| Problem Size     | Recommended Method          | Currently Available                                         |
| ---------------- | --------------------------- | ----------------------------------------------------------- |
| n < 10           | Direct 2D/3D or Pivoting    | Direct ✅, Dantzig ✅, Lemke ✅                             |
| 10 ≤ n < 100     | Pivoting or Newton          | Dantzig ✅, Lemke ✅, Newton ✅ (standard)                  |
| 100 ≤ n < 1000   | PGS, BGS, PGS-SM, or Newton | PGS ✅, BGS ✅, PGS-SM ✅, Dantzig ✅, Newton ✅ (standard) |
| 1000 ≤ n < 10000 | NNCG or PGS                 | PGS ✅, NNCG ✅                                             |
| n ≥ 10000        | NNCG or specialized         | NNCG ✅, PGS ✅ (approx)                                    |

## Conditioning Guidelines

| Matrix Condition     | Recommended Method       | Currently Available                                |
| -------------------- | ------------------------ | -------------------------------------------------- |
| Well-conditioned     | Any method               | All ✅                                             |
| Moderate             | PGS, Newton, Pivoting    | PGS ✅, Dantzig ✅, Lemke ✅, Newton ✅ (standard) |
| Ill-conditioned      | Pivoting, Interior Point | Dantzig ✅, Lemke ✅, Interior Point ✅            |
| Very ill-conditioned | Pivoting only            | Dantzig ✅, Lemke ✅, Interior Point ✅            |

## Implementation Roadmap Impact

### Current State (2025-11-22)

Available solvers:

- ✅ **Dantzig**: BLCP with bounds, friction support
- ✅ **Lemke**: Standard LCP
- ✅ **Baraff**: Incremental pivoting for symmetric PSD standard LCPs
- ✅ **Direct 2D/3D**: Enumeration solver for tiny standard LCPs
- ✅ **Interior Point**: Primal-dual method for robust standard LCP solves
- ✅ **MPRGP**: QP-based solver for standard SPD LCPs
- ✅ **PGS/PSOR**: Iterative boxed LCP with friction index fallback (tune
  `LcpOptions::relaxation`)
- ✅ **Symmetric PSOR**: Forward/backward sweep variant for reduced bias
- ✅ **Jacobi**: Projected Jacobi baseline (parallel-friendly)
- ✅ **Blocked Jacobi**: Parallel block updates for contact-style problems
- ✅ **Red-Black GS**: Two-color Gauss-Seidel variant for parallel-style updates
- ✅ **Staggering**: Normal/friction block staggering for contact structure
- ✅ **BGS**: Blocked Gauss-Seidel for per-contact blocks
- ✅ **PGS-SM**: Subspace minimization hybrid for medium problems
- ✅ **Newton (Minimum Map, FB, Penalized FB)**: Standard LCP only
- ✅ **NNCG**: Conjugate-gradient acceleration on PGS sweeps

**Best Practices Now**:

```cpp
dart::math::LcpProblem problem(A, b, lo, hi, findex);
dart::math::DantzigSolver dantzig;
dart::math::PgsSolver pgs;
auto result = dantzig.solve(problem, x, dantzig.getDefaultOptions());
if (!result.succeeded()) {
  pgs.solve(problem, x, pgs.getDefaultOptions());
}
```

### Remaining Gaps

- Shock-propagation and other contact-structure specialized methods

### Newton Methods (Implemented)

- ✅ High accuracy achievable
- ✅ Superlinear convergence
- ✅ Off-line simulation improved

## Common Pitfalls and Solutions

### Pitfall 1: Using Expensive Method for Real-Time

**Problem**: Using Dantzig (O(n³)) for real-time with n > 100

**Solution**:

```
Current: Use PGS for real-time and keep Dantzig as fallback
Tune:   Use `LcpOptions::relaxation` for PSOR-style relaxation
```

### Pitfall 2: Expecting High Accuracy from Iterative

**Problem**: Running PGS for 10000 iterations expecting 1e-10

**Solution**:

```
PGS: Good for 1e-4 to 1e-6
For 1e-10: Use Newton or Pivoting (Dantzig ✅ available)
```

### Pitfall 3: Not Warm-Starting

**Problem**: Starting Newton or iterative from x=0 every time

**Solution**:

```cpp
// Use previous solution
x_init = x_previous_timestep;

// Or warm-start with few PGS iterations
```

### Pitfall 4: Wrong Method for Problem Structure

**Problem**: Using scalar PGS for contact problems

**Solution**:

```
Current: Use Dantzig ✅
Current: Use BGS (better per-contact structure)
```

### Pitfall 5: Ignoring Convergence Monitoring

**Problem**: Running fixed iterations without checking convergence

**Solution**:

```cpp
// Always check multiple criteria
bool converged = (residual < abs_tol) ||
                 (relative_change < rel_tol) ||
                 (iterations >= max_iter);

// Monitor divergence
if (merit_function > gamma * previous_merit) {
  // Handle divergence
}
```

## Parameter Tuning Guidelines

### PGS/PSOR

```
max_iterations:
  - Real-time: 50-100
  - Accuracy: 500-1000

tolerance:
  - Real-time: 1e-4 to 1e-6
  - Accuracy: 1e-6 to 1e-8

relaxation (PSOR only):
  - Start: 1.0 (PGS)
  - Tune: 1.2-1.5
  - Under-relax if unstable: 0.8-0.95
```

### Newton Methods

```
max_iterations: 20-50

tolerance:
  - Standard: 1e-8
  - High accuracy: 1e-12

line_search:
  - alpha: 1e-4 (sufficient decrease)
  - beta: 0.5 (step reduction)

subsolver_tolerance:
  - Initial: 0.1 * ||H||
  - Final: 0.01 * ||H||
```

### Dantzig (currently available)

```cpp
// Early termination for approximate solutions
bool earlyTermination = true;  // For faster approximate solve

// Set bounds carefully
lo[i] <= 0  // Must be non-positive
hi[i] >= 0  // Must be non-negative

// Friction indices
findex[i] = -1;  // No friction dependency
findex[i] = j;   // Depends on x[j] for friction cone
```

## Troubleshooting Guide

### Method Not Converging

**For Dantzig/Lemke (current)**:

```
1. Check matrix properties
2. Try problem reformulation
3. Add regularization: A += epsilon * I
4. Scale problem: divide A, b by max(|A|)
```

**For Iterative Methods (PGS/PSOR)**:

```
1. Increase max_iterations
2. Try different starting point
3. Check matrix conditioning
4. Try relaxation (PSOR with lambda < 1)
5. Fall back to pivoting
```

### Solution is Infeasible

```
1. Validate input: A, b satisfy LCP structure
2. Check bounds: lo <= 0 <= hi
3. Verify complementarity: x^T(Ax-b) ≈ 0
4. Try different solver (Dantzig vs Lemke)
```

### Performance is Poor

```
Current (with Dantzig/PGS/BGS/Lemke/Newton/Interior Point):
1. Limit problem size (n < 100)
2. Use Dantzig for contacts
3. Reduce contact points
4. Simplify collision geometry
5. Use Interior Point for ill-conditioned problems
```

## Summary Recommendations

### Current State

| Scenario               | Use             | Notes                                    |
| ---------------------- | --------------- | ---------------------------------------- |
| Contact with friction  | BGS or Dantzig  | BGS for blocks, Dantzig for exact solves |
| Bounded variables      | Dantzig         | Supports bounds and friction             |
| Standard LCP           | Lemke or Newton | Newton for high accuracy (standard only) |
| Large problems (n>100) | NNCG or PGS     | NNCG converges faster, both approximate  |
| Real-time (n>50)       | PGS/PSOR        | Tune `relaxation`, keep Dantzig fallback |

### Current State (With Interior Point)

| Scenario        | Primary  | Backup                        | Notes              |
| --------------- | -------- | ----------------------------- | ------------------ |
| Real-time       | PGS/PSOR | -                             | 50-100 iterations  |
| Contact         | BGS      | PGS                           | Per-contact blocks |
| High accuracy   | Newton   | Dantzig                       | 5-20 iterations    |
| Large-scale     | NNCG     | PGS                           | >1000 variables    |
| Ill-conditioned | Dantzig  | Interior Point                | Most robust        |
| Parallel        | Jacobi   | Blocked Jacobi / Red-Black GS | GPU-friendly       |

---

**Navigation**: [← Other Methods](06_other-methods.md) | [Overview](02_overview.md)
