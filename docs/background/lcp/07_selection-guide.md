# LCP Solver Selection Guide

**Navigation**: [← Other Methods](06_other-methods.md) | [Overview](02_overview.md)

## Quick Decision Tree

```
START
  |
  ├─ Real-time/Interactive? ──YES──> PGS or PSOR
  |                                   (when implemented)
  |
  ├─ High accuracy needed? ──YES──> Newton Methods or Pivoting
  |                                  (Dantzig/Lemke available now)
  |
  ├─ Large problem (n>1000)? ──YES──> NNCG or PGS
  |                                    (when implemented)
  |
  ├─ Ill-conditioned? ──YES──> Pivoting (Dantzig/Lemke)
  |                             or Interior Point
  |
  ├─ Contact problem? ──YES──> BGS or ODELCPSolver
  |                             (ODELCPSolver available now)
  |
  └─ Default ──> Start with Dantzig or Lemke
                  (currently implemented)
```

## Detailed Selection by Use Case

### 1. Real-Time Physics Simulation

**Recommended (when implemented)**: PGS > PSOR > BGS
**Currently Available**: Dantzig, Lemke, ODELCPSolver

**Rationale**:

- Need O(n) per-iteration cost
- Acceptable approximate solutions
- 50-100 iterations typical
- Predictable performance

**Configuration**:

```
Method: PGS (when implemented)
Max iterations: 50-100
Tolerance: 1e-4 to 1e-6
Warm start: Previous time-step solution
```

**Current Workaround**:

```cpp
// Use ODELCPSolver for contact problems
ODELCPSolver solver;
solver.Solve(A, b, &x, numContacts, mu, numDir, true);

// Or Dantzig for bounded problems
SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);
```

### 2. High-Accuracy Off-Line Simulation

**Recommended**: Newton > Pivoting > Interior Point
**Currently Available**: Dantzig ✅, Lemke ✅

**Rationale**:

- Need accuracy 1e-8 to 1e-12
- Computational budget allows
- 5-20 iterations typical for Newton
- Exact solutions from pivoting

**Configuration (Future Newton)**:

```
Method: Minimum Map Newton
Max iterations: 20-50
Tolerance: 1e-8 to 1e-12
Warm start: PGS (10 iterations)
Subsolver: GMRES with tolerance 0.1*||H||
```

**Current Best**:

```cpp
// Dantzig for bounded problems
SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);

// Lemke for standard LCP
Lemke(M, q, &z);
```

### For Contact Mechanics with Friction

**Recommended**: ODELCPSolver (uses Dantzig) > Dantzig directly
**Currently Available**: ODELCPSolver ✅, Dantzig ✅

**Rationale**:

- Natural block structure (per-contact)
- Friction cone constraints
- Normal/tangential coupling
- ODELCPSolver handles formulation automatically

**Configuration**:

```cpp
// Current: Use ODELCPSolver (wrapper around Dantzig)
dart::math::ODELCPSolver solver;
bool success = solver.Solve(
    A, b, &x,
    numContacts,  // Number of contact points
    mu,           // Friction coefficient
    numDir,       // Friction directions (0, 2, or 4)
    true          // Use internal Dantzig solver
);

// Or use Dantzig directly with friction indices
int findex[n];
// Set up findex for friction cone
dart::math::SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);
```

**Future BGS (when implemented)**:

```
Per-contact blocks with:
  - Normal: 1D direct solve
  - Friction: 2D/3D geometric solver
  - Pyramid: 4D small iterative
```

### 4. Large-Scale Problems (n > 1000)

**Recommended (when implemented)**: NNCG > PGS > Newton+Iterative
**Currently Available**: Dantzig, Lemke (limited scalability)

**Rationale**:

- O(n) per-iteration essential
- Matrix-free implementations
- Sparse matrix exploitation

**Future Configuration**:

```
Method: NNCG
Max iterations: 100-500
Restart: Every 20 iterations
Warm start: Previous solution or zeros
```

**Current Approach**:

```
# For very large problems, consider:
# 1. Problem decomposition
# 2. Using external solvers (MOSEK, CPLEX)
# 3. Approximation techniques
```

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
SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);

// Or Lemke
Lemke(M, q, &z);
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

**Recommended (when implemented)**: Jacobi > Red-Black GS > Blocked Jacobi
**Currently Available**: None (sequential methods only)

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

| Use Case      | Best Method  | Per-Iter Time | Iterations | Total Time | Available Now |
| ------------- | ------------ | ------------- | ---------- | ---------- | ------------- |
| Real-time     | PGS          | O(n)          | 50-100     | O(50n)     | ❌            |
| Real-time     | Dantzig      | O(n³)         | 1          | O(n³)      | ✅            |
| High accuracy | Newton       | O(n³)\*       | 5-20       | O(20n³)\*  | ❌            |
| High accuracy | Dantzig      | O(n³)         | 1          | O(n³)      | ✅            |
| Contact       | BGS          | O(nb³)        | 50-100     | O(50nb³)   | ❌            |
| Contact       | ODELCPSolver | -             | -          | -          | ✅            |
| Large-scale   | NNCG         | O(n)          | 20-200     | O(200n)    | ❌            |

\* With iterative subsolver

### Accuracy vs Speed Trade-off

```
Higher Accuracy ─────────────────────────> Lower Accuracy
Slower          ─────────────────────────> Faster

Pivoting ─> Newton ─> Interior Point ─> NNCG ─> BGS ─> PGS ─> Jacobi
(exact)     (1e-10)   (1e-8)           (1e-6)  (1e-4) (1e-3) (1e-2)

✅ Available:  Dantzig, Lemke
❌ Future:     All others
```

### Robustness vs Efficiency

```
More Robust ─────────────────────────> Less Robust
Slower      ─────────────────────────> Faster

Pivoting ─> Interior Point ─> Newton ─> BGS ─> PGS ─> Jacobi

✅ Available:  Dantzig, Lemke
❌ Future:     All others
```

## Problem Size Guidelines

| Problem Size     | Recommended Method       | Currently Available  |
| ---------------- | ------------------------ | -------------------- |
| n < 10           | Direct 2D/3D or Pivoting | Dantzig ✅, Lemke ✅ |
| 10 ≤ n < 100     | Pivoting or Newton       | Dantzig ✅, Lemke ✅ |
| 100 ≤ n < 1000   | PGS, BGS, or Newton      | Dantzig ✅ (slow)    |
| 1000 ≤ n < 10000 | NNCG or PGS              | None (need PGS)      |
| n ≥ 10000        | NNCG or specialized      | None                 |

## Conditioning Guidelines

| Matrix Condition     | Recommended Method       | Currently Available  |
| -------------------- | ------------------------ | -------------------- |
| Well-conditioned     | Any method               | All ✅               |
| Moderate             | PGS, Newton, Pivoting    | Dantzig ✅, Lemke ✅ |
| Ill-conditioned      | Pivoting, Interior Point | Dantzig ✅, Lemke ✅ |
| Very ill-conditioned | Pivoting only            | Dantzig ✅, Lemke ✅ |

## Implementation Roadmap Impact

### Current State (2025-11-22)

Available solvers:

- ✅ **Dantzig**: BLCP with bounds, friction support
- ✅ **Lemke**: Standard LCP
- ✅ **ODELCPSolver**: Contact problems wrapper

**Best Practices Now**:

```cpp
// For bounded problems with friction
SolveLCP<double>(n, A, x, b, w, nub, lo, hi, findex);

// For standard LCP
Lemke(M, q, &z);

// For contact problems
ODELCPSolver solver;
solver.Solve(A, b, &x, numContacts, mu, numDir, true);
```

### Phase 1: Core Iterative (Priority)

When PGS/PSOR are implemented:

- ✅ Real-time simulation becomes viable
- ✅ Most contact problems solvable interactively
- ✅ O(n) iteration cost achieved

### Phase 2: Blocked Methods

When BGS is implemented:

- ✅ Better contact force accuracy
- ✅ Natural per-contact structure
- ✅ Sub-solvers for small blocks

### Phase 3: Advanced Iterative

When NNCG is implemented:

- ✅ Large-scale problems solvable
- ✅ Better convergence than PGS
- ✅ Mass ratio handling improved

### Phase 4: Newton Methods

When Newton methods are implemented:

- ✅ High accuracy achievable
- ✅ Superlinear convergence
- ✅ Off-line simulation improved

## Common Pitfalls and Solutions

### Pitfall 1: Using Expensive Method for Real-Time

**Problem**: Using Dantzig (O(n³)) for real-time with n > 100

**Solution**:

```
Current: Use ODELCPSolver or limit problem size
Future: Use PGS or PSOR when implemented
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

// Or warm-start with few PGS iterations (when available)
```

### Pitfall 4: Wrong Method for Problem Structure

**Problem**: Using scalar PGS for contact problems

**Solution**:

```
Current: Use ODELCPSolver ✅
Future: Use BGS (better per-contact structure)
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

### PGS/PSOR (when implemented)

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

### Newton Methods (when implemented)

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

**For Future Iterative Methods**:

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
3. Verify complementarity: x^T(Ax+b) ≈ 0
4. Try different solver (Dantzig vs Lemke)
```

### Performance is Poor

```
Current (with Dantzig/Lemke):
1. Limit problem size (n < 100)
2. Use ODELCPSolver for contacts
3. Reduce contact points
4. Simplify collision geometry

Future (with PGS/Newton):
1. Use appropriate method for problem size
2. Enable warm-starting
3. Matrix-free implementations
4. Exploit sparsity
```

## Summary Recommendations

### Current State (Until Phase 1 Complete)

| Scenario               | Use                           | Notes                        |
| ---------------------- | ----------------------------- | ---------------------------- |
| Contact with friction  | ODELCPSolver                  | Best option now              |
| Bounded variables      | Dantzig                       | Supports bounds and friction |
| Standard LCP           | Lemke                         | Simple and robust            |
| Large problems (n>100) | Limit size or external solver | Current methods O(n³)        |
| Real-time (n>50)       | Reduce problem size           | O(n³) too expensive          |

### Future State (After Implementation)

| Scenario        | Primary | Backup         | Notes              |
| --------------- | ------- | -------------- | ------------------ |
| Real-time       | PGS     | PSOR           | 50-100 iterations  |
| Contact         | BGS     | PGS            | Per-contact blocks |
| High accuracy   | Newton  | Dantzig        | 5-20 iterations    |
| Large-scale     | NNCG    | PGS            | >1000 variables    |
| Ill-conditioned | Dantzig | Interior Point | Most robust        |
| Parallel        | Jacobi  | Red-Black GS   | GPU-friendly       |

---

**Navigation**: [← Other Methods](06_other-methods.md) | [Overview](02_overview.md)
