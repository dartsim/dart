# Projection/Sweeping Methods for LCP

> **Attribution**: This content is derived from "Contact Handling for Articulated
> Rigid Bodies Using LCP" by Jie Tan, Kristin Siu, and C. Karen Liu.
> The original PDF is preserved at [`docs/lcp.pdf`](../../lcp.pdf).

**Navigation**: [← Pivoting Methods](03_pivoting-methods.md) | [Index](../README.md) | [Newton Methods →](05_newton-methods.md)

## Overview

Projection methods (also called sweeping or splitting methods) are iterative solvers based on matrix splitting $A = M - N$. They are the most popular methods for real-time physics simulation due to their low $O(n)$ per-iteration cost.

## Core Concept: Matrix Splitting

All projection methods use:

$$A = M - N \quad \text{(matrix splitting)}$$
$$Mx^{k+1} = Nx^k + b \quad \text{(fixed-point iteration)}$$
$$x^{k+1} = \max(0, M^{-1}(Nx^k + b)) \quad \text{(projection onto positive orthant)}$$

The split is chosen so that $M$ is cheap to invert (non-zero diagonal is mandatory; diagonal dominance or triangular structure helps). If $M$ is a Q-matrix (its LCP is always solvable), then any fixed point of this iteration is also a solution of the original LCP.

### Derivation sketch

Start from the LCP:

$$Ax - b \geq 0, \quad x \geq 0, \quad x^T(Ax - b) = 0$$

Split $A = M - N$ and treat the iteration as a fixed point:

$$Mx^{k+1} - Nx^k - b \geq 0$$
$$x^{k+1} \geq 0$$
$$(x^{k+1})^T(Mx^{k+1} - Nx^k - b) = 0$$

For splittings where $M$ is easy to invert, the complementarity subproblem has the closed form:

$$z^k = M^{-1}(Nx^k + b)$$
$$x^{k+1} = \max(0, z^k)$$

All specific methods below are specializations of this formula.

### Sweep order and symmetry

Gauss-Seidel style methods update $x_i$ in-place, so the sweep order changes the fixed point they approach. A symmetric variant (forward then backward sweep) halves this bias at twice the per-iteration cost; Jacobi has no order dependency because it updates all entries in parallel.

## 1. Jacobi Method ✅ (Implemented)

### Splitting

- $M = D$ (diagonal of $A$)
- $N = D - A$

### Update Rule

$$x_i^{k+1} = \max(0, x_i^{k} - r_i / A_{ii}) \quad \text{for all } i \text{ in parallel}$$

where $r = Ax^k - b$.

Equivalently: $z = x^k - r \oslash \text{diag}(A)$; $x^{k+1} = \max(0, z)$.

### Properties

- **Time**: $O(n)$ per iteration
- **Storage**: $O(n)$
- **Convergence**: Linear (if converges)
- **Parallelization**: Fully parallel

### Advantages/Disadvantages

✅ Embarrassingly parallel - all updates independent
✅ Simple to implement
❌ Slower convergence than Gauss-Seidel
❌ May not converge for some problems

### DART Implementation

```cpp
dart::math::JacobiSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();

// Optional: damped Jacobi via relaxation (0 < relaxation <= 2)
options.relaxation = 1.0;

solver.solve(problem, x, options);
```

## 2. Projected Gauss-Seidel (PGS) ✅ (Implemented)

### Splitting

- $M = D + L$ (diagonal + lower triangular)
- $N = -U$ (negative upper triangular)

### Update Rule

For $i = 1$ to $n$ (forward sweep):
$$r_i = -b_i + \sum_{j < i} A_{ij} x_j^{k+1} + \sum_{j \geq i} A_{ij} x_j^{k}$$
$$x_i^{k+1} = \max(0, x_i^{k} - r_i / A_{ii})$$

The first sum uses freshly updated values.

### Pseudocode

```
function PGS(A, b, x, max_iter, epsilon):
  for iter = 1 to max_iter:
    x_old = x
    for i = 1 to n:
      r_i = dot(A[i,:], x) - b_i
      x[i] = max(0, x[i] - r_i / A[i,i])

    if ||x - x_old|| < epsilon:
      break
  return x
```

### Properties

- **Time**: $O(nk)$ per iteration ($k$ = max non-zeros per row)
- **Storage**: $O(n)$
- **Convergence**: Linear for symmetric PSD
- **Parallelization**: Sequential
- **DART support**: `dart::math::PgsSolver` implements both standard and boxed
  LCPs (including friction `findex` coupling)
  - Configure solver-specific parameters (e.g., randomized sweep order) via
    `dart::math::PgsSolver::setParameters()`.
  - Use `dart::math::LcpOptions::relaxation` to enable PSOR-style relaxation
    (`1.0` = PGS, `>1` = over-relaxation, `<1` = under-relaxation).
- **DART support**: `dart::math::NncgSolver` accelerates PGS for boxed LCPs,
  sharing the same bounds and `findex` handling.

### Advantages/Disadvantages

✅ Fast $O(n)$ iterations
✅ Low memory footprint
✅ Works well for contact problems
✅ In-place updates
❌ Sequential (not parallel)
❌ Convergence depends on sweep order
❌ May be slow for ill-conditioned problems

### Use Cases

- Implemented as `dart::math::PgsSolver` (standard and boxed LCP with optional
  friction index mapping for contact)
- Real-time rigid body simulation
- Contact force computation
- Interactive applications
- First-choice for most physics engines

## 3. Projected SOR (PSOR) ✅ (Implemented via `PgsSolver` relaxation)

### Splitting

- $M = (D + \lambda L)/\lambda$
- $N = ((1-\lambda)D - \lambda U)/\lambda$

### Update Rule

For $i = 1$ to $n$:
$$r_i = -b_i + \sum_{j < i} A_{ij} x_j^{k+1} + \sum_{j \geq i} A_{ij} x_j^{k}$$

Coordinate-wise relaxation step:
$$\tau^* = -r_i / A_{ii} \quad \text{(minimizer of local quadratic)}$$
$$\tau_\lambda = -\lambda \cdot r_i / A_{ii} \quad \text{(relaxed step)}$$
$$x_i^{k+1} = \max(0, x_i^{k} + \tau_\lambda)$$

Derived by using $A = L + D + U$ with the splitting $M = (D + \lambda L)/\lambda$ and $N = ((1-\lambda)D - \lambda U)/\lambda$, so the residual is scaled before projecting. $\lambda$ can be seen as a relaxation of $Ax - b$.

### Relaxation Parameter $\lambda$

- **$\lambda = 1$**: Reduces to PGS
- **$0 < \lambda < 1$**: Under-relaxation (more stable)
- **$1 < \lambda < 2$**: Over-relaxation (faster convergence)
- **Typical**: $\lambda = 1.2$ to $1.5$

### DART Configuration

```cpp
dart::math::PgsSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.relaxation = 1.3;  // PSOR-style over-relaxation
options.maxIterations = 100;
solver.solve(problem, x, options);
```

### Coordinate-descent view

For symmetric $A$, PGS/PSOR are equivalent to coordinate descent on the quadratic objective:

$$f(x) = \frac{1}{2} x^T A x - b^T x$$

Updating coordinate $i$ minimizes the 1D polynomial:

$$g_i(\tau) = \frac{1}{2} A_{ii} \tau^2 + r_i \tau + \text{const}$$
$$\tau^* = -r_i / A_{ii} \quad \text{(minimizer)}$$
$$\tau_\lambda = -\lambda r_i / A_{ii} \quad \text{(relaxed minimizer, PSOR)}$$
$$x_i^{k+1} = \max(0, x_i^k + \tau_\lambda)$$

If $A_{ii} \leq 0$, the local quadratic is non-convex; clip with the projection above and consider regularizing the diagonal to keep the method stable.

### Properties

- **Time**: $O(nk)$ per iteration
- **Storage**: $O(n)$
- **Convergence**: Can be faster than PGS with good $\lambda$
- **Parallelization**: Sequential

### Advantages/Disadvantages

✅ Faster convergence than PGS with good $\lambda$
✅ Same computational cost as PGS
❌ Requires tuning $\lambda$ parameter
❌ Bad $\lambda$ can make convergence worse

## 4. Symmetric PSOR ✅ (Implemented)

### Algorithm

Forward sweep ($i = 1$ to $n$) followed by backward sweep ($i = n$ to $1$).

### Properties

- Reduces sweep-order dependency
- $2\times$ cost per iteration
- Better convergence behavior

### DART Implementation

```cpp
dart::math::SymmetricPsorSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.relaxation = 1.2;  // Optional PSOR-style relaxation
solver.solve(problem, x, options);
```

### Generic projected iteration

A simple implementation shared by Jacobi/PGS/PSOR:

For $k = 1, \ldots, N$:
$$z = M^{-1}(Nx + b)$$
$$x = \max(0, z) \quad \text{(or box projection for BLCP)}$$

Sweep order matters for Gauss-Seidel/PSOR. A symmetric variant performs one forward and one backward sweep to mitigate order bias (useful for PSOR/PGS; not for Jacobi which is already order-free).

## 5. Blocked Gauss-Seidel (BGS) ✅ (Implemented)

### Description

Applies Gauss-Seidel to blocks of variables rather than individual variables. Also known as Nonsmooth Contact Dynamics (NSCD).

### Block Structure for Contact Problems

Block $i$ contains variables for contact point $i$:

- Normal impulse $x_{n,i}$
- Tangential impulses $x_{t1,i}$, $x_{t2,i}$ (or 4 for pyramid)
- Slack variable $\beta_i$ (for friction cone)

### Algorithm

```
for iter = 1 to max_iter:
  for each block i:
    # Compute residual for block
    r_i = -b_i + sum(A_{ij} * x_j, j=1..num_blocks)

    # Solve sub-LCP for block i
    x_i = SolveSubLCP(A_{ii}, r_i, bounds_i)
```

In code, form the local right-hand side as $b'_i = b_i - (A_{ij} x_j)_{j \in \text{blocks}, j \neq i}$, then solve $A_{ii} x_i = b'_i$ under the per-block bounds.

### DART Implementation

```cpp
dart::math::BgsSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();

// Optional explicit block sizes (must sum to n).
dart::math::BgsSolver::Parameters params;
params.blockSizes = {3, 3, 3}; // Example: three contact blocks
options.customOptions = &params;

solver.solve(problem, x, options);
```

### Sub-LCP Solvers

- **1D normal**: Direct solve
- **2D/3D friction**: Direct geometric method
- **4D pyramid**: Direct or small iterative
- **General**: Any LCP solver

> Note: DART uses `DirectSolver` for standard blocks up to 3 variables and
> falls back to `DantzigSolver` for boxed or larger blocks.

For common contact splittings:

- Normal sub-block: 1D projection ($x_n = \max(0, x_n - r_n / A_{nn})$).
- Friction sub-block: 2D/4D problem; if the reduced matrix is symmetric PSD, solve with a tiny PCG or a direct enumerator.

### Properties

- **Time**: $O(n \cdot b^3)$ per iteration ($b$ = block size)
- **Storage**: $O(n)$
- **Convergence**: Linear
- **Block size**: Typically 1-6 variables

### Advantages/Disadvantages

✅ Natural for contact problems
✅ Can use exact sub-solvers
✅ Better convergence than scalar PGS
✅ Flexible block definitions
❌ More complex than PGS
❌ Higher cost per iteration

### Use Cases

- Contact force problems
- One block per contact point
- When sub-problems are small and cheap
- Hierarchical blocking: partition a configuration into sub-blocks (e.g., joints vs contacts) and apply specialized solvers per block, repeating sweeps until convergence.

### Practical blocking tips

- Keep the **normal block** tiny (often 1×1) so it can be solved by direct projection.
- Friction blocks may be **non-symmetric**; small dimension allows direct enumeration or a tiny PCG even when symmetry is missing.
- For joints or tightly coupled contacts, group them into a single block and solve with a symmetric solver (e.g., PCG) while keeping the scalar per-contact structure for the rest.
- Hierarchical strategies work well: first block by joint/contact type, then run BGS sweeps inside each group.

### Staggered normal/friction solve (contact-specific)

When contacts are split into normal ($x_N$) and friction ($x_F$) variables (plus optional slack):

1. Solve normal LCP: $x_N$ via PGS/PSOR/PCG on $A_{NN}$, with $x_F$ held fixed
2. Update friction bounds: $|x_F| \leq \mu x_N$
3. Solve friction subproblem: small LCP or QP on $A_{FF}$, with $x_N$ fixed
4. Repeat until fixed point

Normal blocks are often symmetric PSD, making the normal step amenable to QP/PCG; the friction block can be treated as a cone-constrained QP. One sweep already gives a useful warm start for higher-accuracy methods.

### Extension to Boxed LCP (BLCP)

Use the same splitting but project onto bounds:

$$z^k = M^{-1}(Nx^k + b)$$
$$x^{k+1} = \min(u, \max(l, z^k))$$

For contact problems, $l$ and $u$ are often functions of the normal impulse ($\pm\mu N$), so the projection step should recompute bounds whenever $N$ changes.

## 6. Nonsmooth Nonlinear Conjugate Gradient (NNCG) ✅ (Implemented)

### Description

Conjugate gradient acceleration of PGS using Fletcher-Reeves formula.

### Algorithm

```
function NNCG(A, b, x, max_iter):
  x = PGS(x)              # warm start
  r = PGS(x) - x          # residual (acts as gradient)
  p = -r                  # search direction

  for k = 1..max_iter:
    x = PGS(x + p)        # projected step along p
    r_new = PGS(x) - x

    beta = ||r_new||² / ||r||²
    if beta > 1:          # restart if direction is bad
      p = -r_new
    else:
      p = -r_new + beta * p

    if ||r_new|| < tol: return x
    r = r_new
```

The beta formula is $\beta = \|r_{\text{new}}\|^2 / \|r\|^2$ (Fletcher-Reeves).

### DART Implementation

```cpp
dart::math::NncgSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();

dart::math::NncgSolver::Parameters params;
params.pgsIterations = 1;
params.restartInterval = 10;
params.restartThreshold = 1.0;
options.customOptions = &params;

solver.solve(problem, x, options);
```

### Properties

- **Time**: $O(n)$ per iteration (same as PGS)
- **Storage**: $O(n)$ (need to store $p$ vector)
- **Convergence**: Linear to superlinear (empirically)
- **Restart**: Every 10-20 iterations

### Advantages/Disadvantages

✅ Better convergence than PGS
✅ Same per-iteration cost as PGS
✅ Handles large mass ratios better
❌ Slightly more complex
❌ Needs restart strategy

### Use Cases

- Large-scale problems
- Better accuracy than PGS
- When PGS converges too slowly

## 7. Subspace Minimization (PGS-SM) ✅ (Implemented)

### Description

Two-phase hybrid: PGS for active set estimation + direct solve for refinement.

### Algorithm

```
while not converged:
  # Phase 1: PGS for active set estimation
  for k_pgs iterations:
    PGS_iteration(x)

  # Phase 2: Subspace minimization
  for k_sm iterations:
    # Partition into L (lower), U (upper), A (active)
    L = {i | x_i = l_i}
    U = {i | x_i = u_i}
    A = {i | l_i < x_i < u_i}

    # Solve reduced system
    A_{AA} * x_A = b_A - A_{AL}*l - A_{AU}*u

    # Project back
    x_A = min(u_A, max(l_A, x_A))
```

### DART Implementation

```cpp
dart::math::SubspaceMinimizationSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();

dart::math::SubspaceMinimizationSolver::Parameters params;
params.pgsIterations = 5;
options.customOptions = &params;

solver.solve(problem, x, options);
```

### Properties

- **Time**: $O(n)$ for PGS + $O(|\mathcal{A}|^3)$ for subspace
- **Storage**: $O(n)$
- **Convergence**: Better than pure PGS

### Use Cases

- Small to medium problems with joints
- When PGS alone is not accurate enough
- Problems with clear active/inactive distinction

### Full algorithm (from Silcowitz et al.)

Input: $k_{\text{pgs}}$ (PGS warm-start iterations), $k_{\text{sm}}$ (subspace iterations), $l$, $u$

```
while not converged:
  x = run PGS for at least k_pgs iterations
  if termination reached: return x

  for k = 1..k_sm:
    L = { i | x_i = l_i }, U = { i | x_i = u_i }, A = others

    # Solve reduced system on active set
    solve A_AA x_A = b_A - A_AL l - A_AU u

    # Reconstruct residuals for bound sets
    v_L = A_LA x_A + A_LL l + A_LU u - b_L
    v_U = A_UA x_A + A_UL l + A_UU u - b_U

    # Project active solution back to the box
    x_A = min(u_A, max(l_A, x_A))
    x   = assemble [x_L = l; x_U = u; x_A]

    if termination reached (e.g., active set unchanged): return x
```

The partition sets are $\mathcal{L} = \{ i \mid x_i = l_i \}$, $\mathcal{U} = \{ i \mid x_i = u_i \}$, $\mathcal{A} = \text{others}$.

## 8. Red-Black Gauss-Seidel ✅ (Implemented)

### Description

Two-color blocking for parallelization.

### Algorithm

```
# Color variables: red or black (checkerboard pattern)
for iter = 1 to max_iter:
  # Phase 1: Update all red variables in parallel
  parallel for i in red_indices:
    update x_i

  # Phase 2: Update all black variables in parallel
  parallel for i in black_indices:
    update x_i
```

### Properties

- **Parallelization**: 2-phase parallel
- **Convergence**: Between Jacobi and Gauss-Seidel

### DART Implementation

```cpp
dart::math::RedBlackGaussSeidelSolver solver;
dart::math::LcpOptions options = solver.getDefaultOptions();
options.relaxation = 1.0;
solver.solve(problem, x, options);
```

> Note: DART uses an even/odd index partition as the red/black sets.

### Use Cases

- GPU implementations
- Parallel computing
- Domain decomposition

## Termination Criteria

### Absolute Convergence

$$\|Ax - b\| < \epsilon_{\text{abs}}$$

Typical: $\epsilon_{\text{abs}} = 10^{-6}$

### Relative Convergence

$$\|x^{k+1} - x^k\| / \|x^k\| < \epsilon_{\text{rel}}$$

Typical: $\epsilon_{\text{rel}} = 10^{-4}$

### Complementarity

$$\|x \odot (Ax - b)\| < \epsilon_{\text{comp}}$$

where $\odot$ is element-wise product.

### Infinity Norm (Cheap) and Divergence Check

$$\delta_k = \max_i |x_i^k|$$
$$\rho_k = \max(\delta_k, \delta_{k-1})$$

**Divergence detection**: If $\delta_k > \gamma$, stop (diverging), where $\gamma$ is a user cap.

**Contraction**: If $|\delta_k - \delta_{k-1}| / \max(1, \delta_{k-1}) < \epsilon_{\text{rel}}$, converged.

The infinity norm can be accumulated during the sweep with no extra passes.

**Practical sweep-time test** (cheap and easy to implement):

```
delta_old = ∞
rho = ∞
for k = 1..max_iter:
  delta = 0
  for each i:
    x_i = update_i(...)
    delta = max(delta, |x_i|)

  # Divergence: getting larger
  if delta > rho: return "diverging"
  rho = max(delta, delta_old)
  delta_old = delta

  # Relative contraction
  if delta < rho * (1 - eps_rel): return "converged"
```

This uses only per-iteration maxima; no vector norms are required.

### Maximum Iterations

$$\text{iter} \geq \text{max\_iter}$$

Typical: 50-100 for real-time, 1000+ for accuracy.

## Merit Functions

### QP Formulation

$$\phi(x) = \frac{1}{2} x^T A x + x^T b$$

### Modified for $x = 0$ Safety

$$\phi(x) = \|x \odot (Ax - b)\|$$

### Infinity Norm (Cheap to Compute)

$$\phi(x) = \max_i |x_i|$$

### Complementarity Merit

$$\theta_{\text{compl}}(x) = x^T (Ax - b)$$

Use only when $x \geq 0$; also ensure $Ax - b \geq 0$ when $x = 0$.

## Comparison Table

| Method     | Status           | Parallel | Convergence | Best For              |
| ---------- | ---------------- | -------- | ----------- | --------------------- |
| Jacobi     | ✅ (Implemented) | Yes      | Slow        | Parallel hardware     |
| PGS        | ✅ (Implemented) | No       | Linear      | Real-time boxed LCP   |
| PSOR       | ✅ (Implemented) | No       | Linear      | Real-time with tuning |
| Symm. PSOR | ✅ (Implemented) | No       | Linear      | Reduced sweep bias    |
| BGS        | ✅ (Implemented) | No       | Linear      | Contact problems      |
| NNCG       | ✅ (Implemented) | No       | Superlinear | Large-scale           |
| PGS-SM     | ✅ (Implemented) | No       | Better      | Medium problems       |
| Red-Black  | ✅ (Implemented) | 2-phase  | Medium      | GPU                   |

## Implementation Priority

### Phase 1 (Essential for Real-Time)

1. **Termination criteria** - Multiple stopping conditions
2. **Merit functions** - For convergence monitoring

### Phase 2 (For Contact Problems)

5. ✅ **BGS** - Natural for multi-contact scenarios
6. ✅ **Direct 2D/3D sub-solvers** - For BGS blocks

### Phase 3 (Advanced)

7. **NNCG** - Better convergence for large systems
8. ✅ **PGS-SM** - Hybrid approach

## When to Use Projection Methods

### Advantages

✅ $O(n)$ per-iteration cost
✅ Low memory $O(n)$
✅ Simple to implement
✅ Matrix-free possible
✅ Predictable performance

### Disadvantages

❌ Linear convergence (slow)
❌ Many iterations needed (50-500)
❌ Convergence depends on conditioning
❌ May not converge for all matrices

### Recommended For

- Real-time interactive simulation
- Contact force problems
- Large-scale systems
- When approximate solutions suffice
- First choice for physics engines

### Not Recommended For

- High accuracy requirements (use Newton instead)
- Very ill-conditioned systems (use pivoting instead)
- When exact solutions needed (use pivoting instead)

## References

### Classical Iterative Methods

1. **Young, D. M.** (1971). _Iterative solution of large linear systems_. Academic press.
   - Classical Gauss-Seidel and SOR methods
   - Chapters 3-4: Convergence theory for splitting methods

2. **Saad, Y.** (2003). _Iterative methods for sparse linear systems_ (2nd ed.). SIAM.
   - Modern treatment of iterative methods
   - Chapter 4: Relaxation methods (Jacobi, GS, SOR)

### Projected Gauss-Seidel for LCP

3. **Murty, K. G.** (1988). _Linear complementarity, linear and nonlinear programming_. Heldermann Verlag.
   - Chapter 10: Iterative methods for LCP
   - Convergence analysis for symmetric PSD matrices

4. **Mangasarian, O. L.** (1977). "Solution of symmetric linear complementarity problems by iterative methods". _Journal of Optimization Theory and Applications_, 22(4), 465-485.
   - Convergence theory for PGS on symmetric matrices
   - Sufficient conditions for convergence

5. **Cryer, C. W.** (1971). "The solution of a quadratic programming problem using systematic overrelaxation". _SIAM Journal on Control_, 9(3), 385-392.
   - PSOR method for QP (equivalent to LCP when A symmetric)
   - Optimal relaxation parameter analysis

### Blocked Gauss-Seidel (NSCD)

6. **Moreau, J. J.** (1988). "Unilateral contact and dry friction in finite freedom dynamics". _Nonsmooth mechanics and applications_, 1-82. Springer.
   - Nonsmooth contact dynamics formulation
   - Theoretical foundation for BGS in contact mechanics

7. **Jean, M.** (1999). "The non-smooth contact dynamics method". _Computer methods in applied mechanics and engineering_, 177(3-4), 235-257.
   - NSCD method = Blocked Gauss-Seidel for contacts
   - Per-contact block structure
   - Applications to granular materials

8. **Acary, V., & Brogliato, B.** (2008). _Numerical methods for nonsmooth dynamical systems: applications in mechanics and electronics_. Springer Science & Business Media.
   - Comprehensive treatment of NSCD/BGS
   - Chapter 7: Time-stepping schemes
   - Chapter 8: Numerical methods for LCP/MLCP

9. **Anitescu, M., & Tasora, A.** (2010). "An iterative approach for cone complementarity problems for nonsmooth dynamics". _Computational Optimization and Applications_, 47(2), 207-235.
   - Convergence analysis of Gauss-Seidel for friction cones
   - Practical implementation considerations

### Nonsmooth Nonlinear Conjugate Gradient (NNCG)

10. **Silcowitz, M., Niebe, S., & Erleben, K.** (2009). "Nonsmooth nonlinear conjugate gradient method for interactive contact force problems". _The Visual Computer_, 25(5), 893-905.
    - NNCG method for contact problems
    - Fletcher-Reeves formula adaptation
    - Better convergence than PGS for large mass ratios

11. **Silcowitz, M., Niebe, S., & Erleben, K.** (2010). "A nonsmooth nonlinear conjugate gradient method for interactive contact force problems". _The Visual Computer_, 26(6-8), 893-901.
    - Extended version with more implementation details
    - Performance comparison with PGS and other methods

12. **Fletcher, R., & Reeves, C. M.** (1964). "Function minimization by conjugate gradients". _The Computer Journal_, 7(2), 149-154.
    - Original Fletcher-Reeves conjugate gradient
    - Foundation for NNCG adaptation

### Subspace Minimization

13. **Silcowitz-Hansen, M., Erleben, K., & Niebe, S.** (2010). "A nonsmooth Newton method with applications to computer animation". In _MATHMOD 2009-6th Vienna International Conference on Mathematical Modelling_.
    - Subspace minimization approach
    - Combines PGS with direct solvers
    - Active set prediction

14. **Kaufman, D. M., Sueda, S., James, D. L., & Pai, D. K.** (2008). "Staggered projections for frictional contact in multibody systems". _ACM Transactions on Graphics (TOG)_, 27(5), 1-11.
    - Related subspace approach
    - Staggering for normal/tangential forces

### Red-Black and Parallel Methods

15. **Adams, L. M.** (1982). "Iterative algorithms for large sparse linear systems on parallel computers". _NASA Technical Memorandum_, 83267.
    - Red-black ordering for parallelization
    - Applications to domain decomposition

16. **Hasenbusch, M., Lana, A., & Marcu, M.** (1999). "Accelerated Monte Carlo for fermions". _Nuclear Physics B-Proceedings Supplements_, 73(1-3), 864-866.
    - Red-black ordering applications
    - Checkerboard pattern for parallel updates

### Convergence Theory

17. **Luo, Z. Q., & Tseng, P.** (1992). "On the convergence of the coordinate descent method for convex differentiable minimization". _Journal of Optimization Theory and Applications_, 72(1), 7-35.
    - General convergence theory
    - Applicable to PGS as coordinate descent

18. **Bertsekas, D. P.** (1999). _Nonlinear programming_ (2nd ed.). Athena scientific.
    - Chapter 2.7: Coordinate descent methods
    - Convergence for convex problems

### Physics Simulation Applications

19. **Catto, E.** (2005). "Iterative dynamics with temporal coherence". In _Game Developer Conference_.
    - PGS in Box2D physics engine
    - Practical implementation and warm-starting

20. **Coumans, E.** (2015). "Bullet physics simulation". In _ACM SIGGRAPH 2015 Courses_, 1-87.
    - PGS and NNCG in Bullet Physics
    - Performance comparisons and best practices

21. **Tonge, R., Benevolenski, F., & Voroshilov, A.** (2012). "Mass splitting for jitter-free parallel rigid body simulation". _ACM Transactions on Graphics (TOG)_, 31(4), 1-8.
    - Jacobi-style parallel contact resolution
    - Domain decomposition strategies

### Implementation and Performance

22. **Erleben, K.** (2013). "Numerical methods for linear complementarity problems in physics-based animation". In _ACM SIGGRAPH 2013 Courses_, 8.
    - Comprehensive survey of PGS, PSOR, NNCG, etc.
    - Implementation details and performance analysis

23. **Erleben, K., Silcowitz-Hansen, M., & Niebe, S.** (2017). "Numerical methods for linear complementarity problems in physics-based animation". _Synthesis Lectures on Computer Graphics and Animation_, 11(2), 1-159.
    - Extended book version of the survey
    - Definitive reference for projection methods
    - Detailed pseudocode and analysis

---

**Navigation**: [← Pivoting Methods](03_pivoting-methods.md) | [Newton Methods →](05_newton-methods.md)
