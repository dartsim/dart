# Pivoting Methods for LCP

> **Attribution**: This content is derived from "Contact Handling for Articulated
> Rigid Bodies Using LCP" by Jie Tan, Kristin Siu, and C. Karen Liu.
> The original PDF is preserved at [`docs/lcp.pdf`](../../lcp.pdf).

**Navigation**: [← Overview](02_overview.md) | [Index](../README.md) | [Projection Methods →](04_projection-methods.md)

## Overview

Pivoting methods solve LCPs by exploiting their combinatorial nature through enumerating solution candidates. They can find exact solutions when they exist.

## Active / Free index sets

Let $y = Ax - b$. Partition the indices into:

- Active set $\mathcal{A} = \{ i \mid x_i > 0 \}$
- Free set $\mathcal{F} = \{ i \mid y_i > 0 \}$

Strict complementarity implies $\mathcal{A} \cap \mathcal{F} = \emptyset$ and $\mathcal{A} \cup \mathcal{F} = \mathcal{I}$, where $\mathcal{I} = \{1, \ldots, n\}$. Reordering rows/columns with this partition yields a reduced problem that makes the pivot structure explicit:

$$\begin{bmatrix} I_{\mathcal{F}} & -A_{\mathcal{F}\mathcal{A}} \\ 0 & A_{\mathcal{A}\mathcal{A}} \end{bmatrix} \begin{bmatrix} y_{\mathcal{F}} \\ x_{\mathcal{A}} \end{bmatrix} = \begin{bmatrix} -b_{\mathcal{F}} \\ b_{\mathcal{A}} \end{bmatrix}$$

The stacked matrix built from identity columns (for $y_{\mathcal{F}}$) and negated $A$ columns (for $x_{\mathcal{A}}$) is a **complementarity matrix**. Choosing, for every index, whether the column comes from $I$ or from $-A$ enumerates up to $2^n$ distinct matrices; this combinatorial explosion is what gives pivoting its exponential worst case. For symmetric positive (semi-)definite $A$, the same conditions can instead be written as a quadratic program, avoiding explicit pivot enumeration.

### Constructing a complementarity matrix

1. Decide the membership of each index (active vs free).
2. Zero out columns in $A$ that belong to $\mathcal{F}$ (since $x_{\mathcal{F}} = 0$) and rows that belong to $\mathcal{A}$ (since $y_{\mathcal{A}} = 0$).
3. Reorder unknowns to $[y_{\mathcal{F}}; x_{\mathcal{A}}]$ and build $C = [I_{\mathcal{F}}, -A_{\mathcal{F}\mathcal{A}}]$.
4. Solve $Cs = b$ for $s = [y_{\mathcal{F}}; x_{\mathcal{A}}]$ and keep only solutions with $s \geq 0$.

Each feasible $s$ corresponds to one candidate LCP solution. Pivoting methods search these candidates without testing all $2^n$ possibilities when possible.

## 1. Direct Methods for Small-Sized Problems

### Description

Geometric approaches for 2D and 3D LCPs using complementarity cones and angle intersection tests. They explicitly enumerate all complementarity matrices but remain cheap because the dimension is tiny.

### 2D Algorithm

Given LCP with 2D vector $x$:

Test all 4 complementarity cones:

- $C_1 = [e_1, e_2]$ (both from identity)
- $C_2 = [a_1, e_2]$ (first from $A$, second from identity)
- $C_3 = [e_1, a_2]$ (first from identity, second from $A$)
- $C_4 = [a_1, a_2]$ (both from $A$)

For each cone $C$: if $b$ is in positive cone of $C$, return solution $x = C^{-1}b$.

### 3D Algorithm

Test all 8 complementarity cones using spherical triangle inclusion:

For cone $C = [c_1, c_2, c_3]$:

- if $\det(C) > 0$ and $(c_1 \times c_2) \cdot b \geq 0$ and $(c_2 \times c_3) \cdot b \geq 0$ and $(c_3 \times c_1) \cdot b \geq 0$:
  - return $x = C^{-1}b$

#### Bitmask implementation (2D)

```
for k = 1..4:
  mask = 2^(k-1)
  c1 = (mask & 0b01) ? e1 : -a1
  c2 = (mask & 0b10) ? e2 : -a2
  # b inside cone iff signed areas with b have opposite sign
  if det(c1, b) * det(c2, b) <= 0:
    x = [c1 c2]^{-1} b
    return x
return "no solution"
```

This tests whether $b$ lies in the cone spanned by $c_1, c_2$ by checking that the signed areas (determinants) with $b$ have opposite signs.

### Properties

- **Time Complexity**: $O(2^n)$ for $n$-dimensional problem
- **Space Complexity**: $O(n)$
- **Convergence**: Exact solution (if exists)
- **Applicability**: Only 2D/3D problems

### Use Cases

- Building blocks for blocked splitting methods
- Sub-solvers in BGS for small contact blocks
- Testing and validation

## 2. Dantzig Method ✅ (Implemented in DART)

### Description

BLCP solver using pivoting. Derived from ODE (Open Dynamics Engine).

### Problem Formulation

Given $(A, b, \text{lo}, \text{hi})$, solve:

$Ax = b + w$, where each $(x_i, w_i)$ satisfies one of:

1. $x = \text{lo}$, $w \geq 0$ (at lower bound)
2. $x = \text{hi}$, $w \leq 0$ (at upper bound)
3. $\text{lo} < x < \text{hi}$, $w = 0$ (free variable)

### Key Features

- **Bounded variables**: $\text{lo}_i \leq x_i \leq \text{hi}_i$
- **Unbounded variables**: Represented by $\text{lo} = -\infty$, $\text{hi} = +\infty$ (legacy API uses `nub` to seed the initial unbounded block)
- **Friction support**: `findex[i]` for friction cone constraints
  - When `findex[i] >= 0`: bounds become $\text{hi}_i = |\text{hi}_i \cdot x_{\text{findex}[i]}|$, $\text{lo}_i = -\text{hi}_i$
- **Early termination**: Optional for faster approximate solutions

### Algorithm Pseudocode

```cpp
template <typename Scalar>
bool SolveLCP(int n, Scalar* A, Scalar* x, Scalar* b, Scalar* w,
              int nub, Scalar* lo, Scalar* hi, int* findex) {
  // Initialize basis and perform pivoting
  // 1. Set up initial basis with unbounded variables
  // 2. For each variable not in basis:
  //      a. Find entering variable
  //      b. Find leaving variable (ratio test)
  //      c. Perform pivot operation
  //      d. Update basis
  // 3. Extract solution from final basis
}
```

### DART Implementation

```cpp
#include <dart/math/lcp/pivoting/DantzigSolver.hpp>

using namespace dart::math;

// Build an LcpProblem (boxed LCP with optional findex friction coupling)
LcpProblem problem(A, b, lo, hi, findex);
Eigen::VectorXd x = Eigen::VectorXd::Zero(b.size());

DantzigSolver solver;
LcpResult result = solver.solve(problem, x, solver.getDefaultOptions());
```

The underlying ODE-derived implementation is also available directly:

```cpp
#include <dart/math/lcp/pivoting/dantzig/Lcp.hpp>

using namespace dart::math;

constexpr int n = 6;
double A[n*n], x[n], b[n], w[n], lo[n], hi[n];
int findex[n];

// Initialize A, b, lo, hi, findex...

bool success = SolveLCP<double>(
    n, A, x, b, w,
    0,        // nub (unbounded variables); can be left 0 in most cases
    lo, hi,
    findex,
    false     // earlyTermination
);
```

### Properties

- **Time Complexity**: $O(n^4)$ worst case
- **Space Complexity**: $O(n^2)$
- **Convergence**: Exact solution
- **Matrix Requirements**: None (general BLCP)

### Use Cases

- General BLCP with bounds
- Contact problems with friction
- When exact solutions are needed

## 3. Lemke Method ✅ (Implemented in DART)

### Description

Standard pivoting method for LCP using complementary pivoting.

### Problem Formulation

Find $z$ such that:
$$Mz = q + w$$
$$z \geq 0, \quad w \geq 0$$
$$z^T w = 0$$

### Algorithm Overview

1. Initialize with artificial variable $z_0$
2. Iterate:
   - Select entering variable (complementary to leaving)
   - Perform ratio test for leaving variable
   - Pivot to update basis
   - If $z_0$ leaves basis, solution found
3. Extract final solution

### DART Implementation

```cpp
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

using namespace dart::math;

Eigen::MatrixXd M(n, n);
Eigen::VectorXd q(n), z(n);

// Initialize M and q...

// DART uses w = Mz - b. For the common form w = Mz + q, set b = -q.
Eigen::VectorXd b = -q;
LcpProblem problem(
    M,
    b,
    Eigen::VectorXd::Zero(n),
    Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(n, -1));

LemkeSolver solver;
LcpOptions options = solver.getDefaultOptions();
options.validateSolution = true;
auto result = solver.solve(problem, z, options);
```

### Properties

- **Time Complexity**: $O(n^4)$ worst case
- **Space Complexity**: $O(n^2)$
- **Convergence**: Exact solution (if exists)
- **Matrix Requirements**: None (general LCP)

### Use Cases

- Standard LCP problems
- When matrix A is not symmetric
- Validation and testing

## 4. Direct 2D/3D Solver ✅ (Implemented)

### Description

Direct enumeration of complementarity sets for tiny standard LCPs.

### Algorithm Overview

For each active set $S \subseteq \{1, \ldots, n\}$:

1. Solve $A_{SS} x_S = b_S$
2. Set $x_{\bar{S}} = 0$
3. Compute $w = Ax - b$
4. Accept if $x \geq 0$, $w \geq 0$, and $x^T w \approx 0$

### DART Implementation

```cpp
#include <dart/math/lcp/pivoting/DirectSolver.hpp>

using namespace dart::math;

LcpProblem problem(
    A,
    b,
    Eigen::VectorXd::Zero(n),
    Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(n, -1));

Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
DirectSolver solver;
auto result = solver.solve(problem, x, solver.getDefaultOptions());
```

> Note: DART limits the direct solver to $n \leq 3$ and delegates larger problems
> to the boxed-capable pivoting solver.

### Properties

- **Time Complexity**: $O(2^n)$ (only practical for $n \leq 3$)
- **Space Complexity**: $O(n^2)$
- **Convergence**: Exact solution (if exists)
- **Matrix Requirements**: None (general LCP)

### Use Cases

- 2D/3D LCP validation
- Debugging small contact configurations

## 5. Baraff Incremental Pivoting ✅ (Implemented)

### Description

Incrementally builds active/free index sets while maintaining complementarity constraints as invariants.

The method keeps three sets:

- **Active $\mathcal{A}$**: indices where $x_i > 0$
- **Free $\mathcal{F}$**: indices where $y_i = (Ax-b)_i > 0$
- **Unprocessed $\mathcal{U}$**: indices not yet assigned

### Key Concepts

- **Index Sets**:
  - **Active set $\mathcal{A}$**: indices where $x_i > 0$
  - **Free set $\mathcal{F}$**: indices where $y_i = (Ax - b)_i > 0$
  - **Unprocessed $\mathcal{U}$**: indices not yet assigned
- **Blocking constraints**: bounds on how far a candidate variable may move before violating complementarity in the current basis

### Algorithm Pseudocode

Given symmetric PSD $A$, $b$. Start with $x = 0$, $y = b$, $\mathcal{A} = \emptyset$, $\mathcal{F} = \emptyset$, $\mathcal{U} = \mathcal{I}$:

```
while U not empty:
  j = argmin_{i in U} y_i           # most negative residual
  enter j into tentative active set

  repeat:  # pivoting loop
    # Solve for search direction that increases x_j
    Δx = 0; Δx_j = 1
    Δy = A * Δx

    # Blocking values: how far can we move before violating complementarity?
    B_A = { -x_i / Δx_i | i in A and Δx_i < 0 }
    B_F = { -y_i / Δy_i | i in F and Δy_i < 0 }
    α = min( B_A ∪ B_F ∪ {∞} )

    x = x + α * Δx
    y = y + α * Δy

    if α came from B_A with index q:
      move q from A to F            # swap its complement
    else if α came from B_F with index q:
      move q from F to A
    else:
      # no blocking constraint hit; finalize j
      if x_j > 0: A = A ∪ {j} else F = F ∪ {j}
      U = U \ {j}
      break
  until false
```

Notes:

- Only one index swaps at a time, so an incremental factorization of $A_{\mathcal{A}\mathcal{A}}$ keeps the inner loop $O(n^2)$ while the outer loop runs $O(n)$ times (practical $O(n^3)$, worst-case $O(n^4)$).
- Works best when $A$ is symmetric positive semidefinite (common for contact problems), where it mirrors an active-set QP solve.

### DART Implementation

```cpp
#include <dart/math/lcp/pivoting/BaraffSolver.hpp>

using namespace dart::math;

LcpProblem problem(
    A,
    b,
    Eigen::VectorXd::Zero(n),
    Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity()),
    Eigen::VectorXi::Constant(n, -1));

Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
BaraffSolver solver;
LcpOptions options = solver.getDefaultOptions();
options.validateSolution = true;
auto result = solver.solve(problem, x, options);
```

> Note: DART's implementation assumes symmetric PSD matrices and currently uses
> direct solves per pivot (no incremental factorization yet).

### Properties

- **Time Complexity**: $O(n^4)$ with incremental factorization
- **Space Complexity**: $O(n^2)$
- **Convergence**: Exact solution
- **Matrix Requirements**: Symmetric PD or PSD

### Advantages

- Can handle PSD matrices (common in contact problems)
- Incremental factorization reduces cost
- Maintains complementarity as invariant

### Use Cases

- Contact force problems
- When A is symmetric PSD
- Redundant contact constraints

## Comparison Table

| Method       | Status         | Problem Type | Time     | Matrix Requirements |
| ------------ | -------------- | ------------ | -------- | ------------------- |
| Direct 2D/3D | ✅ Implemented | LCP          | $O(2^n)$ | None                |
| Dantzig      | ✅ Implemented | BLCP         | $O(n^4)$ | None                |
| Lemke        | ✅ Implemented | LCP          | $O(n^4)$ | None                |
| Baraff       | ✅ Implemented | LCP          | $O(n^4)$ | Symmetric PD/PSD    |

## When to Use Pivoting Methods

### Advantages

- ✅ Exact solutions (when they exist)
- ✅ Handle poorly conditioned matrices
- ✅ Work with large mass ratios
- ✅ Guaranteed finite termination

### Disadvantages

- ❌ High computational cost $O(n^4)$
- ❌ Require full matrix assembly
- ❌ Not suitable for real-time with large $n$
- ❌ Memory intensive $O(n^2)$

### Recommended For

- Small to medium problems ($n < 100$)
- Off-line simulations
- High accuracy requirements
- Poorly conditioned systems
- Validation of other solvers

### Not Recommended For

- Real-time interactive simulation
- Large-scale problems ($n > 1000$)
- When approximate solutions suffice
- Matrix-free implementations needed

## Implementation Notes

### For Dantzig (Already Implemented)

- Arrays use C-style indexing
- Matrix A may be modified during solve
- Check return value for solution existence
- Use `findex` for friction cones carefully

### For Lemke (Already Implemented)

- Use Eigen matrices for convenience
- Always validate solution after solving
- Handle non-existent solutions gracefully

### For Future Implementations

- Add incremental factorization to Baraff for faster inner solves
- Use sparse matrix representations when possible
- Implement efficient pivoting strategies
- Add comprehensive unit tests

## References

### Foundational Papers

1. **Cottle, R. W., & Dantzig, G. B.** (1968). "Complementary pivot theory of mathematical programming". _Linear Algebra and its Applications_, 1(1), 103-125.
   - Original complementary pivot algorithm

2. **Lemke, C. E.** (1965). "Bimatrix equilibrium points and mathematical programming". _Management Science_, 11(7), 681-689.
   - Lemke's complementary pivot algorithm with artificial variable

3. **Lemke, C. E., & Howson Jr, J. T.** (1964). "Equilibrium points of bimatrix games". _Journal of the Society for Industrial and Applied Mathematics_, 12(2), 413-423.
   - Original Lemke-Howson algorithm

### Direct Methods

4. **Murty, K. G.** (1988). _Linear complementarity, linear and nonlinear programming_. Heldermann Verlag.
   - Comprehensive treatment of pivoting methods
   - Chapter 3: Principal pivoting methods

5. **Cottle, R. W., Pang, J. S., & Stone, R. E.** (1992). _The linear complementarity problem_. Academic press.
   - Definitive reference on LCP theory
   - Chapters 4-6: Pivoting algorithms and complexity

### Baraff's Incremental Method

6. **Baraff, D.** (1994). "Fast contact force computation for nonpenetrating rigid bodies". _Proceedings of the 21st annual conference on Computer graphics and interactive techniques_, 23-34.
   - Incremental pivoting for contact mechanics
   - Maintains complementarity as invariant
   - O(n^4) with incremental factorization

7. **Baraff, D.** (1989). "Analytical methods for dynamic simulation of non-penetrating rigid bodies". _ACM SIGGRAPH Computer Graphics_, 23(3), 223-232.
   - Earlier work on contact force computation
   - Foundation for incremental pivoting

### ODE Implementation

8. **Smith, R.** (2006). "Open Dynamics Engine" [Software]. http://www.ode.org/
   - Source of DART's Dantzig implementation
   - BLCP solver with friction support
   - License: BSD or LGPL

9. **Smith, R.** (2000). "Open Dynamics Engine v0.035 User Guide".
   - Documentation of ODE's LCP solver
   - Details on `findex` friction cone implementation

### Complexity and Theory

10. **Chung, S. J.** (1989). "NP-completeness of the linear complementarity problem". _Journal of Optimization Theory and Applications_, 60(3), 393-399.
    - Proves general LCP is NP-complete

11. **Cottle, R. W., & Dantzig, G. B.** (1970). "A generalization of the linear complementarity problem". _Journal of Combinatorial Theory_, 8(1), 79-90.
    - Generalized complementarity problems

### Applications to Simulation

12. **Stewart, D. E., & Trinkle, J. C.** (1996). "An implicit time-stepping scheme for rigid body dynamics with inelastic collisions and coulomb friction". _International Journal for Numerical Methods in Engineering_, 39(15), 2673-2691.
    - Time-stepping formulation leading to LCP
    - Contact mechanics application

13. **Anitescu, M., & Potra, F. A.** (1997). "Formulating dynamic multi-rigid-body contact problems with friction as solvable linear complementarity problems". _Nonlinear Dynamics_, 14(3), 231-247.
    - Velocity-level LCP formulation
    - Theoretical foundation for pivoting in dynamics

---

**Navigation**: [← Overview](02_overview.md) | [Projection Methods →](04_projection-methods.md)
