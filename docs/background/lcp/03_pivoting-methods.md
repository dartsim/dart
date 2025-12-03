# Pivoting Methods for LCP

**Navigation**: [← Overview](02_overview.md) | [Projection Methods →](04_projection-methods.md)

## Overview

Pivoting methods solve LCPs by exploiting their combinatorial nature through enumerating solution candidates. They can find exact solutions when they exist.

## Active / Free index sets

Let `y = Ax + b`. Partition the indices into:

- Active set `A = { i | x_i > 0 }`
- Free set `F = { i | y_i > 0 }`

Strict complementarity implies `A ∩ F = ∅` and `A ∪ F = I`, where `I = {1, …, n}`. Reordering rows/columns with this partition yields a reduced problem that makes the pivot structure explicit:

```
[ I_F   -A_FA ] [ y_F ] = b_F
  A_AF    A_AA   x_A   = -b_A
```

The stacked matrix built from identity columns (for `y_F`) and negated `A` columns (for `x_A`) is a **complementarity matrix**. Choosing, for every index, whether the column comes from `I` or from `-A` enumerates up to `2^n` distinct matrices; this combinatorial explosion is what gives pivoting its exponential worst case. For symmetric positive (semi-)definite `A`, the same conditions can instead be written as a quadratic program, avoiding explicit pivot enumeration.

### Constructing a complementarity matrix

1. Decide the membership of each index (active vs free).
2. Zero out columns in `A` that belong to `F` (since `x_F = 0`) and rows that belong to `A` (since `y_A = 0`).
3. Reorder unknowns to `[y_F; x_A]` and build `C = [I_F, -A_FA]`.
4. Solve `C s = b` for `s = [y_F; x_A]` and keep only solutions with `s ≥ 0`.

Each feasible `s` corresponds to one candidate LCP solution. Pivoting methods search these candidates without testing all `2^n` possibilities when possible.

## 1. Direct Methods for Small-Sized Problems

### Description

Geometric approaches for 2D and 3D LCPs using complementarity cones and angle intersection tests. They explicitly enumerate all complementarity matrices but remain cheap because the dimension is tiny.

### 2D Algorithm

Given LCP with 2D vector `x`:

```
Test all 4 complementarity cones:
  C₁ = [e₁, e₂] (both from identity)
  C₂ = [a₁, e₂] (first from A, second from identity)
  C₃ = [e₁, a₂] (first from identity, second from A)
  C₄ = [a₁, a₂] (both from A)

For each cone C:
  if b is in positive cone of C:
    return solution x = C⁻¹b
```

### 3D Algorithm

Test all 8 complementarity cones using spherical triangle inclusion:

```
For cone C = [c₁, c₂, c₃]:
  if det(C) > 0 and
     (c₁ × c₂)·b >= 0 and
     (c₂ × c₃)·b >= 0 and
     (c₃ × c₁)·b >= 0:
    return x = C⁻¹b
```

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

This tests whether `b` lies in the cone spanned by `c1, c2` by checking that the signed areas (determinants) with `b` have opposite signs.

### Properties

- **Time Complexity**: O(2ⁿ) for n-dimensional problem
- **Space Complexity**: O(n)
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

Given `(A, b, lo, hi)`, solve:

```
Ax = b + w, where each (xᵢ, wᵢ) satisfies one of:
  (1) x = lo, w >= 0  (at lower bound)
  (2) x = hi, w <= 0  (at upper bound)
  (3) lo < x < hi, w = 0  (free variable)
```

### Key Features

- **Bounded variables**: `lo[i] <= x[i] <= hi[i]`
- **Unbounded variables**: First `nub` variables have infinite bounds
- **Friction support**: `findex[i]` for friction cone constraints
  - When `findex[i] >= 0`: bounds become `hi[i] = |hi[i] * x[findex[i]]|`, `lo[i] = -hi[i]`
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
#include <dart/math/lcp/pivoting/dantzig/Lcp.hpp>

using namespace dart::math;

constexpr int n = 6;
double A[n*n], x[n], b[n], w[n], lo[n], hi[n];
int findex[n];

// Initialize A, b, lo, hi, findex...

bool success = SolveLCP<double>(
    n, A, x, b, w,
    0,        // nub (unbounded variables)
    lo, hi,
    findex,
    false     // earlyTermination
);
```

### Properties

- **Time Complexity**: O(n⁴) worst case
- **Space Complexity**: O(n²)
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

```
Find z such that:
  Mz = q + w
  z >= 0, w >= 0
  z^T w = 0
```

### Algorithm Overview

```
1. Initialize with artificial variable z₀
2. Iterate:
   a. Select entering variable (complementary to leaving)
   b. Perform ratio test for leaving variable
   c. Pivot to update basis
   d. If z₀ leaves basis, solution found
3. Extract final solution
```

### DART Implementation

```cpp
#include <dart/math/lcp/pivoting/LemkeSolver.hpp>

using namespace dart::math;

Eigen::MatrixXd M(n, n);
Eigen::VectorXd q(n), z(n);

// Initialize M and q...

int result = Lemke(M, q, &z);

// Validate solution
bool valid = validate(M, z, q);
```

### Properties

- **Time Complexity**: O(n⁴) worst case
- **Space Complexity**: O(n²)
- **Convergence**: Exact solution (if exists)
- **Matrix Requirements**: None (general LCP)

### Use Cases

- Standard LCP problems
- When matrix A is not symmetric
- Validation and testing

## 4. Baraff Incremental Pivoting ❌ (Not Implemented)

### Description

Incrementally builds active/free index sets while maintaining complementarity constraints as invariants.

The method keeps three sets:

- **Active A**: indices where `x_i > 0`
- **Free F**: indices where `y_i = (Ax+b)_i > 0`
- **Unprocessed U**: indices not yet assigned

### Key Concepts

- **Index Sets**:
  - **Active set A**: indices where `xᵢ > 0`
  - **Free set F**: indices where `yᵢ = (Ax + b)ᵢ > 0`
  - **Unprocessed U**: indices not yet assigned
- **Blocking constraints**: bounds on how far a candidate variable may move before violating complementarity in the current basis

### Algorithm Pseudocode

```
Given symmetric PSD A, b. Start with x = 0, y = b, A = ∅, F = ∅, U = I

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

- Only one index swaps at a time, so an incremental factorization of `A_AA` keeps the inner loop O(n²) while the outer loop runs O(n) times (practical O(n³), worst-case O(n⁴)).
- Works best when `A` is symmetric positive semidefinite (common for contact problems), where it mirrors an active-set QP solve.

### Properties

- **Time Complexity**: O(n⁴) with incremental factorization
- **Space Complexity**: O(n²)
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

| Method       | Status          | Problem Type | Time  | Matrix Requirements |
| ------------ | --------------- | ------------ | ----- | ------------------- |
| Direct 2D/3D | Not implemented | LCP          | O(2ⁿ) | None                |
| Dantzig      | ✅ Implemented  | BLCP         | O(n⁴) | None                |
| Lemke        | ✅ Implemented  | LCP          | O(n⁴) | None                |
| Baraff       | Not implemented | LCP          | O(n⁴) | Symmetric PD/PSD    |

## When to Use Pivoting Methods

### Advantages

- ✅ Exact solutions (when they exist)
- ✅ Handle poorly conditioned matrices
- ✅ Work with large mass ratios
- ✅ Guaranteed finite termination

### Disadvantages

- ❌ High computational cost O(n⁴)
- ❌ Require full matrix assembly
- ❌ Not suitable for real-time with large n
- ❌ Memory intensive O(n²)

### Recommended For

- Small to medium problems (n < 100)
- Off-line simulations
- High accuracy requirements
- Poorly conditioned systems
- Validation of other solvers

### Not Recommended For

- Real-time interactive simulation
- Large-scale problems (n > 1000)
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

- Consider incremental factorization (Baraff)
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
