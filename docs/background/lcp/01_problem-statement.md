# Linear Complementarity Problem (LCP)

**Navigation**: [Overview →](02_overview.md)

## Definition

The **Linear Complementarity Problem (LCP)** is defined as:

```
Given A ∈ ℝⁿˣⁿ and b ∈ ℝⁿ, find x ∈ ℝⁿ such that:

  w = Ax - b
  w ≥ 0
  x ≥ 0
  xᵀw = 0    (complementarity condition)
```

> Note: Many references write the standard LCP as `w = Ax + q`. DART uses the
> equivalent convention `w = Ax - b` (i.e., `b = -q`), matching the ODE-style
> `Ax = b + w` form used by the constraint solver.

The complementarity condition `xᵀw = 0` means that for each index `i`:

```
Either xᵢ = 0  OR  wᵢ = 0  (or both)
```

This can also be written component-wise as:

```
For each i ∈ {1, ..., n}:
  wᵢ = (Ax - b)ᵢ ≥ 0
  xᵢ ≥ 0
  xᵢ · wᵢ = 0
```

## Equivalent Formulations

### Minimum Map Reformulation

```
x = min(x, Ax - b)
```

where the minimum is taken component-wise.

### Nonlinear Complementarity Problem (NCP)

```
F(x) = 0
where F(x) = min(x, Ax - b)
```

### Variational Inequality (VI)

```
Find x ≥ 0 such that:
  (Ax - b)ᵀ(y - x) ≥ 0  for all y ≥ 0
```

### Quadratic Programming (QP) - When A is Symmetric PD

```
minimize: ½xᵀAx - xᵀb
subject to: x ≥ 0
```

## Problem Variants

### Standard LCP

The basic form as defined above with unbounded variables constrained to be non-negative.

### Boxed LCP (BLCP)

LCP with box constraints on variables:

```
Find x such that:
  l ≤ x ≤ u
  w = Ax - b
  For each i:
    - If xᵢ = lᵢ, then wᵢ ≥ 0
    - If xᵢ = uᵢ, then wᵢ ≤ 0
    - If lᵢ < xᵢ < uᵢ, then wᵢ = 0
```

Typical bounds:

- Lower: `l ≤ 0` (often `-∞` or `-μN` for friction)
- Upper: `u ≥ 0` (often `+∞` or `+μN` for friction)

### Mixed LCP (MLCP)

Combination of equality constraints and complementarity:

```
Find x, z such that:
  Ax + Bz + c = 0    (equality constraints)
  w = Cx + Dz + e
  w ≥ 0, z ≥ 0
  zᵀw = 0            (complementarity)
```

### LCP with Friction (FLCP)

Special BLCP where bounds depend on other variables:

```
For friction at contact i:
  -μNᵢ ≤ Fₜᵢ ≤ μNᵢ
where:
  - Nᵢ is normal force
  - Fₜᵢ is tangential friction force
  - μ is friction coefficient
```

In DART, this coupling is represented using the `findex` array:

- For a tangential variable `i` coupled to normal `j`:
  - Set `findex[i] = j`
  - Store the coefficient in the box bounds: `lo[i] = -μ`, `hi[i] = +μ`
  - The effective bounds are interpreted as `|x[i]| ≤ μ·x[j]`

## Applications in Physics-Based Simulation

### 1. Contact Mechanics

**Unilateral Contact Constraints**:

```
For each contact point i:
  dᵢ ≥ 0         (non-penetration)
  Nᵢ ≥ 0         (no adhesion)
  Nᵢ · dᵢ = 0    (complementarity: either separated or in contact)

where:
  - dᵢ = gap distance (signed distance function)
  - Nᵢ = normal contact force
```

**LCP Formulation**:
After time discretization and linearization:

```
x = [N₁, N₂, ..., Nₙ]ᵀ  (normal forces)
Ax - b represents the constraint-space residual
Complementarity ensures forces are zero when separated
```

### 2. Friction Modeling

**Coulomb Friction as BLCP**:

```
For contact i with normal force Nᵢ:
  Friction cone: ||Fₜᵢ|| ≤ μNᵢ

Discretized (pyramid approximation):
  -μNᵢ ≤ Fₜᵢˣ ≤ μNᵢ
  -μNᵢ ≤ Fₜᵢʸ ≤ μNᵢ

BLCP formulation:
  lᵢ = -μNᵢ
  uᵢ = +μNᵢ
```

### 3. Joint Limits

**Joint Constraints**:

```
For revolute joint with limits θₘᵢₙ ≤ θ ≤ θₘₐₓ:

BLCP formulation:
  l = θₘᵢₙ
  u = θₘₐₓ
  Forces active only at limits
```

### 4. Rigid Body Dynamics

**Time-Stepping Scheme**:

```
Velocity-level formulation:
  M(v₊ - v₋) = h·fₑₓₜ + Jᵀλ

where:
  - M = mass matrix
  - v₋, v₊ = velocities before/after contact
  - h = time step
  - J = contact Jacobian
  - λ = contact impulses (LCP variable)

LCP emerges from non-penetration and friction constraints
```

### 5. Fluid Simulation

**Pressure Projection**:

```
Incompressible flow with boundaries:
  ∇·u = 0        (divergence-free)
  p ≥ 0          (pressure non-negative)
  p·(u·n) = 0    (complementarity)

Discretized → LCP for pressure field
```

## Mathematical Properties

### Existence and Uniqueness

**Theorem**: An LCP has a unique solution if A is:

- **Strictly Copositive**: xᵀAx > 0 for all x ≠ 0, x ≥ 0
- **P-matrix**: All principal minors are positive
- **Symmetric Positive Definite (PD)**: xᵀAx > 0 for all x ≠ 0

**Common Cases**:

- Contact mechanics: A is often symmetric PSD (positive semi-definite)
- Joint constraints may make A singular (PSD but not PD)

### Solvability Classes

| Matrix Class  | Solution Exists? | Solution Unique? | Solvable By         |
| ------------- | ---------------- | ---------------- | ------------------- |
| Symmetric PD  | Always           | Yes              | All methods         |
| Symmetric PSD | Sometimes        | Sometimes        | Pivoting, iterative |
| P-matrix      | Always           | Yes              | Pivoting            |
| Copositive    | Sometimes        | Sometimes        | Depends on b        |
| General       | Sometimes        | Sometimes        | Trial and error     |

### Degeneracy

**Strict Complementarity**: For all i, either xᵢ > 0 OR wᵢ > 0 (but not both zero)

**Degeneracy**: When xᵢ = wᵢ = 0 for some i

- Makes active set identification difficult
- Can slow convergence of iterative methods
- Pivoting methods may cycle

## Complexity

### Computational Complexity

- **General LCP**: NP-complete (worst case)
- **Special cases** (symmetric PD): Polynomial time

### Practical Complexity

For physics simulation with n contact points:

- **Problem size**: n to 6n variables (normal + friction + bounds)
- **Matrix structure**: Often sparse (O(kn) non-zeros, k small)
- **Time discretization**: Solve LCP every time step

## Standard Forms

### Cottle-Dantzig Form

```
w = Mz + q
w ≥ 0, z ≥ 0
wᵀz = 0
```

Used in theoretical analysis and pivoting methods.

### Physics Form

```
(Ax - b) ≥ 0
x ≥ 0
xᵀ(Ax - b) = 0
```

Direct from physics constraints.

### Optimization Form (A symmetric PD)

```
minimize: f(x) = ½xᵀAx + xᵀb
subject to: x ≥ 0

KKT conditions → LCP
```

## Relationship to Other Problems

### Linear Programming (LP)

```
LP ⊂ LCP

LP: minimize cᵀx subject to Ax ≤ b, x ≥ 0
Can be reformulated as LCP
```

### Quadratic Programming (QP)

```
QP (with box constraints) ⊂ LCP

When A is symmetric:
  LCP ↔ QP with non-negativity
```

### Optimization

```
KKT conditions of constrained optimization
often lead to LCP/BLCP
```

### Game Theory

```
Nash equilibria in bimatrix games
can be found by solving LCP
```

## Why LCPs Matter for DART

### Core Use Cases

1. **Contact Resolution**
   - Every contact point → LCP variables
   - Friction cones → BLCP bounds
   - Non-penetration → Complementarity

2. **Constraint Satisfaction**
   - Joint limits → BLCP
   - Motor constraints → MLCP
   - Closed kinematic chains → MLCP

3. **Interactive Simulation**
   - Real-time requires fast LCP solvers
   - Trade-off: speed vs accuracy
   - Iterative methods essential (O(n) per iteration)

4. **High-Fidelity Simulation**
   - Accurate contact forces need tight tolerances
   - Newton methods or pivoting
   - O(n³) acceptable for off-line

### Solver Requirements

| Requirement                   | Method Choice            |
| ----------------------------- | ------------------------ |
| Real-time (30+ FPS)           | PGS, PSOR, BGS           |
| High accuracy                 | Newton, Pivoting         |
| Large scenes (>1000 contacts) | NNCG, PGS                |
| Ill-conditioned               | Pivoting, Interior Point |
| Parallel hardware             | Jacobi, Red-Black GS     |

## Key Challenges

### Numerical Challenges

1. **Ill-conditioning**: Large mass ratios, thin objects
2. **Degeneracy**: Multiple contacts at same point
3. **Sparsity**: Must exploit for large problems
4. **Warm-starting**: Critical for time-stepping

### Modeling Challenges

1. **Friction cone discretization**: Pyramid vs ellipse
2. **Time integration**: Implicit vs explicit
3. **Constraint stabilization**: Baumgarte, post-stabilization
4. **Regularization**: Trade-off with physical accuracy

### Implementation Challenges

1. **Matrix assembly**: Efficient Jacobian computation
2. **Solver selection**: Problem-dependent performance
3. **Parameter tuning**: Tolerances, iterations, relaxation
4. **Robustness**: Handling edge cases, degeneracies

## Further Reading

### Theory

- **Cottle, Pang, Stone** (1992): "The Linear Complementarity Problem" - Comprehensive reference
- **Murty** (1988): "Linear Complementarity, Linear and Nonlinear Programming" - Theoretical foundations

### Physics-Based Animation

- **Erleben et al.** (2017): "Numerical Methods for Linear Complementarity Problems in Physics-Based Animation" - Direct application to simulation
- **Baraff** (1994): "Fast Contact Force Computation for Nonpenetrating Rigid Bodies" - Foundational paper

### Optimization

- **Nocedal & Wright** (1999): "Numerical Optimization" - QP and NCP connections
- **Ferris & Kanzow** (2002): "Engineering and Economic Applications of Complementarity Problems" - Applied perspective

---

**Next**: [Overview of LCP Solvers →](02_overview.md)
