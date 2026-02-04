# Linear Complementarity Problem (LCP)

> **Attribution**: This content is derived from "Contact Handling for Articulated
> Rigid Bodies Using LCP" by Jie Tan, Kristin Siu, and C. Karen Liu.
> The original PDF is preserved at [`docs/lcp.pdf`](../../lcp.pdf).

**Navigation**: [Index](../README.md) | [Overview →](02_overview.md)

## Definition

The **Linear Complementarity Problem (LCP)** is defined as:

Given $A \in \mathbb{R}^{n \times n}$ and $b \in \mathbb{R}^n$, find $x \in \mathbb{R}^n$ such that:

$$w = Ax - b$$
$$w \geq 0$$
$$x \geq 0$$
$$x^T w = 0 \quad \text{(complementarity condition)}$$

> Note: Many references write the standard LCP as $w = Ax + q$. DART uses the
> equivalent convention $w = Ax - b$ (i.e., $b = -q$), matching the ODE-style
> $Ax = b + w$ form used by the constraint solver.

The complementarity condition $x^T w = 0$ means that for each index $i$:

$$\text{Either } x_i = 0 \text{ OR } w_i = 0 \text{ (or both)}$$

This can also be written component-wise as:

For each $i \in \{1, \ldots, n\}$:
$$w_i = (Ax - b)_i \geq 0$$
$$x_i \geq 0$$
$$x_i \cdot w_i = 0$$

## Equivalent Formulations

### Minimum Map Reformulation

$$x = \min(x, Ax - b)$$

where the minimum is taken component-wise.

### Nonlinear Complementarity Problem (NCP)

$$F(x) = 0 \quad \text{where } F(x) = \min(x, Ax - b)$$

### Variational Inequality (VI)

Find $x \geq 0$ such that:
$$(Ax - b)^T (y - x) \geq 0 \quad \text{for all } y \geq 0$$

### Quadratic Programming (QP) - When A is Symmetric PD

$$\min_x \frac{1}{2} x^T A x - x^T b \quad \text{subject to } x \geq 0$$

## Problem Variants

### Standard LCP

The basic form as defined above with unbounded variables constrained to be non-negative.

### Boxed LCP (BLCP)

LCP with box constraints on variables:

Find $x$ such that:
$$l \leq x \leq u$$
$$w = Ax - b$$

For each $i$:

- If $x_i = l_i$, then $w_i \geq 0$
- If $x_i = u_i$, then $w_i \leq 0$
- If $l_i < x_i < u_i$, then $w_i = 0$

Typical bounds:

- Lower: $l \leq 0$ (often $-\infty$ or $-\mu N$ for friction)
- Upper: $u \geq 0$ (often $+\infty$ or $+\mu N$ for friction)

### Mixed LCP (MLCP)

Combination of equality constraints and complementarity:

Find $x$, $z$ such that:
$$Ax + Bz + c = 0 \quad \text{(equality constraints)}$$
$$w = Cx + Dz + e$$
$$w \geq 0, \quad z \geq 0$$
$$z^T w = 0 \quad \text{(complementarity)}$$

### LCP with Friction (FLCP)

Special BLCP where bounds depend on other variables:

For friction at contact $i$:
$$-\mu N_i \leq F_{t,i} \leq \mu N_i$$

where:

- $N_i$ is normal force
- $F_{t,i}$ is tangential friction force
- $\mu$ is friction coefficient

In DART, this coupling is represented using the `findex` array:

- For a tangential variable $i$ coupled to normal $j$:
  - Set `findex[i] = j`
  - Store the coefficient in the box bounds: `lo[i] = -μ`, `hi[i] = +μ`
  - The effective bounds are interpreted as $|x_i| \leq \mu \cdot x_j$

## Applications in Physics-Based Simulation

### 1. Contact Mechanics

**Unilateral Contact Constraints**:

For each contact point $i$:
$$d_i \geq 0 \quad \text{(non-penetration)}$$
$$N_i \geq 0 \quad \text{(no adhesion)}$$
$$N_i \cdot d_i = 0 \quad \text{(complementarity: either separated or in contact)}$$

where:

- $d_i$ = gap distance (signed distance function)
- $N_i$ = normal contact force

**LCP Formulation**:
After time discretization and linearization:

$$x = [N_1, N_2, \ldots, N_n]^T \quad \text{(normal forces)}$$

$Ax - b$ represents the constraint-space residual. Complementarity ensures forces are zero when separated.

### 2. Friction Modeling

**Coulomb Friction as BLCP**:

For contact $i$ with normal force $N_i$:
$$\text{Friction cone: } \|F_{t,i}\| \leq \mu N_i$$

Discretized (pyramid approximation):
$$-\mu N_i \leq F_{t,i}^x \leq \mu N_i$$
$$-\mu N_i \leq F_{t,i}^y \leq \mu N_i$$

BLCP formulation:
$$l_i = -\mu N_i, \quad u_i = +\mu N_i$$

### 3. Joint Limits

**Joint Constraints**:

For revolute joint with limits $\theta_{\min} \leq \theta \leq \theta_{\max}$:

BLCP formulation:
$$l = \theta_{\min}, \quad u = \theta_{\max}$$

Forces active only at limits.

### 4. Rigid Body Dynamics

**Time-Stepping Scheme**:

Velocity-level formulation:
$$M(v_+ - v_-) = h \cdot f_{\text{ext}} + J^T \lambda$$

where:

- $M$ = mass matrix
- $v_-$, $v_+$ = velocities before/after contact
- $h$ = time step
- $J$ = contact Jacobian
- $\lambda$ = contact impulses (LCP variable)

LCP emerges from non-penetration and friction constraints.

### 5. Fluid Simulation

**Pressure Projection**:

Incompressible flow with boundaries:
$$\nabla \cdot u = 0 \quad \text{(divergence-free)}$$
$$p \geq 0 \quad \text{(pressure non-negative)}$$
$$p \cdot (u \cdot n) = 0 \quad \text{(complementarity)}$$

Discretized → LCP for pressure field.

## Mathematical Properties

### Existence and Uniqueness

**Theorem**: An LCP has a unique solution if $A$ is:

- **Strictly Copositive**: $x^T A x > 0$ for all $x \neq 0$, $x \geq 0$
- **P-matrix**: All principal minors are positive
- **Symmetric Positive Definite (PD)**: $x^T A x > 0$ for all $x \neq 0$

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

**Strict Complementarity**: For all $i$, either $x_i > 0$ OR $w_i > 0$ (but not both zero)

**Degeneracy**: When $x_i = w_i = 0$ for some $i$

- Makes active set identification difficult
- Can slow convergence of iterative methods
- Pivoting methods may cycle

## Complexity

### Computational Complexity

- **General LCP**: NP-complete (worst case)
- **Special cases** (symmetric PD): Polynomial time

### Practical Complexity

For physics simulation with $n$ contact points:

- **Problem size**: $n$ to $6n$ variables (normal + friction + bounds)
- **Matrix structure**: Often sparse ($O(kn)$ non-zeros, $k$ small)
- **Time discretization**: Solve LCP every time step

## Standard Forms

### Cottle-Dantzig Form

$$w = Mz + q$$
$$w \geq 0, \quad z \geq 0$$
$$w^T z = 0$$

Used in theoretical analysis and pivoting methods.

### Physics Form

$$(Ax - b) \geq 0$$
$$x \geq 0$$
$$x^T(Ax - b) = 0$$

Direct from physics constraints.

### Optimization Form (A symmetric PD)

$$\min_x f(x) = \frac{1}{2} x^T A x + x^T b \quad \text{subject to } x \geq 0$$

KKT conditions → LCP

## Relationship to Other Problems

### Linear Programming (LP)

$$\text{LP} \subset \text{LCP}$$

LP: $\min c^T x$ subject to $Ax \leq b$, $x \geq 0$ can be reformulated as LCP.

### Quadratic Programming (QP)

$$\text{QP (with box constraints)} \subset \text{LCP}$$

When $A$ is symmetric: LCP ↔ QP with non-negativity.

### Optimization

KKT conditions of constrained optimization often lead to LCP/BLCP.

### Game Theory

Nash equilibria in bimatrix games can be found by solving LCP.

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
   - Iterative methods essential ($O(n)$ per iteration)

4. **High-Fidelity Simulation**
   - Accurate contact forces need tight tolerances
   - Newton methods or pivoting
   - $O(n^3)$ acceptable for off-line

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
