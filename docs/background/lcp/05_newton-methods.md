# Newton Methods for LCP

**Navigation**: [← Projection Methods](04_projection-methods.md) | [Other Methods →](06_other-methods.md)

## Overview

Newton methods solve LCPs by reformulating them as nonsmooth root-finding problems and applying generalized Newton iteration. They offer superlinear to quadratic convergence but require globalization for robustness.

**All Newton methods are currently not implemented** but documented here for future reference.

## Common Framework

All Newton methods follow this pattern:

```
1. Reformulate LCP as H(x) = 0 (nonsmooth equation)
2. Compute generalized Newton direction: J*Δx = -H(x)
3. Line search for step length t
4. Update: x^{k+1} = max(0, x^k + t*Δx^k)
5. Repeat until convergence
```

## 1. Minimum Map Newton Method ❌ (Not Implemented)

### Reformulation

```
H(x) = min(x, Ax + b) = 0
```

### B-Derivative (Generalized Jacobian)

```
Partition indices into:
  A = {i | y_i = (Ax+b)_i < x_i}  (active set)
  F = {i | y_i = (Ax+b)_i >= x_i}  (free set)

Then:
  JH = [ A_AA ]  (block for active set)
       [ I_FF ]  (identity for free set)
```

### Newton Equation

```
Reduced system (only for active set):
  A_AA * Δx_A = -H_A

For free set:
  Δx_F = -H_F = -x_F  (directly set to zero)
```

### Algorithm Pseudocode

```
while not converged:
  # Compute residual
  y = Ax + b
  H = min(x, y)

  # Partition into active/free sets
  A = {i | y_i < x_i}
  F = {i | y_i >= x_i}

  # Solve reduced Newton equation
  Δx_A = solve(A_AA, -H_A)
  Δx_F = -x_F

  # Line search
  t = line_search(x, Δx, H)

  # Update with projection
  x = max(0, x + t*Δx)
```

### Properties

- **Time**: O(n³) with direct solver, O(n) with iterative
- **Storage**: O(n²) with direct solver, O(n) with iterative
- **Convergence**: Superlinear to quadratic
- **Matrix Requirements**: A_AA should be non-singular

### Advantages/Disadvantages

✅ Fast convergence (5-20 iterations)
✅ High accuracy achievable
✅ Well-studied mathematically
❌ Sensitive to starting point
❌ Needs globalization (line search)
❌ More complex than projection methods

## 2. Fischer-Burmeister Newton Method ❌ (Not Implemented)

### Reformulation

```
For each component:
  phi_FB(x_i, y_i) = sqrt(x_i² + y_i²) - x_i - y_i = 0
where y_i = (Ax + b)_i
```

### Generalized Jacobian

```
For (x_i, y_i) != (0, 0):
  p_i = x_i / sqrt(x_i² + y_i²) - 1
  q_i = y_i / sqrt(x_i² + y_i²) - 1

JF = diag(p) + diag(q)*A

For (x_i, y_i) = (0, 0):
  Can choose any (a_i, b_i) with ||(a_i, b_i)|| <= 1
```

### Strategies for Singularity

1. **Random**: Pick random (a, b) in unit circle
2. **Perturbation**: Use epsilon instead of exact 0
3. **Finite Difference**: Approximate with (F(x+h\*p) - F(x))/h
4. **Analytic**: Use (a, b) = (0, 0) or (-1/sqrt(2), -1/sqrt(2))

### Algorithm Pseudocode

```
while not converged:
  # Compute Fischer-Burmeister function
  y = Ax + b
  for i = 1 to n:
    F_i = sqrt(x_i² + y_i²) - x_i - y_i

  # Build generalized Jacobian
  JF = compute_jacobian(x, y)

  # Solve Newton equation (with iterative solver)
  Δx = solve(JF, -F)  # Use GMRES or PCG

  # Line search
  t = line_search(x, Δx, F)

  # Update with projection
  x = max(0, x + t*Δx)
```

### Properties

- **Time**: O(n³) or O(n) per iteration
- **Storage**: O(n²) or O(n)
- **Convergence**: Superlinear to quadratic
- **Smoothness**: Nonsmooth only at (0,0)

### Advantages/Disadvantages

✅ Smoother than minimum map (only singular at origin)
✅ Well-studied in optimization literature
✅ PATH solver uses this reformulation
❌ Singularity handling needed
❌ More complex than minimum map

## 3. Penalized Fischer-Burmeister Newton ❌ (Not Implemented)

### Reformulation

```
phi_lambda(x, y) = lambda * phi_FB(x, y) - (1-lambda) * max(x,0) * max(y,0)
```

where 0 < lambda <= 1

### Effect of lambda Parameter

- **lambda = 1**: Standard Fischer-Burmeister
- **lambda < 1**: More penalty on complementarity
- **lambda ~ 0.5**: Good balance (typical)

### Generalized Jacobian

```
Combines Fischer-Burmeister Jacobian with penalty term:

For x_i > 0, y_i > 0:
  p_i = lambda * (x_i/||z_i|| - 1) - (1-lambda) * y_i
  q_i = lambda * (y_i/||z_i|| - 1) - (1-lambda) * x_i
  where z_i = sqrt(x_i² + y_i²)

For x_i = 0 or y_i = 0 (but not both zero):
  Special formulas apply

For x_i = y_i = 0:
  Use choices similar to standard FB
```

### Properties

- **Time**: O(n³) or O(n) per iteration
- **Storage**: O(n²) or O(n)
- **Convergence**: Similar to Fischer-Burmeister
- **Tuning**: lambda parameter affects convergence

### Advantages/Disadvantages

✅ Additional tuning parameter
✅ Can improve convergence for some problems
❌ More complex Jacobian
❌ Requires tuning lambda

## Supporting Methods

### Line Search (Projected Armijo Back-Tracking)

Essential for globalizing Newton methods.

```
function line_search(x, Δx, H):
  # Parameters
  alpha = 1e-4  # Sufficient decrease
  beta = 0.5    # Step reduction
  t = 1.0       # Initial step

  # Current merit function
  phi_0 = 0.5 * ||H(x)||²
  phi_prime_0 = H(x)^T * JH * Δx  # Directional derivative

  # Back-tracking
  while t > epsilon:
    x_new = max(0, x + t*Δx)
    phi_t = 0.5 * ||H(x_new)||²

    # Armijo condition
    if phi_t <= phi_0 + alpha * t * phi_prime_0:
      return t

    t = beta * t

  return t
```

### Nonsmooth Gradient Descent (Warm Start)

Used to compute good starting iterate for Newton methods.

```
function gradient_descent(x, max_iter):
  for iter = 1 to max_iter:
    # Compute gradient of merit function
    H = reformulation(x)
    grad = JH^T * H

    # Simple line search
    t = line_search(x, -grad, H)

    # Update
    x = max(0, x - t * grad)

    # Early termination if good enough
    if ||H|| < threshold:
      break

  return x
```

### Subsystem Solvers

Newton equation can be solved with iterative methods:

**For Symmetric Systems (Minimum Map):**

```
Use PCG (Preconditioned Conjugate Gradient):
  - Requires A_AA to be symmetric PD
  - Incomplete Cholesky preconditioner
  - Tolerance: ||r|| < gamma * ||H||
```

**For General Systems (Fischer-Burmeister):**

```
Use GMRES (Generalized Minimal Residual):
  - Works for any non-singular matrix
  - Higher storage than PCG
  - Tolerance: ||r|| < gamma * ||H||
```

### Merit Functions

All Newton methods use natural merit function:

```
phi(x) = 0.5 * ||H(x)||²
```

Properties:

- phi(x) >= 0
- phi(x) = 0 iff x is solution
- Descent direction: ∇phi = JH^T \* H

## Comparison Table

| Method             | Smoothness           | Jacobian Complexity | Best For     |
| ------------------ | -------------------- | ------------------- | ------------ |
| Minimum Map        | Nonsmooth everywhere | Simple (blocks)     | General use  |
| Fischer-Burmeister | Smooth except origin | Medium              | Well-studied |
| Penalized FB       | Less smooth          | Complex             | Tunable      |

## Implementation Strategy

### Phase 1: Core Newton Framework

1. Merit function computation
2. Projected line search
3. Termination criteria

### Phase 2: Minimum Map Newton

4. Active/free set partitioning
5. Reduced Newton equation solver
6. GMRES subsystem solver

### Phase 3: Supporting Methods

7. Nonsmooth gradient descent
8. Warm start from PGS

### Phase 4: Fischer-Burmeister (Optional)

9. FB reformulation
10. Generalized Jacobian
11. Singularity handling

## When to Use Newton Methods

### Advantages

✅ Fast convergence (5-20 iterations)
✅ High accuracy (1e-8 to 1e-12)
✅ Superlinear/quadratic convergence rate
✅ Matrix-free with iterative solvers

### Disadvantages

❌ Complex implementation
❌ Sensitive to starting point
❌ Requires line search for robustness
❌ Not guaranteed to converge from arbitrary start

### Recommended For

- Off-line simulations
- High accuracy requirements
- When computational budget allows
- Validation and benchmarking

### Not Recommended For

- Real-time interactive simulation (use PGS instead)
- When robustness is critical (use pivoting instead)
- Very large problems without good warm start

## Best Practices

### Starting Iterates

- Use x = 0 as default
- Warm start with PGS (5-10 iterations)
- Warm start with gradient descent
- Use previous time-step solution

### Convergence Monitoring

- Absolute: ||H|| < 1e-8
- Relative: ||H^{k+1}|| / ||H^k|| < 1e-6
- Maximum iterations: 20-50
- Detect stagnation and divergence

### Subsystem Solver Tolerance

- Don't need exact Newton direction
- Use ||r|| < 0.1 \* ||H|| for descent
- Tighten tolerance as H → 0

### Fallback Strategy

When Newton fails:

1. Try gradient descent for few iterations
2. Restart Newton
3. Fall back to PGS
4. Use pivoting method

## References

### Nonsmooth Analysis and Generalized Derivatives

1. **Clarke, F. H.** (1990). _Optimization and nonsmooth analysis_. SIAM.
   - Chapter 2: Generalized gradients and subdifferentials
   - Chapter 6: B-differentiability
   - Theoretical foundation for nonsmooth Newton methods

2. **Qi, L., & Sun, J.** (1993). "A nonsmooth version of Newton's method". _Mathematical programming_, 58(1), 353-367.
   - Generalized Newton method for nonsmooth equations
   - B-differentiability and semismooth functions
   - Superlinear convergence theory

3. **Pang, J. S., & Qi, L.** (1993). "Nonsmooth equations: motivation and algorithms". _SIAM Journal on Optimization_, 3(3), 443-465.
   - Reformulation of complementarity as nonsmooth equations
   - Newton methods for NCP
   - Global convergence with line search

### Minimum Map Newton Method

4. **Fischer, A.** (1992). "A special Newton-type optimization method". _Optimization_, 24(3-4), 269-284.
   - Minimum map reformulation H(x) = min(x, F(x))
   - B-derivative computation
   - Superlinear convergence

5. **Sun, D., & Qi, L.** (1999). "Solving variational inequality problems via smoothing-nonsmooth reformulations". _Journal of Computational and Applied Mathematics_, 129(1-2), 37-62.
   - Minimum map approach for VI and LCP
   - Active set identification
   - Reduced Newton system

6. **Facchinei, F., & Pang, J. S.** (2003). _Finite-dimensional variational inequalities and complementarity problems_. Springer Science & Business Media.
   - Comprehensive treatment in Chapters 7-8
   - Minimum map and Fischer-Burmeister methods
   - Theory and algorithms

### Fischer-Burmeister Newton Method

7. **Fischer, A.** (1992). "A special newton-type optimization method". _Optimization_, 24(3-4), 269-284.
   - Fischer-Burmeister function φ_FB(a,b) = √(a²+b²) - a - b
   - Nonsmooth Newton method
   - Superlinear convergence

8. **Burmeister, W.** (1990). "Monotone Komplementaritätsprobleme und Variationsungleichungen". PhD thesis, University of Hamburg.
   - Original Fischer-Burmeister function
   - German dissertation

9. **De Luca, T., Facchinei, F., & Kanzow, C.** (1996). "A semismooth equation approach to the solution of nonlinear complementarity problems". _Mathematical programming_, 75(3), 407-439.
   - Semismooth analysis of Fischer-Burmeister
   - Generalized Jacobian computation
   - Handling singularity at origin

10. **Kanzow, C.** (1996). "Some noninterior continuation methods for linear complementarity problems". _SIAM Journal on Matrix Analysis and Applications_, 17(4), 851-868.
    - Fischer-Burmeister path following
    - Regularization strategies
    - Global convergence

### Penalized Fischer-Burmeister

11. **Chen, B., & Harker, P. T.** (1997). "Smooth approximations to nonlinear complementarity problems". _SIAM Journal on Optimization_, 7(2), 403-420.
    - Penalized versions of NCP functions
    - Smoothing parameter effects
    - Convergence analysis

12. **Chen, C., & Mangasarian, O. L.** (1996). "A class of smoothing functions for nonlinear and mixed complementarity problems". _Computational Optimization and Applications_, 5(2), 97-138.
    - General smoothing function framework
    - Penalized Fischer-Burmeister as special case
    - Numerical results

### PATH Solver (Fischer-Burmeister Implementation)

13. **Ferris, M. C., & Munson, T. S.** (2000). "Complementarity problems in GAMS and the PATH solver". _Journal of Economic Dynamics and Control_, 24(2), 165-188.
    - PATH solver implementation
    - Fischer-Burmeister with non-monotone line search
    - GAMS/Matlab/Python interfaces

14. **Dirkse, S. P., & Ferris, M. C.** (1995). "The PATH solver: a non-monotone stabilization scheme for mixed complementarity problems". _Optimization methods and software_, 5(2), 123-156.
    - Non-monotone line search
    - Crash techniques for initial point
    - Robust implementation details

15. **Ferris, M. C., & Kanzow, C.** (2002). "Engineering and economic applications of complementarity problems". _SIAM review_, 44(1), 1-37.
    - Applications survey
    - PATH solver case studies
    - Comparison with other methods

### Line Search and Globalization

16. **Armijo, L.** (1966). "Minimization of functions having Lipschitz continuous first partial derivatives". _Pacific Journal of mathematics_, 16(1), 1-3.
    - Original Armijo condition for sufficient decrease
    - Foundation for backtracking line search

17. **Nocedal, J., & Wright, S. J.** (1999). _Numerical optimization_. Springer.
    - Chapter 3: Line search methods
    - Chapter 11: Nonlinear least squares (relevant for merit functions)
    - Practical algorithms

18. **Dennis, J. E., & Schnabel, R. B.** (1996). _Numerical methods for unconstrained optimization and nonlinear equations_. SIAM.
    - Chapter 6: Line search
    - Chapter 8: Trust regions
    - Globalization strategies

### Merit Functions

19. **Fukushima, M.** (1992). "Equivalent differentiable optimization problems and descent methods for asymmetric variational inequality problems". _Mathematical programming_, 53(1), 99-110.
    - Natural merit function φ(x) = 0.5||F(x)||²
    - Descent properties
    - Stationary point analysis

20. **Geiger, C., & Kanzow, C.** (2002). "On the resolution of monotone complementarity problems". _Computational optimization and applications_, 5(2), 155-173.
    - Merit function properties
    - Exact penalty interpretation
    - Global convergence

### Subsystem Solvers

21. **Saad, Y., & Schultz, M. H.** (1986). "GMRES: A generalized minimal residual algorithm for solving nonsymmetric linear systems". _SIAM Journal on scientific and statistical computing_, 7(3), 856-869.
    - GMRES for general nonsymmetric systems
    - Used for Fischer-Burmeister Jacobian

22. **Hestenes, M. R., & Stiefel, E.** (1952). "Methods of conjugate gradients for solving linear systems". _Journal of research of the National Bureau of Standards_, 49(6), 409-436.
    - Original conjugate gradient method
    - Preconditioned CG for symmetric systems

23. **Benzi, M.** (2002). "Preconditioning techniques for large linear systems: a survey". _Journal of computational Physics_, 182(2), 418-477.
    - Incomplete Cholesky preconditioning
    - Practical preconditioner choices

### Gradient Descent for Warm Starting

24. **Bertsekas, D. P.** (1999). _Nonlinear programming_ (2nd ed.). Athena scientific.
    - Chapter 1.3: Gradient methods
    - Convergence theory
    - Step size selection

25. **Polyak, B. T.** (1969). "The conjugate gradient method in extremal problems". _USSR Computational Mathematics and Mathematical Physics_, 9(4), 94-112.
    - Heavy-ball method
    - Momentum for acceleration

### Applications to Physics Simulation

26. **Silcowitz-Hansen, M., Erleben, K., & Niebe, S.** (2010). "A nonsmooth Newton method with applications to computer animation". In _MATHMOD 2009-6th Vienna International Conference on Mathematical Modelling_.
    - Newton methods for contact problems
    - Minimum map approach
    - Warm-starting strategies

27. **Kaufman, D. M., Tamstorf, R., Smith, B., Aubry, J. M., & Grinspun, E.** (2014). "Adaptive nonlinearity for collisions in complex rod assemblies". _ACM Transactions on Graphics (TOG)_, 33(4), 1-12.
    - Newton methods for complex contact
    - Active set prediction
    - Adaptive strategies

28. **Otaduy, M. A., Tamstorf, R., Steinemann, D., & Gross, M.** (2009). "Implicit contact handling for deformable objects". In _Computer Graphics Forum_, 28(2), 559-568.
    - Newton methods for deformable contact
    - Comparison with projection methods

### Convergence Theory

29. **Ulbrich, M.** (2011). _Semismooth Newton methods for variational inequalities and constrained optimization problems in function spaces_. SIAM.
    - Advanced theory for Newton methods
    - Mesh-independence for PDEs
    - Superlinear convergence proofs

30. **Facchinei, F., & Kanzow, C.** (1997). "Beyond monotonicity in regularization methods for nonlinear complementarity problems". _SIAM Journal on Control and Optimization_, 37(4), 1150-1161.
    - Convergence without monotonicity
    - Regularization techniques
    - Robust algorithms

---

**Navigation**: [← Projection Methods](04_projection-methods.md) | [Other Methods →](06_other-methods.md)
