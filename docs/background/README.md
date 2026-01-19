# DART Background Theory

Theoretical background materials derived from academic publications by DART's original authors.

> **Note**: These documents explain the mathematical foundations. For code-level exploration,
> see [`docs/onboarding/`](../onboarding/README.md).

## Attribution

The materials in this section are derived from:

| Document                    | Authors                            | Original                               | Status                |
| --------------------------- | ---------------------------------- | -------------------------------------- | --------------------- |
| Multibody Dynamics Tutorial | C. Karen Liu, Sumit Jain           | [`docs/dynamics.pdf`](../dynamics.pdf) | Migration in progress |
| LCP Contact Handling        | Jie Tan, Kristin Siu, C. Karen Liu | [`docs/lcp.pdf`](../lcp.pdf)           | ✅ Migrated           |

These documents were reformatted from PDF to Markdown for maintainability while preserving
the original authors' work. The original PDFs remain in the repository as authoritative references.

For the full attribution template, see [ATTRIBUTION.md](ATTRIBUTION.md).

## Contents

### [Dynamics](dynamics/)

Lagrangian dynamics for articulated rigid body systems:

1. [Introduction](dynamics/01_introduction.md) — Prerequisites and scope
2. [Lagrangian Dynamics](dynamics/02_lagrangian-dynamics.md) — D'Alembert's principle, virtual work
3. [Newton-Euler Review](dynamics/03_newton-euler-review.md) — Single rigid body dynamics
4. [Rigid Body Lagrange](dynamics/04_rigid-body-lagrange.md) — Lagrange's equations for rigid bodies
5. [Articulated Dynamics](dynamics/05_articulated-dynamics.md) — Equations of motion: `M(q)q̈ + C(q,q̇) = Q`
6. [Coordinate Conversion](dynamics/06_coordinate-conversion.md) — Cartesian ↔ generalized coordinates
7. [Recursive Inverse Dynamics](dynamics/07_recursive-inverse-dynamics.md) — Efficient O(n) algorithms

**Notation Glossary**: [dynamics/notation-glossary.md](dynamics/notation-glossary.md)

### [LCP Solvers](lcp/)

Contact handling via Linear Complementarity Problems:

1. [Problem Statement](lcp/01_problem-statement.md) — LCP definition, DART convention (`w = Ax - b`)
2. [Overview](lcp/02_overview.md) — Solver taxonomy and comparison
3. [Pivoting Methods](lcp/03_pivoting-methods.md) — Dantzig, Lemke algorithms
4. [Projection Methods](lcp/04_projection-methods.md) — PGS, Jacobi, iterative solvers
5. [Newton Methods](lcp/05_newton-methods.md) — Smooth reformulations
6. [Other Methods](lcp/06_other-methods.md) — Interior point, QP-based
7. [Selection Guide](lcp/07_selection-guide.md) — Which solver to use when

## Related Documentation

| Topic                     | Location                                                                               | Description      |
| ------------------------- | -------------------------------------------------------------------------------------- | ---------------- |
| Dynamics code exploration | [`docs/onboarding/dynamics.md`](../onboarding/dynamics.md)                             | API walkthrough  |
| Constraint system         | [`docs/onboarding/constraints.md`](../onboarding/constraints.md)                       | Solver internals |
| Control theory            | [`docs/readthedocs/topics/control-theory.md`](../readthedocs/topics/control-theory.md) | Notation mapping |

## For AI Agents

See [AGENTS.md](AGENTS.md) for guidelines on working with these documents.
