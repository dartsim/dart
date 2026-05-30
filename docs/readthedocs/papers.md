# Research Papers And References

## Status

Living catalog and single source of truth. This page is the consolidated,
managed list of research papers, textbooks, model-format standards, and
comparative engine implementations that DART cites, implements, extends,
reproduces, or uses as a baseline.

This page is intentionally published in the website documentation rather than
only in internal developer docs. The old
`docs/design/simulation_experimental_references.md` path is kept only as a
compatibility pointer; do not add entries there.

The catalog currently focuses on the DART **experimental simulation world**
(`dart/simulation/experimental/**` and `dartpy.simulation_experimental`) and
its algorithms. The schema is general, so it can extend to the rest of DART
without changing its shape.

It is a companion to the API design docs:

- [`simulation_experimental_cpp_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_cpp_api.md)
- [`simulation_experimental_python_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_python_api.md)

## Why This Exists

DART's [north star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md)
is a research-focused engine where "the
easiest place to reproduce and evaluate a new algorithm should be inside DART,"
and it names a tracked gap: _algorithm-family contracts and baseline comparisons
are not yet tracked as first-class research surfaces._ This catalog is that
surface for the experimental world: it records what the new API and algorithms
reference, implement, plan, evaluate, or reject, with an explicit verdict and
status for each — so a researcher or agent can see at a glance where a method
stands and where it is wired into the code.

## How To Use

- Scan the summary tables for status/priority/verdict at a glance.
- Read the per-entry details for the citation, where it is used in DART, and the
  rationale behind the verdict.
- Use the stable entry IDs in plans, PRs, tests, benchmarks, and release notes.
- To add or change an entry, follow the `dart-references` skill
  (`.claude/skills/dart-references/`); keep entries grounded in the design docs,
  code, or tests.

## Structure And Ordering

The catalog is designed to scale without turning into a flat bibliography:

- Keep one top-level category per reference family: DART citation, textbooks and
  foundational references, algorithms and methods, model formats and standards,
  comparative implementations, and terminology grounding.
- Each category has a summary table followed by detailed entries. Add both when
  adding a new reference.
- Keep entry IDs stable. Use `author-year` for papers and textbooks, a standard
  name for formats, and a project name for software baselines.
- Within each category, keep adopted/high-priority implementation references
  before lower-priority background references, then preserve existing order
  unless a status change makes reordering useful.
- Put full BibTeX blocks on direct citation targets and active implementation
  credits. For lower-level background references, a precise citation plus status
  fields is enough until a feature needs a formal citation block.

### Property legend

Each entry carries these properties:

| Property       | Values                                                                                                     | Meaning                                                                                       |
| -------------- | ---------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------- |
| **Type**       | `textbook`, `paper`, `standard`, `engine`                                                                  | Kind of reference (`engine` = comparative software implementation).                           |
| **Topic**      | e.g. `dynamics`, `kinematics`, `contact`, `integration`, `collision`, `terminology`, `model-format`, `api` | Primary subject area.                                                                         |
| **Status**     | `referenced`, `planned`, `in-progress`, `implemented`, `deferred`, `rejected`                              | The experimental world's relationship to the reference.                                       |
| **Priority**   | `high`, `medium`, `low`, `—`                                                                               | Relative importance for acting on or evaluating the reference; `—` when purely informational. |
| **Verdict**    | `adopt`, `baseline`, `reference`, `evaluate`, `reject`                                                     | The project's decision: build on it, compare against it, cite it, weigh it, or pass.          |
| **Where used** | links / paths                                                                                              | Design doc, code, or test that uses or will use it.                                           |

Status values are written from the experimental world's perspective. A method
already shipping in _classic_ DART but not yet in the experimental world is
`planned` here, with the classic location noted.

## DART Core Citation

If you use DART in an academic publication, please cite:

Jeongseok Lee, Michael X. Grey, Sehoon Ha, Tobias Kunz, Sumit Jain, Yuting Ye,
Siddhartha S. Srinivasa, Mike Stilman, and C. Karen Liu. "DART: Dynamic
Animation and Robotics Toolkit." _Journal of Open Source Software_, 3(22),
500, 2018. DOI: [10.21105/joss.00500](https://doi.org/10.21105/joss.00500).

```bib
@article{Lee2018,
  doi = {10.21105/joss.00500},
  url = {https://doi.org/10.21105/joss.00500},
  year = {2018},
  month = {Feb},
  publisher = {The Open Journal},
  volume = {3},
  number = {22},
  pages = {500},
  author = {Jeongseok Lee and Michael X. Grey and Sehoon Ha and Tobias Kunz and Sumit Jain and Yuting Ye and Siddhartha S. Srinivasa and Mike Stilman and C. Karen Liu},
  title = {{DART}: Dynamic Animation and Robotics Toolkit},
  journal = {The Journal of Open Source Software}
}
```

## Textbooks & Foundational References

| ID                  | Reference                                                                                  | Topic       | Status      | Priority | Verdict   |
| ------------------- | ------------------------------------------------------------------------------------------ | ----------- | ----------- | -------- | --------- |
| `featherstone-2008` | Featherstone, _Rigid Body Dynamics Algorithms_ (2008)                                      | dynamics    | planned     | high     | adopt     |
| `lynch-park-2017`   | Lynch & Park, _Modern Robotics_ (2017)                                                     | kinematics  | implemented | —        | adopt     |
| `murray-1994`       | Murray, Li & Sastry, _A Mathematical Introduction to Robotic Manipulation_ (1994)          | kinematics  | referenced  | —        | reference |
| `shabana-mbs`       | Shabana, _Dynamics of Multibody Systems_ (4th ed.)                                         | terminology | referenced  | —        | reference |
| `siciliano-2009`    | Siciliano, Sciavicco, Villani & Oriolo, _Robotics: Modelling, Planning and Control_ (2009) | dynamics    | referenced  | —        | reference |
| `hairer-2006`       | Hairer, Lubich & Wanner, _Geometric Numerical Integration_ (2006)                          | integration | referenced  | medium   | reference |

### `featherstone-2008`

Featherstone, R. _Rigid Body Dynamics Algorithms._ Springer, 2008.

- **Type:** textbook · **Topic:** dynamics · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Where used:** [cpp design](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_cpp_api.md)
  (articulated-body method); terminology grounding in the
  "Terminology Grounding" section below.
- **Notes:** Canonical reference for the kinematic-tree model and the O(n)
  Articulated-Body Algorithm (ABA). Source of the "kinematic tree" /
  "multibody system" vocabulary the experimental API adopts. ABA is the planned
  basis for the experimental forward-dynamics stage; experimental forward
  kinematics already follows its tree formulation.

### `lynch-park-2017`

Lynch, K. M., & Park, F. C. _Modern Robotics: Mechanics, Planning, and Control._
Cambridge University Press, 2017.

- **Type:** textbook · **Topic:** kinematics/terminology · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** experimental `JointType` taxonomy and frame/transform vocabulary; drove `Ball`→`Spherical` naming.
- **Notes:** Source for the joint taxonomy (revolute, prismatic, helical/screw,
  cylindrical, universal, spherical) and screw-theory framing. "Spherical" (not
  "ball") follows this text.

### `murray-1994`

Murray, R. M., Li, Z., & Sastry, S. S. _A Mathematical Introduction to Robotic
Manipulation._ CRC Press, 1994.

- **Type:** textbook · **Topic:** kinematics · **Status:** referenced · **Priority:** — · **Verdict:** reference
- **Notes:** Screw theory and product-of-exponentials background underpinning the
  kinematics vocabulary; cited as supporting reference, not a direct dependency.

### `shabana-mbs`

Shabana, A. A. _Dynamics of Multibody Systems._ Cambridge University Press.

- **Type:** textbook · **Topic:** terminology/dynamics · **Status:** referenced · **Priority:** — · **Verdict:** reference
- **Notes:** Establishes "multibody system" as the field-standard term; supports
  the `Multibody` naming decision over the simulator-specific "articulation".

### `siciliano-2009`

Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. _Robotics: Modelling,
Planning and Control._ Springer, 2009.

- **Type:** textbook · **Topic:** dynamics/kinematics · **Status:** referenced · **Priority:** — · **Verdict:** reference
- **Notes:** General modelling/control reference for terminology and method
  framing.

### `hairer-2006`

Hairer, E., Lubich, C., & Wanner, G. _Geometric Numerical Integration:
Structure-Preserving Algorithms for Ordinary Differential Equations._ 2nd ed.,
Springer, 2006.

- **Type:** textbook · **Topic:** integration · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Where used:** authority cited by
  [`simulation_variational_integrator.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_variational_integrator.md).
- **Notes:** Authority for symplectic/variational integration, backward-error
  analysis, and the inverse right-trivialized tangent `dexp⁻¹` as the Bernoulli
  series `Σ Bₖ/k! adᵘᵏ` (next nonzero term `+ad⁶/30240`; truncation error
  O(‖ad‖⁶), ~machine-ε at millisecond steps). Grounds the accuracy justification
  for DART's exact `SE3::LeftJacobianInverse`-based `dexp⁻¹`.

## Algorithms & Methods (Papers)

| ID                             | Reference                                                                                                                  | Topic       | Status      | Priority | Verdict   |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------- | ----------- | ----------- | -------- | --------- |
| `featherstone-1983`            | Featherstone, "Calculation of robot dynamics using articulated-body inertias" (1983)                                       | dynamics    | planned     | high     | adopt     |
| `liu-jain-mbs`                 | Liu & Jain, _A Quick Tutorial on Multibody Dynamics_                                                                       | dynamics    | implemented | —        | adopt     |
| `tan-lcp`                      | Tan, Siu & Liu, _Contact Handling for Articulated Rigid Bodies Using LCP_                                                  | contact     | implemented | —        | adopt     |
| `stewart-trinkle-1996`         | Stewart & Trinkle, "An implicit time-stepping scheme … Coulomb friction" (1996)                                            | contact     | referenced  | medium   | baseline  |
| `baraff-1996`                  | Baraff, "Linear-time dynamics using Lagrange multipliers" (1996)                                                           | dynamics    | referenced  | low      | reference |
| `macklin-xpbd-2016`            | Macklin et al., "XPBD: position-based simulation of compliant constrained dynamics" (2016)                                 | integration | referenced  | medium   | evaluate  |
| `gjk-1988`                     | Gilbert, Johnson & Keerthi, GJK distance algorithm (1988)                                                                  | collision   | implemented | —        | adopt     |
| `ipc-2020`                     | Li et al., "Incremental Potential Contact" (2020)                                                                          | contact     | in-progress | high     | adopt     |
| `lee-vi-2016`                  | Lee, Liu, Park & Srinivasa, "A Linear-Time Variational Integrator for Multibody Systems" (2016)                            | integration | planned     | high     | adopt     |
| `marsden-west-2001`            | Marsden & West, "Discrete mechanics and variational integrators" (2001)                                                    | integration | referenced  | medium   | reference |
| `chen-2024-vbd`                | Chen et al., "Vertex Block Descent" (SIGGRAPH 2024)                                                                        | integration | in-progress | high     | adopt     |
| `vbd-2024`                     | Chen et al., "Vertex Block Descent" (2024) — VI contact survey                                                             | contact     | referenced  | medium   | evaluate  |
| `avbd-2025`                    | Giles et al., "Augmented Vertex Block Descent" (2025)                                                                      | contact     | referenced  | medium   | evaluate  |
| `johnson-murphey-2009`         | Johnson & Murphey, "Scalable variational integrators for constrained mechanical systems in generalized coordinates" (2009) | integration | referenced  | high     | baseline  |
| `leyendecker-2008`             | Leyendecker, Marsden & Ortiz, "Variational integrators for constrained dynamical systems" (2008)                           | integration | referenced  | high     | evaluate  |
| `kobilarov-crane-desbrun-2009` | Kobilarov, Crane & Desbrun, "Lie group integrators for animation and control of vehicles" (2009)                           | integration | referenced  | medium   | reference |

### `featherstone-1983`

Featherstone, R. "The calculation of robot dynamics using articulated-body
inertias." _International Journal of Robotics Research_, 2(1), 1983.

- **Type:** paper · **Topic:** dynamics · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Notes:** Original ABA paper; the planned forward-dynamics method for the
  experimental dynamics stage. See the `featherstone-2008` entry for the
  consolidated textbook treatment.

### `liu-jain-mbs`

Liu, C. K., & Jain, S. _A Quick Tutorial on Multibody Dynamics._ Georgia
Institute of Technology. (`docs/dynamics.pdf`)

- **Type:** paper · **Topic:** dynamics · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** [`docs/background/dynamics/`](https://github.com/dartsim/dart/tree/main/docs/background/dynamics);
  classic `dart/dynamics/`.
- **Notes:** DART's own foundational derivation of articulated-body equations of
  motion. Implemented in classic DART; the experimental world reuses the same
  generalized-coordinate formulation.

### `tan-lcp`

Tan, J., Siu, K., & Liu, C. K. _Contact Handling for Articulated Rigid Bodies
Using LCP._ (`docs/lcp.pdf`)

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** [`docs/background/lcp/`](https://github.com/dartsim/dart/tree/main/docs/background/lcp);
  classic `dart/constraint/`, `dart/math/`.
- **Notes:** DART's LCP-based contact formulation. Implemented in classic DART;
  the experimental constraint stage is a planned reimplementation target.

### `stewart-trinkle-1996`

Stewart, D. E., & Trinkle, J. C. "An implicit time-stepping scheme for rigid
body dynamics with inelastic collisions and Coulomb friction." _IJNME_, 1996.

- **Type:** paper · **Topic:** contact/integration · **Status:** referenced · **Priority:** medium · **Verdict:** baseline
- **Notes:** Standard implicit time-stepping contact formulation; a baseline to
  compare the experimental contact/integration stages against.

### `baraff-1996`

Baraff, D. "Linear-time dynamics using Lagrange multipliers." _SIGGRAPH_, 1996.

- **Type:** paper · **Topic:** dynamics · **Status:** referenced · **Priority:** low · **Verdict:** reference
- **Notes:** Constraint-based dynamics background; cited for context on
  maximal-coordinate vs reduced-coordinate trade-offs.

### `macklin-xpbd-2016`

Macklin, M., Müller, M., & Chentanez, N. "XPBD: position-based simulation of
compliant constrained dynamics." _MIG_, 2016.

- **Type:** paper · **Topic:** integration · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:** named as a candidate integration family in
  [cpp design](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_cpp_api.md);
  surveyed as a compliant-constraint contact model in
  [`PLAN-082 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/082-variational-integrator-solver/contact-roadmap.md).
- **Notes:** Compliant position-based dynamics; under evaluation as an optional
  integration/constraint family behind the DART-owned capability matrix. Its
  compliant-constraint formulation (`α̃ = α/Δt²`) is the closest match to the
  variational integrator's `Δt·M⁻¹` quasi-Newton approximation, making it a
  natural compliant-contact option there. Not a commitment.

### `gjk-1988`

Gilbert, E. G., Johnson, D. W., & Keerthi, S. S. "A fast procedure for computing
the distance between complex objects in three-dimensional space." _IEEE Journal
of Robotics and Automation_, 1988.

- **Type:** paper · **Topic:** collision · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** native collision (`dart/collision/`, libccd-derived GJK/EPA).
- **Notes:** GJK distance/intersection underpins DART's native narrowphase; the
  experimental world consumes collision through the shared collision foundation.

### `ipc-2020`

Minchen Li, Zachary Ferguson, Teseo Schneider, Timothy R. Langlois, Denis
Zorin, Daniele Panozzo, Chenfanfu Jiang, and Danny M. Kaufman. "Incremental
Potential Contact: Intersection- and Inversion-free, Large-Deformation
Dynamics." _ACM Transactions on Graphics_, 39(4), Article 49, 2020. DOI:
[10.1145/3386569.3392425](https://doi.org/10.1145/3386569.3392425).

Related public resources:

- Paper/project page: [ipc-sim.github.io](https://ipc-sim.github.io/)
- Reference implementation: [ipc-sim/IPC](https://github.com/ipc-sim/IPC)
- Modern toolkit: [ipc-sim/ipc-toolkit](https://github.com/ipc-sim/ipc-toolkit)

```bib
@article{Li2020IPC,
  author = {Minchen Li and Zachary Ferguson and Teseo Schneider and Timothy R. Langlois and Denis Zorin and Daniele Panozzo and Chenfanfu Jiang and Danny M. Kaufman},
  title = {Incremental Potential Contact: Intersection- and Inversion-free, Large-Deformation Dynamics},
  journal = {ACM Transactions on Graphics},
  volume = {39},
  number = {4},
  articleno = {49},
  year = {2020},
  doi = {10.1145/3386569.3392425}
}
```

- **Type:** paper · **Topic:** contact/integration · **Status:** in-progress · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-081`](https://github.com/dartsim/dart/blob/main/docs/plans/081-deformable-implicit-barrier-solver.md)
  and its [IPC paper gap audit](https://github.com/dartsim/dart/blob/main/docs/plans/081-deformable-implicit-barrier-solver/ipc-paper-gap-audit.md).
- **Notes:** Provides the target method family for robust deformable contact:
  implicit time stepping with barrier contact, conservative CCD, volumetric
  hyperelasticity, codimensional contact, and lagged variational friction. The
  DART implementation is experimental and DART-owned; DART does not vendor or
  link the upstream IPC repository as a runtime dependency.

  The current DART slice is IPC-inspired deformable groundwork: a public
  deformable-body facade, point-mass nodes, distance-spring edges, fixed nodes,
  an implicit step stage, an analytic static-ground barrier, focused tests, a
  benchmark surface, and a Filament GUI example. It should not be described as
  full IPC parity, mesh IPC contact, or IPC friction.

  Full paper parity still requires mesh-backed FE state, tetrahedral and
  surface topology, material/property coverage, backward Euler and implicit
  Newmark options, boundary conditions, scripted motion, restart/diagnostics,
  point-triangle and edge-edge distances and derivatives, tangent bases,
  edge-edge mollifiers, broad-phase candidate generation, conservative CCD line
  search, sparse projected Newton, adaptive barrier stiffness, smoothed lagged
  friction, and the upstream example/test/benchmark corpus. Promoted examples
  require long-horizon headless Filament evidence with nonblank checks,
  motion-difference checks, and PR-linked visual artifacts.

  Public DART APIs should use DART-owned method/capability names rather than
  exposing the upstream project name as a solver identity, and the upstream
  repository should remain a reference/baseline rather than a runtime
  dependency.

### `chen-2024-vbd`

Anka He Chen, Ziheng Liu, Yin Yang, and Cem Yuksel. "Vertex Block Descent."
_ACM Transactions on Graphics (SIGGRAPH 2024)_, 43(4), 2024. DOI:
[10.1145/3658179](https://doi.org/10.1145/3658179).

Related public resources:

- Paper PDF: <https://graphics.cs.utah.edu/research/projects/vbd/vbd-siggraph2024.pdf>
- Project page: <https://ankachan.github.io/Projects/VertexBlockDescent/index.html>
- Talk: <https://www.youtube.com/watch?v=2HCgKfKy3W8>
- Reference implementations: [AnkaChan/Gaia](https://github.com/AnkaChan/Gaia),
  [AnkaChan/TinyVBD](https://github.com/AnkaChan/TinyVBD)

```bib
@article{Chen2024VBD,
  author = {Chen, Anka He and Liu, Ziheng and Yang, Yin and Yuksel, Cem},
  title = {Vertex Block Descent},
  journal = {ACM Transactions on Graphics},
  volume = {43},
  number = {4},
  year = {2024},
  doi = {10.1145/3658179}
}
```

- **Type:** paper · **Topic:** integration/deformable · **Status:** in-progress · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-104`](https://github.com/dartsim/dart/blob/main/docs/plans/104-vertex-block-descent-solver.md)
  and its [VBD paper gap audit](https://github.com/dartsim/dart/blob/main/docs/dev_tasks/vbd_deformable_solver/vbd-paper-gap-audit.md).
- **Notes:** Block coordinate descent on the variational implicit-Euler
  objective for elastodynamics: each vertex is a 3-DOF block updated with one
  regularized Newton step using a positive-definite local 3x3 Hessian, with
  graph-colored parallel Gauss-Seidel sweeps and adaptive
  initialization/Chebyshev acceleration. DART targets it as a second deformable
  inner solver on the same experimental deformable ECS components and the same
  variational objective the current gradient-descent stage already minimizes.
  The DART implementation is experimental and DART-owned; DART does not vendor
  or link `Gaia`/`TinyVBD` as a runtime dependency, and the public deformable
  stage stays algorithm-neutral. It should not be described as full VBD parity,
  FEM hyperelasticity, VBD contact/friction, GPU VBD, or reference/paper
  performance parity until the relevant PLAN-104 phase lands with evidence.

### `lee-vi-2016`

Jeongseok Lee, C. Karen Liu, Frank C. Park, and Siddhartha S. Srinivasa. "A
Linear-Time Variational Integrator for Multibody Systems." _Algorithmic
Foundations of Robotics XII_ (WAFR 2016), Springer Proceedings in Advanced
Robotics. arXiv:1609.02898.

```bib
@inproceedings{Lee2016VI,
  author = {Jeongseok Lee and C. Karen Liu and Frank C. Park and Siddhartha S. Srinivasa},
  title = {A Linear-Time Variational Integrator for Multibody Systems},
  booktitle = {Algorithmic Foundations of Robotics XII (WAFR 2016)},
  series = {Springer Proceedings in Advanced Robotics},
  year = {2020},
  note = {arXiv:1609.02898}
}
```

- **Type:** paper · **Topic:** dynamics/integration · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-082`](https://github.com/dartsim/dart/blob/main/docs/plans/082-variational-integrator-solver.md)
  and its design owner
  [`simulation_variational_integrator.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_variational_integrator.md).
- **Notes:** DART's own research lineage; the target method for the experimental
  variational-integrator family. Reformulates forward integration as a
  root-finding problem on the forced discrete Euler–Lagrange equation and solves
  it in O(n) via a discrete recursive Newton–Euler residual (DRNEA) plus a
  recursive impulse-based quasi-Newton update (RIQN) that reuses an
  articulated-body-inertia forward-dynamics pass. Symplectic and
  near-energy-conserving for smooth conservative forcing; the paper does not
  address contact, friction, or closed loops (PLAN-082 roadmaps those). The
  experimental world has no ABA yet, so the O(n) update is net-new work; classic
  DART implemented the original (reference repo `jslee02/wafr2016`).

### `marsden-west-2001`

Marsden, J. E., & West, M. "Discrete mechanics and variational integrators."
_Acta Numerica_, 10, 357–514 (2001).

- **Type:** paper · **Topic:** integration · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Where used:** foundational citation for
  [`simulation_variational_integrator.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_variational_integrator.md).
- **Notes:** Foundational treatment of discrete mechanics, the discrete
  Euler–Lagrange equation, and the symplectic/momentum-conservation properties
  that `lee-vi-2016` builds on. Cited for the structure-preservation rationale,
  not a direct implementation dependency.

### `vbd-2024`

Chen, A. H., Liu, K., et al. "Vertex Block Descent." _ACM Transactions on
Graphics_ (SIGGRAPH 2024). arXiv:2403.06321.

- **Type:** paper · **Topic:** contact/integration · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:** contact-extension survey in
  [`PLAN-082 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/082-variational-integrator-solver/contact-roadmap.md).
- **Notes:** Vertex-level block coordinate descent on the incremental-potential
  timestep with graph coloring; adopts the IPC lagged friction model.
  Surveyed (not adopted) as inspiration for adding contact/friction to the
  variational integrator; its block-descent solver is largely redundant with the
  VI's O(n) recursive update, but its monotone-energy-descent and warmstart
  discipline are informative.

### `avbd-2025`

Giles, et al. "Augmented Vertex Block Descent." _ACM Transactions on Graphics_
(SIGGRAPH 2025).

- **Type:** paper · **Topic:** contact/integration · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:** contact-extension survey in
  [`PLAN-082 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/082-variational-integrator-solver/contact-roadmap.md).
- **Notes:** Augmented-Lagrangian extension of VBD with bounded constraint
  forces (`f = clamp(k·C+λ, fmin, fmax)`) and Coulomb friction via a lagged
  normal-force clamp. Identified as the best structural fit for adding hard,
  drift-free non-penetration and a friction cone to the variational integrator
  without a global PSD solve (PLAN-082 contact rung C3). Not a commitment.

### `johnson-murphey-2009`

Johnson, E. R., & Murphey, T. D. "Scalable Variational Integrators for
Constrained Mechanical Systems in Generalized Coordinates." _IEEE Transactions
on Robotics_, 25(6), 2009.

- **Type:** paper · **Topic:** integration/dynamics · **Status:** referenced · **Priority:** high · **Verdict:** baseline
- **Where used:** the explicit predecessor `lee-vi-2016` benchmarks against
  ("SVI"); informs the PLAN-082 constraint roadmap (Phase B2).
- **Notes:** The scalable variational integrator in generalized coordinates
  that `lee-vi-2016` improves from O(n²) evaluation / O(n³) Jacobian to O(n). Its
  recursive tree formulation with caching and its constrained discrete
  Euler–Lagrange system (holonomic `h(q)=0` via the augmented multiplier system
  `D2D1Ld − Dhᵀλ = 0`, `Dh(q^{k+1}) = 0`, enforced directly each step with no
  drift) are the reference for DART's loop-closure (Phase B2) work. Uses
  directional pin constraints `n̂·(p1−p2)=0` to avoid the zero-gradient
  singularity of a satisfied distance constraint.

### `leyendecker-2008`

Leyendecker, S., Marsden, J. E., & Ortiz, M. "Variational integrators for
constrained dynamical systems." _ZAMM_, 88(9), 2008. (with the
Betsch–Leyendecker discrete null-space method, _Comput. Methods Appl. Mech.
Engrg._, 2006.)

- **Type:** paper · **Topic:** integration/contact · **Status:** referenced · **Priority:** high · **Verdict:** evaluate
- **Where used:** [`PLAN-082 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/082-variational-integrator-solver/contact-roadmap.md)
  (equality-constraint / closed-loop route, Phase B2).
- **Notes:** Constrained variational integrators with holonomic constraints. The
  discrete null-space method picks `P(q)` spanning `ker Dg(q)` and left-multiplies
  the constrained DEL by `Pᵀ` to annihilate the multiplier term, then
  reparameterizes onto a minimal coordinate set — a drift-free, multiplier-free,
  well-conditioned route for closed loops. For an open kinematic tree the
  generalized coordinates are already minimal, so this matters mainly for
  loop closures; the multiplier reaction `Gᵀλ` can alternatively be injected as a
  generalized impulse into the DART O(n) solve.

### `kobilarov-crane-desbrun-2009`

Kobilarov, M., Crane, K., & Desbrun, M. "Lie group integrators for animation
and control of vehicles." _ACM Transactions on Graphics_, 28(2), 2009.

- **Type:** paper · **Topic:** integration · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Where used:** supporting reference for
  [`simulation_variational_integrator.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_variational_integrator.md).
- **Notes:** Independently confirms the inverse right-trivialized tangent
  coefficients (`I − ½[ad] + (1/12)[ad]²`) DART's `dexp⁻¹` uses, and documents the
  **Cayley map** as a cheaper alternative group-difference map that avoids the
  Bernoulli series and the `log`-map singularity at rotation = π — the recorded
  escape hatch if that singularity ever becomes binding.

## Model Formats & Standards

| ID         | Reference                                | Topic        | Status     | Priority | Verdict   |
| ---------- | ---------------------------------------- | ------------ | ---------- | -------- | --------- |
| `urdf`     | URDF (Unified Robot Description Format)  | model-format | referenced | medium   | reference |
| `sdformat` | SDFormat (Simulation Description Format) | model-format | referenced | medium   | reference |
| `mjcf`     | MJCF (MuJoCo XML)                        | model-format | referenced | low      | reference |

### `urdf`

Unified Robot Description Format. <https://wiki.ros.org/urdf/XML/joint>

- **Type:** standard · **Topic:** model-format/terminology · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Notes:** Robotics interchange standard; source of the `Floating` joint
  naming and the revolute/prismatic/fixed/planar vocabulary. Loading into the
  experimental world is deferred until a public C++ import owner exists; classic
  `dart::io` already parses URDF.

### `sdformat`

Simulation Description Format. <https://sdformat.org/spec>

- **Type:** standard · **Topic:** model-format · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Notes:** Graph-structured model format able to express closed kinematic
  loops; informs the first-class loop-closure design. Classic `dart::io` parses
  SDFormat.

### `mjcf`

MuJoCo XML (MJCF). <https://mujoco.readthedocs.io/en/stable/XMLreference.html>

- **Type:** standard · **Topic:** model-format · **Status:** referenced · **Priority:** low · **Verdict:** reference
- **Notes:** Referenced for joint/equality vocabulary comparison; import is
  deferred.

## Comparative Implementations (Engines)

Software references that informed the experimental API shape, terminology, and
algorithm-family choices. These are baselines/comparisons, not dependencies.

| ID            | Engine                               | Used for                                               | Status     | Verdict   |
| ------------- | ------------------------------------ | ------------------------------------------------------ | ---------- | --------- |
| `drake`       | Drake (`MultibodyPlant`)             | terminology, constraints, stepping API                 | referenced | baseline  |
| `pinocchio`   | Pinocchio (Stack-of-Tasks)           | terminology (Spherical/FreeFlyer), dynamics algos      | referenced | baseline  |
| `rbdl`        | RBDL                                 | terminology (FloatingBase/Helical), ABA/RNEA/CRBA      | referenced | baseline  |
| `mujoco`      | MuJoCo / MJX                         | stepping, equality constraints, state vocabulary       | referenced | baseline  |
| `physx-isaac` | NVIDIA PhysX / Isaac Sim / Isaac Lab | articulation concept, closed-loop rigging              | referenced | reference |
| `newton`      | NVIDIA Newton (Warp)                 | model/solver split, GPU direction                      | referenced | reference |
| `genesis`     | Genesis                              | entity/morph model, batched sim                        | referenced | reference |
| `bullet`      | Bullet / PyBullet                    | facade-over-engine pattern, `btMultiBody`              | referenced | reference |
| `gazebo`      | Gazebo / gz-physics / SDFormat       | downstream integration, kinematic loops                | referenced | baseline  |
| `gaia`        | Gaia (VBD research framework)        | VBD correctness/performance baseline (`chen-2024-vbd`) | referenced | baseline  |
| `tinyvbd`     | TinyVBD (minimal VBD reference)      | VBD algorithm reference (`chen-2024-vbd`)              | referenced | baseline  |

### Notes on engine verdicts

- **`drake`, `pinocchio`, `rbdl`, `mujoco`, `gazebo` — baseline:** mature
  robotics-dynamics references the experimental world should be benchmarked and
  compared against, and whose terminology (multibody, spherical, floating)
  DART follows. gz-physics is also DART's primary downstream integration.
- **`physx-isaac`, `newton`, `genesis`, `bullet` — reference:** consulted for
  API patterns and the GPU/batched roadmap, but their engine-specific names
  (e.g., "Articulation", Unity-style "Rigidbody") are deliberately _not_ adopted
  where they diverge from the robotics literature.
- **`gaia`, `tinyvbd` — baseline:** the reference implementations of Vertex
  Block Descent (`chen-2024-vbd`). DART's PLAN-104 VBD solver is compared
  against them for correctness and performance on matched scenes, but DART
  reimplements VBD independently and does not vendor or link them.

Design-doc links for these comparisons live in
[`simulation_experimental_cpp_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_cpp_api.md)
and
[`simulation_experimental_python_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_experimental_python_api.md).

## Terminology Grounding

The experimental world's naming decisions are grounded in the references above
rather than any single engine:

- `Multibody` (one word) — `shabana-mbs`, `featherstone-2008`, `drake`.
- `Spherical` joint (not "ball") — `lynch-park-2017`, `pinocchio`, `rbdl`,
  `drake`.
- `Floating` joint (not "free") — `urdf`, `rbdl`, `drake`.
- `RigidBody` (two words) — `bullet`, `drake`, PhysX; "rigid body" is two words
  unlike one-word "multibody".

## Maintenance

This catalog is maintained with the `dart-references` skill
(`.claude/skills/dart-references/SKILL.md`), which documents the entry schema,
the status/verdict workflow, and how to keep entries in sync with the design
docs and code. Keep citations accurate, prefer official sources, and update an
entry's `status`/`verdict` when the experimental world's relationship to a
reference changes (for example, when a `planned` algorithm becomes
`implemented`).
