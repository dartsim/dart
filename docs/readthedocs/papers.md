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

The catalog currently focuses on the DART 7 simulation world
(`dart/simulation/**` and `dartpy.simulation`) and its algorithms. The schema
is general, so it can extend to the rest of DART without changing its shape.

It is a companion to the API design docs:

- [`simulation_cpp_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)
- [`simulation_python_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_python_api.md)

## Why This Exists

DART's [north star](https://github.com/dartsim/dart/blob/main/docs/ai/north-star.md)
is a research-focused engine where "the
easiest place to reproduce and evaluate a new algorithm should be inside DART,"
and it names a tracked gap: _algorithm-family contracts and baseline comparisons
are not yet tracked as first-class research surfaces._ This catalog is that
surface for the DART 7 simulation world: it records what the new API and algorithms
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
| **Status**     | `referenced`, `planned`, `in-progress`, `implemented`, `deferred`, `rejected`                              | The DART 7 simulation world's relationship to the reference.                                  |
| **Priority**   | `high`, `medium`, `low`, `—`                                                                               | Relative importance for acting on or evaluating the reference; `—` when purely informational. |
| **Verdict**    | `adopt`, `baseline`, `reference`, `evaluate`, `reject`                                                     | The project's decision: build on it, compare against it, cite it, weigh it, or pass.          |
| **Where used** | links / paths                                                                                              | Design doc, code, or test that uses or will use it.                                           |

Status values are written from the DART 7 simulation world's perspective. A method
already shipping in _classic_ DART but not yet in the DART 7 simulation world is
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
- **Where used:** [cpp design](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)
  (articulated-body method); terminology grounding in the
  "Terminology Grounding" section below.
- **Notes:** Canonical reference for the kinematic-tree model and the O(n)
  Articulated-Body Algorithm (ABA). Source of the "kinematic tree" /
  "multibody system" vocabulary the DART 7 API adopts. ABA is the planned
  basis for the DART 7 forward-dynamics stage; DART 7 forward
  kinematics already follows its tree formulation.

### `lynch-park-2017`

Lynch, K. M., & Park, F. C. _Modern Robotics: Mechanics, Planning, and Control._
Cambridge University Press, 2017.

- **Type:** textbook · **Topic:** kinematics/terminology · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** DART 7 `JointType` taxonomy and frame/transform vocabulary; drove `Ball`→`Spherical` naming.
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

| ID                                 | Reference                                                                                                                  | Topic                              | Status      | Priority | Verdict   |
| ---------------------------------- | -------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- | ----------- | -------- | --------- |
| `featherstone-1983`                | Featherstone, "Calculation of robot dynamics using articulated-body inertias" (1983)                                       | dynamics                           | planned     | high     | adopt     |
| `liu-jain-mbs`                     | Liu & Jain, _A Quick Tutorial on Multibody Dynamics_                                                                       | dynamics                           | implemented | —        | adopt     |
| `tan-lcp`                          | Tan, Siu & Liu, _Contact Handling for Articulated Rigid Bodies Using LCP_                                                  | contact                            | implemented | —        | adopt     |
| `stewart-trinkle-1996`             | Stewart & Trinkle, "An implicit time-stepping scheme … Coulomb friction" (1996)                                            | contact                            | referenced  | medium   | baseline  |
| `baraff-1996`                      | Baraff, "Linear-time dynamics using Lagrange multipliers" (1996)                                                           | dynamics                           | referenced  | low      | reference |
| `macklin-xpbd-2016`                | Macklin et al., "XPBD: position-based simulation of compliant constrained dynamics" (2016)                                 | integration                        | referenced  | medium   | evaluate  |
| `gjk-1988`                         | Gilbert, Johnson & Keerthi, GJK distance algorithm (1988)                                                                  | collision                          | implemented | —        | adopt     |
| `ipc-2020`                         | Li et al., "Incremental Potential Contact" (2020)                                                                          | contact                            | in-progress | high     | adopt     |
| `rigid-ipc-2021`                   | Ferguson et al., "Intersection-free Rigid Body Dynamics" (2021)                                                            | contact                            | in-progress | high     | adopt     |
| `lan-2022-abd`                     | Lan et al., "Affine Body Dynamics" (2022)                                                                                  | contact/dynamics                   | planned     | high     | adopt     |
| `chen-2022-unified-newton-barrier` | Chen et al., "A Unified Newton Barrier Method for Multibody Dynamics" (2022)                                               | contact/integration/multibody      | planned     | high     | adopt     |
| `lan-2022-pdipc`                   | Lan et al., "Penetration-free Projective Dynamics on the GPU" (2022)                                                       | contact/integration/GPU            | planned     | high     | evaluate  |
| `chen-2023-spb`                    | Chen, Diaz & Yuksel, "Shortest Path to Boundary for Self-Intersecting Meshes" (2023)                                       | contact/collision                  | planned     | high     | evaluate  |
| `ando-2024-cubic-barrier`          | Ando, "A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness" (2024)                                                 | contact/deformable/GPU             | planned     | high     | evaluate  |
| `werling-2021`                     | Werling et al., "Fast and Feature-Complete Differentiable Physics … Articulated Rigid Bodies" (2021)                       | differentiable                     | in-progress | high     | adopt     |
| `howell-2022-dojo`                 | Howell et al., "Dojo: A Differentiable Physics Engine for Robotics" (2022)                                                 | differentiable/contact/integration | planned     | high     | evaluate  |
| `lee-vi-2016`                      | Lee, Liu, Park & Srinivasa, "A Linear-Time Variational Integrator for Multibody Systems" (2016)                            | integration                        | planned     | high     | adopt     |
| `marsden-west-2001`                | Marsden & West, "Discrete mechanics and variational integrators" (2001)                                                    | integration                        | referenced  | medium   | reference |
| `chen-2024-vbd`                    | Chen et al., "Vertex Block Descent" (SIGGRAPH 2024)                                                                        | integration                        | in-progress | high     | adopt     |
| `vbd-2024`                         | Chen et al., "Vertex Block Descent" (2024) — VI contact survey                                                             | contact                            | referenced  | medium   | evaluate  |
| `avbd-2025`                        | Giles et al., "Augmented Vertex Block Descent" (2025)                                                                      | contact/integration                | in-progress | high     | adopt     |
| `modi-2024-simplicits`             | Modi et al., "Simplicits: Mesh-Free, Geometry-Agnostic, Elastic Simulation" (2024)                                         | integration/deformable             | planned     | high     | adopt     |
| `smith-2012-rosi`                  | Smith et al., "Reflections on Simultaneous Impact" (2012)                                                                  | contact/impact                     | referenced  | medium   | baseline  |
| `zhang-2015-qce`                   | Zhang et al., "Quadratic Contact Energy Model for Multi-impact Simulation" (2015)                                          | contact/impact                     | referenced  | medium   | evaluate  |
| `vouga-2017-all-well`              | Vouga et al., "All's Well That Ends Well: Guaranteed Resolution of Simultaneous Rigid Body Impact" (2017)                  | contact/impact                     | referenced  | medium   | evaluate  |
| `halm-posa-2024-set-valued-impact` | Halm & Posa, "Set-valued rigid-body dynamics for simultaneous, inelastic, frictional impacts" (2024)                       | contact/impact                     | referenced  | medium   | evaluate  |
| `lelidec-2024-contact-models`      | Le Lidec et al., "Contact Models in Robotics: a Comparative Analysis" (2024)                                               | contact/survey                     | referenced  | medium   | reference |
| `ogc-2025`                         | Chen et al., "Offset Geometric Contact" (2025)                                                                             | contact/collision                  | planned     | high     | evaluate  |
| `nakamura-1987-task-priority`      | Nakamura, Hanafusa & Yoshikawa, "Task-Priority Based Redundancy Control of Robot Manipulators" (1987)                      | kinematics                         | planned     | medium   | adopt     |
| `buss-kim-2005-sdls`               | Buss & Kim, "Selectively Damped Least Squares for Inverse Kinematics" (2005)                                               | kinematics                         | planned     | medium   | evaluate  |
| `aristidou-lasenby-2011-fabrik`    | Aristidou & Lasenby, "FABRIK: A fast, iterative solver for the Inverse Kinematics problem" (2011)                          | kinematics                         | planned     | medium   | evaluate  |
| `beeson-ames-2015-tracik`          | Beeson & Ames, "TRAC-IK: An open-source library for improved solving of generic inverse kinematics" (2015)                 | kinematics                         | planned     | medium   | baseline  |
| `starke-2020-bioik`                | Starke, _Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective Inverse Kinematics_ (2020)                   | kinematics                         | planned     | medium   | evaluate  |
| `rakita-2018-relaxedik`            | Rakita, Mutlu & Gleicher, "RelaxedIK: Real-time Synthesis of Accurate and Feasible Robot Arm Motion" (2018)                | kinematics                         | planned     | high     | baseline  |
| `zucker-2013-chomp`                | Zucker et al., "CHOMP: Covariant Hamiltonian Optimization for Motion Planning" (2013)                                      | motion-planning                    | referenced  | medium   | baseline  |
| `mukadam-2018-gpmp2`               | Mukadam et al., "Continuous-time Gaussian process motion planning via probabilistic inference" (2018)                      | motion-planning                    | referenced  | medium   | evaluate  |
| `ames-2022-ikflow`                 | Ames, Morgan & Konidaris, "IKFlow: Generating Diverse Inverse Kinematics Solutions" (2022)                                 | kinematics                         | deferred    | medium   | evaluate  |
| `johnson-murphey-2009`             | Johnson & Murphey, "Scalable variational integrators for constrained mechanical systems in generalized coordinates" (2009) | integration                        | referenced  | high     | baseline  |
| `leyendecker-2008`                 | Leyendecker, Marsden & Ortiz, "Variational integrators for constrained dynamical systems" (2008)                           | integration                        | referenced  | high     | evaluate  |
| `kobilarov-crane-desbrun-2009`     | Kobilarov, Crane & Desbrun, "Lie group integrators for animation and control of vehicles" (2009)                           | integration                        | referenced  | medium   | reference |

### `featherstone-1983`

Featherstone, R. "The calculation of robot dynamics using articulated-body
inertias." _International Journal of Robotics Research_, 2(1), 1983.

- **Type:** paper · **Topic:** dynamics · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Notes:** Original ABA paper; the planned forward-dynamics method for the
  DART 7 dynamics stage. See the `featherstone-2008` entry for the
  consolidated textbook treatment.

### `liu-jain-mbs`

Liu, C. K., & Jain, S. _A Quick Tutorial on Multibody Dynamics._ Georgia
Institute of Technology. (`docs/dynamics.pdf`)

- **Type:** paper · **Topic:** dynamics · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** [`docs/background/dynamics/`](https://github.com/dartsim/dart/tree/main/docs/background/dynamics);
  classic `dart/dynamics/`.
- **Notes:** DART's own foundational derivation of articulated-body equations of
  motion. Implemented in classic DART; the DART 7 simulation world reuses the same
  generalized-coordinate formulation.

### `tan-lcp`

Tan, J., Siu, K., & Liu, C. K. _Contact Handling for Articulated Rigid Bodies
Using LCP._ (`docs/lcp.pdf`)

- **Type:** paper · **Topic:** contact · **Status:** implemented · **Priority:** — · **Verdict:** adopt
- **Where used:** [`docs/background/lcp/`](https://github.com/dartsim/dart/tree/main/docs/background/lcp);
  classic `dart/constraint/`, `dart/math/`.
- **Notes:** DART's LCP-based contact formulation. Implemented in classic DART;
  the DART 7 constraint stage is a planned reimplementation target.

### `stewart-trinkle-1996`

Stewart, D. E., & Trinkle, J. C. "An implicit time-stepping scheme for rigid
body dynamics with inelastic collisions and Coulomb friction." _IJNME_, 1996.

- **Type:** paper · **Topic:** contact/integration · **Status:** referenced · **Priority:** medium · **Verdict:** baseline
- **Notes:** Standard implicit time-stepping contact formulation; a baseline to
  compare the DART 7 contact/integration stages against.

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
  [cpp design](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md);
  surveyed as a compliant-constraint contact model in
  [`PLAN-084 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/084-variational-integrator-solver/contact-roadmap.md).
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
  DART 7 simulation world consumes collision through the shared collision foundation.

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

### `rigid-ipc-2021`

Zachary Ferguson, Minchen Li, Teseo Schneider, Francisca Gil-Ureta, Timothy
Langlois, Chenfanfu Jiang, Denis Zorin, Danny M. Kaufman, and Daniele Panozzo.
"Intersection-free Rigid Body Dynamics." _ACM Transactions on Graphics_, 40(4),
Article 183, 2021. DOI:
[10.1145/3450626.3459802](https://doi.org/10.1145/3450626.3459802).

Related public resources:

- Paper/project page:
  [ipc-sim.github.io/rigid-ipc](https://ipc-sim.github.io/rigid-ipc/)
- Reference implementation:
  [ipc-sim/rigid-ipc](https://github.com/ipc-sim/rigid-ipc)

```bib
@article{Ferguson2021RigidIPC,
  author = {Zachary Ferguson and Minchen Li and Teseo Schneider and Francisca Gil-Ureta and Timothy Langlois and Chenfanfu Jiang and Denis Zorin and Danny M. Kaufman and Daniele Panozzo},
  title = {Intersection-free Rigid Body Dynamics},
  journal = {ACM Transactions on Graphics},
  volume = {40},
  number = {4},
  articleno = {183},
  year = {2021},
  doi = {10.1145/3450626.3459802}
}
```

- **Type:** paper · **Topic:** contact/integration · **Status:** in-progress · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-082`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact.md)
  and its
  [rigid-ipc manifest](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json).
- **Notes:** Provides the target method family for robust rigid-body contact:
  reduced-coordinate incremental potentials, contact barriers, curved-trajectory
  continuous collision detection with minimum separation, projected Newton,
  friction, and a fixture/test/benchmark corpus for rigid scenes. The DART
  implementation is experimental and DART-owned; the upstream repository is a
  reference and benchmark baseline, not a runtime dependency.

  Current DART work has the audited upstream manifest, an internal fixture
  reader with first experimental-`World` replay state, supported
  OBJ/OFF/MSH/STL mesh replay plus polygonal inline geometry replay into
  native-backed mesh collision shapes, upstream comparison `.txt` shape-row
  ingestion, direct CCD row readers, and first internal curved-trajectory CCD
  tests for face-vertex, edge-edge, and point-edge primitives. The first direct
  evaluator regressions cover upstream-style
  expected-TOI rows, parameter-space residual equations for those contacts, and
  first parameter-box subdivision queries for those contacts, plus first
  root-row full-step misses, not full corpus parity. The complete implementation
  still needs remaining comparison script command coverage, full conservative
  CCD parity, barrier derivatives and Hessians, sparse projected Newton, lagged
  smoothed friction, same-domain rigid method selection, CPU/GPU benchmark
  packets, comparison baselines, and headless Filament evidence for promoted
  scenes.

### `lan-2022-abd`

Lei Lan, Danny M. Kaufman, Minchen Li, Chenfanfu Jiang, and Yin Yang.
"Affine Body Dynamics: Fast, Stable & Intersection-free Simulation of Stiff
Materials." _ACM Transactions on Graphics_, 41(4), Article 67, 2022. DOI:
[10.1145/3528223.3530064](https://doi.org/10.1145/3528223.3530064).

Related public resources:

- Paper: [arxiv.org/abs/2201.10022](https://arxiv.org/abs/2201.10022)
- Presentation deck used for PLAN-083:
  [Multibody ABD deck](https://games-1312234642.cos.ap-guangzhou.myqcloud.com/pdf/Games2022242%E6%9D%8E%E6%97%BB%E8%BE%B0.pdf)

```bib
@article{Lan2022ABD,
  author = {Lei Lan and Danny M. Kaufman and Minchen Li and Chenfanfu Jiang and Yin Yang},
  title = {Affine Body Dynamics: Fast, Stable \& Intersection-free Simulation of Stiff Materials},
  journal = {ACM Transactions on Graphics},
  volume = {41},
  number = {4},
  articleno = {67},
  year = {2022},
  doi = {10.1145/3528223.3530064}
}
```

- **Type:** paper · **Topic:** contact/dynamics · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-083`](https://github.com/dartsim/dart/blob/main/docs/plans/083-unified-newton-barrier-multibody.md)
  as the planned affine/stiff-body representation track, coordinated with
  [`PLAN-082`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact.md).
- **Notes:** ABD is the planned reduced stiff-body branch of DART's unified
  Newton-barrier family. It treats a stiff body as a compact affine coordinate
  system with an orthogonality stiffness, preserving IPC-style
  nonintersection/friction guarantees while avoiding the expensive curved
  trajectory machinery of exact rigid IPC.

  DART should evaluate ABD as an additional internal method family, not as an
  immediate replacement for the existing exact/reduced rigid IPC path. A
  promoted ABD slice must prove correctness against rigid IPC fixtures, current
  DART rigid contact scenes, and the paper/deck benchmarks before any default
  solver behavior changes. Public APIs should use DART-owned names such as
  affine stiff-body dynamics rather than exposing `ABD` as a user-facing solver
  identity.

### `chen-2022-unified-newton-barrier`

Yunuo Chen, Minchen Li, Lei Lan, Hao Su, Yin Yang, and Chenfanfu Jiang. "A
Unified Newton Barrier Method for Multibody Dynamics." _ACM Transactions on
Graphics_, 41(4), Article 66, 2022. DOI:
[10.1145/3528223.3530076](https://doi.org/10.1145/3528223.3530076).

Related public resources:

- Paper PDF supplied for PLAN-083:
  [UCSD-hosted PDF](https://cseweb.ucsd.edu/~haosu/Other_Doc/papers_212s4-yaml25_finalppr.pdf)

```bib
@article{Chen2022UnifiedNewtonBarrier,
  author = {Yunuo Chen and Minchen Li and Lei Lan and Hao Su and Yin Yang and Chenfanfu Jiang},
  title = {A Unified Newton Barrier Method for Multibody Dynamics},
  journal = {ACM Transactions on Graphics},
  volume = {41},
  number = {4},
  articleno = {66},
  year = {2022},
  doi = {10.1145/3528223.3530076}
}
```

- **Type:** paper · **Topic:** contact/integration/multibody · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-083`](https://github.com/dartsim/dart/blob/main/docs/plans/083-unified-newton-barrier-multibody.md)
  as the cross-variant plan for consolidating deformable IPC, rigid IPC, ABD,
  articulation constraints, restitution, CPU/GPU evidence, and py-demos examples.
- **Notes:** This paper is the target for DART's unified Newton-barrier
  multibody direction. It combines IPC-style contact/friction, affine/stiff
  bodies, deformables, codimensional geometry, linear equality constraints
  through change of variables, nonlinear equality penalties, inequality barrier
  ranges, BDF-2 integration, and semi-implicit Rayleigh damping for restitution.

  DART should implement the method as a DART-owned solver family inside the
  DART 7 `World`, using PLAN-081 and PLAN-082 as variant owners and
  PLAN-083 as the shared API/kernel/benchmark plan. Completion requires mapped
  figure/table/unit-test coverage, py-demos examples, CPU and GPU benchmark
  packets, and explicit comparisons against DART incumbents, upstream
  references, and paper-reported numbers. The paper and deck remain references
  and baselines, not runtime dependencies.

### `lan-2022-pdipc`

Lei Lan, Guanqun Ma, Yin Yang, Changxi Zheng, Minchen Li, and Chenfanfu Jiang.
"Penetration-free Projective Dynamics on the GPU." _ACM Transactions on
Graphics_, 41(4), Article 69, 2022. DOI:
[10.1145/3528223.3530069](https://doi.org/10.1145/3528223.3530069).

Related public resources:

- Paper PDF:
  [lan2022pdipc.pdf](https://www.math.ucla.edu/multiples/publication/lan2022pdipc.pdf)
- GAMES webinar deck:
  [Games2022240](https://games-1312234642.cos.ap-guangzhou.myqcloud.com/pdf/Games2022240%E8%93%9D%E7%A3%8A.pdf)
- Summary page:
  [Physics-Based Animation](https://www.physicsbasedanimation.com/2022/05/27/penetration-free-projective-dynamics-on-the-gpu/)
- Paper video:
  [youtube.com/watch?v=acEplzIIOEs](https://www.youtube.com/watch?v=acEplzIIOEs)
- CCD reference implementation:
  [Continuous-Collision-Detection/Fast-Approximate-Root-CCD](https://github.com/Continuous-Collision-Detection/Fast-Approximate-Root-CCD)
- Robotics projective-dynamics reference:
  [DragonZoom/projective-dynamics-for-robotics](https://github.com/DragonZoom/projective-dynamics-for-robotics)

- **Type:** paper · **Topic:** contact/integration/GPU · **Status:** planned · **Priority:** high · **Verdict:** evaluate
- **Where used:**
  [`PLAN-081 PD-IPC GPU gap audit`](https://github.com/dartsim/dart/blob/main/docs/plans/081-deformable-implicit-barrier-solver/pd-ipc-gpu-gap-audit.md).
- **Notes:** Candidate GPU acceleration path for IPC-class deformable contact:
  it combines projective dynamics, IPC barriers, two-level local/global
  iteration, A-Jacobi, patch-based GPU collision culling, and
  minimum-gradient-Newton CCD pruning for solids and shells. DART should treat
  the fast CCD shortcut as an optimization to validate against conservative
  native CCD, because the public non-author CCD implementation reports missed
  point-triangle and edge-edge collisions. Keep the method internal and
  backend-neutral until CPU reference tests, A-Jacobi convergence evidence,
  conservative-CCD fallback policy, GPU packet benchmarks, and headless visual
  evidence pass.

### `werling-2021`

Werling, K., Omens, D., Lee, J., Exarchos, I., & Liu, C. K. "Fast and
Feature-Complete Differentiable Physics for Articulated Rigid Bodies."
_Robotics: Science and Systems (RSS)_, 2021. arXiv:2103.16021.

Related public resources:

- Paper: [arxiv.org/abs/2103.16021](https://arxiv.org/abs/2103.16021)
- Project docs: [nimblephysics.org/docs](https://nimblephysics.org/docs/)
- Reference implementation (a DART fork):
  [keenon/nimblephysics](https://github.com/keenon/nimblephysics)

```bib
@inproceedings{Werling2021Differentiable,
  author = {Keenon Werling and Dalton Omens and Jeongseok Lee and Ioannis Exarchos and C. Karen Liu},
  title = {Fast and Feature-Complete Differentiable Physics for Articulated Rigid Bodies},
  booktitle = {Robotics: Science and Systems (RSS)},
  year = {2021},
  eprint = {2103.16021}
}
```

- **Type:** paper · **Topic:** differentiable · **Status:** in-progress · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-110`](https://github.com/dartsim/dart/blob/main/docs/plans/110-differentiable-simulation.md),
  its [Nimble paper gap audit](https://github.com/dartsim/dart/blob/main/docs/plans/110-differentiable-simulation/nimble-gap-audit.md),
  and [differentiable-simulation design](https://github.com/dartsim/dart/blob/main/docs/design/differentiable_simulation.md).
- **Notes:** Target method for opt-in differentiable simulation: analytic
  gradients through the contact LCP in generalized coordinates by implicit
  differentiation of the complementarity conditions, exploiting active-set
  sparsity (a small clamping-block solve) for speed. The paper itself is a DART
  fork, so its forward-pass ingredients map directly onto DART's boxed-LCP
  library, articulated-body dynamics, and `tan-lcp` contact formulation.

  The DART implementation is experimental, DART-owned, and **opt-in** (off by
  default, zero cost when disabled) — a deliberate divergence from the
  always-on upstream. DART does not vendor or link the upstream repository as a
  runtime dependency, and public APIs use DART-owned method/quantity names
  rather than the upstream project name as a solver identity.

  This entry should not be described as complete: full parity requires the
  boxed-LCP contact path in the DART 7 simulation world (PLAN-080 prerequisite), the
  clamping/separating/tied classification and clamping-block gradient, contact
  Jacobian derivatives, the parameter (mass/COM/inertia/friction) selector,
  the PyTorch `autograd.Function` bridge, finite-difference-checked correctness,
  and trajectory-optimization plus system-identification examples.

### `howell-2022-dojo`

Howell, T. A., Le Cleac'h, S., Bruedigam, J., Chen, Q., Sun, J., Kolter, J. Z.,
Schwager, M., & Manchester, Z. "Dojo: A Differentiable Physics Engine for
Robotics." arXiv:2203.00806, 2022.

Related public resources:

- Paper: [arxiv.org/abs/2203.00806](https://arxiv.org/abs/2203.00806)
- Project site: [sites.google.com/view/dojo-sim/home](https://sites.google.com/view/dojo-sim/home)
- Reference implementation:
  [dojo-sim/Dojo.jl](https://github.com/dojo-sim/Dojo.jl)

```bib
@article{howelllecleach2022,
  title={Dojo: A Differentiable Physics Engine for Robotics},
  author={Howell, Taylor and Le Cleac'h, Simon and Bruedigam, Jan and Chen, Qianzhong and Sun, Jiankai and Kolter, J. Zico and Schwager, Mac and Manchester, Zachary},
  journal={arXiv preprint arXiv:2203.00806},
  url={https://arxiv.org/abs/2203.00806},
  year={2022}
}
```

- **Type:** paper · **Topic:** differentiable/contact/integration · **Status:** planned · **Priority:** high · **Verdict:** evaluate
- **Where used:** [`PLAN-110`](https://github.com/dartsim/dart/blob/main/docs/plans/110-differentiable-simulation.md)
  and its [Dojo solver gap audit](https://github.com/dartsim/dart/blob/main/docs/plans/110-differentiable-simulation/dojo-gap-audit.md).
- **Notes:** Dojo is an additional differentiable-rigid-body solver reference,
  not the current PLAN-110 implementation target. The method combines
  maximal-coordinate rigid bodies, variational integration, hard contact as a
  nonlinear complementarity problem with second-order cone friction constraints,
  a custom primal-dual interior-point solver, and implicit gradients through the
  relaxed solve. DART will evaluate whether those ideas justify a separate
  opt-in solver family after the active Nimble-style boxed-LCP path lands. The
  Dojo.jl repository is a comparison baseline only; DART does not vendor or link
  it as a runtime dependency.

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
  and its [VBD paper gap audit](https://github.com/dartsim/dart/blob/main/docs/plans/104-vertex-block-descent-solver/vbd-paper-gap-audit.md).
- **Notes:** Block coordinate descent on the variational implicit-Euler
  objective for elastodynamics: each vertex is a 3-DOF block updated with one
  regularized Newton step using a positive-definite local 3x3 Hessian, with
  graph-colored parallel Gauss-Seidel sweeps and adaptive
  initialization/Chebyshev acceleration. DART targets it as a second deformable
  inner solver on the same DART 7 deformable ECS components and the same
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
- **Where used:** [`PLAN-084`](https://github.com/dartsim/dart/blob/main/docs/plans/084-variational-integrator-solver.md)
  and its design owner
  [`simulation_variational_integrator.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_variational_integrator.md).
- **Notes:** DART's own research lineage; the target method for the experimental
  variational-integrator family. Reformulates forward integration as a
  root-finding problem on the forced discrete Euler–Lagrange equation and solves
  it in O(n) via a discrete recursive Newton–Euler residual (DRNEA) plus a
  recursive impulse-based quasi-Newton update (RIQN) that reuses an
  articulated-body-inertia forward-dynamics pass. Symplectic and
  near-energy-conserving for smooth conservative forcing; the paper does not
  address contact, friction, or closed loops (PLAN-084 roadmaps those). The
  DART 7 simulation world has no ABA yet, so the O(n) update is net-new work; classic
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
  [`PLAN-084 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/084-variational-integrator-solver/contact-roadmap.md).
- **Notes:** Vertex-level block coordinate descent on the incremental-potential
  timestep with graph coloring; adopts the IPC lagged friction model.
  Surveyed (not adopted) as inspiration for adding contact/friction to the
  variational integrator; its block-descent solver is largely redundant with the
  VI's O(n) recursive update, but its monotone-energy-descent and warmstart
  discipline are informative.

### `avbd-2025`

Chris Giles, Elie Diaz, and Cem Yuksel. "Augmented Vertex Block Descent." _ACM
Transactions on Graphics_ 44(4), Article 90, 2025. DOI:
[10.1145/3731195](https://doi.org/10.1145/3731195).

Related public resources:

- Project page:
  [graphics.cs.utah.edu/research/projects/avbd](https://graphics.cs.utah.edu/research/projects/avbd/)
- Paper PDF:
  [Augmented_VBD-SIGGRAPH25.pdf](https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf)
- Reference demo sources:
  [avbd-demo2d](https://github.com/savant117/avbd-demo2d) and
  [avbd-demo3d](https://github.com/savant117/avbd-demo3d)
- Online demos:
  [2D demo](https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo2d.html)
  and
  [3D demo](https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo3d.html)

```bib
@article{Giles2025AVBD,
  author = {Chris Giles and Elie Diaz and Cem Yuksel},
  title = {Augmented Vertex Block Descent},
  journal = {ACM Transactions on Graphics},
  volume = {44},
  number = {4},
  articleno = {90},
  year = {2025},
  doi = {10.1145/3731195}
}
```

- **Type:** paper · **Topic:** contact/integration · **Status:** in-progress · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-104`](https://github.com/dartsim/dart/blob/main/docs/plans/104-vertex-block-descent-solver.md)
  and its
  [AVBD paper gap audit](https://github.com/dartsim/dart/blob/main/docs/plans/104-vertex-block-descent-solver/avbd-paper-gap-audit.md);
  also informs the PLAN-084 contact-extension survey.
- **Notes:** Adopted as the hard-constraint extension of DART's VBD-family
  solver work: augmented-Lagrangian dual rows, bounded equality/inequality
  forces, hard contact/friction, finite-stiffness ramping for high stiffness
  ratios, quasi-Newton Hessian approximation, alpha-regularized error
  correction, warm-started dual/stiffness state, rigid-body 6-DOF blocks,
  articulated joints, fracture limits, and unified soft/rigid interactions.
  Full completion requires CPU and GPU implementations, all paper/site/video
  experiments and demos integrated into DART tests/benchmarks/`py-demos`, and
  benchmark packets proving DART beats the reference demo repositories and the
  published RTX-4090 paper numbers before any parity claim.

### `ando-2024-cubic-barrier`

Ryoichi Ando. "A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness."
_ACM Transactions on Graphics_ 43(6), 2024, 1-13. DOI:
[10.1145/3687908](https://doi.org/10.1145/3687908).

Related public resources:

- Software baseline:
  [st-tech/ppf-contact-solver](https://github.com/st-tech/ppf-contact-solver)
- SIGGRAPH Asia 2024 reference branch noted by the project:
  [project branch](https://github.com/st-tech/ppf-contact-solver/tree/sigasia-2024)
- Project articles:
  [hindsight](https://github.com/st-tech/ppf-contact-solver/blob/main/articles/hindsight.md),
  [elastic eigenvalue notes](https://github.com/st-tech/ppf-contact-solver/blob/main/articles/eigensys.md)

```bib
@article{Ando2024CubicBarrier,
  author = {Ryoichi Ando},
  title = {A Cubic Barrier with Elasticity-Inclusive Dynamic Stiffness},
  journal = {ACM Transactions on Graphics},
  volume = {43},
  number = {6},
  pages = {1--13},
  year = {2024},
  doi = {10.1145/3687908}
}
```

- **Type:** paper · **Topic:** contact/deformable/GPU · **Status:** planned · **Priority:** high · **Verdict:** evaluate
- **Where used:** [`PLAN-083 PPF intake`](https://github.com/dartsim/dart/blob/main/docs/plans/083-unified-newton-barrier-multibody/ppf-contact-solver-intake.md).
- **Notes:** Evaluate as a candidate internal contact and strain-limiting
  primitive for DART's unified Newton-barrier family. The method combines a
  cubic barrier, elasticity-inclusive dynamic stiffness, friction, and strain
  limiting for contact-rich shells, solids, and rods, with a GPU-first reference
  implementation in ZOZO's Contact Solver. DART should compare its formulas,
  ACCD precision caveats, strain-limit rows, solver diagnostics, and
  single-precision GPU behavior against the existing IPC, rigid IPC, VBD/AVBD,
  and deformable solver plans before adopting any shared primitive or public
  option.

### `modi-2024-simplicits`

Vismay Modi, Nicholas Sharp, Or Perel, Shinjiro Sueda, and David I. W. Levin.
"Simplicits: Mesh-Free, Geometry-Agnostic, Elastic Simulation." _ACM
Transactions on Graphics_ 43(4), Article 117, 2024. DOI:
[10.1145/3658184](https://doi.org/10.1145/3658184).

Related public resources:

- Project page:
  [research.nvidia.com/labs/toronto-ai/simplicits](https://research.nvidia.com/labs/toronto-ai/simplicits/)
- arXiv / PDF:
  [2407.09497](https://arxiv.org/abs/2407.09497),
  [Simplicits.pdf](https://research.nvidia.com/labs/toronto-ai/simplicits/assets/Simplicits.pdf)
- Supplemental parameter/time table:
  [SimplicitsSupplementalTable.pdf](https://research.nvidia.com/labs/toronto-ai/simplicits/assets/SimplicitsSupplementalTable.pdf)
- Videos:
  [Barely Rendered](https://www.youtube.com/watch?v=3Zameqg66aA) and
  [NVIDIA Research](https://www.youtube.com/watch?v=kxEpXkSv2cM)
- Kaolin notes and tutorial sources:
  [Kaolin Simplicits notes](https://kaolin.readthedocs.io/en/latest/notes/simplicits.html),
  [examples/tutorial/physics](https://github.com/NVIDIAGameWorks/kaolin/tree/master/examples/tutorial/physics)

```bib
@article{Modi2024Simplicits,
  author = {Vismay Modi and Nicholas Sharp and Or Perel and Shinjiro Sueda and David I. W. Levin},
  title = {Simplicits: Mesh-Free, Geometry-Agnostic, Elastic Simulation},
  journal = {ACM Transactions on Graphics},
  volume = {43},
  number = {4},
  articleno = {117},
  year = {2024},
  doi = {10.1145/3658184}
}
```

- **Type:** paper · **Topic:** integration/deformable · **Status:** planned · **Priority:** high · **Verdict:** adopt
- **Where used:** [`PLAN-105`](https://github.com/dartsim/dart/blob/main/docs/plans/105-simplicits-geometry-agnostic-elastic-solver.md).
- **Notes:** Adopted as a new DART-owned geometry-agnostic reduced elastic
  solver family. The method treats input geometry through occupancy/point
  sampling, trains implicit neural skinning weights as a reduced deformation
  basis from random perturbations and Monte Carlo elastic-energy sampling, then
  advances handle-space dynamics with implicit integration/Newton solves and
  maps deformations back to the original representation. DART should compare
  against the Kaolin `kaolin.physics.simplicits` implementation and the paper /
  supplemental table rows, but must not expose Simplicits, Kaolin, Warp, Torch,
  CUDA resources, solver registries, or ECS storage through public APIs. Full
  completion requires CPU and GPU paths, every paper/site/video/Kaolin demo in
  DART tests/benchmarks/`py-demos`/visual evidence, and benchmark packets
  proving DART beats the reference and paper numbers before any parity claim.

### `smith-2012-rosi`

Breannan Smith, Danny M. Kaufman, Etienne Vouga, Rasmus Tamstorf, and Eitan
Grinspun. "Reflections on Simultaneous Impact." _ACM Transactions on Graphics_,
31(4), Article 106, 2012. DOI:
[10.1145/2185520.2185602](https://doi.org/10.1145/2185520.2185602).

Related public resources:

- Project page: [cs.columbia.edu/cg/rosi](https://www.cs.columbia.edu/cg/rosi/)
- Paper PDF: [rosi.pdf](https://www.cs.columbia.edu/cg/rosi/rosi.pdf)
- Supplement:
  [rosi_supp.pdf](https://www.cs.columbia.edu/cg/rosi/rosi_supp.pdf)

- **Type:** paper · **Topic:** contact/impact · **Status:** referenced · **Priority:** medium · **Verdict:** baseline
- **Where used:**
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** Baseline for event-level simultaneous impact: generalized
  reflections, physical desiderata such as feasibility, symmetry, break-away,
  kinetic-energy behavior, and a restitution model aimed at avoiding inelastic
  collapse. DART should use it first as a benchmark/specification source, not as
  a public solver identity.

### `zhang-2015-qce`

Tianxiang Zhang, Sheng Li, Dinesh Manocha, Guoping Wang, and Hanqiu Sun.
"Quadratic Contact Energy Model for Multi-impact Simulation." _Computer Graphics
Forum_, 34(7), 133-144, 2015. DOI:
[10.1111/cgf.12752](https://doi.org/10.1111/cgf.12752).

Related public resource:

- Paper PDF:
  [graphics.pku.edu.cn/docs/20220703165309179406.pdf](https://www.graphics.pku.edu.cn/docs/20220703165309179406.pdf)

- **Type:** paper · **Topic:** contact/impact · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:**
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** Candidate multi-impact model that frames contact-point potential
  energy with a closed-form quadratic contact energy, then combines it with LCP
  handling for restitution and friction. Useful for a minimal internal baseline
  if PLAN-082 proves that finite-time rigid IPC or AVBD-style contact leaves a
  true restitution / wave-propagation gap.

### `vouga-2017-all-well`

Etienne Vouga, Breannan Smith, Danny M. Kaufman, Rasmus Tamstorf, and Eitan
Grinspun. "All's Well That Ends Well: Guaranteed Resolution of Simultaneous
Rigid Body Impact." _ACM Transactions on Graphics_, 36(4), Article 151, 2017.
DOI: [10.1145/3072959.3073689](https://doi.org/10.1145/3072959.3073689).

Related public resources:

- Paper PDF:
  [term-revised.pdf](https://www.cs.utexas.edu/~evouga/uploads/4/5/6/8/45689883/term-revised.pdf)
- Project page:
  [evouga/all's-well-that-ends-well](https://www.cs.utexas.edu/~evouga/allrsquos-well-that-ends-well-guaranteed-resolution-of-simultaneous-rigid-body-impact.html)

- **Type:** paper · **Topic:** contact/impact · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:**
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** The strongest termination-focused follow-up to generalized
  reflections and pairwise impact schemes. It belongs in DART as a go/no-go
  benchmark for finite termination, approximate energy behavior, drift, and
  break-away before any event-level impact operator is promoted.

### `halm-posa-2024-set-valued-impact`

Mathew Halm and Michael Posa. "Set-valued rigid-body dynamics for simultaneous,
inelastic, frictional impacts." _International Journal of Robotics Research_,
43(10), 2024. DOI:
[10.1177/02783649241236860](https://doi.org/10.1177/02783649241236860).

Related public resources:

- Journal page:
  [journals.sagepub.com/doi/10.1177/02783649241236860](https://journals.sagepub.com/doi/10.1177/02783649241236860)
- arXiv: [2103.15714](https://arxiv.org/abs/2103.15714)

- **Type:** paper · **Topic:** contact/impact · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:**
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** Robotics-specific newer reference that argues simultaneous
  frictional impacts should sometimes be represented as a set of plausible
  outcomes instead of a single heuristic ordering. Evaluate as an uncertainty /
  benchmark direction before exposing any set-valued public API.

### `lelidec-2024-contact-models`

Quentin Le Lidec, Wilson Jallet, Louis Montaut, Ivan Laptev, Cordelia Schmid,
and Justin Carpentier. "Contact Models in Robotics: a Comparative Analysis."
_IEEE Transactions on Robotics_, 40, 3716-3733, 2024. DOI:
[10.1109/TRO.2024.3434208](https://doi.org/10.1109/TRO.2024.3434208).

Related public resources:

- Project page:
  [di.ens.fr/willow/projects/contact_models_in_robotics](https://www.di.ens.fr/willow/projects/contact_models_in_robotics/)
- Paper PDF:
  [lelidec2024contacts.pdf](https://simple-robotics.github.io/publications/contact-models/static/paper/lelidec2024contacts.pdf)
- arXiv: [2304.06372](https://arxiv.org/abs/2304.06372)

- **Type:** paper · **Topic:** contact/survey · **Status:** referenced · **Priority:** medium · **Verdict:** reference
- **Where used:**
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** Survey and benchmark framing for robotics contact models, including
  Signorini/Coulomb/maximum-dissipation laws, simulator approximations, and
  physical/computational criteria. Use it to keep the PLAN-082 comparison matrix
  honest, not as a direct implementation target.

### `chen-2023-spb`

He Chen, Elie Diaz, and Cem Yuksel. "Shortest Path to Boundary for
Self-Intersecting Meshes." _ACM Transactions on Graphics_, 42(4), 2023. DOI:
[10.1145/3592136](https://doi.org/10.1145/3592136).

Related public resources:

- Project page:
  [graphics.cs.utah.edu/research/projects/shortest-path-to-boundary](https://graphics.cs.utah.edu/research/projects/shortest-path-to-boundary/)
- arXiv: [2305.09778](https://arxiv.org/abs/2305.09778)
- Supplemental:
  [sig23_shortest_path-supplemental.pdf](https://graphics.cs.utah.edu/research/projects/shortest-path-to-boundary/sig23_shortest_path-supplemental.pdf)
- Code:
  [AnkaChan/Shortest-Path-to-Boundary-for-Self-Intersecting-Meshes](https://github.com/AnkaChan/Shortest-Path-to-Boundary-for-Self-Intersecting-Meshes)
- Paper video:
  [youtube.com/watch?v=qRBHY9ntwbU](https://www.youtube.com/watch?v=qRBHY9ntwbU)

- **Type:** paper · **Topic:** contact/collision · **Status:** planned · **Priority:** high · **Verdict:** evaluate
- **Where used:**
  [`PLAN-081 SPB gap audit`](https://github.com/dartsim/dart/blob/main/docs/plans/081-deformable-implicit-barrier-solver/spb-gap-audit.md).
- **Notes:** Volumetric self-intersection recovery method for tetrahedral
  deformables: DCD finds preexisting intersections, SPB finds a valid shortest
  path from an internal point to the boundary, and CCD remains necessary for new
  crossings. It is not a codimensional cloth/strand method and should remain an
  internal recovery sidecar until DART has query tests, recovery constraints,
  hybrid CCD/DCD evidence, benchmark packets, and visual captures.

### `ogc-2025`

Anka He Chen, Jerry Hsu, Ziheng Liu, Miles Macklin, Yin Yang, and Cem Yuksel.
"Offset Geometric Contact." _ACM Transactions on Graphics_, 44(4), Article 160, 2025. DOI: [10.1145/3731205](https://doi.org/10.1145/3731205).

Related public resources:

- Project page:
  [graphics.cs.utah.edu/research/projects/ogc](https://graphics.cs.utah.edu/research/projects/ogc/)
- Paper PDF:
  [Offset_Geometric_Contact-SIGGRAPH2025.pdf](https://graphics.cs.utah.edu/research/projects/ogc/Offset_Geometric_Contact-SIGGRAPH2025.pdf)
- Code (Gaia):
  [github.com/AnkaChan/Gaia](https://github.com/AnkaChan/Gaia)
- Code (Newton):
  [newton/\_src/solvers/vbd](https://github.com/newton-physics/newton/tree/main/newton/_src/solvers/vbd)
- Paper video:
  [youtube.com/watch?v=xxyniqSLJik](https://www.youtube.com/watch?v=xxyniqSLJik)

- **Type:** paper · **Topic:** contact/collision · **Status:** planned · **Priority:** high · **Verdict:** evaluate
- **Where used:**
  [`PLAN-104 OGC gap audit`](https://github.com/dartsim/dart/blob/main/docs/plans/104-vertex-block-descent-solver/ogc-gap-audit.md),
  [`PLAN-082 simultaneous-impact intake`](https://github.com/dartsim/dart/blob/main/docs/plans/082-rigid-implicit-barrier-contact/simultaneous-impact-intake.md).
- **Notes:** Newer finite-time contact alternative for codimensional objects:
  offset geometry, local displacement bounds, and GPU-friendly local operations
  that avoid expensive CCD in its target setting. It is not a rigid-impact
  operator. Its implementation path is now tracked under PLAN-104 as a VBD
  contact sidecar with required source/code audit, CPU proof-of-contact,
  IPC/VBD comparison packets, and public-boundary review before promotion.

### `nakamura-1987-task-priority`

Nakamura, Y., Hanafusa, H., & Yoshikawa, T. "Task-Priority Based Redundancy
Control of Robot Manipulators." _International Journal of Robotics Research_,
6(2), 1987. DOI:
[10.1177/027836498700600201](https://doi.org/10.1177/027836498700600201).

- **Type:** paper · **Topic:** kinematics · **Status:** planned · **Priority:** medium · **Verdict:** adopt
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Grounds whole-body and task-priority IK: higher-priority task
  residuals constrain lower-priority objectives through redundancy/null spaces.
  Classic DART already has related `HierarchicalIK`, `CompositeIK`, and
  `WholeBodyIK` APIs; the DART 7 simulation world needs a DART-owned task model
  before carrying them forward.

### `buss-kim-2005-sdls`

Buss, S. R., & Kim, J.-S. "Selectively Damped Least Squares for Inverse
Kinematics." _Journal of Graphics Tools_, 10(3), 2005. DOI:
[10.1080/2151237X.2005.10129202](https://doi.org/10.1080/2151237X.2005.10129202).

- **Type:** paper · **Topic:** kinematics · **Status:** planned · **Priority:** medium · **Verdict:** evaluate
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Candidate singularity-robust extension to classic DART's Jacobian
  DLS baseline. Evaluate against DART's manifold state-space contract and
  near-singular benchmark scenes before adopting.

### `aristidou-lasenby-2011-fabrik`

Aristidou, A., & Lasenby, J. "FABRIK: A fast, iterative solver for the Inverse
Kinematics problem." _Graphical Models_, 73(5), 2011. DOI:
[10.1016/j.gmod.2011.05.003](https://doi.org/10.1016/j.gmod.2011.05.003).

- **Type:** paper · **Topic:** kinematics · **Status:** planned · **Priority:** medium · **Verdict:** evaluate
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Candidate fast heuristic or seed generator for simple chains and
  animation-style tasks. Must be benchmarked honestly for joint limits,
  orientation objectives, trees, whole-body targets, and closed chains before
  any broad support claim.

### `beeson-ames-2015-tracik`

Beeson, P., & Ames, B. "TRAC-IK: An open-source library for improved solving of
generic inverse kinematics." _IEEE-RAS International Conference on Humanoid
Robots_, 2015. DOI:
[10.1109/HUMANOIDS.2015.7363472](https://doi.org/10.1109/HUMANOIDS.2015.7363472).

- **Type:** paper · **Topic:** kinematics · **Status:** planned · **Priority:** medium · **Verdict:** baseline
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Baseline for generic constrained IK with joint limits and
  optimization-based fallback behavior. DART should not expose a TRAC-IK-named
  public solver; use method-family names and compare against the behavior.

### `starke-2020-bioik`

Starke, S. _Bio IK: A Memetic Evolutionary Algorithm for Generic Multi-Objective
Inverse Kinematics._ Dissertation, University of Hamburg, 2020.
<https://ediss.sub.uni-hamburg.de/handle/ediss/8703>

- **Type:** paper · **Topic:** kinematics · **Status:** planned · **Priority:** medium · **Verdict:** evaluate
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Candidate evolutionary/local-search family for multi-objective,
  multi-effector, whole-body IK and seed generation. Evaluate as a bounded
  fallback or pre-trial method, not as a replacement for checked local solvers.

### `rakita-2018-relaxedik`

Rakita, D., Mutlu, B., & Gleicher, M. "RelaxedIK: Real-time Synthesis of
Accurate and Feasible Robot Arm Motion." _Robotics: Science and Systems XIV_, 2018. DOI:
[10.15607/RSS.2018.XIV.043](https://doi.org/10.15607/RSS.2018.XIV.043).

- **Type:** paper · **Topic:** kinematics/motion-planning · **Status:** planned · **Priority:** high · **Verdict:** baseline
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Baseline for treating IK as real-time motion synthesis with
  competing objectives such as pose accuracy, continuity, collision avoidance,
  and singularity avoidance. This is the closest published fit for the
  "not just per-frame IK" requirement.

### `zucker-2013-chomp`

Zucker, M., Ratliff, N., Dragan, A., Pivtoraiko, M., Klingensmith, M., Dellin,
C., Bagnell, J. A., & Srinivasa, S. "CHOMP: Covariant Hamiltonian Optimization
for Motion Planning." _International Journal of Robotics Research_, 32(9), 2013.
<https://publications.ri.cmu.edu/chomp-covariant-hamiltonian-optimization-for-motion-planning>

- **Type:** paper · **Topic:** motion-planning · **Status:** referenced · **Priority:** medium · **Verdict:** baseline
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Baseline trajectory-optimization framing for smoothness,
  obstacle/collision cost, and local-minimum mitigation over a whole motion
  rather than independent pose IK frames.

### `mukadam-2018-gpmp2`

Mukadam, M., Dong, J., Yan, X., Dellaert, F., & Boots, B. "Continuous-time
Gaussian process motion planning via probabilistic inference." _International
Journal of Robotics Research_, 37(11), 2018. arXiv:
[1707.07383](https://arxiv.org/abs/1707.07383).

- **Type:** paper · **Topic:** motion-planning/statistics · **Status:** referenced · **Priority:** medium · **Verdict:** evaluate
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Candidate statistical trajectory prior for smooth continuous-time
  IK/motion solves. Evaluate after DART owns explicit state-space, collision
  query, and rollout APIs.

### `ames-2022-ikflow`

Ames, B., Morgan, J., & Konidaris, G. "IKFlow: Generating Diverse Inverse
Kinematics Solutions." _IEEE Robotics and Automation Letters_, 7(3), 2022.
arXiv: [2111.08933](https://arxiv.org/abs/2111.08933).

- **Type:** paper · **Topic:** kinematics/learning · **Status:** deferred · **Priority:** medium · **Verdict:** evaluate
- **Where used:** [`PLAN-120`](https://github.com/dartsim/dart/blob/main/docs/plans/120-inverse-kinematics-and-motion.md).
- **Notes:** Learned diverse-solution generator to evaluate as a proposal
  distribution for classical refinement, not as an unchecked correctness source
  or default solver. Requires datasets, model-version diagnostics, deterministic
  fallback, and cross-robot generalization evidence.

### `johnson-murphey-2009`

Johnson, E. R., & Murphey, T. D. "Scalable Variational Integrators for
Constrained Mechanical Systems in Generalized Coordinates." _IEEE Transactions
on Robotics_, 25(6), 2009.

- **Type:** paper · **Topic:** integration/dynamics · **Status:** referenced · **Priority:** high · **Verdict:** baseline
- **Where used:** the explicit predecessor `lee-vi-2016` benchmarks against
  ("SVI"); informs the PLAN-084 constraint roadmap (Phase B2).
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
- **Where used:** [`PLAN-084 contact-roadmap`](https://github.com/dartsim/dart/blob/main/docs/plans/084-variational-integrator-solver/contact-roadmap.md)
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
  DART 7 simulation world is deferred until a public C++ import owner exists; classic
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

Software references that informed the DART 7 API shape, terminology, and
algorithm-family choices. These are baselines/comparisons, not dependencies.

| ID                   | Engine                                    | Used for                                                                                | Status     | Verdict   |
| -------------------- | ----------------------------------------- | --------------------------------------------------------------------------------------- | ---------- | --------- |
| `drake`              | Drake (`MultibodyPlant`)                  | terminology, constraints, stepping API                                                  | referenced | baseline  |
| `pinocchio`          | Pinocchio (Stack-of-Tasks)                | terminology (Spherical/FreeFlyer), dynamics algos                                       | referenced | baseline  |
| `rbdl`               | RBDL                                      | terminology (FloatingBase/Helical), ABA/RNEA/CRBA                                       | referenced | baseline  |
| `mujoco`             | MuJoCo / MJX                              | stepping, equality constraints, state vocabulary                                        | referenced | baseline  |
| `physx-isaac`        | NVIDIA PhysX / Isaac Sim / Isaac Lab      | articulation concept, closed-loop rigging                                               | referenced | reference |
| `newton`             | NVIDIA Newton (Warp)                      | model/solver split, GPU direction                                                       | referenced | reference |
| `genesis`            | Genesis                                   | entity/morph model, batched sim                                                         | referenced | reference |
| `bullet`             | Bullet / PyBullet                         | facade-over-engine pattern, `btMultiBody`                                               | referenced | reference |
| `gazebo`             | Gazebo / gz-physics / SDFormat            | downstream integration, kinematic loops                                                 | referenced | baseline  |
| `gaia`               | Gaia (VBD research framework)             | VBD correctness/performance baseline (`chen-2024-vbd`)                                  | referenced | baseline  |
| `tinyvbd`            | TinyVBD (minimal VBD reference)           | VBD algorithm reference (`chen-2024-vbd`)                                               | referenced | baseline  |
| `kaolin-simplicits`  | NVIDIA Kaolin `kaolin.physics.simplicits` | Simplicits method/API/code baseline (`modi-2024-simplicits`)                            | referenced | baseline  |
| `ppf-contact-solver` | ZOZO's Contact Solver                     | GPU shell/solid/rod contact stack and API/platform baseline (`ando-2024-cubic-barrier`) | referenced | baseline  |
| `dojo`               | Dojo.jl                                   | Dojo-style differentiable rigid-body solver evaluation (`howell-2022-dojo`)             | referenced | evaluate  |

### Notes on engine verdicts

- **`drake`, `pinocchio`, `rbdl`, `mujoco`, `gazebo` — baseline:** mature
  robotics-dynamics references the DART 7 simulation world should be benchmarked and
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
- **`kaolin-simplicits` — baseline:** NVIDIA Kaolin's
  `kaolin.physics.simplicits` module, notes, and tutorial notebooks provide
  the public Simplicits method/API/code baseline for PLAN-105. DART should use
  them for source audit, tests, demos, and benchmark comparison, but must not
  make Kaolin, Warp, Torch, or notebook infrastructure a runtime dependency.
- **`ppf-contact-solver` — baseline:** ZOZO's Contact Solver is an
  independent Apache-2.0 software stack around `ando-2024-cubic-barrier`, with
  Python frontends, solver logs, Rust/C++/CUDA internals, Docker/Windows
  deployment, Blender/Jupyter workflows, and large contact-rich shell/solid/rod
  examples. DART should use it to audit API, diagnostics, examples, precision,
  and GPU performance lessons for PLAN-083, but must not vendor it or expose
  PPF, ZOZO, server, CUDA, or frontend-specific concepts through public APIs.
- **`dojo` — evaluate:** a Julia implementation and paper/site reference for a
  differentiable maximal-coordinate variational hard-contact NCP/IPM solver.
  PLAN-110 uses it as method evidence for a possible second solver family, but
  DART must first complete the Dojo gap audit and de-risking spike.

Design-doc links for these comparisons live in
[`simulation_cpp_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_cpp_api.md)
and
[`simulation_python_api.md`](https://github.com/dartsim/dart/blob/main/docs/design/simulation_python_api.md).

## Terminology Grounding

The DART 7 simulation world's naming decisions are grounded in the references above
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
entry's `status`/`verdict` when the DART 7 simulation world's relationship to a
reference changes (for example, when a `planned` algorithm becomes
`implemented`).
