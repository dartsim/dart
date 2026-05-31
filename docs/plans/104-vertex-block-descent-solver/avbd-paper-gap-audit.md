# AVBD Paper / Reference Gap Audit

This audit grounds PLAN-104 against _Augmented Vertex Block Descent_ and the
public 2D/3D reference demo repositories. It records the full implementation
target for DART's AVBD work. It is a planning artifact; it does not itself
claim implemented behavior.

## Sources

- Chris Giles, Elie Diaz, and Cem Yuksel. "Augmented Vertex Block Descent."
  _ACM Transactions on Graphics_ 44(4), Article 90, 2025.
  DOI: <https://doi.org/10.1145/3731195>.
  Project page:
  <https://graphics.cs.utah.edu/research/projects/avbd/>.
  Paper PDF:
  <https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf>.
- Online demos:
  <https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo2d.html> and
  <https://graphics.cs.utah.edu/research/projects/avbd/avbd_demo3d.html>.
- Reference demo repositories:
  - `avbd-demo2d`: <https://github.com/savant117/avbd-demo2d>,
    inspected at `74699a11f8586d3ac34474c92b1ef8feb5f587de`.
  - `avbd-demo3d`: <https://github.com/savant117/avbd-demo3d>,
    inspected at `7701bd427d55ca5d03ea1fdf331912ded9169f4b`.

The paper positions AVBD as a hybrid primal-dual extension of VBD that adds a
dual update after each primal VBD sweep, supports hard constraints through an
augmented Lagrangian, improves high-stiffness-ratio convergence by ramping
finite forces, and evaluates rigid stacking/friction, articulated chains,
attachments, breakage, soft/rigid coupling, and large GPU scenes.

## Full-Scope Rule

AVBD is not complete when DART has only a small contact experiment or a
paper-inspired row update. For this paper, "implemented" means:

- all algorithms and features from the paper, project page, videos, and 2D/3D
  demo sources are implemented in DART-owned code;
- CPU and GPU implementations both exist for the solver, constraints, collision
  candidate generation, demos, and benchmark paths where the paper/reference
  exercises them;
- every paper/site/video/demo experiment is represented by a DART test,
  benchmark, and `py-demos` scene or a documented equivalent;
- DART benchmark JSON proves DART beats the reference demo repositories and the
  paper's reported CPU/GPU numbers for every claimed case; and
- API and pipeline refactors are allowed when they make the DART 7/8
  experimental architecture cleaner and more scalable.

## Verified Reference Details

- **Hard constraints:** Each scalar row carries finite stiffness `k` and dual
  value `lambda`. A hard row applies `clamp(k * C(x) + lambda, lower, upper)` as
  the force magnitude, then writes that value back to `lambda`.
- **Stiffness growth:** Hard rows increase `k` by `beta * abs(C)` only while the
  force is not clamped. Finite-stiffness forces use the same progressive ramp
  but saturate at the material stiffness and do not carry a Lagrange multiplier.
- **Bounds and friction:** Contacts and joint limits are inequality rows. The
  normal row has a non-negative lower bound. Friction rows are bounded by the
  lagged normal force and coefficient of friction; static/dynamic friction
  switching is based on whether the previous tangential dual lies inside the
  cone.
- **Error regularization:** Constraint values are regularized as
  `C(x) = C*(x) - alpha * C*(x_t)` so pre-existing hard-constraint error is
  corrected gradually instead of injecting a large impulse-like momentum spike.
- **Warm starting:** `lambda` and `k` persist across frames and decay by
  `alpha * gamma` and `gamma` respectively, with `k` never below `k_start`.
- **Hessian approximation:** Hard constraints use a quasi-Newton Hessian: the
  geometric stiffness term is replaced by a diagonal matrix built from column
  norms so each local block remains SPD.
- **Parallelization:** AVBD keeps VBD's vertex/body coloring, then adds one
  parallel dual/stiffness update pass after each primal iteration.
- **Rigid bodies:** Rigid rows use a 6-DOF block with linear and angular terms;
  the paper represents rotation by unit quaternions and maps rotational
  differences/updates through tangent-space angular vectors.
- **Reference demo sources:** `avbd-demo2d` implements boxes, joints, springs,
  motors, contact manifolds, fracture thresholds, and 19 scenes. `avbd-demo3d`
  implements 6-DOF rigid boxes, spring/joint/contact rows, bridge/breakable
  examples, and 14 scenes. Both demos state they are clarity references, not
  heavily optimized implementations.

## DART Current State

PLAN-104 already has a DART-owned VBD foundation: CPU block kernels, vertex
coloring, colored Gauss-Seidel sweeps, Stable Neo-Hookean/FEM tetrahedra,
Chebyshev/Rayleigh acceleration, opt-in World wiring, static ground/sphere/box
penalty contact, lagged surface self-contact, CUDA mass-spring/tet rollouts,
benchmarks, and first py-demos.

The first AVBD implementation slices add:

- a tested internal scalar-row utility for Eq. 11, Eq. 12, Eq. 16, Eq. 18, and
  Eq. 19: `detail/deformable_vbd/avbd_constraint.hpp` plus
  `test_avbd_constraint.cpp`;
- deterministic scalar-row descriptors, keys, role/axis separation, and
  warm-start inventory cache:
  `detail/deformable_vbd/avbd_row_inventory.hpp`;
- an active half-space contact-normal kernel slice:
  `addAvbdHalfSpaceContactNormal`, `updateAvbdHalfSpaceContactNormalRow`, and
  `blockDescentMassSpringAvbdGround`, with focused `VbdContact.Avbd*` tests;
- an active scalar hard point-attachment kernel slice:
  `AvbdPointAttachmentRow`, `addAvbdPointAttachment`,
  `updateAvbdPointAttachmentRow`, and
  `blockDescentMassSpringAvbdAttachments`, with focused `VbdAttachment.*`
  tests; and
- a narrow internal World VBD integration path for active static half-space
  contact-normal rows in supported serial, frictionless mass-spring scenes,
  keyed by body/entity, vertex, and static obstacle feature IDs, with
  `VbdWorldSolver.AvbdContactNormalRowsHardenGroundContact` coverage; and
- a narrow internal World VBD integration path for pinned or scripted hard
  point-attachment rows in supported serial, frictionless mass-spring scenes,
  keyed by body/entity, vertex, and axis, with per-step internal row counters
  and `VbdWorldSolver.AvbdAttachmentRowsHoldPinnedNode` coverage; and
- a narrow internal World VBD integration path for progressive finite-stiffness
  spring rows in supported serial, contact-free, frictionless mass-spring
  scenes, keyed by body/entity and spring index, with focused
  `VbdFiniteStiffness.*` tests and
  `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenSpringChain` coverage; and
- a finite-stiffness tetrahedral material row slice:
  `AvbdTetMaterialFiniteStiffnessRow`, strain-norm row error, scaled Lamé
  material stamping, and `blockDescentTetMeshAvbdFiniteStiffness`, with focused
  `VbdFiniteStiffness.*` tests plus a narrow internal World VBD integration
  path for supported serial, frictionless pure-tet scenes, separate tet-row
  diagnostics, coexistence with the existing lagged VBD self-contact penalty,
  and `VbdWorldSolver.AvbdFiniteStiffnessRowsHardenTetrahedralMaterial`
  coverage; and
- a combined serial mass-spring AVBD row solve for those contact-normal,
  attachment, and finite-stiffness spring rows, with
  `VbdWorldSolver.AvbdRowsCombineContactAttachmentAndFiniteStiffness`
  coverage; and
- a bounded half-space friction-tangent row primitive:
  `AvbdHalfSpaceFrictionRow`, `avbdFrictionTangentBounds`,
  `addAvbdHalfSpaceFrictionTangent`, and
  `updateAvbdHalfSpaceFrictionTangentRow`, with optional participation in the
  serial mass-spring AVBD row driver and focused `VbdContact.Avbd*` coverage;
  World-level friction row generation, static/dynamic switching, and full
  friction-cone persistence are still missing; and
- explicit World fallback coverage so unsupported mixed spring-plus-tet,
  mass-spring self-contact, frictional World scenes, Chebyshev,
  Rayleigh-damped, parallel, and unsupported-row requests keep using the
  existing VBD path without reporting partial AVBD row counters.

Those are foundation pieces only. They are not a full World AVBD solver, not a
full hard-contact/friction solver, and not CPU/GPU parity.

## Component Gap Matrix

| AVBD component                                                                                     | DART status                                                                                                                                                                                                             | Closing phase |
| -------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| Scalar hard-row dual state, clamping, warm start, alpha regularization, finite-stiffness ramp      | First internal utility and unit tests started                                                                                                                                                                           | A0            |
| Constraint row inventory and storage for contacts, joints, attachments, friction, motors, fracture | Deterministic scalar row inventory started; active static contact-normal, hard-attachment, finite-spring World rows wired; bounded friction-tangent primitive started                                                   | A1            |
| CPU AVBD primal/dual loop over DART VBD vertices and rigid bodies                                  | Narrow serial mass-spring contact-normal, hard-attachment, finite-spring, and optional friction-tangent rows now combine; pure-tet finite-material rows wired; unsupported envelopes fall back; rigid/full rows missing | A2-A4         |
| 6-DOF rigid block assembly with quaternion/tangent angular updates                                 | Missing in VBD path; DART has separate multibody foundations                                                                                                                                                            | A3            |
| Inequality/contact/friction row bounds with static/dynamic friction switching                      | Contact-normal lower-bound slice and bounded friction-tangent primitive started; static/dynamic switching and World friction generation missing                                                                         | A4            |
| Joint, attachment, motor, fracture, and breakable hard constraints                                 | Scalar hard point-attachment kernel and narrow World attachment wiring started; joints, motors, and fracture missing                                                                                                    | A5            |
| Unified rigid/soft interactions and cloth/articulated-body coupling                                | Missing                                                                                                                                                                                                                 | A6            |
| Collision candidate generation, contact persistence, and row warm-start mapping                    | Static half-space row keys started; manifolds, dynamic contacts, and broad row generation missing                                                                                                                       | A7            |
| Hessian approximation for hard constraints and stiffness-rescaling friction                        | Finite-stiffness spring and pure-tet material ramps started; hard-constraint quasi-Newton Hessian and friction stiffness rescaling missing                                                                              | A8            |
| CPU parallel color sweeps plus deterministic dual update pass                                      | Serial contact-normal dual pass started; parallel dual/update scheduling missing                                                                                                                                        | A9            |
| CUDA/GPU AVBD backend for all row families and scene corpus                                        | VBD CUDA mass-spring/tet rollout only                                                                                                                                                                                   | G1-G5         |
| DART-owned reproductions of 2D/3D demos and paper/video scenes                                     | Missing beyond existing VBD demos                                                                                                                                                                                       | D1-D6         |
| Benchmark packets beating reference demos and paper numbers on CPU/GPU                             | Missing                                                                                                                                                                                                                 | P1-P4         |

## Implementation Workstreams

1. **A0 - scalar row foundation:** Land the internal AVBD scalar-row utility,
   focused tests for hard rows, finite-stiffness rows, bounds, warm starting,
   and alpha regularization.
2. **A1 - row model and inventory:** Add internal row descriptors for hard
   equality, bounded inequality, finite stiffness, contact normal, friction
   tangent, joint, motor, and fracture rows. Keep public API method names
   DART-owned and backend-neutral.
3. **A2 - CPU deformable AVBD:** Integrate scalar rows into the existing VBD
   deformable block descent for hard attachments, finite-stiffness ramped
   springs/tets, ground/static obstacle contact, and self-contact.
4. **A3 - CPU rigid AVBD:** Add 6-DOF rigid block assembly, tangent angular
   update, box inertia, rigid contact manifolds, and row Jacobians.
5. **A4 - friction and bounds:** Implement contact normal bounds, tangential
   friction cone bounds, static/dynamic friction switching, and stiffness
   rescaling/approximate Hessian options.
6. **A5 - joints, motors, and fracture:** Implement ball/revolute/limited DOF
   joints, motor torque rows, attachment rows, maximum-force fracture, and
   breakage persistence.
7. **A6 - unified rigid/soft coupling:** Couple rigid rows and deformable VBD
   rows in one AVBD solve so soft cloth/elastic bodies interact with articulated
   rigid chains and stacks.
8. **A7 - candidate generation and persistence:** Build broad-phase/narrow-phase
   row generation with persistent row IDs for warm-started lambda/stiffness,
   including CPU and GPU candidate paths.
9. **A8 - Hessian and stability policy:** Implement the quasi-Newton diagonal
   geometric-stiffness approximation, post-stabilization option, finite maximum
   bounds for conflicting constraints, and diagnostics for impossible row sets.
10. **A9 - CPU performance:** Add SoA layouts, task-parallel color sweeps,
    deterministic reductions, benchmark JSON, and direct comparisons against
    `avbd-demo2d`/`avbd-demo3d`.
11. **G1-G5 - GPU parity:** Port every row family, candidate-generation path,
    dual update, color update, and benchmark scene to the CUDA/private compute
    boundary; record same-GPU packets for the published RTX-4090 cases.
12. **D1-D6 - demos and experiments:** Reproduce every source demo, online demo,
    paper figure, table, parameter sweep, and supplemental/video scenario in
    DART `py-demos`, tests, benchmarks, and visual capture gates.

## Required Experiment And Demo Corpus

Minimum corpus before any AVBD parity claim:

- 2D demo scenes: empty, ground, dynamic friction, static friction, pyramid,
  card tower, rope, heavy rope, hanging rope, spring, spring ratio, stack, stack
  ratio, rod, soft body, joint grid, net, motor, and fracture.
- 3D demo scenes: empty, ground, dynamic friction, static friction, pyramid,
  rope, heavy rope, spring, spring ratio, stack, stack ratio, soft body, bridge,
  and breakable.
- Paper figures: high-stiffness ratio chains, flag/pole attachment, card tower,
  50-body pendulum with 50,000:1 mass ratio, two-heavy-ball chain, box stack,
  friction coefficient comparison, chain mail impact, breakable wall, large
  cloth plus articulated-rigid-body coupling, and parameter sweeps for `beta`,
  `alpha`, `gamma`, and iteration count.
- Website/video headline scenes: rigid stacking/piling with friction and the
  large block-pile/sphere-smash GPU scenes.

Each promoted scene needs a correctness invariant, a benchmark profile, and a
headless visual artifact or py-demo smoke where visual behavior matters.

## Performance Targets

Reference targets are not complete until measured in DART benchmark JSON:

- Paper Table 1, RTX 4090 GPU:
  - Figure 1, 110,000 bodies: AVBD 4 iterations at 3.5 ms/frame.
  - Figure 3, 510,000 bodies: AVBD 3 iterations at 10.3 ms/frame.
  - Baseline rows for sequential impulse, XPBD, and VBD must be reproduced or
    documented with audited equivalent setups before claiming relative speedup.
- Paper Figure 14: 35,000 rigid bodies, 72,000 joints, cloth with 10,000
  vertices and 20,000 triangles, 10 AVBD iterations, 16 ms/frame including
  collision detection.
- Reference demo CPU baselines: DART CPU must beat native `avbd-demo2d` and
  `avbd-demo3d` on matched scene counts, timestep, iteration count, and compiler
  mode before a CPU win is claimed.
- DART GPU claims must use the same GPU class when comparing to published
  paper numbers, and must record fallback hardware separately.

## DART-Specific Constraints

- Keep public selection and config DART-owned and backend-neutral; avoid
  exposing reference project names, row-storage internals, ECS storage, CUDA
  types, or solver registries as public API.
- Do not vendor or link the reference demo repositories at runtime.
- Use existing DART foundations first: experimental `World`, deformable VBD
  kernels, deformable/rigid contact kernels, multibody dynamics, compute
  resource metadata, CUDA private boundary, benchmark JSON, and py-demos.
- Because DART 7 is an API-breaking clean-up release and this solver family is
  experimental, prefer long-term scalable API and pipeline refactors over
  backward-compatible compromises.
- Every slice must keep `pixi run lint`, focused build/tests, relevant
  benchmark or visual evidence, and `check-api-boundaries` green before it is
  described as complete.
