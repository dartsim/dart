# IPC Paper Gap Audit For PLAN-081

This sidecar records the current gap between DART's first deformable slice and
the complete Incremental Potential Contact (IPC) method family. It is intended
as the next-session handoff for implementing the remaining IPC-class solver,
examples, tests, benchmarks, and material/property coverage.

## Source Snapshot

- Paper: Li et al., "Incremental Potential Contact: Intersection- and
  Inversion-free, Large-Deformation Dynamics," ACM Transactions on Graphics
  39(4), article 49, 2020. DOI: <https://doi.org/10.1145/3386569.3392425>.
  Audited from the public PDF at <https://cims.nyu.edu/gcl/papers/2020-IPC.pdf>.
- Upstream reference implementation:
  <https://github.com/ipc-sim/IPC>, audited at commit
  `573d2c7e04104d3f9baf526bdaee7745891a571a`.
- Modern IPC toolkit reference:
  <https://github.com/ipc-sim/ipc-toolkit>, audited at commit
  `a510337396c83ddcc22d0f98220bedb9bb967eb3`.
- DART branch state audited: PR #2711 at
  `73341328b7c7b4b0e71a7a31d9f36fd29f8f9729`, before this
  documentation-only update.
- Dependency note: upstream IPC and `ipc-toolkit` remain references and test
  baselines. PLAN-081 continues to require DART-owned implementation, DART API
  names, and no upstream runtime dependency.

## Audit Method And Scope

This audit is intentionally stricter than the first-slice acceptance criteria.
It treats every paper section, Algorithm 1 phase, upstream input scene, toolkit
test family, benchmark statistic, material option, and visualization path as a
future DART obligation unless the row explicitly says it is out of scope for a
DART-owned solver.

"Line-by-line" here means coverage by paper obligation, not copying paper text.
Agents implementing the next session should keep the paper PDF and upstream
`IPC` checkout open beside this file and retire rows only after DART has
matching code, tests, examples, benchmark/profiling output, and visual evidence.
The first PR's point-mass spring net is useful scaffolding, but it must not be
described as full IPC, mesh IPC, IPC contact, or IPC friction.

The complete upstream `IPC/input` corpus currently contains 480 files. The
runnable scene inventory is the tracked `.txt` path subset: paper examples,
time-step/video variants, tutorial examples, other validation/stress scenes,
comparison benchmarks, and failure cases. Meshes, segment sequences, and
supplemental application files are required assets for those scenes, not
optional coverage. The authoritative scene-path owner is
[`ipc_scene_corpus_manifest.json`](ipc_scene_corpus_manifest.json), generated
from `git ls-tree` against upstream commit
`573d2c7e04104d3f9baf526bdaee7745891a571a`, so it includes 144 regular scene
files plus 10 tracked symlink aliases.

## Current DART Slice

DART currently has an experimental point-mass deformable slice, not a full IPC
solver:

- `DeformableBody` stores world-space nodes, velocities, masses, fixed nodes,
  and distance-spring edges.
- `DeformableDynamicsStage` is in the default `World::step()` pipeline.
- The step objective is implicit-Euler inertia plus edge-spring energy and a
  C2-style analytic vertical ground barrier for explicitly opted-in static
  boxes/spheres. Gravity and damping are folded into the inertial target.
- The internal solve is deterministic steepest-descent with Armijo-style
  backtracking and feasibility rejection against the analytic ground barrier.
- Public GUI rendering now has reusable deformable surface descriptors through
  `dart::gui::makeDeformableSurfaceRenderable()`. This is render-only support;
  it does not imply mesh-backed simulation state, triangle contact, or FEM
  material support.
- Tests cover public facade validation, fixed nodes, analytic one-spring and
  free-particle ground truth, static-ground opt-in/opt-out, determinism,
  serialization, and GUI extraction.
- Benchmarks cover spring-grid stepping and stage counters.

Everything below remains either partially implemented or missing.

## Paper Section Coverage

| Paper area                                  | IPC requirement                                                                                                                                                       | DART status                                                                                                          | Gap                                                                                                                                        |
| ------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------ |
| Abstract / intro                            | Robust nonlinear elastodynamics with intersection- and inversion-free trajectories across broad time steps, materials, velocities, boundary conditions, and contacts. | Partial architecture only. Current guarantees apply only to point masses and analytic static-ground height barriers. | Need volumetric mesh state, FE energy, mesh-surface contact constraints, inversion-aware energy/filtering, and end-to-end proof tests.     |
| Contributions                               | Unsigned-distance contact, smooth local barriers, CCD line search, lagged variational friction, and benchmark suite.                                                  | Only local analytic ground barrier and no mesh contact/friction suite.                                               | Implement the full contribution set before claiming IPC-class parity.                                                                      |
| Section 2 contact model                     | Admissible trajectories defined by primitive-pair unsigned distances over points, edges, triangles, and codimensional obstacles.                                      | No surface mesh collision constraint set for deformables.                                                            | Add surface extraction, primitive adjacency filters, PT/EE pair generation, and codimensional collision objects.                           |
| Section 2 time discretization               | Incremental potential for implicit Euler and implicit Newmark over nodal FE state.                                                                                    | Implicit Euler-like point-mass objective only.                                                                       | Add volumetric FE mass/state, Newmark option, and per-step potential terms matching material models.                                       |
| Section 2.1 accuracy                        | Expose/track dynamics accuracy `epsilon_d`, geometric accuracy `d_hat`, stiction accuracy `epsilon_v`, positivity, complementarity, and injectivity.                  | No public accuracy controls beyond fixed internal tolerances; no complementarity/contact-force reporting.            | Add DART-owned options and diagnostics for solver residual, clearance, force positivity, and injectivity status.                           |
| Section 3 related work                      | Avoid local signed-distance proxies and SQP drift; compare against prior methods.                                                                                     | Current docs mention IPC but do not yet carry comparison harnesses.                                                  | Add benchmark baselines and failure-mode regression tests for SQP-style gaps where possible.                                               |
| Section 4 barrier mechanics                 | Barrier-augmented incremental potential over culled primitive pair set.                                                                                               | Analytic node-vs-ground barrier only.                                                                                | Add PT/EE unsigned-distance barriers, stiffness adaptation, and exact active-set energy assembly.                                          |
| Section 4.2 smooth barriers                 | Smoothly clamped log barrier with zero support beyond `d_hat`, evaluated on all close constraints.                                                                    | Static-ground barrier uses a fixed internal activation distance and scale.                                           | Generalize barrier function, expose `d_hat` policy, and validate derivatives against finite differences.                                   |
| Section 4.3 Newton solver                   | Projected Newton with per-stencil PSD Hessian projection and SPD assembled solve.                                                                                     | Steepest descent with diagonal mass scaling; no Hessian or sparse linear solve.                                      | Implement Hessian assembly, PSD projection, sparse solve abstraction, and robust fallback policy.                                          |
| Section 4.4 line search                     | CCD upper bound before backtracking, combined with inversion-aware filtering for barrier elasticity.                                                                  | Feasibility check only against current analytic ground height.                                                       | Use conservative PT/EE CCD for every Newton step and add inversion-aware line-search filters for volumetric elements.                      |
| Section 4.5 solution accuracy               | Momentum balance, contact force positivity, complementarity, admissibility, and injectivity checks.                                                                   | Tests cover simple analytic dynamics and clearance for static ground only.                                           | Add solver diagnostics and tests for each IPC accuracy condition.                                                                          |
| Section 4.6 constraint set/CCD acceleration | Spatial hash/distance filtering plus conservative CFL-style CCD culling.                                                                                              | No deformable broad phase or candidate culling.                                                                      | Build reusable deformable broad phase, candidate cache, and CCD-culling benchmarks.                                                        |
| Section 5 friction                          | Coulomb/MDP friction as a smoothed, lagged dissipative potential with tangent bases and contact-force lagging.                                                        | No deformable contact friction.                                                                                      | Add `epsilon_v`, friction coefficient/material combination, tangent basis, lagged iterations, force Jacobian, and convergence diagnostics. |
| Section 6 distance computation              | PT/EE closest-distance cases, derivatives, tangent bases, and edge-edge mollifier for near-parallel cases.                                                            | CCD kernels exist from the native collision work, but PLAN-081 does not yet use distance derivatives or mollifiers.  | Implement distance value/gradient/Hessian stencils, derivative tests, edge-edge mollifier tests, and tangent-basis tests.                  |
| Section 7 evaluation                        | Unit tests, stress tests, friction tests, scaling, performance, accuracy, and exact-CCD admissibility checks.                                                         | Current suite is a small spring-net slice.                                                                           | Port the upstream example/test matrix below into DART-native tests, examples, and benchmarks.                                              |
| Section 8 comparisons                       | Compare against graphics, engineering, and SQP-type methods.                                                                                                          | DART has native CCD comparison infrastructure, but no IPC solver comparison corpus.                                  | Add DART-native comparison harnesses; external commercial-code comparisons should be documented as manual/non-CI baselines.                |
| Section 9 conclusion/future                 | Further Newton methods, faster exact CCD, higher-order elements, and better friction convergence.                                                                     | All are future work.                                                                                                 | Keep these out of public API promises until backed by tests and benchmarks.                                                                |

## Detailed Paper Obligation Checklist

Use this as the next-session paper walk. A row is complete only when the DART
implementation has a focused test, a representative example or benchmark when
the paper uses one, and diagnostics sufficient to debug regressions.

| Paper obligation          | Required DART artifact                                                                                                                                      | Current DART state                                                                                  | Completion evidence required before closing the gap                                                                                                |
| ------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- |
| FE state variables        | Mesh-backed deformable body with nodal positions, velocities, accelerations, mass matrix, rest configuration, surface extraction, and element connectivity. | Point-mass nodes, velocities, masses, and spring edges only.                                        | Tetrahedral and surface-mesh construction tests; serialization; invalid mesh/material rejection; surface extraction adjacency tests.               |
| Incremental potential     | Backward Euler incremental potential over mass, inertia, elastic energy, contact barriers, friction potential, BC work, and optional Newmark terms.         | Point-mass implicit-Euler-like inertia target, edge springs, and analytic ground barrier.           | Manufactured single-DOF and mesh FE energy/gradient/Hessian tests; BE/NM comparison tests; energy decomposition diagnostics.                       |
| Admissible trajectory set | Signed-free admissibility over primitive-pair unsigned distances, plus element inversion filters for invertible elasticity.                                 | Analytic z-up clearance against opted-in box/sphere top surfaces.                                   | Post-step no-intersection checks, exact/conservative CCD validation, inversion-filter tests, and failure-case fixtures.                            |
| Accuracy knobs            | DART-owned equivalents for dynamics accuracy, geometric activation distance, stiction velocity threshold, CCD tolerance, and convergence mode.              | Internal fixed tolerances only.                                                                     | Public or semi-public options where appropriate; per-step stats for residuals, clearances, active contacts, friction iterations, and tolerances.   |
| Smooth barrier            | C2 clamped log barrier with zero support beyond the activation distance, including value, gradient, Hessian, and PSD projection behavior.                   | Analytic ground C2-style energy/gradient only, fixed scale and activation distance.                 | Finite-difference derivative tests, Hessian PSD-projection tests, support cutoff tests, and barrier force magnitude tests.                         |
| Barrier stiffness policy  | Adaptive barrier stiffness/homotopy policy with minimum stiffness scale and force-balance diagnostics.                                                      | Fixed internal scale.                                                                               | Unit tests for stiffness increase/decrease, complementarity/positivity metrics, and reproducible stats under large stiffness/mass ratios.          |
| Projected Newton solve    | Sparse assembled Hessian, local PSD projection, direct/iterative linear solver abstraction, residual stopping, and robust fallback.                         | Diagonal scaled steepest descent.                                                                   | Solver microbenchmarks, residual tests, fallback tests, matrix pattern/cache tests, and deterministic solve tests.                                 |
| Constraint set build      | Primitive-pair candidate set for point-triangle, edge-edge, point-edge/point-point codimensional cases, adjacency exclusions, and broad-phase culling.      | No deformable primitive-pair candidate set.                                                         | Brute-force vs broad-phase tests, candidate normal tests, adjacency filter tests, safe-distance tests, and active-set rebuild counters.            |
| CCD line search           | Conservative feasible step upper bound for every Newton direction, then backtracking for energy decrease with recomputed constraints.                       | Feasibility rejection only for static ground height.                                                | PT/EE CCD line-search tests, exact CCD post-step audits, line-search rejection counters, and high-speed impact examples.                           |
| Inversion-aware filtering | Energy-model filters that prevent element inversion during line search for invertible elasticity.                                                           | No volumetric elements.                                                                             | Neo-Hookean/fixed-corotational inversion tests and stress examples with failed candidates counted.                                                 |
| Distance computation      | Point-triangle and edge-edge closest-feature classification, values, gradients, Hessians, tangent bases, and edge-edge mollifier.                           | Native primitive CCD exists elsewhere; PLAN-081 does not use distance derivatives or tangent bases. | Distance derivative finite-difference tests; edge-edge mollifier tests; tangent-basis orthonormality/continuity tests.                             |
| Friction potential        | Lagged smoothed Coulomb friction with tangent bases, lagged normal forces, configurable iterations, anisotropic option if adopted, and convergence checks.  | No deformable friction.                                                                             | Slope threshold tests, stick-slip oscillation test, friction potential/Jacobian derivative tests, lagged-iteration convergence diagnostics.        |
| Codimensional contact     | Points, segments, triangle soups, half-spaces, mesh collision objects, moving mesh sequences, and self-contact/self-friction policy.                        | Static box/sphere ground top surfaces only.                                                         | Codimensional unit tests, moving obstacle examples, self-contact examples, mesh-sequence examples, and collision-object serialization.             |
| Boundary conditions       | Dirichlet and Neumann boxes, time ranges, scripted linear/angular motion, initial linear/angular velocity, and restartable state.                           | Fixed nodes and node velocities only.                                                               | DBC/NBC time-range tests, scripted motion tests, restart round-trip tests, and tutorial examples.                                                  |
| Output and diagnostics    | Per-step output/restart, mesh/state dumps, contact counts, timing, memory, Newton/friction iterations, accuracy metrics, and visual replay data.            | Binary world serialization plus spring-grid benchmark counters.                                     | JSON/CSV stats schema, benchmark JSON, `/usr/bin/time -v` or equivalent profiling, replay/visual capture scripts, and docs for interpreting stats. |
| Comparison baselines      | Paper comparison corpus against SQP/QP/gap methods and external packages where dependencies are acceptable.                                                 | No comparison harness for deformables.                                                              | DART-native comparison fixtures; optional/manual external baselines documented separately from required CI gates.                                  |
| Visual evidence           | Long-horizon images/video for every promoted example family, including nonblank/motion-difference checks and human-inspected contact sheets.                | One spring-net GUI example with surface/points/combined modes.                                      | Headless Filament captures for tutorial, unit, stress, friction, scaling, and comparison example families; PR-linked screenshots/videos.           |

## Algorithm 1 Coverage

| Algorithm 1 step                         | IPC behavior                                                 | DART status                                                            | Required work                                                                               |
| ---------------------------------------- | ------------------------------------------------------------ | ---------------------------------------------------------------------- | ------------------------------------------------------------------------------------------- |
| Inputs                                   | Time-step state plus dynamics accuracy `epsilon`.            | Time step uses world state; tolerance is internal.                     | Add solver options/diagnostics with DART-owned names.                                       |
| `x <- x_t`                               | Warm start from current state, with previous step available. | Current stage initializes from current positions and inertial targets. | Preserve warm-start policy for mesh DOFs and restarts.                                      |
| `ComputeConstraintSet(x, d_hat)`         | Build close primitive-pair constraint set.                   | Missing for deformables.                                               | Add surface primitive graph, adjacency exclusions, broad phase, and candidate cache.        |
| Evaluate `B_t`                           | Incremental potential plus all active barriers.              | Point-mass/spring plus analytic ground barrier only.                   | Add FE elasticity, contact barriers, friction potential, BC terms, and diagnostics.         |
| Hessian projection                       | Project local elasticity/barrier/friction Hessians to PSD.   | Missing.                                                               | Add local stencil Hessians and PSD projection tests.                                        |
| Newton direction                         | Solve sparse SPD system.                                     | Uses diagonal scaled gradient descent.                                 | Add solver abstraction, direct solver path, iterative fallback, and residual checks.        |
| CCD upper bound                          | Compute conservative feasible step before backtracking.      | Missing except analytic ground feasibility rejection.                  | Use DART primitive PT/EE CCD for self/external contact and exact validation in tests.       |
| Backtracking                             | Require energy decrease while recomputing constraints.       | Present only for simple objective.                                     | Recompute/cull contact constraints during line search and record rejected candidates.       |
| Update stiffness/BC/equality constraints | Adapt barrier stiffness and moving boundary/equality data.   | Static ground only; fixed nodes only.                                  | Add adaptive barrier stiffness, DBC/NBC time ranges, moving obstacles, and scripted motion. |
| Termination                              | Stop on scaled Newton direction vs `epsilon_d`.              | Stops on gradient norm or iteration limits.                            | Match IPC-style convergence metrics and expose stats.                                       |

## Upstream Optimizer Phase Coverage

The reference implementation decomposes more than Algorithm 1's top-level
loop. DART should not copy the upstream API, but it must cover the same solver
responsibilities with DART-owned names:

| Upstream responsibility         | Required DART equivalent                                                                                  | Current status                                     |
| ------------------------------- | --------------------------------------------------------------------------------------------------------- | -------------------------------------------------- |
| Warm start policy               | Configurable or internally documented warm-start choices for current state, velocity prediction, and BCs. | Current point-mass step starts from current state. |
| Mesh/material initialization    | Per-shape mesh loading, scaling, rotation, translation, density, stiffness, and damping validation.       | Missing except hand-authored point-mass options.   |
| Collision-object initialization | Half-space, mesh collision object, point/segment/triangle codimensional object, and mesh sequence paths.  | Static box/sphere top barrier only.                |
| Boundary-condition scripting    | DBC/NBC selection by spatial box, time ranges, linear/angular scripted motion, and force work.            | Fixed nodes only.                                  |
| Elastic solve setup             | Sparse matrix pattern, local-to-global DOF layout, DBC projection, and matrix-cache invalidation.         | Missing.                                           |
| Energy assembly                 | Separate inertia, elastic, barrier, friction, damping, BC, and comparison-mode terms for diagnostics.     | One combined point-mass objective.                 |
| Line-search filtering           | Elastic inversion filter, CCD upper bound, active set rebuild, and energy decrease check.                 | Analytic ground feasibility and backtracking.      |
| Homotopy/adaptation             | Barrier/stiction distance schedule, minimum barrier stiffness scale, active-set convergence policy.       | Missing.                                           |
| Friction outer loop             | Lagged friction-force updates until tolerance or iteration cap.                                           | Missing.                                           |
| Output/restart                  | Position, velocity, acceleration, search direction, contact/friction data, and restart metadata.          | Binary world serialization only.                   |

## Implementation Backlog

1. **Mesh and material model**
   - Add tetrahedral/triangular mesh-backed deformable bodies with surface
     extraction and adjacency metadata.
   - Add density-based mass assembly and per-body material properties:
     Young's modulus, Poisson ratio, density, and damping.
   - Implement neo-Hookean and fixed-corotational energy options with
     inversion-aware behavior.
2. **Time integration and boundary conditions**
   - Keep backward Euler and add implicit Newmark with `beta`/`gamma`.
   - Add Dirichlet and Neumann boundary conditions, optional time ranges,
     scripted linear/angular motion, and restartable state.
3. **Distance and contact primitives**
   - Implement PT/EE distance values, closest-feature classification,
     gradients, Hessians, tangent bases, and edge-edge mollifier.
   - Connect native primitive CCD to Newton line search for self-contact and
     deformable-rigid/codimensional contact.
4. **Barrier solver**
   - Replace steepest descent with projected Newton over sparse stencils.
   - Add clamped log barrier with `d_hat`, adaptive stiffness, force positivity,
     complementarity metrics, and exact/admissibility validation checks.
5. **Friction**
   - Add smoothed static friction with `epsilon_v`, lagged tangent bases,
     lagged normal forces, configurable friction iterations, and convergence
     tests.
6. **Collision objects**
   - Support volumetric, surface, segment, point, analytical plane/half-space,
     moving mesh collision objects, and mesh sequences.
   - Keep public API names DART-owned; do not expose "IPC" as a solver selector.
7. **Output and diagnostics**
   - Add per-step output suitable for visual replay, solver stats, energy,
     momentum, contact counts, memory, and restart files.
   - Add headless GUI verification for every promoted example family.
8. **Python facade**
   - Bind the stable user-facing deformable body/options only after the C++
     mesh/contact options settle.

## Bounded Implementation Slices

Full IPC parity is too broad for one coding pass. Use these stop points when
creating the next `docs/dev_tasks/` folder and PR sequence.

| Slice                                      | Dependencies                   | Stop point                                                                                       | Minimum gate                                                                                                                              |
| ------------------------------------------ | ------------------------------ | ------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------- |
| 1. Mesh/material state and scene loading   | Current point-mass slice       | Mesh-backed deformable bodies load, validate, serialize, render, and step with no contact.       | Mesh/material unit tests, BE point-mass parity, surface extraction tests, `pixi run lint`, `pixi run build`, `check-api-boundaries`.      |
| 2. Boundary conditions and diagnostics     | Slice 1                        | DBC/NBC/time ranges, scripted motion, restart, and stats schema work on contact-free meshes.     | DBC/NBC/restart tests, diagnostics JSON schema test, one tutorial scene replay, benchmark JSON for no-contact mesh stepping.              |
| 3. Distance and candidate kernels          | Slice 1 surface adjacency      | PT/EE distances, derivatives, tangent bases, mollifier, broad phase, and adjacency filters land. | Distance finite-difference tests, candidate brute-force comparisons, `bm_ipc_distance_kernels`, `bm_ipc_constraint_set`.                  |
| 4. CCD line search and barrier constraints | Slices 2-3                     | Conservative CCD upper bounds and clamped barriers prevent mesh contact intersections.           | CCD line-search tests, exact post-step audits, barrier derivative/PSD tests, `bm_ipc_barrier_kernels`, `bm_ipc_ccd_line_search`.          |
| 5. Projected Newton solver                 | Slice 4                        | Sparse projected Newton replaces gradient descent for mesh IPC constraints.                      | Solver residual tests, sparse assembly/fallback tests, matrix-cache tests, `bm_ipc_projected_newton`, profiling hot-path summary.         |
| 6. Friction                                | Slice 5                        | Lagged smoothed friction supports self/external contact with convergence diagnostics.            | Friction potential/Jacobian tests, slope/stick-slip scenes, `bm_ipc_friction`, friction iteration stats.                                  |
| 7. Corpus port                             | Slices 1-6 as needed per scene | The upstream scene manifest has zero unclassified rows and all required DART artifacts exist.    | Tutorial/paper/stress/comparison CTest groups, `bm_ipc_scene_corpus`, `bm_ipc_scaling`, headless Filament evidence for promoted examples. |
| 8. Python facade                           | Stable C++ options/diagnostics | Python exposes only stable user-facing deformable options/state and no solver internals.         | `pixi run test-py`, Python example smoke, API-boundary review.                                                                            |

For every slice, define DART-owned option names, units, defaults, validation,
serialization behavior, and public/internal boundary before exposing the
option. Upstream setting names are inputs to the audit, not public DART API
names.

## Material, Property, And Scene-Option Coverage

This table is intentionally broader than public API requirements. Some rows
should remain internal, test-only, or example-only in DART, but every upstream
property family needs an explicit DART decision before full IPC parity is
claimed.

| Upstream setting/property family                                      | Required DART support or decision                                                                  | Current status                                               |
| --------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `shape`, `shapes`, transform/scale                                    | Mesh-backed body loading with per-shape transform, scale, material overrides, and validation.      | Missing; point-mass options are hand-authored.               |
| `shapeMatrix`, legacy replicated shapes                               | Either port as examples through a DART scene generator or document as legacy-only input shorthand. | Missing.                                                     |
| `meshSeq`                                                             | Moving mesh sequence collision objects for scripted obstacles.                                     | Missing.                                                     |
| `energy NH`                                                           | Non-inverting neo-Hookean volumetric energy.                                                       | Missing.                                                     |
| `energy FCR`                                                          | Fixed-corotational energy with inversion-aware filter.                                             | Missing.                                                     |
| `timeIntegration BE`                                                  | Backward Euler incremental potential.                                                              | Partial point-mass-only surrogate.                           |
| `timeIntegration NM beta gamma`                                       | Implicit Newmark with acceleration state and high-speed validation.                                | Missing.                                                     |
| `density`, per-shape `material density`                               | Density-based mass from volume/area with per-shape overrides.                                      | Missing.                                                     |
| `stiffness E nu`, per-shape material overrides                        | Young's modulus and Poisson ratio with validity ranges and Lamé conversion tests.                  | Missing.                                                     |
| `dampingStiff`, `dampingRatio`                                        | Rayleigh/lagged damping options and energy diagnostics.                                            | One scalar velocity damping on spring model only.            |
| `turnOffGravity`                                                      | Explicit gravity control per scene or world.                                                       | World gravity exists; no deformable scene-loader parity.     |
| `time duration dt`                                                    | Scene duration/time-step controls, including large `dt` and sub-frame/high-speed examples.         | World timestep and frame count exist; no IPC scene loader.   |
| `tol`, `useActiveSetConvergence`, `noActiveSetConvergence`            | Dynamics residual/convergence policy and diagnostics.                                              | Internal fixed gradient/iteration limits only.               |
| `dHat`, `tuning`, `useAbsParameters`                                  | Geometric accuracy/contact activation distance schedule.                                           | Internal fixed activation distance only.                     |
| `kappaMinMultiplier`, `minBarrierStiffnessScale`                      | Minimum barrier stiffness scale and adaptation diagnostics.                                        | Missing.                                                     |
| `epsv`                                                                | Stiction velocity accuracy for smoothed friction.                                                  | Missing.                                                     |
| `fricIterAmt`                                                         | Lagged friction iteration cap or convergence-until-satisfied mode.                                 | Missing.                                                     |
| `ground`, `halfSpace`                                                 | Analytic half-space collision objects with friction and moving variants where applicable.          | Partial static box/sphere top-surface barrier only.          |
| `meshCO`, `shapes ... linearVelocity/angularVelocity` collision paths | Static/moving mesh collision objects, triangle/edge/point object modes, and friction.              | Missing.                                                     |
| `selfCollisionOn`, `selfCollisionOff`, `selfFric`                     | Self-contact and self-friction policy per body/scene.                                              | Missing.                                                     |
| `DBC`, `DBCTimeRange`, `handleRatio`                                  | Dirichlet boundary regions, scripted linear/angular motion, and time ranges.                       | Fixed nodes only; no time range or scripted BC motion.       |
| `NBC`, `NBCTimeRange`                                                 | Neumann force regions and time ranges.                                                             | Missing.                                                     |
| `initVel`, `linearVelocity`, `angularVelocity`, `script`              | Initial velocity, scripted object/BC motion, and scene-control scripts.                            | Node initial velocities only.                                |
| `warmStart`                                                           | Warm-start strategy decision and restart interaction.                                              | Starts from current point-mass state only.                   |
| `restart`, `statusPath`, output paths                                 | Restartable position/velocity/acceleration state plus replay outputs.                              | Binary world serialization only.                             |
| `linearSolver`, `linSysSolver`                                        | Sparse linear solver abstraction and fallback policy.                                              | Missing for deformables.                                     |
| `constraintSolver`, `constraintType`, `constraintOffset`, `QPSolver`  | IPC/SQP/QP/gap comparison harnesses; likely internal/test-only in DART.                            | Missing.                                                     |
| `CCDMethod`, `CCDTolerance`                                           | Runtime CCD policy for tests/validation and conservative default path.                             | Native primitive CCD exists, not wired into deformables.     |
| `view`, `zoom`, `cameraTracking`, `playBackSpeed`                     | Example/visualization controls for reproduced upstream scenes.                                     | Current example supports camera/render options only broadly. |
| `appendStr`, `disableCout`, output naming                             | DART-owned output/diagnostic naming and quiet-mode policy for benchmarks.                          | Missing.                                                     |

## Upstream Example Matrix

Every row below needs either a runnable DART example, a focused test, a
benchmark, or a documented non-CI/manual baseline before PLAN-081 can claim
paper-level coverage.

Coverage policy:

- every upstream `.txt` scene must map to one DART artifact row;
- every mesh/segment/sequence asset referenced by a scene must load through
  DART-owned IO or a documented importer shim;
- paper/tutorial scenes should become maintained examples when they are useful
  for users, and otherwise focused tests/benchmarks when they are primarily
  regression fixtures;
- external application files under `supplementB` are manual comparison assets
  unless their dependencies become CI-safe.

Before claiming parity, keep the machine-readable scene corpus manifest current
with one row per upstream `.txt` scene path and these fields:

- upstream path;
- upstream commit and symlink alias target, when applicable;
- DART target type: `test`, `benchmark`, `example`, `manual`, or
  `not-applicable`;
- required assets/importer path;
- implemented DART command, CTest label, or benchmark binary;
- expected invariant, such as no intersections, min distance, friction
  threshold, momentum/energy tolerance, or visual deformation;
- benchmark/profiling artifact path when the scene is performance-relevant;
- visual evidence requirement when the scene is user-facing or paper-facing.

The manifest must have zero `unclassified` rows before a full-parity claim.

Audited `.txt` scene path counts by family: 24 top-level paper examples, 10
time-step variants, 10 Erleben cases, 9 scaling scenes, 8 video examples, 30
SQP benchmark scenes, 1 Utopia comparison scene, 22 tutorial scenes, 6 CCD
scenes, 9 codimensional unit scenes, 7 friction scenes, 5 typical scenes, 5
material/resolution sweep scenes, 1 tunnel scene, 1 bar-twist scene, and 6
failure-case scenes. The 30 SQP rows include 10 symlink aliases to Erleben
scenes and must remain distinct manifest rows because they are distinct
upstream paths and benchmark obligations.

### Paper Figures And Benchmark Scenes

| Paper/example                    | Upstream input                                                                         | Required DART target                                                       |
| -------------------------------- | -------------------------------------------------------------------------------------- | -------------------------------------------------------------------------- |
| Squeeze out                      | `input/paperExamples/1_squeezeOut.txt`                                                 | Long-horizon GUI example, contact-count benchmark, headless video evidence |
| Nonsmooth codimensional contacts | `input/paperExamples/2_matKnives.txt`, `input/paperExamples/2_spherePoints.txt`        | Codimensional surface/point obstacle tests and GUI examples                |
| Rods twist                       | `input/paperExamples/4_rodsTwist.txt`                                                  | 100s stress benchmark, time-step sweep baseline                            |
| Stiff card house                 | `input/paperExamples/5_hitCardHouse.txt`                                               | Friction/stiction regression and visual example                            |
| Masonry arch                     | `input/paperExamples/7_masonryArch.txt`                                                | Static-friction convergence test with no upper lagged-iteration cap        |
| Roller ball                      | `input/paperExamples/8_rollerBall.txt`                                                 | Large-deformation friction example                                         |
| Contact unit cases               | `input/paperExamples/10_cubeStack.txt`, `10_tetSlots.txt`, `10_tetTet.txt`             | Degenerate contact tests                                                   |
| Erleben cases                    | `input/paperExamples/11_erleben/*.txt`                                                 | Exact-alignment/nonsmooth regression suite                                 |
| Large mass/stiffness ratios      | `input/paperExamples/12_largeMassRatio.txt`, `12_matOnBoard.txt`, `12_sphereOnMat.txt` | Ill-conditioning tests and benchmarks                                      |
| Dolphin funnel                   | `input/paperExamples/13_dolphinFunnel.txt`                                             | Codimensional funnel stress example                                        |
| Mat twist                        | `input/paperExamples/14_matTwist.txt`                                                  | Thin volumetric stress benchmark                                           |
| Trash compactor                  | `input/paperExamples/15_trashComp_octocat.txt`, `15_trashComp_shapes.txt`              | Extreme compression examples                                               |
| Armadillo roller                 | `input/paperExamples/16_armaRoller_E1e5.txt`                                           | Stick-slip and material-model comparison                                   |
| Pin cushion                      | `input/paperExamples/17_pinCushionBall.txt`                                            | Segment/point obstacle compression example                                 |
| Codimensional rollers            | `input/paperExamples/18_pointRollerBall.txt`, `18_segRollerBall.txt`                   | Point/segment moving obstacle examples                                     |
| High-speed impact                | `input/paperExamples/19_golfBall.txt`                                                  | Newmark/high-speed validation and visual comparison                        |
| Stick-slip rod                   | `input/paperExamples/20_pencilStickSlip.txt`                                           | Friction oscillation benchmark                                             |
| Scaling                          | `input/paperExamples/21_scalability/*.txt`                                             | Resolution scaling benchmarks and memory stats                             |
| Squishy ball                     | `input/paperExamples/22_squishyBall.txt`                                               | Large iterative-solver stress benchmark                                    |

### Time-Step And Video Variants

- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.002.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.005.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.01.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.025.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.05.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.1.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.2.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt0.5.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt1.txt`
- `input/paperExamples/tb1_diffDt/rodsTwist_E1e4_dt2.txt`
- `input/paperExamples/videoExamples/armaRoller_stickSlip.txt`
- `input/paperExamples/videoExamples/chain5.txt`
- `input/paperExamples/videoExamples/chain10.txt`
- `input/paperExamples/videoExamples/chain35.txt`
- `input/paperExamples/videoExamples/chain100.txt`
- `input/paperExamples/videoExamples/heavyBallHit.txt`
- `input/paperExamples/videoExamples/hitLargeCardHouse.txt`
- `input/paperExamples/videoExamples/tightFitCube.txt`

### Tutorial And Feature-Option Scenes

- Basic cube fall variants:
  `input/tutorialExamples/2cubesFall.txt`,
  `2cubesFall_full.txt`, `2cubesFall_heavyTop.txt`,
  `2cubesFall_largerDt.txt`, `2cubesFall_smallerDt.txt`,
  `2cubesFall_stiffer.txt`.
- Boundary conditions:
  `input/tutorialExamples/BC/2cubesFall_DBC.txt`,
  `2cubesFall_DBC_timeRange.txt`, `2cubesFall_NBC.txt`,
  `2cubesFall_NBC_timeRange.txt`.
- Mesh collision objects:
  `input/tutorialExamples/MCO/2cubesFall_rotateCO.txt`,
  `2cubesFall_rotateCO_closedSurface.txt`,
  `2cubesFall_rotateCO_edges.txt`,
  `2cubesFall_rotateCO_points.txt`,
  `2cubesFall_rotateCO_triangle.txt`,
  `2cubesFall_translateCO.txt`.
- Integration/advanced settings:
  `input/tutorialExamples/advanced/2cubesFall_BE.txt`,
  `2cubesFall_NM.txt`, `2cubesFall_attach.txt`,
  `2cubesFall_rotateCO_meshSeq.txt`.
- Initial velocity:
  `input/tutorialExamples/initVel/2cubesFall_initVel.txt`,
  `2cubesFall_initRot.txt`.

### Additional Upstream Validation Scenes

- CCD: `input/otherExamples/ccd/matTwist.txt`,
  `octocatBowl.txt`, `octocatPlane.txt`, `pointTriangleCO.txt`,
  `somethingPlane.txt`, `torusCone.txt`.
- Codimensional unit tests:
  `input/otherExamples/coDimUnitTests/mat40x40_pointPlaneDrop.txt`,
  `mat40x40_segPlaneDrop.txt`, `mat40x40_squarePlaneDrop.txt`,
  `octocat_pointPlaneDrop.txt`, `octocat_segPlaneDrop.txt`,
  `octocat_squarePlaneDrop.txt`, `sphere5K_pointPlaneDrop.txt`,
  `sphere5K_segPlaneDrop.txt`, `sphere5K_squarePlaneDrop.txt`.
- Friction:
  `input/otherExamples/friction/2tets.txt`,
  `masonryArch_25.txt`, `masonryArch_25_0.2.txt`,
  `matHouse_sharp.txt`, `matHouse_sharp0.2.txt`,
  `slopeTest_highSchoolPhysics_0.49.txt`,
  `slopeTest_highSchoolPhysics_0.5.txt`.
- Typical/stress/material sweeps:
  `input/otherExamples/barTwist_noCollisions.txt`,
  `tunnel.txt`, `typical/2matsFall.txt`,
  `typical/mat40x40Twist.txt`, `typical/rodsTwist.txt`,
  `typical/sphere1K_DCORotCylinders.txt`,
  `typical/sphere1K_segBedSquash.txt`,
  `varyResolution/bar-test.txt`,
  `varyYoungs/2_mat40x40s_fall_E1e2.txt`,
  `varyYoungs/2_mat40x40s_fall_E1e4.txt`,
  `varyYoungs/2_mat40x40s_fall_E1e6.txt`,
  `varyYoungs/2_mat40x40s_fall_E1e8.txt`.

### SQP/Comparison Benchmark Corpus

The upstream comparison supplement includes these frictionless SQP benchmark
scripts. DART should port them into a native benchmark/test harness; Gurobi or
commercial-code baselines should stay optional/manual unless a CI-safe
dependency policy is approved.

- `input/paperExamples/supplementB/SQPBenchmark/01_pointTriangleCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/02_trianglePointCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/03_edgeEdgeCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/04_pointTriangle.txt`
- `input/paperExamples/supplementB/SQPBenchmark/05_edgeEdge.txt`
- `input/paperExamples/supplementB/SQPBenchmark/06_tetTet.txt`
- `input/paperExamples/supplementB/SQPBenchmark/07_cubePlaneCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/08_cubeCubeCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/09_tetCube.txt`
- `input/paperExamples/supplementB/SQPBenchmark/10_alignedTetCube.txt`
- `input/paperExamples/supplementB/SQPBenchmark/11_cubes.txt`
- `input/paperExamples/supplementB/SQPBenchmark/12_alignedCubes.txt`
- `input/paperExamples/supplementB/SQPBenchmark/13_fiveCubePyramid.txt`
- `input/paperExamples/supplementB/SQPBenchmark/14_fiveCubeStack.txt`
- `input/paperExamples/supplementB/SQPBenchmark/15_cubesCornerCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/16_tetsCornerCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/17_ErlebenSpikes.txt`
- `input/paperExamples/supplementB/SQPBenchmark/18_ErlebenSpikeWedge.txt`
- `input/paperExamples/supplementB/SQPBenchmark/19_ErlebenWedges.txt`
- `input/paperExamples/supplementB/SQPBenchmark/20_ErlebenSpikeHole.txt`
- `input/paperExamples/supplementB/SQPBenchmark/21_ErlebenSpikeCrack.txt`
- `input/paperExamples/supplementB/SQPBenchmark/22_ErlebenWedgeCrack.txt`
- `input/paperExamples/supplementB/SQPBenchmark/23_ErlebenSlidingSpike.txt`
- `input/paperExamples/supplementB/SQPBenchmark/24_ErlebenSlidingWedge.txt`
- `input/paperExamples/supplementB/SQPBenchmark/25_ErlebenCliffEdges.txt`
- `input/paperExamples/supplementB/SQPBenchmark/26_ErlebenInternalEdges.txt`
- `input/paperExamples/supplementB/SQPBenchmark/27_chain.txt`
- `input/paperExamples/supplementB/SQPBenchmark/28_rodPlaneCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/29_cowHeadPlaneCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/30_matTwist.txt`
- `input/paperExamples/supplementB/Utopia/utopiaComparison.txt`

### Failure-Mode Corpus

These upstream scenes encode solver/dependency failure cases. DART should port
the physical scene when useful, but the commercial-solver failure itself should
remain a documented optional/manual baseline unless a CI-safe dependency policy
is approved.

- `input/failures/GurobiException.txt`
- `input/failures/GurobiIncomplete.txt`
- `input/failures/SQPMemoryIssue1.txt`
- `input/failures/SQPMemoryIssue2.txt`
- `input/failures/SQPMemoryIssue3.txt`
- `input/failures/SQPMemoryIssue4.txt`

## Upstream Test Coverage To Mirror

DART already has native primitive CCD tests from the continuous-collision work,
but full IPC parity should mirror the modern toolkit's focused test families:

- Barrier: adaptive stiffness, barrier values/gradients/Hessians, force
  magnitude, and barrier potential.
- Broad phase/candidates: AABB, spatial hash/LBVH, candidate normals,
  per-vertex safe distances, and brute-force comparisons.
- CCD: point-point, point-edge, point-triangle, edge-edge, nonlinear CCD, GPU
  CCD if a backend is added, and benchmark-generated cases.
- Distances: point-point, point-line, point-edge, point-plane,
  point-triangle, line-line, edge-edge, signed distance, distance type, and
  edge-edge mollifier.
- Friction: isotropic/anisotropic friction, smooth friction mollifier, smooth
  friction coefficient, force Jacobian, potential, relative velocity, tangent
  basis, and closest point.
- Collision mesh/filtering: collision mesh construction, intersections,
  collision filters, normal collisions, plane-vertex collisions, feasible
  regions, and trust-region helpers.
- Utilities: matrix cache, local/global DOF layout, profiler hooks, and interval
  arithmetic.

The DART test suite should split these into small, debuggable targets instead
of a single monolithic IPC test. Suggested owners:

- `tests/unit/simulation/experimental/ipc_distance/` for distance, tangent,
  mollifier, and finite-difference derivative checks;
- `tests/unit/simulation/experimental/ipc_barrier/` for barrier value,
  gradient, Hessian, PSD projection, stiffness adaptation, and force magnitude;
- `tests/unit/simulation/experimental/ipc_collision/` for collision mesh,
  broad phase, candidate filtering, CCD line search, and exact-CCD audits;
- `tests/unit/simulation/experimental/ipc_friction/` for friction potential,
  tangent basis, force Jacobian, smooth friction coefficient, and lagged
  convergence;
- `tests/unit/simulation/experimental/ipc_solver/` for sparse assembly,
  projected Newton, linear-solver fallback, DBC/NBC projection, restart, and
  diagnostics;
- `tests/unit/simulation/experimental/world/` for public `World` facade
  integration, serialization, and API-boundary behavior.

## Benchmark And Profiling Targets

Add these benchmark families before promoting the solver beyond experimental
status:

1. **Kernel microbenchmarks**: PT/EE distance derivatives, edge-edge mollifier,
   barrier energy/gradient/Hessian, tangent basis, friction potential, and CCD
   line-search upper bounds.
2. **Solver microbenchmarks**: projected Newton iteration time, sparse matrix
   assembly, linear solve, line-search rejection rate, constraint rebuild time,
   and memory per active contact.
3. **Scene benchmarks**: upstream paper examples, time-step sweep, material
   stiffness sweep, resolution scaling, friction examples, codimensional
   obstacles, and high-speed impact.
4. **Correctness benchmarks**: analytic slope friction thresholds, simple
   spring/FE manufactured solutions, exact-CCD post-step checks, and
   complementarity/positivity/injectivity metrics.
5. **Visual benchmarks**: every GUI example should have a long-horizon
   headless Filament capture, motion-difference sanity check, and at least one
   inspected contact sheet or video artifact attached to the PR rather than
   committed.

At minimum, promoted GUI coverage should include long-horizon Filament runs for
squeeze-out, rods twist, stiff card house, masonry arch, roller ball, pin
cushion or another codimensional-obstacle scene, high-speed golf ball,
stick-slip pencil, scaling scenes, and tutorial variants. Each run should fix
camera pose, resolution, frame count, profile output, screenshot/contact-sheet
path, and expected motion/contact invariant.

Minimum benchmark/profiling output for each full-parity PR:

- Google Benchmark JSON with repetitions and aggregate rows for each touched
  kernel, solver, and scene group.
- Dedicated benchmark families named by responsibility:
  `bm_ipc_distance_kernels`, `bm_ipc_barrier_kernels`,
  `bm_ipc_ccd_line_search`, `bm_ipc_friction`,
  `bm_ipc_projected_newton`, `bm_ipc_constraint_set`,
  `bm_ipc_scene_corpus`, and `bm_ipc_scaling`.
- Per-scene stats including nodes/elements/faces, active contacts average/max,
  Newton iterations average/max, friction iterations average/max, line-search
  trials/rejections, CCD calls, constraint rebuilds, sparse assembly time,
  linear solve time, total step time, and memory/RSS.
- Scaling sweeps over resolution, timestep, stiffness, mass ratio, friction
  coefficient, and contact density.
- A rigid-only/no-contact baseline and a no-friction/no-self-contact baseline
  so regressions can be attributed.
- Profiling notes that identify the current top three hot paths before and
  after optimization.

## Next-Session Execution Order

1. **Mesh/state foundation**: land mesh-backed deformable model/state,
   density/mass assembly, surface extraction, adjacency metadata, scene asset
   loading, material parameters, BE/NM state, DBC/NBC, scripted motion, restart,
   and output diagnostics.
2. **Distance kernels**: land PT/EE distance values, feature classification,
   gradients, Hessians, tangent bases, edge-edge mollifier, and
   finite-difference tests.
3. **Constraint and CCD pipeline**: wire candidate generation, spatial
   filtering, adjacency exclusions, conservative CCD upper bounds, exact-CCD
   post-step validation, and line-search diagnostics.
4. **Barrier/Newton solver**: replace gradient descent with projected Newton,
   sparse assembly, local PSD projection, linear-solver abstraction, barrier
   stiffness adaptation, and IPC-style convergence metrics.
5. **Friction**: add smoothed lagged friction, tangent-basis lagging, normal
   force lagging, friction iteration policy, and convergence diagnostics.
6. **Example/test corpus**: port tutorial examples first, then unit/failure
   cases, then paper stress/friction/scaling scenes, then SQP/comparison
   benchmarks.
7. **GUI evidence**: promote every user-facing scene with long-horizon
   headless Filament captures in surface, points/debug, and combined modes when
   those modes are meaningful.
8. **Python facade**: add dartpy bindings only after C++ solver options,
   diagnostics, and scene-loading choices stop moving.

## Required Gate For A Full IPC-Parity PR

- `pixi run lint`
- `pixi run build`
- `pixi run test-all` before final promotion, unless a maintainer explicitly
  scopes a smaller experimental-only PR.
- `pixi run test-eigen-overalignment` for Eigen-heavy storage/solver changes.
- `pixi run test-dart-gui-smoke` when promoted GUI examples or Filament
  extraction/rendering behavior changes.
- `pixi run check-api-boundaries`
- Focused C++ unit/CTest groups for mesh/material state, BE/Newmark,
  DBC/NBC/restart, PT/EE distance derivatives, CCD line search, barrier
  stiffness/admissibility, projected Newton, friction, serialization,
  diagnostics, and upstream scene end-to-end runs.
- `pixi run test-py` after dartpy bindings land.
- Google Benchmark JSON for kernel, solver, scene, scaling, and comparison
  groups, including the named benchmark families above.
- Headless Filament long-horizon captures for every GUI example family, with
  nonblank image checks, motion-difference checks, and human-inspected contact
  sheets or videos.
- PR body with external screenshots/videos and benchmark summaries.
- Paper/documentation updates that clearly distinguish implemented behavior
  from planned IPC parity.
- A final grep/audit pass that removes inaccurate "IPC parity" wording from
  examples, docs, and public API comments unless all rows above are backed by
  evidence.
