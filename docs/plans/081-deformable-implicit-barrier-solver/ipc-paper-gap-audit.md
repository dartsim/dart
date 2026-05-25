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
  `bca884663a85fbbceaef36826f164edd48cca9c6`.
- DART branch state audited: PR #2711 at
  `9cac3e50135eabc7497ccd767f47dbd705be9503`.
- Dependency note: upstream IPC and `ipc-toolkit` remain references and test
  baselines. PLAN-081 continues to require DART-owned implementation, DART API
  names, and no upstream runtime dependency.

## Current DART Slice

DART currently has an experimental point-mass deformable slice, not a full IPC
solver:

- `DeformableBody` stores world-space nodes, velocities, masses, fixed nodes,
  and distance-spring edges.
- `DeformableDynamicsStage` is in the default `World::step()` pipeline.
- The step objective is implicit-Euler inertia plus edge-spring energy, gravity,
  damping, and a C2-style analytic vertical ground barrier for explicitly
  opted-in static boxes/spheres.
- The internal solve is deterministic steepest-descent with Armijo-style
  backtracking and feasibility rejection against the analytic ground barrier.
- Public GUI rendering now has reusable deformable surface descriptors through
  `dart::gui::makeDeformableSurfaceRenderable()`.
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

## Material And Property Coverage

| Upstream setting/property                           | Required DART support                              | Current status                                    |
| --------------------------------------------------- | -------------------------------------------------- | ------------------------------------------------- |
| `energy NH`                                         | Non-inverting neo-Hookean volumetric energy        | Missing                                           |
| `energy FCR`                                        | Fixed-corotational energy                          | Missing                                           |
| `timeIntegration BE`                                | Backward Euler incremental potential               | Partial point-mass only                           |
| `timeIntegration NM beta gamma`                     | Implicit Newmark                                   | Missing                                           |
| `density`                                           | Density-based mass from mesh volume/area           | Missing                                           |
| `stiffness E nu`                                    | Young's modulus and Poisson ratio                  | Missing                                           |
| `dampingStiff`, `dampingRatio`                      | Rayleigh/lagged damping options                    | Missing                                           |
| `dHat`                                              | Geometric accuracy/contact activation distance     | Internal fixed value only                         |
| `tol`                                               | Dynamics accuracy schedule                         | Internal fixed tolerance only                     |
| `epsv`                                              | Stiction accuracy                                  | Missing                                           |
| `fricIterAmt`                                       | Lagged friction iteration control                  | Missing                                           |
| `ground`, `halfSpace`                               | Analytical static/moving collision objects         | Partial static box/sphere ground only             |
| `selfCollisionOff`, `selfFric`                      | Self-contact and self-friction policy              | Missing                                           |
| `DBC`, `DBCTimeRange`                               | Dirichlet boundary conditions and time ranges      | Fixed nodes only                                  |
| `NBC`, `NBCTimeRange`                               | Neumann boundary conditions and time ranges        | Missing                                           |
| `initVel`, `linearVelocity`, `angularVelocity`      | Initial and scripted rigid/collision-object motion | Node velocities only                              |
| `restart`                                           | Restart from saved simulation state                | Binary world serialization only                   |
| `CCDMethod`                                         | Runtime CCD policy, exact validation path          | DART primitive CCD exists, not wired here         |
| `constraintSolver`                                  | IPC/SQP/QP comparison modes                        | Missing; DART should keep this internal/test-only |
| `view`, `zoom`, `cameraTracking`, playback settings | Example/visualization controls                     | Partial GUI run options only                      |

## Upstream Example Matrix

Every row below needs either a runnable DART example, a focused test, a
benchmark, or a documented non-CI/manual baseline before PLAN-081 can claim
paper-level coverage.

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
  `2cubesFall_translateCO.txt`,
  `2cubesFall_rotateCO_meshSeq.txt`.
- Integration/advanced settings:
  `input/tutorialExamples/advanced/2cubesFall_BE.txt`,
  `2cubesFall_NM.txt`, `2cubesFall_attach.txt`.
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
- `input/paperExamples/supplementB/SQPBenchmark/27_chain.txt`
- `input/paperExamples/supplementB/SQPBenchmark/28_rodPlaneCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/29_cowHeadPlaneCO.txt`
- `input/paperExamples/supplementB/SQPBenchmark/30_matTwist.txt`
- `input/paperExamples/supplementB/Utopia/utopiaComparison.txt`

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

## Next-Session Execution Order

1. Land mesh-backed deformable model/state, mesh loading for examples, material
   parameters, BE/NM integration options, DBC/NBC, and output diagnostics.
2. Land PT/EE distance derivatives, edge-edge mollifier, tangent bases, and
   finite-difference tests.
3. Wire constraint-set generation, spatial filtering, and conservative CCD line
   search for self-contact and deformable-collision-object contact.
4. Replace the gradient-descent solve with projected Newton plus sparse linear
   solve and barrier stiffness adaptation.
5. Add friction potential and lagged friction iterations.
6. Port tutorial examples first, then paper unit tests, then stress examples,
   then comparison/benchmark corpora.
7. Add dartpy bindings only after C++ solver options and diagnostics stop
   moving.

## Required Gate For A Full IPC-Parity PR

- `pixi run lint`
- `pixi run build`
- Focused C++ unit tests for model, distance, barrier, CCD, friction, solver,
  serialization, and examples.
- `pixi run test-py` after dartpy bindings land.
- Google Benchmark JSON for kernel, solver, scene, scaling, and comparison
  groups.
- Headless Filament long-horizon captures for every GUI example family.
- PR body with external screenshots/videos and benchmark summaries.
- Paper/documentation updates that clearly distinguish implemented behavior
  from planned IPC parity.
