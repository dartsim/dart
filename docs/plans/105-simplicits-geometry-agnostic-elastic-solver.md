# PLAN-105: Simplicits Geometry-Agnostic Elastic Solver

- Operating state: `PLAN-105` in [`dashboard.md`](dashboard.md)
- Outcome: DART can step elastic objects from meshes, point clouds, implicit
  fields, volumetric scans, radiance fields, Gaussian splats, and other
  occupancy-queryable geometry through a DART-owned, geometry-agnostic reduced
  elastic solver inspired by Simplicits. The public `World` path stays
  representation- and backend-neutral, while the implementation preserves the
  full paper/site/video/Kaolin parity target on CPU and GPU.
- Current evidence:
  - Simplicits is a SIGGRAPH 2024 / TOG paper and project page that presents a
    data-, mesh-, and grid-free elastic simulation method. It maps input
    geometry to an occupancy function, trains an implicit neural skinning field
    as a reduced deformation basis using random perturbations and Monte Carlo
    elastic-energy sampling, then runs implicit reduced-space dynamics and maps
    deformations back to the original representation.
  - The official project page and paper report 140+ simulations across signed
    distance fields, point clouds, neural primitives, tomography scans,
    radiance fields / NeRFs, Gaussian splats, surface meshes, and volume
    meshes, with multiple material energies, contact models, and time
    integration schemes.
  - NVIDIA Kaolin exposes `kaolin.physics.simplicits` with point-sampled
    physics objects (`pts`, Young's moduli, Poisson ratios, densities, and
    approximate volume), learned skinning modules, a `SimplicitsScene`,
    backward-Euler/Newton solve utilities, gravity, floor, boundary conditions,
    point collision, QR conditioning, and tutorial notebooks for mesh, Gaussian
    splat, and other point-sampled geometry.
  - DART already has DART 7 `World` / `DeformableBody` foundations, material
    fields, VBD/AVBD and IPC-family deformable work, shared contact/barrier
    primitives, benchmark/py-demo surfaces, and private CUDA gates that this
    plan must reuse rather than duplicating.

## Owner Docs

- Research catalog:
  [`../readthedocs/papers.md`](../readthedocs/papers.md)
  (`modi-2024-simplicits`).
- Solver-family intake:
  [`solver-family-intake.md`](solver-family-intake.md).
- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md),
  [`../design/algorithm_extension_contracts.md`](../design/algorithm_extension_contracts.md).
- Related solver plans:
  [`081-deformable-implicit-barrier-solver.md`](081-deformable-implicit-barrier-solver.md),
  [`083-unified-newton-barrier-multibody.md`](083-unified-newton-barrier-multibody.md),
  [`104-vertex-block-descent-solver.md`](104-vertex-block-descent-solver.md).
- Compute and demo owners:
  [`../design/scalable_compute_decisions.md`](../design/scalable_compute_decisions.md),
  [`103-examples-strategy.md`](103-examples-strategy.md).

## Sources

- Vismay Modi, Nicholas Sharp, Or Perel, Shinjiro Sueda, and David I. W.
  Levin. "Simplicits: Mesh-Free, Geometry-Agnostic, Elastic Simulation." _ACM
  Transactions on Graphics_ 43(4), Article 117, 2024. DOI:
  <https://doi.org/10.1145/3658184>.
- Project page:
  <https://research.nvidia.com/labs/toronto-ai/simplicits/>.
- arXiv / PDF:
  <https://arxiv.org/abs/2407.09497>,
  <https://arxiv.org/pdf/2407.09497>,
  <https://research.nvidia.com/labs/toronto-ai/simplicits/assets/Simplicits.pdf>.
- Supplemental parameter/time table:
  <https://research.nvidia.com/labs/toronto-ai/simplicits/assets/SimplicitsSupplementalTable.pdf>.
- Videos:
  <https://www.youtube.com/watch?v=3Zameqg66aA>,
  <https://www.youtube.com/watch?v=kxEpXkSv2cM>.
- Kaolin notes and tutorial code:
  <https://kaolin.readthedocs.io/en/latest/notes/simplicits.html>,
  <https://github.com/NVIDIAGameWorks/kaolin/tree/master/examples/tutorial/physics>.
- Kaolin implementation surfaces audited for planning:
  `kaolin.physics.simplicits.training`, `simulation`, `network`, `losses`, and
  the `examples/tutorial/physics` notebooks.

## Full-Scope Rule

Simplicits is not complete when DART can run one point-sampled elastic demo or
when DART has a baked-weight CPU prototype. For this paper, "implemented" means:

- every algorithm, feature, parameter, edge case, and limitation in the paper,
  project page, supplemental table, videos, Kaolin notes, tutorial notebooks,
  and Kaolin implementation is classified and either implemented or explicitly
  rejected with a maintainer decision;
- DART has CPU and GPU implementations for the source sampling, reduced-basis
  evaluation, runtime solve, contact, benchmark, and demo paths where the paper
  or Kaolin reference exercises them;
- every paper/site/video/Kaolin example is represented by DART-owned tests,
  benchmark JSON, `py-demos`, and visual evidence as appropriate;
- DART benchmark packets beat the Kaolin/reference implementation and the
  published paper/supplemental timing rows for every claimed CPU/GPU case at
  matched scene parameters and accuracy; and
- public C++ and dartpy APIs expose DART-owned domains, reduced elastic
  capabilities, policies, diagnostics, and value types, not Simplicits as a
  public solver registry, Kaolin/Warp/Torch types, CUDA resources, neural
  training framework objects, or ECS storage.

## Solver-Family Intake

1. **Family routing.** Simplicits gets a new PLAN-105 owner because the core
   model is a learned, occupancy-driven reduced elastic basis over arbitrary
   point-sampled geometry. It is not mesh-backed IPC (PLAN-081), not VBD/AVBD
   block descent (PLAN-104), and not the unified Newton-barrier target
   (PLAN-083), though it must reuse their materials, barriers, contact,
   diagnostics, benchmark, and compute conventions.
2. **Shared-component inventory.** Reuse DART `World` domain assignment,
   `DeformableBody` material/state components, DART material parameterization,
   `detail/newton_barrier` contact/friction primitives where applicable,
   internal sparse/Newton helpers, private compute/CUDA boundaries from
   PLAN-030/031, benchmark dashboard schema, and PLAN-103 py-demo categories.
3. **Promotion trigger.** A source sampler, reduced-basis evaluator, contact
   primitive, sparse reduced-matrix helper, neural-field training utility, or
   benchmark schema may move to a shared internal owner only after a second
   DART solver consumes it and focused tests prove old and new consumers agree.
4. **Apples-to-apples evidence.** Compare against DART's mesh-backed
   deformable/VBD/IPC incumbents, the Kaolin implementation, and paper /
   supplemental table rows. Every benchmark row records representation type,
   object name, cubature points, handles, timestep, simulation steps, Newton
   iterations, barrier iterations, material parameters, contact settings,
   hardware, CPU/GPU timing, accuracy metric, and visual artifact link.
5. **Public boundary.** Public APIs may offer a DART-owned geometry-agnostic or
   reduced-elastic capability, baked reduced-body assets, material/cubature
   inputs, and diagnostics. They must not expose `Simplicits`, Kaolin, Warp,
   Torch, neural-network module types, solver registries, backend resources, or
   ECS storage as public dependencies.
6. **Configuration surface.** The common path should be a small `World` /
   dartpy configuration that accepts a geometry source or baked reduced asset,
   material fields, handle count, sample count, timestep, contact/floor/gravity
   options, and validation rules. Advanced options may cover training,
   cubature, QR conditioning, Newton/line-search tolerances, collision radius,
   and GPU execution, all serializable where they affect results.
7. **Failure and fallback semantics.** Unsupported geometry, missing occupancy
   queries, invalid material fields, failed training, non-convergence,
   insufficient collision capacity, unsupported GPU paths, and stale baked
   reduced assets must fail with actionable diagnostics rather than silently
   falling back to a different deformable solver.
8. **World-step schedule integration.** The runtime path must state which
   deformable schedule slot it adds or replaces, which domain flags activate it,
   how it handles collision/contact candidate generation, and which tests prove
   default `World::step()` and custom final-stage stepping share the same
   schedule without placeholder work when no reduced elastic bodies exist.

## Strategic DART 7 Pipeline Investigation

Simplicits should be investigated as a DART 7 solver-family addition, not as a
Kaolin port. The investigation must map every paper, project, video, and Kaolin
capability onto the existing DART 7 model/state/control/contact/solver/coupler/
backend pipeline before implementation starts.

- **Domain and model.** Treat geometry-agnostic reduced elasticity as its own
  deformable-domain solver family. The model layer should accept DART-owned
  geometry sources or baked reduced-body assets, then store occupancy sampling,
  material fields, cubature weights, handles, and learned-or-baked skinning
  weights as internal model data.
- **State and controls.** Runtime state should be handle transforms,
  velocities, reduced mass/operator caches, convergence diagnostics, and
  serialization records. Boundary conditions, gravity, floor/contact controls,
  and external forces should use DART-owned policy/value objects shared where
  possible with the deformable IPC and VBD/AVBD plans.
- **Contacts and coupling.** Point-sampled collision candidates, collision
  radius/capacity diagnostics, barrier or penalty rows, and friction should
  flow through shared DART contact buffers when the primitives are stable.
  Mixed rigid/reduced-elastic coupling should use the solver-architecture
  coupler seam instead of a Simplicits-specific branch.
- **Solver schedule.** The first runtime target is an opt-in reduced-elastic
  deformable stage in `World::step()`. It may reuse internal Newton,
  line-search, barrier, and benchmark infrastructure, but PLAN-105 remains the
  owner until a second solver proves a shared contract.
- **Backend and platform.** DART should keep a CPU reference path first, then
  add private CUDA kernels for reduced-basis evaluation, assembly, collision,
  and solves after matched CPU evidence exists. Public APIs must not require
  Torch, Warp, Kaolin, CUDA objects, or a training framework for ordinary
  simulation.
- **DART improvements likely needed.** Occupancy-query and point-sampling
  adapters, baked reduced-asset serialization, reduced-body contact buffers,
  representation-to-render deformation maps, long-horizon visual evidence for
  non-mesh inputs, and benchmark schemas that record representation-specific
  sampling/cubature choices.

## Workstreams

1. **Corpus and source audit.** Build a manifest from the paper, project page,
   both videos, Kaolin notes, Kaolin notebooks, and supplemental table. Start
   with the supplemental rows: Garden/Mic/Ficus/Lego splats, Bladder CT,
   SkullStrippedBrain, 511BeamELU, Iron, Iron Stiffer, Tree, Spike, LargeBox,
   Link, Mandelbulb, Ribbon, SimpleSkullBrain, Suzanne, Bezier, and Burger.
2. **Representation sampling and material fields.** Add DART-owned occupancy
   and point-sampling adapters for meshes, signed/unsigned distance fields,
   point clouds, tomography/density volumes, radiance-field or splat samples,
   and volume meshes; assign per-point Young's modulus, Poisson ratio, density,
   and approximate volume/cubature weights.
3. **Reduced basis fitting.** Implement an internal reduced-basis training path
   that learns shape-aware skinning weights from random handle perturbations,
   elastic energy, orthogonality regularization, Monte Carlo volume sampling,
   and material interpolation. The public runtime must also accept baked
   weights so DART does not require a tensor framework for ordinary simulation.
4. **Reduced elastic runtime.** Assemble the LBS/reduced mass and deformation
   gradient operators, handle transforms, QR or equivalent conditioning,
   implicit-Euler/Newton solve, line search, gravity, floor/boundary
   conditions, and Neo-Hookean/linear-elastic energy terms with focused tests.
5. **Contact and friction.** Add point/contact candidate generation, barrier
   or penalty contact, friction, collision capacity diagnostics, and comparison
   packets against PLAN-081/083 contact primitives and VBD/IPC incumbents.
6. **World and dartpy integration.** Wire an opt-in DART-owned reduced elastic
   configuration through `World`, diagnostics, serialization/restart, and
   dartpy. Keep the default deformable path unchanged.
7. **GPU path.** Route reduced-basis evaluation, matrix assembly, Newton
   kernels, collision, and benchmark scenes through the private CUDA boundary
   only after CPU correctness and benchmark packets exist.
8. **Examples and visuals.** Port the Kaolin easy/interactive/low-level
   tutorials and the paper/video examples into `py-demos` or another durable
   demo surface with headless visual captures.
9. **Performance leadership.** Record CPU/GPU benchmark JSON against DART
   incumbents, Kaolin, and the paper/supplemental numbers, then optimize until
   DART beats every claimed reference row before any Simplicits-complete claim.

## Current Next Gaps

1. Create the corpus/source manifest and classify every paper/project/video /
   Kaolin row into tests, benchmarks, py-demos, visual evidence, CPU reference
   comparison, GPU parity, and the DART 7 pipeline surface it exercises.
2. Start a `docs/dev_tasks/` tracker before implementation because the solver
   spans model loading, training/baked assets, runtime stepping, demos,
   benchmarks, and GPU parity.
3. Prototype only the first CPU source-audit slice after the manifest exists:
   one DART-owned point-sampled reduced elastic object with baked weights,
   Neo-Hookean energy, implicit-Euler/Newton solve, focused tests, and no
   public tensor-framework dependency.

## Acceptance Criteria

Simplicits progress is not complete until DART:

- keeps PLAN-105 as the owner for geometry-agnostic reduced elastic simulation
  and routes any shared primitive extraction through PLAN-020/083/104 evidence;
- classifies every source in the paper, project page, supplemental table,
  videos, Kaolin notes, notebooks, and implementation;
- supports the representation families named by the paper: signed distance
  functions, point clouds, neural primitives/radiance fields, tomography scans,
  Gaussian splats, surface meshes, and volume meshes;
- implements and tests occupancy/point sampling, material/cubature fields,
  learned or baked skinning weights, spatial weight gradients, reduced mass /
  deformation-gradient operators, implicit time integration, Newton/line search,
  QR or equivalent conditioning, boundary/floor/gravity controls, contact, and
  friction;
- records CPU and GPU benchmark/profiling JSON against DART incumbents, Kaolin,
  and the paper/supplemental table rows, with hardware and scene parameters;
- ports all source demos and paper/video scenes into DART tests, benchmarks,
  `py-demos`, and headless visual evidence as appropriate;
- keeps public APIs DART-owned and free of Simplicits/Kaolin/Warp/Torch/CUDA,
  solver-registry, ECS, and backend-resource leaks; and
- keeps `pixi run lint`, `pixi run build`, focused C++/Python tests,
  benchmark smokes, `check-api-boundaries`, and the relevant CUDA gates green
  for every promoted implementation slice.
