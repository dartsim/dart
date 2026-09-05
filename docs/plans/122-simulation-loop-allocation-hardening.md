# PLAN-122: DART 7 Simulation-Loop Allocation Hardening

- Operating state: `PLAN-122` in [`dashboard.md`](dashboard.md)
- Outcome: every DART 7 built-in `World::step()` domain and solver path that is
  selectable through the DART 7 `World` stack has direct post-bake evidence that
  same-shape simulation steps do not allocate from the World base allocator, the
  process global heap, or raw malloc-family paths. Legacy DART 6 code paths are
  excluded unless they are being migrated into the DART 7 `World` pipeline; once
  migration work starts, the DART 7 allocation contract applies to the migrated
  path.
- Current evidence:
  [`../design/hierarchical_allocator.md`](../design/hierarchical_allocator.md)
  owns the memory hierarchy and current contract; focused gates in
  `tests/unit/simulation/world/test_world.cpp` cover many baked DART 7 rigid,
  multibody, deformable, and differentiable slices. The coverage matrix in
  [`122-simulation-loop-allocation-hardening/coverage-matrix.md`](122-simulation-loop-allocation-hardening/coverage-matrix.md)
  records which rows are final evidence, which rows are steady-state-only
  evidence, and which rows remain open.
- Progress snapshot: 14 of 22 matrix rows are closed with first-post-bake
  evidence. `L-001` is now closed: the legacy `dynamics::Skeleton` → DART 7
  `World` model-loading bridge (`dart::simulation::io::addSkeleton`) has
  first-post-bake world-base, global-heap, and raw-malloc gates over an imported
  scene, with the source-model translation kept in the pre-bake configuration
  phase. These matrix rows do not establish universal parallel-executor coverage: `D-004` is closed by routing
  systems above the retained dense-direct cutoff to the sparse iterative path
  instead of Eigen sparse-direct numeric factorization. The remaining open rows
  are executor-inventory row `H-002`, existing-path rows `R-005`, `M-005`
  and `G-002`, plus promotion-gated rows
  `M-004`, `F-002`, and `G-001`. Each stays with its named owner until its
  evidence closes. No new `docs/dev_tasks/` folder is
  needed for those rows while this plan and its coverage matrix remain the
  durable owner.

## Additional Coverage Investigations

The [current assessment](../design/dart7_architecture_assessment.md#f6--clean-break-boundaries-and-scoped-foundation-evidence)
identifies parallel unified-island scratch and function-static deformable CUDA
PSD buffers that existing representative rows do not qualify. PLAN-122 owns
allocation coverage, coordinated with PLAN-030 for runtime/ownership. The matrix
already tracks these selectable paths as open `R-005` and `G-002`, and parallel
multibody ranges as `M-005`. `H-002` owns the complete executor-mode inventory;
default sequential evidence cannot qualify untested supplied executors. Deferring
their closure to M2/family work does not defer their admission to the matrix.
Record execution mode, concurrency and first-step evidence. WP-122.8 completion
must not close or erase these wider rows.

## Scope

In scope:

- DART 7 `dart::simulation::World` built-in step stages and selectable solver
  families.
- DART 7 migrations from legacy APIs when the migrated path enters the DART 7
  `World::step()` loop.
- Host-side allocation in CPU solvers, including Eigen dynamic storage that
  bypasses `operator new` through malloc-family calls.
- Future accelerator-backed DART 7 stages once they participate in
  `World::step()`; those rows also need a device-allocation gate owned by the
  relevant GPU plan.

Out of scope:

- Classic DART 6 `Skeleton` and legacy constraint-solver step paths that remain
  outside the DART 7 `World` pipeline.
- Public return-by-value diagnostics and standalone helper APIs whose outputs
  intentionally outlive a step, unless a built-in `World` stage calls them in
  the simulation loop.
- Third-party allocations outside the measured simulation loop. M1 includes
  runtime submission and any third-party allocation inside the step: an
  unavoidable allocation is a failed qualification, not an exemption.

## Harness Contract

The final gate shape is stricter than steady-state warm-up evidence:

1. Build the scene, select the DART 7 domain/solver path, and call
   `World::enterSimulationMode()` or the equivalent bake/prepare boundary.
2. Start measuring on the first `World::step()` after bake. A test may run more
   than one measured step, but it must not hide allocation by running unmeasured
   prewarm simulation steps first.
3. Check World base allocator activity for same-shape steps. A fixed-capacity
   free-list configuration is preferred when runtime growth should be a hard
   failure.
4. Check global `operator new` traffic for every built-in path.
5. Check raw malloc-family traffic on platforms where the interposer is
   available for paths that use Eigen dynamic matrices, decompositions, sparse
   factorizations, or third-party buffers. A skip because the interposer is
   unavailable is not evidence for closing a row.
6. Record the exact test name in the coverage matrix. Any PR that promotes or
   migrates a DART 7 `World::step()` path must update the row and gate in the
   same change.

Current raw-malloc tests that prewarm the simulation loop before measurement are
still useful regression guards, but they are classified as steady-state evidence
until the relevant bake path sizes every needed buffer before the first measured
post-bake step.

## Work Packets

WP-122.1–122.7 retain the existing coverage workstreams. Their row-level
progress is in the linked coverage matrix; do not re-execute a closed row or
infer that a whole packet/family is complete from its representative tests.
WP-122.8 is the next M1 qualification packet after its dependencies close.
Parallel and device paths require their own evidence even when a CPU row is
already closed.

### WP-122.1 Harness Manifest And First-Step Gates

- Objective: make the no-allocation harness reusable and make final evidence
  unambiguous.
- Scope: consolidate or wrap the existing world-base, global-heap, and raw-malloc
  test helpers so DART 7 simulation tests can express "post-bake first step"
  without copy/paste; add a lightweight checker or meta-test that verifies each
  closed matrix row cites an existing test; relabel existing prewarm-based raw
  gates as steady-state until their underlying path bakes all scratch.
  `H-002` inventories every selectable World stage/method and its default or
  supplied executor, actual graph/range/device dispatch, profiling and fallback
  behavior. Qualify each mode directly or link it to a named open row. Include
  Taskflow submission, lazy range pools and nested/concurrent execution; a
  cached graph's constant allocation floor is not zero-allocation evidence.
  Name inline versus Taskflow graph dispatch and kinematics cache rebuilds;
  multibody, island and VBD/AVBD colored-block/row ranges; profiling adapters;
  CUDA selection and CPU fallback; explicit rigid stages and custom pipelines
  containing built-in stages. Classify standalone WorldBatch graph construction
  separately from the measured World step. Inventory DART-owned executors and
  adapter contracts; caller-written executor/stage behavior requires separate
  conformance evidence.
- Non-goals: proving every domain row in this packet; changing the DART 6
  compatibility lane.
- Acceptance evidence: the helper/API name is documented in
  `docs/onboarding/testing.md`; the coverage matrix distinguishes final,
  steady-state-only, and open rows; a focused test proves the helper fails when
  an allocation is injected after bake.
  `H-002` closes only when the inventory records each supported mode's activation
  predicate and source location, with evidence that exercises that exact mode
  or a direct mapping to a distinct, concrete open row with scope, owner and
  acceptance criteria. Self/circular mappings and generic future-promotion
  placeholders cannot close the inventory. Removing a mapping or adding an
  unmapped dispatch mode must fail its validation. Inventory completeness does
  not qualify its open rows, which still block PLAN-122 completion. New modes
  invalidate the inventory until mapped. The Taskflow submission floor stays
  with WP-122.8/PLAN-030 and needs a concrete row before H-002 can close.
- Gates: `pixi run lint`, focused `test_world` allocation filters, and the new
  matrix/helper checker if added.
- Dependencies: none.

### WP-122.2 Rigid IPC Scale Closure

- Objective: close the known rigid IPC allocation gaps beyond the current
  representative gates.
- Scope: large equality KKT systems above the stack-solve cap, larger active
  barrier sets, and kinematic/dynamic mixed scenes whose dense/sparse scratch is
  sized lazily today.
- Non-goals: classic DART 6 constraint solver paths.
- Acceptance evidence: every in-scope rigid IPC matrix row has world-base,
  global-heap, and raw-malloc post-bake gates, or an explicit maintainer-approved
  size cap recorded in the matrix. `R-004` is closed for large equality KKT
  systems above the stack solve cap.
- Gates: `pixi run lint`, focused `test_world` rigid IPC allocation filters,
  and `test_rigid_ipc_barrier`.
- Dependencies: coordinate with PLAN-082 and
  `docs/dev_tasks/rigid_ipc_solver/` if that task is active.

### WP-122.3 Deformable IPC And FEM Scale Closure

- Objective: close deformable allocation gaps for mesh-backed and larger sparse
  solve paths.
- Scope: sparse-direct systems above the retained dense cutoff, larger FEM
  ground/self-contact scenes, static/moving obstacle surfaces, and mesh-backed
  IPC-class scenes from PLAN-081.
- Non-goals: paper-complete IPC feature parity itself; that remains owned by
  PLAN-081 and its dev task.
- Acceptance evidence: deformable rows that are selectable in DART 7
  `World::step()` have post-bake world-base, global-heap, and raw-malloc gates;
  larger sparse-direct paths either replace Eigen sparse numeric factorization
  with allocation-free retained storage or have a documented bake-time capacity
  cap that routes them to iterative/matrix-free solves. `D-004` is closed by
  the retained dense-direct cap plus default sparse iterative routing above it.
- Gates: `pixi run lint`, focused `test_world` deformable allocation filters,
  and PLAN-081 focused solver tests.
- Dependencies: PLAN-081 and `docs/dev_tasks/ipc_deformable_solver/`.

### WP-122.4 Articulated, Variational, AVBD, VBD, And Unified Newton Rows

- Objective: make every DART 7 articulated/multibody solver family row carry
  allocation evidence before promotion.
- Scope: semi-implicit multibody velocity paths, variational integration,
  AVBD/VBD, parallel unified-island `R-005` and multibody-range `M-005` rows
  already selectable in `World::step()`, and future
  unified Newton-barrier multibody rows from PLAN-083.
- Non-goals: DART 6 articulated-body parity paths outside the DART 7 `World`.
- Acceptance evidence: each promoted DART 7 row in the matrix cites post-bake
  world-base/global/raw gates, including contacts, motors, loop closures, and
  actuator combinations that use dynamic Eigen storage.
  For `R-005`, use at least two unequal active contact islands, more than one
  worker and an assertion of actual parallel unified dispatch. Measure worker
  allocations and runtime submission from the first post-bake step, including
  lazy range-pool creation; add worker-thread allocation negative controls.
  Preserve sequential-reference parity and dispatch with profiling enabled.
  `M-005` additionally requires multiple multibodies with actual multi-worker
  range dispatch. Counter coverage starts on the first post-bake step and
  includes worker/runtime setup; small ranges that execute inline do not
  qualify the parallel path. Compare against the sequential reference.
- Gates: `pixi run lint`, focused `test_world` multibody/variational/AVBD/VBD
  allocation filters, and the relevant PLAN-083/104 focused tests.
- Dependencies: PLAN-080, PLAN-083, PLAN-084, PLAN-104, and the corresponding
  active dev-task folders when they are in flight.

### WP-122.5 Differentiable And Trajectory Capture Rows

- Objective: ensure opt-in differentiable DART 7 stepping does not introduce
  hidden simulation-loop allocation after bake.
- Scope: contact-free derivative capture, future contact derivative rows,
  reverse/trajectory snapshot storage that is part of an opt-in simulation loop,
  and Python-facing calls that reuse DART-owned step scratch once they enter the
  C++ `World::step()` path.
- Non-goals: Python interpreter allocation outside the C++ step loop.
- Acceptance evidence: every promoted differentiable row has post-bake
  world-base/global/raw gates, and any trajectory storage growth is either
  preallocated at bake or explicitly outside the no-allocation loop.
- Gates: `pixi run lint`, focused `test_world` differentiable filters, and
  `test_diff_smooth_jacobian` when `DART_BUILD_DIFF=ON`.
- Dependencies: PLAN-110 and its durable design/gap-audit owner docs.

### WP-122.6 Loader And Migration Acceptance Gates

- Objective: prevent DART 7 model-loading migrations from bypassing the
  no-allocation contract.
- Scope: URDF, SDF, MJCF, USD, and skeleton-loading bridges that build a
  DART 7 `World`; once a loaded model can be stepped through a DART 7 solver
  family, add a same-shape post-bake allocation gate for at least one imported
  scene in that family.
- Non-goals: file parsing allocation before bake.
- Acceptance evidence: each promoted loader bridge cites one or more imported
  DART 7 `World` scenes in the matrix and proves the step loop is allocation-free
  after bake.
- Gates: `pixi run lint`, focused loader tests, focused `test_world` allocation
  filters, and relevant Python binding smoke if the loader is public in dartpy.
- Dependencies: PLAN-041, PLAN-042, PLAN-080, and loader-specific dev tasks.

### WP-122.7 Accelerator Allocation Semantics

- Objective: define and enforce allocation evidence when a GPU or other
  accelerator-backed DART 7 stage enters `World::step()`.
- Scope: host allocation gates still apply; device buffer growth, graph-capture
  allocation, and per-call `cudaMalloc`/`cudaFree` or equivalent backend
  allocation need a backend-specific counter or deterministic diagnostic.
  `G-002` tracks the existing World-selected CUDA PSD offload; `G-001` retains
  admission of future accelerator stages.
- Non-goals: routing device memory through the CPU `MemoryManager`.
- Acceptance evidence: each accelerator row has host no-allocation gates plus a
  device-allocation diagnostic or benchmark packet showing no per-step device
  allocation after bake.
  For `G-002`, begin with fresh owner/process state and nontrivial PSD batches
  above the actual offload threshold. Verify CUDA execution, prepared capacity
  and first-post-bake host/device counters. Two different-sized Worlds must
  match isolated results when interleaved and invoked concurrently, with safe
  scratch ownership and completion before reclamation. Earlier process-global
  buffer warming cannot determine success. This closes only the PSD offload
  slice, not full CUDA deformable simulation.
- Gates: `pixi run lint`, focused CPU tests, and `pixi run -e cuda test-cuda` or
  the backend-specific gate on capable hosts.
- Dependencies: PLAN-030, PLAN-031, PLAN-081, PLAN-083, and PLAN-104.

### WP-122.8 M1 Allocation And Lifecycle Qualification

- Objective/value: prove that the chosen runtime, kernels and full M1 workflow
  honor the storage and isolation contracts together.
- Scope: extend the coverage matrix with explicit serial, parallel range,
  task-graph submission and CUDA rows for the accepted RB corpus; graph/cache
  invalidation, reset/restore and concurrent-state lifetime tests.
- Architecture impact: state isolation, prepared capacities, graph submission
  and device lifetime; update the allocation matrix and architecture assessment.
- Non-goals: weakening existing zero-allocation gates, excluding third-party
  submission from measurement, or qualifying every research family.
- Assumptions/open decisions: capacities are established at bake; setup,
  explicit observations and checkpoint I/O are measured separately. Restoring
  may rebuild scratch/graphs before re-entering the prepared step boundary.
- Acceptance evidence: measure the first post-bake and first post-restore
  prepared step without hidden stepping/prewarm. No World-base/global/raw host
  allocation or device allocation occurs in any selected M1 step path. Include
  changing active contact counts within capacity, two interleaved independent
  states, model rebuild and stale-handle rejection, profiling on/off, nested
  execution, errors/cancellation and completion before teardown. Over-capacity
  work fails explicitly or rebakes outside the step according to contract.
  Record total actual workers and device memory, not only Taskflow's pool size.
  The current constant Taskflow submission allocation floor is an open gap.
- Gates: focused allocation/lifecycle/concurrency tests, `pixi run lint`,
  `pixi run test-all`, `pixi run -e cuda test-all`; record platform interposer
  availability. Skipped allocation/runtime measurements cannot close a row.
- Dependencies: accepted WP-030.5 and WP-041.1; consume the WP-030.4 runtime
  decision. A runtime that fails hard gates must be replaced or excluded from
  the accepted M1 path rather than receiving an undocumented waiver.

## Completion Criteria

This plan is complete when:

- every in-scope row in the coverage matrix is `Closed` with cited post-bake
  tests, or is explicitly moved out of scope by maintainer decision;
- no row relies on unmeasured prewarm simulation steps for its final claim;
- DART 7 migration work has an allocation gate before a migrated path is
  considered promoted into `World::step()`;
- `docs/design/hierarchical_allocator.md`, `docs/onboarding/testing.md`, and the
  coverage matrix agree on the contract; and
- the focused gates named by each closing packet pass.
