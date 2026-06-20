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
- Progress snapshot: 14 of 18 matrix rows are closed with first-post-bake
  evidence. `L-001` is now closed: the legacy `dynamics::Skeleton` → DART 7
  `World` model-loading bridge (`dart::simulation::io::addSkeleton`) has
  first-post-bake world-base, global-heap, and raw-malloc gates over an imported
  scene, with the source-model translation kept in the pre-bake configuration
  phase. There are no remaining implementation-capacity rows for currently
  selectable DART 7 CPU `World::step()` paths: `D-004` is closed by routing
  systems above the retained dense-direct cutoff to the sparse iterative path
  instead of Eigen sparse-direct numeric factorization. The remaining open rows
  are promotion-gated rows `M-004`, `F-002`, and `G-001`; they stay owned by
  their named plans until the corresponding solver, derivative, or accelerator
  path is promoted into `World::step()`. No new `docs/dev_tasks/` folder is
  needed for those rows while this plan and its coverage matrix remain the
  durable owner.

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
- Third-party internal allocations that DART cannot control, unless a built-in
  DART 7 path can avoid them by preparing reusable DART-owned scratch at bake.

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

### WP-122.1 Harness Manifest And First-Step Gates

- Objective: make the no-allocation harness reusable and make final evidence
  unambiguous.
- Scope: consolidate or wrap the existing world-base, global-heap, and raw-malloc
  test helpers so DART 7 simulation tests can express "post-bake first step"
  without copy/paste; add a lightweight checker or meta-test that verifies each
  closed matrix row cites an existing test; relabel existing prewarm-based raw
  gates as steady-state until their underlying path bakes all scratch.
- Non-goals: proving every domain row in this packet; changing the DART 6
  compatibility lane.
- Acceptance evidence: the helper/API name is documented in
  `docs/onboarding/testing.md`; the coverage matrix distinguishes final,
  steady-state-only, and open rows; a focused test proves the helper fails when
  an allocation is injected after bake.
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
  AVBD/VBD rows that are already selectable in `World::step()`, and future
  unified Newton-barrier multibody rows from PLAN-083.
- Non-goals: DART 6 articulated-body parity paths outside the DART 7 `World`.
- Acceptance evidence: each promoted DART 7 row in the matrix cites post-bake
  world-base/global/raw gates, including contacts, motors, loop closures, and
  actuator combinations that use dynamic Eigen storage.
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
- Dependencies: PLAN-110 and `docs/dev_tasks/differentiable_simulation/`.

### WP-122.6 Loader And Migration Acceptance Gates

- Objective: prevent DART 7 model-loading migrations from bypassing the
  no-allocation contract.
- Scope: URDF, SDF, MJCF, SKEL, USD, and skeleton-loading bridges that build a
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
- Non-goals: routing device memory through the CPU `MemoryManager`.
- Acceptance evidence: each accelerator row has host no-allocation gates plus a
  device-allocation diagnostic or benchmark packet showing no per-step device
  allocation after bake.
- Gates: `pixi run lint`, focused CPU tests, and `pixi run -e cuda test-cuda` or
  the backend-specific gate on capable hosts.
- Dependencies: PLAN-030, PLAN-031, PLAN-081, PLAN-083, and PLAN-104.

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
