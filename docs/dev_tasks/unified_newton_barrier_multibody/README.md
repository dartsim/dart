# Unified Newton-Barrier Multibody - Dev Task

## Current Status

- [x] Phase 1: promote shared world-primitive math into an internal
      Newton-barrier owner.
  - [x] Add `detail/newton_barrier` owners for primitive distances, clamped-log
        barriers, and tangent stencils.
  - [x] Preserve `detail/deformable_contact` compatibility forwarding so active
        deformable IPC branches keep compiling during reconciliation.
  - [x] Update rigid IPC primitive calls to consume the Newton-barrier owner
        directly.
  - [x] Add cross-variant primitive tests for old/new alias parity, rigid
        consumer parity, and the internal namespace boundary.
  - [x] Record smoke output from the existing distance, barrier, tangent, and
        rigid IPC benchmark binaries without renaming them.
- [x] Phase 2: add the ABD first-slice internal prototype: affine body state,
      affine surface adapter, orthogonality energy, affine primitive/friction
      chain rules, rigid-equivalence tests, and first benchmark packet shape.
  - [x] Add internal `AffineBodyState` and `AffineSurfaceAdapter` owners under
        `detail/` with 12-DOF affine state vectorization and per-vertex
        Jacobians.
  - [x] Map shared Newton-barrier point-triangle, point-edge, edge-edge, and
        point-point primitive rows through affine Jacobians.
  - [x] Add stiff affine orthogonality energy with analytic gradient/Hessian
        coverage.
  - [x] Add rigid-equivalence oracle by projecting affine gradients and
        Hessians onto the rigid tangent space, including the rotation-vector
        curvature term needed to match PLAN-082 rigid IPC.
  - [x] Add the first ABD benchmark packet shape with a matched rigid IPC
        point-triangle oracle row and orthogonality-energy row.
  - [x] Add reduced friction equivalence rows through affine tangent stencils
        for point-point, point-edge, edge-edge, and point-triangle primitives.
  - [x] Promote the first packet into the PLAN-083 manifest as an in-progress
        `abd-alg-affine-body` comparison row. Add
        `pixi run bm-abd-comparison-packet` to generate/validate
        `.benchmark_results/abd_comparison_packet.json`.
- [x] Phase 3: generalize second-use PSD projection and projected-Newton
      contracts when ABD or another solver-family slice needs the shared
      contract; use the PLAN-083 variant consolidation map to keep IPC-family
      responsibilities in the right owner while promoting only proven
      second-use contracts.
  - [x] Promote the first second-use PSD projection contract into
        `detail/newton_barrier` and route rigid IPC plus ABD Hessian projection
        through it with shared tests.
  - [x] Wrap the deformable batched PSD backend behind a
        `detail/newton_barrier` owner and route deformable projected-Newton
        Hessian batches through it while preserving the existing CPU/CUDA
        backend seam.
  - [x] Promote the first shared line-search option/stat contract into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD stats
        accumulation through it with shared tests.
  - [x] Promote the shared line-search positive-step predicate into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD result
        methods through it while keeping hit/limited result ownership
        variant-local.
  - [x] Promote the shared conservative native-CCD option adapter into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD
        line-search queries through it while keeping their CCD implementations
        and limiting-primitive payloads variant-local.
  - [x] Promote the shared line-search step-scale policy into
        `detail/newton_barrier` and route rigid IPC Newton step scaling plus
        deformable/world CCD limiters through it while keeping their result
        payloads and zero-step diagnostics variant-local.
  - [x] Promote the shared native-CCD primitive outcome accounting policy into
        `detail/newton_barrier` and route rigid IPC plus deformable CCD hit,
        miss, indeterminate, and step-bound clamping through it while keeping
        limiting-payload ownership and indeterminate-result policy
        variant-local.
  - [x] Count zero-step indeterminate native-CCD outcomes through the shared
        line-search accounting helper so rigid IPC and deformable CCD
        diagnostics report blocking indeterminate queries consistently while
        keeping their limiting payloads variant-local.
  - [x] Promote the shared Armijo sufficient-decrease and backtracking scalar
        policy into `detail/newton_barrier` and route rigid IPC plus deformable
        projected-Newton line-search checks through it while keeping
        variant-specific acceptance/fallback semantics local.
  - [x] Promote the shared full-step line-search feasibility predicate into
        `detail/newton_barrier` and route rigid IPC kinematic feasibility plus
        deformable CCD result methods through it while keeping limited/hit
        payloads variant-local.
  - [x] Promote shared projected-Newton residual/tolerance helpers into
        `detail/newton_barrier` and route rigid IPC plus deformable
        convergence diagnostics through them while keeping solver status enums
        variant-local.
  - [x] Promote shared lagged-friction work diagnostics into
        `detail/newton_barrier` and route deformable, rigid IPC, and ABD
        friction diagnostics through the same smoothed Coulomb work contract.
  - [x] Promote the first shared Google Benchmark packet row parser into
        `scripts/benchmark_packet_utils.py` and route the ABD comparison packet
        checker plus the Phase 5 GPU packet checker through it while keeping
        packet-specific metadata rules in their owners.
  - [x] Route the Phase 5 CUDA packet writer through the shared Google
        Benchmark canonical row identity helper, removing its duplicate row
        parser while keeping packet-specific max-error extraction local.
  - [x] Promote the first benchmark packet timing schema for per-step counts
        and solver-subphase timing fields into `scripts/benchmark_packet_utils.py`
        while keeping packet-specific required subphases and go/no-go gates in
        their owners.
  - [x] Close the remaining Phase 3 scouting by routing deformable
        projected-Newton backtracking through the shared Newton-barrier default
        scale and rigid IPC's option defaults through the same shared scalar
        constants. The implementation-roadmap Phase 2 shared solver contracts
        are complete; remaining solver result/status payloads and
        packet-specific gates stay variant-local until another phase proves
        identical behavior.
- [ ] Phase 4: expand the unified manifest into diagnostics, benchmark packets,
      CPU/GPU evidence, and visual evidence rows.
- [ ] Phase 5: add runtime and py-demos scenes only after the relevant solver
      slices exist and have correctness/performance evidence.

## Goal

Implement PLAN-083 incrementally until DART owns the IPC solver family through
the shared Newton-barrier primitive layer, affine stiff-body track, CPU/GPU
evidence, paper/deck parity rows, and py-demos examples behind DART-owned DART 7
`World` capabilities without exposing upstream solver names, registries, ECS
storage, or backend resources as public API.

## Non-Goals For Current Internal Slices

- No public API changes.
- No dartpy binding changes.
- No scene, fixture, or py-demos migration.
- No rigid curved-trajectory CCD move.
- No sparse Newton loop merge.
- No rigid IPC default behavior change.
- No ABD runtime stage.
- No GPU speedup claim.

## Key Decisions

- Phase 1 is an internal ownership and contract slice, not a behavior change.
- Batch all work within one implementation-roadmap phase into a single branch
  and PR. Commits within the branch stay atomic and self-describing, but the PR
  is the review unit, not the commit. A phase may split into at most two PRs
  only if it crosses a public-API boundary or touches unrelated CI/build
  infrastructure.
- The old `deformable_contact` include paths remain as forwarding
  compatibility headers to avoid unnecessary PLAN-081 merge conflicts.
- Rigid IPC should include the new Newton-barrier owner directly because it is
  the second active consumer and the reduced-coordinate correctness oracle.
- Existing benchmark binary names stay stable for performance-history
  continuity.
- ABD starts only after the primitive contract has cross-variant tests.
- The first ABD chain-rule slice uses the affine map `x = a + A X`, so its
  Hessian has no transform-curvature term. Rigid Hessian equivalence projects
  onto the rigid tangent space and adds the rotation-vector exponential
  curvature term before comparing against rigid IPC.
- The first second-use friction contract is the shared tangent-displacement
  friction kernel under `detail/newton_barrier`, consumed by rigid IPC and the
  affine primitive-family friction rows.
- Public docs and APIs keep method/capability names DART-owned; internal tests
  and manifests may cite IPC, rigid IPC, ABD, and paper row provenance.
- Treat IPC as the representative solver-family name for PLAN-083 when the
  unified Newton-barrier method is the most advanced shared IPC variant.
  `Newton-barrier` names the implementation contracts and paper lineage;
  variant names such as rigid IPC, deformable IPC, and ABD remain internal
  provenance until shared behavior is proven.
- The first `abd-alg-affine-body` micro-packet is primitive/oracle evidence and
  does not need a two-body affine contact micro-solve before Phase 3. Add a
  solved-state micro-solve only when the next broader ABD packet needs runtime
  residuals rather than primitive, friction, or orthogonality rows.

## Immediate Next Steps

1. Merge the latest `origin/main`, run the Phase 2 validation gates, and open
   one phase-scoped PR for implementation-roadmap Phase 2: Shared Solver
   Contracts. After that PR lands, start implementation-roadmap Phase 3:
   Unified Articulation Constraints; do not treat dev-task Phase 4 manifest
   expansion as the next implementation-roadmap phase.
2. Keep the two-body affine contact micro-solve deferred until the
   `abd-alg-affine-body` row expands beyond the primitive/oracle micro-packet
   and needs a solved-state residual or runtime stepping diagnostic.
3. Promote only the smallest proven shared contract, with cross-variant tests
   showing identical behavior; keep variant-specific terms in their owner plans.
4. Keep runtime stepping, py-demos, and GPU claims out of scope until the
   internal ABD oracle and benchmark packet exist.

## Validation Gates For Current Slices

```bash
pixi run lint
pixi run build-simulation-tests
pixi run test-simulation
pixi run check-api-boundaries
pixi run bm bm_ipc_distance_kernels -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_barrier_kernel -- --benchmark_min_time=0.05s
pixi run bm bm_ipc_tangent_stencil -- --benchmark_min_time=0.05s
pixi run bm bm_rigid_ipc_solver -- --benchmark_min_time=0.05s
```

CUDA evidence is required only if the slice touches the PSD wrapper or another
GPU-backed path:

```bash
pixi run -e cuda test-cuda
```

Phase 1 local evidence:

- `pixi run lint`
- `pixi run build-simulation-tests`
- `pixi run test-simulation`
- `pixi run check-api-boundaries`
- `pixi run bm bm_ipc_distance_kernels -- --benchmark_min_time=0.05s`
- `pixi run bm bm_ipc_barrier_kernel -- --benchmark_min_time=0.05s`
- `pixi run bm bm_ipc_tangent_stencil -- --benchmark_min_time=0.05s`
- `pixi run bm bm_rigid_ipc_solver -- --benchmark_min_time=0.05s`

Phase 2 local evidence so far:

- `pixi run build-simulation-tests`
- `pixi run test-simulation` (64/64)
- `pixi run bm bm_affine_body_dynamics -- --benchmark_min_time=0.05s`
- `pixi run bm-abd-comparison-packet`

Phase 3 line-search option/stat slice local evidence:

- `pixi run lint`
- `pixi run build-simulation-experimental-tests`

Phase 3 PSD backend wrapper slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_deformable_psd_backend test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_deformable_psd_backend|test_world)$'`
- `pixi run test-simulation-experimental` (65/65)
- `pixi run check-api-boundaries`

Phase 3 line-search positive-step predicate slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_newton_barrier_primitives$'`
- `pixi run build-simulation-tests`
- `pixi run test-simulation` (65/65)
- `pixi run check-api-boundaries`
- `pixi run lint`

Phase 3 benchmark packet utility slice local evidence:

- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`
- `pixi run lint`

Phase 3 benchmark row identity slice local evidence:

- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`
- `pixi run lint`

Phase 3 line-search step-scale slice local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`
- `pixi run build`
- `pixi run test-unit`

Phase 3 native-CCD primitive outcome accounting slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_continuous_collision_step test_rigid_ipc_barrier --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_continuous_collision_step|test_rigid_ipc_barrier)$'`
- `pixi run lint`
- `pixi run build`
- `pixi run test-unit`
- `pixi run test-all`
- `pixi run -e cuda test-all` (docs passed with the existing
  `dartpy._world_render_bridge` autodoc warnings)

Phase 3 native-CCD zero-step diagnostic accounting slice local evidence:

- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^test_newton_barrier_primitives$'`

Phase 3 sufficient-decrease policy slice local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`

Phase 3 closeout local evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_rigid_ipc_barrier test_world --parallel 8`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R '^(test_newton_barrier_primitives|test_rigid_ipc_barrier|test_world)$'`

Implementation-roadmap Phase 2 branch-local closeout evidence:

- `pixi run lint`
- `pixi run -- cmake --build build/default/cpp/Release --target test_newton_barrier_primitives test_affine_body_dynamics test_rigid_ipc_barrier test_world test_deformable_body --parallel <safe-jobs>`
- `pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -j <safe-jobs> -R '^(test_newton_barrier_primitives|test_affine_body_dynamics|test_rigid_ipc_barrier|test_world|test_deformable_body)$'`
- `pixi run python -m pytest tests/test_benchmark_packet_utils.py`

## Owner Docs

- PLAN-083 owner:
  [`../../plans/083-unified-newton-barrier-multibody.md`](../../plans/083-unified-newton-barrier-multibody.md)
- IPC-family variant consolidation:
  [`../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
- Implementation roadmap:
  [`../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md`](../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md)
- Primitive promotion slice:
  [`../../plans/083-unified-newton-barrier-multibody/primitive-promotion-slice.md`](../../plans/083-unified-newton-barrier-multibody/primitive-promotion-slice.md)
- ABD first-slice design:
  [`../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md`](../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md)
- Shared primitive audit:
  [`../../plans/083-unified-newton-barrier-multibody/shared-primitive-audit.md`](../../plans/083-unified-newton-barrier-multibody/shared-primitive-audit.md)
- Deformable IPC variant:
  [`../../plans/081-deformable-implicit-barrier-solver.md`](../../plans/081-deformable-implicit-barrier-solver.md)
- Rigid IPC variant:
  [`../../plans/082-rigid-implicit-barrier-contact.md`](../../plans/082-rigid-implicit-barrier-contact.md)
