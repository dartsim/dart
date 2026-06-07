# Resume: Unified Newton-Barrier Multibody

## Current Reality (2026-06-07)

Use this folder's `README.md`, PLAN-083, `docs/plans/dashboard.md`, and the
current code as the live status. The branch-local "Current Branch" section below
is historical handoff context, not current checkout state. Unified
Newton-barrier work should continue by promoting shared primitives and solver
contracts consumed by rigid IPC, deformable IPC, ABD, and future couplers rather
than adding another public solver stack.

The first Phase 3 shared-contract slice promotes fixed-size symmetric PSD
projection into `detail/newton_barrier`. Rigid IPC and ABD now use the same
helper for reduced/affine Hessian projection; this is intentionally smaller
than the existing deformable batched CPU/CUDA PSD backend and does not rename or
generalize that backend yet.

The next Phase 3 slice promotes the first shared line-search option/stat
contract into `detail/newton_barrier`. Rigid IPC and deformable continuous
collision detection now share option defaults, primitive check counters, and the
stats accumulation helper while keeping their distinct result semantics and CCD
implementations in variant-local owners.

The current Phase 3 slice promotes only the shared line-search positive-step
predicate, `allowsPositiveLineSearchStep(stepBound, indeterminate)`, into
`detail/newton_barrier`. Rigid IPC and deformable continuous collision result
methods route through it, but hit/limited fields and limiting-primitive payloads
remain variant-local.

## Last Session Summary

Current slice: the first ABD benchmark packet has been promoted from
smoke-only evidence into the PLAN-083 manifest.
`pixi run bm-abd-comparison-packet` now builds/runs `bm_affine_body_dynamics`, writes
`.benchmark_results/abd_comparison_packet.json`, and validates the affine
point-triangle barrier, matched rigid IPC oracle, and orthogonality-energy rows.
The `abd-alg-affine-body` manifest row is now `in-progress`, explicitly limited
to this internal primitive/oracle micro-packet rather than a runtime ABD solver
or paper-scale claim. The micro-packet does not require a two-body affine
contact micro-solve before Phase 3 shared-contract scouting; add that
solved-state row only when a broader ABD packet needs runtime residuals.

PLAN-083 was created for the unified Newton-barrier multibody solver family,
covering the supplied unified Newton barrier paper, the ABD deck, existing
deformable IPC work, existing rigid IPC work, CPU/GPU evidence, and py-demos
obligations. Phase 1 promoted the pure distance, barrier, and tangent-stencil
primitives into `detail/newton_barrier`; Phase 2 now has an internal
`detail/affine_body_dynamics` foundation with affine state/surface mapping,
four shared primitive barrier chain-rule rows, orthogonality energy, rigid
gradient/Hessian equivalence, affine primitive-family friction equivalence rows
through the shared tangent-displacement friction kernel,
`test_affine_body_dynamics`, and the first `bm_affine_body_dynamics` benchmark
packet.

The first Phase 3 contract slice was merged as PR #2936. It added a shared
fixed-size PSD projection helper, routed rigid IPC and ABD Hessian projection
through it, and added focused Newton-barrier primitive tests for eigenvalue
clamping and input symmetrization.

The line-search stats slice merged as PR #2937. It added
`detail/newton_barrier/line_search.hpp`, aliases rigid IPC and deformable CCD
option/stat types to the shared owner, and keeps line-search result semantics
variant-local.

The current positive-step predicate slice lives on
`simx/shared-newton-barrier-positive-step`. It is intentionally smaller than a
projected-Newton diagnostics merge: current evidence shows rigid and deformable
share the positive-step predicate, but not yet a full projected-Newton status
or solver-loop contract. Local validation passed `test_newton_barrier_primitives`,
`build-simulation-experimental-tests`, `test-simulation-experimental`,
`check-api-boundaries`, and `lint`.

## Current Branch

`simx/shared-newton-barrier-positive-step` - contains the Phase 3 shared
line-search positive-step predicate slice. Verify the exact status with
`git status --short --branch` because this section is a resume snapshot.

## Immediate Next Step

Continue Phase 3 from
[`../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md`](../../plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md):
validate and publish the line-search positive-step predicate helper, then
resume shared-contract scouting from the existing rigid IPC, deformable IPC,
and ABD evidence. Do not add a two-body affine contact micro-solve for the
current `abd-alg-affine-body` micro-packet; add projected-Newton,
line-search result semantics, diagnostics, or benchmark-schema contracts only
after second-use behavior is proven identical across variants. Use
[`../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md`](../../plans/083-unified-newton-barrier-multibody/implementation-roadmap.md)
to keep the Phase 2 packet, shared solver contracts, articulation rows,
restitution work, mixed-domain coupling, CPU/GPU parity, and py-demos rows
sequenced without lowering the final completion target. Add a two-body affine
contact micro-solve only if a later manifest row needs a solved-state residual
or runtime stepping diagnostic. Use the new
[`../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md`](../../plans/083-unified-newton-barrier-multibody/ipc-variant-consolidation.md)
sidecar when deciding whether a primitive/API/benchmark row belongs to
deformable IPC, codimensional IPC, rigid IPC, ABD, PD-IPC GPU, SPB recovery,
VBD/OGC-adjacent work, or shared Newton-barrier infrastructure.

## Context That Would Be Lost

- Rigid IPC now includes the shared Newton-barrier primitive owner directly for
  barrier and tangent math.
- Preserve old deformable-contact include paths as forwarding headers to avoid
  unnecessary conflicts with active PLAN-081 work.
- Keep benchmark binary names such as `bm_ipc_distance_kernels`,
  `bm_ipc_barrier_kernel`, `bm_ipc_tangent_stencil`, and
  `bm_rigid_ipc_solver`.
- The primitive contract has old/new alias parity tests and a rigid consumer
  test in `test_newton_barrier_primitives`.
- `detail/affine_body_dynamics` currently uses the affine map `x = a + A X`;
  its primitive Hessian is a pure `J^T H J` chain rule. The rigid Hessian oracle
  in `test_affine_body_dynamics` projects onto the rigid tangent space and adds
  the rotation-vector curvature correction before comparing to rigid IPC.
- `test_affine_body_dynamics` covers finite-difference vertex Jacobians,
  point-triangle/point-edge/edge-edge/point-point affine barrier derivatives,
  point-triangle/point-edge/edge-edge/point-point affine friction derivatives,
  orthogonality energy derivatives, and rigid value/gradient/Hessian
  equivalence for barrier and friction rows.
- The tangent-displacement friction kernel now lives under
  `detail/newton_barrier` so rigid IPC and ABD share the same Coulomb smoothing
  branch before reduced/affine chain rules.
- `detail/newton_barrier/line_search.hpp` owns the shared line-search
  option/stat types plus the positive-step predicate. Do not move full
  line-search result structs there until rigid, deformable, ABD, or another
  variant prove identical result semantics.
- `bm_affine_body_dynamics` currently contains the first ABD benchmark packet:
  affine point-triangle barrier mapping, matched rigid IPC point-triangle oracle
  row, and orthogonality energy.
- `scripts/check_abd_comparison_packet.py` is the reproducible packet checker
  for those rows; it deliberately does not validate paper-scale ABD scene
  timings yet.
- The current packet is enough to proceed to Phase 3 shared-contract scouting;
  it does not require a two-body solved-state micro-solve first.
- Do not expose `detail/newton_barrier` through public headers or dartpy
  bindings.
- Do not expose ABD through public headers or dartpy bindings until runtime
  and benchmark evidence exists.
- Phase 1 validation passed: `pixi run lint`,
  `pixi run build-simulation-experimental-tests`,
  `pixi run test-simulation-experimental`, `pixi run check-api-boundaries`, and
  the four benchmark smokes listed in the dev-task README.
- Phase 2 validation so far passed: `pixi run build-simulation-experimental-tests`,
  `pixi run test-simulation-experimental` (64/64),
  `pixi run check-api-boundaries`, `pixi run check-docs-policy`,
  `pixi run lint`, and
  `pixi run bm bm_affine_body_dynamics -- --benchmark_min_time=0.05s`.
  The micro-packet slice adds `pixi run bm-abd-comparison-packet`.

## How To Resume

```bash
git status --short --branch
sed -n '1,220p' docs/dev_tasks/unified_newton_barrier_multibody/README.md
sed -n '1,220p' docs/plans/083-unified-newton-barrier-multibody/abd-first-slice-design.md
```

Then verify the current branch with the focused simulation-experimental tests,
API-boundary check, lint, and `pixi run bm-abd-comparison-packet` if that has
not already been done. After this slice, continue with Phase 3 shared-contract
scouting; do not describe the micro-packet as a runtime ABD solver or a
paper-scale performance row.
