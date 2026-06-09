# Resume: Unified Newton-Barrier Multibody

## Current Reality (2026-06-08)

PR granularity: batch all work within one implementation-roadmap phase into a
single branch and PR. Keep commits atomic within the branch. A phase may split
into at most two PRs only when it crosses a public-API boundary or touches
unrelated CI/build infrastructure.

Use this folder's `README.md`, PLAN-083, `docs/plans/dashboard.md`, and the
current code as the live status. The branch-local "Current Branch" section below
is historical handoff context, not current checkout state. Treat IPC as the
representative solver-family name when PLAN-083's unified Newton-barrier method
is the most advanced shared IPC variant. Unified Newton-barrier work should
continue by promoting shared primitives and solver contracts consumed by rigid
IPC, deformable IPC, ABD, and future couplers rather than adding another public
solver stack.

The first Phase 3 shared-contract slice promotes fixed-size symmetric PSD
projection into `detail/newton_barrier`. Rigid IPC and ABD now use the same
helper for reduced/affine Hessian projection; this is intentionally smaller
than the existing deformable batched CPU/CUDA PSD backend and does not rename or
generalize that backend yet.

The PSD backend wrapper slice promotes only the solver-family owner name for
the existing deformable batched CPU/CUDA seam: `detail/newton_barrier` now wraps
the compute PSD backend, and deformable projected-Newton Hessian batching calls
through that internal owner. The core CPU/CUDA backend hooks, acceleration
control, and public compute names stay unchanged.

The sufficient-decrease policy slice promotes only the scalar Armijo/backtracking
policy that rigid IPC and deformable projected-Newton line search already share:
default sufficient-decrease factor, default backtracking scale, factor/scale
sanitization, and the finite candidate-value threshold check. Rigid IPC still
owns best-decreasing fallback, kinematic-candidate rejection, projected-Newton
result status, and solve stats. Deformable IPC still owns its CCD limiter chain,
steepest-descent fallback, minimum-step loop, and per-body solver diagnostics.

A Phase 3 slice promoted the first shared line-search option/stat
contract into `detail/newton_barrier`. Rigid IPC and deformable continuous
collision detection now share option defaults, primitive check counters, and the
stats accumulation helper while keeping their distinct result semantics and CCD
implementations in variant-local owners.

The positive-step predicate slice promoted only
`allowsPositiveLineSearchStep(stepBound, indeterminate)` into
`detail/newton_barrier`. Rigid IPC and deformable continuous collision result
methods route through it, but hit/limited fields and limiting-primitive payloads
remain variant-local.

The native-CCD option adapter slice merged as PR #2942. Rigid IPC and
deformable continuous collision detection now use one `LineSearchOptions` ->
`CcdOption` adapter for non-negative separation/tolerance clamping, positive
iteration count clamping, and conservative advancement selection, while their
CCD implementations, hit/limited result fields, and limiting-primitive payloads
remain variant-local.

The line-search step-scale slice promoted the shared line-search step-scale
policy into `detail/newton_barrier`. Rigid IPC Newton step scaling and
deformable/world CCD limiters now share finite `[0, 1]` step-bound clamping,
safety-scale handling, and the strictly-interior CCD fraction used before
applying a line-search candidate. Variant-local owners still keep their
hit/limited payloads, candidate identities, and zero-step diagnostic counters.

The benchmark-packet utility slice promoted the first shared packet contract:
Google Benchmark row-name canonicalization, timing-unit conversion, median-row
lookup, and timing-field validation now live in
`scripts/benchmark_packet_utils.py`. The ABD comparison packet checker and the
Phase 5 GPU packet checker share those row-level rules while keeping their
packet-specific expected rows, metadata, speedup gates, and evidence flags in
their variant owners. The shared canonical row identity helper is also consumed
by the Phase 5 CUDA packet writer so packet generation and validation strip
Google Benchmark repeat/aggregate suffixes the same way.

The native-CCD primitive outcome accounting scout promoted only the outcome
accounting that is identical across rigid IPC and deformable CCD: hit/miss/
indeterminate counter updates, hit and indeterminate time-of-impact clamping to
`[0, 1]`, and zero-step counting. Limiting-payload ownership,
indeterminate-result policy, and line-search result structs remain
variant-local.

The native-CCD zero-step diagnostic follow-up keeps that shared accounting
contract honest for indeterminate primitive queries: if a native CCD backend can
only prove an indeterminate zero step, `detail/newton_barrier` now increments
the same zero-step counter used for zero-time hits. Rigid IPC and deformable CCD
still own their distinct limiting payloads and indeterminate-result policies.

The Phase 3 closeout routes deformable projected-Newton backtracking through
the shared Newton-barrier default scale and routes rigid IPC's option defaults
through the same shared scalar constants without moving projected-Newton
result/status terminology, line-search result payloads, solver diagnostics, or
additional benchmark-schema contracts out of their variant owners.

The implementation-roadmap Phase 2 closeout is now branch-local on
`simx/plan083-phase-2-shared-solver-contracts`: full-step line-search
feasibility, projected-Newton residual/tolerance diagnostics, lagged-friction
work diagnostics, and benchmark packet timing schema helpers are shared under
their internal owners with focused cross-variant tests. This completes the
Shared Solver Contracts phase; do not open more sub-item PRs for this phase.

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

The positive-step predicate slice merged as PR #2938. It promoted
`allowsPositiveLineSearchStep(stepBound, indeterminate)` into
`detail/newton_barrier`, routing rigid IPC and deformable CCD result methods
through it without moving hit/limited result ownership.

The benchmark-packet utility slice merged as PR #2940. It promoted
Google Benchmark row parsing into `scripts/benchmark_packet_utils.py`, routing
the ABD comparison packet checker plus the Phase 5 GPU packet checker through
the shared utility while keeping packet-specific metadata and gates in their
owners. The follow-up row-identity slice routes the Phase 5 CUDA packet writer
through the same canonical row identity helper and removes its duplicate parser.
Focused local validation passed `pixi run python -m pytest
tests/test_benchmark_packet_utils.py` and `pixi run lint`.

The line-search step-scale slice merged as PR #2943. It is intentionally
smaller than a line-search result or projected-Newton diagnostics merge:
current evidence shows rigid and deformable share finite step-bound scaling and
the strictly-interior CCD fraction, but not yet full line-search result
payloads, projected-Newton status terminology, or solver-loop diagnostics.
Focused local validation passed `pixi run lint`, the focused
`test_newton_barrier_primitives` / `test_rigid_ipc_barrier` / `test_world`
CTest entries, `pixi run build`, and `pixi run test-unit`.

The native-CCD primitive outcome accounting slice merged as PR #2945 after PR
#2943 landed. Focused local validation passed the
`test_newton_barrier_primitives`, `test_continuous_collision_step`, and
`test_rigid_ipc_barrier` build/CTest entries plus `pixi run lint`. Stronger
local validation also passed `pixi run build`, `pixi run test-unit`,
`pixi run test-all`, and `pixi run -e cuda test-all`. Both full gates passed;
the docs phase still emits the existing `dartpy._world_render_bridge` autodoc
warnings.

The PSD backend wrapper slice is on
`simx/shared-newton-barrier-psd-backend-wrapper`, merged as PR #2946 after
PR #2945 landed. Focused local validation passed
`test_deformable_psd_backend` and `test_world` build/CTest entries.

The sufficient-decrease policy slice is on
`simx/shared-newton-barrier-sufficient-decrease`, retargeted to `main` after
PR #2946 landed. Focused local validation passed `pixi run lint` plus the
`test_newton_barrier_primitives`, `test_rigid_ipc_barrier`, and `test_world`
build/CTest entries.

## Current Branch

`simx/plan083-phase-2-shared-solver-contracts` - contains the
implementation-roadmap Phase 2 shared solver contract closeout. It should become
one phase-scoped PR after merging the latest `origin/main` and rerunning the
required gates.

## Immediate Next Step

Merge the latest `origin/main` into
`simx/plan083-phase-2-shared-solver-contracts`, rerun lint and the focused Phase
2 validation gates, then open one phase-scoped PR for implementation-roadmap
Phase 2. After that PR lands, start implementation-roadmap Phase 3: Unified
Articulation Constraints. Do not start dev-task Phase 4 manifest expansion as a
substitute for the roadmap Phase 3 articulation work.

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
  option/stat types, positive/full-step predicates, the conservative native CCD
  option adapter, and the step-scale helpers. Do not move full line-search
  result structs there until rigid, deformable, ABD, or another variant prove
  identical result semantics.
- `scripts/benchmark_packet_utils.py` owns the shared Google Benchmark row
  parsing utilities plus the per-step/subphase timing schema for packet
  validators and writers. Keep packet-specific metadata and go/no-go gates in
  each checker until more variants prove identical semantics.
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
  `pixi run build-simulation-tests`,
  `pixi run test-simulation`, `pixi run check-api-boundaries`, and
  the four benchmark smokes listed in the dev-task README.
- Phase 2 validation so far passed: `pixi run build-simulation-tests`,
  `pixi run test-simulation` (64/64),
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

Then merge the latest `origin/main`, verify the current branch with lint and
the focused Phase 2 simulation/Python tests, and open one PR for the whole
implementation-roadmap Phase 2. After that PR lands, continue with roadmap Phase
3 articulation constraints; do not describe the micro-packet as a runtime ABD
solver or a paper-scale performance row.
