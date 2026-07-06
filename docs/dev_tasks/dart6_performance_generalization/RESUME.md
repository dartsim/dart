# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packet

**Current claimed packet: WP-PG.42 — SoA broadphase sweep batching**
([05-simd-enablement-lane.md](05-simd-enablement-lane.md)). Branch
`wp-pg-42-soa-broadphase-simd` off `origin/release-6.20` carries the
first production `dart/simd` consumer in DART 6: AVX-width finite
broadphase candidate screening in the existing DART detector, with
scalar/SSE/NEON builds preserving the previous scalar sweep shape. The
standalone WP-PG.40 PR #3270 was closed by maintainer direction; its
D1/D2 evidence rides with this actual SIMD-kernel PR.

Immediate next step: refresh the local evidence after any edits, including
the focused detector test, scalar/SSE4.2/AVX/AVX2 SIMD workflow-equivalent
matrix, contact-container macro rows, and the `--profile` finite-finite
sweep share now exposed by this branch's text-profiler labels.

After WP-PG.42 lands, return to **WP-PG.01 — Round-2 baseline evidence
packet** ([06-infra-evidence-lane.md](06-infra-evidence-lane.md)) unless a
newer maintainer direction overrides it. Branch `wp-pg-01-baseline-evidence`
off `origin/release-6.20`; capture the exact cell matrix in
[01-baseline-evidence.md](01-baseline-evidence.md), triage the six
prior-art branches, and commit the guard-row tables into that doc.

Available in parallel after (or alongside, in a second session):
WP-PG.02, WP-PG.03, then WP-PG.10, WP-PG.11, WP-PG.20, WP-PG.22,
WP-PG.30, WP-PG.31. WP-PG.40 is folded into WP-PG.42. Packets marked
blocked/gated (PG.04, PG.13, PG.14, PG.15, PG.23, PG.33, PG.41) must not
be claimed until their decision or evidence gate lands.

## Verify commands (every packet)

```bash
pixi run lint          # auto-fix
pixi run check-lint    # verify — matches CI's Check Lint
pixi run cmake --build build/default/cpp/Release --target ALL --parallel 8   # capped build
DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz   # collision/constraint/solver/header packets
pixi run test-eigen-overalignment                 # allocation/alignment packets
# Determinism guard + benchmark evidence: see 01-baseline-evidence.md
```

(`DART_PARALLEL_JOBS` is honored by the gz and eigen-overalignment
scripts; `pixi run test-all` runs unbounded `cmake --build ... -j`, hence
the explicit `--parallel 8` build command.)

## Standing decisions awaiting maintainer

D1 (SIMD FP contract), D2 (ISA delivery), D3 (matrix-free opt-in — see
revisit trigger), D4 (executor tooling), D5 (ODE lane depth =
PG.20/21/22), D7 (penetration-creep remediation policy), D8 (manifold
reduction now vs WS-F phase 3) — see README "Open decisions".

## Session log

- 2026-07-04: Folder created on branch `perf/dart6-3056-performance-plan`
  (planning PR). Investigation evidence: issue #3056 reopened 2026-06-30;
  round-1 stack inventory reconstructed; active-regime baseline probe and
  Dantzig-share profile smoke recorded in 01-baseline-evidence.md.
- 2026-07-04: Incorporated the #3209 root-cause findings (penetration
  creep vs island-rest veto → WP-PG.15/D7; ODE cylinder-fallback cap
  flood → WP-PG.23/D8 + ODE-row validity rule) and the round-1 experiment
  branch handoff (prior-art inventory + rejected-experiments list).
  Adversarial plan review applied (sequencing consistency, WP-PG.11
  re-scope, WP-PG.13 premise correction, success criteria added).
- 2026-07-05: WP-PG.40 standalone PR #3270 closed per maintainer
  direction. WP-PG.42 claimed on `wp-pg-42-soa-broadphase-simd`; live
  WS-F check found #3281 merged only internal native collision math, so
  this branch remains the DART 6 detector SIMD-consumer path.
