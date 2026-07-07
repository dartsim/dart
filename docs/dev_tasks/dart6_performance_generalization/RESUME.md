# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packets

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

**WP-PG.01 is captured** (branch `wp-pg-01-baseline-evidence`, PR
pending) — guard rows, profile splits, and prior-art triage are in
[01-baseline-evidence.md](01-baseline-evidence.md). Its evidence changed
priorities: the dense-pile fixture is 88.1% Dantzig solve-proper (D3
revisit trigger FIRED; WP-PG.12 deprioritized), and the active-3k
many-islands regime is ~50% integration (WS-C is the lever there).

Claimable now, in priority order:

1. **WP-PG.30/31** (WS-C — highest evidence-backed default-on value:
   integration/per-skeleton overhead on many-islands scenes; mind the
   prior-art rejection caveat in the packet).
2. **WP-PG.11** (WS-A — includes re-measuring the two mined
   single-reactive commits; hash-preserving).
3. **WP-PG.21** re-scope and **WP-PG.22** (WS-B — WP-PG.20 is PR
   pending and already absorbed the pair-keyed map/pruning sweep).
4. **WP-PG.03**, **WP-PG.02** (independent). WP-PG.40 is
   folded into WP-PG.42 per maintainer direction.

Blocked/gated (do not claim): PG.04 (D4), PG.12 (evidence), PG.13
(PG.10 census), PG.14 (D3 — now urgent), PG.15 (D7 — now urgent), PG.23
(D8), PG.33, PG.41. PG.42 is currently claimed on
`wp-pg-42-soa-broadphase-simd`. **D3 and D7 are the decisions that unblock
the dense-pile fixture**; everything claimable above serves the many-islands
and ODE regimes meanwhile.

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

D1 (SIMD FP contract), D2 (ISA delivery), D3 (matrix-free — **revisit
trigger fired by WP-PG.01**: solve-proper is 88.1% of the dense-pile
step), D4 (executor tooling), D5 (ODE lane depth = PG.20/21/22), D7
(penetration-creep remediation — **S6 confirms 0.362 m penetration,
0/71 resting at baseline**), D8 (manifold reduction now vs WS-F phase 3)
— see README "Open decisions". D3 and D7 are now the highest-leverage
decisions.

## Session log (round-2 execution)

- 2026-07-04/05: WP-PG.01 executed on `wp-pg-01-baseline-evidence`:
  original matrix/profile/dashboard capture on `origin/release-6.20`
  @ `5bee91ad6be`, then current-base guard refresh on
  `origin/release-6.20` @ `b9e6910c066`. Full S1–S6 guard matrix is
  recorded in 01-baseline-evidence.md; profile splits recorded (P1
  dense-pile: solve 88.1% / assembly 7.4%; P2 active-3k: integration
  50.1%); six prior-art branches triaged (two unmerged hash-preserving
  solver wins queued into WP-PG.11; one branch recommended for deletion).
- 2026-07-06/07: WP-PG.20 executed on
  `wp-pg-20-ode-history-spans`: ODE contact-history spans, pair-keyed
  history map, and current-frame pruning sweep. Base/WP hashes were
  bit-identical on the required guard rows; dense ODE timing improved
  from 194.2 to 17.6 ms/step on `S3_ode`, and the controlled-load crash
  gate was 30/30 clean on both arms. WP-PG.21 needs re-scope after this
  PR because the map/pruning part is already included.

## Session log

- 2026-07-04: WP-PG.30 executed (Codex worker + orchestrator review):
  version-keyed cached root-FreeJoint classification; four dynamic_cast
  sites removed; part (b) FD fast path analyzed and skipped with in-code
  rationale (recursion already minimal; round-1 landmine avoided).
  Adversarial review: 4 findings fixed (write ordering, zero-body
  contract, private section, getRootJoint reuse). Evidence: 8/8
  bit-identical hashes; win concentrated in many-islands regime.

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
