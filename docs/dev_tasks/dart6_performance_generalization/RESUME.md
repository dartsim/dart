# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packets

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
3. **WP-PG.20** then **WP-PG.21** (WS-B — the 12x active-3k ODE gap).
4. **WP-PG.40** (WS-D design packet; resolves D1/D2), **WP-PG.03**,
   **WP-PG.02**, **WP-PG.22** (independent).

Blocked/gated (do not claim): PG.04 (D4), PG.12 (evidence), PG.13
(PG.10 census), PG.14 (D3 — now urgent), PG.15 (D7 — now urgent), PG.23
(D8), PG.33, PG.41, PG.42. **D3 and D7 are the decisions that unblock
the dense-pile fixture**; everything claimable above serves the
many-islands and ODE regimes meanwhile.

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
(penetration-creep remediation — **S6 confirms 0.387 m penetration,
0/71 resting at baseline**), D8 (manifold reduction now vs WS-F phase 3)
— see README "Open decisions". D3 and D7 are now the highest-leverage
decisions.

## Session log (round-2 execution)

- 2026-07-04: WP-PG.01 executed on `wp-pg-01-baseline-evidence`
  (`origin/release-6.20` @ `5bee91ad6be`): full S1–S6 matrix captured
  (zero row failures), profile splits recorded (P1 dense-pile: solve
  88.1% / assembly 7.4%; P2 active-3k: integration 50.1%), six prior-art
  branches triaged (two unmerged hash-preserving solver wins queued into
  WP-PG.11; one branch recommended for deletion), dashboard artifacts
  generated.

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
