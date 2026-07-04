# RESUME — DART 6 performance generalization

Read [README.md](README.md) first (envelope, gates, success criteria,
decisions D1–D8), then the lane doc of the packet you claim, then update
[07-orchestration-dashboard.md](07-orchestration-dashboard.md). Read the
prior-art inventory in
[01-baseline-evidence.md](01-baseline-evidence.md) before claiming any
packet that overlaps the `origin/perf/dart6-*` experiment branches.

## Next packet

**WP-PG.01 — Round-2 baseline evidence packet**
([06-infra-evidence-lane.md](06-infra-evidence-lane.md)). Everything else
depends on it. Branch `wp-pg-01-baseline-evidence` off
`origin/release-6.20`; capture the exact cell matrix in
[01-baseline-evidence.md](01-baseline-evidence.md) (canonical commands
are spelled there, including `pixi run -e gazebo download-gz-sim` for the
3k scenes); triage the six prior-art branches; commit the guard-row
tables into that doc; PR title `WP-PG.01: Round-2 baseline evidence
packet`.

Available in parallel after (or alongside, in a second session):
WP-PG.02, WP-PG.03, WP-PG.40 (no deps; resolves D1/D2), then WP-PG.10,
WP-PG.11, WP-PG.20, WP-PG.22, WP-PG.30, WP-PG.31. Packets marked
blocked/gated (PG.04, PG.13, PG.14, PG.15, PG.23, PG.33, PG.41, PG.42)
must not be claimed until their decision or evidence gate lands.

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
- 2026-07-04: WP-PG.40 claimed and drafted on branch
  `wp-pg-40-simd-contracts` (worktree, off `origin/release-6.20`).
  Produced [08-simd-contracts.md](08-simd-contracts.md): D1/D2 proposals
  awaiting maintainer ratification, plus concrete evidence that (a)
  `dart::simd`'s own `fmadd`/`fmsub` are not yet bit-identical across the
  SSE4.2/AVX backends `ci_simd.yml` already builds (no-FMA fallback takes
  an unfused two-rounding path), and (b) `dart::simd` resolves to its
  Scalar backend in every default/packaged DART build today (no CI
  workflow outside `ci_simd.yml` passes ISA flags to the `dart` target),
  which sharpens D2 considerably. Compile-only prototype at
  [prototypes/wp_pg_40_cross3_batch.hpp](prototypes/wp_pg_40_cross3_batch.hpp)
  (+ `_test.cpp`) demonstrates the bit-equality mechanism against the
  already-merged SoA batch `dart::simd::cross()`; verified with
  `g++ -fsyntax-only -Wall -Wextra -Werror` under default,
  `-DDART_SIMD_FORCE_SCALAR`, and `-mavx2 -mfma` — all three pass. No
  cmake/pixi builds or test executions were run (see the decision doc's
  "Hard constraints" section); the harness's assertions have not been
  executed, only compiled. WP-PG.41/42 remain blocked pending D1/D2
  ratification.
