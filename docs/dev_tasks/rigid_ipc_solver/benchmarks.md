# Rigid IPC Benchmarks And Performance Methodology

This document tracks the performance pillar of PLAN-082 (Workstream 7). The
maintainer directive is explicit: the rigid IPC path must eventually **beat the
current DART rigid contact path, the audited reference implementation, and the
paper-reported numbers**, with any slower row carrying an explicit accepted
tradeoff rather than a parity claim.

This is a living tracker, not a parity claim. Comparison against external
baselines is gated on completing the algorithm's correctness (rigorous interval
CCD, corpus parity, production convergence). Until then these numbers exist to
(a) establish a DART-internal baseline, (b) catch regressions, and (c) point at
the next optimization.

## Harness

- C++ Google Benchmark target: `tests/benchmark/simulation/experimental/bm_rigid_ipc_solver.cpp`
  (registered as `bm_rigid_ipc_solver`).
- Run: `pixi run bm --target bm_rigid_ipc_solver --build-type Release`
- JSON capture (for regression gates / manifest evidence):
  `pixi run bm --target bm_rigid_ipc_solver --build-type Release -- \`
  `--benchmark_out=.benchmark_results/rigid_ipc_solver.json --benchmark_out_format=json`

Cases covered:

- `BM_RigidIpcReducedBarrier_{PointTriangle,PointEdge,EdgeEdge,PointPoint}` —
  per-primitive reduced-coordinate barrier value/gradient/Hessian, the inner
  hot kernels.
- `BM_RigidIpcAssembleBarrierSystem/<N>` — scene-level sparse assembly over `N`
  dynamic triangles above one static triangle (`Complexity()` enabled).
- `BM_RigidIpcProjectedNewtonSolve_TwoBody/<iters>` — full projected-Newton
  barrier solve on a two-body point contact.
- `BM_RigidIpcLineSearchStepBound` — conservative curved-CCD line-search bound
  for a crossing triangle pair.

## Comparison baselines (the bar to beat)

1. **Current DART rigid contact path** — the sequential-impulse
   `RigidBodyContactStage`. Same-scene wall-clock per `World::step()`. This is
   the in-repo incumbent and the first thing the IPC path must not regress
   without an accepted tradeoff.
2. **Audited reference implementation** — `ipc-sim/rigid-ipc` at
   `23b6ba6fbf8434056444ae106356fd2209136988` (checkout `/tmp/rigid-ipc`). Its
   harness is `tools/benchmark.py` (and `tools/scalability.py`), which steps the
   fixture corpus and emits per-scene CSV: body/vertex/edge/face counts,
   timestep, `dhat`, `mu`, friction iterations, average contacts, and solve
   time. Scene families live under `tools/benchmarks/` (chains, codimensional,
   friction, mechanisms, unit-tests) and `comparisons/`.
3. **Paper-reported numbers** — the rigid IPC paper's scene families and timing
   tables. Recorded per-scene with the paper's parameters.

Methodology: port matched scenes into DART benchmarks, run all three under the
same host, normalize by scene parameters, and record the ratio. A row is a
performance win only when DART is faster (or within an explicitly accepted
tradeoff) on the same scene at matched accuracy/`dhat`.

## Scaffold baseline snapshot (indicative, not authoritative)

Captured 2026-05-28, host `32 x 5300 MHz` (L3 36 MiB), Release, CPU scaling
enabled (noisy), `--benchmark_min_time=0.05s`. Re-measure before acting on any
single number.

| Benchmark                                | Time     |
| ---------------------------------------- | -------- |
| ReducedBarrier PointTriangle             | ~7.9 µs  |
| ReducedBarrier PointEdge                 | ~8.2 µs  |
| ReducedBarrier EdgeEdge                  | ~7.3 µs  |
| ReducedBarrier PointPoint                | ~6.0 µs  |
| AssembleBarrierSystem N=1 (broad-phase)  | ~0.21 ms |
| AssembleBarrierSystem N=8 (broad-phase)  | ~3.5 ms  |
| AssembleBarrierSystem N=32 (broad-phase) | ~14.7 ms |
| ProjectedNewtonSolve (8 iters, two-body) | ~47 µs   |
| LineSearchStepBound (triangle crossing)  | ~16 µs   |

## Resolved performance findings

- **Scene assembly was O(N²); now O(N).** The original `assembleRigidIpcBarrierSystem`
  evaluated every surface-primitive pair with no broad phase
  (`~63098 · N²`, RMS 7%). A conservative world-AABB cull now skips surface pairs
  whose AABB lower-bound distance already reaches the activation distance (where
  every barrier is provably inactive), so each body only does the expensive
  barrier evaluation against its spatial neighbors. Re-measured complexity:
  `~455531 · N` (RMS 4%) — linear. At N=32, ~64 ms dropped to ~14.7 ms (~4.4x),
  widening with N. The cull is behavior-preserving (covered by
  `RigidIpcBarrier.SceneAssemblyBroadPhaseIsBehaviorPreserving`).

## Open performance findings

- **Pair enumeration is still all-pairs O(N²) AABB tests.** The cull removes the
  dominant per-pair barrier work but still iterates every surface pair to test
  AABBs (cheap, ~ns each, not dominant at the measured N). A spatial index
  (uniform grid / sort-and-sweep, reusing the deformable candidate-set pattern)
  is the follow-up to make the enumeration itself sub-quadratic for large scenes.
- **Per-primitive barrier kernels cost ~6–8 µs.** Dominated by the
  reduced-coordinate chain rule plus PSD eigen-projection. A candidate later
  optimization now that the broad phase bounds the active set.

## Status against the manifest

The 8 `benchmark-script` rows and 77 `comparison` rows in
`docs/plans/082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json`
stay `planned` until DART has a matching benchmark target, command evidence, and
a recorded comparison packet against the baselines above. `bm_rigid_ipc_solver`
is the first DART-owned rigid IPC benchmark; it does not yet map to a specific
upstream `tools/benchmark.py` scene, so no manifest row is promoted yet.
