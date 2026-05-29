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

## DART rigid path comparison (baseline #1)

`bm_rigid_ipc_solver` includes a same-scene per-`World::step()` comparison of the
incumbent sequential-impulse rigid path against the opt-in rigid IPC path,
selected through the public `World::setRigidBodySolver()`, on a stack of dynamic
boxes resting in light contact above a static ground box
(`BM_RigidWorldStep_SequentialImpulse` vs `BM_RigidWorldStep_Ipc`). The contact
models differ (velocity-level impulses vs position-level barriers), so this is a
throughput comparison of advancing the same scene one step, not a
matched-accuracy claim.

Snapshot (2026-05-28, host `32 x 5300 MHz`, Release, CPU scaling enabled; rigid
IPC column after the assembly and line-search broad-phase culls):

| Boxes | Sequential impulse | Rigid IPC | IPC slower by |
| ----- | ------------------ | --------- | ------------- |
| 1     | ~45 µs             | ~15 ms    | ~330x         |
| 2     | ~51 µs             | ~51 ms    | ~1000x        |
| 4     | ~57 µs             | ~123 ms   | ~2160x        |

The broad-phase culls (assembly + line search) shaved ~10–20% off the earlier
rigid IPC step (was ~18 ms / ~65 ms / ~137 ms) — real but incremental.

Reading: the rigid IPC path is currently ~3 orders of magnitude slower than the
incumbent and scales super-linearly (~2x per added box) while sequential impulse
is near-flat. This is the expected starting point for a projected-Newton barrier
scaffold — per step it runs many ~6-8 us primitive-barrier kernels across Newton
iterations, friction passes, and CCD line searches. It quantifies the
optimization gap to close before any reference/paper comparison is meaningful;
the next targets (per-primitive kernel cost, denser-system Newton solve, warm
starting, active-set reuse) follow from here.

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

## Optimization roadmap (CPU and GPU)

The maintainer directive is to beat the incumbent, the reference, and the paper,
optimizing both CPU and GPU. Baseline #1 shows the rigid IPC path is ~3 orders
of magnitude slower per step than the incumbent, so the climb is large and
sequenced. Correctness gates come first, because a performance win on a scene
the solver cannot correctly run is not a win.

Order of execution (bounded slices, each benchmarked and regression-guarded):

1. **CPU hot-path, behavior-preserving first.**
   - [DONE] Swept broad-phase cull in the line search. A correct cull cannot use
     endpoint-union AABBs (a rotating body swings vertices outside the box
     spanned by its start/end poses — the mid-step contact the curved CCD must
     catch). The landed cull instead keeps each surface's START AABB and expands
     the reach by the curved-trajectory speed bound `rigidIpcPointTrajectorySpeedBound`
     (`||Δpos|| + ||Δrot||·||vertex||`, the same bound the per-primitive CCD
     uses), plus the minimum separation and the CCD convergence tolerance. An
     adversarial verification pass (>100M numerical samples plus a code audit)
     confirmed the rotation bound and caught an off-by-`convergeAbs` reach bug
     that was fixed before commit. Guarded by anti-tunneling (180° rotation
     through a static point), far-skip, and tolerance-band regressions.
   - Spatial index (uniform grid / sort-and-sweep) so pair enumeration itself
     is sub-quadratic, reusing the deformable candidate-set pattern.
   - Per-primitive kernel cost (~6–8 µs): reduce the reduced-coordinate chain
     rule and PSD eigen-projection cost (skip projection when the local Hessian
     is already PSD; cheaper 12x12 projection); these touch numerics, so gate on
     finite-difference and solver regressions.
2. **CPU solver-structure, behavior-aware.**
   - Warm-start the projected-Newton solve from the previous step's pose delta
     and active set; reuse the active set across iterations where the barrier
     stays active; lag assembly recomputation. Each changes the iterate path, so
     gate on convergence regressions and the activated-contact runtime tests.
3. **Correctness gates (run in parallel; block reference/paper claims).**
   - Rigorous interval-arithmetic CCD and direct-row corpus parity vs the
     audited reference; production convergence criteria; robust multi-body
     contact. Until these land, reference/paper comparisons are not meaningful.
4. **Reference and paper comparison.**
   - Port matched `tools/benchmark.py` scene families and the paper tables into
     DART benchmarks; record per-scene ratios; a win requires faster-at-matched-
     accuracy or an explicitly accepted tradeoff.
5. **GPU, private and benchmark-gated (PLAN-082 Workstream 7).**
   - GPU work stays out of the public facade (no device/stream/memory-pool
     types) and follows the established Phase-5 CUDA pattern
     (`build-cuda`, `lint-phase5-cuda-*`, `bm_cuda_*`). It is gated until the CPU
     algorithm has a representative, correct workload — a GPU port of an
     immature solver optimizes the wrong thing. Target the data-parallel hot
     path (per-primitive barrier/friction kernels, CCD) once the CPU active set
     and convergence are stable, with CPU-vs-GPU benchmark packets.

## Status against the manifest

The 8 `benchmark-script` rows and 77 `comparison` rows in
`docs/plans/082-rigid-implicit-barrier-contact/rigid_ipc_fixture_manifest.json`
stay `planned` until DART has a matching benchmark target, command evidence, and
a recorded comparison packet against the baselines above. `bm_rigid_ipc_solver`
is the first DART-owned rigid IPC benchmark; it does not yet map to a specific
upstream `tools/benchmark.py` scene, so no manifest row is promoted yet.
