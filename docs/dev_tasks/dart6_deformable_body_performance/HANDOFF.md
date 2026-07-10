# Handoff - DART 6 deformable body performance

Updated: 2026-07-09

This file is the short, current handoff for a fresh agent. Treat the repository
state as authoritative, then use this note to avoid rediscovering the recent
native-collision slice.

## Current branch state

- Worktree: `/home/js/dev/dartsim/dart/task_2`
- Branch: `wp-db-native-soft-fallback`
- Target base: `origin/release-6.20`
- Base observed for this handoff: `2ba7364b79e`
  (`Optimize native solver-facing manifolds (#3364)`)
- Latest implementation commit: `0ed32afba03`
  (`Tighten native soft fallback hot paths`)
- The branch was local-only when this handoff was written: no upstream and no
  GitHub PR attached to `wp-db-native-soft-fallback`.
- Do not push, open a PR, retrigger reviews, or mutate GitHub state without
  explicit maintainer approval.

The local branch intentionally contains several stacked commits. Because it has
not been pushed, a future agent may clean up or squash the PR history before
first publication, but only after preserving the evidence and rerunning the
required gates on the final history.

## Latest committed slice

Commit `0ed32afba03` is a WP-DB.08 direct-native hot-path slice. It keeps
`SoftMeshShape` and `EllipsoidShape` in the direct
`COLLISION_DETECTOR=native` path through cached DART fallback objects while
tightening the native broadphase/update path:

- recompute fallback-shape traits only when shape identity/version changes;
- use the cached DART fallback transform/AABB for fallback shapes;
- avoid creating the persistent native manifold cache until native-owned
  manifold contacts are actually produced;
- skip native manifold-cache refresh for groups containing soft fallback shapes;
- update native collision groups in one object pass before broadphase refresh;
- use `BruteForceBroadPhase::updateRange()` with an id/order fast path instead
  of unsafe unordered AABB updates;
- use an inline brute-force pair visitor for direct native group collision;
- add `BruteForceBroadPhase.UpdateRangeHandlesOutOfOrderIds` to prove
  out-of-order IDs still update the correct AABBs.

During audit, an unsafe `updateOrdered()` broadphase shortcut was found and
removed before commit. Do not reintroduce ordered AABB updates unless the
collision group and broadphase entry order invariant is proven and tested.

## Validation already run

These checks passed after formatting and before `0ed32afba03` was committed:

```bash
pixi run lint
pixi run cmake --build build/default/cpp/Release \
  --target soft_body_headless BM_INTEGRATION_soft_body \
    UNIT_collision_native_detector_adapter UNIT_collision_native_brute_force \
    test_DARTCollisionDetector test_SoftDynamics INTEGRATION_StepAllocation \
  --parallel 8
pixi run ctest --test-dir build/default/cpp/Release \
  -R 'UNIT_collision_native_brute_force$|UNIT_collision_native_detector_adapter$|test_DARTCollisionDetector$|test_SoftDynamics$|INTEGRATION_StepAllocation$' \
  --output-on-failure
```

Result: `pixi run lint` passed, all requested targets built, and all five
focused CTest entries passed.

## Interrupted comparison run

A current/parent/base comparison was started and then intentionally stopped
after the user asked to stop active implementation progress and update handoff
docs. The processes were killed. No benchmark `summary.md`, `raw/`, or `logs/`
outputs from that attempt should be used as evidence.

The partial artifact directory is:

```text
.benchmark_results/wp-db08-native-soft-fallback-0ed32af-parent-40445e-base-2ba736
```

It contains detached worktrees/build metadata for:

- current `0ed32afba0341b896ee86dd320f6005307d85b10`
- parent `40445efb71e0afdce13d565d1915c7ed48e147f9`
- base `2ba7364b79ecc0aa7a313d2c53d5f7b58099c0df`

The run reached the post-build idleness wait after cold builds. A future agent
may either reuse that output directory for an incremental rerun or choose a new
output directory. Do not cite the interrupted run as benchmark evidence.

## Next required benchmark gate

Run the formal comparison after checking the machine is idle. Prefer a fresh
output directory for final evidence:

```bash
pixi run python scripts/compare_soft_body_performance.py \
  --current HEAD \
  --parent HEAD^1 \
  --base origin/release-6.20 \
  --detectors fcl,dart,native,bullet,ode \
  --threads 1,16 \
  --benchmark-min-time 0.1s \
  --benchmark-repetitions 7 \
  --benchmark-cycles 2 \
  --benchmark-run-order detector \
  --correctness-scenes soft_cubes,soft_bodies \
  --correctness-steps 200 \
  --wait-for-local-dart-builds \
  --idle-max-load-1m 4 \
  --idle-cooldown 5 \
  --output-dir .benchmark_results/wp-db08-native-soft-fallback-0ed32af-parent-40445e-base-2ba736-rerun
```

For PR-quality evidence, do not rely on `--keep-going`; use that only for
diagnostics. The comparison must prove checksum eligibility and show direct
`native` matching or beating every checksum-equivalent backend on the tracked
current rows. The generated `summary.md` includes the table and graph views
needed for the PR description.

## Completion status

Do not mark `docs/dev_tasks/dart6_deformable_body_performance` complete yet.
Open work remains:

- WP-DB.05: adaptive active contact neighborhoods with deterministic hashes.
- WP-DB.06: heap-free contiguous point-mass object storage, retained SoA
  scratch, SIMD investigation through `dart/simd/`, and final idle-host
  threshold evidence.
- WP-DB.07: broader multicore scaling beyond the current small-pair native
  soft-soft worker lane.
- WP-DB.08: final current/parent/base detector matrix, fuller
  triangle/contact-neighborhood coverage, longer deterministic checksum rows,
  and downstream collision/constraint compatibility gates.
- WP-DB.09: paper-parity demos and any required GUI/video evidence.

Before publishing a PR, update `06-pr-evidence.md` with the final comparison
artifact path, summarize the generated graphs, run `pixi run lint`, and get
explicit approval before pushing.
