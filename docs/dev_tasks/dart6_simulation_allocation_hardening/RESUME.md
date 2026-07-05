# Resume: DART 6 Simulation-Loop Allocation Hardening

## Last Session Summary

WP-D6M.1, WP-D6M.2, WP-D6M.3, WP-D6M.4, WP-D6M.9, the default rigid/native
WP-D6M.5 closure, and WP-D6M.6 first-post-bake allocation gates are landed on
the local branch. The native DART-collision gates now assert zero operator-new,
zero raw malloc, and zero counting-allocator growth for explicit
`World::enterSimulationMode()` followed by the first step, and for implicit
preparation followed by the second step. WP-D6M.7 evidence docs are drafted in
`03-results.md` and `PR_BODY_DRAFT.md`, and `CHANGELOG.md` has a draft DART
6.20 Simulation entry without a PR link. `pixi run test-all` passed after
fixing the zero-DOF body-force reset regression exposed by
`test_IslandDeactivation`.

## Current Branch

`dart6-memory-hardening` (worktree `/home/js/dev/dartsim/dart/task_9-mem620`,
based on `origin/release-6.20`) — local runtime/test/docs commits exist; no PR
is open for this branch yet.

## Immediate Next Step

Finish WP-D6M.7 gates: run `pixi run -e gazebo test-gz`. Then decide how to
handle the timing caveat recorded in `03-results.md` before presenting a broad
performance-speedup claim.

## Context That Would Be Lost

- Maintainer constraints: gz-physics/gz-sim backward compatibility is
  mandatory; explicit simulation preparation must be optional with implicit
  preparation in the first `step()`; additive public API OK; first-post-bake
  strict gate. The landed API name is `World::enterSimulationMode()`, not
  `World::bake()`, because `bake()` is already used by recording.
- The default rigid native-DART-collision hot path is allocation-free after
  explicit or implicit preparation. WP-D6M.6 added strict first-post-bake gates
  and kept Bullet/ODE on World-base-allocator-only compatibility gates because
  global counters include backend internals.
- The final timing packet is mixed. `boxes_headless 8 2000 500` improves from
  9325.991 ms to 8147.949 ms with an empty checkpoint diff, but the full
  Google Benchmark rerun is slower in short-lived contact-container cases.
  Two shortcut attempts were rejected because they weakened the allocation
  gates: skipping implicit solver prewarm left 8 allocations on the implicit
  second native step, and a single prewarm pass left 8 allocations on the
  explicit first post-bake native step.
- DART 7 harness reference: `main:tests/unit/simulation/world/test_world.cpp`
  (operator-new override at 112-527, CountingMemoryAllocator at 772-829).
- Strict gates must use native DART collision scenes (global counters see
  backend-internal allocations otherwise).

## How to Resume

```bash
cd /home/js/dev/dartsim/dart/task_9-mem620
git status && git log -3 --oneline
```

Then: continue with the remaining WP-D6M.7 gates in `README.md`, keeping
authoring and review lanes separate.
