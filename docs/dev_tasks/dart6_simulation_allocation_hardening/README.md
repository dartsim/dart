# DART 6 Simulation-Loop Allocation Hardening — Dev Task

Port the DART 7 (PLAN-122) memory-management architecture back to the DART 6
LTS line (`release-6.20`) so the simulation loop performs zero heap allocation
in DART-owned code, with measured baseline-vs-improved performance evidence.

## Current Status

- [x] Phase 0: Territory scout (DART 6 allocation map, DART 7 porting map,
      benchmark/CI surface) and maintainer decision interview
- [x] Phase 1: Evidence foundation — allocation-count test harness port +
      baseline timing/allocation packet (WP-D6M.1, WP-D6M.2)
- [x] Phase 2: Runtime port — allocator primitives, World-owned
      MemoryManager, optional bake API (WP-D6M.3, WP-D6M.4)
- [x] Phase 3: Hot-path closure — solver/step scratch through retained
      scratch, remaining default rigid/native allocation sites closed
      (WP-D6M.5)
- [x] Phase 4: Gates — first-post-bake allocation gates in ctest (WP-D6M.6)
- [ ] Phase 5: Evidence + close — improved performance packet, gz-physics
      compatibility, changelog, PR body with real numbers, cleanup
      (WP-D6M.7, WP-D6M.8)
      - WP-D6M.7 evidence docs and changelog draft are present in
        `03-results.md`, `PR_BODY_DRAFT.md`, and `CHANGELOG.md`.
      - `pixi run test-all` passed after the zero-DOF body-force reset fix.
      - Remaining WP-D6M.7 gate: `pixi run -e gazebo test-gz`.
      - Timing caveat: the allocation contract is proven, but the final
        Google Benchmark timing packet is mixed. See `03-results.md` before
        making a broad performance-speedup claim.

## Goal

Same-shape `World::step()` on `release-6.20` allocates zero heap memory in
DART-owned code after simulation preparation (explicit
`World::enterSimulationMode()` or the implicit preparation in the first
`step()`), enforced by CI gates, with measured performance
improvement reported as baseline-vs-improved numbers in the PR description.

## Specification Intake

- **Value:** DART 6 LTS users (including Gazebo via gz-physics) get lower and
  more predictable step latency, no steady-state heap traffic, and a durable
  allocation regression gate; the DART 6 line converges on the proven DART 7
  memory architecture, easing future maintenance.
- **Scope:** `dart/common/` (FrameAllocator port, MemoryManager Frame role,
  FreeListAllocator growth policy), `dart/simulation/World.*` (owned
  MemoryManager, WorldConfig options, optional `enterSimulationMode()`,
  per-step snapshot vectors), `dart/constraint/` (solver scratch routing,
  position-correction
  LCP path, soft-contact pooling, contact pool pre-reserve),
  `dart/lcpsolver/` (Dantzig scratch allocator wiring), `tests/`
  (allocation harness + gates), `CHANGELOG.md`, benchmark evidence docs.
- **Non-goals:**
  - No zero-allocation guarantee inside external collision backends
    (Bullet/ODE/FCL internals); strict gates run on native DART collision
    scenes.
  - No hard zero-allocation gate for the soft-body path (cheap pooling
    improvements only).
  - No behavior or numerical changes: simulation results stay bit-identical
    for existing users; no existing public signature changes.
  - No DART 7 ECS/stage/registry concepts; classic World/ConstraintSolver
    architecture stays.
  - No renaming or removal of existing allocator APIs.
- **Assumptions (evidence-based, self-resolved):**
  - The three-surface gate harness from DART 7
    (`tests/unit/simulation/world/test_world.cpp` on `main`: counting base
    allocator, global `operator new` override, glibc raw-malloc interposer)
    ports to DART 6 unchanged in mechanism; glibc interposer is Linux-only,
    skipped under ASan/codecov, and a skip is not closing evidence.
  - Benchmark evidence uses `BM_INTEGRATION_boxes` (Bullet),
    `BM_INTEGRATION_contact_container` (ODE, contact-heavy), and
    `BM_INTEGRATION_empty` (harness floor) with google-benchmark
    repetitions on one recorded host, plus `boxes_headless` determinism
    checkpoints for bit-exactness evidence.
  - Sequential packet execution in the single `release-6.20` worktree
    (incremental builds dominate wall-clock; parallel worktrees would pay a
    cold pixi-env + full-build cost per lane). Parallelism is reserved for
    read-only lanes (review, research).
  - This work is one PR to `release-6.20` (milestone: matching DART 6.x
    release). The dual-PR bugfix rule does not apply: `main` already has the
    equivalent architecture (PLAN-122).
- **Traceability:** DART 7 sources: `docs/plans/122-simulation-loop-allocation-hardening.md`,
  `docs/design/hierarchical_allocator.md`, key commits `0c23c4a85a4`
  (hierarchical allocators, #2501), `52683364ae3` (FixedCapacity, #2892),
  `c9ff3fc7a9a` (first post-bake gates, #3053), `1bb5f068b79` (boxed-LCP
  fallback assembly, #3096). DART 6 precedent forward-ports: #3140, #3141,
  #3142, #3242.

## Maintainer Decisions (interview record, 2026-07-05)

1. **Scope:** Rigid path, DART-owned code. Gates on native-DART-collision
   scenes; external backend internals exempt; soft-body gets cheap pooling
   without a hard gate.
2. **Approach:** Full World-owned MemoryManager (deepest option): port
   FrameAllocator + Frame role + FixedCapacity growth policy, wire
   ConstraintSolver through a reset-per-step arena, and give World an owned
   MemoryManager with base-allocator injection through WorldConfig.
3. **Public API:** Additive public API is acceptable on `release-6.20`
   (DART 6.13 allocator precedent). No breaking changes.
4. **Gate shape:** First-post-bake strict gate with an explicit bake API.
5. **Follow-up constraints (maintainer message, 2026-07-05):**
   - MUST keep backward compatibility with gz-physics and gz-sim: additive
     API only, default behavior unchanged, bit-identical results; the
     `gazebo` pixi environment tests and `ci_gz_physics.yml` lane are
     required evidence.
   - The explicit preparation API MUST be optional: explicit call only when the
     user wants to control when preparation happens; without it, the first
     `step()` performs preparation implicitly and efficiently. Post-preparation
     steps (explicit: from the first step after `enterSimulationMode()`;
     implicit: from the
     second step) must be allocation-free in DART-owned code.

## Work Packets

Routing: well-defined implementation packets go to Codex executors (goal
mode, one packet per run); evidence/coordination lanes stay with the Claude
orchestrator; every packet is reviewed by an independent session before it
is recorded done. Sequential execution in this worktree (see Assumptions).

#### WP-D6M.1 — Allocation-count test harness port

- Objective: DART 6 tests can measure global `operator new`, raw
  malloc-family (Linux/glibc), and injected counting-allocator activity
  across `World::step()` runs.
- Scope: new shared test support header under `tests/`; a reporting test
  that logs per-step allocation counts for a native-collision boxes scene
  and the contact-container scene; `CountingMemoryAllocator`; negative
  self-test proving the counters catch an injected allocation. Reference:
  `main:tests/unit/simulation/world/test_world.cpp:112-527,772-829`.
- Non-goals: no runtime code changes; no assertions that would fail today.
- Evidence: reporting-test output recorded in `02-baseline.md`; negative
  self-test green.
- Gates: `pixi run lint`, `pixi run build-tests`, new tests green in ctest.
- Dependencies: none. **Route: Codex.**

#### WP-D6M.2 — Baseline evidence packet

- Objective: pre-change timing and allocation baseline recorded.
- Scope: `BM_INTEGRATION_{empty,boxes,contact_container}` with repetitions
  → JSON + host capture; WP-D6M.1 reporting-test allocation counts;
  `boxes_headless` determinism checkpoints. Recorded in `02-baseline.md`.
- Evidence: committed baseline doc + JSON artifacts.
- Gates: none beyond reproducible commands recorded in the doc.
- Dependencies: WP-D6M.1 for allocation counts (timing part can precede).
  **Route: orchestrator (local commands).**

#### WP-D6M.3 — Common allocator primitives port

- Objective: DART 6 `dart/common/` gains the missing DART 7 primitives.
- Scope: `FrameAllocator` + `FrameStlAllocator` (CamelCase DART 6 style),
  `MemoryManager::Type::Frame` + `getFrameAllocator()`,
  `FreeListAllocator::GrowthPolicy{Expand,FixedCapacity}` (default Expand);
  unit tests ported from `main`. Additive public API; no consumers yet.
- Non-goals: no World/ConstraintSolver changes; no FixedPoolAllocator
  unless a later packet proves the need.
- Evidence: new unit tests green; API additions documented in headers.
- Gates: `pixi run lint`, `pixi run build`, ctest common tests.
- Dependencies: none. **Route: Codex.**

#### WP-D6M.4 — World-owned MemoryManager + optional preparation API

- Objective: every `World` owns a `MemoryManager` configured through
  additive `WorldConfig` fields (base allocator injection, free-list initial
  allocation + growth policy, frame scratch capacity);
  `World::enterSimulationMode()` is public and optional; first `step()`
  prepares the world when not explicitly prepared;
  the frame arena resets at each step boundary.
- Non-goals: no solver scratch migration yet (WP-D6M.5); no change in
  results or defaults for existing users.
- Evidence: unit tests for explicit preparation, implicit preparation, and
  re-preparation on world shape change; bit-exact `boxes_headless`
  checkpoints vs baseline; gz-physics-facing API surface unchanged except
  additions.
- Gates: `pixi run lint`, `pixi run build`, ctest simulation tests.
- Dependencies: WP-D6M.3. **Route: Codex (orchestrator reviews API).**

#### WP-D6M.9 — Allocation-free built-in profiler steady state

- Objective: the built-in text profiler performs zero heap allocation on
  warm scope pushes, removing the dominant measured per-step allocation
  source in default (`-DDART_BUILD_PROFILE=ON`) builds.
- Evidence basis (backtrace attribution, 2026-07-05): ~92% of measured
  steady-state allocations (~5,400/step of ~5,835/step) come from
  `Profiler::pushScope`/`findOrCreateChild` composing `std::string` keys
  and path strings per push (`dart/common/detail/profiler.cpp`). DART 7
  `main` has the same pattern, so this is new work, not a port; consider
  forward-porting later.
- Scope: `dart/common/detail/profiler.{hpp,cpp}` — replace per-push
  composed-string child lookup with an allocation-free lookup (small-vector
  children with `string_view` compare), build path strings lazily at
  dump/report time, keep first-visit node creation allocations (warm-up).
- Non-goals: no public API change; no change to report content/format
  beyond incidental ordering guarantees already unspecified; Tracy backend
  untouched.
- Evidence: StepAllocation reporting test shows profiler sites gone
  (operator-new/step drops accordingly); `boxes_headless` profile dump
  still prints the same tree (spot-diff); determinism checkpoints
  unchanged.
- Gates: `pixi run lint`, `pixi run build`, ctest, reporting-test rerun.
- Dependencies: WP-D6M.1. **Route: Codex.**

#### WP-D6M.5 — Close remaining hot-path allocation sites

- Objective: same-shape post-bake `World::step()` performs zero DART-owned
  heap allocation on the default rigid path.
- Scope: route ConstraintSolver persistent scratch through the free-list
  and one-shot temporaries through the frame arena (model:
  `main:dart/simulation/compute/rigid_body_contact_stage.cpp:519-533,827-830`);
  fix `solvePositionConstrainedGroup` fresh Eigen/vector allocations
  (`dart/constraint/BoxedLcpConstraintSolver.cpp:569-602`); close the
  measured Dantzig pivot allocation in
  `LCP<double>::transfer_i_from_C_to_N` (~26/step, attribution 2026-07-05)
  and wire `DantzigLcpScratch` through the existing allocator seam
  (`dart/lcpsolver/dantzig/DantzigLcp.hpp:90`); replace per-step
  `World.cpp:962,970` (and `:1127,:1136`) snapshot vectors (~29/step
  measured) with reused storage; pre-reserve contact-constraint pools at
  bake; pool `SoftContactConstraint` creation (cheap improvement, ungated).
  Before implementation, rerun the attribution dump with a deep site list
  (raise the dump budget past 200 sites) to attribute the remaining
  ~385/step long tail and fold those sites into this packet.
- Non-goals: no algorithmic changes; bit-identical simulation results.
- Evidence: WP-D6M.1 reporting test shows zero post-bake counts on the
  native-DART-collision gate scene; `boxes_headless` checkpoints
  bit-identical to baseline. Bullet comparison remains non-strict: zero
  operator-new and counting-allocator growth, with a raw-malloc residual in
  `ConstraintSolver::solveConstrainedGroups()` that WP-D6M.6 should classify
  under the external-backend compatibility matrix rather than the native strict
  gate.
- Gates: `pixi run lint`, `pixi run build`, full `pixi run test`.
- Dependencies: WP-D6M.4. **Route: Codex (may split 5a/5b on scope
  mismatch).**

#### WP-D6M.6 — First-post-bake allocation gates

- Objective: CI enforces the zero-allocation contract.
- Scope: ctest gates asserting, on native-DART-collision scenes: explicit
  bake → zero global new / zero raw malloc / zero base-allocator growth
  from the first step; implicit bake → same from the second step;
  world-base-allocator-only gates for Bullet/ODE scenes; negative
  self-test; platform guards (glibc-only interposer; ASan/codecov skips are
  recorded as skips, not passes).
- Evidence: gates green locally; injected-allocation negative test fails
  the gate as designed.
- Gates: `pixi run lint`, `pixi run build-tests`, ctest.
- Dependencies: WP-D6M.5. **Route: Codex.**

#### WP-D6M.7 — Improved performance packet + PR evidence

- Objective: honest baseline-vs-improved comparison on the same host.
- Scope: re-run WP-D6M.2 benchmarks on the final tree; comparison table
  (time/step, allocations/step) in `03-results.md` and the PR body draft;
  `pixi run -e gazebo test` (gz-physics compatibility proxy); changelog
  entry via the changelog workflow.
- Evidence: committed results doc; PR body draft with real numbers.
- Gates: `pixi run lint`, `pixi run test-all`, gazebo env tests.
- Dependencies: WP-D6M.6. **Route: orchestrator + independent reviewer.**

#### WP-D6M.8 — Promote insights and clean up

- Objective: durable artifacts promoted; this folder deleted in the
  completing PR.
- Scope: brief design-insight section in the relevant onboarding doc;
  delete `docs/dev_tasks/dart6_simulation_allocation_hardening/`.
- Dependencies: WP-D6M.7. **Route: orchestrator.**

## Acceptance Evidence

- Allocation gates (three surfaces) green on Linux; first post-bake step
  allocation-free in DART-owned code on native-collision scenes.
- `boxes_headless` determinism checkpoints bit-identical pre/post change.
- Baseline-vs-improved benchmark table with host info, repetitions, and
  JSON artifacts for `empty`, `boxes`, `contact_container`.
- `pixi run test-all` green; `pixi run -e gazebo test-gz` green (gz-physics
  compatibility); no existing public signature changed.
- CHANGELOG.md entry for DART 6.20.

## Gates

- `pixi run lint` — mandatory before every commit.
- `pixi run build` / `pixi run build-tests` — C++ changes.
- `pixi run test` — full ctest suite (no test-unit split on this branch).
- `pixi run -e gazebo test-gz` — downstream gz-physics compatibility proxy.
- New allocation gates (WP-D6M.6) — the objective-specific evidence.

## Open Decisions

- None. The landed explicit preparation API is
  `World::enterSimulationMode()` because `World::bake()` is already used by
  recording.

## Key Decisions

- Strict gates only on native DART collision scenes: global counters cannot
  distinguish DART-owned from backend-internal allocations, so external
  backend scenes are gated on the World-base-allocator surface only.
- Lazy automatic preparation in first `step()` rather than mandatory explicit
  preparation: preserves existing user code and gz-physics behavior unchanged.
- Sequential packets in one worktree: incremental builds dominate
  wall-clock; parallel mutation lanes would pay cold-build costs and risk
  build-dir contention.

## Immediate Next Steps

1. Finish WP-D6M.7 gates: run `pixi run -e gazebo test-gz`.
2. Decide how to handle the timing caveat in `03-results.md`: either qualify
   the PR as allocation-hardening with a narrow `boxes_headless` timing win, or
   implement an exact cheaper implicit-prewarm reservation before claiming a
   broad performance speedup.
3. Then run WP-D6M.8 durable-doc promotion and dev-task cleanup.
