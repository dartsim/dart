# Supervisor Notes for Codex

This file is supervisory feedback from Claude-side review agents over the
`native_collision` dev task. It is not the planning contract. Treat
`README.md`, `01-design.md`, `02-milestones.md`, and
`06-completion-audit.md` as authoritative.

Last updated: 2026-05-16 (compacted per Round 20 doc-hygiene policy).

## Current Codex Status

The historical steering below has mostly been executed. Step A's documentation
truthfulness cleanup is reflected in `README.md` and `RESUME.md`. Step B1 is
represented by the clean dartpy API plus default-on C++ compatibility-facade
deprecation policy. Step B2 is represented by the reference-file cleanup audit,
which found no unreferenced FCL/Bullet/ODE implementation files to delete.
Step C is represented by `PR-DRAFT.md` and `07-pr-evidence-transfer.md`.

`06-completion-audit.md` is the source of truth for requirement-to-evidence
mapping and remaining blockers. Step D remains blocked by the standing
instruction not to open or reopen a PR. Do not delete
`docs/dev_tasks/native_collision/` until the maintainer opens the completing
PR and the evidence is transferred there.

Active local guidance:

- Keep `PR-DRAFT.md` and `07-pr-evidence-transfer.md` current.
- Do not open or reopen a PR, trigger workflows, mutate PR metadata, or delete
  this folder without explicit maintainer/user approval.
- Treat only the current open gates in `README.md` and
  `06-completion-audit.md` as active completion blockers.
- Historical rounds below are retained as anchors and rationale. Use git
  history for the pre-compaction full text when needed.

## Independent Spot-Check Results

These claims were verified against the working tree, not only from docs:

| Claim                                                                  | Result   | Evidence                                                                                                                                                              |
| ---------------------------------------------------------------------- | -------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `check-collision-runtime-isolation` is wired into lint and check-lint. | Verified | `pixi.toml` references the task from both lint paths.                                                                                                                 |
| Old-engine implementation is isolated outside runtime collision paths. | Verified | Runtime `dart/collision/{bullet,fcl,ode}/reference/` directories are absent; implementation files live under `tests/dart/test/reference_collision/{bullet,fcl,ode}/`. |
| `dart/collision/` layout matches native+adapter+facade split.          | Verified | Top-level layout has `dart/`, `native/`, `bullet/`, `fcl/`, `ode/`, and public headers.                                                                               |

## Active Anti-Goals

- Do not delete reference implementations under
  `tests/dart/test/reference_collision/{fcl,bullet,ode}/` while they are still
  used by `dart-test-reference-*` targets.
- Do not open a new GitHub PR. Per Q2, the user opens the successor PR
  manually.
- Do not delete this dev-task folder until the successor PR exists, evidence
  is transferred, and deletion happens in the completing PR.
- Do not silently disable `DART_COLLISION_DEPRECATE_LEGACY_NAMES` or remove
  C++ deprecation attributes when downstream tests fail. Document downstream
  migration instead.
- Do not compact authoritative design docs (`01-design.md`,
  `02-milestones.md`, `04-reference-gap-analysis.md`,
  `05-downstream-migration.md`) as part of supervisor side-channel cleanup.

## Answered Questions

### Open Question Q1 for User - ANSWERED

**Q1 ANSWER (user, 2026-05-14):** Yes, proceed with C++
`[[deprecated]]` warnings on retained legacy spellings.

**Q1 ADDENDUM (user, 2026-05-14):** dartpy backward compatibility is not
required through DART 7. Prefer the clean API: expose
`DartCollisionDetector` and do not retain Python legacy aliases.

### Open Question Q2 for User - ANSWERED

**Q2 ANSWER (user, 2026-05-14):** PR #2652 stays closed. A successor PR will
be opened manually by the user. Codex must not open a PR; keep accumulating
local evidence and draft the PR description in this folder.

### Open Question Q3 for User - ANSWERED

**Q3 ANSWER (user, 2026-05-15):** Fix everything in the box-box regression
slice rather than narrowing to one small symptom.

### Open Question Q5 for User - ANSWERED

**Q5 ANSWER (user, 2026-05-15):** Prefer the cleaner long-term approach for
invalid convex/soft mesh handling and keep compatibility behavior explicit.

### Open Question Q6 for User - ANSWERED

**Q6 ANSWER (user, 2026-05-15):** Move old FCL/Bullet/ODE reference
implementation under `tests/dart/test/reference_collision/`; keep runtime
paths as facades only.

### Open Question Q7 for User - ANSWERED

**Q7 ANSWER (user, 2026-05-15):** Fix box-box quality directly and hold the
native detector to the same stable face-patch behavior expected from mature
engines.

### Open Question Q8 for User - Stretch scope-decision items

**Q8 ANSWER (user, 2026-05-15):** Defer all stretch scope-decision items until
real evidence or user requests force them.

### Open Question Q9 for User - Batch API ownership and ECS timing

**Q9a ANSWER (user, 2026-05-15):** Option C. Provide both per-pair direct
batch functions and a central `NarrowPhase::collideBatch` dispatcher.

**Q9b ANSWER (user, 2026-05-15):** Option B. Tier 3 ECS integration lands
after Round 7, Round 6, and Q4 are clean.

### Open Question Q12 for User - ANSWERED

**Q12 ANSWER (user, 2026-05-15):** Follow the bigger picture: finish the
ordered native-collision quality queue rather than only the visible Atlas
regression.

### Open Question Q16 for User - `test_native_backend.cpp` rename

**Q16 ANSWER (user, 2026-05-15):** Option B. Rename the native backend test
when flattening the test layout.

## Round Summaries

## Round 1 - Initial Supervisor Review

Outcome: Identified missing downstream deprecation criteria, final runtime
cleanup scope, PR evidence transfer, and CI-evidence visibility. Later work
resolved local documentation truthfulness and added PR staging packets.

## Round 2 - Box-Box Regression Slice

Outcome: Reconciled reviewer findings around native box-box contact quality and
set a higher bar for stable face/edge/vertex contacts under simulation.

## Round 3 - Box-Box Local Work

Outcome: Landed box-box contact fixes and validation notes. Later rounds added
stronger world-level stability and SAT-axis tests.

## Round 3 Local Completion Notes

Status: Historical. The durable evidence now lives in `03-evidence-gates.md`,
`09-test-coverage-matrix.md`, and the box-box tests.

## Round 4 - Supervisor Review Of Round 3 Local Work

Outcome: Flagged scope creep around empty convex mesh fallback and required
user direction before expanding public compatibility behavior.

## Round 5 - Invalid Mesh Compatibility

Outcome: User chose the clean long-term path. Later code made invalid
convex/soft mesh data non-collidable with warnings and focused tests.

## Round 5 Local Completion Notes

Status: Historical. Current evidence is represented by clean dartpy API,
invalid mesh tests, and downstream migration docs.

## Round 6 - Reference Code Belongs Under tests/

Outcome: Old FCL/Bullet/ODE implementation moved under
`tests/dart/test/reference_collision/`; runtime paths remain facades.

## Round 6 Local Completion Notes

Status: Complete. Runtime isolation and compatibility-facade audits guard this
boundary.

## Round 7 - Box-Box Rest Stability Regression

Outcome: Established the face-patch, long-horizon no-tunneling, SAT stability,
and pair-order bars for native box-box.

## Round 8 - Test & Benchmark Coverage Matrix

Outcome: Created and maintained `09-test-coverage-matrix.md` as the canonical
coverage inventory. Stretch scope-decision rows were marked deferred per Q8.

## Round 8 Spot-Check - Round 7 Box-Box Slice

Outcome: Independent spot-check confirmed the Round 7 box-box slice moved the
relevant matrix rows and had focused validation.

## Round 9 - Batch + SIMD as First-Class Collision Dimension

Outcome: User ratified per-pair batch APIs plus a central dispatcher. Tier 3
ECS integration is deferred until the Tier 2 surface is stable.

## Round 10 - DART 7 vs DART 8 Compatibility Horizon

Outcome: DART 7 keeps narrow gz-physics/source compatibility facades. DART 8 is
the cleanup horizon for removing compatibility-only legacy spellings.

## Round 11 - Round 7 Acceptance Review

Outcome: User-side confirmation and independent review accepted the Round 7
box-box direction with followups, not as a final north-star completion.

## Round 12 - Atlas-Simbicon Regression

Outcome: Reframed the visible Atlas issue as part of the broader quality queue:
complete box-box, reference move, Q4 benchmark parity, batch work, then
regression diagnosis.

## Round 13 - Performance Bar Locked

Outcome: Locked the release bar: native must beat the best reference by at
least 1.05x on required workloads, with correctness preserved.

## Round 14 - Apples-to-Apples Perf Comparison

Outcome: Split perf evidence into Bar 1 raw narrow-phase and Bar 2
adapter-on-both-sides. Bar 2 losses remain release-gate PERF-GAPs.

## Round 15 - F11-1 Priority Reminder

Outcome: Reasserted that the next implementation slice had to be F11-1:
box-box Tier 2 batch API plus benchmark refresh before more backlog work.

## Round 16 - Drop the tests/unit/collision/native/ Subfolder

Outcome: User chose the flattening/rename option. Native collision tests were
flattened so source layout and test layout are easier to scan.

## Round 16 Local Completion Notes

Status: Complete. The current test tree no longer depends on the old native
subfolder layout.

## Round 17 - F11-1 Escalation

Outcome: Required Codex to stop matrix backlog work and land F11-1 before any
further broadening.

## Round 17 Acknowledgment

Status: Historical. Codex acknowledged the required F11-1 ordering.

## Round 17 Local Completion Notes

Status: Complete. F11-1 landed and later batch work extended beyond box-box.

## Q4 Raw Reference Edge-Case Local Completion Notes

Outcome: Raw reference narrow-phase benchmark coverage was refreshed and
validated. Later benchmark-audit work hardened result consumption and
contact-count checks.

## Atlas/Hello-World Regression Local Completion Notes

Outcome: Added direct `hello_world`-style and Atlas Simbicon controller-loop
native collision regressions. Current validation evidence is recorded in
`03-evidence-gates.md`.

## Round 18 - Suspicious-Win Perf Audit Protocol (2026-05-16)

User direction: if DART wins by a large or unrealistic margin, inspect
correctness, fairness, output consumption, and measurement quality before
claiming a durable win.

Status: Bar 1 raw narrow-phase audit hardening landed in commits
`0b425b3f7af`, `184c8be739d`, `c6ba832b844`, `1f59af53d30`,
`9c78540e933`, and later mesh audit work. The protocol is now durable:
large wins are provisional until audited for `DoNotOptimize`, symmetric work,
output assertions, cache-state symmetry, shape construction outside the timed
loop, compiler/threading symmetry, timer granularity, variance, and
cross-engine sanity.

Current implication: adapter-lane Bar 2 optimization remains a next performance
wave; audited benchmark guardrails stay in this feature-level pass.

## Round 19 - No FCL/Bullet/ODE Refs in Implementation (2026-05-16)

User direction: implementation code must not explicitly reference FCL, Bullet,
or ODE except for documented DART 7 compatibility facades and test/benchmark
reference code.

Status: `check_collision_runtime_isolation.py` and
`audit_collision_compat_facades.py` enforce the runtime/reference boundary.
The protected exception is the legacy factory-key compatibility surface in
`dart_collision_detector.cpp`; DART 8 removes it after downstream migration.

## Round 20 - Planning-Doc Cleanup Cadence (2026-05-16)

User direction: clean up `docs/dev_tasks/native_collision/` for completed work
without deleting the active task folder. Focus on current progress, general
lessons, and plans; avoid polluting agent context with detailed history.

Policy:

- Compact when a side-channel doc crosses the threshold by more than 50%.
- Keep authoritative design/evidence docs intact unless they are explicitly
  being edited for current evidence.
- Preserve grep anchors: `## Round N`, `### Open Question Qn`,
  `**Qn ANSWER`, and `## Round N Local Completion Notes`.
- Make compaction its own docs-only change; never bundle it with code.
- Promote durable lessons before deleting detailed history.

Status: This file was compacted under that policy. Use git history for the
pre-compaction full supervisor narrative.

## Round 21 - SIMD Final Target for Batch Collision (2026-05-16)

User direction: the final batch collision path should fully utilize SIMD; the
current scalar for-loop batch entries are scaffolding, not the destination.

### Final target

Every Tier 2 `collide<Pair>Batch` entry eventually has a SIMD-vectorized body
for batch sizes `N >= 4`, with smaller batches falling back to scalar. The
batch benchmark rows must show per-pair time below the equivalent single-pair
loop at matching `N`, and Bar 2 adapter PERF-GAPs must close to audited wins.

### Sequencing

SIMD lands after Round 9 main batch APIs, Q4 followup benchmark work, and
Round 18 audit completion. SIMD lands before DART 8 cleanup and before Tier 3
ECS integration relies on Tier 2 batch performance.

### Strategy

Use a portable SIMD abstraction such as xsimd rather than handwritten
architecture-specific intrinsics. First target box-box SAT axes, then
face-polygon clipping, then contact reduction and mixed-pair grouping if the
speedup justifies added complexity.

### Anti-goals

- Do not SIMD-ize scalar single-pair entries.
- Do not start SIMD while per-pair Tier 2 batch coverage is incomplete.
- Do not skip the Round 18 benchmark audit protocol.
- Do not lose pair-order symmetry or gz-physics compatibility.

### Round 21 status

Currently planned, not started. This is its own multi-commit performance wave
after current feature-level completion.

### Durable preference recorded

The performance bar should explicitly tie Bar 2 PERF-GAP closure to
SIMD-vectorized Tier 2 batch collision.

### No new question

Round 21 documents the final target; sequencing is already answered by the
existing queue.
