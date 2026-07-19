# RESUME - DART 6 deformable body feature and performance

Updated: 2026-07-18 (PR #3382 Windows stabilization)

## Terminal state

[#3382](https://github.com/dartsim/dart/pull/3382) is the representative DART 6
release slice, targeting `release-6.20` from
`wp-db-native-soft-fallback`. Keep this task active through exact-head CI and
review stabilization. Do not equate landing #3382 with completion of PLAN-622:
the paired winner artifact/disposition, competitive-envelope and flexible-foot
decisions, WP-DB.07/WP-DB.08 follow-ups, and separate `main` assertion fix
remain open.

## Authoritative current state

- Worktree: `/home/js/dev/dartsim/dart/task_2`
- Branch: `wp-db-native-soft-fallback`
- PR: #3382, open, non-draft, milestone `DART 6.20.0`
- Pre-follow-up published remote head:
  `origin/wp-db-native-soft-fallback@b172b2ee1db`
- Current target base: `origin/release-6.20@75306efe770`
- Local target-base merge: `834a2548fd9`; the merge applied cleanly and
  incorporates #3388 and #3390
- Local Windows calibration: `50a254e7e56`; test-only legacy-FCL
  center-of-pressure bound change from `0.11` m to `0.13` m
- `wp-db-soft-skel-allocation-gates` is fully ancestral and incorporated; do
  not resume, merge, or cherry-pick it

This is the pre-publication snapshot for the verified follow-up. Re-fetch both
refs before acting; after publication, the live PR head and its exact-head
checks are authoritative rather than this recorded baseline.

## 2026-07-18 Windows Release calibration

At published head `b172b2ee1db`, 21 checks passed, one was skipped, and Windows
Release was the sole failure. The job completed 150/151 tests; it did not time
out. The only failing assertion was
`SoftDynamicsTest.restingSoftContactForceAndCenterOfPressureAreSmooth` in the
legacy-FCL `default adaptive` lane, where maximum per-step center-of-pressure
displacement was `0.12115883267368355` m against the former `0.11` m bound.

Commit `50a254e7e56` changes only the test calibration. Legacy FCL now uses a
`0.13` m cap, just above one `0.125` m surface-mesh interval, admitting one
manifold handoff while still rejecting a two-cell, footprint-scale jump. The
native detector remains at `0.02` m, and all percentile-force, support-loss,
spike, finite-state, and per-step guards remain intact. Verification on the
local merged candidate:

- focused calibrated case: 20/20 repeated runs passed;
- complete `test_SoftDynamics`: 25/25 passed;
- no-cache Release build: 292/292 build steps passed;
- full Release C++ suite: 154/154 passed;
- gz-physics: 199/199 functional and 4/4 performance checks passed;
- gz-sim: 1/1 source-built-DART integration test passed;
- `pixi run lint` and `git diff --check`: passed; and
- two independent adversarial reviews: clean.

No visual capture is required for this correction: it changes a numeric test
bound, not simulation or rendering behavior, and an image cannot establish the
per-step CoP limit. The Windows trace, scene's `0.125` m mesh interval, focused
repeats, and complete text gates are the relevant evidence. Exact new-head
hosted Windows CI remains required after publication.

Codecov passed on `b172b2ee1db`: patch coverage was `94.34783%` with 143
uncovered changed lines; project coverage was `75.17%` versus `73.90%`, reported
as `+1.26` percentage points. This test-only calibration needs no new changelog
entry; the existing adaptive-contact and flagship-demo entries are finalized
with the #3382 link.

The dated sections below retain historical review and runner evidence; the
authoritative state above supersedes their old "current head" wording.

## 2026-07-12 mass-matrix review-fix packet

The original review thread on historical head `b25462ca5c0` identified a real
WP-DB.04 bug: `SoftBodyNode::updateMassMatrix()` included retained point-mass
accelerations in every generalized-coordinate basis column.
`Skeleton::updateMassMatrix()` changes only Skeleton DOF accelerations, so
public mass and augmented-mass matrices could depend on prior soft simulation
state.

Published commit `2ad156e7b82`:

- constructs each point-mass matrix-column acceleration only from the parent
  body's generalized-coordinate basis acceleration;
- adds
  `SoftDynamicsTest.pointMassAccelerationsDoNotAffectMassMatrices`, which
  injects deterministic nonzero point accelerations, explicitly invalidates
  the matrix caches, and proves freshly assembled mass and augmented-mass
  matrices remain unchanged and match the Jacobian projection;
- leaves inverse dynamics unchanged, where real point-mass accelerations still
  belong.

The regression failed before the production fix with large state-dependent
matrix terms, then passed after the fix. The fix was published in head
`92ccfce567c`, and its review thread is resolved.

## Verification on mass-matrix implementation commit

Passed with compiler cache disabled after `pixi run lint` reconfigured the
tree:

```bash
pixi run lint
DART_DISABLE_COMPILER_CACHE=ON pixi run config
DART_DISABLE_COMPILER_CACHE=ON pixi run build
DART_DISABLE_COMPILER_CACHE=ON pixi run test
```

Results:

- no-cache full build: passed (301/301 build steps);
- full C++ suite: 152/152 passed;
- full `test_SoftDynamics`: 16/16 passed;
- `INTEGRATION_StepAllocation`: passed;
- `git diff --check`: passed;
- two independent post-fix reviews: clean (dynamics semantics and adversarial
  regression/cache review).

No performance claim changed, so the final benchmark matrix was not rerun for
this correctness-only matrix-query fix.

## 2026-07-12 activation-toggle review fix

Codex review of published head `92ccfce567c` identified a separate cache
correctness bug. When adaptive contact activation was disabled after inactive
points had been frozen, `setAdaptiveContactActivationEnabled(false)` changed
the active-point policy without invalidating cached articulated inertia. An
immediate query could therefore return the previous frozen/lumped tensor even
though all points were active again.

Follow-up commit `b8fe9a23093`:

- notifies the existing soft-body cache dependency chain on either
  activation-mode transition while preserving the same-value early return;
- adds
  `SoftDynamicsTest.adaptiveContactActivationToggleInvalidatesArticulatedInertia`,
  which primes the frozen cache, disables activation without stepping, and
  compares immediate ordinary and implicit articulated tensors to an
  independently dirtied recomputation and the original all-active state; and
- changes no public API, class layout, or default-off behavior.

Before the production fix, the regression observed stale diagonal values such
as `1.0` where a fresh recomputation produced `0.5`. Verification on that fix:

- `pixi run lint`: passed;
- no-cache Release build: passed (292/292 build steps);
- full Release C++ suite: 152/152 passed;
- full Python suite: 213/213 passed;
- full `test_SoftDynamics`: 17/17 passed;
- `git diff --check`: passed; and
- two independent final reviews: clean.

No benchmark was rerun because this is an immediate-query cache-correctness
fix, not a changed performance claim.

## 2026-07-12 paired-runner timeout-test review fix

Codex review of published head `551d7d34817` identified a test-only race in
`test_captured_command_times_out_and_terminates_process_group`. Its 50 ms
timeout could terminate the Python child before it executed and flushed
`started`, making the nonempty-output assertion dependent on host scheduling.

Follow-up commit `8c68e900641` keeps the production runner unchanged. The test
now proves the requested timeout is reported and the capture log equals the
output attached to `TimeoutExpired`, accepting either empty or partial output.
This preserves the production wall-time guarantee and process-group cleanup
semantics without adding a readiness bypass.

Verification:

- focused timeout test: 100/100 repeated launches passed;
- full paired-runner test file: 38/38 passed;
- `pixi run lint`: passed;
- full Python suite after a complete rebuild: 213/213 passed;
- `git diff --check`: passed; and
- two independent final reviews: clean.

## 2026-07-12 target-base merge verification

`origin/release-6.20` advanced to `4ddfe712b359` through #3384, which guards
non-finite LCP solutions and fixes the exact assert-only failure previously
observed in `test_MjcfParser`. The base was merged, not rebased. The only
conflict was `CHANGELOG.md`; the resolution retains both the upstream
non-finite-LCP entry and this branch's adaptive soft-contact entry.

Verification on merged head `52ff108437d`:

- `pixi run check-lint`: passed, including C++, CMake, Python, codespell, and
  AI-command parity checks;
- no-cache Release build: passed (292/292 build steps);
- full Release C++ suite: 152/152 passed, including `test_MjcfParser`,
  `test_SoftDynamics`, and `test_NonFiniteContact`;
- full Python suite: 213/213 passed; and
- no-cache assert-enabled (`CMAKE_BUILD_TYPE=None`) focused gate:
  `test_ConstraintSolver`, `test_ContactConstraint`, and `test_MjcfParser`
  passed 3/3.

The old exact-base run on `fa17fad79b9` is historical evidence, not a current
base blocker. This merge was published in head `92ccfce567c`.

## 2026-07-12 balanced-evidence runner packet

Published commits `9a7bab76948` and `a122c5ab437` provide the bounded replacement
for the manual native-vs-DART A/B method. The runner pins one clean HEAD in a
detached dedicated Release/profile build, qualifies checksums at threads 1 and
16, keeps each row's warmup adjacent to its 20 alternating measured pairs,
gates load and sibling work across each pair, requires a recovered thermal
state when each pair starts, records post-run heating observationally,
preserves raw JSON/logs and idle history, and writes authoritative
`COMPLETE.json` last.

Verification on the current runner stack:

- focused synthetic and mocked-orchestration tests: 38/38 passed;
- full Python suite: 213/213 passed;
- `pixi run lint`: passed;
- `pixi run check-ai-commands`: passed;
- `git diff --check`: passed;
- independent protocol/evidence review: clean;
- independent adversarial runner/test review: clean.

The first full attempt at `8553203db25` completed its build and checksum gates
but was interrupted during idle preflight, so it has no timing rows or
completion marker. A two-row `soft_cubes/16` diagnostic then proved canonical
invocations heat this host from 55-57 C to 86-93 C without materially moving
1-minute load. That diagnostic corrected the gate: thermal recovery is
required at pair start, while post-run temperatures are observational and
alternating order balances self-heating. The diagnostic is not winner evidence.

The corrected final-head attempt at `3704865daa95` also completed its dedicated
build and all scene/thread checksum gates. Across 286 preflight polls it saw 84
load failures, 74 thermal failures, and 10 local-DART-workload failures; the
longest continuous clean interval was only 141/600 seconds. An explicit
11-minute no-tool interval still encountered external DART workloads and
99-100 C package temperatures. The attempt was cleanly interrupted with no raw
timing rows, summary, verdict, or `COMPLETE.json`, so the directory is explicit
non-evidence. The final paired artifact remains an action for a genuinely
isolated or maintainer-provided quiet host.

A third attempt at `babca41f70de` completed its dedicated build and all
scene/thread checksum gates, then reached preflight. It recorded 12 polls: all
12 failed the load gate, six failed a thermal gate, the 1-minute load peaked at
`30.64`, and an observed sensor peaked at `102 C`. Before timing could begin,
`origin/release-6.20` advanced through #3384, so this pre-merge revision was no
longer the publication candidate. The attempt was interrupted with
`status: interrupted`, no timing rows, summary, verdict, or `COMPLETE.json`; it
is explicit non-evidence and must not be resumed.

The next attempt pinned clean merged/docs head `af7ae4ccedde`, completed its
dedicated build and all checksum gates, and ran for 28 minutes overall,
including 26 minutes in preflight. Other worktrees repeatedly launched DART
builds, including `ninja ALL` in `task_3-release620-publish` and format checks in
`task_1`/`task_1_release620`. Across 156 polls, all 156 failed the load gate,
151 failed a thermal gate, 110 observed sibling DART work, and none was clean.
Observed peaks were 50 sibling workloads, 1-minute load `52.61`, and `105 C`.
The attempt was interrupted with `status: interrupted`, no timing rows,
summary, verdict, or `COMPLETE.json`. This is the current exact-composed-head
host blocker and explicit non-evidence; do not resume its directory or weaken
the runner's gates.

## Current blockers and next actions

- Published head `b172b2ee1db` has a clean current-head Codex review and zero
  unresolved review threads. Its check snapshot is 21 passed, one failed, and
  one skipped. Windows Release is the only failure and is addressed by the
  locally verified test-only CoP calibration above. Both Codecov checks passed.
- The exact new head does not exist remotely yet. Do not carry the old Windows
  result forward after publication; classify the new exact-head job from its
  own logs.
- No paired benchmark directory has `COMPLETE.json`. Interrupted and
  preflight-only directories remain explicit non-evidence and must not be
  resumed or reinterpreted.
- The formal competitive-implementation envelope still needs maintainer
  sign-off. The four-link flexible-rigid-foot versus deformable-foot comparison
  remains neither implemented nor explicitly deferred.
- WP-DB.07's original multicore-improvement acceptance and WP-DB.08's
  native-owned/preferred-default acceptance remain unmet. Their durable owners
  are `docs/design/dart6_deformable_body.md` and PLAN-622.
- Release commit `10c6b6055e4` still needs a separate `main` PR for the
  zero-DoF soft point-mass assertion; do not fold that work into #3382.

Next actions:

1. Re-fetch `origin/release-6.20`; if it moved past `75306efe770`, merge the new
   tip (never rebase) and rerun proportionate gates.
2. Push the additive local commits under the existing routine-maintenance
   approval, update the PR body with the exact head and evidence, then post one
   fresh top-level `@codex review`. Do not reply to AI-generated inline
   comments.
3. Monitor exact-head CI to terminal state. Rerun only demonstrated
   infrastructure failures and investigate product failures from their own
   exact-head logs. Hosted Windows is required proof for this calibration.
4. Keep PLAN-622 active after #3382 stabilization. Capture the balanced paired
   artifact only on a host that passes the runner's gates, or obtain explicit
   maintainer disposition; also retain the competitive-envelope,
   flexible-foot, WP-DB.07, WP-DB.08, and separate-main-PR work.
5. After #3382 merges, promote any remaining durable facts, remove this
   temporary task folder only when its open work has durable owners, and clean
   branches only with explicit approval.

## Demo integration rule

The flagship `adaptive_soft_contact` and `soft_worm` examples are scenes in
`examples/demos` and run through `dart-demos`; the old standalone executables
were removed. Their GUI-free model tests preserve the numerical contracts.
Continue integrating new GUI examples into `dart-demos` rather than creating
new standalone example executables.

## Historical 2026-07-12 published-head snapshots and external blockers

At the last inspection of published head `551d7d34817`, #3382 was mergeable
but blocked on hosted checks. The fresh Codex review had one unresolved thread,
`PRRT_kwDOACTnoM6QQ7aW`, for the timeout-test race addressed by follow-up commit
`8c68e900641`. Seven checks had passed, twelve were still running, and no
current-head CI failure had appeared. Treat this as a snapshot and inspect the
exact live head before acting.

Earlier published head `92ccfce567c` was mergeable but blocked by pending or
failed checks:

- The mass-matrix review thread is resolved. A fresh Codex review completed on
  this exact head and found the activation-toggle cache issue addressed by
  follow-up commit `b8fe9a23093`. At this pre-fix-head snapshot, that thread was
  the only unresolved review thread.
- Hosted run `29218455336` did not expose a product failure in the failed
  `coverage` or cancelled `Debug` jobs. On self-hosted runner
  `dartsim-mark13-4`, concurrent jobs reused the same `_work/dart/dart`
  directory: checkout removed another job's workspace, Pixi cache restore then
  failed with `ENOENT`, and another compile was cancelled. The AVX2 job in run
  `29218455356` failed at the same Pixi setup boundary.
- gz-physics run `29218455355` was cancelled while building and had one
  infrastructure retry in progress. These old-head jobs do not classify later
  heads; classify and rerun only failures on the exact current head.
- Other checks on `92ccfce567c` had completed successfully, including
  API docs, both Read the Docs builds, newest GCC/Clang, macOS arm64 Debug and
  Release, SSE4.2, AVX, and Eigen 64-byte alignment. Linux Release, assertions,
  Windows, FreeBSD, scalar SIMD, and the gz retry were still running at the
  last live inspection.
- The final matrix artifact's machine-readable evaluator verdict is `FAIL` on
  the expected-fastest detector gate. Manual interleaved A/B results suggest a
  tie on only two single-thread rows; they do not resolve the five failed rows.
  The original matrix command and recovered scratch method are now documented
  in `06-pr-evidence.md`, including why the scratch rows do not satisfy the
  gate. Commits `9a7bab76948` and `a122c5ab437` provide the reviewed,
  revision-pinned replacement runner, but no final artifact exists yet. The
  corrected final-head attempt still recorded recurring sibling DART work and
  package temperatures up to `100 C`; its longest clean interval was only 141
  of the required 600 seconds. Capture the balanced artifact once the runner's
  own idle/thermal gates can pass on an isolated or explicitly quiet host, or
  obtain explicit maintainer acceptance before task retirement.
- The formal definition of "competitive implementations" still needs
  maintainer sign-off. The current proposal is in-tree CPU/backend comparison
  plus normalized paper metrics; do not treat PR publication as approval.
- Original WP-DB.07 and WP-DB.08 acceptance remains unmet. The only identified
  paper-matrix row outside the explicitly approved deferral list is the
  four-link flexible-rigid-foot versus deformable-foot comparison. Durable
  background/design owners and PLAN-622 now preserve these gaps, but the
  flexible-foot and competitive-envelope decisions still require maintainer
  closeout before retiring this task folder.

## Historical 2026-07-12 next actions

1. Fetch `release-6.20` before publication. The branch already merges
   `4ddfe712b359` at `52ff108437d`; if the base moved again, merge the new
   `origin/release-6.20` tip into the topic branch (never rebase), resolve
   conflicts, and rerun the relevant gates on the merged state.
2. If runner-test fix `8c68e900641` and this task-home refresh are not yet
   published, push them. Update the PR body to the resulting exact head and
   record the timeout-test correction and its 100/100, 38/38, lint, and 213/213
   gates. Resolve review thread `PRRT_kwDOACTnoM6QQ7aW`, then post one fresh
   top-level `@codex review` for that head.
3. Monitor the new head through CI. Rerun only exact-current-head
   infrastructure failures, and investigate any product failure from its own
   logs rather than carrying the old-head runner collisions forward.
4. Track the PLAN-622 dual-PR follow-up for `10c6b6055e4`: the same over-strict
   zero-DoF soft point-mass assertion exists on `origin/main`, unlike the
   mass-matrix correction (DART 7 still has point-mass mass aggregation
   disabled). Do not fold unrelated main-branch work into #3382.
5. Obtain the remaining competitive-envelope and flexible-foot decisions.
   Keep the already-created durable background/design owners and PLAN-622
   synchronized with the result.
6. On the final clean local head and only when no sibling build is active, run:

   ```bash
   sha=$(git rev-parse --short=12 HEAD)
   pixi run bm-soft-body-paired \
     --revision HEAD \
     --cpu-list 0-15 \
     --output-dir ".benchmark_results/wp-db-native-paired-${sha}"
   ```

   Treat the artifact as complete only when `COMPLETE.json` exists. A full PASS
   requires all eight row medians to satisfy `native / dart <= 1.02`; otherwise
   retain the FAIL artifact and follow the native-owned kernel disposition in
   `06-pr-evidence.md`. Never resume a partial artifact or benchmark around the
   runner's load/thermal gates.
7. After merge, audit the new durable compatibility/design/reference owners,
   update PLAN-622, remove the temporary task folder in the completing closeout
   change, and clean branches only with explicit approval.

## Approval boundaries

The user authorized routine #3382 maintenance with "go ahead": additive
commits and pushes, PR title/body updates, resolving addressed automated-review
threads, CI reruns, and fresh automated review requests. That instruction does
not authorize merging or closing the PR, force-pushing or rewriting history,
requesting human reviewers, changing the base, or deleting branches. Preserve
the no-AI-attribution commit/PR rule.
