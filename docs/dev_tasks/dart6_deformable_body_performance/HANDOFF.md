# Handoff - DART 6 deformable body performance

Updated: 2026-07-12

This is a cross-agent/worktree handoff for PR #3382 stabilization. The complete
fresh-session state and commands are in `RESUME.md`; do not use the historical
pre-PR benchmark instructions that this file previously contained.

## Packet contract

**Goal:** land the current deformable-body release slice through #3382 with
review findings fixed, gates honest, and DART 6 compatibility preserved.

**Done when:**

- the mass-matrix review correction is published and the thread is addressed;
- the current PR head has the required clean review evidence;
- branch-local CI is green and exact-base/infrastructure failures are
  dispositioned without weakening product gates;
- remaining competitive-envelope/packet scope decisions are explicit, and
  task docs plus durable owners describe the final landed/open state.

**Risks:** public matrix correctness, DART 6 ABI/API compatibility, default-off
bit identity, noisy performance evidence, and confusing current branch-local
failures with failures already present on `release-6.20`.

## Current checkout

```text
worktree  /home/js/dev/dartsim/dart/task_2
branch    wp-db-native-soft-fallback
impl      2ad156e7b82  (followed locally by this handoff/docs update)
handoff   dbfed2fdd88  (followed locally by durable-owner promotion)
runner    9a7bab76948  (balanced final-head evidence infrastructure)
correction a122c5ab437 (pair-start thermal gate)
follow-up ab6e8edede4 (durable native-kernel and main-fix owners)
contract  b615f5f1f6e (binding native-kernel details)
evidence  babca41f70d (records the sustained paired-run host blocker)
merge     52ff108437d (merges current release-6.20 through #3384)
docs      af7ae4ccedd (merged-head verification and takeover state)
published 92ccfce567c (complete pre-review-fix stack)
review    b8fe9a23093 (activation-toggle cache invalidation)
base      4ddfe712b359
PR        https://github.com/dartsim/dart/pull/3382
```

`wp-db-soft-skel-allocation-gates` is already fully incorporated and stale
against the release branch. Do not resume it.

## Completed locally

Published commit `2ad156e7b82` fixes the original P2 mass-matrix review finding
and adds a regression proving retained point-mass accelerations cannot affect
public mass or augmented-mass matrices. Verification on that commit:

```text
pixi run lint                                      PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run build     PASS
DART_DISABLE_COMPILER_CACHE=ON pixi run test      PASS (152/152)
test_SoftDynamics                                 PASS (16/16)
INTEGRATION_StepAllocation                        PASS
independent post-fix reviews                      CLEAN x2
```

The complete stack through `92ccfce567c` is published, the mass-matrix thread is
resolved, and the PR title/body now state the representative release scope and
failed winner gate accurately.

Commits `9a7bab76948` and `a122c5ab437` provide the reviewed
`bm-soft-body-paired` evidence runner. The current runner stack has 38/38
focused tests, 213/213 full Python tests, clean lint/AI-command checks, and two
clean independent reviews. It resolves the old manual A/B reproducibility gap
at the tooling level, but no final artifact has been captured: sibling DART
builds still drove the observed host to load `14.74` and package temperature
`100 C`. Use the exact clean-HEAD command and completion rules in `RESUME.md`;
do not bypass its idle/thermal gates or resume a partial artifact.

The corrected final-head attempt at `3704865daa95` completed its dedicated
build and checksum qualification but never passed preflight. Its best clean
interval was 141/600 seconds; even an 11-minute no-tool interval encountered
external DART workloads and 99-100 C package temperatures. It was interrupted
with no timing rows, summary, verdict, or `COMPLETE.json` and is non-evidence.
Commits `ab6e8edede4` and `b615f5f1f6e` promote and complete the staged
native-owned soft-kernel contract in the durable design owner and the separate
`main` zero-DoF fix in PLAN-622.

The base advanced through #3384 after prior published head `b25462ca5c0`.
Merge commit `52ff108437d` retains both changelog entries and passes
`pixi run check-lint`, a 292/292 no-cache Release build, the 152/152 Release C++
suite, the 213/213 Python suite, and a no-cache assert-enabled focused gate covering
`test_ConstraintSolver`, `test_ContactConstraint`, and `test_MjcfParser` (3/3).
#3384 resolves the historical non-finite-LCP `test_MjcfParser` failure on the
composed branch. The merge is published in head `92ccfce567c`.

A later attempt at `babca41f70de` also completed its dedicated build and
checksum gates, but was interrupted after the base moved and before timing. Its
12 preflight polls all failed load, six also failed thermal, load peaked at
`30.64`, and one sensor reached `102 C`. It has no timing rows, summary,
verdict, or `COMPLETE.json` and is explicit non-evidence.

The exact merged/docs-head attempt at `af7ae4ccedde` completed its build and
checksum gates, then ran for 28 minutes overall, including 26 minutes in
preflight while other DART worktrees repeatedly built. All 156 polls failed
load, 151 failed thermal, 110 detected sibling DART work, and none was clean;
peaks were 50 workloads, load `52.61`, and `105 C`. It was interrupted with no
timing rows, summary, verdict, or `COMPLETE.json`. This is explicit host-blocker
evidence, not a result to resume or reinterpret.

Current review fix `b8fe9a23093` invalidates the soft cache dependency chain
when adaptive contact activation changes mode and adds
`SoftDynamicsTest.adaptiveContactActivationToggleInvalidatesArticulatedInertia`.
The regression failed before the fix with stale frozen/lumped articulated
inertia and passes after it. Verification is clean: lint; 292/292 no-cache
Release build; 152/152 C++; 213/213 Python; 17/17 `test_SoftDynamics`;
`git diff --check`; and two independent reviews. If it is not yet published,
publish it with this task-home refresh. Then update the PR body to the resulting
head and 17/17 `test_SoftDynamics` gate, resolve thread
`PRRT_kwDOACTnoM6QQkkC`, and request one fresh current-head Codex review.

## Pre-fix hosted snapshot and blockers

- At the last pre-fix snapshot, published head `92ccfce567c` was mergeable with
  checks pending or failed. Its failures were self-hosted-runner workspace
  collisions, not observed product failures: coverage and AVX2 lost Pixi during
  setup, Debug was cancelled mid-compile, and the first gz-physics attempt was
  cancelled while building. Classify only exact-current-head results.
- The mass-matrix review thread is resolved. At that snapshot, the only
  actionable Codex thread was the activation-toggle issue addressed by
  follow-up commit `b8fe9a23093`.
- The original WP-DB.07/WP-DB.08 contracts are not complete, and the four-link
  flexible-rigid-foot versus deformable-foot row is not covered by the approved
  deferral. Their durable owners exist, but the open work or disposition must
  remain explicit before task retirement.
- The formal competitive-implementation envelope still needs maintainer
  sign-off.
- The balanced runner is ready locally, but the eight-row artifact and its
  independent winner-gate verdict remain pending a genuinely isolated or
  maintainer-provided quiet host.

See `RESUME.md` for the ordered next actions, dual-PR applicability, and
approval boundaries.
