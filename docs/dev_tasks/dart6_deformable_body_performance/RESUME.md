# RESUME - DART 6 deformable body feature and performance

Updated: 2026-07-12 (PR #3382 stabilization)

## Terminal state

The implementation is being completed through
[#3382](https://github.com/dartsim/dart/pull/3382), targeting
`release-6.20` from `wp-db-native-soft-fallback`. Keep this task active until
the PR is mergeable, its current review findings are addressed, required CI is
either green or dispositioned by maintainers with exact-base evidence, and the
temporary task docs have a clear promotion/retirement path.

Do not start another speculative optimization packet while PR stabilization is
unresolved. The immediate work is review/CI stewardship and honest closeout.

## Authoritative current state

- Worktree: `/home/js/dev/dartsim/dart/task_2`
- Branch: `wp-db-native-soft-fallback`
- Latest implementation commit: `2ad156e7b82` (`Keep soft accelerations out
  of mass matrix columns`)
- Published head: `origin/wp-db-native-soft-fallback` at `b25462ca5c0`
- Local state: the implementation commit plus this following handoff/docs
  update are not published; verify the exact current HEAD and ahead count
- Target base observed: `origin/release-6.20` at `fa17fad79b9`
- PR: #3382, open, non-draft, milestone `DART 6.20.0`
- `wp-db-soft-skel-allocation-gates` is fully ancestral to the active branch
  and fully incorporated. Do not resume, merge, or cherry-pick that branch
  again; use live refs rather than recording a volatile ahead count.

Run `git status --short --branch`, fetch the base and PR refs, and re-check the
live PR before relying on these hashes.

## 2026-07-12 review-fix packet

The only current unresolved review thread on published head `b25462ca5c0`
identified a real WP-DB.04 bug: `SoftBodyNode::updateMassMatrix()` included
retained point-mass accelerations in every generalized-coordinate basis
column. `Skeleton::updateMassMatrix()` changes only Skeleton DOF
accelerations, so public mass and augmented-mass matrices could depend on prior
soft simulation state.

Local commit `2ad156e7b82`:

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
matrix terms, then passed after the fix.

## Verification on local implementation commit

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

## Live PR blockers and external evidence

At published head `b25462ca5c0`, GitHub reports #3382 mergeable but blocked:

- Linux `coverage` and `Asserts enabled (no -DNDEBUG)` fail only in
  `test_MjcfParser`, aborting at
  `BodyNode::addConstraintImpulse()` on a NaN assertion.
- Exact-base run `29178779447` on `fa17fad79b9` fails the same two jobs in the
  same test and assertion. Treat these as current base-branch failures, not
  evidence against the deformable-body changes.
- Windows Release run `29188317164` was cancelled at the workflow's 300-minute
  limit while still compiling/linking. Its primary error is
  `The operation was canceled.`; the later `EBUSY` cleanup message is
  secondary.
- The current mass-matrix review thread remains unresolved on GitHub until the
  local fix is approved, pushed, and reviewed.
- The most recent current-head `@codex review` request hit the Codex review
  quota. Do not post duplicate triggers until review capacity is available or
  the approved follow-up push is ready.
- The final matrix artifact's machine-readable evaluator verdict is `FAIL` on
  the expected-fastest detector gate. `06-pr-evidence.md` resolves that gate
  from manual interleaved A/B results, but no durable command/raw artifact for
  those interleaved rows is currently present. Capture reproducible evidence or
  obtain explicit maintainer acceptance before task retirement.
- The formal definition of "competitive implementations" still needs
  maintainer sign-off. The current proposal is in-tree CPU/backend comparison
  plus normalized paper metrics; do not treat PR publication as approval.
- Original WP-DB.07 and WP-DB.08 acceptance remains unmet, and some
  paper-matrix rows fall outside the explicitly approved deferral list. Promote
  those contracts to durable roadmap/design owners or obtain explicit closeout
  decisions before retiring this task folder.

## Next actions

1. Obtain explicit maintainer/user approval for the follow-up push and related
   PR mutations.
2. Immediately before any push, fetch `release-6.20`. If it moved, merge
   `origin/release-6.20` into the published topic branch (never rebase), resolve
   conflicts, and rerun the relevant gates on the merged state.
3. Push local commit `2ad156e7b82`, resolve the addressed automated-review
   thread only if approval covers thread mutation, and request a fresh top-level
   Codex review only if approval covers the PR comment and review capacity is
   available.
4. Monitor the new head through CI. Do not rerun or weaken the exact-base MJCF
   failure without explicit approval; keep its base-run evidence attached to
   any disposition. Treat the Windows timeout as infrastructure until a rerun
   completes with a product failure.
5. Track the dual-PR follow-up for `10c6b6055e4`: the same over-strict
   zero-DoF soft point-mass assertion exists on `origin/main`, unlike the
   mass-matrix correction (DART 7 still has point-mass mass aggregation
   disabled). Do not fold unrelated main-branch work into #3382.
6. Obtain the remaining competitive-envelope and packet-closeout decisions;
   promote any accepted open work to durable owner docs.
7. Close the final-matrix evidence limitation documented in
   `06-pr-evidence.md` before claiming independently reproducible winner-gate
   completion.
8. After merge, promote durable compatibility/design facts, update the plan
   dashboard, remove the temporary task folder in the completing closeout
   change, and clean branches only with explicit approval.

## Approval boundaries

No push, PR-body edit, CI rerun, review-thread resolution, review trigger,
merge, or branch deletion is authorized by this handoff alone. Preserve the
no-AI-attribution commit/PR rule.
