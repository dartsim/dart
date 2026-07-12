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
- Latest completed handoff commit: `dbfed2fdd88` (`Refresh deformable PR
  stabilization handoff`)
- Latest durable-owner commit: `574dc2a28cf` (`Promote deformable closeout
  owners`)
- Latest balanced-evidence runner commit: `9a7bab76948` (`Add balanced soft-body
  detector runner`)
- Published head: `origin/wp-db-native-soft-fallback` at `b25462ca5c0`
- Local state: the implementation and following handoff/durable-owner updates
  are not published; verify the exact current HEAD and ahead count
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

## 2026-07-12 balanced-evidence runner packet

Local commit `9a7bab76948` adds the bounded replacement for the manual
native-vs-DART A/B method. It pins one clean HEAD in a detached dedicated
Release/profile build, qualifies checksums at threads 1 and 16, keeps each
row's warmup adjacent to its 20 alternating measured pairs, gates load and
available thermal sensors before and across each pair, preserves raw JSON/logs
and idle history, and writes authoritative `COMPLETE.json` last.

Verification on that commit:

- focused synthetic and mocked-orchestration tests: 37/37 passed;
- full Python suite: 212/212 passed;
- `pixi run lint`: passed;
- `pixi run check-ai-commands`: passed;
- `git diff --check`: passed;
- independent protocol/evidence review: clean;
- independent adversarial runner/test review: clean.

No real timing run was used as verification for the runner itself. The final
paired artifact remains an explicit next action once the shared host is idle
and cool.

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
  the expected-fastest detector gate. Manual interleaved A/B results suggest a
  tie on only two single-thread rows; they do not resolve the five failed rows.
  The original matrix command and recovered scratch method are now documented
  in `06-pr-evidence.md`, including why the scratch rows do not satisfy the
  gate. Local commit `9a7bab76948` now provides the reviewed, revision-pinned
  replacement runner, but no final artifact exists yet. The 2026-07-12 host
  probe still showed sibling builds, 1-minute load `14.74`, and package
  temperature `100 C`; running under that state would be invalid. Capture the
  balanced artifact once the runner's own idle/thermal gates can pass, or
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

## Next actions

1. Obtain explicit maintainer/user approval for the follow-up push and related
   PR mutations.
2. Immediately before any push, fetch `release-6.20`. If it moved, merge
   `origin/release-6.20` into the published topic branch (never rebase), resolve
   conflicts, and rerun the relevant gates on the merged state.
3. Push the complete unpublished stack: review fix `2ad156e7b82`, handoff
   `dbfed2fdd88`, durable owners `574dc2a28cf`, runner `9a7bab76948`, and this
   final handoff refresh. Resolve the addressed automated-review thread only if
   approval covers thread mutation, and request a fresh top-level Codex review
   only if approval covers the PR comment and review capacity is available.
4. Monitor the new head through CI. Do not rerun or weaken the exact-base MJCF
   failure without explicit approval; keep its base-run evidence attached to
   any disposition. Treat the Windows timeout as infrastructure until a rerun
   completes with a product failure.
5. Track the dual-PR follow-up for `10c6b6055e4`: the same over-strict
   zero-DoF soft point-mass assertion exists on `origin/main`, unlike the
   mass-matrix correction (DART 7 still has point-mass mass aggregation
   disabled). Do not fold unrelated main-branch work into #3382.
6. Obtain the remaining competitive-envelope and flexible-foot decisions.
   Keep the already-created durable background/design owners and PLAN-622
   synchronized with the result.
7. On the final clean local head and only when no sibling build is active, run:

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
8. After merge, audit the new durable compatibility/design/reference owners,
   update PLAN-622, remove the temporary task folder in the completing closeout
   change, and clean branches only with explicit approval.

## Approval boundaries

No push, PR-body edit, CI rerun, review-thread resolution, review trigger,
merge, or branch deletion is authorized by this handoff alone. Preserve the
no-AI-attribution commit/PR rule.
