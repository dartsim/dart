# Local And Automated Reviews

How DART agents and contributors handle review comments from AI bot accounts
(Codex, GitHub Copilot, code-quality bots) and run the review-fix loop on pull
requests. Tool compatibility details live in `ai-tools.md`; the approval
boundary itself lives in `docs/ai/principles.md`.

## Independent Local Review Before Publication

Before every substantive branch push, including the first draft publication,
merge the latest fetched base, commit intended changes, and run the required
validation. Prepare that immutable commit with `pixi run review-gate` using
[the local evidence interface](ai-tools.md#local-review-evidence-interface).
The policy applies regardless of the authoring tool or file extension.

Require two completed, clean reviews from distinct sessions that did not author
any part of the candidate:

- **Correctness**: inspect the complete PR diff from its merge base, relevant
  surrounding code, acceptance criteria, and tests. Review the resulting
  behavior, including interactions introduced by repairs.
- **Contracts**: independently challenge consequential assumptions. Trace
  affected consumers, sibling cases, and negative cases. For exclusions,
  parsers, and validators, record an input/consumer coverage matrix: input
  class, actual consumer or contract, expected behavior, and evidence. Derive
  expected behavior from consumers and requirements; tests that repeat the
  implementation's assumptions do not establish correctness. Documentation,
  packaging metadata, examples, and instructions can be executable inputs.
  For broad exclusions, inventory every excluded rule and search repository-wide
  for build/install declarations and test/runtime file reads. Cross-check both
  directions: excluded paths to consumers, and consumer inputs to exclusions.
  A row named only "ordinary docs" cannot discharge an unexplored subtree.
  Check representative edits and deletions for each consumer class, and verify
  that claimed cheaper checks preserve the original consumer's semantics.
  Continue through the inventory after finding the first related issue.

For changed runbooks and agent handoffs, reconcile state, next actions, paths,
and action prerequisites against the supplied candidate/PR facts. Read the
instructions as an operator would; outdated actionable steps require correction
or an evidenced explanation of why their guards make them safe. Record this
assessment even when the production code is otherwise sound.

Use separate read-only sessions through `dart-review-pr` with a local candidate
and scope. Give each the objective, repository guidance, candidate identity,
factual validation, known limitations, and earlier findings to verify. Do not
substitute the author's fix narrative for inspection. Record effective model
and effort for agent reviewers from observable session settings; if unavailable,
report the limitation and obtain a reviewer with observable settings. Project
configuration remains unpinned; explicit user model restrictions still apply.
The implementation author cannot serve as either publication reviewer.

Fix the complete accepted finding batch, rerun affected gates, and prepare a new
candidate. Every substantive change needs a fresh pair of reviews covering
the new full candidate; earlier findings and evidenced dispositions carry
forward. An incomplete report, missing required acceptance coverage, or a later
clean response without dispositions cannot erase an outstanding finding.
Reject false positives with concrete evidence; add tests only to close gaps.

A genuinely trivial change may use one independent **non-substantive**
assessment. It must inspect behavior and explain why it cannot change. For an
update it must name an already reviewed ancestor baseline with the same base
and assess the entire intervening delta. Extensions such as `.md` cannot grant
exemptions: agent instructions, docs consumed by tests, and packaging inputs
remain substantive. Formatting-only changes still need that recorded assessment.

These passes satisfy the local completion requirement; do not add redundant
local passes to count overlapping wording. Hosted review remains an additional
current-head publication/ready/merge requirement under the rules below. Missing
independent sessions or acceptance evidence blocks publication while useful
local work continues; author self-review cannot substitute for this gate.

Review completeness follows the current stage. Before an initial publication,
reviewers must inspect all required local evidence and explicitly list hosted
acceptance checks that can run only after publication. Those checks remain
pending in the draft PR; they are not platform passes or evidence of readiness.
A failed required local check cannot be relabeled as a pending hosted check.

The installed pre-push hook checks every outgoing branch commit against these
local records, including non-HEAD and multi-ref pushes. It performs no network,
model, build, or test work. It checks the locally fetched base; contributors
remain responsible for fetching current remote state before publication.
Never use hook bypasses automatically to escape a failed gate. Evidence presence
and consistency do not authenticate reviewers or guarantee their correctness.

## Detecting AI-Generated Reviews

**Bot usernames always end with `[bot]`:**

- `chatgpt-codex-connector[bot]` — Codex automated reviews
- `github-actions[bot]` — GitHub Actions automated comments
- `github-code-quality[bot]` — GitHub code-quality review comments
- `copilot[bot]` — GitHub Copilot suggestions

**If the reviewer username ends in `[bot]`, treat it as an automated review.**

## Rules for AI Agents (CRITICAL)

**NEVER reply to AI-generated review comments. This means:**

- ❌ **NO** `gh pr comment` commands responding to bot feedback
- ❌ **NO** PR comment replies acknowledging or addressing bot feedback
- ❌ **NO** comments like "Addressed the Codex review feedback"
- ✅ **YES** Make the local code fix silently
- ✅ **YES** Verify explicit maintainer/user approval covers each push, PR
  comment, thread resolution, reviewer request, merge, or review re-trigger;
  ask only for missing authority, per `docs/ai/principles.md`

**The code change IS the response. No acknowledgment needed.**

**Guidance for AI agents addressing automated reviews**:

- Address feedback locally; reuse existing authorization for its action, PR,
  and scope, and ask before external mutations only where authority is missing
- If the feedback is valid, implement the fix without commenting
- If feedback is a false positive, record its rejection with code, existing
  tests, or other concrete evidence in the current task/PR verification notes.
  Add a regression test only when it closes a coverage gap; do not change code
  merely to reject a claim or obtain a clean bot verdict.
- Treat repeated comments as claims to recheck against the current head.
  Addressed threads can recur, but recurrence is not a product guarantee or
  proof that a fix failed. Resolve only individually verified, addressed
  threads with approval; thread resolution itself does not prove correctness.
- Follow the single review-fix loop below for all automated reviewers.

## Codex Review For Draft PRs

The settings and their limitations live in
[ai-tools.md](ai-tools.md#codex-hosted-review-settings). With the recommended
PR-open trigger, automatic review owns the initial eligible event; the agent
owns deliberate follow-ups. Check for an automatic run, pending request, or
completed review of the intended head before requesting one manually. Absence
of a reaction immediately after an event does not prove no run was queued.

A stable draft can receive early feedback with a top-level comment when no
review already covers or is queued for its head and explicit maintainer/user
approval covers PR comments:

```bash
gh pr comment <PR> --body "@codex review"
```

If every-push or experimental smart detection is configured instead, establish
who owns the next request before manually triggering. Do not race an automatic
run or rely on smart detection as proof that the current head was reviewed.

Keep a compact record in the existing task/PR evidence: head SHA, trigger owner
and comment/run ID, requested time, completion evidence, completed round count,
and finding dispositions. Preserve it across handoffs; no separate review
service or ledger is required. Poll bounded status summaries and fetch new
findings once, rather than repeatedly loading the full history into context.

An eyes reaction acknowledges a request; it is not completion. A submitted
review, no-issues comment, or bot thumbs-up must be associated with the intended
head. For reactions without a SHA, require an unambiguous request-to-head link
and no intervening push; otherwise coverage is unknown. With Exhaustive enabled,
the first posted finding or submitted review may not be the final batch: inspect
the associated task/run completion when available. If completion cannot be
established, record that uncertainty and monitor; do not infer completion from
silence. Local investigation may proceed while the run finishes, but collect
the final batch before the next approved push or review request.

## Draft Ready Fast Path

To move quickly without bypassing branch protection, a draft PR can be marked
ready for review once all of these are true on the current head:

- Codex review completed on the current head and all findings have verified
  dispositions with no unresolved actionable issues. Rejected false positives
  need evidence, not another identical review. A clean result on an older head
  or resolving threads alone does not satisfy this gate.
- Local validation passed after the last pushed change, and the worktree is
  clean: `pixi run test-all`, plus the Gazebo compatibility gate when the touched
  surface requires it.
- PR metadata is correct: base, milestone, title, template, and testing
  evidence match the current branch.

Hosted CI may still be pending when the draft is marked ready. Merge still waits
for branch protection and required checks unless a maintainer explicitly
approves a policy bypass.

## Codex Re-Trigger Cadence And Throttling

Request at most one Codex run per intended head, after a validated fix batch or
another change leaves the required current-head coverage missing, and only with
approval for the PR comment. A non-Codex finding is not itself a trigger; the
resulting change and missing review coverage determine whether one is needed.

If Codex remains silent beyond its observed turnaround, inspect run state and
quota before diagnosing a timeout; latency alone does not prove throttling.
Do not spam retries. After a confirmed failed/cancelled run, retry once only
when its cause is cleared, no run is pending, and existing approval covers it.
Weekly quota exhaustion or inaccessible completion evidence is a blocker:
finish useful local investigation/validation, record known findings and the
missing hosted evidence, and report the blocker. Do not enable credits, claim a
clean review, or bypass readiness gates to make the loop finish.

## Updating Published PRs

Prefer additive follow-up commits for updates to already-published PRs. This
keeps review history inspectable and makes each review round clear. Pushing any
such update is an external mutation that requires explicit maintainer/user
approval.

### Merge The Base Branch Before Every Push (MANDATORY)

**Before every push, first merge the latest base branch into
the working branch.** Do this on every push, not just the first, so each
pushed/CI-tested state reflects the current target base and conflicts surface early
instead of at merge time.

```bash
git fetch origin <base-branch>
git merge --no-ff origin/<base-branch>   # never rebase a published PR branch
# commit, validate, prepare and record local reviews, then review-gate check
# push only with approval
git push
```

Merging the base in locally is a routine pre-push step. The `git push` itself is
still an external mutation that requires explicit maintainer/user approval. Do
not rebase published PR branches by default: rebasing invalidates existing CI
runs and makes PR review/comment history harder to follow. Rebase or force-push
only when the maintainer explicitly requests it.

Amend or force-push only when the user explicitly requests it or when there is a
clear reason, such as removing sensitive content, repairing broken branch
history, or cleaning up noisy local work before the PR is first published.
Force-pushes are PR mutations and require explicit maintainer/user approval.

If a push is rejected because the remote PR branch moved, fetch the PR branch
and inspect the local/remote divergence before retrying. When the remote already
contains an equivalent fix, validate the remote PR head instead of pushing a
duplicate follow-up commit, and realign the local branch to the remote head
after preserving any useful local-only work on a backup branch. Do not leave the
main working checkout detached or visibly diverged while continuing PR
management; it makes later status, IDE branch indicators, and CI evidence easy
to misread.

If a PR was temporarily based on another PR branch, and that base PR lands into
its target branch, GitHub may retarget the dependent PR and mark it behind. Treat
the retargeted branch as the new base state: fetch and merge its latest remote ref
into the published PR branch, confirm the diff against that ref still
contains only the intended changes, run the required gates, then push only with
explicit maintainer/user approval. Do not keep acting on stale checks from the
pre-retarget head; the post-merge-base push is the state that matters.

## Review-Fix Loop Workflow

1. **Collect the completed batch.** Verify head and run completion; paginate
   reviews, inline threads/comments, and relevant issue comments/reactions.
   Include human feedback. Group findings by the invariant they challenge;
   record accepted, rejected-with-evidence, or explicitly deferred dispositions.
   Do not silently defer a valid in-scope defect to reach a clean verdict.
2. **Check convergence before another round.** Count completed hosted runs,
   including the initial automatic run, not individual comments or internal
   Exhaustive passes. If the second run still leaves valid issues, perform the
   strategy checkpoint below before requesting a third run. Keep the count
   across pushes and handoffs; later rounds with valid issues require the same
   checkpoint again.
3. **Repair the defect family.** Inspect related callers, sibling cases, and
   interactions before implementing all accepted findings as a coherent batch.
   For parsers, cover syntax boundaries, reserved delimiters, and literal versus
   interpreted content; for validators, define valid inputs and a negative-case
   matrix rather than appending exclusions one at a time. Keep unrelated
   features out of the repair batch.
4. **Verify the batch locally.** Run focused regression gates. Fix repair-induced
   regressions before publication. Run `pixi run lint` before every commit;
   merge the latest base and rerun affected gates, then obtain the two clean
   independent local reviews (or evidenced non-substantive assessment) and
   pass `pixi run review-gate check <candidate>` before each approved push.
5. **Publish once the batch is ready.** Verify explicit approval covers each
   intended push, thread resolution, and PR comment; reuse existing authority
   for this action, PR, and scope (including `dart-manage-pr` maintenance).
   With approval, push silently and resolve only individually verified,
   addressed threads. Apply the trigger ownership/cadence rules above for one
   follow-up review of the resulting head, never one request per finding.
6. **Assess completion.** Monitor the run and CI; new findings return to batch
   assessment and the convergence checkpoint. Apply the draft-ready and merge
   gates, or report the specific external blocker. A round budget never grants
   readiness or excuses a valid defect.

### Strategy Checkpoint

Pause automatic re-triggering and perform a bounded independent root-cause
review of the affected subsystem. Use the existing independent reviewer when
possible; on the final candidate this can also supply its contracts pass. If an independent
reviewer is unavailable, continue useful local investigation and validation,
but report the missing checkpoint and hold further hosted requests. Existing
trigger approval does not waive this checkpoint, including for docs-only PRs.

Distinguish missed sibling cases, incomplete fixes, regressions introduced by
repairs, false positives, and newly expanded scope. Record the underlying
contract, a coherent repair approach, and regression cases that could disprove
it in existing verification evidence. Complete those checks before the next
approved request. Continue within existing authorization; this checkpoint is
not a routine permission stop. Ask only when repair requires a consequential
scope decision or missing authority. Preserve unresolved findings explicitly.

## GraphQL Commands for Thread Resolution

```bash
# List all unresolved threads; replace PR_NUMBER with the integer PR number.
gh api graphql --paginate -f query='
  query($endCursor: String) {
    repository(owner: "dartsim", name: "dart") {
      pullRequest(number: PR_NUMBER) {
        reviewThreads(first: 100, after: $endCursor) {
          nodes { id isResolved isOutdated path line }
          pageInfo { hasNextPage endCursor }
        }
      }
    }
  }
' --jq '.data.repository.pullRequest.reviewThreads.nodes[] | select(.isResolved == false)'

# Resolve a thread by ID only after explicit maintainer/user approval
gh api graphql -f query='
  mutation {
    resolveReviewThread(input: {threadId: "PRRT_xxxx"}) {
      thread { isResolved }
    }
  }
'

# Resolve only reviewed, addressed thread IDs after approval. Do not
# bulk-resolve unresolved threads; that can hide human feedback or unresolved
# bot findings.
```

**Why resolve after explicit maintainer/user approval**: Clicking "Resolve
conversation" in the UI adds no comment noise. The code change is the response;
the resolved thread shows the fix was acknowledged after the maintainer/user
approved the PR mutation.

## Review And CI Monitoring

```bash
# List all reviews with reviewed commits and timestamps.
gh api --paginate repos/dartsim/dart/pulls/<PR>/reviews \
  --jq '.[] | {id, user: .user.login, state, commit_id, submitted_at}'

# Fetch all inline comments, including replies and review associations.
gh api --paginate repos/dartsim/dart/pulls/<PR>/comments \
  --jq '.[] | {id, pull_request_review_id, in_reply_to_id, user: .user.login, path, line, original_line, commit_id, body}'

# Find request/completion comments, then reactions on the relevant comment.
gh api --paginate repos/dartsim/dart/issues/<PR>/comments \
  --jq '.[] | {id, user: .user.login, created_at, body}'
gh api --paginate repos/dartsim/dart/issues/comments/<COMMENT_ID>/reactions \
  --jq '.[] | {user: .user.login, content, created_at}'
```

**Monitoring CI:**

```bash
# Check all CI status checks
gh pr checks <PR>

# Watch until all checks complete (useful for waiting)
gh pr checks <PR> --watch
```

During DART's long CI matrix, `gh pr checks --watch` can exit early on a
transient network error (for example a dropped `api.github.com` connection) and
look like completion. For long runs prefer a resilient poll that re-queries
`gh pr checks <PR>` on an interval, tolerates transient failures, and stops only
when nothing is pending, any check fails, or the head SHA moves.

Use the readiness gates above and `dart-manage-pr` for merge requirements.
Classify pre-existing, skipped, cancelled, and external failures with evidence;
do not treat a required check that never ran as a pass.

## Workflow Validation Cases

For changes to this policy, use fresh read-only sessions with the relevant
workflow entrypoint and hypothetical PR state. Ask for next actions and
readiness evidence without authorizing real GitHub mutations. Evaluate decisions
against these cases; structural routing checks alone do not exercise the loop.

| Case                                                          | Expected decision                                                                         |
| ------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| Automatic review queued, no reaction yet                      | Inspect/wait; no duplicate manual trigger                                                 |
| Exhaustive run posts findings but completion is unknown       | Investigate locally; hold publication/re-trigger until final batch is established         |
| Multiple related valid findings                               | Repair and test the underlying family in one batch                                        |
| Second completed run finds a repair regression                | Independent strategy checkpoint before another approved request; no lowered gate          |
| Old clean result, new head, all threads resolved              | Current-head review coverage is missing                                                   |
| More than 100 threads/comments                                | Paginate to exhaustion; examine late human and bot findings                               |
| Existing test disproves the only finding on the reviewed head | Record rejection; no gratuitous test, push, or repeat review                              |
| Quota exhausted after fixes                                   | Finish local evidence, report missing hosted review; no credits change or readiness claim |
| Clean current-head review with required local/CI evidence     | Stop reviewing; take only the authorized readiness/merge transition                       |

### Local Gate Behavioral Replay (2026-09)

The implementation trial used archived #3485 commits `1963de86858` (base
`57b9cfe49bf`) and `4562d976903` (base `fda07ac5671`), with the corresponding
newly published draft and already-open PR state. Read-only reviewer sessions
received the relevant workflow, full candidate diff, and factual state. The
historical findings were withheld. Session settings confirmed GPT-6 Astra Max.

An initial review using the existing instructions missed the known exclusion
family. After one revision of the proposed instructions, the final correctness
and contracts pair together reported these historical cases:

| Historical case | Final revised pair |
| --------------- | ------------------ |
| Root README consumed by packaging | Found in the initial snapshot; recognized as fixed in the update |
| Demo README consumed by Python tests | Found |
| LICENSE consumed by installation | Found |
| Tutorial documentation consumed by build/install | Found |
| Python API documentation consumed by a Python test | Missed |
| Handoff still instructing an already-open PR to be created | Found |

The missed Python API case is a concrete limit: the archived
`python/tests/unit/gui/test_gui_scene.py` reads
`docs/python_api/modules/gui.rst`; retained documentation checks do not prove
that test's contract. The batch found the underlying consumer family and stale
handoff, but did not exhaust all members of the family.

A separate blind contrast review accepted a clean README typo as non-substantive
and rejected an unrelated validator repair that coerced noncanonical inputs
with `int(raw)`. It identified boolean/float/string acceptance and an unhandled
infinity case against the supplied exact ASCII-decimal contract.

This small qualitative exercise had unequal review scopes and no controlled
cost comparison. It establishes neither universal detection nor fewer hosted
iterations. The [ten-PR trial](ai-tools.md#codex-hosted-review-settings) owns
ongoing measurement of local review time, escaped families, and total cost.
