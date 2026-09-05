# Handling Automated Reviews

How DART agents and contributors handle review comments from AI bot accounts
(Codex, GitHub Copilot, code-quality bots) and run the review-fix loop on pull
requests. Tool compatibility details live in `ai-tools.md`; the approval
boundary itself is axiom 9 of `docs/ai/principles.md`.

When AI agents (Claude Code, Codex, etc.) work on PRs, they may encounter review comments from other AI systems (e.g., Codex bot, GitHub Copilot).

## Independent Review Lane

For substantive code PRs, an independent reviewer session — a human, or a
separate agent session running `/dart-review-pr` that did not author the
change — records findings before merge approval. Docs-only and mechanical
changes are exempt. This complements `@codex review`; it does not replace it.
`dart-manage-pr` checks this gate in `mode=merge`.

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
- If the feedback appears incorrect (false positive):
  1. **Verify** the claim is false by running standalone tests or examining the code
  2. **Add a test** that explicitly documents the correct behavior AND refutes the claim
  3. Example: If Codex claims `hprod([2,3,5,7])` returns 294 instead of 210:
     ```cpp
     EXPECT_FLOAT_EQ(result, 210.0f);  // Verify correct behavior
     EXPECT_NE(result, 294.0f);        // Explicitly refute false claim
     ```
  4. Add the test locally (no comment needed) - the test serves as permanent documentation
- **Follow the review-fix loop** (see workflow below)

This avoids noisy bot-to-bot conversations while still leveraging automated verification.

> **Note**: False positives can recur across reviews. Tests that explicitly refute incorrect claims prevent future confusion and document the verification.

## Codex Review For Draft PRs

For fast-moving work, trigger the first Codex review while the PR is still a
draft when explicit maintainer/user approval covers PR comments. Use a top-level
comment:

```bash
gh pr comment <PR> --body "@codex review"
```

This keeps the PR draft for human readiness while getting automated feedback
early. If a Codex activity signal or submitted review already appears, do not
post a duplicate trigger. After posting, wait for a submitted review, a no-issues
comment, a thumbs-up reaction, or an eyes reaction before treating the trigger as
accepted; do not re-trigger unless there is a concrete timeout/blocker or a
follow-up push addressed Codex findings.

After an approved follow-up push that addresses Codex review comments, request a
fresh top-level Codex review with `@codex review`. This is the normal completion
step for a Codex review-fix round, not an inline reply. A manual trigger is a PR
comment and still requires explicit maintainer/user approval.

## Draft Ready Fast Path

To move quickly without bypassing branch protection, a draft PR can be marked
ready for review once all of these are true on the current head:

- Codex review has no unresolved actionable threads, or the latest Codex result
  is a no-issues comment/reaction.
- Local validation passed after the last pushed change, and the worktree is
  clean: default `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux
  hosts with a visible NVIDIA CUDA runtime.
- PR metadata is correct: base, milestone, title, template, and testing
  evidence match the current branch.

Hosted CI may still be pending when the draft is marked ready. Merge still waits
for branch protection and required checks unless a maintainer explicitly
approves a policy bypass.

## Codex Re-Trigger Cadence And Throttling

After explicit maintainer/user approval for the PR comment, re-trigger Codex at
most once per review-fix round, and only after an approved push that addressed
its comments. Rapid, repeated `@codex review` requests across many quick rounds
can slow or suspend Codex: observed review latency grows round over round and a
later re-trigger can receive no review at all. If Codex stays silent well beyond
its usual turnaround after a re-trigger, treat it as a throttle/timeout blocker,
not a reason to re-request. Weekly usage limits are the same class of blocker:
when quota is exhausted mid-loop, record the converged state plus local
verification (gates, tests, and an independent or role-separated review pass)
instead of waiting for another hosted round. Record the converged state as
evidence — all surfaced findings fixed and their threads resolved — and report
the throttle rather than re-spamming the PR with more triggers.

Codex re-emits every unresolved inline thread verbatim on later rounds, even
when the current head already contains the fix. Verify each re-raised comment
against the current head; once a thread is genuinely addressed, resolve it
(after the approval that covers thread resolution) before the next round —
otherwise the no-issues verdict cannot converge no matter how many rounds run.

## Updating Published PRs

Prefer additive follow-up commits for updates to already-published PRs. This
keeps review history inspectable and makes each review round clear. Pushing any
such update is an external mutation that requires explicit maintainer/user
approval.

### Merge The Base Branch Before Every Push (MANDATORY)

**Before every push, first merge the latest base branch (usually `main`) into
the working branch.** Do this on every push, not just the first, so each
pushed/CI-tested state reflects current `main` and conflicts surface early
instead of at merge time.

```bash
git fetch origin <base-branch>
git merge --no-ff origin/<base-branch>   # never rebase a published PR branch
# rebuild + retest if the merge touched code, then push (an approved mutation)
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
`main`, GitHub may retarget the dependent PR to `main` and mark it behind. Treat
the retargeted branch as the new base state: fetch `main`, merge `origin/main`
into the published PR branch, confirm the diff against `origin/main` still
contains only the intended changes, run the required gates, then push only with
explicit maintainer/user approval. Do not keep acting on stale checks from the
pre-retarget head; the post-merge-base push is the state that matters.

## Review-Fix Loop Workflow

After identifying an AI-generated review comment to address:

1. **Make the code fix**
2. **Run `pixi run lint`** — MANDATORY before every commit (auto-fixes formatting)
3. **Verify explicit maintainer/user approval covers the intended external
   mutations.** Reuse existing authorization for the action, PR, and scope
   (including routine maintenance authorized by `dart-manage-pr`); ask only
   for missing authority. Follow `docs/ai/principles.md`'s shared-state rule.
4. **If approved, commit and push** silently (no reply to the comment)
5. **If approved, resolve the thread** using GraphQL (see commands below)
6. **If the addressed review was Codex, after the approved push, verify
   approval also covers the PR comment (ask only if missing), then re-trigger the
   review**:
   `gh pr comment <PR> --body "@codex review"`
7. **Monitor for results**:
   - New review comments → repeat from step 1
   - "No issues" or 👍 reaction + local validation on the current head: default
     `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts
     → draft PR is ready for human review

Apply the same no-inline-reply handling to `github-code-quality[bot]` findings:
fix valid findings locally, push only after approval, and do not post an
acknowledgment reply. The `@codex review` re-trigger is only for Codex review
rounds.

**Agents MUST:**

- Run `pixi run lint` before EVERY commit (CI will fail otherwise)
- Treat PR comments, pushes, thread resolution, reviewer requests, merges, and
  review re-triggers as external mutations that require explicit approval
- Keep local fixes read-only with respect to GitHub until that approval exists

## GraphQL Commands for Thread Resolution

```bash
# List unresolved threads (get thread IDs)
gh api graphql -f query='
  query {
    repository(owner: "dartsim", name: "dart") {
      pullRequest(number: PR_NUMBER) {
        reviewThreads(first: 20) {
          nodes { id isResolved path line }
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

## Autonomous Review-Fix-Monitor Loop

For agents iterating on automated reviews, the complete loop is:

```
1. Fetch latest review comments
2. For each comment:
   a. Implement the fix (or add a test refuting a false positive)
   b. Run `pixi run lint` (MANDATORY)
   c. Build and run relevant tests
   d. Verify explicit approval covers each push or PR mutation; ask only if missing
3. If approved, commit and push silently (no reply to bot comment)
4. If approved, resolve addressed threads via GraphQL
5. If the addressed review was Codex, after the approved push, verify approval
   also covers the PR comment (ask only if missing), then re-trigger:
   `gh pr comment <PR> --body "@codex review"`
6. For non-Codex bot findings, including `github-code-quality[bot]`, do not
   re-trigger Codex solely for those fixes unless Codex review comments were
   also addressed in the same push
7. Monitor CI: `gh pr checks <PR>`
8. Wait for new review (poll with `gh api repos/dartsim/dart/pulls/<PR>/reviews`)
9. If new review has comments → go to step 2
10. If no new comments AND local validation passed on the current head (default
    `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts)
    → mark draft PRs ready for human review after approval
11. Keep monitoring hosted CI until required checks pass before merge
```

**Checking for new reviews:**

```bash
# List all reviews with timestamps
gh api repos/dartsim/dart/pulls/<PR>/reviews \
  --jq '.[] | "ID:\(.id) User:\(.user.login) State:\(.state) At:\(.submitted_at)"'

# Fetch comments from a specific review
gh api repos/dartsim/dart/pulls/<PR>/reviews/<REVIEW_ID>/comments \
  --jq '.[] | "File:\(.path) Line:\(.line // .original_line) Body:\(.body)"'
```

**Monitoring CI:**

```bash
# Check all CI status checks
gh pr checks <PR>

# Watch until all checks complete (useful for waiting)
gh pr checks <PR> --watch
```

During DART's long CI matrix (the full run can take a couple of hours, with
`Release Tests` as the long pole), `gh pr checks --watch` can exit early on a
transient network error (for example a dropped `api.github.com` connection) and
look like completion. For long runs prefer a resilient poll that re-queries
`gh pr checks <PR>` on an interval, tolerates transient failures, and stops only
when nothing is pending, any check fails, or the head SHA moves.

**Stop conditions:**

- Codex review returns no comments (or only 👍 reactions)
- Local validation passes on the current head for draft-ready state: default
  `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux CUDA hosts
- All required CI checks pass for merge-ready state
- Pre-existing failures (e.g., `simulation` "Not Run") can be ignored

---
