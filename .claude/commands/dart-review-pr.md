---
description: review a PR or address review feedback
argument-hint: "<pr-number> [address]"
agent: build
---

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-tools.md (for AI-generated review handling)
@docs/ai/verification.md

## Workflow

### To Review

```bash
gh pr view $1 && gh pr diff $1
```

Check code style, tests, docs, and focused commits. For claims involving 3D
structure or behavior, require the `dart-verify-sim` text oracle plus assessed
claim-tied OSG/debug-overlay evidence, or a justified replacement; a screenshot
alone is not correctness evidence.

### To Address Feedback

```bash
gh pr view $1 --comments
```

Apply minimal fixes locally and verify. Do not push, comment, resolve threads,
or re-trigger review without explicit maintainer/user approval for that
external mutation.

For published PRs, prefer a new follow-up commit for review fixes so reviewers
can inspect what changed since the previous round. Amend or force-push only
after explicit maintainer/user approval and only when the user explicitly
requests it or when there is a clear reason such as removing sensitive content
or repairing broken branch history.

Before every push, first merge the latest base branch into the PR branch (on
every push, not just the first) so each pushed/CI-tested state reflects current
target base branch and conflicts surface early: `git fetch origin <base>` then
`git merge --no-ff origin/<base>`, rebuild/retest if the merge touched code, then
push. The local base merge is a routine pre-push step; the push itself still
requires explicit maintainer/user approval. Do not rebase a published PR branch
by default because it invalidates existing CI runs and makes PR review/comment
history harder to follow. Rebase or force-push only when the maintainer
explicitly requests it.

If the push is rejected because the remote PR head moved, fetch and compare the
remote head before retrying. If it already contains an equivalent review fix,
validate that head and follow `docs/onboarding/ai-tools.md` instead of pushing a
duplicate commit.

## Automated Reviews (Codex, Code Quality, Copilot, etc.)

When a draft PR is first published, request the first Codex review with a
top-level `@codex review` once explicit maintainer/user approval covers PR
comments; it can run while the PR remains draft. If Codex already shows an
activity signal or submitted review, do not post a duplicate trigger.

1. Make the local fix silently (no reply)
2. Run the relevant local gates, including `pixi run lint` before any commit
3. Ask for explicit maintainer/user approval before push, thread resolution,
   PR comment, or review re-trigger
4. If approved, push the fix silently
5. If approved, resolve only reviewed and addressed thread IDs via GraphQL (see
   ai-tools.md)
6. After the approved push, if the fixes addressed Codex review comments, ask
   for explicit maintainer/user approval for the PR comment, then re-trigger:
   `gh pr comment $1 --body "@codex review"`
7. Also handle `github-code-quality[bot]` review comments with the same
   no-inline-reply loop. Fix valid findings locally and push silently after
   approval; do not re-trigger Codex solely for non-Codex bot findings unless
   Codex comments were also addressed.
8. Monitor CI: `gh pr checks $1`
9. Check for new review, repeat until no actionable comments remain
10. For draft PRs, mark ready after explicit approval once Codex is clean and
    local validation passes on the current head: `pixi run test-all` for
    build coverage, `pixi run test` when C++ runtime behavior could be
    affected, `pixi run test-py` when Python behavior could be affected, plus
    the Gazebo gate when package or downstream compatibility could be affected;
    merge still waits for required hosted checks unless a maintainer explicitly
    approves a policy bypass

Full iterative loop: `docs/onboarding/ai-tools.md` § "Autonomous Review-Fix-Monitor Loop"

## Output

- Review findings ordered by severity, or the feedback addressed
- Verification commands run for addressed feedback
- Any external mutation left pending explicit approval
