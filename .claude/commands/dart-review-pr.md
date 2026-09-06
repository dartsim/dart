---
description: review a PR or address review feedback
argument-hint: "<pr-number> [mode=review|feedback] | candidate=<id> scope=correctness|contracts|non-substantive"
agent: build
---

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-reviews.md
@docs/onboarding/ai-tools.md
@docs/ai/verification.md

## Workflow

Pick the sub-workflow from `mode=` in `$ARGUMENTS`, defaulting to `review`.

### Review A Local Candidate Or PR

For `candidate=<id>`, read its `candidate.json` at the path printed by
`review-gate prepare`. Verify the supplied base/head/tree and inspect
`git diff <merge_base> <head>` with surrounding code. Work from the immutable
candidate in an isolated read-only checkout; do not accidentally review dirty
files or a later HEAD. No PR needs to exist. The parent supplies objective,
acceptance criteria, factual gates, prior findings, and author-session IDs.

For a PR number, obtain its current head/base and complete diff with
`gh pr view` and `gh pr diff`, then follow the same coverage policy. A PR review
without a prepared local candidate is useful feedback, not publication evidence.

Apply the assigned scope from `docs/onboarding/ai-reviews.md`: correctness
covers the complete PR diff and acceptance evidence; contracts independently
traces consumers, sibling cases, and negative cases and records the required
input/consumer matrix for exclusions, parsers, or validators. Challenge test
oracles against actual requirements. A non-substantive assessment must prove
unchanged behavior under the owner's strict baseline rules. Missing evidence
or unobserved effective reviewer settings makes the report incomplete.

Use a distinct non-author session for each substantive scope. Check code style,
tests, docs, and focused commits. For 3D claims,
require the `dart-verify-sim` text oracle plus assessed visual/debug evidence,
or a justified replacement. Report every surviving finding as a coherent batch, including
repair regressions and earlier findings whose disposition is unsupported.

Stay read-only. For a local candidate return the final JSON report defined in
`docs/onboarding/ai-tools.md` for the parent to import with `review-gate record`.
Include observed session/model/effort, coverage, completion, findings with stable
IDs and concrete evidence, and verified dispositions. Do not mutate the evidence
store yourself. A clean verdict requires complete coverage for the current stage under the
review owner; explicitly retain pending hosted acceptance checks.

### Address Feedback

Use the paginated review/CI inspection commands and the single Review-Fix Loop
Workflow in `docs/onboarding/ai-reviews.md`. Collect the completed batch, verify
claims, and repair the underlying defect family. That owner defines trigger
ownership, current-head completion, the two-round strategy checkpoint, false
positive dispositions, blockers, and readiness; do not restart a per-comment
fix/push/review loop here.

For published PRs, prefer a new
follow-up commit so reviewers can inspect each round; amend or force-push only
after explicit maintainer/user approval and only when the user requests it or a
clear reason exists (removing sensitive content, repairing branch history).

Run the relevant local gates, including `pixi run lint` before every commit.
Merge the latest base, validate and pass the independent local review gate
before each approved push, and apply the owner's remote
divergence recovery if the head moved. Reuse existing explicit authority for
this PR, action, and scope; ask only where it is missing. No inline bot replies.
Monitor CI (`gh pr checks $1`); readiness and merge remain separately gated and
require approval for the corresponding external mutation.

## Output

- PR number and whether the pass was a review or a feedback round
- Findings or fixes applied, with file/line references
- Reviewed head, completion/trigger evidence, completed round count, and any
  strategy-checkpoint outcome
- Which actions were local-only and which external mutations were explicitly
  approved
- Codex/CI state and any remaining blocker
