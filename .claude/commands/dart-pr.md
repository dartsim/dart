---
description: create a branch, commit, push, and open a DART pull request
argument-hint: "[base=<branch>] [draft]"
agent: build
---

Prepare the PR locally within the authorized task: $ARGUMENTS
Before an action requiring explicit maintainer/user approval under the owner
docs, verify that existing authorization covers its action, target, and scope.
Ask only for missing or changed authority after completing authorized preparation.

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ai-reviews.md
@docs/ai/verification.md
@docs/onboarding/changelog.md
@.github/PULL_REQUEST_TEMPLATE.md

## Drafting the PR

Follow the PR-writing guidance in
`docs/onboarding/contributing.md#submitting-a-pull-request` and fill the compact
PR template. Keep titles plain, scoped, and outcome-focused, without agent
prefixes. Recent PRs can supply relevant context; use the current owner guidance
rather than copying their length or structure.

Draft from the final diff and the reason for it, not from the session report.
Before publication, read the rendered body as a reviewer: the opening must
explain the concrete problem and solution, and the rest must earn its space
through review-relevant rationale or evidence. Short but generic is not enough.

For 3D structure or behavior changes (model/scene, dynamics, collision/contact,
simulation, rendering, GUI, visual examples), use `dart-verify-sim`. Preserve the
owner's visible media, assessed comparisons, text oracle, claim boundaries, and
reproduction evidence. Publish transient evidence with `pixi run evidence-publish`
as described in `docs/onboarding/agent-sim-verification.md`; never commit
transient evidence. A compact template does not reduce these requirements.

## Workflow

1. Inspect scope:
   ```bash
   git status --short --branch
   git diff --stat
   git diff --check
   ```
2. Exclude unrelated dirty files unless the user explicitly includes them.
3. Choose the target branch and milestone:

   | Target                          | Milestone                      |
   | ------------------------------- | ------------------------------ |
   | `main`                          | `DART 7.0`                     |
   | Active DART 6 LTS `release-6.*` | Branch-matching DART 6.x patch |

4. For bug fixes, use the dual-PR flow: fix the active DART 6 LTS branch first,
   then cherry-pick or reapply to `main`.
5. Before every commit, run `pixi run lint`. Also run `pixi run build` for C++
   or Python changes and focused tests for behavior changes.
6. Create or update a topic branch when needed:
   ```bash
   git checkout -b <type>/<topic> origin/<target-branch>
   ```
7. Commit only intended files with a plain descriptive commit title.
8. Merge the latest base branch into the PR branch before any push, and follow
   the base-merge and automated-review rules in `docs/onboarding/ai-reviews.md`
   (no inline bot replies; one trigger owner and completed fix batches).
   Commit and validate the immutable candidate, obtain two clean non-author
   local reviews (or the evidenced trivial exception), and pass
   `pixi run review-gate check <candidate>` before publication. Complete this
   local work before asking for missing publication approval.
   Verify explicit maintainer/user approval covers pushing and opening the draft
   PR; ask only for missing authority. With that approval:
   ```bash
   git push -u origin HEAD
   gh pr create --draft --base <target-branch> --milestone "<milestone>" \
     --title "<plain title>" --body-file <filled-template-file>
   ```
9. Prefer additive follow-up commits for updates to a published PR. Amend or
   force-push only after explicit maintainer/user approval and only when the
   user requests it or a clear reason exists (removing sensitive content,
   repairing branch history).
10. Invoke the `dart-changelog` routine for the changelog decision, entry
    wording, and PR-link follow-up. If `CHANGELOG.md` needs the PR number, keep
    the follow-up changelog commit local unless existing explicit maintainer/user
    approval covers its push or PR update; otherwise request only the missing
    authority.
11. Follow the single Review-Fix Loop Workflow in
    `docs/onboarding/ai-reviews.md`, reusing explicit approval for its action,
    PR, and scope. Account for the initial automatic review before requesting
    another; apply the strategy checkpoint and current-head readiness gates.
    Monitor CI: `gh pr checks <PR_NUMBER>`.

## Output

- Branch, target base, and milestone used
- Commit titles and files included
- PR URL and draft/ready state, or the prepared PR text awaiting approval
- Changelog decision
- CI status and any remaining blocker
