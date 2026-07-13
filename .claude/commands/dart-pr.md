---
description: create a branch, commit, push, and open a DART pull request
argument-hint: "[title-or-topic]"
agent: build
---

Prepare or open a DART pull request after explicit maintainer/user approval:
$ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ai-tools.md
@docs/onboarding/changelog.md
@.github/PULL_REQUEST_TEMPLATE.md

## Recent PR Patterns

When the expected PR style is unclear, inspect recently merged PRs before
drafting the title or body:

```bash
gh pr list --repo dartsim/dart --state merged --base <target-branch> --limit 10 \
  --json number,title,body,mergedAt
```

Use these practices:

- Keep titles plain, scoped, and outcome-focused. Do not add agent prefixes.
- Fill the PR template in DART's default order: Summary, Motivation / Problem,
  Changes / Key Changes, optional Before / After, Testing, Breaking Changes,
  and Related Issues / PRs. Keep Summary first because reviewers need the
  skimmable outcome before the rationale. If the motivation is necessary to
  understand the outcome, make the first Summary sentence problem-oriented,
  then put the fuller why in Motivation / Problem rather than moving Motivation
  above Summary.
- Write the opening Summary and Motivation from the perspective of a user or
  downstream maintainer who is not already familiar with the implementation:
  lead with what changes for them, what stays compatible, how they would opt in
  or migrate, and why the evidence matters. Keep implementation details in
  Changes unless they explain a user-visible outcome or risk.
- When the change has meaningful user-facing API, workflow, behavior, or
  performance impact, add a concise `## Before / After` section before
  Testing. Use a small table or bullets that cover only relevant dimensions
  such as public API, commands/workflows, behavior, migration, and performance
  baseline. Phrase each row as a user-visible before/after, then name the
  implementation mechanism only as supporting context. For performance claims,
  name the baseline explicitly: CPU path, parent commit, `main`, or prior
  implementation, plus workload, metric, and important limitations.
- In Testing, list exact commands, targets, or test names that ran.
- For CI, performance, or infrastructure work, include evidence such as CI run
  observations, timing, reruns, benchmark output, or why a skipped check is
  expected.
- For model/scene, dynamics, collision/contact, simulation, rendering, mesh,
  texture, GUI, or visual-example changes, use `dart-verify-sim`: report the
  text correctness oracle and include assessed, claim-tied OSG/debug-overlay
  evidence when applicable (an image alone is not correctness proof):
  - Prefer an existing headless example path such as `--headless`,
    `--frames`, `--width`, `--height`, and `--screenshot` over manual
    screenshots.
  - Capture the before image from the base branch or by temporarily restoring
    the replaced sample/assets, then capture the after image from the final
    branch with the same camera, dimensions, frame count, and renderer.
  - Inspect the images yourself and include the commands plus any environment
    variables such as software rendering flags in the PR body.
  - Upload transient comparison images, GIFs, and videos through the GitHub
    PR/issue Markdown attachment flow so the PR body contains GitHub-hosted
    `https://github.com/user-attachments/assets/...` URLs that render inline.
    Do not commit screenshots, headless renders, GIFs, or screencast videos
    solely as PR evidence. Commit visual files only when they are durable
    documentation, fixtures, or source assets that should live in the
    repository.
  - The official GitHub attachment flow is the web PR/issue editor drag/drop or
    file picker. `gh pr edit`, `gh pr comment`, and the public REST API do not
    provide a supported generic attachment upload command; any command or action
    that edits or comments on a PR still requires explicit maintainer/user
    approval. Use a maintainer-approved upload helper only when it produces
    GitHub attachment URLs without committing files to the branch; helpers that
    mimic the web upload may require a browser session cookie and must not be
    used unless the maintainer has explicitly approved that credential handling.
  - If the current tool cannot upload PR attachments, keep the local artifact
    paths in the working note, ask a maintainer to upload them through the PR
    editor, and then update the PR body with the returned GitHub attachment
    URL after explicit maintainer/user approval. Do not fall back to committing
    transient evidence into `docs/assets/`.
  - If no headless path exists, either add a narrowly scoped capture mode when
    it fits the example or document why visual comparison is not practical.
- Mark non-applicable checklist items as "N/A" with a short reason.
- Mention related PRs, issues, backports, and follow-ups explicitly, including
  "None" when there is no related work.

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
5. Before every commit, run:
   ```bash
   pixi run lint
   ```
   Also run `pixi run build` for C++ or Python changes and focused tests for
   behavior changes.
6. Create or update a topic branch when needed:
   ```bash
   git switch --no-track -c <type>/<topic> origin/<target-branch>
   ```
7. Commit only intended files with a plain descriptive commit title.
8. Ask for explicit maintainer/user approval before pushing or opening the draft
   PR. Never push directly to `release-*`. If approved:
   ```bash
   branch=$(git branch --show-current)
   git push -u origin "HEAD:${branch}"
   gh pr create --draft --base <target-branch> --milestone "<milestone>" \
     --title "<plain title>" --body-file <filled-template-file>
   ```
   For fast feedback on a draft PR, also request Codex review after publication
   when approval covers PR comments:
   ```bash
   gh pr comment <PR_NUMBER> --body "@codex review"
   ```
   If Codex already shows an activity signal or submitted review, do not post a
   duplicate trigger.
9. After a PR is published, prefer additive follow-up commits for updates so
   reviewers can inspect each review round. Amend or force-push only after
   explicit maintainer/user approval and only when the user explicitly requests
   it or when there is a clear reason such as removing sensitive content or
   repairing broken branch history.
10. Before every push to a published PR branch, first merge the latest base
    branch into it (on every push, not just the first) so each pushed/CI-tested
    state reflects the current target base branch and conflicts surface early:
    ```bash
    git fetch origin <target-branch>
    git merge --no-ff origin/<target-branch>  # never rebase a published PR branch
    # rebuild + retest if the merge touched code
    git push                                   # after explicit approval
    ```
    The local base merge is a routine pre-push step; the push itself still
    requires explicit maintainer/user approval. Do not rebase a published PR
    branch by default because it invalidates existing CI runs and makes PR
    review/comment history harder to follow. Rebase or force-push only when the
    maintainer explicitly requests it.
11. Use `docs/onboarding/changelog.md` for the changelog decision. If
    `CHANGELOG.md` needs the PR number, keep the follow-up changelog commit
    local until explicit maintainer/user approval is given for the additional
    push or PR update.
12. Monitor CI:
    ```bash
    gh pr checks <PR_NUMBER>
    ```

## AI Review Comments

Never reply to AI-generated review comments from bot users such as
`chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
`github-actions[bot]`, or `copilot[bot]`.
When a draft PR is first published, a top-level `@codex review` is the preferred
fast path once explicit maintainer/user approval covers PR comments; it can run
while the PR remains draft. Make fixes silently. Push and ask for a new AI review
with `@codex review` only after explicit maintainer/user approval and only when
the approved follow-up push addressed Codex review comments, or when the first
trigger has a concrete timeout/blocker.

After Codex returns no actionable issues and local validation passes on the
current head (`pixi run test-all` for build coverage, `pixi run test` when C++
runtime behavior could be affected, `pixi run test-py` when Python behavior
could be affected, plus the Gazebo gate when package or downstream compatibility
could be affected), a draft PR is ready to mark ready for human review after
explicit approval even if hosted CI is still pending. Do not merge until branch
protection and required checks pass unless a maintainer explicitly approves a
policy bypass.

## Output

- Branch, target base, and milestone used
- Commit titles and files included
- Verification commands run and their results
- PR URL after approved creation, and the changelog decision
