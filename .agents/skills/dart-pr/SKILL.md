---
name: dart-pr
description: "DART PR: create a branch, commit, push, and open a DART pull request"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-pr

Use this skill to run the DART `dart-pr` workflow. The editable workflow
source lives in `.claude/commands/`; this generated skill is its entrypoint for
Codex and Muse.

## Invocation

- Claude Code: `/dart-pr <arguments>`
- Codex: `$dart-pr <arguments>`
- Muse: `/dart-pr <arguments>`

Treat the text after the skill name, or the user's request when the skill is
loaded on demand, as `$ARGUMENTS`. When the workflow references `$1`, `$2`,
etc., map those to the positional values supplied by the user.

## Command Body

Prepare or open a DART pull request after explicit maintainer/user approval:
$ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md
@docs/onboarding/ai-tools.md
@docs/onboarding/changelog.md
@.github/PULL_REQUEST_TEMPLATE.md

## Recent PR Patterns

When project context is unclear, inspect recently merged PRs for examples;
the current PR-description owner below takes precedence over older bodies:

```bash
gh pr list --repo dartsim/dart --state merged --base <target-branch> --limit 10 \
  --json number,title,body,mergedAt
```

Use these practices:

- Keep titles plain, scoped, and outcome-focused. Do not add agent prefixes.
- Describe private plans in prose or link public PRs/issues. Keep private,
  local, and machine-specific paths out of the title and body.
- Follow `docs/onboarding/contributing.md` § "PR Descriptions" for the
  Summary and Testing default, optional material context, evidence formats,
  and collapsed detail. This is the PR-description owner for agents and contributors.
  Write each paragraph and list item of the body on one line: GitHub renders
  newlines as line breaks, so never wrap PR text at a fixed width.
- For CI, performance, or infrastructure work, include evidence that supports
  the outcome and relevant verification gaps.
- For model/scene, dynamics, collision/contact, simulation, rendering, mesh,
  texture, GUI, or visual-example changes, use `dart-verify-sim`: report the
  text correctness oracle and include assessed, claim-tied Filament/debug-overlay
  evidence when applicable (an image alone is not correctness proof):
  - Prefer an existing headless example path such as `--headless`,
    `--frames`, `--width`, `--height`, and `--shot` over manual
    screenshots.
  - Follow `docs/ai/verification.md` § "Simulation Verification Route" for
    before/after 3D highlights, matched captures, and compute-only plots.
  - Inspect the images yourself and retain reproduction commands and
    environment details in collapsed evidence details. Keep limiting caveats
    and failed or uncertain results prominent.
  - Use `pixi run evidence-publish` to render the section without network access.
    Follow `docs/ai/verification.md` § "Visual Verification (Headless Capture)"
    for publication after explicit maintainer/user approval. Attachments use
    the same store as drag-and-drop, never release assets or tags;
    `--body-file` replaces the whole PR body. Keep transient PR evidence out of
    Git; commit visual files only as durable documentation, fixtures, or source
    assets.
  - If no headless path exists, either add a narrowly scoped capture mode when
    it fits the example or document why visual comparison is not practical.
- Link relevant PRs, issues, backports, and follow-ups when useful to the review.

## Workflow

1. Inspect scope:
   ```bash
   git status --short --branch
   git diff --stat
   git diff --check
   ```
2. Exclude unrelated dirty files unless the user explicitly includes them.
3. Resolve the target from the live PR base, or use `main` for new development.
   Fetch that base and read its `docs/onboarding/release-management.md`
   § "Release target". Use `DART <Next release>` as the milestone and verify
   that exact title is open on GitHub. For an explicit rollover, follow the
   owner's proposed-target rule. Publishing or updating the PR requires
   explicit maintainer/user approval.
4. New fixes target `main`; backports to a maintained `release-*` branch use
   `dart-backport-pr`. Release-specific packaging, CI, and branch guidance may
   target the resolved release branch directly.
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
8. Every `gh pr create` or `gh pr edit` requires explicit maintainer/user
   approval. Require both checks below to exit 0 on the exact proposed title
   and body, including retained text.
   Complete the applicable actions in `docs/onboarding/contributing.md`
   § "PR Author Checklist" before publication:
   ```bash
   printf '%s\n' "$pr_title" | pixi run python scripts/check_local_paths.py --stdin
   pixi run python scripts/check_local_paths.py --text-file "$pr_body_file"
   ```
   Ask for explicit maintainer/user approval before pushing or opening the
   draft PR. Never push directly to `main` or `release-*`. If approved:
   ```bash
   branch=$(git branch --show-current)
   git push -u origin "HEAD:${branch}"
   gh pr create --draft --base <target-branch> --milestone "<milestone>" \
     --title "$pr_title" --body-file "$pr_body_file"
   ```
   Request Codex review after publication when approval covers PR comments:
   ```bash
   gh pr comment <PR_NUMBER> --body "@codex review"
   ```
   Skip the trigger if Codex already shows activity or a submitted review.
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
11. Use `docs/onboarding/changelog.md` for the changelog decision. Keep any
    follow-up entry needing a PR number local until explicit maintainer/user
    approval covers the additional push or PR update.
12. Monitor CI:
    ```bash
    gh pr checks <PR_NUMBER>
    ```

## AI Review Comments

Never reply to AI-generated review comments from bot users such as
`chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
`github-actions[bot]`, or `copilot[bot]`.
Make fixes silently. Re-request Codex review, mark the draft ready, and merge
only as `docs/onboarding/ai-tools.md` § "PR Lifecycle" describes, each after
explicit maintainer/user approval.

## Output

- Branch, target base, milestone, commit titles, and files included
- Verification results, PR URL after approved creation, and changelog decision
