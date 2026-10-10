---
description: create a branch, commit, push, and open a DART pull request
argument-hint: "[title-or-topic]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

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

When the expected PR style is unclear, inspect recently merged PRs before
drafting the title or body:

```bash
gh pr list --repo dartsim/dart --state merged --base <target-branch> --limit 10 \
  --json number,title,body,mergedAt
```

Use these practices:

- Keep titles plain, scoped, and outcome-focused. Do not add agent prefixes.
- Describe private plans in prose or link public PRs/issues. Keep private,
  local, and machine-specific paths out of the title and body.
- Follow `docs/onboarding/contributing.md` § "PR Descriptions" for the
  Effect-first template order, comparisons, plots, collapsed raw tables, and
  Testing content. This is the PR-description owner for agents and contributors.
- For CI, performance, or infrastructure work, include evidence such as CI run
  observations, timing, reruns, benchmark output, or why a skipped check is
  expected.
- For model/scene, dynamics, collision/contact, simulation, rendering, mesh,
  texture, GUI, or visual-example changes, use `dart-verify-sim`: report the
  text correctness oracle and include assessed, claim-tied OSG/debug-overlay
  evidence when applicable (an image alone is not correctness proof):
  - Prefer an existing headless example path such as `--headless`,
    `--frames`, `--width`, `--height`, and `--shot` over manual
    screenshots.
  - Follow `docs/ai/verification.md` § "Simulation Verification Route" for
    before/after 3D highlights, matched captures, and compute-only plots.
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
3. Resolve the target from the live PR base, or use `main` for new development.
   Fetch that base and read its `docs/onboarding/release-management.md`
   § "Release target". Use `DART <Next release>` as the milestone and verify
   that exact title is open on GitHub. Publishing or updating the PR requires
   explicit maintainer/user approval.
4. New fixes target `main`; backports to a maintained `release-6.*` branch use
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
8. Before every `gh pr create` or `gh pr edit`, require both checks below to
   exit 0 on the exact proposed title and body, including retained text:
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
