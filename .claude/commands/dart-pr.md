---
description: create a branch, commit, push, and open a DART pull request
argument-hint: "[base=<branch>] [draft]"
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
gh pr list --repo dartsim/dart --state merged --base main --limit 10 \
  --json number,title,body,mergedAt
```

Use these practices:

- Keep titles plain, scoped, and outcome-focused. Do not add agent prefixes.
- Fill the PR template in DART's default order: Summary, Motivation / Problem,
  Changes / Key Changes, optional Before / After, Testing, Breaking Changes,
  and Related Issues / PRs. Keep Summary first because reviewers need the
  skimmable outcome before the rationale. If the motivation is necessary to
  understand the outcome, make the first Summary sentence problem-oriented,
  then keep the fuller why in Motivation / Problem rather than moving it above
  Summary.
- Write Summary and Motivation for a user or downstream maintainer unfamiliar
  with the implementation: lead with what changes for them, what stays
  compatible, how they opt in or migrate, and why the evidence matters. Keep
  implementation mechanics in Changes unless they explain a user-visible
  outcome or risk.
- When the change has user-facing API, workflow, behavior, or performance
  impact, add a concise `## Before / After` section before Testing covering
  only relevant dimensions (public API, commands/workflows, behavior,
  migration, performance baseline). Phrase each row as a user-visible
  before/after, then name the mechanism as supporting context. For performance
  claims, name the baseline explicitly (CPU path, parent commit, `main`, or
  prior implementation) plus workload, metric, and limitations.
- In Testing, list exact commands, targets, or test names that ran. For CI,
  performance, or infrastructure work, include evidence such as CI observations,
  timing, reruns, benchmark output, or why a skipped check is expected.
- For rendering, model, mesh, texture, GUI, or visual-example changes, include a
  before/after visual comparison when practical:
  - Prefer an existing headless example path (`--headless`, `--frames`,
    `--width`, `--height`, `--screenshot`) over manual screenshots. Capture
    before and after with the same camera, dimensions, frame count, and
    renderer; restore the replaced sample/assets or use the base branch to
    capture the before image. Inspect the images yourself and include the
    commands and any software-rendering flags in the PR body.
  - Upload transient comparison images, GIFs, and videos through the GitHub
    PR/issue Markdown attachment flow so the body contains GitHub-hosted
    `https://github.com/user-attachments/assets/...` URLs. Do not commit
    transient visual evidence; commit visual files only when they are durable
    documentation, fixtures, or source assets.
  - The supported attachment flow is the web PR/issue editor; `gh pr edit`,
    `gh pr comment`, and the REST API do not provide a generic upload, and any
    command that edits or comments on a PR still requires explicit
    maintainer/user approval. If the current tool cannot upload attachments,
    keep the local artifact paths in the working note, ask a maintainer to
    upload them, then update the PR body with the returned URL after explicit
    maintainer/user approval. Do not fall back to committing evidence into
    `docs/assets/`.
  - If no headless path exists, add a narrowly scoped capture mode when it fits
    the example or document why visual comparison is not practical.
- Mark non-applicable checklist items as "N/A" with a short reason, and mention
  related PRs, issues, backports, and follow-ups explicitly, including "None".

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
   the base-merge and automated-review rules in `docs/onboarding/ai-tools.md`
   (no inline bot replies; `@codex review` re-triggers are throttled to one per
   approved review-fix round). Ask for explicit maintainer/user approval before
   pushing or opening the draft PR. After approval:
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
    the follow-up changelog commit local until explicit maintainer/user approval
    is given for the additional push or PR update.
11. Monitor CI: `gh pr checks <PR_NUMBER>`.

## Output

- Branch, target base, and milestone used
- Commit titles and files included
- PR URL and draft/ready state, or the prepared PR text awaiting approval
- Changelog decision
- CI status and any remaining blocker
