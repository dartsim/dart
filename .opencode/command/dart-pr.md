---
description: create a branch, commit, push, and open a DART pull request
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
@docs/onboarding/ai-tools.md
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
- Fill the PR template with concrete Summary, Motivation, Changes, Testing,
  Breaking Changes, and Related Issues details.
- In Testing, list exact commands, targets, or test names that ran.
- For CI, performance, or infrastructure work, include evidence such as CI run
  observations, timing, reruns, benchmark output, or why a skipped check is
  expected.
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

   | Target         | Milestone     |
   | -------------- | ------------- |
   | `main`         | `DART 7.0`    |
   | `release-6.16` | `DART 6.16.x` |

4. For bug fixes, use the dual-PR flow: fix `release-6.16` first, then
   cherry-pick or reapply to `main`.
5. Before every commit, run:
   ```bash
   pixi run lint
   ```
   Also run `pixi run build` for C++ or Python changes and focused tests for
   behavior changes.
6. Create or update a topic branch when needed:
   ```bash
   git checkout -b <type>/<topic> origin/<target-branch>
   ```
7. Commit only intended files with a plain descriptive commit title.
8. Ask for explicit maintainer/user approval before pushing or opening the draft
   PR. If approved:
   ```bash
   git push -u origin HEAD
   gh pr create --draft --base <target-branch> --milestone "<milestone>" \
     --title "<plain title>" --body-file <filled-template-file>
   ```
9. If `CHANGELOG.md` needs the PR number, keep the follow-up changelog commit
   local until explicit maintainer/user approval is given for the additional
   push or PR update.
10. Monitor CI:
    ```bash
    gh pr checks <PR_NUMBER>
    ```

## AI Review Comments

Never reply to AI-generated review comments from bot users such as
`chatgpt-codex-connector[bot]`, `github-actions[bot]`, or `copilot[bot]`.
Make fixes silently. Push and ask for a new AI review with `@codex review` only
after explicit maintainer/user approval.
