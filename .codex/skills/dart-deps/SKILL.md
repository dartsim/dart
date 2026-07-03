---
name: dart-deps
description: "DART Deps: triage and shepherd dependency and bot pull requests"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-deps.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-deps

Use this skill in Codex to run the DART `dart-deps` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-deps <arguments>`
- Codex: `$dart-deps <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Triage and shepherd dependency and bot PRs: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Scope

Handle automated dependency and maintenance PRs: Dependabot GitHub Action
version bumps and the scheduled lockfile PRs opened by `update_lockfiles.yml`.
Default to read-only triage; merging any bot PR is an explicit approval gate.

## Workflow

1. List open bot PRs:
   ```bash
   gh pr list --repo dartsim/dart --state open \
     --search "author:app/dependabot OR author:app/github-actions" \
     --json number,title,headRefName,author
   ```
2. For each PR, review the diff (`gh pr diff <PR_NUMBER>`):
   - Action bumps: confirm the new ref is a pinned commit SHA and sanity-check
     the upstream changelog for the bumped version.
   - Lockfile PRs: confirm the regenerated lockfile provenance matches the
     workflow that produced it and that no unrelated content changed.
3. Confirm CI is green for the PR head: `gh pr checks <PR_NUMBER>`.
4. Merge a reviewed, CI-green bot PR only after explicit maintainer/user
   approval, using the current head SHA and the repository merge method.
5. Batch-report every triaged PR with its recommendation and status.

## Output

- Table of open bot PRs with kind (action bump or lockfile), CI state, and
  recommendation
- Diff-review notes per PR
- Which PRs were merged after explicit approval and which remain pending
