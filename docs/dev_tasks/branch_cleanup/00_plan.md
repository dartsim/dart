# Branch Cleanup Plan (00)

## Status

- Draft: initial plan defined; execution in progress.

## Objective

Clean up stale branches by comparing them to origin/main, deleting branches that
are no longer needed, and planning or rebasing branches that still carry useful
changes.

## Guardrails

- Use origin/main as the target branch for DART 7.
- Ask before deleting if ownership is unclear.
- Keep changes minimal; avoid drive-by refactors.
- Run git fetch --all --prune before triage work.
- Use rg for repo search.

## Phase 0 - Setup and scope

- Read AGENTS.md, docs/onboarding/ci-cd.md, docs/onboarding/build-system.md, and
  CONTRIBUTING.md.
- Confirm origin/main is the target base for DART 7.
- Fetch and prune remotes; capture the remote branch list.
- Identify branches likely based on origin/release-7.0 or other release bases
  for initial comparison.

## Phase 1 - Branch triage

For each branch:

- Confirm it exists on origin.
- Determine the original base branch; compare to that base first if it is not
  origin/main.
- Compute ahead/behind vs origin/main.
- If ahead is 0: delete the remote branch after confirmation.
- If ahead > 0: inspect commits, diff, and cherry results; decide delete, rebase
  + PR, or research + plan.
- If rebase is needed: create a new cleanup branch off origin/main.

## Phase 2 - Remove origin/release-7.0

- Ensure no active branches still depend on origin/release-7.0.
- Rebase or delete dependent branches.
- Confirm no docs or automation refer to origin/release-7.0.
- Delete origin/release-7.0 after confirmation.

## Phase 3 - Closeout

- Summarize decisions, deletions, PRs, and open questions.
- Remove this task folder when complete and add a brief onboarding note if
  needed.
