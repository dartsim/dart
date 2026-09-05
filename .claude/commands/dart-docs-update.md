---
description: update docs or AI instruction visibility without code changes, or audit the docs tree for stale, duplicated, and orphaned content
argument-hint: "<topic> | audit [bucket]"
agent: build
---

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md
@docs/AGENTS.md
@docs/ai/principles.md
@docs/ai/verification.md

For AI instruction or workflow changes, follow the owner read order in
`docs/ai/README.md`; read the relevant tool section of
`docs/onboarding/ai-tools.md` only when compatibility is affected. Load
`docs/onboarding/changelog.md` when making the closeout changelog decision.

## Modes

- `<topic>` (default): make the requested documentation change.
- `audit [bucket]`: find stale, duplicated, unowned, or oversized docs and
  propose or apply consolidations; limit to one bucket when given.

## Workflow

1. Create a branch from the target branch: `git checkout -b docs/<topic> origin/main`
2. For `audit`, build the finding list first:
   - run `pixi run check-docs-policy` and collect every failure and advisory
     (broken links, orphans, dev-task size budgets, dashboard budgets);
   - walk the "Keeping Docs Current" table in `docs/README.md` against the
     bucket: completed tasks whose folders remain, archived plans still
     pointing at numbered files, resolved `Decision needed` blocks, superseded
     pages, rules copied into more than one doc, running logs in snapshot
     files, and pages describing a state the code has left behind;
   - for each finding record the owner doc, the proposed action (rewrite,
     merge, delete, promote, link), and the evidence; prefer deletion and
     consolidation over new files or sections. Apply the findings that stay
     within the requested scope and list the rest in the output.
3. Edit docs and AI workflow sources only:
   - Regular docs: `docs/**`, `README.md`, `AGENTS.md`, `CONTRIBUTING.md`,
     and `CHANGELOG.md` when `dart-changelog` requires a release-note entry
   - AI source files: `.claude/commands/**`, `.claude/skills/**`
4. For AI workflow changes, edit `.claude/` sources and run `pixi run sync-ai-commands`; do not hand-edit generated `.agents/skills/` or `.opencode/command/` files (`.codex/` is a maintained source)
5. Classify new or moved docs by lifecycle first, then audience, then topic,
   using the placement matrix in `docs/README.md`. For AI docs, keep
   always-loaded entrypoints compact: improve owner placement or pointers
   instead of duplicating procedures.
6. Update indexes and cross-references that point to changed docs; redirect
   every link into a deleted file.
7. Use `docs/ai/verification.md` to select the docs-only or AI docs/adapters
   gate set, then run `pixi run lint` before committing
8. Invoke the `dart-changelog` routine for the `CHANGELOG.md` decision and any
   required entry.
9. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, use `.github/PULL_REQUEST_TEMPLATE.md` and the proper
   milestone.

## Output

- Docs and AI workflow sources changed
- For `audit`: findings applied, findings deferred with their owner, and
  advisories left in place with the reason
- Sync and verification commands run
- Changelog decision
- PR readiness, noting any external mutation that was explicitly approved
