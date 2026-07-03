---
description: decide, draft, finalize, or audit DART changelog entries
argument-hint: "[decide|draft|finalize|audit] [pr-number|release]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-changelog.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Maintain DART changelog entries: $ARGUMENTS

## Purpose

`dart-changelog` is the reusable changelog decision and writing routine. It is
usually invoked by other DART workflows when they reach a changelog decision,
not directly by users.

Use it to decide whether `CHANGELOG.md` needs an entry, draft an entry at the
right level of detail, add a PR link after publication, or audit a release
section for missing or over-detailed entries. Keep style, placement, evidence,
and release-note density aligned with `docs/onboarding/changelog.md`.

## Required Reading

@AGENTS.md
@docs/onboarding/changelog.md
@docs/onboarding/release-roadmap.md
@docs/onboarding/release-management.md

## Modes

Interpret `$ARGUMENTS` as one of these modes when present:

- `decide`: determine whether the current change needs a changelog entry and
  record the reason for the PR checklist/body when no entry is needed.
- `draft`: write or revise the entry before a PR number exists.
- `finalize`: add the PR link or adjust the entry after a PR exists, keeping the
  follow-up local until explicit maintainer/user approval permits a push.
- `audit`: scan a release section or PR set for missing, duplicate,
  over-detailed, misplaced, or stale entries.
- `release-audit`: alias for `audit` when the caller is finalizing a release
  section through `dart-release-packaging`.

If no mode is given, infer the smallest mode that satisfies the caller's need.

## Workflow

1. Inspect the change and target:
   ```bash
   git status --short --branch
   git diff --stat
   git diff --cached --stat
   BASE_REF="$(gh pr view --json baseRefName --jq .baseRefName 2>/dev/null || true)"
   # If the caller or arguments name a release branch before PR creation, set
   # BASE_REF to that branch before falling back to automatic inference.
   if [ -z "$BASE_REF" ]; then
     CURRENT_BRANCH="$(git branch --show-current)"
     UPSTREAM_REF="$(git rev-parse --abbrev-ref --symbolic-full-name @{upstream} 2>/dev/null || true)"
     for REF in "$CURRENT_BRANCH" "${UPSTREAM_REF#origin/}"; do
       case "$REF" in
         main|release-*) BASE_REF="$REF"; break ;;
       esac
     done
   fi
   BASE_REF="${BASE_REF:-main}"
   git fetch origin "$BASE_REF"
   git diff --stat "origin/$BASE_REF...HEAD"
   gh pr diff --name-only 2>/dev/null || true
   gh pr list --head "$(git branch --show-current)"
   ```
   Use the base comparison or PR diff even when the worktree is clean. If a PR,
   issue, release, or target branch is named, inspect that live object before
   writing and prefer its base over the `main` fallback.
2. Read `docs/onboarding/changelog.md` and the relevant `CHANGELOG.md` release
   section. Compare nearby bullets before drafting so wording, section choice,
   and level of detail match the current file.
3. Decide whether an entry is required using the guide:
   - user-visible API, behavior, packaging, CI, docs workflow, AI-infra,
     simulation correctness, release, or migration impact usually needs an
     entry;
   - typo-only, formatting-only, generated-only, and tiny internal refactors
     usually do not.
4. When writing, start with the reader-visible outcome, not the implementation
   chore. Use one concise bullet, merge closely related changes, avoid author
   credits, and avoid one-bullet-per-PR diary style.
5. Place the entry under the target branch's release section and nearest
   existing category. Do not create a new category for one PR unless the release
   shape genuinely needs it.
6. Add the best evidence link:
   - if a PR number exists, use `([#1234](https://github.com/dartsim/dart/pull/1234))`;
   - if no PR number exists yet, draft without the link and leave the follow-up
     local until explicit approval permits another push or PR update.
7. For release audits, consolidate noisy implementation ledgers, confirm
   breaking/removal/deprecation bullets name a migration or support lane, and
   preserve human-readable release notes over exhaustive history.
8. Validate with the gate appropriate to the caller. For changelog-only edits,
   run the docs-only checks from `docs/ai/verification.md`; before any commit,
   run `pixi run lint`.

## Caller Contract

Other workflows should call this routine whenever they touch behavior or docs
that may need release notes. The caller keeps ownership of the overall task,
validation, PR body, and approval boundary; `dart-changelog` owns the changelog
decision, wording, placement, and evidence-link hygiene.

## Output

Report:

- the changelog decision (entry needed or not) with the rule that decided it;
- the drafted or finalized entry text and its `CHANGELOG.md` placement;
- gates run (`pixi run lint`, docs-only checks) and their results;
- any follow-up left local pending explicit maintainer/user approval.
