---
name: dart-release-packaging
description: "DART Release Packaging: create a release version bump and changelog PR"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-release-packaging.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-release-packaging

Use this skill in Codex to run the DART `dart-release-packaging` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-release-packaging <arguments>`
- Codex: `$dart-release-packaging <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Create release packaging PR after explicit maintainer/user approval: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/release-management.md
@docs/onboarding/contributing.md
@docs/onboarding/changelog.md

## Workflow

1. Confirm the new version, for example `6.16.6`.
2. Derive the release branch: `release-<major>.<minor>`.
3. Fetch and branch from the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH>
   git checkout -B release/<NEW_VERSION>-version-bump origin/<RELEASE_BRANCH>
   ```
4. Bump versions in `package.xml` and `pixi.toml`.
5. Update version requirements in examples/tutorials if needed.
6. Invoke the `dart-changelog` routine in release-audit mode, then add or
   finalize the `CHANGELOG.md` release section with the release date, milestone
   link, release summary, and audit from `docs/onboarding/changelog.md`.
7. Run `pixi run lint` and relevant packaging checks.
8. Commit as `Packaging <NEW_VERSION>`.
9. Ask for explicit maintainer/user approval before pushing, creating the PR,
   or setting milestones.
10. After approval, create the PR against the release branch with the release
    milestone for that branch, for example the specific version milestone if
    available.

## Output

- PR URL
- Version files changed
- Changelog section added
- Checks run
