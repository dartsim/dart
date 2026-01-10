# Release Packaging Branch Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
Release <MAJOR.MINOR>.*

Read AGENTS.md and follow repo conventions. I want a release packaging branch/PR for <NEW_VERSION>. Auto-fill everything else:

- prev_version: read from package.xml (<PREV_VERSION>)
- base_branch_name: release-<major.minor> derived from <NEW_VERSION> (<BASE_BRANCH_NAME>)
- base_branch: origin/<BASE_BRANCH_NAME>
- remote: origin
- release_date: today (local date) (<RELEASE_DATE>)
- milestone: find the GitHub milestone titled "DART <NEW_VERSION>" (use `gh api repos/{owner}/{repo}/milestones` if available) (<MILESTONE>)

Then:
1) Sync with origin and ensure base_branch is up to date.
2) Create branch release/<NEW_VERSION>-version-bump (or similar).
3) Bump version metadata in package.xml and pixi.toml to <NEW_VERSION>.
4) Update all find_package(DART <PREV_VERSION>) references in examples/ and tutorials/ to <NEW_VERSION>.
5) Add a new CHANGELOG.md section for <NEW_VERSION> at the top with <RELEASE_DATE> + <MILESTONE> link; summarize commits since tag v<PREV_VERSION> on the base branch, keeping the <PREV_VERSION> entry intact.
6) Commit as "Packaging <NEW_VERSION>".
7) Push to origin with `git push -u origin HEAD` and open a PR to <BASE_BRANCH_NAME> using `gh pr create` with a body modeled after the latest release packaging PR (e.g., #2296), including checklist; set the milestone if possible.
8) Skip tests unless necessary.

If any derived info is missing/ambiguous (milestone, base branch, gh auth), ask before proceeding. Report the PR URL at the end.
```
