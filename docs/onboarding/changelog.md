# Changelog Guidance

Update `CHANGELOG.md` when a PR adds user-visible features, fixes bugs, changes
dependencies, changes public behavior, or introduces a breaking change.

`CHANGELOG.md` is written for users of the released library and packages.
CI, tooling, AI-harness, docs-workflow, and other internal changes do not need
an entry unless they change a command or workflow that users of the release
run; keep each entry to what the reader must know or do (what changed for them,
any rebuild or migration step) and leave the mechanism to the PR body or a
design doc.

Describe DART behavior or API conditions that apply to any downstream; do not
name a specific downstream project in entry descriptions. Keep bug-report
issue links as references; PR descriptions may still cite downstream evidence.

When a changelog entry needs a PR number, create the PR first, then add the
entry in a follow-up commit after explicit approval for the additional push.
