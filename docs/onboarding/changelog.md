# Changelog Guidance

A PR needs a changelog entry when it adds user-visible features, fixes bugs,
changes dependencies, changes public behavior, or introduces a breaking change.

On `main`, add the entry as a fragment in `changelog.d/` instead of editing
`CHANGELOG.md`. Entries inserted into one shared release section make every
merge conflict with the other open PRs. Uniquely named fragments never
conflict.
[`changelog.d/README.md`](../../changelog.d/README.md) owns the fragment format.
Run `pixi run check-changelog-fragments` after editing a fragment;
`pixi run check-lint` runs it too.

Fragments move into `CHANGELOG.md` at checkpoints: each milestone exit, before
any release candidate or tag, before a release branch is cut from `main`, and
when the maintainer asks. One PR then runs `pixi run changelog-assemble`. It
requires each fragment to link the PR that introduced it, taken offline from the
full Git history (the terminal `(#N)` of the adding commit's subject, so it needs
a complete clone and squash-merged fragments). It appends each entry to its
category under the unreleased release heading in that PR's order and deletes the
fragments. Review the assembled section as release notes in that PR.
Commit all changes in `CHANGELOG.md` and `changelog.d/` before assembly. If a
fragment deletion fails, review and commit the partial changes, then re-run to
delete the rest without duplicating entries.

`release-*` branches have no fragments, and `check-changelog-fragments` and
assembly reject them there: edit `CHANGELOG.md` directly, including in
backports, and turn a cherry-picked fragment into a direct entry.

`CHANGELOG.md` is written for users of the released library and packages.
CI, tooling, AI-harness, docs-workflow, and other internal changes do not need
an entry unless they change a command or workflow that users of the release
run; keep each entry to what the reader must know or do (what changed for them,
any rebuild or migration step) and leave the mechanism to the PR body or a
design doc.

Describe DART behavior or API conditions that apply to any downstream; do not
name a specific downstream project in entry descriptions. Keep bug-report
issue links as references; PR descriptions may still cite downstream evidence.

When a changelog entry needs a PR number, create the PR first, then add the PR
link line to the fragment (or, on a release branch, to the entry) in a
follow-up commit after explicit approval for the additional push.
