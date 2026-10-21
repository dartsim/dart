# Changelog Fragments

On `main`, a pull request that needs a changelog entry adds a fragment here
instead of editing `CHANGELOG.md`, so open pull requests never conflict over
that file. [Changelog Guidance](../docs/onboarding/changelog.md) says which
changes need an entry, how to write it, and when fragments are assembled.

Name each fragment `<topic>.md` after a unique, descriptive kebab-case topic,
one fragment per entry. The first line is `Section: <Category>`, naming a
category allowed below. One blank line follows. Then comes the entry exactly
as it will appear in `CHANGELOG.md`: a `  * ` bullet with continuation lines
indented 4 spaces. Once the PR exists, a follow-up commit adds its link as the
last line:

```markdown
Section: Dynamics

  * State the outcome for release readers: what changed for them and any
    rebuild or migration step they need.
    [#1234](https://github.com/dartsim/dart/pull/1234)
```

An entry that cites several PRs or issues lists one link per line, with a
comma after every link but the last. The category must already be under the
unreleased heading or be one of these, which assembly creates in this order:
Breaking Changes, Behavior Changes, API, Performance, Collision, Constraint,
Simulation, Dynamics, Parsers, Python, GUI, Examples, Math, Build.

`pixi run check-changelog-fragments` validates the fragments, and
`pixi run check-lint` runs that check. Every file in `changelog.d/` must be a
valid fragment, hidden ones included; only editor swap files are ignored.
`release-*` branches have no fragments, so the check and assembly reject any
there (the PR's base branch counts when CI reports one).

At a checkpoint, `pixi run changelog-assemble` moves each entry to the end of
its category and deletes the fragments. Each fragment must link the PR that
introduced it, which is the PR number ending the subject of the commit that
added the file, as in `Add x (#123)`; related PRs and issues may be linked too.
Entries are ordered by that PR. Assembly reads the full Git history offline and
fails on a shallow clone or a subject without a terminal `(#N)`, such as a
rebase-merged commit; fetch the history, or write the entry in `CHANGELOG.md`.

Commit all changes in `CHANGELOG.md` and `changelog.d/` before assembly. If
deleting a fragment fails, `CHANGELOG.md` is already updated: review and commit
the changes, then re-run to delete the remaining fragments without duplicating
entries.
