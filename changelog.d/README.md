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
`pixi run check-lint` runs that check. At a checkpoint,
`pixi run changelog-assemble` moves each entry to the end of its category,
in PR order, and deletes the fragments.
Commit all changes in `changelog.d/` before assembly; re-running after a failed
fragment deletion is safe.
