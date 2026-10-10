# Release Management

New features and fixes target `main`. Backport merged fixes to a `release-6.*`
branch using `dart-backport-pr`; release-specific packaging, CI, and branch
guidance may target that branch directly.

## Release Target

This table owns the planned release for this base branch. Each development or
release branch maintains exactly one row; update it when that branch advances to
its next release. Reusable skills and docs link here instead of copying values.

| Branch         | Phase         | Next release |
| -------------- | ------------- | ------------ |
| `main`         | Development   | `6.21.0`     |

Before creating or updating a PR, resolve its target from the live PR base
(`main` by default for new work), fetch that branch, and read this file from
the fetched base rather than the topic checkout:

```bash
git fetch origin <target-branch>
git show origin/<target-branch>:docs/onboarding/release-management.md
gh api --paginate 'repos/dartsim/dart/milestones?state=open' --jq '.[] | .title'
```

Confirm the row names the target branch and that the exact milestone
`DART <next-release>` exists and is open before a GitHub mutation. If either
is missing, resolve the inconsistency with the maintainer. Use the full next
release in the row, including its patch number; neither the branch name nor
the newest milestone determines it.

The version sources serve different purposes:

- **Compatibility line:** DART 6; reusable policy preserves this contract.
- **Source/package version:** `package.xml`, also exposed as Sphinx `release`;
  `pixi.toml` matches it during packaging. These may retain a previously
  published version while new changes accumulate.
- **Planned release:** this branch's row above; put new changelog entries under
  its release section. Preserve earlier sections when advancing the target.
- **Published releases and milestone state:** live GitHub
  [releases](https://github.com/dartsim/dart/releases) and
  [milestones](https://github.com/dartsim/dart/milestones).

At a rollover, update this branch's row and start its changelog section. Leave
other branches' rows, reusable skills, pointer docs, and translated landing
text alone. Update package versions only as part of release packaging.

## Compatibility Policy

DART 6 PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.

## DART 6 Release Closeout

A DART 6.x.y release on a stabilization or maintenance branch is packaged by
one "Packaging 6.x.y" PR on its `release-6.x` branch, including the first
`6.x.0` release after an early stabilization cut. It bumps `package.xml` and the
`pixi.toml` workspace version, dates the release's `CHANGELOG.md` heading, links
that heading to the closed milestone (`?closed=1`), and adds a short release
summary under it. Its
squash commit is the release candidate: once the gates below pass, tag it
`v6.x.y` (annotated, message `DART 6.x.y`) and publish the GitHub release
`DART 6.x.y`.

Before tagging any DART 6.x.y release, record passing compatibility evidence on
the exact candidate SHA for the forced optional-dependency-off gate and
`pixi run -e gazebo test-gz`, confirming that both `test-gz-physics` and
`test-gz-sim` ran. Evidence from a different SHA is not release evidence. When
activating a new `release-6.x` branch, confirm its branch protection requires
uniquely named contexts for both gates.

Development and release branches enforce these gates through the required
`Asserts enabled (no -DNDEBUG)` context, owned only by CI Linux and
configuring/building with OpenSceneGraph forcibly disabled, and the required
`ubuntu-latest` context, owned only by CI gz-physics and running both Gazebo
tasks. Keep each required context single-owner when editing workflows.

Before tagging, also record the release performance comparison on the exact
candidate SHA:
`gh workflow run perf.yml --ref main -f tier=release -f head=<candidate-sha>`.
It takes the version from the candidate's `package.xml`, compares the candidate
with the previous `v6` tag on the rows both can run (`-f base=v6.x.y` selects
another baseline), and publishes `performance/releases/v6.x.y.json`, `.md` and
`index.md` on `gh-pages`. Only `main` candidates list merged changes since that
tag that moved a gated row, broke a row or carried a rationale line;
`release-6.x` candidates carry the tag comparison only. Read it before
tagging; a FAIL does not block by itself but needs an explanation. After
publishing the GitHub release, check that the record names the tagged commit,
then attach both files as `dart-perf-v6.x.y.json` and `dart-perf-v6.x.y.md`
(commands in [CI/CD](ci-cd.md#performance-records-and-guards)); if the record names a
different candidate, dispatch again with `-f tag=v6.x.y` and no `head` first.
Hosted records take precedence over local records. Within the same producer,
the tagged commit's record replaces any stored candidate, including a newer
or diverged one.

A maintainer may cut `release-6.x` before the first minor-release tag to
stabilize that release while `main` develops the next minor version. In that
case, package and tag the first `6.x.0` release on the stabilization branch,
as for later patch releases.
Without a stabilization cut, package and tag the minor release on `main`,
then cut a maintenance branch from the tag when patch releases must diverge.
Protect each new release branch and verify that its required check contexts,
including both Read the Docs builds, are emitted by release-target PRs before
relying on the merge gate. The `Nightly` workflow remains scheduled on `main`;
its `nightly-failure` issues track `main`.

The DART 6.20 stabilization branch was cut at
`5179ee945ada735c49eab772476cd7981ce239ad`, before the Python tutorial migration,
so its first minor release retains the C++ tutorials. This records the cut;
the table above owns the next release target.

## Verifying DART 6 Changes

Verify before merging: `pixi run test-all` for the complete default CMake graph.
The branch configuration pins `BUILD_TESTING=ON`, so `ALL` builds the default
targets and invokes `tests_and_run` and `pytest`. Use `pixi run test` or
`pixi run test-py` for focused failure attribution; both use the same sanitized
CTest/pytest runners as the aggregate. Run `pixi run lint` separately because
`test-all` does not format or check lint. For any
collision/constraint/parser/default-solver/public-header change, also run
`pixi run -e gazebo test-gz`. Run the **full** `pixi run check-lint`
(clang-format with gersemi, black/isort, and codespell) — the CI "Check Lint"
step runs the whole aggregate, so checking only the sub-check you touched
misses failures.

Platform gotchas the default Linux/gcc build does not catch:

- macOS arm64 and FreeBSD build with clang `-Werror`. They flag
  `-Wdeprecated-declarations` (wrap deliberate deprecated-API use, e.g. binding
  Assimp shims, in `DART_SUPPRESS_DEPRECATED_BEGIN/END`) and
  `-Wpotentially-evaluated-expression` (`typeid(*smart_ptr)` →
  `typeid(*raw_ptr)`) that gcc ignores.
- MSVC does not zero-initialize — an unset count/index (e.g. an Assimp
  `mNumMaterials` / `mMaterialIndex`) can surface as `std::bad_alloc` on Windows
  only.
- The FreeBSD VM applies `tools/freebsd/patches/*` with `patch -p0`; reformatting
  `CMakeLists.txt` (e.g. gersemi) shifts context and breaks those patches —
  regenerate them against the current source.

When a sibling lane has open PRs, do not edit files in their diff; if a new lint
gate would trip on their pre-existing issues, skip those files temporarily with a
labeled "remove once that lane merges" note.
