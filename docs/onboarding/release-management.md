# Release Management

`main` develops DART 6.21; new features and fixes target `main` with the
`DART 6.21.0` milestone. `release-6.20` stabilizes DART 6.20 with the
`DART 6.20.0` milestone. It was cut at
`a4404d367858562a8f9719f04dabfbc20490a92d`, before the Python tutorial migration,
so the C++ tutorials remain in 6.20. Backport merged fixes from `main` using
`dart-backport-pr`; release-specific packaging, CI, and branch guidance may
target the stabilization branch directly.

Version note: `package.xml` and the Pixi workspace retain the latest published
version until the release packaging change. A configured development build can
therefore still report 6.19.x. Packaging on `release-6.20` bumps them to
`6.20.0`; the next minor release developed on `main` is `6.21.0`.
Keep the accumulated 6.20 changelog entries in their release section and place
new `main` work under DART 6.21.0.

DART 6 PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.

## DART 6 Release Closeout

A DART 6.x.y release on a stabilization or maintenance branch is packaged by
one "Packaging 6.x.y" PR on its `release-6.x` branch, including the first
`6.20.0` release on `release-6.20`. It bumps `package.xml` and the `pixi.toml`
workspace version, dates the release's `CHANGELOG.md` heading, links that
heading to the closed milestone (`?closed=1`), and adds a short release summary under it. Its
squash commit is the release candidate: once the gates below pass, tag it
`v6.x.y` (annotated, message `DART 6.x.y`) and publish the GitHub release
`DART 6.x.y`.

Before tagging any DART 6.x.y release, record passing compatibility evidence on
the exact candidate SHA for the forced optional-dependency-off gate and
`pixi run -e gazebo test-gz`, confirming that both `test-gz-physics` and
`test-gz-sim` ran. Evidence from a different SHA is not release evidence. When
activating a new `release-6.x` branch, confirm its branch protection requires
uniquely named contexts for both gates.

`main` and `release-6.20` enforce these gates through the required
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
as for later patch releases. `release-6.20` uses this early-cut policy.
Without a stabilization cut, package and tag the minor release on `main`,
then cut a maintenance branch from the tag when patch releases must diverge.
Protect each new release branch and verify that its required check contexts,
including both Read the Docs builds, are emitted by release-target PRs before
relying on the merge gate. The `Nightly` workflow remains scheduled on `main`;
its `nightly-failure` issues track `main`.

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
