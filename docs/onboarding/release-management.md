# Release Management

`main` is the development branch for the next release, currently DART 6.20.
PRs target `main` and the matching DART 6.x release milestone (currently
`DART 6.20.0`). `release-6.19` stays the maintenance branch; backports use
`dart-backport-pr`.

Version note: `package.xml` on this branch carries the latest published
DART 6.19.x point release forward (so a configured build can report a
6.19.x version) and is bumped to `6.20.0` only by the release packaging
change. The `CHANGELOG.md` section "DART 6.20.0 (Unreleased)" is the
authoritative statement of what this branch is becoming.

DART 6 PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.

## DART 6 Release Closeout

A DART 6.x.y patch release is packaged by one "Packaging 6.x.y" PR on its
`release-6.x` branch. It bumps `package.xml` and the `pixi.toml` workspace
version, dates the release's `CHANGELOG.md` heading, links that heading to the
closed milestone (`?closed=1`), and adds a short release summary under it. Its
squash commit is the release candidate: once the gates below pass, tag it
`v6.x.y` (annotated, message `DART 6.x.y`) and publish the GitHub release
`DART 6.x.y`.

Before tagging any DART 6.x.y release, record passing compatibility evidence on
the exact candidate SHA for the forced optional-dependency-off gate and
`pixi run -e gazebo test-gz`, confirming that both `test-gz-physics` and
`test-gz-sim` ran. Evidence from a different SHA is not release evidence. When
activating a new `release-6.x` branch, confirm its branch protection requires
uniquely named contexts for both gates.

`main` enforces these gates through the required
`Asserts enabled (no -DNDEBUG)` context, owned only by CI Linux and
configuring/building with OpenSceneGraph forcibly disabled, and the required
`ubuntu-latest` context, owned only by CI gz-physics and running both Gazebo
tasks. Keep each required context single-owner when editing workflows.

At a release, tag `main`. Cut a `release-6.x` branch from the tag only when
patch releases must diverge from `main`. The `Nightly` workflow is scheduled
directly on `main`; its `nightly-failure` issues track `main`.

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
