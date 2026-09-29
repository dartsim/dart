# Release Management

DART 6.20 work targets `release-6.20` and the branch-matching DART 6.x
release milestone (currently `DART 6.20.0`).

Version note: `package.xml` on this branch carries the latest published
DART 6.19.x point release forward (so a configured build can report a
6.19.x version) and is bumped to `6.20.0` only by the release packaging
change. The `CHANGELOG.md` section "DART 6.20.0 (Unreleased)" is the
authoritative statement of what this branch is becoming.

Release-branch PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.

## DART 6 Release Closeout

Before tagging any DART 6.x.y release, record passing compatibility evidence on
the exact candidate SHA for the forced optional-dependency-off gate and
`pixi run -e gazebo test-gz`, confirming that both `test-gz-physics` and
`test-gz-sim` ran. Evidence from a different SHA is not release evidence. When
activating a new `release-6.x` branch, confirm its branch protection requires
uniquely named contexts for both gates.

`release-6.20` enforces these gates through the required
`Asserts enabled (no -DNDEBUG)` context, owned only by CI Linux and
configuring/building with OpenSceneGraph forcibly disabled, and the required
`ubuntu-latest` context, owned only by CI gz-physics and running both Gazebo
tasks. Keep each required context single-owner when editing workflows.

`main` mirrors `release-6.20`: after tagging each DART 6.20.x release,
fast-forward it with `git push origin release-6.20:main`. Open every PR against
`release-6.20`.

## Verifying Release-Branch Changes

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
