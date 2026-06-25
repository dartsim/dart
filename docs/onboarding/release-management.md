# Release Management

DART 6.20 work targets `release-6.20` and the branch-matching DART 6.x
milestone.

Release-branch PRs should:

- preserve DART 6 compatibility unless explicitly approved otherwise;
- document package and dependency changes clearly;
- run Gazebo/gz-physics gates when downstream behavior can be affected;
- keep changelog and version metadata changes separate from unrelated cleanup
  when possible.

## Backporting `main` (DART 7) → `release-6.20` (DART 6)

Cherry-pick the source commit onto a fresh `backport/<pr>-to-release-6.20`
branch and adapt it to the DART-6 layout:

- PascalCase headers (`dart/dynamics/Joint.hpp`), not snake_case; `dart/utils/…`,
  not `dart/io/…`. pybind11, not nanobind (`python/dartpy/…`).
- `SmartPointer.hpp` + `DART_COMMON_DECLARE_SHARED_WEAK(...)` instead of
  `Fwd.hpp`; no `DART_API` / `<dart/Export.hpp>` on 6.20 constraint/shape classes.
- `HAVE_BULLET` / `HAVE_ODE`, not `DART_HAVE_*`.
- Tests auto-register through `dart_build_tests(... GLOB_SOURCES)` — drop a
  `test_*.cpp` into the dir; no per-test CMake edit. Drop DART-7-only infra
  (coupler/ECS solvers, `MeshLoader`, the `tests/unit/gui` tree).

Verify before merging: `pixi run test-all` (full C++ + Python) and, for any
collision/constraint/parser/default-solver/public-header change,
`pixi run -e gazebo test-gz`. Run the **full** `pixi run check-lint` (clang-format
+ gersemi + black/isort + codespell) — the CI "Check Lint" step runs the whole
aggregate, so checking only the sub-check you touched misses failures.

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
