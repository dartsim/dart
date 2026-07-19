# Release Management

DART 6.20 work targets `release-6.20` and the branch-matching DART 6.x
milestone.

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

## Backporting `main` (DART 7) → `release-6.20` (DART 6)

Cherry-pick the source commit onto a fresh `backport/<pr>-to-release-6.20`
branch and adapt it to the DART-6 layout:

- PascalCase headers (`dart/dynamics/Joint.hpp`), not snake_case; `dart/utils/…`,
  not `dart/io/…`. pybind11, not nanobind (`python/dartpy/…`).
- `SmartPointer.hpp` + `DART_COMMON_DECLARE_SHARED_WEAK(...)` instead of
  `Fwd.hpp`; no `DART_API` / `<dart/Export.hpp>` on 6.20 constraint/shape classes.
- `HAVE_BULLET` / `HAVE_ODE`, not `DART_HAVE_*`.
- Most test dirs auto-register via `dart_build_tests(... GLOB_SOURCES)` (drop a
  `test_*.cpp` in, no CMake edit), but some — e.g. `tests/integration/` and
  `tests/unit/collision/` — use an explicit `SOURCES` list, so add the new file
  there too. Drop genuinely DART-7-only infra (the ECS `ClassicRigidSolver`,
  `MeshLoader`, the `tests/unit/gui` tree); confirm against the 6.20 tree first,
  since much is already backported (e.g. `CouplerConstraint`).
- For AI workflow or generated-adapter backports, compare
  `docs/ai/capabilities.json`, `docs/ai/workflows.md`, `.claude/commands/`,
  `.claude/skills/`, `.agents/skills/`, and `.opencode/command/` before
  cherry-picking. If the requested outcome names a workflow that is absent on
  this branch, add a release-tailored capability only when that is the explicit
  request; otherwise adapt the guidance to the existing DART 6.20 workflow
  surface and do not import main-only workflows just to make a patch apply.
  Regenerate adapters with `pixi run sync-ai-commands`.

Verify before merging: `pixi run test-all` (full C++ + Python) and, for any
collision/constraint/parser/default-solver/public-header change,
`pixi run -e gazebo test-gz`. Run the **full** `pixi run check-lint` (clang-format
with gersemi, black/isort, and codespell) — the CI "Check Lint" step runs the whole
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
