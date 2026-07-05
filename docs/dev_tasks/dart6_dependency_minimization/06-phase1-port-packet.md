# Phase-1 port packet — native collision math core (internal-only)

> Executes phase 1 of
> [03-native-collision-port-scoping.md](03-native-collision-port-scoping.md).
> Baseline envelope: [05-phase0-baseline-packet.md](05-phase0-baseline-packet.md).
> Port source: DART 7 `origin/main` @ `dbe6fcccb1c2d42eb6827188cf3d523f0a732285`,
> tree `dart/collision/native/`.

## Goal

Land the DART 7 native collision **algorithm core** on `release-6.20` as
C++17, no EnTT, no new dependency, **internal-only** (no installed
headers, no public component, no behavior change — FCL stays the default
detector), with the DART 7 deterministic unit tests ported alongside.

## Grounded port facts (verified 2026-07-04)

- The only C++20 feature in the entire DART 7 `native/` tree is
  `std::span` (13 files + transitive reach through `types.hpp`, which 19
  files include). No concepts/ranges/expected/spaceship anywhere.
- EnTT coupling is confined to `collision_object.{hpp,cpp}`,
  `collision_world.{hpp,cpp}`, `comps/collision_object.hpp` — all
  **out of scope** for phase 1 (they are phase-2 re-plumbing material).
- `dart/common/parallel_for.hpp` (DART-7-only, 113 lines) is used only by
  `collision_world.cpp` — also out of scope for phase 1.
- voxblox appears only in an opt-in benchmark — never in scope.
- Style precedent: the `dart/simd` backport (#3229) kept DART 7
  conventions on 6.20 — snake_case files, C++17 nested namespace,
  explicit CMake source lists. The native port does the same:
  namespace `dart::collision::native`, export macro
  `DART_COLLISION_NATIVE_API`.

## Scope (rev. 2 — unified slice)

> Rev. 1 split this into "Slice A / Slice B" and claimed the tests were
> span-clean. The first execution attempt correctly stopped and
> falsified both: DART 7's unit *tests* use `std::numbers` (C++20;
> zero occurrences in the library tree), `test_box_box`/`test_gjk`/
> `test_sphere_sphere` include `shapes/shape.hpp` and `types.hpp`, and
> `test_gjk` includes `<span>` directly. The ported set below is the
> **verified include closure** — every `dart/` include of every listed
> file resolves inside the list.

**Step 0 — span shim.** `dart/collision/native/detail/span.hpp`: a
minimal C++17 `dart::collision::native::span<T>` (pointer+size view with
`begin/end/size/empty/operator[]/data`, constructible from container /
pointer+size). Ported code replaces `std::span<T>` with this alias and
`#include <span>` with the shim include.

**Library files (closure-verified, 20 files + shim):**
`export.hpp`, `fwd.hpp`, `aabb.{hpp,cpp}`, `contact_point.hpp`,
`types.{hpp,cpp}`, `contact_manifold.{hpp,cpp}`,
`shapes/shape.{hpp,cpp}`, `sdf/signed_distance_field.hpp` (abstract
interface only — the dense field impls stay in phase 3),
`narrow_phase/gjk.{hpp,cpp}` + `gjk_inl.hpp`, `narrow_phase/mpr.{hpp,cpp}`,
`narrow_phase/box_box.{hpp,cpp}`,
`narrow_phase/box_box/{sat,face_clip,contact_reduction}.{hpp,cpp}`,
`narrow_phase/sphere_sphere.{hpp,cpp}`.

**Tests (ported from DART 7 `tests/unit/collision/`, wired as
`UNIT_collision_native_*` in `tests/unit/collision/native/`):**
`test_aabb`, `test_types`, `test_box_box`, `test_gjk`,
`test_gjk_degenerate`, `test_sphere_sphere`.

**Allowed substitutions (exhaustive):**

1. Library + tests: `std::span` / `#include <span>` → the shim.
2. Tests only: `#include <numbers>` + `std::numbers::pi_v<double>` (and
   variants) → a file-local `constexpr double kPi =
   3.141592653589793238462643383279502884;` (or the exact constant the
   test uses). The library tree has zero `std::numbers` uses.
3. Include-path/export-macro reconciliation + DART 6 copyright block.

Anything else that fails under C++17: stop and record here.

**Documented behavioral deviation (post-review):** DART 7's
`sphere_sphere.cpp` ignores `CollisionOption::enableContact` and mutates
the `CollisionResult` on binary checks, unlike `box_box.cpp` which gates
contact creation at four sites (verified against `origin/main` @
`dbe6fcccb`). This is an upstream bug surfaced by the phase-1 PR review.
The port fixes it (both `collideSpheres` entry points return the boolean
overlap without touching the result when `enableContact` is false,
matching box-box's contract) with a regression test, and the same fix is
dual-PR'd to `main` per the dual-PR policy so the trees stay in sync for
later phase diffs.

## Layout / build integration

- New dir `dart/collision/native/` mirroring DART 7 subpaths
  (`narrow_phase/box_box/`, `detail/`).
- `dart/collision/native/CMakeLists.txt`: explicit source list (no
  GLOB), `dart_add_core_headers`/`dart_add_core_sources` so the code
  builds into the core `dart` target, **but NO `install()` block and NO
  generated include-header** — that is what keeps phase 1 internal.
  Add `add_subdirectory(native)` in `dart/collision/CMakeLists.txt`.
- Keep DART 7 file/license headers adjusted to the DART 6 copyright
  block (same BSD-style text used across `dart/`).
- No changes to any existing public header, component, `package.xml`,
  or `pixi.toml`.

## Porting rules

1. Take file content from `origin/main` verbatim
   (`git show <main-sha>:dart/collision/native/<path>`), then apply
   only: (a) span shim substitution, (b) include-path/export-macro
   reconciliation, (c) copyright-block swap. Do not "improve" algorithm
   code — diff-vs-DART-7 review depends on minimal drift.
2. Zero EnTT includes anywhere in the ported set; `rg entt
   dart/collision/native/` must return nothing.
3. Any file that fails to compile under C++17 for a reason other than
   `std::span`: stop and record it in this doc rather than patching
   around it silently.

## Verification gates (all before PR)

- `pixi run lint` + full `pixi run check-lint`.
- Default `pixi run config` + capped build
  (`pixi run cmake --build build/default/cpp/Release --target ALL
  --parallel 8`).
- New unit tests green:
  `ctest --test-dir build/default/cpp/Release -R UNIT_collision_native
  --output-on-failure`.
- Full `pixi run test` (ctest) unchanged — proves no behavior change.
- `pixi run test-py` unchanged.
- `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz` green (gz gate every
  phase, even though phase 1 is internal-only).
- Guard-row spot check (S5/S4 fcl+dart rows) hash-identical to
  `05-phase0-baseline-packet.md` — proves the default path is untouched.

## Out of scope (phase 2+)

`collision_world`/`collision_object`/`comps` (ECS re-plumbing),
`parallel_for`, broadphase implementations, remaining narrow-phase pairs
(capsule/cylinder/mesh/plane/convex-convex), CCD, distance, raycast, SDF
fields, manifold cache, `collision/dart/` facade plumbing, any detector
registration or default change.
