# C++23 Modernization Plan

Status: **in progress** (DART 7). Branch: `dart7-cpp23-modernization`.

DART is already idiomatic C++20 (pervasive `std::span`/ranges/concepts,
`using enum`, `std::source_location`, `[[nodiscard]]`, fmt logging) and used
**zero** C++23 before this effort. C++23 here is about _deleting hand-rolled
standard-library reimplementations and repetitive boilerplate_ — making the code
cleaner, simpler, less error-prone, and in a few spots faster — not about new
capability.

This document is the source of truth for the phased rollout and the portability
gate. The machine-readable counterpart is
[`dart/common/feature_support.hpp`](../../dart/common/feature_support.hpp).

## Compiler floor

DART's pinned toolchains (via pixi/conda) are the real constraint — **not** the
worst-case upstream compilers:

| Platform | Compiler                          | Standard library |
| -------- | --------------------------------- | ---------------- |
| Linux    | GCC 15.2 (and conda clang)        | libstdc++ 15     |
| macOS    | conda clang (no AppleClang in CI) | libc++ 22        |
| Windows  | MSVC 14.51 / VS2026               | MSVC STL         |

Consequences that shape every decision below:

- There is **no AppleClang** in the matrix — macOS CI runs entirely through
  `pixi run` on the conda toolchain. The usual "AppleClang lags" deferral does
  **not** apply to DART; core-language C++23 and `move_only_function` clear on
  every platform.
- The real gate is **library** completeness: libc++ 22 still lacks
  `std::generator`, `views::chunk`/`slide`/`cartesian_product`, `<stacktrace>`,
  `<spanstream>`; and libstdc++ ships `<mdspan>` only from GCC 16 (our Linux
  floor is GCC 15).
- MSVC has no stable `/std:c++23`; CMake maps `cxx_std_23` to `/std:c++latest`,
  so Windows rides _preview_ C++23. Keep Windows on the adopt-now subset.

## Portability gate

| Bucket                 | Features                                                                                                                                                                                                                                          | Rule                                                                                       |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------ |
| **Adopt now**          | `std::expected`, `optional` monadic ops, `std::to_underlying`, `std::unreachable` (via `DART_UNREACHABLE`), deducing-this, multidim `operator[]`, static `operator()`, `[[assume]]`/relaxed `constexpr`, ranges `zip`/`enumerate`/`to`/`contains` | Use unconditionally once compiling as C++23.                                               |
| **Adopt with guard**   | `std::print` (use `fmt::print`), `flat_map`/`flat_set`, `move_only_function`                                                                                                                                                                      | Use only behind the matching `DART_HAS_*` macro with a fallback.                           |
| **Defer (do not use)** | `std::generator`, `views::chunk`/`slide`/`cartesian_product`, `<stacktrace>`, `<spanstream>`, `std::mdspan` (vendor Kokkos if needed), `import std`                                                                                               | Blocked by libc++/GCC 15 gaps. Treat as a hard lint. Never use `mdspan` in CUDA `.cu` TUs. |

## Phased rollout

Each phase ends green (build + `test-unit`) and is committed as a checkpoint.

- **Phase 0 — Enablement.** `DART_UNREACHABLE` macro (release `std::unreachable`,
  debug fatal+abort); `dart/common/feature_support.hpp` probe header; a
  compile-time `static_assert` smoke test (`tests/unit/common/test_feature_support.cpp`)
  that fails loudly if a runner silently lacks an adopt-now feature. No standard
  bump yet.
- **Phase 1 — Zero-risk mechanical.** `std::to_underlying` for enum→underlying
  casts; `DART_UNREACHABLE` on _audited_ exhaustive switches (parser/fallback
  sites stay error-returns/asserts); static `operator()` on stateless functors.
- **Phase 2 — Flip the switch.** Bump `cxx_std_20 → cxx_std_23` at
  `cmake/dart_defs.cmake:475` and `:2312`, plus `dartsim/{ui,engine}/CMakeLists.txt`.
  Verify all CI platforms + CUDA. Document `/std:c++latest` on Windows.
- **Phase 3 — Core-language dedup.** Deducing-this to collapse CRTP `derived()`
  and const/non-const accessor pairs — start in leaf/`detail/` headers
  (`math/lie_group`, `simulation/ecs`) before public-ABI headers; multidim
  `operator[](i,j)` on matrix types with call sites updated in lockstep. Verify
  dartpy at each public-header step.
- **Phase 4 — Vocabulary types.** Migrate `dart::common::Result` → `std::expected`
  behind a thin `using` alias, then convert call sites; adopt `optional` monadic
  ops in parsers; convert `bool`+out-param solver/IK/IO signatures via **new
  overloads** to protect ABI/dartpy/gz-physics.
- **Phase 5 — Guarded / optional.** `std::print` via fmt at debug/HUD/benchmark
  sites; safe-subset ranges (`zip`/`enumerate`/`contains`/`to`) for parallel-array
  loops; `flat_map`/`flat_set` and `move_only_function` only behind feature-test
  guards with std fallbacks.
- **Phase 6 — Deferred.** Optionally vendor Kokkos `mdspan` to unblock the
  SoA/SDF/LCP buffer rewrites; keep `generator`/`chunk`/`stacktrace` out until
  libc++ ships them.

## Execution status

Landed on `dart7-cpp23-modernization` (each commit green: build + 162/162 unit
tests, validated against the default and CUDA pixi environments):

| Phase              | State                  | What landed                                                                                                                                                                                                                                                         | Follow-on                                                                                                                  |
| ------------------ | ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| 0 Enablement       | **Done**               | `feature_support.hpp`, `DART_UNREACHABLE`, smoke test                                                                                                                                                                                                               | —                                                                                                                          |
| 1 Mechanical       | **Done**               | `std::to_underlying` (Joint, SDF/URDF parsers, GizmoFlags); `DART_UNREACHABLE` at the one provably-unreachable site (`Skeleton::allocateBodyNodeMemory`). An audit confirmed the other ~144 `DART_ASSERT(false)` are reachable input validation and correctly stay. | static `operator()` on stateless functors                                                                                  |
| 2 C++23 bump       | **Done**               | All 17 targets → `cxx_std_23`; redundant `target_compile_features` consolidated to sources of truth                                                                                                                                                                 | —                                                                                                                          |
| 3 Deducing-this    | **Deferred**           | reverted — deducing-this needs GCC 14+, but CI's default pixi env uses the system compiler (GCC 13 on ubuntu-24.04, below the documented GCC 15 floor)                                                                                                              | re-adopt once the default env pins GCC 14+; then extend to `lie_group` CRTP bases and P2128 multidim `operator[]`          |
| 4 Vocabulary types | **Core done**          | `Result` reimplemented on `std::expected` (API preserved)                                                                                                                                                                                                           | `optional` monadic ops in parsers; `bool`+out-param solver/IK/IO → `std::expected` via new overloads                       |
| 5 Guarded/ranges   | **Deferred (ranges)**  | `std::ranges::contains` reverted — `<algorithm>` `ranges::contains` needs GCC 15 (libstdc++ 15), above the CI default-env GCC 13                                                                                                                                    | re-adopt once the default env pins GCC 15; `std::print` via fmt; `flat_map`/`move_only_function` behind `DART_HAS_*` gates |
| 6 mdspan           | **Deferred (by gate)** | —                                                                                                                                                                                                                                                                   | blocked by libstdc++ `<mdspan>` only in GCC 16 (Linux floor is GCC 15); revisit or vendor Kokkos                           |

The **Core done** phase (4) ships a validated conversion; the foundation
(phases 0–2) plus the `DART_HAS_*` gates and `DART_UNREACHABLE` let the remaining
sites be swept incrementally without re-litigating the portability or
API-compatibility design. The deeper sweeps are deliberately staged to keep each
PR reviewable and to protect the public ABI / dartpy surface (see Phase 4's
new-overload rule).

**Compiler-floor caveat (important).** The documented floor is GCC 15, but DART's
**default pixi environment uses the system compiler** (`/usr/bin/c++`), which on
the CI runners (ubuntu-24.04) is **GCC 13**. So the effective floor for code that
must compile on every CI job is GCC 13, which lacks deducing-this (GCC 14) and
`std::ranges::contains` (GCC 15) — hence both are reverted/deferred here. To
re-adopt those features, first pin a conda GCC ≥ 15 in the default pixi
environment so CI matches the documented floor.

## What NOT to do

- No `std::generator` or `views::chunk`/`slide`/`cartesian_product` — they break
  the libc++ build. Keep the hand-rolled `SimdChunksView` and vector-returning
  tree traversals.
- No repo-wide `mdspan` or `import std`.
- `DART_UNREACHABLE` is not find-and-replace: many `DART_ASSERT(false)` sites are
  reachable on malformed input or return a safe fallback. Audit each.
- ABI/dartpy churn is the dominant `std::expected` cost: prefer new overloads,
  and reserve breaking enum promotions for the DART 8 ABI window.
