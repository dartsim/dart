# Dependency inventory

Facts as of `main` `17ce7eefa` on 2026-10-18. Ubuntu versions come from
`apt-cache policy` in `ubuntu:22.04` and `ubuntu:24.04` containers (with
updates) and an Ubuntu 26.04 host. Footprint numbers come from the research
workflow (readelf closures of a shared Release build with GUI, dartpy and
assets on, and the published 6.20.1 and nightly wheels); they stay
"reported" until a repository script reproduces them.

Rows for urdfdom, sdformat, libcurl, OpenSSL and Dear ImGui follow the
maintainer's later decisions (DD-5, DD-11 to DD-14, DD-28, DD-31) and no
longer match the 2026-10-18 research; closure sizes and environment savings
the old plan predicted for them are superseded and unmeasured.

## Categories

- **A** essential, keep
- **B** keep, raise the floor, delete compat code
- **C** make optional or move out of the default
- **D** replace with the C++ standard library
- **E** replace with in-house code
- **F** vendor or fetch as a fallback
- **G** move to Python
- **H** remove from environments

## Core (`dart`)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Why it stays or goes |
| --- | --- | --- | --- | --- |
| Eigen | A | 3.4.0 → 3.4.0, tested range `[3.4.0, 6)` | 3.4.0 / 3.4.0 / 3.4.0 | The API vocabulary; gz-physics needs it independently. conda-forge already builds DART 6.20 against Eigen 5.0.1, but DART 7 CI has never built against Eigen 5 (DEP-5). |
| FCL | A | 0.7.0 → 0.7.0 | 0.7.0 everywhere | PLAN-620 and D13 keep it as a core dependency and the C++ default detector. No release since 0.7.0 (2021). Ubuntu's build has assertions on (S3 `test_Distance`). |
| libccd | A | none (2.1) | 2.1 everywhere | Reached through FCL; one direct include guards double precision. Upstream dormant since 2018. |
| octomap (optional) | A | 1.9.7 → 1.9.7; pixi and wheels ≥ 1.10.1 | 1.9.7 / 1.9.7 / 1.10.0 | Mandatory in practice, because FCL's config requires it. Versions ≤ 1.10.0 emit a `<ciso646>` warning under C++20 with libstdc++ ≥ 15. |
| assimp | B | 5.2.2 → 5.3.1, 6.x accepted | 5.2.2 / 5.3.1 / 6.0.4 | `MeshShape.hpp` exposes `aiScene`, and gz-physics' `CustomMeshShape` builds one. Largest library in the wheel (reported 4.8 MB compressed). Optional only on the 8.0 track. |
| fmt | E | 8.1.1 → removed | 8.1.1 / 9.1.0 / 10.1.1 | Reaches installed headers only through the spdlog logging branch; five direct calls in `XmlHelpers.cpp`. Removable under C++17 (DEP-15). |
| spdlog (optional) | E + H | 1.9.2 → removed | 1.9.2 / 1.12.0 / 1.15.3 | Linked PUBLIC header-only into every TU that includes `Logging.hpp` (reported 98.7K vs 37.1K preprocessed lines). gz-physics and gz-sim use none of it. Replaced by a DART log sink (DEP-14). |
| Boost | H | pixi, wheel image, FreeBSD CI only | 1.74 / 1.83 / — | No DART source includes it; one exported define remains (DEP-1). |

## Collision backends (compiled into core; legacy components forward)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Why it stays or goes |
| --- | --- | --- | --- | --- |
| Bullet | B | 3.06 → 3.05 detection floor (3.06/3.07 headers report 3.05, #3829) | 3.06 / 3.24 / 3.24 | gz-physics requires `collision-bullet` and calls `btCollisionWorld::rayTest` through DART. No code needs a newer Bullet. Upstream unmaintained since 3.25 (2022). |
| ODE | A | 0.16.2 → 0.16.2 | 0.16.2 / 0.16.2 / 0.16.6 | gz-physics' default detector subclasses `OdeCollisionDetector`. The fetch fallback likely fails under CMake 4 (DEP-4). |

## IO (`dart-io`; legacy `utils` and `utils-urdf` forward here)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Why it stays or goes |
| --- | --- | --- | --- | --- |
| tinyxml2 | B | 9.0.0 → 10.0.0 | 9.0.0 / 10.0.0 / 11.0.0 | Public through `XmlHelpers.hpp`; sdformat needs the same system library, so vendoring removes nothing. |
| urdfdom | B | 3.0.0 → 4.0.0 on source builds; Linux wheels 6.0.1 (DD-31) | 3.0.1 / 4.0.0 / 5.1.0 | Kept as the URDF parser (DD-5); the in-house reader is dropped. DART calls only `urdf::parseURDF` and already re-parses every URDF with tinyxml2. URDF acceptance differs by version (4.0 accepts URDF 1.0 only, 6.x up to 1.2); below 5.1 load only trusted URDF. `DartLoader` uses urdf types only in private static helpers (DEP-17). |
| urdfdom_headers | B | transitive (3.0.x in wheels) | 1.0.5 / 1.0.5 / 2.1.0 | Kept with urdfdom. The urdfdom-typed headers `IncludeUrdf.hpp` and `urdf_world_parser.hpp` leave the installed set (DD-28). |
| console_bridge | B | transitive of urdfdom | 1.0.1 everywhere | DART never includes it; it stays because urdfdom needs it. |
| sdformat (+ gz-math, gz-utils) | C | 14.8 (majors 14 to 16) → 14.8, accept 17 once a lane builds it | archive 12.3 / none / none; OSRF ships 14.9, 15.4, 16.1 for noble | Added by #3789 (unreleased). Kept (DD-5) as an optional part of `dart-io`, not a separate library (DD-11). Installed first: a standalone build fetches it only when absent (an explicit exception to DD-10's fetch-first rule, once the DEP-25 fetch path supports it), and pixi, packagers, CI and wheels use the environment's with the strict switch. gz-physics never calls `SdfParser`, yet it requests `utils`, so every gz configure needs DART's sdformat major, and its Harmonic and Jetty plugins load DART's sdformat at runtime. |

## GUI (`dart-gui`; dartpy links it)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Why it stays or goes |
| --- | --- | --- | --- | --- |
| Filament | A | ==1.77.3 → ==1.77.3, enforced in CMake | unusable 1.9.x / none / unusable 1.9.x | The renderer (#3696). The pin is not enforced by CMake today. Source builds need Clang ≥ 20 (noble-updates has clang-20). |
| GLFW | F | 3.4 → 3.4 plus a fetch fallback | 3.3.6 / 3.3.10 / 3.4 | Only 3.4 dlopens libX11, which the headless-import gate relies on; the one 3.4-only call is `glfwInitHint(GLFW_PLATFORM, ...)`. |
| libpng | B | none (needs ≥ 1.6.0) → 1.6.43 | 1.6.37 / 1.6.43 / 1.6.57 | Texture decode and screenshots; add decode caps (DEP-20). |
| libjpeg-turbo | A | none | 2.1.2 / 2.1.5 / 2.1.5 | Texture decode; add decode caps. |
| Dear ImGui | F | 1.92.8 docking, fetched as a hash-pinned tag archive without a patch (DD-14, DEP-30) | 1.86 / 1.90.1 / 1.92.2b | No Ubuntu LTS has the docking release, so builds whose environment lacks it fetch the archive; a compatible supplied system package and its switch remain possible. The in-tree bundled copy and the compatibility patch go. |

## Python (dartpy)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Notes |
| --- | --- | --- | --- | --- |
| CPython | B | 3.10 → 3.12 | 3.10 / 3.12 / 3.14 | D20 already chose 3.12+ and cp312-abi3. |
| NumPy (runtime) | B | 1.21.5 → 1.26.4 | 1.21.5 / 1.26.4 / 2.3.5 | Every Eigen value converts through nanobind ndarray. |
| nanobind | F | 3.1 → 3.1.x | — / 1.9.2 / 2.11.0 | DART includes nanobind's private internals and asserts internals ABI 22. |
| setuptools, wheel | H | `setup.py` → scikit-build-core (M5a step 2) | — | |

## Assets (`utils-assets`, compiled into `dart-io`; default OFF; unreleased)

| Dependency | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Notes |
| --- | --- | --- | --- | --- |
| libcurl | B | 7.85.0 → 8.5.0 with DEP-19; kept (DD-12) | 7.81.0 / 8.5.0 / 8.18.0 | One HTTPS download path for `ModelResourceRetriever`, which stays in C++ with its dartpy binding. Exported only for static builds (DEP-2); wheels ship it (DD-13). In assets-ON builds curl's closure is reported at 40.5 MB and 13 libraries, mostly ICU through libpsl. |
| OpenSSL | B | 1.1.1 → 3.0.13 with DEP-19; kept (DD-12) | 3.0.2 / 3.0.13 / 3.5.5 | One SHA-256 digest (EVP) and `RAND_bytes` only, which is why the retriever keeps it. Wheels ship OpenSSL 3 (DD-13). |

## Build and developer tooling

| Tool | Cat | Floor today → DART 7 | 22.04 / 24.04 / 26.04 | Notes |
| --- | --- | --- | --- | --- |
| CMake | B | 3.22.1 → 3.28 | 3.22.1 / 3.28.3 / 4.2.3 | The installed `DARTConfig.cmake` consumer fallback stays at 3.22.1. |
| GCC | B | 11.2 → 13.2 | 11.2 / 13.2 / 15.2 | |
| Clang | B | 13 → 18 (20 for the private Filament SDK) | 14 / 18 / 21 | |
| AppleClang | B | 14 → 17 (Xcode 16.3+) | — | macos-15 defaults to Xcode 16.4. |
| MSVC | B | 19.30 → 19.32 under C++20 option B | — | 17.2 is the first release with a stable `/std:c++20` ABI. |
| Ninja | B | none → 1.11 (documented) | 1.10.1 / 1.11.1 / — | |
| GoogleTest, Benchmark | B + F | fetched 1.15.2 / 1.9.1 → 1.17+ / 1.10.0; system ≥ 1.14 / 1.8.3 | 1.11 / 1.14 / 1.17; 1.6.1 / 1.8.3 / 1.9.1 | |
| Tracy | B | 0.11.1 → 0.14.x | — | dev only |
| Docs, coverage and profiling tools | H | default env → own features | — | The Sphinx stack, Doxygen, lcov, pyperf. |
| clang-format | B | 14 → 23 | 14 / 18 / 21 | One mechanical reformat after M1d. |

## Ubuntu 24.04 floor

| Item | Today | Ubuntu 24.04 | DART 7 floor |
| --- | --- | --- | --- |
| GCC / Clang / CMake | 11.2 / 13 / 3.22.1 | 13.2 / 18 / 3.28.3 | 13.2 / 18 / 3.28 |
| Python / NumPy | 3.10 / 1.21.5 | 3.12.3 / 1.26.4 | 3.12 / 1.26.4 |
| Eigen, FCL, libccd, octomap, ODE | 3.4.0, 0.7.0, —, 1.9.7, 0.16.2 | same | unchanged |
| assimp, tinyxml2, libpng | 5.2.2, 9.0.0, — | 5.3.1, 10.0.0, 1.6.43 | 5.3.1, 10.0.0, 1.6.43 |
| Bullet | 3.06 | 3.24 | 3.05 detection floor |
| fmt, spdlog | 8.1.1, 1.9.2 | 9.1.0, 1.12.0 | removed (DD-4) |
| urdfdom, libcurl, OpenSSL | 3.0.0, 7.85.0, 1.1.1 | 4.0.0, 8.5.0, 3.0.13 | kept (DD-5, DD-12): the 24.04 versions (urdfdom 4.0.0 on source builds, 6.0.1 in Linux wheels) |
| GoogleTest / Benchmark (system) | any | 1.14.0 / 1.8.3 | 1.14 / 1.8.3 |

### Gaps where DART needs more than noble ships

| Gap | Noble | DART needs | Closed by |
| --- | --- | --- | --- |
| GLFW | 3.3.10 | 3.4 | A static X11 fetch fallback behind `DART_USE_SYSTEM_GLFW` (DEP-20) |
| Filament | none | ==1.77.3 | The private SDK built with clang-20 from noble-updates (DEP-20 spike first) |
| Dear ImGui | 1.90.1 | 1.92.8 docking | A fetched, hash-pinned tag archive (DD-14, DEP-30) |
| nanobind | 1.9.2 | 3.1.x | FetchContent, or pip nanobind 3.1 |
| sdformat | none in the archive | 14.8 to 16 | The OSRF apt repository, a fetch when absent once DEP-25 supports it (DD-11), or building without SDF (DEP-18) |
| scikit-build-core | 0.7.0 | ≥ 1.0 | pip build isolation (M5a) |

### Support tiers

| Tier | Environment | What builds | Enforced by |
| --- | --- | --- | --- |
| 1 | Stock noble apt plus OSRF sdformat | `dart` and `dart-io`, headless | A nightly apt lane (DEP-12) |
| 2 | Noble plus DART's fallbacks (network at configure, clang-20, pip nanobind and scikit-build-core) | adds `dart-gui` and dartpy | The Tier 2 spike, then a nightly lane (DEP-20) |
| 3 | pixi, conda-forge and wheels | everything | Existing CI |

## Footprint (reported)

The "after" column is the 2026-10-18 research's prediction for the old plan,
which also removed urdfdom, libcurl and OpenSSL. DD-5 and DD-12 keep them, so
the predicted closure sizes and environment savings below are superseded and
unmeasured; a new measurement needs a repository script.

| Item | Today | After the 2026-10-18 plan (superseded) |
| --- | --- | --- |
| dartpy shared-import closure, assets-ON builds | 43 third-party libraries, 89.9 MB | about 29 libraries, 42.1 MB |
| `dart-io` closure, assets ON | 30 libraries, 82.1 MB | about 15 libraries, 33.1 MB |
| Core `libdart` closure | 9 libraries, 27.9 MB (assimp and zlib 51%, FCL 37%) | unchanged in 7.0 |
| Third-party symbols exported by `libdart` | 372 spdlog, 61 fmt | 0 |
| Packages in DART's build lists | — | 3 fewer after DD-4 and DEP-1: Boost, spdlog, fmt (the other five of the old plan stay) |
| Default pixi environment (Linux) | 1.9 GB | superseded; DEP-7 reports 1.6 GB (#3890; reported, unverified, no provenance recorded here) |

## In-tree code worth a separate decision

- `dart-simd`: about 15K lines of installed headers with no production
  consumer.
- C++17-era compatibility headers: `Filesystem.hpp`, `Optional.hpp`,
  `common::make_unique`, `DART_CREATE_MEMBER_CHECK`.
- About 300 lines of C++20 polyfills, all in installed headers
  (`collision/dart/detail/Span.hpp`, the `dart::simd` bit and numbers
  helpers); they stay while installed headers are C++17.
