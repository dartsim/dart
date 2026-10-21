# Decisions

Revisit a decision only with new evidence and maintainer approval.

## Maintainer direction (2026-10-18)

- **DD-0. Minimal, self-contained DART 7.** Optimize DART 7's dependencies
  (not DART 6's): use as few as possible so DART is self-contained, more
  feature complete and faster to evolve.
- **DD-1. Ubuntu 24.04 floor.** Raise the minimum supported Ubuntu LTS from
  22.04 to 24.04 on `main`, and raise every dependency floor to match.
  `release-6.20` keeps Ubuntu 22.04.
- **DD-2. C++ standard.** Raise the C++ standard where that is possible on
  the new floor. The evidence and options are in Q1 below.

## Answers to the decision batch (2026-10-19)

- **DD-3. C++ standard: option B.** DART's own targets build as C++20;
  exported targets keep `cxx_std_17`; installed headers stay C++17-clean and
  the `ContactSurface.hpp` include tree stays parseable at gz-physics'
  `-std=c++14` probe; the C++20 library components GCC 16 changed are banned
  in DART code until the GCC floor reaches 16. AGENTS.md's "preserve C++17"
  and the PLAN-624 constraint are reworded to say exactly this (DEP-21).
- **DD-4. Standard library over third-party libraries.** The maintainer's
  words: "Using STL is good approach over 3p libraries." Logging moves to a
  DART-owned sink and formatter built on the standard library, and spdlog and
  fmt leave DART (DEP-14 to DEP-16).
- **DD-5. Specialized libraries for robot description formats.** The
  maintainer's words: "urdf and sdf would be better to use the specialized
  libraries." urdfdom stays as the URDF parser and sdformat as the SDF
  parser; the in-house URDF reader (former DEP-17) is dropped. Whether SDF
  moves into its own library while still using sdformat (DEP-18) is part of
  the follow-up study below.
- **DD-6. Asset networking needs a tradeoff study.** The maintainer's words:
  "Not sure about assets. Need more tradoff study for me." DEP-19 waits for it.
- **DD-7. Clean, long-term public surface.** The maintainer's words: "Prefer
  the clean/long-term/proper approach, as long as keeping backward
  compatibility for gz-physics and gz-sim." The Q3 batch is approved, and
  later packets prefer the clean fix over a compatibility shim wherever the
  gz-physics and gz-sim compatibility gates still pass.
- **DD-8. Dependency acquisition.** The maintainer's words: "Probably
  case-by-case. However, fetch first for build from source (so that users
  don't need to install deps unnecessarily), system first when building for
  package manager (but in this case package managers can explicitly set the
  DART_USE_SYSTEM_<X>). Bundled should be removed whenever possible over the
  other two options." Default source builds fetch pinned versions where that
  works, packagers opt into system packages with `DART_USE_SYSTEM_<X>`, and
  in-tree bundled copies give way to fetched or system packages. The
  per-dependency matrix is part of the follow-up study.

- **DD-9. Balance, not blind removal.** The maintainer's words: "find good
  balance between self-contained (less dependency) vs.
  simplify/productivity (more dependency)" and "not simply/blindly removing
  dependencies". Every removal, replacement or internalization must show a
  net benefit: what it saves (packaging, footprint, exposure in installed
  headers, upstream churn, security surface) against what it costs (code DART
  then owns and maintains, its bugs and security fixes, build and developer
  productivity, features the library provided). A dependency that is well
  maintained, cheap to obtain and kept out of DART's public interface may stay.
  This also governs how DD-4 and DD-8 apply case by case.

## Follow-up study (requested 2026-10-19)

- Asset networking: keep, isolate or move to Python, with security,
  footprint, packaging and maintenance tradeoffs (DD-6).
- SDF packaging with sdformat kept: its own library or not (DD-5).
- The per-dependency acquisition matrix under DD-8, and an inventory of
  in-tree bundled third-party code.

## Second decision batch (2026-10-19, open)

The balance review under DD-9 ([balance-review.md](balance-review.md)) asks
five questions and lists the approvals its packet plan needs. The
recommended option comes first.

- **Q1. fmt (DD-4).** (a) DEP-15 as recorded: a standard-library formatter
  without byte parity with fmt. (b) Keep fmt hidden in `Logging.cpp` until
  `std::format` is allowed, which re-opens DD-4.
- **Q2. C++ asset fetching in the 7.0 API (DD-6).** (a) Delete the
  unreleased `utils-assets` component before F; a standard-library Python
  helper fetches sample data and later becomes `dart.data`. (b) Keep it and
  add DEP-2. Also: should D22's "a fetcher" fetch the humanoid bundles for
  wheel users?
- **Q3. SDF packaging (DEP-18).** (a) An optional part of `dart-io` that
  warns and skips SDF without sdformat. (b) The same, but a missing sdformat
  stops configure unless `DART_SKIP_SDFORMAT=ON`. (c) A separate
  `dart-io-sdf` library.
- **Q4. Fetch first for the libraries Gazebo also loads (re-opens DD-8).**
  (a) Fetch first only for ImGui, nanobind, GoogleTest, Benchmark and Tracy
  (and the prebuilt Filament SDK where the DEP-20 spike agrees); system only
  for Eigen, libccd, octomap, FCL, assimp, tinyxml2, the urdfdom group,
  sdformat, GLFW 3.3.10, libpng and libjpeg-turbo. (b) As (a), plus a
  fetched static GLFW. (c) As (a), plus find-or-fetch fallbacks in 7.x.
- **Q5. The Dear ImGui patch.** (a) Drop it in 7.0 once DEP-31 fixes DART's
  own font path. (b) Keep its guards, applied without Git.
- **Approvals:** DEP-17 PR4 (the urdfdom-typed headers leave the installed
  set before F); DEP-12(b) keeps libpng at its API-minimum floor (a DD-1
  exception); DEP-25 and DEP-8(f) as shared mechanical steps; DEP-19 PR2
  removes curl and OpenSSL together; the release-6.20 backport of DEP-17
  PR3's wheel bump; the labeled changes listed in the review's §4.

## Original questions (2026-10-18)

Asked as one batch; the answers are above.

- **Q1. C++ standard.** Recommended: DART's own targets build as C++20 while
  exported targets keep `cxx_std_17` and installed headers stay C++17-clean
  (the `ContactSurface.hpp` include tree stays parseable at gz-physics'
  `-std=c++14` probe), with the GCC-16-changed C++20 library components
  banned in DART code until the GCC floor reaches 16. Alternatives: export
  C++20 to consumers (gz-physics' plugin silently becomes C++20, gz-physics'
  Bazel and C++17 pkg-config users break, and GCC < 16's experimental C++20
  library ABI enters DART's ABI), or stay on C++17. Evidence:
  [verification.md](verification.md) S1, S3c, S2 and S2b.
- **Q2. Removals in 7.0.** Recommended: all four, before API freeze `F`:
  logging (spdlog and fmt, DEP-14 to DEP-16), URDF (the urdfdom stack,
  DEP-17), asset networking (libcurl and OpenSSL, DEP-19), and SDF in its own
  library (DEP-18). Unselected items keep today's dependency at the 24.04
  floor.
- **Q3. Small public-surface cleanups for 7.0.** Recommended: approve all.
  The items: drop the exported Boost define; drop the always-1 `ASSIMP_*`
  config macros; trim transitive includes (FCL from `VoxelGridShape.hpp`,
  assimp from `BulletCollisionDetector.hpp`, forward-declared octomap types);
  7.0 deprecation attributes for 8.0 removal (`getAssimpMesh`,
  `AssimpInputResourceAdaptor`, fcl `createMesh(aiScene)`, the namespace-Eigen
  aliases); drop the dartpy `aiScene` overloads; an assimp importer allowlist
  in Linux wheels; retired switches kept as no-ops through 7.x; Bullet and ODE
  usage requirements moved from core to their components. No gz-physics,
  gz-sim or dartpy use was found for any of them.
- **Q4. Fetch policy.** Recommended: keep per-dependency defaults (fetch
  first only where an exact version matters: the ImGui docking release,
  nanobind, GoogleTest, Benchmark, Tracy; system first with a fetch fallback
  elsewhere, adding GLFW 3.4), and give every fetched dependency a
  `DART_USE_SYSTEM_<X>` switch that fails fast for offline packagers.

## Defaults taken without a question

Object to change any of these.

- FCL stays a core dependency and the C++ default detector in 7.0 (PLAN-620,
  D13). assimp, Bullet and ODE stay; their removal is the 8.0 track (DEP-24).
- Compiler floors: GCC 13.2, Clang 18 (20 only for the private Filament SDK),
  AppleClang 17 with Xcode 16.3+, and MSVC 19.32 under the recommended C++
  option. CMake 3.28; the installed consumer fallback stays at 3.22.1.
- Support tiers: Tier 1 (core and `dart-io` from noble apt plus OSRF
  sdformat) is enforced by a nightly apt lane; Tier 2 (GUI and dartpy on
  noble through DART's fallbacks) follows the DEP-20 spike.
- Bullet keeps the 3.05 detection floor from #3829 (3.06 and 3.07 report
  3.05; no code needs a newer Bullet, and Fedora and EPEL ship 3.08). The
  Eigen, FCL, libccd, octomap and ODE floors do not change.
- #3829 (the maintainer's open packaging PR) is independent: if it merges
  first, DEP-12 raises the floors on top of it.
- libpng and libjpeg-turbo stay system libraries with decode caps; swapping
  them for vendored codecs is deferred.
- Dear ImGui stays bundled and is vendored in-tree in 7.x; Filament stays
  exactly pinned at 1.77.3, now enforced in CMake, and moves to 1.78.x in 7.x.
- dartpy stays one all-in-one module that requires the GUI and IO (the
  recorded 7.0 decision).
- The default pixi environment stays on Eigen 3.4.0, with a separate Eigen 5
  lane.
- Dependency packets run alongside the M1 critical path, decision-free ones
  first; step-path hunks follow the PLAN-624 program-owned merge order, each
  re-qualified on current `main`.
- gz-sim's `INTEGRATION_reset_sensors` failure is external (a use-after-free
  in the test); the DART 7 program session owns its record and the upstream
  report.
- `dart-simd` (about 15K lines of installed headers with no production
  consumer) and the C++17-era compatibility headers are left for a separate
  decision.
