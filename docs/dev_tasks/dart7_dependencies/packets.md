# Packet details

Each packet is one branch and one verification story. File references are
as of `main` `17ce7eefa`; re-check them before starting. An executor that
finds the real scope materially different stops and reports. Gate shorthand
is defined in the [README](README.md#gates). Decisions DD-0 to DD-8
are in [decisions.md](decisions.md); "after Q3" items are approved by DD-7.

## DEP-0. Assert the gz contact-surface probe

- **Objective:** a regression in gz-physics's detection of
  `dart/constraint/ContactSurface.hpp` fails the Gazebo lanes instead of
  silently compiling out contact-surface customization.
- **Scope:** in `tools/gazebo/compat/lane.sh` (`step_gz_physics`, after the
  second configure), fail unless the gz-physics `CMakeCache.txt` has
  `DART_HAS_CONTACT_SURFACE_HEADER:INTERNAL=1`, and log the probe's compile
  command. `test-gz` configures once, so its common tests skip the contact
  expectations; assert there too after a second configure. Record that
  Windows has no gz lane and that gz's `/std:c++14` MSVC probe likely already
  fails against DART's nested namespaces.
- **Non-goals:** changing the probe or gz-physics.
- **Gates:** gz, plus a negative control: a scratch C++20-only token in a
  header of the probe's include closure makes the lanes fail.

## DEP-1. Remove Boost

- **Objective:** delete a requirement nothing uses.
- **Scope:** `libboost-devel` in the default and `gz-compat-base` pixi
  features, `boost-devel` in `docker/wheels/Dockerfile.manylinux_2_28`,
  `boost-libs` in `ci_freebsd.yml`; regenerate `pixi.lock` in the merging PR.
  After Q3, remove the exported `BOOST_ALL_NO_EMBEDDED_GDB_SCRIPTS` define in
  `dart/CMakeLists.txt` with a CHANGELOG entry.
- **Non-goals:** the libboost runtime that conda assimp pulls into the Gazebo
  environments.
- **Gates:** base, gz, pkg, `pixi install` for every environment, the Linux
  wheel image build, BSD.

## DEP-2. Export CURL and OpenSSL only for static builds

- **Objective:** an assets-ON shared build stops forcing curl and OpenSSL dev
  packages on every `io`, `utils` and `utils-urdf` consumer (which includes
  gz-physics).
- **Scope:** guard the component dependency in
  `dart/utils/assets/CMakeLists.txt` with `if(NOT BUILD_SHARED_LIBS)`, as
  `dart/gui/CMakeLists.txt` does for the GUI packages; correct
  `docs/onboarding/io-parsing.md`.
- **Gates:** base, pkg (a shared consumer without curl or OpenSSL dev files
  configures; a static one still requires them), gz.

## DEP-3. Bullet link trim and Bullet/ODE usage requirements

- **Objective:** no unused Bullet libraries in the link interface; after Q3,
  core-only consumers no longer need Bullet and ODE dev packages.
- **Scope:** (a) link `BULLET_COLLISION_LIBRARY` and `BULLET_MATH_LIBRARY`
  instead of all four Bullet libraries in `cmake/DARTFindBullet.cmake`;
  (b) after Q3, link Bullet and ODE PRIVATE into core in shared builds and
  carry their include directories, `BT_USE_DOUBLE_PRECISION` and static link
  on the `collision-bullet` and `collision-ode` forwarding targets.
- **Non-goals:** the floor (DEP-12); backend facades (8.0).
- **Gates:** base, gz, pkg (a `dart`-only consumer configures without Bullet
  or ODE dev packages), `readelf` NEEDED, wheel.

## DEP-4. Find-module hygiene and fetch fallbacks

One PR per dependency.

- (a) Delete the dead octomap target shim and the unreachable
  `octomap_VERSION` branch; pin pixi octomap ≥ 1.10.1 and move the wheel to
  1.10.1.
- (b) Repair the ODE and Bullet FetchContent fallbacks under CMake 4: a scoped
  `CMAKE_POLICY_VERSION_MINIMUM`, ODE from its GitHub home, explicit double
  precision, a lane with `DART_USE_SYSTEM_ODE=OFF` and
  `DART_USE_SYSTEM_BULLET=OFF`, and a configure check that system ODE uses
  double precision.
- (c) Delete the dead GCC `-faligned-new` block in `dart/CMakeLists.txt`.
- (d) After Q3: remove the vestigial FCL include from `VoxelGridShape.hpp`
  and forward-declare the octomap types in `VoxelGridShape.hpp` and
  `PointCloudShape.hpp`.
- (e) An opt-in switch that turns "warn and drop" for a missing Bullet or ODE
  into a hard error for Gazebo-facing packagers.
- **Non-goals:** removing the `Findfcl` MODULE fallback; FCL optionality.
- **Gates:** base, gz, pkg (installed find modules run under consumer policy
  3.5.1), dis, BSD, voxel tests.

## DEP-5. Eigen 5 readiness

- **Objective:** warning-clean `-Werror` builds on Eigen 3.4.0 and 5.0.x with
  no public API or ABI change, an Eigen 5 lane, then wider pins.
- **Scope:** the deprecated runtime-option `JacobiSVD::compute` calls in
  `BalanceConstraint.cpp`, `HierarchicalIK.cpp` and `InverseKinematics.cpp`
  (the IK files are near the quiet window; confirm with the PLAN-624 owner);
  the two `eulerAngles` display callers; an Eigen 5 lane on Linux, macOS or
  FreeBSD (MSVC's `/wd4996` hides the deprecations); a build-time Eigen major
  recorded by the installed finder; pins `>=3.4.0,<6` with the default
  environment staying on 3.4.0.
- **Non-goals:** JacobiSVD layout or allocator changes (8.0); vendoring Eigen.
- **Gates:** base on both Eigen majors, py, `ContactSurfaceProbe`,
  `pixi run test-eigen-overalignment`, gz.

## DEP-6. Fetched test tooling

- **Scope:** fetched GoogleTest 1.15.2 → 1.17.0 or newer, Benchmark 1.9.1 →
  1.10.0 with install off, and a lane that builds them
  (`DART_USE_SYSTEM_GOOGLETEST=OFF`, `DART_USE_SYSTEM_GOOGLEBENCHMARK=OFF`).
- **Gates:** base both ways; apt with system GoogleTest 1.14 and Benchmark
  1.8.3.

## DEP-7. Slim the default pixi environment

- **Scope:** docs feature (Sphinx stack, Doxygen) used by the docs tasks and
  `api_doc.yml`; coverage feature (lcov); pyperf into `profile`; drop pipx;
  pin pyyaml and packaging explicitly; move conda imgui into the feature the
  `DART_USE_SYSTEM_IMGUI=ON` lanes use.
- **Gates:** every pixi task still resolves; check-lint; test-all; docs build
  in the docs environment; ai; `du` of the default environment before and
  after.

## DEP-8. Wheel hygiene

Independent PRs.

- (a) Stop packaging `python/tests` as a top-level `tests` package.
- (b) Strip bundled shared libraries during repair (reported 7.2 MB
  unpacked).
- (c) Move the Linux wheel from sdformat15 to sdformat16, as macOS and
  Windows already are.
- (d) Build libpng ≥ 1.6.57 in the image unless AlmaLinux's package carries
  the CVE-2025-65018 fix.
- (e) After Q3: an assimp importer allowlist for Linux wheels (Collada, STL,
  OBJ at minimum; OBJ is needed for the TriMesh round trip) with exporters off
  by default but never `ASSIMP_NO_EXPORT`.
- **Gates:** wheel, including loading `.dae`, `.stl` and `.obj` from `data/`,
  a clone test and a TriMesh-built mesh rendered in the GUI.

## DEP-9. Python 3.12 and NumPy 1.26.4

- **Scope:** one `find_package(Python 3.12 ...)` lookup in
  `python/CMakeLists.txt` replacing the lookup in
  `cmake/DARTFindDependencies.cmake` (keep the `Python3_EXECUTABLE` cache
  name); `python_requires>=3.12` and `numpy>=1.26.4`; fix the NumPy major
  sniff; retire the py310 and py311 wheel environments and publish rows; run
  the abi3 smoke test on a current Python and add a NumPy 1.26.4 smoke test;
  update the requirement tests and `docs/onboarding/python-bindings.md`.
- **Non-goals:** scikit-build-core (M5a step 2); a NumPy 2 floor.
- **Gates:** base, test-build-requirements, py, a Python 3.11 configure fails
  clearly, wheel.

## DEP-10. nanobind 3.1.x at configure

- **Scope:** check `NB_INTERNALS_VERSION == 22` at configure in
  `python/CMakeLists.txt` instead of failing on a static assert; pin pixi
  nanobind 3.1.*; one lane with `DART_USE_SYSTEM_NANOBIND=ON`.
- **Gates:** base, py, a mismatched nanobind fails at configure.

## DEP-11. C++20-clean tree and header dialect gates

Four PRs in order; the default standard stays C++17.

1. Fix today's header `-Wdeprecated` warnings (redundant out-of-line
   constexpr statistics in the aspect headers, Clang's
   `-Wdeprecated-copy-with-dtor` in `ShapeFrameAspect.hpp`) and replace
   `throw()` with `noexcept` in `StlAllocator`.
2. Explicit captures for the six implicit-`this` lambdas:
   `InverseKinematics.cpp` (three), `optimizer/Function.cpp` (one) now;
   `World.cpp` (two) after M1d or through the PLAN-624 owner.
3. Third-party header warnings: SYSTEM includes or a scoped suppression for
   bundled ODE's enum-enum conversion, and push/pop suppression of octomap's
   `<ciso646>` warning around the FCL and octomap includes.
4. A CTest that compiles every installed header standalone (header list from
   the install manifest) at C++17 (MSVC `/std:c++17` with DART's parsing
   options) and at C++20 with new diagnostics as errors;
   `ContactSurfaceProbe` stays at `-std=c++14` on Unix.

- **Gates:** base at C++17; a `-DCMAKE_CXX_STANDARD=20
  -DDART_TREAT_WARNINGS_AS_ERRORS=ON` build on GCC 13.3, GCC 15 and Clang 21,
  plus a variant with octomap 1.10.0; bit-identical outputs; gz.

Follow-up (from the DEP-11 review): about 30 other property and state
structs with virtual destructors (joint, body node, skeleton, point mass and
optimizer function properties) share the use-triggered deprecated-copy
pattern. A mechanical PR can default their copy operations the same way.

## DEP-12. Ubuntu 24.04 floor

- **Before starting:** #3829 is the maintainer's own open PR and is
  independent: if it merges first, DEP-12 raises the floors on top of it (its
  assimp 5.2.0 relaxation is superseded by DEP-13; its Bullet 3.05 detection
  correction stays).
- **(a) Toolchain, CMake and CI (the shared mechanical step):** compiler
  floors in `cmake/dart_defs.cmake` (GCC 13.2, Clang 18, AppleClang 17, MSVC
  19.32 under option B); CMake 3.28 in `CMakeLists.txt`,
  `cmake/Components.cmake`, examples, tests and tools; delete the guards it
  obsoletes; set `CMAKE_CXX_SCAN_FOR_MODULES` OFF; leave the
  `DARTConfig.cmake.in` consumer fallback alone; sync `dart.pc`; retire the
  Ubuntu 22.04 GCC 11 and Clang 13 nightly rows and add GCC 13 and Clang 18
  rows on 24.04; add the Tier 1 apt lane after triaging the distro FCL
  assertion; update `build.rst`, `building.md`, the requirement tests and
  `.readthedocs.yml`.
- **(b) Library floors, one PR per library:** tinyxml2 10.0.0, libpng
  1.6.43, and the remaining 24.04 floors for any dependency Q2 keeps.
- **Gates:** base, test-build-requirements, pkg, apt, dis, Windows, macOS,
  BSD, gz.

## DEP-13. assimp 5.3.1 and a narrower role

- **Scope:** (a) floor 5.3.1; delete the issue-451 try-compile, the
  `aiScene`/`aiMaterial` shims in `MeshShape.cpp`, the up-axis fallbacks and
  the unit-size test guard; after Q3, delete the always-1 `ASSIMP_*` config
  macros. (b) Delete the never-called ODE and Bullet `aiScene` paths; after
  Q3, drop `<assimp/scene.h>` from `BulletCollisionDetector.hpp`. (c) After
  Q3, before F: deprecation attributes on `SoftMeshShape::getAssimpMesh`,
  `AssimpInputResourceAdaptor`/`createFileIO` and fcl `createMesh(aiScene)`,
  plus an additive TriMesh accessor. (d) A regression test mirroring
  gz-physics' `CustomMeshShape`, and an ASan check of the reported
  `MeshOwnership::None` leak.
- **Non-goals:** removing the `aiScene` API or making assimp optional (8.0).
- **Gates:** base, mesh tests, gui, apt, gz.

## DEP-14. Logging boundary and DART log sink; drop spdlog (DD-4)

- **Scope:** `Logging.hpp` keeps its macros and template signatures,
  type-erases arguments behind an inline level check, and stays parseable as
  C++14; a new `Logging.cpp` formats and dispatches to a thread-safe sink with
  `setLogSink`/`setLogLevel` and a default writer (warn and above to stderr);
  move `DART_ACTIVE_LOG_LEVEL` into the generated `config.hpp` so pkg-config
  consumers match; remove the spdlog link, definition, pkg-config entry, pixi
  pins, wheel build, Brewfile and FreeBSD entries; keep `DART_SKIP_spdlog` as
  a deprecated no-op; fix the dartpy logging format bug.
- **Gates:** base, gz, `ContactSurfaceProbe`, pkg (no fmt or spdlog in the
  installed targets), TSan, no new steady-state allocations.
- **Compat:** a labeled default-behavior change with a migration note.

## DEP-15. Standard-library formatter; drop fmt (DD-4)

- **Scope:** a formatter in `Logging.cpp` supporting `{}`, `{:.Ne}`, `{:.Nf}`,
  `{:.Ng}` and escaped braces, tolerant of malformed strings, using
  `std::to_chars`; switch `XmlHelpers.cpp`; delete `DARTFindfmt.cmake`, its
  fetch fallback and every fmt pin and image build step; keep
  `DART_USE_SYSTEM_FMT` as a no-op through 7.x; a table test pins the output
  against today's fmt.
- **Gates:** base on Linux, macOS, Windows and FreeBSD; wheel; pkg; gz.

## DEP-16. Diagnostics through the sink (DD-4)

- **Scope:** the dartpy logging bridge (`logging.getLogger("dartpy")`,
  queued from executor threads, drained under the GIL); route the
  `Console.hpp` streams through the sink without changing their text; the GUI
  log panel subscribes to the sink.
- **Gates:** base, py including multithreaded `step(n)`, gui, gz.

## DEP-17. urdfdom kept: floor and cleanup (DD-5)

- **Objective:** keep urdfdom as DART's URDF parser (DD-5) on a floor that
  matches the Ubuntu 24.04 baseline, and clean up DART's own URDF glue.
- **Scope:** set by the follow-up study (floor choice, the TinyXML v1 compat
  code a higher floor removes, the duplicated joint-metadata parser, the
  null-attribute handling in `urdf_world_parser.cpp`, and whether DART's
  world-file parser copy should defer to urdfdom's).
- **Superseded:** the in-house URDF reader planned on 2026-10-18 is dropped.

## DEP-18. SDF packaging with sdformat kept (DD-5; scope pending the follow-up study)

- **Scope:** move `dart/utils/sdf/*.cpp` into a shared `dart-io-sdf` library
  and `io-sdf` component; `dart-io` drops sdformat; option `DART_IO_SDF`
  (default ON; a missing sdformat stays a hard error unless set OFF); legacy
  `utils` forwards to `io` and `io-sdf`, `utils-urdf` to `io` only; export
  sdformat as a dependency package only for static builds; accept sdformat 17
  once a lane builds it; replace the `sdformat` rosdep key; gate SDF tests and
  examples on the option.
- **Gates:** base, py, gz (including a plugin that requests only
  `utils-urdf`), pkg, a build with every sdformat lookup disabled, an
  identical exported `SdfParser` symbol list, BSD.

## DEP-19. Asset networking (DD-6; option pending the follow-up study)

1. A Python stdlib fetcher (HTTPS-only redirects, limits, timeouts, a byte
   cap, CA bundle overrides, atomic cache writes) used by the robot-asset
   tasks.
2. The C++ `ModelResourceRetriever` becomes an always-built offline verifier;
   online requests get an actionable error; curl code removed.
3. An in-house SHA-256 tested against NIST vectors and `std::random_device`
   staging names; OpenSSL removed. `DART_BUILD_UTILS_ASSETS` becomes a no-op
   and `utils-assets` stays a forwarding component.

- **Gates:** base, every TLS, integrity and concurrency test ported, the C++
  verifier accepts fetcher caches, test-robot-assets, wheel, ai, gz, pkg.

## DEP-20. GUI on stock 24.04

- (a) Spike: the private Filament 1.77.3 SDK on noble with clang-20 and
  GCC 13's libstdc++, the GUI tests under lavapipe, and dartpy against system
  Python 3.12.
- (b) A static X11 GLFW 3.4 fetch fallback behind `DART_USE_SYSTEM_GLFW`.
- (c) Decide, with the spike's evidence, what configure does when no 1.77.3
  SDK is found (today it fails).
- (d) Cap PNG and JPEG decode dimensions before allocating.
- (e) Enforce the Filament pin in CMake.
- **Gates:** base, gui, wheel including the headless-import check.

## DEP-21. DART-owned targets build as C++20 (Q1)

- **Scope:** the rule and doc changes (AGENTS.md, PLAN-624's constraint, the
  branch profile and its checker, code-style); `CXX_STANDARD 20` and `PRIVATE
  cxx_std_20` on `dart`, `dart-io`, `dart-gui` and dartpy while every
  exported requirement stays `cxx_std_17`; the C++20 library rule with a grep
  gate.
- **Gates:** in [verification.md](verification.md#c-standard-gates).
- **Timing:** after M1d, DEP-0, DEP-11 and DEP-12(a).

## DEP-22. Developer-tooling refresh (7.x)

- Lift the GLUT-era `<4.3.4` CMake cap after wheel validation; Tracy 0.14.x;
  upstream the FreeBSD patches; clang-format 14 → 23 with one recorded
  mechanical reformat after M1d; formatter bumps one per PR.

## DEP-23. Packaging and downstream handoff

- A "Packaging DART 7" section in `docs/onboarding/building.md`
  (`DART_USE_SYSTEM_*` table, offline builds, fetched installs, precision by
  channel, tiers, removed dependencies) and recipe notes for conda-forge,
  Homebrew, vcpkg, Debian/OSRF and `gz_dartsim_vendor`.
- Upstream proposals, each drafted and approval-gated: gz-physics'
  `CustomMeshShape` on the TriMesh constructor, a DART-owned batch raycast
  API, gz-physics's MSVC probe at `/std:c++17`, and gz-physics requesting
  `utils-urdf` only against DART 7.

## DEP-24. 8.0 design record

- assimp optional and the `aiScene` API removed; FCL optional behind an
  FCL-free `FCLCollisionDetector` facade that keeps the header path, class,
  factory key and observed semantics; Bullet and ODE facades on a DART-owned
  raycast API; tinyxml2 out of installed headers; the 7.0 deprecations
  removed; a C++20 export once gz-physics updates its probes and the GCC floor
  is 16; a DART-native voxel grid to retire octomap.
