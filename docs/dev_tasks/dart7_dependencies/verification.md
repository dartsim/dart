# Verification

Evidence per spike and packet. A packet is done only after its gates pass and
two clean review passes are recorded on its final state. Every claim records
its revision, command, date and scope. Spike logs are local to the machine
that ran them and are not published.

## Research workflow (2026-10-18, `main` `17ce7eefa`)

- Eleven dependency dossiers (Eigen; fmt and spdlog; assimp; FCL, libccd and
  octomap; Bullet and ODE; tinyxml2 and urdfdom; sdformat; the GUI stack;
  libcurl and OpenSSL; the Python stack; build and developer tooling) and seven
  studies (C++ toolchains, C++ downstream effects, C++20 benefit sites,
  packaging, footprint, upstream health, prior art), run by Claude agents.
- Each result went through one adversarial verifier, or two with separate
  code and external-facts lenses for assimp, the collision core, URDF,
  logging, the GUI and both C++ standard studies. A synthesis, a completeness
  critique and a revision followed. Corrections the verifiers made are carried
  into [inventory.md](inventory.md).
- Independent Codex audit (`gpt-6.1-sol`, ultra, read-only) of the C++
  standard question: recommends building DART's own code as C++20 while the
  installed interface keeps C++17, with the gates listed under
  [C++ standard gates](#c-standard-gates).
- Orchestrator spot checks of load-bearing claims, by reading the sources:
  `DartLoader` uses urdf types only in forward-declared private static
  helpers; the only GLFW 3.4-only call is `glfwInitHint(GLFW_PLATFORM, ...)`
  in `dart/gui/Viewer.cpp`; OpenSSL is used only for EVP SHA-256 and
  `RAND_bytes` in `ModelResourceRetriever.cpp`; gz-physics (Ionic and main)
  requests `collision-bullet collision-ode utils utils-urdf` and its dartsim
  sources include no `dart/utils` header.

## Ubuntu package versions (2026-10-18)

`apt-cache policy` in `ubuntu:22.04` and `ubuntu:24.04` containers after
`apt-get update`, and on an Ubuntu 26.04.1 host. Results are tabulated in
[inventory.md](inventory.md).

## S3: current `main` on stock Ubuntu 24.04 (2026-10-18)

- Setup: `ubuntu:24.04` container; apt packages for every dependency (GCC
  13.3.0, CMake 3.28.3, Eigen 3.4.0, fmt 9.1.0, spdlog 1.12.0, assimp 5.3.1,
  FCL 0.7.0, octomap 1.9.7, tinyxml2 10.0.0, urdfdom 4.0.0, Bullet 3.24, ODE
  0.16.2, GoogleTest 1.14.0, Benchmark 1.8.3), and libsdformat14-dev 14.9.0
  from the OSRF apt repository.
- Configure: `cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
  -DDART_BUILD_GUI=OFF -DDART_BUILD_DARTPY=OFF -DDART_BUILD_UTILS_ASSETS=ON
  -DDART_USE_SYSTEM_GOOGLETEST=ON -DDART_USE_SYSTEM_GOOGLEBENCHMARK=ON
  -DDART_TREAT_WARNINGS_AS_ERRORS=OFF`, then build `all tests` and run
  `ctest -LE 'gui-device|long-measurement'`.
- Result: configure and build succeed with no warnings; 186 of 187 tests
  pass. `test_Distance` aborts on `fcl::detail::libccd_extension::doSimplex2`'s
  assertion (`gjk_libccd-inl.h:335`, "A is not in region 1") inside Ubuntu's
  FCL 0.7.0, which is built with assertions enabled. Related upstream reports:
  flexible-collision-library/fcl#485 and #366. To triage before the apt lane
  gates (DEP-12).
- With `-DDART_BUILD_DARTPY=ON` configure stops: "The all-in-one dartpy
  package requires DART_BUILD_GUI", and the GUI needs a Filament SDK, which
  Ubuntu does not package.

## S3c: the same container, DART built and exported as C++20 (2026-10-18)

- Change: `CMAKE_CXX_STANDARD 20` and every `cxx_std_17` switched to
  `cxx_std_20`; otherwise as S3.
- Result: the build succeeds with exactly six new warnings, all
  `-Wdeprecated` implicit capture of `this` through `[=]`:
  `InverseKinematics.cpp:1033,1807,1816`, `optimizer/Function.cpp:132`,
  `simulation/World.cpp:3087,3257`. 186 of 187 tests pass; the same
  `test_Distance` abort.

## S1: DART built and exported as C++20 on the host (2026-10-18)

- Setup: pixi default environment (GUI and dartpy on, Filament 1.77.3), host
  GCC 15.2, the S3c change, `pixi run config` then
  `-DDART_TREAT_WARNINGS_AS_ERRORS=OFF`, build `all tests dartpy`.
- Result: the build succeeds with the same six warnings; `ctest -LE
  'gui-device|long-measurement'` passes 197 of 197; the `pytest` target
  passes 1641 tests (26 skipped, 54 expected failures).

## S2: Gazebo lanes against DART exporting C++20 (option C, 2026-10-18)

- Setup: the S1 tree; `pixi run -e gazebo-harmonic gz-compat-harmonic` and
  `pixi run -e gazebo-jetty gz-compat-jetty`; then base variants built from
  unmodified `main` (C++17) in the same work directories and compared with
  `GZ_COMPAT_BASE_VARIANT=base`.
- CMake raised 23 gz-physics compile commands per lane to `-std=c++20`, and
  gz-physics still detected the contact-surface header.
- Harmonic: no new failures or slow cases against the base.
- Jetty: one `NEW` entry, `INTEGRATION_triggered_camera` (timeout, then
  passed), seen while the host load average was about 150. Rerun alone three
  times on the same build at normal load: passed 3 of 3. Classified as load.

## S2b: Gazebo lanes for option B (2026-10-18)

- Setup: DART targets build as C++20 (`CMAKE_CXX_STANDARD 20` and `PRIVATE
  cxx_std_20`) while every exported requirement stays `cxx_std_17`; run as
  variant `optb` in the S2 work directories with `GZ_COMPAT_BASE_VARIANT=base`.
- Observed: DART compiled with `-std=gnu++20` (269 commands); the installed
  targets export `INTERFACE_COMPILE_FEATURES "cxx_std_17"`; every gz-physics
  command used `-std=c++17` (129 Harmonic, 130 Jetty);
  `DART_HAS_CONTACT_SURFACE_HEADER:INTERNAL=1`.
- Result: Harmonic and Jetty both PASS, with no new failures or slow cases
  against the stored baselines and the C++17 base. Remaining entries:
  `UnsupportedRayIntersections` (accepted difference) and
  `INTEGRATION_reset_sensors` (also fails on the base; see below).

## gz-sim `INTEGRATION_reset_sensors` (2026-10-18)

- Observed in every lane above at `reset_sensors.cc:301` and `:351`: after a
  world reset the model's Z reads 1 instead of 100 and 80.39. This is not the
  documented air-pressure race at lines 259 and 352.
- Codex diagnosis (`gpt-6.1-sol`, ultra): the test keeps a raw `Pose`
  component pointer from line 200, and the reset's
  `EntityComponentManagerPrivate::CopyFrom` clears and clones component
  storage, so lines 301 and 351 read freed memory. AddressSanitizer reports a
  heap-use-after-free, also in a reproducer that does not load DART. The test
  fails the same way with DART 6.20.0, `7ceaf5ede` and `17ce7eefa`. A DART-only
  replay of gz-physics' reset sequence passes on all three.
- Ownership: the DART 7 program session records it as an external failure
  and drafts the upstream gz-sim fix (approval-gated).

## C++ standard gates

Required before DEP-21 lands (option B):

1. `-DDART_TREAT_WARNINGS_AS_ERRORS=ON` builds of `dart`, `dart-io`,
   `dart-gui`, dartpy and the tests on GCC 13.3, GCC 15 and 16, Clang 18 and
   21, MSVC v143 (17.2+) and v145 with DART's exported parsing options,
   AppleClang 17, and FreeBSD 14.4; plus a variant with octomap 1.10.0.
2. Bit-identical outputs between the C++17 and C++20 builds for the PLAN-624
   refactor, determinism and regression gates.
3. `abidiff` (or exported symbols plus a DWARF layout comparison) of
   `libdart`, `libdart-io` and `libdart-gui`, C++17 against C++20: no
   exported-symbol or layout difference.
4. Installed targets still export `cxx_std_17`, and `dart.pc` is unchanged.
5. Every installed header compiles standalone at C++17 and at C++20;
   `ContactSurfaceProbe` passes at `-std=c++14`.
6. An external C++17 consumer builds and runs against the C++20-built
   installed libraries.
7. The Harmonic, Ionic and Jetty lanes and `test-gz` with DEP-0's assertion:
   gz-physics compiled as C++17, no new failures against the base (S2b
   already shows this for Harmonic and Jetty on the spike tree).
8. Wheels build, repair and pass `abi3audit --strict` and the headless import
   check on every platform.

## Packet evidence

All packets below are based on `main` `548ff24ad` and are local until the
maintainer approves publishing them.

### Gazebo base variants at `548ff24ad` (2026-10-19)

Built from an exported `548ff24ad` tree as variant `base548` in shared lane
work directories, for every Wave 1 packet to compare against. Harmonic and
Jetty: the only failures are the accepted `UnsupportedRayIntersections`
difference and the external `INTEGRATION_reset_sensors` defect; gz-physics
detects `ContactSurface.hpp` (`DART_HAS_CONTACT_SURFACE_HEADER=1`). Ionic adds
`INTEGRATION_contact_system`, which is already in the Ionic expected-failure
file: its two attempts failed different cases that count asynchronous contact
messages (`ExtraContactData`, then `MultipleCollisionsAsContactSensors`) while
several lanes ran concurrently on the host, the documented load sensitivity of
timing-dependent gz-sim tests. Ionic candidates are compared against it on a
quieter host.

### DEP-0: assert the gz contact-surface probe

- Writer: Claude Sonnet; orchestrator follow-up edits.
- Gates: `pixi run test-gz-compat-tools`, `pixi run lint`; on a reused Jetty
  lane build, the positive control passed and a C++20-only token injected
  into the installed `ContactSurface.hpp` failed the gate with the probe log;
  `pixi run -e gazebo test-gz` passed with gz-physics configured twice
  (gz-physics 199/199 tests, 4 performance tests, gz-sim
  `INTEGRATION_entity_system`), so the README's warning that a second
  configure would turn `test-gz` red was stale and is removed;
  `pixi run -e gazebo config-gz` plus the gate on the final `pixi.toml`.
- Review round 1: Codex found that `config-gz` hid a failed first pass
  (fixed with `bash -ec`); Muse found the stale README warning, a stale-cache
  gap in `config-gz-install`, the untested empty-cache failure shape, and
  wording nits (all fixed).
- Review round 2: Codex (no actionable defects; 3,727 tooling tests pass)
  and Muse (no blockers or majors; every supported flow fails closed on a lost
  probe). Its two optional suggestions, gating manual single-step invocations
  (`run_gz_physics_task.sh test`, `lane.sh <lane> test-gz-physics`), are not
  applied: the supported flows are gated, and DD-9 favors not adding more.
- Committed locally on `test/gz-contact-surface-probe-gate`; its gate also
  ran inside every Wave 1 integration lane below.

### DEP-1: remove Boost

- Writer: Claude Sonnet. Committed locally on `build/remove-boost`.
- Gates: lint, check-lint, build, 199/199 tests, `pixi run test-package-core`
  (no Boost definition in the installed targets), every Gazebo pixi
  environment resolves; the 17 sources the wheel image builds were checked
  for Boost use.
- Reviews: Codex (no actionable regressions) and Muse (nothing substantive).
- Owed: the Gazebo lanes against `base548`.

### DEP-11 PRs 1-2: C++20-clean code

- Writer: Codex. Committed locally on `refactor/cxx20-clean-code` as two
  commits.
- Gates: default build and 199/199 tests; a C++20 build passes 199 C++ and
  1,641 Python tests; all 12,852 exported dynamic symbols unchanged; C++17 and
  C++20 header checks and a C++14 ODR-use probe pass; the only remaining C++20
  diagnostics are the two `World.cpp` captures (quiet window).
- Reviews: Claude (nothing substantive) and Muse (no blockers; follow-up:
  about 30 sibling property structs share the use-triggered deprecated-copy
  pattern).
- Owed: the Gazebo lanes against `base548` (running with DEP-1 and DEP-12(a)
  as one integration tree).

### DEP-12(a): Ubuntu 24.04 toolchain floor

- Writer: Claude Sonnet; orchestrator follow-up edits.
- Gates: lint, check-lint, `pixi run test-build-requirements` (245), build,
  199/199 tests, `pixi run test-package-core`; a consumer with
  `cmake_minimum_required(VERSION 3.5.1)` finds the installed package.
- Review round 1: Codex (no actionable regressions); Muse found that consumer
  projects and the installed examples had been raised to CMake 3.28, which
  removed the committed 3.22.1 consumer coverage (fixed: they stay at 3.22.1).
  The macOS host for Xcode 16.3 is 15.2 per Apple's support table (fixed).
- Review round 2 on the post-fix state: Codex (no actionable regressions) and
  Muse (nothing substantive; it asked for the changelog entry, added at
  commit). Committed locally on `build/ubuntu-2404-toolchain-floor`.
- Owed: the Gazebo lanes (in the same integration tree as DEP-1 and DEP-11).

### DEP-14: DART log sink; spdlog removed

- Writer: Codex.
- Gates (round 2 state): lint, check-lint, build, 199/199 tests, 1,645 Python
  tests, `pixi run test-package-core` with both fmt layouts, the
  ContactSurfaceProbe and a strict C++14 logging consumer,
  INTEGRATION_StepAllocation (disabled levels allocate nothing), 16/16
  logging tests under ThreadSanitizer; libdart exports 0 spdlog symbols
  (was 731); `Logging.hpp` preprocesses to about 39K lines (was 80K).
- Review round 1 (Claude, Muse): fifteen findings fixed, among them
  `std::string_view` and enum arguments, zero-argument calls, colors only on
  terminals, and the Debug-build runtime level. Round 2 (Claude, Muse): no
  blockers; a final small fix round followed (GUI panel classification
  without colors, `setLogSink` returning the previous sink, stream flush at
  exit, fmt include-only linking in `dart-io`). Round 3 (Claude, Muse): no
  blockers.
- Regression caught by the Gazebo integration lane: under the conda-forge
  toolchain (older glibc sysroot, `-Wl,--no-undefined`) `libdart.so` failed to
  link with `undefined reference to pthread_create`. DART uses `std::thread`
  in core but never linked `Threads::Threads` itself; spdlog's CMake target
  had supplied it, and the host's glibc 2.39 hid the gap. Fixed: `dart` links
  `Threads::Threads` PUBLIC, an installed `DARTFindThreads.cmake` resolves it
  for consumers, and `dart.pc` carries the thread flags; the conda-toolchain
  DART build and a gz-physics build against it pass with the probe at 1, and
  the installed `libdart.so` NEEDs `libpthread.so.0`. Round 4 on the fix
  (Claude, Muse): nothing substantive. A disputed comment about which
  overload string literals take was settled by compiling with GCC and Clang
  (literals take `const char*`; mutable `char[]` arrays take the array
  template).
- Committed locally on `refactor/dart-log-sink` with changelog entries.
- Owed: the Gazebo lanes on the full Wave 1 integration tree; Muse's two
  cosmetic nits (an rvalue-string constructor, one `FMT_HEADER_ONLY`
  definition site) move to DEP-15, which reworks the same files.

### Wave 1 Gazebo integration lanes (2026-10-19)

Each packet branch was applied to an exported `548ff24ad` tree (Codex and
Claude review the packets separately; the lanes judge their combination) and
run as an unpatched lane variant against `base548`, using DEP-0's `lane.sh` and
gate script.

| Tree | Harmonic | Jetty | Ionic |
| --- | --- | --- | --- |
| DEP-1 + DEP-11 + DEP-12(a) | PASS | PASS | not run |
| DEP-1 + DEP-11 + DEP-12(a) + DEP-14 (before the Threads fix) | build failed: `pthread_create` undefined under the conda toolchain | same | not run |
| DEP-1 + DEP-11 + DEP-12(a) + DEP-14 (final) | PASS | PASS | running |

PASS means no new failures or slow cases against the stored expected-failure
files and the `base548` results; gz-physics detected `ContactSurface.hpp` in
every run, and the only remaining entries are the accepted
`UnsupportedRayIntersections` difference and the external
`INTEGRATION_reset_sensors` defect (reported `BASE`).
