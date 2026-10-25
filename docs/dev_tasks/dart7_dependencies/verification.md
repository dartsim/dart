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

Status (2026-10-24): the [wave-7 spikes](#wave-7-2026-10-24) partly cover
gate 1 (Linux GCC passes; Linux Clang fails only in two test files, one fixed
by #3936), gates 4 and 6, and parts of 2, 3 and 5. Gate 3
keeps its intent (no ABI difference a C++17 consumer can see), checked by the
scripted checks the ABI spike lists, because `abidiff` itself always reports
harmless weak-symbol and libstdc++ base-class differences.

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
- Gazebo lanes: PASS on Harmonic, Ionic and Jetty in the Wave 1 integration
  tree (below).

### DEP-11 PRs 1-2: C++20-clean code

- Writer: Codex. Committed locally on `refactor/cxx20-clean-code`. Its
  second commit (explicit `this` captures in `InverseKinematics.cpp` and
  `optimizer/Function.cpp`) was dropped: open PR #3804 from an outside
  contributor makes the identical change and also covers the two
  `World.cpp` lambdas.
- Gates: default build and 199/199 tests; a C++20 build passes 199 C++ and
  1,641 Python tests; all 12,852 exported dynamic symbols unchanged; C++17 and
  C++20 header checks and a C++14 ODR-use probe pass; the only remaining C++20
  diagnostics are the two `World.cpp` captures (quiet window).
- Reviews: Claude (nothing substantive) and Muse (no blockers; follow-up:
  about 30 sibling property structs share the use-triggered deprecated-copy
  pattern).
- Gazebo lanes: PASS on Harmonic, Ionic and Jetty in the Wave 1 integration
  tree (below).

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
- Gazebo lanes: PASS on Harmonic, Ionic and Jetty in the Wave 1 integration
  tree (below).

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
- Gazebo lanes: PASS on Harmonic, Ionic and Jetty in the Wave 1 integration
  tree (below).
- Owed: the migration note (the log format, the narrower `DART_*` argument
  set, `Threads` in the package config, fmt no longer found through
  `find_package(DART)`, and a `setLogSink` recipe for spdlog users). Muse's
  two cosmetic nits (an rvalue-string constructor, one `FMT_HEADER_ONLY`
  definition site) move to DEP-15, which reworks the same files.
- CI on #3853 (2026-10-19): macOS arm64 failed in `test_FilamentBackend`
  (`-std=gnu++20`) because libc++ builds the object inside `construct_at`,
  so AppleClang reports the deprecated Assimp `MeshShape` constructor outside
  the call site's suppression. The fix calls `new` directly at 17 test and
  binding sites (writer Codex; reviews Muse and the orchestrator). Muse's
  claim that four parser sites also fail was refuted by the same CI log: the
  `dart-utils` parsers compiled as C++17 before the failure. After merging
  `main`, eight affected test targets pass.
- Perf A/B on #3853: every row is behaviour-change because removing spdlog
  changes the benchmark compile commands. `pend/dart` rose 2.56% and `dyn`
  0.95% Ir under GCC 13.3; a local reproduction attributes nearly all of it
  to Eigen's `allFinite()` reduction, which GCC 13 no longer inlines into
  `GenericJoint<R1>`'s articulated-inertia updates (208 instructions per
  call; 9,152 of `pend/dart`'s 9,173 extra instructions per step). GCC 15.2
  on the same revision pair (`d46dc795e` to `3819534246`) measures -0.06%
  and -0.07%. All default-environment package resolutions match across the
  CI revisions.
  The PR body carries `Rebaseline-Rationale` lines; the fix
  (`array().isFinite().all()`) touches `GenericJoint` and waits for M1d.

### Wave 1 Gazebo integration lanes (2026-10-19)

Each packet branch was applied to an exported `548ff24ad` tree (Codex and
Claude review the packets separately; the lanes judge their combination) and
run as an unpatched lane variant against `base548`, using DEP-0's `lane.sh` and
gate script.

| Tree | Harmonic | Jetty | Ionic |
| --- | --- | --- | --- |
| DEP-1 + DEP-11 + DEP-12(a) | PASS | PASS | not run |
| DEP-1 + DEP-11 + DEP-12(a) + DEP-14 (before the Threads fix) | build failed: `pthread_create` undefined under the conda toolchain | same | not run |
| DEP-1 + DEP-11 + DEP-12(a) + DEP-14 (final) | PASS | PASS | PASS |

PASS means no new failures or slow cases against the stored expected-failure
files and the `base548` results; gz-physics detected `ContactSurface.hpp` in
every run, and the only remaining entries are the accepted
`UnsupportedRayIntersections` difference and the external
`INTEGRATION_reset_sensors` defect (reported `BASE`). On Ionic,
`INTEGRATION_entity_system`'s `PublishCmd` passed although the Ionic
expected-failure file lists it (reported `FIXED`); the entry stays.

### Wave 1 integration branch (2026-10-19)

The five packet branches were rebased onto `origin/main` `0d7f6d77a` without
conflicts and merged into a local integration branch. The merge conflicts in
`ci_freebsd.yml`, `CHANGELOG.md` and `dart/CMakeLists.txt` resolve to DEP-14's
side without Boost. Gates: `pixi install` (lock consistent), lint, check-lint,
build, 186/186 C++ tests, 2,028 Python tests passed (121 skipped, 54 xfailed),
and `pixi run test-package-core`. The Gazebo lanes above ran on the
`548ff24ad` base; the packet diffs did not change in the rebase.

### Wave 2 (2026-10-19)

Decision-free packets from the balance review's order that need no unmerged
Wave 1 branch. Each ran in its own worktree on `origin/main` `0d7f6d77a`
(DEP-27a on top of DEP-28) and is committed locally with two clean reviews
from engines other than its writer; fixes were re-reviewed on the final
state.

| Packet | Branch | Writer | Reviews | Gates |
| --- | --- | --- | --- | --- |
| DEP-28 | `refactor/delete-dead-files` | Muse | Codex, orchestrator | build is a no-op, installed file set identical, 186/186 |
| DEP-20(e) | `build/enforce-filament-pin` | Muse | orchestrator, Codex | configure with the 1.77.3 SDK, a material-version-78 SDK fails configure, 251 requirement tests |
| DEP-20(d) | `fix/gui-texture-decode-limits` | Codex | Muse and orchestrator, two rounds | 10 decoder tests (the valid-image cases also pass against the old decoders), 186/186, 2,024 Python, GUI gate |
| DEP-31 | `fix/gui-empty-overlay-font` | Muse | orchestrator, Codex | 20/20 GuiDescriptors (the new test crashes without the fix), 186/186, GUI gate |
| DEP-17 PR2 | `refactor/urdf-glue-cleanup` | Codex | Claude, Muse, then Muse and orchestrator | 33/33 DartLoader, the malformed-file regression under ASan, 186/186, 2,024 Python |
| DEP-32 | `fix/voxel-ray-key-limit` | Codex | Claude, Muse, then Muse and orchestrator | 7 voxel tests also under ASan (they crash or hang without the fix), 186/186, 2,024 Python |
| DEP-3(a) | `build/bullet-collision-libraries-only` | Muse | orchestrator, Codex | installed `Bullet` interface lists two libraries; libdart resolves 0 symbols from BulletDynamics or BulletSoftBody; 186/186, 2,024 Python |
| DEP-27a | `build/third-party-notices` | Codex | Claude and Muse, three rounds | 93 notice tests, 341 requirement tests, LICENSE and NOTICE install |
| DEP-13 PR1 | `build/assimp-5.3-floor` | Codex | Claude and Muse, then Muse and orchestrator | 9 MeshShape tests (the gz-physics-style custom mesh leaked 5,208 bytes before), ASan/LSan mesh tests, 186/186, 2,024 Python, 252 requirement tests, gz-physics and gz-sim build and test |

- DEP-28: Python sdists, which glob every file under `dart/`, also drop
  the deleted files (accepted as intended).
- DEP-20(e): the SDK exposes no release version, so configure checks the
  material format version (77 for every 1.77.x release; 1.78.0 ships 78).
- DEP-32: its first run stopped before the final report; the orchestrator
  verified its logs and ran lint. Valid rays reach OctoMap unchanged in all
  three overloads. It is a release-6.20 backport candidate (needs approval).
- DEP-17 PR2: gz-physics includes `dart/utils/urdf/urdf.hpp` and uses only
  `DartLoader`; no gz-physics or gz-sim code uses the
  `URDFDOM_HEADERS_VERSION_*` macros.
- DEP-27a: 16 entries with pinned sources, verified against upstream.
  Outside the packet: the provenance of assets without copyright
  declarations (Atlas, WAM, kima and Fetch meshes) and whether the
  Roboti-derived MuJoCo models under `data/mjcf/openai/` ship (DEP-33,
  maintainer); libccd attribution comments in the MPR sources after M1d; a
  test that also checks entry contents.
- DEP-13 PR1: upstream assimp 5.3.1 and Ubuntu 24.04's package report
  5.3.0, so the floor is 5.3.0 (as 5.2.0 was for Ubuntu 22.04's 5.2.2); the
  v5.3.0 tag contains every API the removed code guarded. A raw scene
  assigned to `MeshShape::mMesh` is owned again, as in DART 6; gz-physics'
  custom mesh shape relies on that and frees it nowhere. It lands after
  DEP-12(a), which moves the documented OS baseline to Ubuntu 24.04.
- Found and parked (pre-existing, unrelated to dependencies): under LSan,
  DART's Bullet detector and its filter group leak about 4.4 MB in 35
  allocations through an ownership cycle.

| Tree on `0d7f6d77a` | Harmonic | Jetty | Ionic |
| --- | --- | --- | --- |
| base (`main`) | only `reset_sensors` | only `reset_sensors` | only `reset_sensors` |
| Wave 2 without DEP-13 | PASS | PASS | PASS |
| Wave 2 with DEP-13 PR1 | PASS | PASS | PASS |

PASS means no new failures or slow cases against the expected-failure files
and the base results; gz-physics detected `ContactSurface.hpp` in every run.
The local integration branch (all Wave 2 packets but DEP-13) passes
`pixi install`, lint, check-lint, build, 186/186 C++ tests, 2,024 Python
tests, `test-package-core`, 344 build-requirement tests and the notices
tests.

### DEP-15: fmt replaced by a standard-library formatter (2026-10-19)

- Writer: Codex. Committed locally on `refactor/remove-fmt`, stacked on
  DEP-14's `refactor/dart-log-sink` (the Q1 default, DD-4).
- Gates: `pixi install` (lock consistent), build, 24 logging tests (a table
  of every argument type and grammar form), 63 XmlHelpers tests, 186/186,
  2,028 Python tests, package consumers, 160 dependency tests; `nm -DC`
  shows no `fmt::` or spdlog symbols in libdart or libdart-io; all 92 in-tree
  `DART_*` call sites use the supported grammar.
- Differential test against fmt 11.2 through the log sink (1,039,336 values:
  random double and float bit patterns, human-scale values, specials, every
  supported form, surplus arguments): the first version printed float `{}`
  in [1e7, 1e16) in fixed notation, where fmt uses exponent notation
  (85,390 mismatches, all of that kind; fmt's threshold is
  min(16, digits10 + 1)). After the fix: 0 mismatches. Both round-1
  reviewers had judged the threshold correct; the differential test caught
  it.
- Reviews: round 1 (Claude, Muse) led to the float fix, a precision cap of
  1000, checked conversions, surplus arguments ignored as fmt does, explicit
  `nan`/`inf` spelling, and a macOS availability define scoped to conda or
  pixi compilers (CONDA_PREFIX or BUILD_PREFIX) with targets below 13.3.
  Rounds 2 and 3 (Claude, Muse): no blockers or majors; Muse's round 3 had
  no findings. Floating-point `std::to_chars` needs GCC 11 or libc++ 14, so
  DEP-15 lands after DEP-12(a)'s floors.
- Gazebo lanes on DEP-14 + DEP-15 against the `0d7f6d77a` base: Harmonic
  and Ionic PASS. Jetty flagged gz-physics' `StepWorld` once at 15.6 s
  (limit 6.1 s); the same build took 2.2-2.5 s in three solo reruns and
  2.6-3.5 s on the other lanes, so it was load, not a regression.

### Spike: the distro FCL assertion in `test_Distance` (2026-10-19)

- Trigger: `Distance.testBasicInterface` and `Distance.Options` query the
  distance between two ellipsoids (radii 0.5 and 0.25, centers at the origin
  and (0.75, 0, 0)) that touch exactly at (0.5, 0, 0). FCL 0.7.0's GJK
  (libccd path) takes an initial Minkowski support point that is already the
  origin, then asks for support in a zero direction and gets NaN
  (`gjk_libccd-inl.h`, initialization and the ellipsoid support function).
  Ellipsoid-sphere pairs and non-spherical ellipsoids reproduce it; the
  input order matters; the collision query reports no contact at that exact
  tangency.
- Ubuntu's `libfcl0.7` 0.7.0-3build2 keeps assertions, so the process
  aborts in `doSimplex2` ("A is not in region 1"); defining `NDEBUG` in
  DART does not change the prebuilt library. With assertions compiled out
  (as conda-forge builds FCL), FCL returns its unsigned sentinel distance
  -1 and nearest points (0, 0, 0) for both shapes; DART clamps the distance
  to 0, which is correct, but the nearest points are wrong (the contact is
  (0.5, 0, 0)). DART's tests pass there only because they check that the
  points are finite.
- Upstream: FCL issue #366 (open) covers related GJK defects; PR #367
  (merged before 0.7.0) does not fix this case, and the reproducer still
  aborts with FCL master's GJK source (`708d2e10`). No Debian or Ubuntu
  report matches.
- Exposure: FCL-backed distance queries between ellipsoids or spheres at
  exact tangency, from C++ and from dartpy, on stock Ubuntu 24.04.
- Superseded recommendation (historical, kept with its reproducer): fix
  upstream in FCL with a regression test, link Debian and Ubuntu reports, and
  document the stock-noble limitation; keep DART's ellipsoid coverage (no
  jitter or skip). **DD-18 supersedes the reporting part:** the maintainer
  said not to report, but to fix it internally with a clear note and patch
  file (DEP-35). The exact internal patch policy is unchanged: DART carries
  the patch where it builds FCL itself, system FCL stays unpatched, and the
  noble apt lane (DEP-12(b)) records `test_Distance` as an expected external
  failure pointing at the note. No upstream report or external message is
  authorized. The opt-in `MESH` primitive mode gives correct results but
  changes approximation and speed, so it is not a default change.

### Wave 3 (2026-10-19)

> **Unverified historical notes (Waves 3 to 5 and the tinyxml2 prototype
> below).** The quantitative assertions in these sections (test counts,
> byte counts, hashes, "N of N" results) were summarized from PR bodies and
> reviewer reports. This file does not record the tested revision or exact
> command for them, and the PR heads they cite moved. They are not accepted
> evidence and no acceptance decision rests on them. A PR's acceptance
> evidence is its own CI and review state at its merged head, recorded by
> the PR itself. A claim returns to accepted evidence only after it is
> re-run and recorded with its revision, command, date and scope.

Each packet below passed two clean reviews from engines other than its
writer; the PR bodies carry the user-facing evidence.

- DEP-37 (#3875, Codex; Muse and orchestrator): the new
  `ReleasesRetainedFilterGroup` test leaked 13,242,765 bytes under ASan
  before the fix; after it, the randomized convex case is leak-free;
  186/186, gz-physics 203 tests, gz-sim integration.
- DEP-30 (#3876, Claude Sonnet; Codex and Muse, two rounds): round 1 found
  that archive timestamps could keep objects compiled from the patched Git
  checkout; the fix extracts with current timestamps and moves ImGui to
  FetchContent's default source directory. Hash re-verified independently;
  `nm` shows `ImFormatString` exported again; test-gui 496; 184/184.
- DEP-36 (#3877, Codex; Claude and Muse, three rounds): semantics checked
  against MuJoCo's `user_objects.cc`; 32 MJCF tests, 55 Python MJCF tests,
  186/186, 2,078 Python tests.
- DEP-33(b) (#3878, Codex; Claude and Muse): the orchestrator's capture
  review sent the arm back for an upright zero pose and rounded meshes;
  oracles: target error about 1e-17 m, four dominoes toppled, point-cloud
  height 0.49 m.
- DEP-25 (#3879, Codex; Claude and Muse, two rounds): installed Tracy
  packages carry no version file or `TracyVersion.hpp`, so the system lookup
  now takes no version (the profile environment configures and builds
  against its own Tracy); default cache unchanged except the new option;
  installed CMake files byte-identical.
- DEP-33(a) (#3880, Claude Sonnet; Codex and Muse): the COLLADA fixture is
  schema-valid and its absolute extents catch ignored units and unwanted
  up-axis conversion.
- DEP-34 (#3881, Codex; Claude and Muse): replaces the 14 top-level OpenAI
  models (the six Fetch scenes are DD-19's separate removal scope); all 14
  pinned DeepMind files
  hash-checked against upstream at `437b7a6a`; expectations re-derived from
  the XML by both reviewers; the weld regression test crashed (SIGSEGV)
  without its fix.
- DEP-33(c) (#3882, Codex; Claude, Muse and Claude Sonnet after a Muse run
  hung): both licenses byte-identical to their pins; WAM's STLs are the
  original visual meshes; captures before and after inspected.
- CI after these pushes: the macOS and Windows Python jobs fail only on
  `test_api_surface_budget` (`dartpy.experimental` imported earlier), on
  every PR that merged current `main`; open PR #3872 fixes it.

### Wave 4 (2026-10-20)

- DEP-2 (#3883, Claude Sonnet; Codex and Muse): a shared assets-on
  install's `io;utils;utils-urdf` consumer configures with curl and OpenSSL
  discovery disabled; a static configure still exports both; gz-physics 199.
- DEP-17 PR1 (#3884, Codex; Claude and Muse): Ubuntu 24.04's
  `liburdfdom-dev` 4.0.0 ships `urdfdom-configVersion.cmake` (container
  check), so the floor works with the distribution package;
  `pixi install --locked` still passes.
- DEP-8(f) (#3885, Claude Sonnet; Codex and Muse): 17 archive hashes
  reproduced by the image build; 15 trees identical to their clones (assimp
  and GLFW differ only by export-ignored dotfiles);
  `scripts/wheel_linux.py cp312` builds, repairs and smoke-tests the wheel.
- DEP-4(e) (#3886, Claude Sonnet; Codex and Muse, two rounds): a default
  configure's cache differs from `main` only by the new option and the
  removed Doxygen entries; gz-physics 199 with the switch on.
- DEP-17 PR3 (#3887, Codex; Claude and Muse): the PR body's 14-row table
  (11 files under `data/urdf/` plus the 3 Atlas files under `data/sdf/atlas/`)
  reports the same load results and skeleton, body and joint counts with
  urdfdom 4.0.1 and 6.0.1: `atlas_v3.urdf` is rejected by both (its `head`
  link is missing; see [RESUME.md](RESUME.md)), while `atlas_v3_no_head.urdf`
  and `ground.urdf` load with both; URDF 1.1 and 1.2 load only with 6.0.1; the
  wheel builds, repairs and smoke-tests on Python 3.12 and 3.13. This is
  quarantined historical reported evidence: the mutable PR body records no
  tested commit, command or date. The current tree holds 14 files under
  `data/urdf/` plus the 3 Atlas files (17 paths), which is a different set from
  the 14 table rows (the missing-entity and missing-include fixtures are not in
  the table), and no new run is claimed.
- DEP-17 PR4 (#3888, Codex; Claude and Muse): a shared consumer configures,
  builds and runs with urdfdom package discovery disabled (the runtime
  library remains required); 450 installed headers carry no urdfdom include;
  the API inventory is unchanged.

### Wave 5 (2026-10-20)

- DEP-18 (#3889, Codex; Claude Opus and Muse, two rounds): an SDF-off build
  runs 2,073 Python tests, skipping SDF tests one by one; SDF-on builds keep
  the same 21 `SdfParser` exports; 442 installed headers compile
  standalone; a shared `io` consumer configures without sdformat, a static
  one requires it, and `COMPONENTS io-sdf` fails cleanly on an SDF-off
  install; gz lanes.
- DEP-7 (#3890, Muse; Codex and Claude Sonnet): the default environment
  shrinks from 1.9 GB to 1.6 GB; every task resolves to one environment; the
  AI infrastructure suite (5,085) and `test-all` pass, apart from #3872's
  known failure.
- DEP-5 remainder (#3891, Codex; Claude Sonnet and Muse): 186 of 186 tests
  on Eigen 3.4.0 and on 5.0.1; the overalignment suite (175); a consumer
  built against the other Eigen major fails at configure with the
  installed finder's message; 253 requirement tests; gz lanes.
- DEP-9 (#3892, Codex; Claude Sonnet and Muse): a Python 3.11 configure
  fails clearly; the abi3 smoke test passes on Python 3.12, 3.13 and 3.14;
  NumPy 1.26.4 passes on 3.12; 244 requirement tests.
- DEP-6 (#3893, Codex; Claude Sonnet and Muse): fetched GoogleTest 1.17.0
  and Benchmark 1.10.0 pass 176 of 176 tests, and so do Ubuntu 24.04's
  GoogleTest 1.14 and Benchmark 1.8.3 in a container; a configure on a host
  without Git works.
- DEP-10 (#3894, Codex; Claude Sonnet and Muse): a system nanobind 3.2.0
  fails at configure with the new message; fetched and conda-forge 3.1.0
  builds both pass; 24 configure cases and 269 requirement tests.
- DEP-4(c) (orchestrator): `GCC_VERSION` is never set, so the deleted
  branch never ran; with `DART_ENABLE_SIMD=ON`, `build.ninja` and every
  generated export file are byte-identical before and after.
- DEP-12(b) tinyxml2 (#3896, Codex; two Claude Sonnet passes): both lookup
  paths reject a tinyxml2 9.0.0 package with the minimum-version message
  and accept 10.0.0; 186 of 186; 239 requirement tests.
- Perf harness (#3897, Codex; two Claude Sonnet passes): the unmodified
  harness reproduces #3889's driver link failure (`libsdformat.so.16 ...
  not found`); with the driver linked through an `rpath-link` path, an A/B
  from `main` to #3889's head builds both drivers and measures every row with
  equal instruction counts, allocations and guards; compile commands and
  workload hashes are unchanged.
- DEP-4(a) (#3898, Claude Sonnet; two Codex passes plus two confirmations):
  the first reviews showed the octomap target shim is reachable (octomap's
  build-tree config imports no targets), so it stays; 186 of 186 with
  octomap 1.10.1; the wheel builds, repairs and smoke-tests.
- DEP-16 (#3899, Codex; Claude Opus and Sonnet, five rounds): default
  console output is byte-identical to `main` in 12 terminal and 12 GUI-App
  cases; a ThreadSanitizer build of the logging tests passes 55 of 55; both
  teardown orders of the GUI panel and the Python bridge end on the default
  sink; 186 of 186, 2,130 Python and 496 GUI Python tests; gz-physics 199
  plus 4 and gz-sim 1 of 1.

### tinyxml2 fetch prototype (2026-10-20)

The same unverified-historical-notes caveat applies; the prototype branch is
local and unpublished, and this section records no revision for it.

Codex fetched tinyxml2 11.0.0 as a shared library installed next to DART,
with an ELF check that urdfdom's and sdformat's tinyxml2 has the same SONAME:
176 of 176 tests in both fetched configurations and a working installed
consumer, for 410 added lines (178 in two new CMake modules). Two reviews
(Claude Opus and Sonnet) showed the fetch can only pass where a system
tinyxml2 of the same ABI is already loaded, because `dart-io` requires
urdfdom (and sdformat with SDF on), and both link a shared tinyxml2. As the
standalone default it would fail on Ubuntu 24.04 (apt urdfdom 4 and OSRF
sdformat 14 link tinyxml2 10) and on macOS, Windows, FreeBSD and cross
builds. The branch stays local; the maintainer decides how DD-10 applies.

### Wave 6 and the fetch lanes (2026-10-21 to 2026-10-24)

Each PR's two clean reviews came from engines other than its writer; Muse
was out of quota, so Codex reviewed Claude's packets.

- DEP-4(b) (#3900, #3901): fetched ODE and Bullet build, install, configure
  an installed consumer and uninstall in a nightly lane; the uninstall
  template reads the manifest as raw bytes and splits only on newlines, after
  a review showed non-ASCII names could delete files outside it.
- Installed DART finds its fetched Bullet (#3929): the nightly fetch lane
  failed because an installed DART did not record the Bullet it fetched.
  Three review rounds closed a partial cache hit that mixed recorded and
  consumer libraries, Windows Debug names, a consumer's earlier
  `FindBullet`, and Debug-only installs; 420 Bullet configure tests and the
  lane replay (175 of 175) pass.
- DEP-19's Linux wheels (#3931): the wheel is 21,303,502 bytes, 2,649,828
  more than `main`'s, under a 23,000,000-byte budget; six TLS checks
  (untrusted, wrong host and missing CA fail; explicit CA and the system
  fallback succeed) pass in the smoke test, and on Ubuntu 24.04 and Debian
  12. A first review measured conda-forge's libcurl closure on macOS and
  Windows (about 18.2 MB and 16.5 MB compressed), which is why those wheels
  wait on the maintainer.
- libcurl 8.5.0 (#3922) and OpenSSL 3.0.13 (#3923): a bot review found the
  OpenSSL configure tests failed on CMake 3.28 (FindOpenSSL calls
  `pkg_check_modules`, which the tests disable); a no-op stub fixed it, and
  the dependency suite passes 718 of 718 on CMake 3.28.3.
- DEP-13(d) (#3924): gz-style raw-`aiScene` and TriMesh meshes collide, and
  each ownership mode passes under ASan and LSan, after fixing a double
  release in `MeshShape.cpp`. Its test then failed Clang `-Werror` builds;
  #3936 fixes that (below).
- DEP-11 PR3 (#3925): scoped suppressions at the octomap and FCL includes
  (`-Wcpp`, `-W#warnings`) and the ODE includes (enum-enum conversion); the
  Gazebo lanes pass.
- DEP-33(e) (#3921): `check_urdf` confirmed `atlas_v3.urdf` never loaded.

### Wave 7 (2026-10-24)

Claude wrote every packet in isolated worktrees on `main` (`95ee796a5`,
`c98e4591f` or `38b1484eb`); each was then rebased onto `765e26661` and
reviewed twice by Codex.

- Tier 1 apt lane (#3937): the job's steps, run in a fresh `ubuntu:24.04`
  container, pass cold in 795 s (build 679 s at `-j 4`, no warnings, 175 of
  175 tests) and warm in 121 s (448 of 448 ccache hits). Versions: GCC
  13.3.0, CMake 3.28.3, FCL 0.7.0, octomap 1.9.7, assimp 5.3.1, tinyxml2
  10.0.0, urdfdom 4.0.0, Bullet 3.24 (double precision detected), ODE
  0.16.2, GoogleTest 1.14.0, Benchmark 1.8.3, curl 8.5.0, OpenSSL 3.0.13,
  sdformat 14.9.0. `test_Distance` is excluded: two of its cases abort on the
  distro FCL's libccd assertion (see the spike above). The first hosted run,
  on #3937, passed 176 of 176 in about 16 minutes on `ubuntu-24.04`.
- DEP-11 PR4 (#3943): 442 installed headers compile alone at C++17 and at
  C++20 with `-Werror` (884 translation units) on GCC 15; the same consumer
  passes on GCC 13, Clang 18 and Clang 21. Two headers needed an include.
  Cold: about 14 CPU-minutes with GCC; warm: 5 s.
- DEP-13(c) (#3938): a consumer that calls each deprecated API gets exactly
  four warnings on GCC 15 and Clang 18, DART's own build gets none, and the
  gz lane (gz-physics 8 and gz-sim 9) passes; the API inventory check
  passes. DART's FCL and Bullet soft-mesh paths keep the `aiMesh`, so
  soft-contact results stay bit-identical.
- DEP-22 CMake cap (#3941): CMake 4.4.4 exposed CMP0219 warnings in
  `DARTMacros.cmake` and four Bullet configure-test failures (CMP0200: since
  4.4, `try_compile` imports targets with the policy NEW, which can select
  other library files than the caller links). A version-keyed workaround in
  the Bullet precision probe failed review, because each imported target
  records its own CMP0200 setting, so the cap moves to `<4.4` (lock at 4.3.4)
  and keeps the CMP0219 fix. On 4.3.4 the build, 185 of 185 tests and 967
  requirement tests pass.
- DEP-13(c) (#3938), after rebasing onto `main`: #3926's new face-count check
  in `FCLCollisionObject.cpp` called the deprecated `getAssimpMesh()` outside
  the suppression, and `getTriMesh()` refreshed shared transform caches from a
  const reader; both fixed, 185 of 185.
- DEP-22 Tracy (#3939): a fetched 0.14.1 client builds and records zones that
  0.14.1's `tracy-capture` reads; DART's old `ScopedZone` call bound `true`
  to 0.14's callstack-depth parameter, so the call now passes it explicitly.
- DEP-20(b) (#3942): the fetched GLFW build passes the same GUI test counts as
  the system-GLFW build, except two native-window tests that hand the test's
  own GLFW windows to `dart-gui` and skip with a stated reason in shared
  fetched builds; a relocated installed consumer passes. Review found that a
  GLFW private to a shared `dart-gui` on macOS would register the same
  Objective-C classes as any other GLFW in the process, so shared macOS
  builds keep the system GLFW.
- **C++20 warning budget spike** (no PR; base `38b1484eb`). With
  `-Wall -Wextra`, `dart`, `dart-io`, `dart-gui` and dartpy have zero
  warnings as C++20 on GCC 13.4, 14.4, 15.2 and 16.2 and Clang 18.1.8,
  21.1.8 and 22.1.8. DEP-21's per-target design (default 17; the four
  targets at 20; `-Werror` on) builds 868 of 868 steps on GCC 13 and 15. On
  Clang it fails only in `test_MeshShape.cpp` (fixed by #3936) and in
  `bm_simd.cpp`'s unused `kMeshMax`, both already at C++17. A global
  `CMAKE_CXX_STANDARD=20` is not viable: fetched ODE emits 1,050 warnings,
  and fetched GoogleTest fails Clang 21's `-Wcharacter-conversion`.
- **ABI spike** (no PR; GCC 15.2, libstdc++ 15). Built as C++20 with a
  C++17 interface, `libdart` and `libdart-io` keep every `STB_GLOBAL`
  symbol (5,198 and 445); `-fdump-lang-class` over all 296 installed headers
  shows no size, alignment, offset or vtable difference. A C++17
  `tools/package` consumer, DART's 176 tests built at C++17 and two
  determinism drivers (bit-identical checkpoints) all pass against the C++20
  libraries; `abicompat` accepts the gz-physics 8 plugin. `abidiff` still
  exits 12, on weak template instantiations and on libstdc++'s
  dialect-dependent base classes of `std::atomic<double>` and
  `std::optional`, so gate 3's "no exported-symbol or layout difference"
  cannot pass as written. The spike also found a hazard `abidiff` cannot
  see: `DART_STRING_TYPE` defines a function-local `static const
  std::string` that C++20 constant-initializes and C++17 initializes behind
  a guard. With a C++17 executable on C++20 libraries, the string is lost
  when library code reads it first (reproduced on `CAllocator`). Fix it
  before DEP-21, and replace gate 3 with scripted checks: identical
  exported global symbols, no class size or alignment change in the
  installed-header closure, no exported object moving between sections, and
  `abidiff` as an informational report.

### Wave 8 (2026-10-24 to 2026-10-25)

Six wave-8 packets merged, and the DEP-20(a) spike reported. Claude wrote
the packets in isolated worktrees, except DEP-3(b), which Codex wrote. Each
passed two reviews from the engine that did not write it, then the PR
Lifecycle gate (DD-35).

- DEP-3(b) (#3952): shared `dart` links Bullet and ODE privately, and their
  usage requirements moved to the collision components. Round 1 found five
  issues: the fetched-prefix RUNPATH, static builds losing PUBLIC scope,
  fetched ODE's rpath-link, a precision `#error`, and changelog wording.
  Round 2 was clean in both reviews. `pixi run -e gazebo test-gz` passed 204
  of 204; the packet's writer did not run the unpatched Gazebo
  compatibility lanes locally.
  The perf gate saw Ir change by 0.000% everywhere. It failed only on the
  dyn and lcp `input_sha`. The workload hash includes the benchmark compile
  commands, so the PR body carries a `Rebaseline-Rationale` line.
- `DART_STRING_TYPE` (#3953), a DEP-21 prerequisite: the macro now binds its
  function-local static to a heap string, so both dialects initialize it on
  first use. Before the fix, three `libdart` objects moved section between
  C++17 and C++20 builds; after it, none do. A scan of 501 headers alone in
  both dialects found no other case. `UNIT_common_StringType` fails on the
  old macro and passes on the new one.
- DEP-23 (#3954): `building.md` gained a "Packaging DART 7" section. Its
  research found two packaging hazards. A GUI configure against an SDK
  without Filament's notices downloads them even when
  `FETCHCONTENT_FULLY_DISCONNECTED` is on. OSRF's noble packaging sets only
  `DART_USE_SYSTEM_GOOGLETEST`, so it would stop at Benchmark.
- DEP-8(b) (#3955): `auditwheel repair --strip` saves 1,021,842 B (4.8%)
  compressed and 5,936,768 B (7.4%) unpacked on x86_64. The script rejects
  auditwheel older than 6.8.0, reading only the first line of its wrapped
  `--version` output.
- DEP-8(e) (#3957): the wheel's assimp 6.0.5 keeps four importers: Collada,
  OBJ, STL and glTF 2.0. The x86_64 wheel drops from 21,303,988 B to
  17,683,778 B compressed (17%), measured without `--strip`. The wheel
  smoke test loads a `.dae`, an `.STL` and an `.obj` file.
- DEP-8(c) (#3958): the wheel image builds sdformat 16.1.0. sdformat 15.3
  and 16.1 both implement SDF 1.12, so wheels accept the same SDF versions.
  Loading a 1,000-model world takes 0.54 s and peaks at 185 MB, against
  6.2 to 6.9 s and 4,783 MB before. The wheel grows by 5,314 B.
- Backports to `release-6.20` for 6.20.4:
  - #3947 (#3859, voxel-grid rays) passed 173 of 173 tests and test-gz.
  - #3948 (#3887, urdfdom 6.0.1 wheels) matches PyPI 6.20.3's load and
    reject results on all 15 repository URDF files.
  - With #3948, files that declare URDF 1.1 or 1.2 load unless they use
    geometry DART lacks, such as capsules.
- **DEP-20(a) spike** (no PR; base `0821e694b`): stock Ubuntu 24.04 builds
  and tests `dart-gui` and dartpy with DART's fallbacks and apt packages.
  The private Filament 1.77.3 SDK builds from source with apt's clang-20.
  - Results: 185 of 186 C++ tests passed; only `test_Distance` failed, the
    known distro FCL abort. All 11 GUI tests passed on Lavapipe, and all 5
    native-window tests under Xvfb, where two FilamentNative cases skip by
    design. dartpy passed 2,361 tests, with 126 skipped and 52 expected
    failures.
  - Undocumented prerequisites: `libgl-dev` is a build blocker (Filament
    includes `GL/glx.h`), and one test needs `fonts-dejavu-core`.
  - Configure reports a missing Clang 20 only after the 828 MB source
    download. It reports the missing `GL/glx.h` only after about 15
    CPU-minutes of SDK build.
  - On 16 CPUs, configure took 163 s, including 60 s of download, and the
    build took 233 s. A 4-vCPU nightly lane would take an estimated 35 to
    45 minutes.
  - Apt names Lavapipe's ICD `lvp_icd.json`, not the conda name in DART's
    docs. `test_compare_soft_body_detector_pairs` fails outside a git
    checkout, and its runner leaves a lock-file handle open; the resulting
    warning fails `test_evidence_publish`.
