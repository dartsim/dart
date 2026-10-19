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

None yet; each packet records its gates and review passes here.
