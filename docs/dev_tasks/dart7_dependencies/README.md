# DART 7 dependency floor and minimization

This project home tracks the effort, directed by the maintainer on
2026-10-18, to make DART 7 depend on as little as possible and to raise its
platform floor to Ubuntu 24.04 LTS.
[PLAN-625](../../plans/dashboard.md#plan-625-dart-7-dependency-floor-and-minimization)
tracks its roadmap status. It runs inside the
[DART 7.0 program](../dart7_python_first/README.md) (PLAN-624) and follows
its gates; the FCL and collision-backend lifecycle stays with PLAN-620.

- [Inventory](inventory.md): every dependency, its category, floors and
  footprint.
- [Packets](packets.md): scope, non-goals and gates per packet.
- [Decisions](decisions.md): the maintainer direction, pending questions and
  the defaults taken.
- [Verification](verification.md): spikes, lanes and reviews per packet.
- [Progress log](progress-log.md): completed chunks.
- [Resume](RESUME.md): the next-session handoff.

## North star

DART 7 builds from a short, current and well-understood dependency list.
Every remaining external dependency is one of:

- essential to the math vocabulary (Eigen);
- bound by the C++ API that gz-physics and gz-sim use (FCL, assimp, Bullet,
  ODE, tinyxml2);
- a specialized format library (urdfdom for URDF, sdformat for SDF);
- a GUI or Python necessity (Filament, GLFW, the image codecs, nanobind,
  NumPy).

Third-party libraries that the C++ standard library can replace go (spdlog,
fmt); unused ones go (Boost). A source build fetches what it needs, package
managers opt into system packages, and in-tree bundled copies give way to
either. Ubuntu 24.04 is the oldest supported platform, and DART's own code
builds as C++20 behind a C++17 interface.

This makes DART more complete and faster to evolve: a logging sink the Python
bridge can use, smaller wheels and environments, builds that work without
preinstalled dependencies, and fewer upstreams that set DART's pace.

## Final deliverable and acceptance

The effort is done when all of these hold on `main`:

- **Ubuntu 24.04 floor.** The compiler, CMake, Python, NumPy and library
  floors in [inventory.md](inventory.md#ubuntu-2404-floor) are enforced by
  CMake, documented in the build docs and covered by the requirement tests.
  A stock-noble lane (core and `dart-io` from apt plus OSRF sdformat) is
  green. `release-6.20` keeps its Ubuntu 22.04 floor.
- **Approved removals landed.** Every removal the maintainer approves in
  [decisions.md](decisions.md) has merged, with breaking or default-behavior
  parts merged before the PLAN-624 API freeze `F`.
- **C++ standard.** The decided option is in place with its gates: on the
  recommended option, DART's own targets build as C++20, exported targets
  keep `cxx_std_17`, and installed headers pass C++17 and C++20 dialect gates.
- **Gazebo.** Unmodified gz-physics and gz-sim pass the Harmonic, Ionic and
  Jetty lanes with no new failures against each packet's base, and the lanes
  assert that gz-physics detects `dart/constraint/ContactSurface.hpp`.
- **Packaging.** Every fetched or vendored dependency keeps a
  `DART_USE_SYSTEM_<X>` switch, a disconnected configure works when the
  system packages are present, and the build docs carry a packaging section.
- **Records.** Durable decisions move to `docs/design/` or
  `docs/onboarding/`, the dashboard entry is updated, and this folder is
  removed by the completing PR.

## Scope

- Platform and toolchain floors, CMake floor, Python floor, library floors.
- Dependency removals: Boost (unused), spdlog and fmt (DD-4). libcurl and
  OpenSSL, and SDF packaging with sdformat kept, follow the follow-up study
  ([decisions](decisions.md)).
- Dependency acquisition (DD-8): fetch-first source builds,
  `DART_USE_SYSTEM_<X>` for packagers, and in-tree bundled code replaced by
  fetched or system packages where possible.
- Trimming the dependencies that stay: link interfaces, dead find modules and
  compat code, wheel contents, the default pixi environment.
- The C++20 build, dialect gates and the C++20 library rule.
- Eigen 5 readiness, nanobind and test-tool pins, fetch-fallback repair.
- Packaging guidance and downstream handoff notes.

## Non-goals

- Removing FCL, assimp, Bullet or ODE in 7.0. They are bound by the
  gz-facing API; [DEP-24](#packets) records the 8.0 path.
- Replacing urdfdom or sdformat (DD-5), or changing SDF semantics (#3789 stays).
- Exporting C++20 to consumers in 7.0, and any use of C++23.
- Splitting the GUI out of dartpy, or replacing Filament.
- Fixing gz-sim or gz-physics themselves; upstream proposals are drafted
  only, and each needs approval.

## Constraints

- The [compatibility rules](../../../AGENTS.md#compatibility-rules): the C++
  API that gz-physics and gz-sim use, installed headers and package components
  (including legacy forwarding components) stay compatible; other public-API,
  ABI-layout or default-behavior changes need maintainer approval.
- PLAN-624: step-path hunks (World stepping, the constraint solver including
  `lcpsolver`, the native collision detector and generic-joint dynamics)
  follow that program's merge order, each re-qualified on current `main`;
  API freeze `F`, bit-identical refactors, and the Gazebo lanes for core
  changes still apply.
- D5: no deprecated C++ API is removed during 7.x.
- One dependency or one vendored tree per PR. Three shared mechanical steps
  may touch several dependencies in one PR: the toolchain floors, DEP-25
  (the umbrella `DART_USE_SYSTEM_DEPENDENCIES` switch, the offline fail-fast,
  CMP0170 at every existing fetch site) and DEP-8(f) (hash-checked wheel-image
  sources), with no version or default changes (DD-30).
- Packagers build offline: debhelper and vcpkg force
  `FETCHCONTENT_FULLY_DISCONNECTED`, and poudriere has no network.
- Every empirical claim records its revision, command, date and scope;
  numbers from the research workflow stay "reported" until a repository
  script reproduces them.

## Evidence so far

Recorded in detail in [verification.md](verification.md); all on `main`
`17ce7eefa`, 2026-10-18:

- **Stock Ubuntu 24.04 builds core and `dart-io` today.** In an `ubuntu:24.04`
  container with apt packages (GCC 13.3, CMake 3.28.3) and OSRF sdformat14,
  the build has no warnings and 186 of 187 tests pass at both C++17 and
  C++20. `test_Distance` aborts on an assertion inside Ubuntu's FCL 0.7.0,
  which is built with assertions on. GUI and dartpy cannot configure there
  (no Filament, GLFW 3.3.10, and dartpy requires the GUI).
- **C++20 is close to free for DART itself.** Built and exported as C++20 with
  the full default environment (GUI and dartpy), main has six new warnings,
  all implicit `this` captures in `.cpp` files, and passes 197 of 197 C++
  tests and the Python suite.
- **Gazebo tolerates both C++20 options.** Exporting `cxx_std_20` silently
  compiles part of the gz-physics plugin as C++20 and still shows no new
  Harmonic or Jetty failures against the C++17 base. Building DART as C++20
  while exporting `cxx_std_17` keeps every gz-physics command at C++17 with
  the contact-surface probe passing.
- **`INTEGRATION_reset_sensors` fails on the base too.** Codex traced it to a
  gz-sim test that keeps a component pointer across a world reset
  (heap-use-after-free under ASan, reproducible without DART); it fails the
  same way with DART 6.20.0. It is a downstream test defect, not a DART
  regression.
- **Research workflow.** Eleven dependency dossiers and seven studies, each
  checked by one or two adversarial verifiers, produced the inventory and the
  packets below. The independent Codex audit of the C++ standard reached the
  same recommendation.

## Packets

Owners follow the team-mode routing in `docs/ai/orchestration.md`: Codex for
solver, ABI and design-heavy packets (DEP-14 at ultra effort, because the
logging header sits in gz-physics' probe closure), Claude Sonnet for bounded
build, CI and docs packets, Muse for long mechanical ones. Every packet gets
two clean reviews from engines other than its writer. Gate
shorthand is defined under [Gates](#gates).

| ID | Packet | Needs | Gates | Owner | Target | Status |
| --- | --- | --- | --- | --- | --- | --- |
| DEP-0 | Assert the gz contact-surface probe in the Gazebo lanes | none | gz | Claude Sonnet | 7.0 | Merged (#3849) |
| DEP-1 | Remove Boost from environments, the wheel image and FreeBSD CI | none (DD-7) | base, gz, pkg, wheel image | Claude Sonnet | 7.0 | Draft #3851 |
| DEP-2 | Export CURL and OpenSSL only for static builds | none | base, gz, pkg | Claude Sonnet | 7.0 | Merged (#3883) |
| DEP-3 | Link only BulletCollision and LinearMath; move Bullet/ODE usage requirements to their components | none (DD-7) | base, gz, pkg, wheel | Muse (a) | 7.0 | (a) merged (#3864) |
| DEP-4 | Find-module hygiene; repair the ODE and Bullet fetch fallbacks under CMake 4 | none (DD-7) | base, gz, pkg, dis, BSD | Claude Sonnet ((a), (e)), Codex high ((b)), orchestrator ((c)) | 7.0 | (e) ready #3886, (c) merged (#3895), (a) draft #3898; (b) in review |
| DEP-5 | Eigen 5 readiness and the `[3.4.0, 6)` range | none (DD-7) | base on both Eigen majors, gz, py | Codex high | 7.0 | Ready #3891; #3836 fixed the deprecated calls |
| DEP-6 | Current fetched GoogleTest and Benchmark plus a fetch lane | none | base, apt | Codex high | 7.0 | Ready #3893 |
| DEP-7 | Slim the default pixi environment | none | base, ai | Muse | 7.0 | Merged (#3890) |
| DEP-8 | Wheel hygiene (one concern per PR) | none (DD-7) | wheel | Claude Sonnet ((f)) | 7.0 | (f) merged (#3885); (a) already done by #3847 |
| DEP-9 | Python 3.12 and NumPy 1.26.4 floors; one wheel per platform | none (D20) | base, py, wheel | Codex high | 7.0 | Ready #3892 |
| DEP-10 | nanobind 3.1.x checked at configure | none | base, py | Codex high | 7.0 | Draft #3894 (stacked on #3892) |
| DEP-11 | C++20-clean tree and installed-header dialect gates (4 PRs) | none | base, gz, C++20 Werror builds | Codex high + Claude Sonnet | 7.0 | PR1 merged (#3852); the captures are in #3804 |
| DEP-12 | Ubuntu 24.04 floor: (a) toolchain, CMake 3.28, CI; (b) library floors | none (DD-1) | base, gz, pkg, apt, dis, BSD | Orchestrator ((a)), Codex high ((b)) | 7.0 | (a) draft #3850; (b) tinyxml2 ready #3896 (stacked on #3850) |
| DEP-13 | assimp 5.3.1 floor and a narrower assimp role | none (DD-7) | base, gz, gui, apt | Codex high | 7.0 | PR1 merged (#3865) |
| DEP-14 | Compiled logging boundary and DART log sink; drop spdlog | DD-4 | base, gz, pkg, TSan | Codex ultra + orchestrator | 7.0, before F | Draft #3853 |
| DEP-15 | Standard-library formatter; drop fmt | DD-4 | base, gz, pkg, wheel, BSD | Codex high | 7.0 | Draft #3854 (stacked on #3853; Q1 default) |
| DEP-16 | Native diagnostics and the dartpy logging bridge through the sink | DD-4 | base, py, gui, gz | Codex ultra + orchestrator | 7.0 | Draft #3899 (stacked on #3854) |
| DEP-17 | urdfdom kept (DD-5): floor and cleanup; in-house URDF reader dropped | follow-up study | base, gz, pkg, apt, py | Codex high | 7.0 | PR1 ready #3884; PR2 merged (#3863); PR3 draft #3887; PR4 merged into #3884 (#3888) |
| DEP-18 | SDF packaging with sdformat kept (DD-5) | follow-up study | base, gz, pkg, BSD, py | Orchestrator + Codex high | 7.0, before F | Ready #3889 |
| DEP-19 | Asset networking (libcurl, OpenSSL) | follow-up study (DD-6) | base, gz, pkg, py, wheel, ai | Codex ultra | 7.0, before F | Wheel half in review; base #3885 merged; the source-build fetch waits on the tinyxml2 decision |
| DEP-20 | GUI on stock 24.04 (GLFW fallback, Filament pin enforcement, texture-decode caps) | spike first | base, gui, wheel | Codex high, Muse | 7.0 / 7.x | (d), (e) merged (#3860, #3862) |
| DEP-21 | DART-owned targets build as C++20 | DD-3 | C++20 gates in [verification.md](verification.md#c-standard-gates) | Orchestrator + Codex ultra | 7.0 or 7.1, in PLAN-624 merge order | Planned |
| DEP-22 | Developer-tooling refresh (CMake cap, Tracy, clang-format, formatters) | none | check-lint parity, gz | Muse / Codex high | 7.x | Planned |
| DEP-23 | Packaging section and downstream handoff notes | approval per upstream action | lint, local paths | Orchestrator | 7.0 / 7.x | Planned |
| DEP-24 | 8.0 design record: optional assimp and FCL, backend facades, C++20 export | none | review | Orchestrator | 8.0 | Planned |
| DEP-25 | Umbrella `DART_USE_SYSTEM_DEPENDENCIES` switch, offline fail-fast and CMP0170 (shared step), then fetch paths one dependency per PR starting with tinyxml2 (DD-10, DD-30) | none | base, pkg, dis | Codex high; the tinyxml2 prototype Codex ultra | 7.0 | Ready #3879; the tinyxml2 fetch prototype is done and parked pending a maintainer decision |
| DEP-27a | Root NOTICE for derived code, installed and in wheels, with a coverage test | none | base, wheel | Codex high | 7.0 | Ready #3857 (stacked on #3856) |
| DEP-28 | Delete dead `dart/utils/*_` files and `tests/.deprecated` | none | base | Muse | 7.0 | Ready #3856 |
| DEP-30 | Fetch Dear ImGui as a hash-pinned, TLS-verified tag archive and drop its compatibility patch (DD-14) | DEP-31 | base, gui, dis | Claude Sonnet | 7.0 | Ready #3876 |
| DEP-31 | Reject empty overlay font files before ImGui | none | base, gui | Muse | 7.0 | Merged (#3861) |
| DEP-32 | Skip voxel-grid rays longer than OctoMap can trace | none | base, py, gz | Codex | 7.0 | Merged (#3859) |
| DEP-33 | Sample-data provenance and modernization: attribute Atlas v3 (DD-23), WAM's BSD files only (DD-20), remove kima (DD-21), replace KR5 with an authored fixture and UR5e (DD-22), catalog G1, UR5e and FR3 (DD-24) | DD-16, DD-17 | base, py, gui | Codex high, Claude Sonnet | 7.0 | (a) merged (#3880), (b) ready #3878, (c) draft #3882 (stacked on #3857); (d) the online catalog after DEP-19 |
| DEP-34 | Replace the Roboti-derived MuJoCo models with DeepMind's (DD-15) and remove the Fetch assets (DD-19), with authored MJCF fixtures | DEP-36 first | base, py | Codex high | 7.0 | Draft #3881 |
| DEP-35 | Carry an FCL 0.7.0 patch for GJK distance on touching shapes in Linux wheels (DD-18) | none | base, wheel | Codex ultra | 7.0 | Merged (#3870) |
| DEP-36 | Fix MJCF joint limits and declared inertia (DD-25) | none | base, py | Codex high | 7.0 | Merged (#3877) |
| DEP-37 | Fix the Bullet detector leak (DD-26) | none | base, ASan, gz | Codex high | 7.0 | Merged (#3875) |

Ordering: DEP-0 lands first, because no lane notices a lost contact-surface
probe today and many later packets touch installed headers. DEP-1 to DEP-12
can then run in parallel lanes. Hunks in `World.cpp` and `lcpsolver`, the
whole-library C++20 switch (DEP-21) and the clang-format reformat (DEP-22)
follow the PLAN-624 program-owned merge order, each re-qualified on current
`main`.

Packet scope, non-goals and gates are in [packets.md](packets.md); an
executor that finds the real scope materially different stops and reports.

## Gates

- **base:** `pixi run lint`, `pixi run check-lint`, `pixi run build`,
  `pixi run test`; add `pixi run test-py`, `pixi run test-gui` and
  `pixi run test-build-requirements` when bindings, the GUI or floors change.
- **gz:** `pixi run -e gazebo test-gz`, `pixi run -e gazebo gz-compat-ionic`,
  `pixi run -e gazebo-harmonic gz-compat-harmonic`,
  `pixi run -e gazebo-jetty gz-compat-jetty`, compared against a base
  variant (`GZ_COMPAT_BASE_VARIANT`), plus the gz API inventory check and the
  `ContactSurfaceProbe` CTest.
- **pkg:** `pixi run test-package-core` plus installed-consumer configures,
  including one that sets `cmake_minimum_required(VERSION 3.5.1)` as
  gz-physics Harmonic does.
- **wheel:** cp312-abi3 build, repair, verify and test on every platform,
  `abi3audit --strict`, `scripts/check_headless_wheel.py`, and the size before
  and after.
- **apt:** the stock-noble lane (DEP-12).
- **dis:** configure with `-DFETCHCONTENT_FULLY_DISCONNECTED=ON`, with and
  without the system packages.
- **BSD:** the FreeBSD lane. **ai:** the AI infrastructure checks.

## Risks

- **A silently lost contact-surface feature in Gazebo.** gz-physics drops
  contact-surface customization if `ContactSurface.hpp` stops parsing under
  its probe, and gz-sim's tracked vehicles and conveyors stop working with no
  error. Mitigation: DEP-0 first, the existing `ContactSurfaceProbe`, and the
  header dialect gates.
- **Mixed-dialect ODR or ABI drift** under the recommended C++ option.
  Mitigation: the C++20 library rule, `abidiff`, bit-identical outputs and an
  external C++17 consumer; fall back to C++17.
- **Review bandwidth against the M1 critical path.** Mitigation: small
  decision-free packets routed to Sonnet, Muse or Codex; the orchestrator
  keeps integration.
- **Behavior changes that users see:** the log format (DEP-14) and the
  fetch-first default for source builds (DD-8). Each is a labeled change with
  a migration note.
- **Fetch-first defaults versus ODR.** A fetched copy of a library whose types
  appear in DART's installed headers (Eigen, assimp, tinyxml2, FCL) can
  disagree with the copy a consumer links. Mitigation: the per-dependency
  matrix keeps such libraries system-first where correctness needs it.
- **Platforms the floor drops:** Ubuntu 22.04 (intended), Debian 12 and
  EL9. Fedora 44 and EPEL10 cannot build the core at any floor, because they
  retired FCL and urdfdom.

## Blockers and next actions

See [RESUME.md](RESUME.md) for the published PRs, their merge order and the
next steps. The maintainer answered the second decision batch one question
at a time ([decisions.md](decisions.md), DD-10 to DD-34).
