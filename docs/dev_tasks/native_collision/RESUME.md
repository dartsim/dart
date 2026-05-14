# Resume: Native Collision Default

## Last Session Summary

Work resumed on `feature/new_coll` to make the native collision detector the
default replacement for FCL, Bullet, and ODE. This session moved core default
paths to `dart`, fixed native feature and performance parity gaps, preserved
gz-physics compatibility through the legacy `DARTCollisionDetector` facade,
made FCL/Bullet/ODE optional for native default builds, and reran full
validation successfully. The working tree now also starts reference harness
isolation with CMake opt-out options for reference tests and benchmarks, with
focused reference-disabled and reference-enabled validation recorded. The
reference toggles now propagate through the major configure entry points, wheel
build defaults are explicitly native-only, and core native-only link inspection
shows `libdart.so` does not link FCL, Bullet, ODE, or libccd. The public
factory keys `fcl`, `fcl_mesh`, `bullet`, and `ode` now resolve to the built-in
`DartCollisionDetector` in both native-only and reference-enabled builds.
The native-only package-export cleanup now keeps default `find_package(DART)`
on the `dart` component and retains `collision-fcl`, `collision-bullet`, and
`collision-ode` only as native-backed compatibility component facades. Normal
pixi configure paths now also default FCL, Bullet, ODE, reference correctness
tests, and reference benchmarks to `OFF`, while explicit
`DART_BUILD_COLLISION_*_OVERRIDE=ON` settings restore the reference component,
test, and benchmark targets for comparison jobs. A fresh native-only runtime
install probe now also shows no old-engine/reference collision libraries and no
FCL, Bullet, ODE, or libccd runtime links from installed shared libraries or
the built dartpy extension. Default and wheel Pixi dependency metadata now omit
FCL, Bullet, ODE, and their FCL transitive packages; the explicit
`collision-reference` environment owns those packages and its focused
`test_reference_backends` target builds and passes. A repaired py312 wheel
artifact built with old collision engines and reference harnesses disabled
imports successfully and contains no old collision component files, old
collision CMake exports, or FCL, Bullet, ODE, or libccd runtime links. Python
detector compatibility names now resolve to `DartCollisionDetector`, and
dartpy no longer links legacy collision component targets even in a
reference-enabled build. The latest slice splits retained package component
names from old-engine comparison targets: `collision-fcl`, `collision-bullet`,
and `collision-ode` are native-backed interface facades, while old-engine
libraries/components use explicit `collision-reference-*` names. Focused
reference builds/tests and a default native-only downstream package smoke
passed with that split. Installed legacy detector headers now compile without
FCL, Bullet, or ODE and provide native-backed detector facades in both
native-only and reference-enabled installs. The latest performance-guard slice
adds `scripts/check_collision_benchmarks.py` and broadens
`pixi run -e collision-reference bm-collision-check` across checked
narrowphase, distance, raycast, mixed-primitive, mesh-heavy, raycast-batch, and
public DART adapter benchmark subsets. Native/reference subsets compare JSON
against the best enabled reference backend, while the public adapter subset
records JSON for collision, dirty-world collision, distance, and raycast
through `DartCollisionDetector`.
The latest source-cleanup slice makes top-level source-tree FCL, Bullet, and
ODE detector/group, All, PascalCase, and component headers native-backed
facades and moves old-engine implementation headers/sources under explicit
`reference/` paths used by reference tests and benchmarks. The latest planning
update makes the built-in collision component/layer design explicit: the
north-star layer table now tracks public API cleanliness, compatibility
facades, the DART adapter, the native scene/query core, and optional reference
harnesses as separate review gates.
The latest CI/performance slice adds a scheduled/manual CI Linux
`Collision Benchmark Guard` job that runs the broad
`pixi run --locked -e collision-reference bm-collision-check` guard and uploads
`.benchmark_results/collision_check_*.json` artifacts. It still needs GitHub
run evidence.
The latest wheel-isolation slice adds `scripts/verify_wheel_collision_isolation.py`
and wires it into `wheel-verify-core` so every dartpy wheel verify rejects old
runtime libraries, reference collision libraries, and reference component
exports while allowing native-backed FCL/Bullet/ODE compatibility component
facades. The existing repaired py312 Linux wheel passes the new verifier
locally; the full CI wheel matrix still needs run/artifact evidence.
The latest downstream-migration slice documents the compatibility contract in
`05-downstream-migration.md` and annotates the FCL, Bullet, and ODE
compatibility facades so downstream users can see that retained legacy runtime
names route to the built-in detector. The documented migration path still needs
fresh gz-physics and package-smoke run evidence before final deletion.
The latest runtime-isolation slice adds
`scripts/check_collision_runtime_isolation.py` and wires it into `lint` and
`check-lint` so non-reference DART source paths cannot include FCL, Bullet,
ODE, libccd, or explicit collision reference backend headers, and legacy engine
implementation sources cannot move back outside `reference/` paths.
The latest documentation slice makes the built-in collision architecture gate
explicit on the north-star scale: Phase 11 now requires API cleanliness,
adapter scalability, native core scalability, performance-oriented internals,
reference isolation, and compatibility facade evidence before the abstraction
cleanup can be marked complete.
The latest compatibility cleanup preserves legacy direct C++ facade display
strings for gz-physics while keeping those facades native-backed. It also adds
native plane/mesh dispatch, unbounded-plane AABB coverage, focused DART
mesh-plane regression tests, pair-order contact normal tests,
parallel-cylinder cap/side contact handling for stacked support, axial
cylinder-cap support patches against large boxes, tilted
cylinder/plane-like-box support, capped large flat box/mesh contact patches,
and FCL/ODE legacy facade raycast compatibility. Focused DART tests pass, the
focused `COMMON_TEST_simulation_features` run passes 15/15, and fresh local
`pixi run -e gazebo test-gz` passes 65/65.
Temporary contact logging after that checkpoint showed only base-vs-ground
contacts during the failing window; the detached upper/lower links were not in
contact. The support contacts are the base plate cylinder and base pole box
against gz-physics' plane-as-large-box fallback. Two local tilted
cylinder-cap/box-face experiments were rejected because they made
`JointDetach` fail earlier at the expected upper-link upward-motion check, so
the retained source state keeps the convex fallback for tilted cap/box and only
keeps the validated axial cap support patch.
This session also rejected a narrower mirrored cap-contact experiment because
it again failed the same expected upward-motion check. The retained working
tree instead keeps the architecture-valid solver cache bridge fix:
`ContactConstraint` writes native `CachedContact` impulses whenever native
cache metadata is attached, even if a compatibility detector facade reports a
legacy display string such as `"ode"`. The focused
`UNIT_constraint_SoftContactConstraint` regression covers that behavior.
The latest working-tree slice also canonicalizes warm-start local points to
the same scene-cache ID order used by the persistent manifold key. The focused
`UNIT_collision_NativeBackend` regression now proves cached impulses survive
when the same pair is queried as group B/A and then A/B. Reinstalling DART into
the gz environment and rerunning the isolated `JointDetach` case still reported
the same off-axis residual values, so this is a kept cache correctness fix but
not by itself the final gz compatibility fix.
The following gz velocity diagnostic showed that the remaining `JointDetach`
off-axis residual followed base support motion: at step 9, the base angular
velocity around `Y` was `-6.6445229724690596e-05`, the upper-link angular `Y`
was `-6.6438361021132697e-05`, and the upper-link linear `X` residual was
consistent with that base rotation through the joint offset. The diagnostic
instrumentation in `.deps/gz-physics` was removed after capture. Two local
support-contact shaping experiments were rejected and reverted: a centered
cylinder-cap/large-box support point made gz fail `JointDetach` at
`upperLinkLinearVelocity.Z() > 1e-5` with `Z = 1.63203e-06` and still left
off-axis support motion, and a weighted support-centroid variant again failed
the same upward-motion check with `Z = -6.02409e-08`.
The latest working-tree slice closes that focused gz blocker with a tilted
cylinder/plane-like-box support patch: a reduced native
`ConstraintSolver.GzPlaneBoxJointDetachKeepsSupportVelocitySymmetric` test now
passes, isolated gz `JointFeaturesDetachTest/0.JointDetach` passes, the full
local `COMMON_TEST_joint_features` binary passes 21/23 with 2 skips, and the
related `COMMON_TEST_collisions`, `COMMON_TEST_detachable_joint`, and
`COMMON_TEST_joint_transmitted_wrench_features` binaries pass against the DART
plugin.
The newest working-tree slice closes the full local gz gate: native large flat
box/mesh contacts are capped at 32 while still exposing more than 30 contacts
for gz max-contact selection tests, FCL/ODE compatibility facades keep
gz-required unsupported raycast behavior, focused
`COMMON_TEST_simulation_features` passes 15/15, and a fresh
`DART_PARALLEL_JOBS=15 pixi run -e gazebo test-gz` run passes 65/65 tests.
The latest evidence slice adds a reproducible native compatibility package
smoke under `docs/dev_tasks/native_collision/smoke/native_compat_package/` and
reruns it against a current install. The smoke verifies retained
`collision-fcl`, `collision-bullet`, and `collision-ode` package components
link the built-in stack, factory keys create the canonical `dart` detector,
direct legacy class `create()` calls are backed by `DartCollisionDetector`
while keeping legacy display strings, and the installed library directory has
no old-engine or reference collision runtime libraries.
The broad local performance guard was also refreshed at `892e50d02e4` with
`pixi run --locked -e collision-reference bm-collision-check`; it passed the
checked narrowphase, distance, raycast, mixed-primitive, mesh-heavy, and
raycast-batch native-vs-reference subsets plus the public adapter JSON subset.
The generated `.benchmark_results/collision_check_*.json` files are local
evidence only; GitHub workflow artifact evidence is still required for the CI
benchmark gate.
The latest audit slice adds `06-completion-audit.md`, which restates the
single-PR objective as concrete deliverables and maps every north-star
requirement to current artifacts, local evidence, missing CI/artifact evidence,
and final cleanup gates. It records that the task is not complete yet.
The newest validation slice reran `pixi run test-all` after the audit and
found a stale optional libccd cache failure in `build-tests`: CMake still had
`LIBCCD_LIBRARY` cached to a removed default-environment
`.pixi/envs/default/lib/libccd.so`, so Ninja generated
`test_libccd_algorithms` with a missing link dependency even though normal
native collision reference paths were disabled. The working tree now resets
missing cached libccd include/library paths in `cmake/dart_test_libccd.cmake`,
and targeted `pixi run build-tests ON Release` passes with the optional
libccd comparison target excluded when libccd is unavailable.
The follow-up full local
`DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all` run passed at
`da532729bec`: lint, Release build, Release C++ test build, C++ unit tests,
simulation-experimental tests, Python tests, and documentation all passed, with
Release CTest reporting 264/264 tests passed and 29 `collision-native` label
tests.
The latest architecture-cleanup slice removes unused direct native
`ShapeType` values for cone, heightfield, and point-cloud. Those DART shapes
remain covered at the adapter layer through explicit convex, mesh, compound, or
non-collidable behavior, while the native shape taxonomy now names only real
native shape classes. Focused `test_shapes` and the rebuilt
`dart_collision_native_tests` aggregate passed; the full `collision-native`
label passed 29/29 after rebuilding stale test binaries.
The current audit reran full validation and found one stale downstream
compile-time dependency on those removed native shape tags:
`dart/gui/vsg/geometry_builders.cpp` still switched on `ShapeType::Cone`,
`ShapeType::HeightField`, and `ShapeType::PointCloud`. That VSG switch now
falls through only through the valid native shape taxonomy. The focused Debug
`dart-gui-vsg` target passes, and a fresh
`DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all` run passes:
lint, build, unit tests, simulation-experimental tests, Python tests, and
documentation all passed.
After seeding durable onboarding architecture docs and clarifying the final
native-backed compatibility facade policy, the full local gate was refreshed
again at `864dd56d944c` with
`DART_PARALLEL_JOBS=15 CTEST_PARALLEL_LEVEL=15 pixi run test-all`. It passed
all 6 top-level gates: linting, build, unit tests,
simulation-experimental tests, Python tests, and documentation.
The branch was published and draft PR #2652
(https://github.com/dartsim/dart/pull/2652) was used as the initial CI
surface. PR #2652 is now closed per user direction and remains anchored to old
head `714d220d82a`; the latest pushed branch head before the local closed-PR
follow-up was `0dda069d59d`. The first PR CI refresh exposed a workflow-only
failure
in `Asserts enabled (no -DNDEBUG)`: the custom CMake configure still enabled
Bullet in the default Pixi environment, where Bullet is intentionally absent.
The latest repair makes that custom asserts configure explicitly native-only
for FCL, Bullet, ODE, and collision reference harnesses, and the matching local
`CMAKE_BUILD_TYPE=None` configure now passes. That repair was pushed as
`901d56c4260` (`Fix native collision asserts CI configure`). A follow-up
`origin/main` merge was committed locally as `3120a4fce9b` to clear PR
mergeability conflicts and pushed to `feature/new_coll`; focused local
configure, build, and anchored CTest coverage for the merge-resolved native
collision, contact, world, and GUI targets pass. The latest PR CI refresh then
found a Windows-only Pixi task parser failure before CMake ran:
`CI Windows` / `Tests (Release)` rejected Bash-style collision override
defaults in the raw Windows `config` task. The working tree now wraps Windows
`config`, `config-py`, and `config-install` in `bash -lc`, matching the
existing Windows test tasks. The next PR refresh reached past that Windows
parser failure and exposed an Alt Linux configure failure: Eigen 5 is installed
there, and the experimental dependency helper rejected it by requesting
`find_package(Eigen3 3.4 ...)` directly instead of using DART's explicit
minimum-version check pattern. That repair was pushed at `d2f1b7233bd`.
The next PR refresh then found a Windows wheel compile failure in
`dart-collision-native`: MSVC did not receive `/utf-8`, and the packaged `fmt`
headers require that mode for Unicode support. The same refresh showed the
Alt Linux Eigen repair got past Eigen, then failed on missing EnTT package
config because `dart-collision-native` was still using the broader
simulation-experimental dependency bundle. The first EnTT fallback repair got
past package discovery, then failed during CMake generation because EnTT's
fetched build target was pulled into DART's export set; the current working
tree changes the fallback to expose an imported interface target over fetched
headers only. The next PR refresh on `97a9d3ca6d6` reached macOS arm64 release
compilation and failed in `INTEGRATION_collision_native_backend_consistency`
because new native collision test/benchmark sources still included generated
CamelCase compatibility headers. The current working tree switches those files
to lowercase canonical headers and explicit lowercase `reference/` detector
headers, with focused local rebuilds of
`INTEGRATION_collision_native_backend_consistency` and
`bm_scenarios_raycast_batch` passing. The next PR refresh on `623f0180fa8`
then reached macOS arm64 Debug linking and failed `UNIT_collision_DistanceFilter`
because `BodyNodeDistanceFilter` still had public declarations but no compiled
implementation. The current working tree restores the historical
`BodyNodeDistanceFilter::needDistance()` and `areAdjacentBodies()` definitions,
with focused Debug and Release local build/CTest coverage for
`UNIT_collision_DistanceFilter`. The next PR refresh on `f726ee5e4ee` reached
macOS arm64 Release compilation and failed `UNIT_collision_NativeBackend`
because `DistanceFilter` was a polymorphic `std::shared_ptr` base without a
virtual destructor; the current working tree adds the destructor. The same
refresh showed `Wheels | macos-latest Py314` building and delocating
successfully, then failing `wheel-verify` because the verifier still rejected
native-backed `dart_collision-{fcl,bullet,ode}Component.cmake` compatibility
facade files. The current working tree keeps old runtime/reference artifacts
forbidden while allowing those facade component files. Focused local validation
for this repair has passed: the Release `UNIT_collision_NativeBackend` target
and CTest pass, synthetic wheel verifier probes accept the facade files and
reject reference/runtime artifacts, and the existing repaired py312 Linux wheel
passes the updated verifier.
After the closed PR direction, follow-up local validation found one
test-harness problem unrelated to the native collision runtime:
`pixi run test-all` failed linking `dartpy` because
`scripts/test_all.py` treated a valid nanobind `buf.put(" = ");` line as a
corruption marker, refreshed `nb_func.cpp`, and left stale
`nb_internals.cpp` static-library objects behind. The helper now only matches
the invalid escaped default-argument source, refreshes `nb_internals.cpp` and
`nb_internals.h` with any upstream nanobind source refresh, and touches
internals when `nb_func.cpp` is newer. Focused Release and Debug `dartpy`
target rebuilds passed, then
`DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
passed all 6 top-level gates: linting, build, unit tests,
simulation-experimental tests, Python tests, and documentation.

## Current Branch

`feature/new_coll` - local branch tracking `origin/feature/new_coll`. PR #2652
is closed and still points at old head `714d220d82a`. The branch now includes
a docs-only resume checkpoint on top of the latest code/evidence head,
`1e1faf6feb1` (`Fix native capsule convex casts in CI`). That code commit
fixes the closed-PR native-only CI failure by replacing endpoint-only
`capsuleCastConvex()` checks with capsule-support conservative advancement
against convex targets, and fixes the benchmark guard artifact upload by
allowing hidden `.benchmark_results/` files.

Primary local validation on the pushed head is green:
`pixi run lint` passed before commit, then `pixi run build` and
`pixi run test-unit` passed after the push; `test-unit` reported 277/277 tests
passed, including `test_ccd` and all 29 `collision-native` tests. A focused
local ASAN attempt is blocked by the host toolchain rather than by DART code:
GCC's `libasan.so` linker script points to missing
`/usr/lib64/libasan.so.8.0.0`, so ASAN linking fails before tests can run.

Manual workflow-dispatch runs for `1e1faf6feb1` are reference evidence only,
not the main validation surface. Current status: CI gz-physics run
`25887940214` passed, Publish dartpy run `25887941240` passed across the full
wheel matrix, and CI Linux run `25887939088` has already passed
`Native Collision (no FCL/Bullet/ODE)` job `76084265248` plus
`Collision Benchmark Guard` job `76084265196`. The benchmark guard uploaded
artifact `collision-benchmark-guard-25887939088-1`, id `7006005918`, digest
`sha256:c92c993b9d6a2eaf0ac234d7526cc9893c6544992f0ed55fd77a8bf7f02ba2f5`.
The broad CI Linux `Release Tests` job is still in progress in its ASAN tail,
so do not block local progress on that reference-only signal.

## Immediate Next Step

Continue from `docs/dev_tasks/native_collision/06-completion-audit.md`.
Current local focused validation is green on pushed head `1e1faf6feb1`, and
manual workflow-dispatch reference evidence now covers native-only CI,
gz-physics, the wheel matrix, and the benchmark artifact upload for that head.
Do not create or reopen a PR/diff until the user asks. Continue by recording
downstream migration/deprecation evidence, deciding the final
compatibility-facade/runtime cleanup slice, rerunning full local validation
after any further code changes, then transferring final evidence to the PR
description and deleting the dev-task folder in the completing PR. Read
`06-completion-audit.md` before deciding whether a future checkpoint is
complete; it is the prompt-to-artifact checklist for the north-star goal.

The persistent DART adapter scene path is now started: public collision,
distance, and raycast calls use synced native scene state owned by
`DartCollisionGroup`, and public raycast uses native broadphase segment-AABB
candidate pruning. Public distance uses scene AABB lower bounds to skip
narrowphase pairs that cannot improve the current best distance. Manifold
warm-starting now uses scene-issued cache IDs instead of DART object addresses,
so shape replacement invalidates stale solver impulses. A public adapter-path
benchmark target now covers collision, dirty-world collision, distance, and
raycast through `DartCollisionDetector`. Shape invalidation/behavior coverage
now includes primitives, mesh scale, soft-mesh point-mass motion, heightmaps,
voxel-grid occupancy when OctoMap is enabled, and explicitly non-collidable
point clouds. DART collision filters now adapt into native pair checks before
narrowphase. The initial Linux native-only CI job is in the working tree and
still needs GitHub CI evidence. Reference test/benchmark isolation has started:
the branch adds `DART_BUILD_COLLISION_REFERENCE_TESTS` and
`DART_BUILD_COLLISION_REFERENCE_BENCHMARKS`, validates focused
reference-disabled and reference-enabled configurations locally, propagates the
toggles through the major configure entry points, defaults normal pixi configure
paths to native-only collision, and has core native-only link evidence. It
now also has fresh runtime-link, package-export, default/wheel Pixi dependency
metadata, explicit `collision-reference` environment evidence, and repaired
py312 wheel artifact evidence. It still needs CI wheel-matrix artifact
evidence.
Collision abstraction cleanup has also started: `DartCollisionDetector` owns
native-backed public factory aliases for `experimental`, `fcl`, `fcl_mesh`,
`bullet`, and `ode`; the FCL/Bullet/ODE component registrars also publish
native-backed creators so linking old reference component libraries cannot
restore public backend selection; and the Python compatibility names
`DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`, and
`OdeCollisionDetector` now construct/report the built-in `dart` detector.
Reference tests and benchmarks now use explicit `createReference()` APIs on the
FCL, Bullet, and ODE detector classes, and direct public C++ legacy detector
`create()` paths now return the built-in `DartCollisionDetector`. Retained
legacy package component names are native-backed CMake interface facades, and
old-engine libraries/components use `collision-reference-*` names. Installed
legacy detector headers and source-tree top-level detector/group/All/PascalCase
headers are native-backed facades in native-only and reference-enabled
installs. Old-engine FCL, Bullet, and ODE implementation headers/sources now
live under explicit `reference/` paths and remain available only to reference
tests and benchmarks. Runtime source isolation is now enforced by lint so those
old-engine includes and implementation sources cannot return to the normal DART
source paths unnoticed.
User-facing examples/tutorials have also been moved off the old collision
components: their CMake no longer requires `collision-bullet`/`collision-ode`,
source no longer includes or selects Bullet/ODE/FCL detector APIs, and affected
examples/tutorials build in the default native-only configuration.
`human_joint_limits` is still skipped in this environment because TinyDNN is
unavailable.
`docs/dev_tasks/native_collision/01-design.md` is the canonical architecture
contract for that remaining cleanup: public API and compatibility facades
outside the DART adapter, `dart/collision/native/` as the scene/query core, and
explicit gates for API cleanliness, scalability, performance hooks, reference
isolation, and gz-physics-safe compatibility facades. The latest design update
adds a concrete built-in component blueprint covering the public API boundary,
adapter-owned scene model, native scene/query data model, query lifecycle,
scalability rules, performance-oriented internals, and a subcomponent review
map for detector/factory surface, compatibility facades, adapter scene, native
scene/geometry tables, broadphase snapshots, narrowphase dispatch, result
builder, solver cache bridge, and optional reference harness. It also pins the
contact result contract: native contacts reported through public DART APIs must
follow collision object pair order, with canonical shape-specialized
narrowphase functions wrapped by dispatch/result orientation when needed.
Package-export cleanup has also started: native-only `find_package(DART)` now
defaults to `dart` only, generated component templates no longer emit
deprecated Bullet/ODE collision component text, and a native-only install probe
found no old collision component/library names in searched installed
CMake/pkg-config metadata beyond `DART_BUILD_COLLISION_*` variables set to
`OFF`. Normal pixi configure paths now request the same native-only default;
old engines and reference harnesses are opt-in through override variables. A
fresh install probe built the normal native runtime components plus dartpy from
an empty build tree and found no old-engine runtime links. The latest Pixi
metadata split removes old collision engines from default and wheel
environments and adds `collision-reference` for reference-only package
availability. The newest native-only install/export probe also confirms
`dart_collision-fclComponent.cmake`, `dart_collision-bulletComponent.cmake`,
and `dart_collision-odeComponent.cmake` import interface targets linking
`dart`, while the installed lib directory contains no
`libdart-collision-reference-*`, FCL, Bullet, ODE, or libccd collision runtime
libraries.

## Context That Would Be Lost

- `DartCollisionDetector::getStaticType()` returns `dart`; `experimental`,
  `fcl`, `fcl_mesh`, `bullet`, and `ode` are factory aliases for compatibility.
- `ConstraintSolver`, `WorldConfig`, and SKEL loading now prefer `dart`.
- Native box-box face contacts now emit a contact patch when `maxNumContacts`
  allows it; this fixed the default solver expanded-manifold regression.
- `pixi run -e gazebo test-gz` passed after the legacy uppercase compatibility
  header kept gz-physics ray intersection behavior unsupported while lowercase
  `DartCollisionDetector` retained native raycast support.
- Benchmarks show native now winning comparative primitive, narrow-phase,
  supported distance, raycast, batch-raycast, mesh-heavy, and the recorded
  mixed-primitive scenario cases, including dense 1000 after cached snapshot
  reuse.
- FCL, Bullet, and ODE can be disabled for a core `dart` build. Focused
  native/default C++ tests and `dartpy` also build and pass in that
  configuration.
- Normal pixi configure paths now default FCL, Bullet, ODE,
  `DART_BUILD_COLLISION_REFERENCE_TESTS`, and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS` to `OFF`. A default configure,
  target-help probe, native/default build, and focused detector CTest passed in
  that state.
- A fresh native-only runtime install probe built and installed `libdart`,
  `libdart-collision-native`, utils, URDF, IO, GUI, VSG GUI,
  simulation-experimental, and dartpy with old engines/reference harnesses off.
  Installed files, installed CMake/pkg-config metadata, installed shared-object
  `ldd`, dartpy `ldd`, and target-help inspection found no FCL, Bullet, ODE,
  libccd, old collision component, or reference-only target leakage beyond
  `DART_BUILD_COLLISION_*` variables reporting `OFF`.
- Pixi dependency metadata now matches the native-only runtime direction:
  `default`, `py312-wheel`, `py313-wheel`, and `py314-wheel` list no
  `bullet-cpp`, `fcl`, `libode`, `libccd-double`, `flann`, or `hdf5` packages
  in the lock. The explicit `collision-reference` environment lists those
  packages, configures FCL/Bullet/ODE plus reference tests/benchmarks `ON`,
  and builds/runs `test_reference_backends` successfully.
- The repaired py312 wheel was built with FCL, Bullet, ODE, reference tests,
  and reference benchmarks all disabled. `wheel-repair`, `wheel-verify`, and
  `wheel-test` passed. Precise wheel-content checks found no old collision
  component headers, old collision component libraries, old collision CMake
  exports, or FCL/Bullet/ODE/libccd runtime libraries; extracted shared-object
  `ldd` checks found no FCL, Bullet, ODE, or libccd links. `wheel-verify` now
  also runs a wheel-content collision-isolation check for every wheel matrix
  entry.
- dartpy no longer links `dart-collision-fcl`, `dart-collision-bullet`, or
  `dart-collision-ode`. Default dartpy build and Python collision/world tests
  passed. A reference-enabled dartpy build also passed with FCL/Bullet/ODE
  components present, built dartpy shared-object `ldd` checks found no
  FCL/Bullet/ODE/libccd links, and the Python compatibility names printed
  `dart dart` for instance and static detector type.
- The reference-enabled dartpy build required a GUI compatibility fix because
  ImGui 1.92.8 removed the obsolete `ImGuiKey_Mod*` aliases. DART now submits
  left/right modifier keys directly in `im_gui_handler.cpp`.
- FCL, Bullet, and ODE detector classes now expose explicit `createReference()`
  methods. Reference tests and benchmark call sites were mechanically moved to
  those methods. The reference target build, focused CTest set, and tiny
  comparative benchmark smoke runs passed.
- Explicit `DART_BUILD_COLLISION_*_OVERRIDE=ON` opt-in restores the old
  component targets, reference consistency tests, and comparative benchmark
  targets for comparison jobs.
- The last audited native feature gap was `VoxelGridShape`; native now adapts
  occupied OctoMap leaves into a compound of box cells and has unit plus
  integration coverage.
- `pixi run test-all` passes after the native voxel-grid work and GUI headless
  fixes. The fresh full run passed lint, build, Release C++ tests,
  simulation-experimental tests, Python tests, and docs.
- The current source state has fresh full gz evidence:
  `DART_PARALLEL_JOBS=15 pixi run -e gazebo test-gz` passed 65/65 tests from a
  fresh `.deps/gz-physics` clone.
- Commit `6e04945b29d6` adds native axial cylinder-cap support patches against
  large boxes and native DART tests for support-patch generation plus
  `maxNumContacts` limiting. The later tilted cylinder/plane-like-box support
  patch and large flat box/mesh contact cap now close the local gz gate.
- Rejected diagnostics after `6e04945b29d6`: multi-point and single-point
  near-parallel tilted cylinder-cap/box-face replacements both passed focused
  native tests but made the gz `JointDetach` case fail earlier at
  `upperLinkLinearVelocity.Z() > 1e-5`. Those diagnostics ruled out a simple
  "more cap contacts" fix before the later tilted support/contact-cap changes
  closed the local gz gate.
- Tests and benchmark comments now use native collision naming except for the
  explicit `"experimental"` compatibility alias.
- gz-physics compatibility and performance parity are explicit gates, not
  optional follow-up work.
- The next work is no longer feature-parity implementation. The single PR still
  needs CI hardening, wheel-matrix run/artifact evidence, downstream
  migration/deprecation coverage, built-in collision layer hardening for API
  cleanliness, scalable native scene/query state and performance-oriented
  internals, GitHub run/artifact evidence for the scheduled benchmark guard,
  final legacy runtime backend deletion validation, and then final PR
  packaging.
- Quality order for the remaining work: feature coverage first, correctness
  tests as the permanent gate, then gradual benchmark-driven performance work
  until native beats Bullet, FCL, and ODE on required workloads.
- Built-in architecture target: public DART APIs and compatibility facades stay
  outside the core, `dart/collision/dart/` adapts DART shapes/results/filters,
  and `dart/collision/native/` owns scene state, broadphase, narrowphase,
  query algorithms, caches, deterministic results, and profiling/benchmark
  hooks.
- Downstream migration target: runtime code should move to `dart`,
  `DartCollisionDetector`, `CollisionDetectorType::Dart`, and the default
  `dart` package component. Legacy factory keys/classes/components remain
  native-backed compatibility aliases during the migration window; intentional
  old-engine comparisons use `collision-reference-*` targets and
  `createReference()`.
- `04-reference-gap-analysis.md` captures the concrete gaps found from the
  reference capability comparison. The first adapter gap is started in code:
  `DartCollisionGroup` owns persistent native scene state and routes public
  collision, distance, and raycast through it, with broadphase-pruned public
  raycast candidates, AABB lower-bound distance pruning, and scene-issued
  manifold cache IDs. The public adapter benchmark target covers collision,
  dirty-world collision, distance, and raycast. Shape invalidation/behavior
  coverage now covers primitives, mesh scale, soft meshes, heightmaps, voxel
  grids, and point clouds. DART collision filters now flow through a native
  filter adapter before narrowphase. The recurring benchmark guard now also
  runs public DART adapter collision, dirty-world collision, distance, and
  raycast scenarios; CI Linux has a scheduled/manual benchmark job for the
  broad guard, and remaining benchmark work is GitHub run/artifact evidence
  plus adding new workloads as native capabilities land.
- CI hardening has started: `.github/workflows/ci_ubuntu.yml` now includes
  `Native Collision (no FCL/Bullet/ODE)`, which configures with all legacy
  collision backends disabled, builds `dart`, `dartpy`, native collision tests,
  and focused default-detector tests, then runs the `collision-native` label
  and dartpy collision/world smoke tests. The matching local command sequence
  passed with all three legacy collision backends disabled, including
  `collision-native` 29/29, focused default-detector CTests 4/4, and Python
  collision/world smoke 9 passed and 1 skipped. This still needs GitHub CI
  evidence.
- Reference harness isolation has started: CMake now exposes
  `DART_BUILD_COLLISION_REFERENCE_TESTS` and
  `DART_BUILD_COLLISION_REFERENCE_BENCHMARKS`, and the main `pixi run config`
  path accepts matching override environment variables. Focused
  reference-disabled builds/tests passed with old runtime backend options still
  ON but reference tests/benchmarks OFF; reference-only targets were absent in
  that configuration. Normal pixi configure paths now default FCL, Bullet,
  ODE, reference tests, and reference benchmarks to `OFF`. An explicit
  all-`ON` override configure restored old component targets,
  `test_reference_backends`, comparative benchmarks, mixed/mesh scenario
  benchmarks, and `INTEGRATION_simulation_MimicConstraint`.
- Configure-entry-point propagation has started: the main debug, dartpy,
  install, coverage, ASAN, Windows, and wheel paths now carry the reference
  test/benchmark toggles. Local `config-debug`, `config-py`, and
  `config-install` probes with FCL, Bullet, ODE, reference tests, and reference
  benchmarks all disabled reported those CMake cache values as `OFF`.
- Package/default runtime removal has started: wheel CMake defaults now
  explicitly disable FCL, Bullet, ODE, and reference harnesses, and normal pixi
  configure paths now use those native-only defaults. A native-only
  install-style build produced only `libdart.so` in the top-level lib directory;
  `ldd` found no FCL, Bullet, ODE, or libccd runtime links; and the
  `collision-native` label passed 29/29 in that configuration. After building
  installable utility and IO components, a native-only install probe installed
  no FCL, Bullet, ODE, libccd, or `libdart-collision-reference-*` libraries.
  The latest package-export cleanup also keeps retained `collision-fcl`,
  `collision-bullet`, and `collision-ode` component names as native-backed
  interface facades and moves old-engine component exports to explicit
  `collision-reference-*` names. The fresh runtime install probe now broadens
  that evidence to all installed native shared libraries, dartpy, target-help,
  installed package metadata from a clean build tree, and Pixi default/wheel
  lock metadata.
- Factory-level abstraction cleanup has focused evidence:
  `UNIT_collision_DartCollisionDetector`, `UNIT_simulation_World`,
  `INTEGRATION_collision_Collision`, `INTEGRATION_collision_CollisionGroups`,
  `INTEGRATION_io_SkelParser`, and `INTEGRATION_simulation_World` passed in a
  reference-enabled build after old component registrars were changed to
  native-backed creators. A native-only configure/build/test with FCL, Bullet,
  ODE, reference tests, and reference benchmarks all disabled also passed
  `UNIT_collision_DartCollisionDetector`, `INTEGRATION_io_SkelParser`, and
  `UNIT_simulation_World`; native-only target-help inspection listed no old
  collision component or reference-only targets.
- A reference-disabled probe showed
  `INTEGRATION_simulation_MimicConstraint` failing on the native/default
  detector before the target was gated under reference tests. Treat that as a
  tracked native behavior gap or reference-engine-specific coverage point when
  finishing abstraction cleanup; do not count it as native/default coverage yet.
- The north-star progress scale lives in `README.md` and `03-evidence-gates.md`;
  the detailed single-PR phase plan lives in `02-milestones.md`; the
  implementation-ready gap plan lives in `04-reference-gap-analysis.md`; the
  completion audit and missing-evidence checklist lives in
  `06-completion-audit.md`.
- Per the latest user direction, do not create a new diff or review request
  yet. PR #2652 is closed; keep publishing focused repair commits to
  `feature/new_coll` as needed, but do not assume those pushes trigger CI.
- The closed PR head was `714d220d82a`. Its macOS arm64 refresh reached
  Release native collision filter compilation and Debug
  `test_ccd`, `UNIT_gui_vsg_simple_viewer`, and
  `INTEGRATION_simulation_World`. The follow-up repairs remove unused
  callback-filter locals, handle sphere-box CCD initial overlap without
  indexing an unset axis, skip VSG simple-viewer tests when headless creation
  throws on a runner, and construct the world before overriding the `"dart"`
  factory in the null-creator typed-setter fallback test. Focused local Debug
  build/CTest for those three failing Debug targets passed, and focused
  Release build/CTest for `test_collision_filter` passed.
- The same closed-PR Linux run exposed a Debug-only
  `test_collision_world` failure where world-level sphere/capsule cast results
  pointed at loop-local collision-object handles. The current working tree
  caches stable world query result handles for raycast, sphere-cast, and
  capsule-cast results. Focused Debug `test_collision_world`, focused Release
  `test_ccd`/`test_collision_world`, and the full Release `collision-native`
  label all pass locally.
- PR #2652 is closed per user direction and still points at old head
  `714d220d82a`. Follow-up commits can be pushed to `feature/new_coll`, but
  those pushes do not start the main workflows because the workflow `push`
  filters exclude feature branches. Using the `jslee02` token, manual
  `workflow_dispatch` started CI Linux run `25885373625`, CI gz-physics run
  `25885373580`, and Publish dartpy run `25885373596` for head
  `658da0edb10`.
- Closed-PR manual dispatch evidence and current repair:
  - CI gz-physics run `25885373580` passed.
  - Publish dartpy run `25885373596` passed across all nine wheel jobs:
    Ubuntu, macOS, and Windows for Python 3.12, 3.13, and 3.14.
  - CI Linux run `25885373625` failed
    `Native Collision (no FCL/Bullet/ODE)` job `76075528745` in `test_ccd`,
    `CapsuleCastConvex.DirectHit`.
  - Root cause: `capsuleCastConvex()` tested only the two capsule endpoint
    spheres, so a full capsule-body hit against a convex target depended on a
    numerically fragile grazing endpoint contact.
  - Current working-tree repair: use a capsule support function and
    conservative advancement for capsule-vs-convex CCD.
  - Local validation after the repair: focused `test_ccd` passed, the full
    `collision-native` label passed 29/29, the native-only CI focused CTest
    regex passed 4/4, and the Python collision/world smoke passed 17/17.
  - The same Linux run's `Collision Benchmark Guard` job passed the benchmark
    command but uploaded no artifact because `upload-artifact@v6` excluded the
    hidden `.benchmark_results/` directory. The current working tree sets
    `include-hidden-files: true` for that upload step.
- The latest local broad Debug validation now passes after one additional
  simulation-experimental logging repair. The broad Debug `tests` target built
  normal C++ tests; the separate `dart_experimental_tests` target was needed
  for the 13 configured simulation-experimental executables. After preserving
  the source file path in Debug logging when `DART_EXPERIMENTAL_SOURCE_DIR` is
  not defined, the Debug simulation-experimental label passed 13/13 and the
  full Debug CTest rerun passed 277/277, including 29 `collision-native`
  tests.
- The latest full local validation now passes after a nanobind test harness
  repair in `scripts/test_all.py`. Focused Release and Debug `dartpy` target
  rebuilds passed, then
  `DART_PARALLEL_JOBS=5 CTEST_PARALLEL_LEVEL=5 CMAKE_BUILD_PARALLEL_LEVEL=5 pixi run test-all`
  passed all 6 top-level gates.

## How to Resume

```bash
git checkout feature/new_coll
git status -sb
git log -5 --oneline --decorate
```

Then read:

- `docs/dev_tasks/native_collision/README.md`
- `docs/dev_tasks/native_collision/01-design.md`
- `docs/dev_tasks/native_collision/02-milestones.md`
- `docs/dev_tasks/native_collision/03-evidence-gates.md`
- `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`
- `docs/dev_tasks/native_collision/06-completion-audit.md`

Use `pixi run ...` tasks for validation and update the evidence gates after
each meaningful test or benchmark run.
