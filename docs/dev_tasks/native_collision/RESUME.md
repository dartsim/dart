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
FCL/Bullet/ODE collision component exports, reference collision libraries, and
FCL, Bullet, ODE, or libccd runtime libraries. The existing repaired py312
Linux wheel passes the new verifier locally; the full CI wheel matrix still
needs run/artifact evidence.
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
mesh-plane regression tests, pair-order contact normal tests, and
parallel-cylinder cap/side contact handling for stacked support. Commit
`6e04945b29d6` also adds axial cylinder-cap support patches against large
boxes, covering gz-physics plane geometry as represented by its DART plugin's
large-box fallback. Focused DART tests pass. The latest focused gz-physics run
now passes
`COMMON_TEST_collisions_dartsim`, `COMMON_TEST_detachable_joint_dartsim`, and
`COMMON_TEST_joint_transmitted_wrench_features_dartsim`. The remaining focused
gz blocker is `COMMON_TEST_joint_features_dartsim`
`JointFeaturesDetachTest/0.JointDetach`, which reports tiny off-axis velocity
components against exact-zero tolerances after richer box/cylinder contact
support. Re-running the isolated case at `6e04945b29d6` reports
`upperLinkLinearVelocity.X() = -0.00013953469787260998`,
`upperLinkLinearVelocity.Y() = 1.0002864929523347e-06`, and
`upperLinkAngularVelocity.Y() = -6.6438361021132697e-05` against `1e-6`.
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
The newest temporary gz velocity diagnostic also showed that the remaining
`JointDetach` off-axis residual follows base support motion: at step 9, the
base angular velocity around `Y` is `-6.6445229724690596e-05`, the upper-link
angular `Y` is `-6.6438361021132697e-05`, and the upper-link linear `X`
residual is consistent with that base rotation through the joint offset. The
diagnostic instrumentation in `.deps/gz-physics` was removed after capture.
Two later local support-contact shaping experiments were also rejected and
reverted: a centered cylinder-cap/large-box support point made gz fail
`JointDetach` at `upperLinkLinearVelocity.Z() > 1e-5` with
`Z = 1.63203e-06` and still left off-axis support motion, and a weighted
support-centroid variant again failed the same upward-motion check with
`Z = -6.02409e-08`. Do not retry cylinder-cap support shaping without a new
reduction that preserves the expected upward motion and reduces base support
angular drift.

## Current Branch

`feature/new_coll` - local branch tracking `origin/feature/new_coll`, ahead by
the native-collision checkpoint commits in this task. Use `git status -sb` for
the exact count.

## Immediate Next Step

Continue from `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`.
The immediate blocker is now the remaining gz-physics `JointDetach` residual.
Reduce `COMMON_TEST_joint_features_dartsim`
`JointFeaturesDetachTest/0.JointDetach` to DART/gz contact evidence, decide
whether the tiny off-axis velocity components come from a native contact bug,
model-level filtering semantics, or downstream exact-zero tolerance debt, then
fix or document the accepted downstream-compatible path before rerunning the
full gz gate.
The strongest current lead is native base-vs-ground support stability, not
detach-state restoration: the upper-link residual follows base angular `Y`
motion through the upper joint offset. Focus on plane-as-large-box support
contacts, manifold persistence, solver warm-start behavior, and base pole/plate
support symmetry.
Do not retry broad tilted cap support patches without first preserving the
test's expected `upperLinkLinearVelocity.Z() > 1e-5`; the last two local
variants overconstrained the base support contacts and failed that earlier
sanity check. The later mirrored cap-contact variant failed the same way and
should also stay rejected unless a new reduction shows a different issue. The
centered support-point and weighted support-centroid variants are rejected for
the same reason and should not be repeated as blind contact-generation changes.
After that, continue the broader plan below.

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
- A previous gz-physics run passed 65/65 tests and the plugin link check. That
  remains historical evidence for the earlier checkpoint, but it does not close
  the current source state because the latest focused gz-physics run still has
  the `JointDetach` exact-zero velocity residual.
- Commit `6e04945b29d6` adds native axial cylinder-cap support patches against
  large boxes and native DART tests for support-patch generation plus
  `maxNumContacts` limiting. This improves the gz plane-as-large-box contact
  path but does not close the remaining `JointDetach` exact-zero residual.
- Rejected diagnostics after `6e04945b29d6`: multi-point and single-point
  near-parallel tilted cylinder-cap/box-face replacements both passed focused
  native tests but made the gz `JointDetach` case fail earlier at
  `upperLinkLinearVelocity.Z() > 1e-5`. The current blocker is therefore not a
  simple "more cap contacts" fix.
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
  implementation-ready gap plan lives in `04-reference-gap-analysis.md`.

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

Use `pixi run ...` tasks for validation and update the evidence gates after
each meaningful test or benchmark run.
