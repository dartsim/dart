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
The native-only package-export cleanup now removes the default `collision-fcl`
fallback and generated old collision component text; the install probe reports
only `DART_BUILD_COLLISION_*` variables as `OFF` in searched installed
CMake/pkg-config metadata. Normal pixi configure paths now also default FCL,
Bullet, ODE, reference correctness tests, and reference benchmarks to `OFF`,
while explicit `DART_BUILD_COLLISION_*_OVERRIDE=ON` settings restore the
reference component, test, and benchmark targets for comparison jobs. A fresh
native-only runtime install probe now also shows no old collision component
files, no old collision package-export references beyond `OFF` state variables,
and no FCL, Bullet, ODE, or libccd runtime links from installed shared
libraries or the built dartpy extension. Default and wheel Pixi dependency
metadata now omit FCL, Bullet, ODE, and their FCL transitive packages; the
explicit `collision-reference` environment owns those packages and its focused
`test_reference_backends` target builds and passes. A repaired py312 wheel
artifact built with old collision engines and reference harnesses disabled
imports successfully and contains no old collision component files, old
collision CMake exports, or FCL, Bullet, ODE, or libccd runtime links. Python
detector compatibility names now resolve to `DartCollisionDetector`, and
dartpy no longer links legacy collision component targets even in a
reference-enabled build.

## Current Branch

`feature/new_coll` - local branch tracking `origin/feature/new_coll`, ahead by
the native-collision checkpoint commits in this task. Use `git status -sb` for
the exact count.

## Immediate Next Step

Continue from `docs/dev_tasks/native_collision/04-reference-gap-analysis.md`.
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
evidence and downstream-component checks.
Collision abstraction cleanup has also started: `DartCollisionDetector` owns
native-backed public factory aliases for `experimental`, `fcl`, `fcl_mesh`,
`bullet`, and `ode`; the FCL/Bullet/ODE component registrars also publish
native-backed creators so linking old reference component libraries cannot
restore public backend selection; and the Python compatibility names
`DARTCollisionDetector`, `FCLCollisionDetector`, `BulletCollisionDetector`, and
`OdeCollisionDetector` now construct/report the built-in `dart` detector.
Reference tests and benchmarks now use explicit `createReference()` APIs on the
FCL, Bullet, and ODE detector classes, and direct public C++ legacy detector
`create()` paths now return the built-in `DartCollisionDetector`. Legacy C++
detector classes, headers, CMake component surfaces, and old-engine source
placement still contain explicit reference-engine implementations and remain a
north-star cleanup gate.
User-facing examples have also been moved off the old collision components:
their CMake no longer requires `collision-bullet`/`collision-ode`, source no
longer includes or selects Bullet/ODE/FCL detector APIs, and affected examples
build in the default native-only configuration. `human_joint_limits` is still
skipped in this environment because TinyDNN is unavailable.
`docs/dev_tasks/native_collision/01-design.md` is the canonical architecture
contract for that remaining cleanup: public API and compatibility facades
outside the DART adapter, `dart/collision/native/` as the scene/query core, and
explicit gates for API cleanliness, scalability, performance hooks, reference
isolation, and gz-physics-safe compatibility facades.
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
availability.

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
  `ldd` checks found no FCL, Bullet, ODE, or libccd links.
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
- A fresh gz-physics run passed 65/65 tests and the plugin link check.
- Tests and benchmark comments now use native collision naming except for the
  explicit `"experimental"` compatibility alias.
- gz-physics compatibility and performance parity are explicit gates, not
  optional follow-up work.
- The next work is no longer feature-parity implementation. The single PR still
  needs CI hardening, optional reference-engine test/benchmark isolation with
  CMake opt-out support, backend removal from default packaging, downstream
  migration/deprecation coverage, `dart/collision/` abstraction cleanup so all
  retained legacy names route to the built-in detector, built-in collision
  layer cleanup for API cleanliness, scalable native scene/query state and
  performance-oriented internals, recurring performance guardrails, final
  legacy runtime backend deletion, and then final PR packaging.
- Quality order for the remaining work: feature coverage first, correctness
  tests as the permanent gate, then gradual benchmark-driven performance work
  until native beats Bullet, FCL, and ODE on required workloads.
- Built-in architecture target: public DART APIs and compatibility facades stay
  outside the core, `dart/collision/dart/` adapts DART shapes/results/filters,
  and `dart/collision/native/` owns scene state, broadphase, narrowphase,
  query algorithms, caches, deterministic results, and profiling/benchmark
  hooks.
- `04-reference-gap-analysis.md` captures the concrete gaps found from the
  reference capability comparison. The first adapter gap is started in code:
  `DartCollisionGroup` owns persistent native scene state and routes public
  collision, distance, and raycast through it, with broadphase-pruned public
  raycast candidates, AABB lower-bound distance pruning, and scene-issued
  manifold cache IDs. The public adapter benchmark target covers collision,
  dirty-world collision, distance, and raycast. Shape invalidation/behavior
  coverage now covers primitives, mesh scale, soft meshes, heightmaps, voxel
  grids, and point clouds. DART collision filters now flow through a native
  filter adapter before narrowphase. Remaining adapter gaps include recurring
  benchmark/profiling guardrails for the public DART API path.
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
  no FCL, Bullet, ODE, or libccd libraries and no old collision component
  target files. The latest package-export cleanup also removed the implicit
  `collision-fcl` fallback from default package initialization and removed old
  Bullet/ODE component text from generated component metadata; the follow-up
  install metadata search found only `DART_BUILD_COLLISION_*` variables set to
  `OFF`. The fresh runtime install probe now broadens that evidence to all
  installed native shared libraries, dartpy, target-help, installed package
  metadata from a clean build tree, and Pixi default/wheel lock metadata.
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
