# DART 6.20 Dependency Minimization Plan

## Branch policy and history

This task is for the DART 6.20 support lane.

- Historical planning branches
  `plan/dart6-dependency-minimization-6.20` and
  `plan/dart6-native-collision-port` are evidence only. Do not resume them.
- PR #3381's implementation branch
  `feature/dart-detector-consolidation` is merged. Do not add later phases to
  it.
- New work must use a fresh non-tracking topic branch from the explicitly
  authorized target release. The absence of a future release branch is a stop
  condition, not permission to place a default flip on `release-6.20`.
- DART 7 reference: use `origin/main` directly, or create a local worktree from
  `origin/main` for comparisons. Do not rely on a developer-specific checkout
  path.
- Do not rely on any per-machine worktree directory name for comparisons; use
  `origin/main` or remote refs (or your own separate `main` worktree) as
  comparison evidence only.

Evidence was first collected on 2026-06-19 after fetching `origin/main`,
`origin/release-6.20`, and `origin/release-6.19`. The native-collision port
evidence in `03-native-collision-port-scoping.md` and the dashboard state in
`04-orchestration-dashboard.md` were refreshed through the verified merge of
PR #3381 on 2026-07-30.

## Current Branch State

- `origin/release-6.20` currently points at
  `ac7b9462612a9ef54eeb9d6841375c7789cf23d8`. That tip contains the
  `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5` squash merge of
  [PR #3381](https://github.com/dartsim/dart/pull/3381) plus #3406's
  formatter-only post-merge lint repair.
- `origin/main` currently points at
  `83110ef54abf41f54c1e03500e49c1c12c305b8a`.
- `package.xml` and `pixi.toml` on `origin/release-6.20` still report a 6.19.x
  package version (`6.19.4`).
- DART 6.20 intentionally uses the release lane while package version metadata
  catches up to the branch/milestone naming.
- Collision status: PR #3381 consolidated the DART-owned engine into
  `DARTCollisionDetector`, with `"dart"` as its only factory key. The interim
  `"native"` key was introduced only on the unreleased 6.20 branch and was
  removed rather than carried as an alias. FCL remains the 6.20 default and a
  core dependency; the FCL, Bullet, and ODE implementations, components,
  dependencies, and default-selection paths are unchanged.
- Remaining collision-dependency work is deliberately later-release work.
  Phase 6 must first prove and receive approval for a default flip on its
  proposing release; phase 7 can then decouple FCL and implement the
  compatibility-facade/package plan. As of 2026-07-30, no `release-6.21` or
  `release-6.22` branch or milestone exists, so those phases are not executable
  on the DART 6.20 support branch.
- The separate default-environment OSG demotion was not completed by #3116.
  That PR removed GLUT and `lodepng` while retaining OSG. The 2026-06-21
  maintainer decision recorded in #3107 held all interim Pixi demotions, and
  no later authorization supersedes it. PR #3393 proves that a C++ build with
  GUI OSG disabled can succeed; it does not move OSG out of the default Pixi
  environment, and default dartpy still hard-requires `dart-gui-osg`.
- A live PR and ancestry audit confirmed that every listed phase-0–5 merge
  commit and every completed earlier dependency-minimization slice is an
  ancestor of the current release tip. PR #3106 is the deliberate exception:
  its Bullet/ODE-only Pixi demotion was closed unmerged by maintainer decision.
- Local post-merge closeout is on
  `docs/dependency-minimization-closeout`, one unpublished documentation-only
  commit ahead of `origin/release-6.20`. It has no upstream or PR; publishing
  it requires explicit maintainer approval.

The durable architecture and compatibility rationale now lives in
[DART 6 Collision Backend Consolidation](../../design/dart6_collision_backend_consolidation.md).

## DART 6 Dependency Inventory

`package.xml` on `release-6.20` currently lists these package dependencies:

- Required or exported build dependencies: `assimp`, `eigen`, `libfcl-dev`,
  `liburdfdom-dev`, and `tinyxml2`.
- Optional or component-related package dependencies that are still advertised:
  `bullet`.
- Already removed from the package manifest on this baseline: `glut`,
  `libxi-dev`, and `libxmu-dev`.

`pixi.toml` on `release-6.20` keeps a broad default environment. The important
project dependencies include:

- Core and package-surface dependencies: `assimp`, `eigen`, `fcl`, `fmt`,
  `urdfdom`, `tinyxml2`, and optional `spdlog`.
- Collision and constraint ecosystem dependencies: `bullet-cpp`, `libode`,
  `octomap`, and `fcl`.
- GUI dependencies: `openscenegraph`, `imgui`, and the OSG/OpenGL stack pulled
  through those packages.
- Already removed from the default Pixi environment on this baseline:
  `freeglut`, `ipopt`, `nlopt`, and `pagmo-devel`.
- Test, build, and docs tools: CMake, Ninja, GoogleTest, Google Benchmark,
  Doxygen, Sphinx, pytest, and related Python tooling.

CMake on `release-6.20` makes `fmt`, Eigen, FCL, and Assimp part of the core
configure/build path. Bullet, ODE, GUI, OctoMap, and ImGui are component or
feature surfaces, but the default Pixi environment still makes many of them
available by default.

## DART 6 `dart/external` Inventory

`release-6.20` started with these vendored source trees; the current baseline
has retired the top-level `dart/external/` source directory, but the list is
kept here as historical task context:

- `dart/external/convhull_3d`
- `dart/external/ikfast`
- `dart/external/imgui`
- `dart/external/lodepng`
- `dart/external/odelcpsolver`

Usage summary:

- `convhull_3d` was included by the math geometry implementation and has been
  replaced by DART-owned native math detail code.
- `ikfast` moved to DART's dynamics path with a compatibility forwarding
  header for the old installed include path.
- `imgui` now provides the DART 6 `external-imgui` compatibility component from
  the system package by default, including headless package builds, and uses a
  DART-patched fetched copy when `DART_USE_SYSTEM_IMGUI` is disabled.
- `lodepng` was removed with the GLUT screenshot path.
- `odelcpsolver` was replaced in production by DART-owned LCP solver code, with
  legacy ODE code retained only where needed as test baseline material.

## DART 7 Reference State

The main checkout has no `dart/external/` directory.

DART 7 moved or removed the old vendored surfaces as follows:

- Convex hull code moved into DART-owned native math detail files.
- IKFast moved to a DART-owned dynamics header path.
- ImGui is resolved through the GUI dependency/fetch path instead of an
  in-tree `dart/external` copy.
- The legacy GLUT GUI stack was removed after the remaining GLUT examples were
  converted to OSG. The relevant DART 7 reference PRs are:
  - [#2044](https://github.com/dartsim/dart/pull/2044): removed all GLUT code,
    removed GLUT CMake/dependency plumbing, converted the remaining GLUT
    examples to OSG, and made OSG the exclusive GUI backend at that point.
  - [#2051](https://github.com/dartsim/dart/pull/2051): removed `lodepng`
    after GLUT screenshot handling disappeared.
  - [#2203](https://github.com/dartsim/dart/pull/2203): refreshed tutorials,
    manifests, dependency tooling, and onboarding docs to remove GLUT-era
    references.
- ODE LCP solver code became test-baseline/reference material after the DART 7
  solver path moved to DART-owned LCP/math infrastructure.

DART 7 also removes Bullet, FCL, GLUT/Xi/Xmu, and ODE from the normal package
dependency path. Gazebo compatibility testing still carries FCL, Bullet, and
ODE in the Gazebo feature lane.

## DART 6 Constraints

- DART 6.20 is a compatibility support branch. It cannot assume DART 7
  clean-break latitude.
- Gazebo/gz-physics compatibility is a release-branch constraint. Any package,
  collision, constraint, or default-solver change that can affect gz-physics
  needs the Gazebo gate before landing.
- Exported package components are public behavior. If
  `find_package(DART COMPONENTS collision-bullet collision-ode gui-osg ...)`
  changes, the PR needs an explicit compatibility decision and migration note.
- Installed public headers are source compatibility surfaces. In particular,
  FCL/Bullet/ODE collision headers and the old IKFast include path cannot be
  removed casually from a 6.x branch.
- The default `World` path and gz-physics compatibility still depend on the
  boxed LCP constraint-solver behavior. The ODE LCP-derived solver code is
  high-risk even if the folder name is undesirable.

## Completed and deferred slices

### First Slices

`dart/external/convhull_3d`

- Risk: low.
- Status: complete in #3076.
- Result: replaced the vendored C implementation with the DART-owned native
  `dart/math/detail/ConvexHull.hpp` implementation adapted from DART 7,
  updated the geometry include, and kept the legacy C implementation only under
  `tests/unit/math/legacy_convhull_3d` for regression coverage.
- Compatibility decision: intentionally remove the previously installed
  `dart/external/convhull_3d/convhull_3d.h` and
  `dart/external/convhull_3d/safe_convhull_3d.h` headers as a breaking
  dependency-removal slice instead of keeping forwarding headers under
  `dart/external`.
- Validation: `pixi run build-tests`, focused `UNIT_math_ConvexHull` and
  `UNIT_math_TriMesh`, plus the branch-required formatting gate.

Default Pixi/package metadata for optional components

- Risk: low to medium, depending on the surface.
- Status: the optimizer packages were removed; the Bullet/ODE-only demotion was
  abandoned because FCL remains core; GLUT/freeglut was removed with the GUI
  migration; OSG and ImGui remain in the default environment by explicit hold;
  OctoMap remains part of the exported-header contract.
- Later work: move FCL/Bullet/ODE packages only as part of the approved
  default-flip/facade sequence, not as an isolated DART 6.20 environment edit.
- Validation: default configure/build, explicit feature configure/build for each
  moved surface, package-component smoke tests, and Gazebo when the change is
  visible to downstream package discovery.

### Medium-Risk Slices

`dart/external/ikfast`

- Risk: medium because the old installed include path may be source-visible.
- Status: complete in #3078.
- Result: moved the header to the DART-owned `dart/dynamics/ikfast.h` path,
  updated in-repo includes, removed the source `dart/external/ikfast` tree, and
  generated
  a forwarding header at the old `dart/external/ikfast` path for build-tree and
  installed DART 6 source compatibility.
- Validation: IKFast integration tests, generated WAM example builds, and an
  installed-header smoke if the old include path changes.

`dart/external/imgui`

- Risk: medium because the OSG GUI path remains part of DART 6.
- Status: complete in #3081.
- Result: made the packaged/system ImGui path the default for normal GUI builds,
  preserved the `external-imgui` component plus old installed
  `dart/external/imgui` include path through a system-backed compatibility
  target in default builds, and used an explicit DART-patched FetchContent
  target when `DART_USE_SYSTEM_IMGUI=OFF`.
- Validation: default configure/build with system ImGui, headless system-ImGui
  build of `dart-external-imgui`, explicit fetched-ImGui fallback
  configure/build, OSG GUI target build, install-tree headers, and package
  smoke proving ImGui headers are found through `find_package(DART COMPONENTS
  external-imgui gui-osg)`.

`dart/external/lodepng`

- Risk: medium to high because it is tied to legacy GUI screenshot behavior.
- Status: complete in #3116. GLUT-supported behavior moved to OSG and the
  `lodepng` tree was removed as part of the same migration.
- Validation: OSG screenshot smoke, all migrated GUI examples/tutorials, package
  metadata proving `glut`, `libxi-dev`, `libxmu-dev`, and `freeglut` are no
  longer advertised, and install/package smokes proving the remaining GUI
  component is OSG-only.

### High-Risk Or Deferred Slices

FCL in DART 6 core

- Risk: high.
- Status: deliberately retained in DART 6.20. PR #3381 consolidated the
  DART-owned engine but kept FCL as the default and core dependency.
- Later plan: do not remove FCL until a future release accepts the complete
  default flip and then passes the phase-7 package, API, ABI, and gz gates.
- Validation: native-collision parity, installed package/component smokes, full
  collision tests, dartpy coverage, and Gazebo.

Bullet and ODE collision components

- Risk: high for removal, lower for default-environment demotion.
- Status: retained as real backends and components in DART 6.20.
- Later plan: preserve their public classes, factory keys, and component names
  through the approved facade lifecycle. ODE remains gated on its gz
  subclassing decision.
- Validation: component configure/build with dependencies absent and present,
  examples/tutorials that request those components, package smoke, and Gazebo.

`dart/external/odelcpsolver`

- Risk: very high.
- Status: complete in #3088.
- Result: replaced the production ODE-style source tree with the DART-owned
  native Dantzig kernel ported from DART 7, wired through DART 6's existing
  `dart/lcpsolver` and constraint APIs.
- Installed only the DART-owned `dart/lcpsolver/dantzig` headers and sources.
  The original ODE implementation remains only as a
  test-only baseline under `tests/baseline/odelcpsolver` for parity,
  correctness, and performance-comparison evidence.
- Validation: full unit tests for constraints/contact/dynamics, focused LCP
  solver regressions, dartpy smoke where constraint solver types are bound, and
  Gazebo.

OpenSceneGraph and GLUT GUI dependencies

- Risk: high for API removal, medium for default-environment demotion.
- Status: GLUT removal is complete in #3116. DART 6.20 removed GLUT and its
  package/header surface after migrating supported behavior to OSG;
  `lodepng` was removed in the same PR. OSG demotion is not complete: OSG,
  ImGui, and their transitive stack remain in the default Pixi environment,
  and default dartpy still hard-requires `dart-gui-osg`.
- Scope decision: #3107 records the 2026-06-21 maintainer decision to hold all
  interim Pixi demotions. PR #3393 later added a forced no-OSG C++ CI gate, but
  did not authorize or implement the default-environment split. Reactivating
  this slice requires fresh maintainer scope, including the dartpy source
  split and wheel/publishing environment contract.
- Validation: OSG feature-parity smoke for every migrated GLUT example,
  tutorial/example builds, GUI screenshot smoke, package-component smoke for
  `gui-osg`, absence checks for installed `dart/gui/glut` headers and top-level
  GLUT forwarding headers such as `dart/gui/GlutWindow.hpp`,
  `dart/gui/SimWindow.hpp`, `dart/gui/Win2D.hpp`, and `dart/gui/Win3D.hpp`, and
  package smokes proving the public `gui`/`dart-gui` component no longer
  advertises or links GLUT while `gui-osg` remains usable. Also run a default
  configure/build without GLUT packages and an explicit GUI build with OSG
  enabled.

## Sequenced Workstreams

1. **Branch and package baseline: complete.** The release lane, package state,
   gz-physics gate, and dependency inventory are recorded in this task packet.
2. **Default-environment split: partially complete.** Optional optimizer,
   GLUT, and vendored-source cleanup landed. OSG demotion remains held by the
   #3107 maintainer decision. Collision package demotion is intentionally
   coupled to the future default-flip/facade sequence because FCL remains core
   in DART 6.20.
3. **Low-risk DART-owned replacements: complete.** `convhull_3d` and IKFast
   landed in #3076 and #3078, including the required IKFast compatibility
   forwarding path.
4. **ImGui vendored-source removal: complete.** The system-backed default and
   explicit patched-fetch fallback landed in #3081.
5. **GLUT-to-OSG migration and screenshot cleanup: complete.** GLUT and
   `lodepng` were removed in #3116 after supported behavior moved to OSG.
6. **Collision detector consolidation: complete for DART 6.20.** PR #3381
   consolidated the DART-owned implementation behind
   `DARTCollisionDetector`, retained only the canonical `"dart"` factory key,
   and preserved FCL as the default and core dependency. Performance
   exploration is closed for this lane. The default flip and FCL dependency
   decoupling remain ordered later-release phases.
7. **LCP solver cleanup: complete.** The production `odelcpsolver` tree was
   replaced by the DART-owned Dantzig kernel in #3088; legacy ODE code remains
   only as test-baseline evidence.

## Implementation Gates

Select gates by touched surface:

- Always run the branch-required formatting gate before committing.
- Run a default configure/build gate after package or dependency changes.
- Run focused C++ unit tests for the touched module, then broaden to unit tests
  for collision, constraint, solver, package, or GUI behavior changes.
- Run Python tests when dartpy bindings, package exports, or linked
  optimizer/collision targets change.
- Run explicit feature-environment configure/build tests for every dependency
  moved out of the default environment.
- Run installed-package/component smoke tests whenever package metadata,
  exported CMake targets, installed headers, or component discovery changes.
- Run the Gazebo gate for package, collision, constraint, or default-solver
  surfaces that can affect gz-physics:

  ```bash
  N=${DART_SAFE_JOBS:-$(python3 scripts/parallel_jobs.py)}
  DART_PARALLEL_JOBS=$N CTEST_PARALLEL_LEVEL=$N pixi run -e gazebo test-gz
  ```

## Non-Goals

- Do not port the full DART 7 GUI, C++23 architecture, or native-collision stack
  into DART 6 as part of this cleanup.
- Do not remove DART 6 public package components or installed include paths
  without an explicit compatibility decision.
- Do not treat deletion of `dart/external/` as evidence that runtime behavior is
  simpler. Some code, especially LCP solver code, may need to move under a
  DART-owned internal path before it can be replaced.

## Open Decisions

- Which future DART 6 release is authorized to propose the
  `DARTCollisionDetector` default flip? No such release branch or milestone
  exists as of 2026-07-30.
- What exact parity, performance, ABI, package, and gz-physics evidence will
  that release require before accepting the default flip?
- Can the ODE compatibility facade preserve the current gz-physics subclassing
  contract, or does it require a coordinated downstream migration?
- After a default flip has shipped and stabilized, which later release may
  deprecate or remove compatibility classes, factory keys, components, and
  dependency exports?
- Will a separately authorized task demote OSG from the default Pixi
  environment while preserving the public `gui-osg` component and the
  published dartpy GUI surface? The current task does not supply that
  authorization.

Resolved decisions remain part of the acceptance record:

- DART 6.20 keeps FCL as its default collision detector and core dependency.
- The authoritative downstream gate is the pinned
  `pixi run -e gazebo test-gz` environment.
- The old IKFast installed include path remains available through a forwarding
  header.
- The ImGui system-backed default and explicit patched-fetch fallback are both
  preserved.
- GLUT and `lodepng` were removed only after supported behavior moved to OSG;
  that migration did not demote OSG itself.
- The temporary public `"native"` factory key was removed before release;
  `"dart"` is the sole DART-owned detector key.
