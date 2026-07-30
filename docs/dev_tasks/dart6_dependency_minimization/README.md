# DART 6.20 Dependency Minimization Plan

## Working Branches

This task has no active implementation branch after merged PR #3381. This
folder remains the project home for the unresolved later-release decision; do
not recreate or resume historical Phase 0–4 branches.

- Before future implementation, obtain maintainer approval for the target
  release, fetch it, and create a fresh topic branch from that exact remote tip.
- DART 7 reference: use `origin/main` directly, or create a local worktree from
  `origin/main` for comparisons. Do not rely on a developer-specific checkout
  path.
- Do not rely on any per-machine worktree directory name for comparisons; use
  `origin/main` or remote refs (or your own separate `main` worktree) as
  comparison evidence only.

Evidence was first collected on 2026-06-19 after fetching `origin/main`,
`origin/release-6.20`, and `origin/release-6.19`. The native-collision port
evidence in `03-native-collision-port-scoping.md` and the dashboard state in
`04-orchestration-dashboard.md` were refreshed on 2026-07-09 after fetching
`origin/main` and `origin/release-6.20`, after phase-3 D5 plus the native
dashboard fix (#3363) merged, and after the first Phase 4 optimization slice
(#3364) landed. The active owner state was refreshed again on 2026-07-29 after
the detector consolidation (#3381) merged.

## Current Branch State

- `origin/release-6.20` currently points at
  `ac7b9462612a9ef54eeb9d6841375c7789cf23d8` (#3406, after #3381).
- `origin/main` currently points at
  `83110ef54abf41f54c1e03500e49c1c12c305b8a`.
- `package.xml` and `pixi.toml` on `origin/release-6.20` still report a 6.19.x
  package version (`6.19.4` at this refresh).
- DART 6.20 intentionally uses the release lane while package version metadata
  catches up to the branch/milestone naming.
- Collision status: merged PR #3381 consolidates the DART-owned engine into
  `DARTCollisionDetector`, with `"dart"` as its only factory key. The interim
  `"native"` key was introduced only on the unreleased 6.20 branch and is
  removed by the consolidation rather than carried as an alias. FCL remains
  the 6.20 default and a core dependency; a default flip and dependency
  removal are deferred until a later release and their full acceptance gates.
  The final PR head was `64d476b68ad5ae0dcca4e98abb9bba15b6962b87`;
  the release merge is `46719bfbd75e1f70e69b2c76fb34a3fa2b78edd5`.

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

## Historical removal work and deferred collision work

The standalone-removal entries below are retained as design history, not as
active branch instructions. Their authoritative status is:

| Area | Current status |
| --- | --- |
| Convex hull replacement | ✅ merged (#3076) |
| IKFast relocation/forwarder | ✅ merged (#3078) |
| ImGui vendored-source replacement | ✅ merged (#3081) |
| ODE LCP production replacement | ✅ merged (#3088) |
| GLUT and lodepng removal | ✅ merged (#3116) |
| DART-owned collision consolidation | ✅ merged (#3381); FCL remains default |
| Default flip and FCL/Bullet/ODE dependency reduction | ⏸ deferred to an approved later release after Phase 5 ratification |

### Completed first slices

`dart/external/convhull_3d`

- Risk: low.
- Status: completed by #3076.
- Result: replaced the vendored C implementation with the DART-owned native
  `dart/math/detail/ConvexHull.hpp` implementation adapted from DART 7, updated
  the geometry include, and kept the legacy C implementation only under
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
- Plan: move optional packages out of the default path one at a time only after
  proving the default configure still succeeds and explicit feature/component
  environments keep their tests.
- Deferred candidates for feature-only treatment: Bullet and ODE, only as part
  of the approved later-release collision plan. Optimizer packages were already
  removed; GUI and OctoMap contracts require separate live owner evidence.
- Validation: default configure/build, explicit feature configure/build for each
  moved surface, package-component smoke tests, and Gazebo when the change is
  visible to downstream package discovery.

### Completed medium-risk slices

`dart/external/ikfast`

- Risk: medium because the old installed include path may be source-visible.
- Status: completed by #3078.
- Result: moved the header to the DART-owned `dart/dynamics/ikfast.h` path,
  updated in-repo includes, removed the source `dart/external/ikfast` tree, and
  generated a forwarding header at the old `dart/external/ikfast` path for
  build-tree and installed DART 6 source compatibility.
- Validation: IKFast integration tests, generated WAM example builds, and an
  installed-header smoke if the old include path changes.

`dart/external/imgui`

- Risk: medium because the OSG GUI path remains part of DART 6.
- Status: completed by #3081.
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

- Risk: medium to high because it was tied to legacy GUI screenshot behavior.
- Status: completed with the GLUT removal in #3116. Do not revive standalone
  `lodepng` work.
- Validation: OSG screenshot smoke, all migrated GUI examples/tutorials, package
  metadata proving `glut`, `libxi-dev`, `libxmu-dev`, and `freeglut` are no
  longer advertised, and install/package smokes proving the remaining GUI
  component is OSG-only.

### High-Risk Or Deferred Slices

FCL in DART 6 core

- Risk: high.
- Plan: do not remove FCL from core until maintainers ratify Phase 5 and a later
  default-flip packet passes package, API, and gz-physics gates.
- Validation: native-collision parity, installed package/component smokes, full
  collision tests, dartpy coverage, and Gazebo.

Bullet and ODE collision components

- Risk: high for removal, lower for default-environment demotion.
- Plan: retain the real components until Phase 5 and the default flip are
  accepted; any later facade/package change must preserve component discovery
  and downstream behavior.
- Validation: component configure/build with dependencies absent and present,
  examples/tutorials that request those components, package smoke, and Gazebo.

`dart/external/odelcpsolver`

- Risk: very high.
- Status: completed by #3088.
- Result: replaced the production ODE-style source tree with the DART-owned native
  Dantzig kernel ported from DART 7, wired through DART 6's existing
  `dart/lcpsolver` and constraint APIs.
- The merged implementation installs only the DART-owned `dart/lcpsolver/dantzig`
  headers and sources. It keeps the original ODE implementation only as a
  test-only baseline under `tests/baseline/odelcpsolver` for parity,
  correctness, and performance-comparison evidence.
- Validation: full unit tests for constraints/contact/dynamics, focused LCP
  solver regressions, dartpy smoke where constraint solver types are bound, and
  Gazebo.

OpenSceneGraph and GLUT GUI dependencies

- Risk: high for API removal, medium for default-environment demotion.
- Status: completed by #3116. GLUT code, headers, package metadata, and the
  lodepng screenshot path are removed; OSG remains the supported DART 6 GUI
  surface.
- Validation: OSG feature-parity smoke for every migrated GLUT example,
  tutorial/example builds, GUI screenshot smoke, package-component smoke for
  `gui-osg`, absence checks for installed `dart/gui/glut` headers and top-level
  GLUT forwarding headers such as `dart/gui/GlutWindow.hpp`,
  `dart/gui/SimWindow.hpp`, `dart/gui/Win2D.hpp`, and `dart/gui/Win3D.hpp`, and
  package smokes proving the public `gui`/`dart-gui` component no longer
  advertises or links GLUT while `gui-osg` remains usable. Also run a default
  configure/build without GLUT packages and an explicit GUI build with OSG
  enabled.

## Historical workstream sequence

Steps 1–5 and 7 are complete; collision consolidation in step 6 is complete
through #3381. The only active owner action is Phase 5 later-release
ratification, not replaying this sequence.

1. **Branch and package baseline** established the release and gz lane.
2. **Default-environment split** removed optimizer defaults; collision
   dependency work remains deferred.
3. **Low-risk native replacements** completed convhull and IKFast migration.
4. **ImGui vendored-source removal** completed the system/fetch compatibility
   path.
5. **GLUT-to-OSG migration and screenshot cleanup** removed GLUT and lodepng.
6. **Collision dependency reduction** completed consolidation but intentionally
   retained the default and external backends pending later-release decisions.
7. **LCP solver cleanup** replaced production ODE code with DART-owned Dantzig
   code and retained test-only baselines where required.

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

- Do not port the DART 7 GUI or C++23 architecture into DART 6. The collision
  algorithms already consolidated into DART 6 were deliberately adapted to its
  C++17/API/ABI constraints; do not reopen that port.
- Do not remove DART 6 public package components or installed include paths
  without an explicit compatibility decision.
- Do not treat deletion of `dart/external/` as evidence that runtime behavior is
  equivalent; preserve the merged parity baselines and focused tests.

## Current decision boundary

- Ratify facade-over-dart for `OdeCollisionDetector` versus a coordinated
  gz-physics change, and ratify the target later-release sequence. The
  6.21/6.22 labels in `08-phase5-facade-decision.md` are proposals, not an
  approved schedule.
- Do not implement a default flip or dependency removal on `release-6.20`.
- IKFast/ImGui future-major compatibility questions belong to their API/package
  owners and are not active work in this folder.
- GLUT/lodepng removal is resolved and merged (#3116).
