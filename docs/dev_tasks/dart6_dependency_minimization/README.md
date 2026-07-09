# DART 6.20 Dependency Minimization Plan

## Working Branches

This task is for the DART 6.20 support lane.

- Implementation branch:
  `plan/dart6-dependency-minimization-6.20`, based on
  `origin/release-6.20`.
- Native-collision planning refresh branch:
  `plan/dart6-native-collision-port`, based on `origin/release-6.20`.
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
`origin/main` and `origin/release-6.20`, and after phase-3 D4 merged.

## Current Branch State

- `origin/release-6.20` currently points at
  `1a33843125dde544c1a1c4caa11fae71af35b6e5`.
- `origin/main` currently points at
  `a70fc2ed5cb7ea40f72dce68b7d374583ab7feee`.
- `package.xml` and `pixi.toml` on `origin/release-6.20` still report a 6.19.x
  package version (`6.19.3` in the current CMake configure output).
- DART 6.20 intentionally uses the release lane while package version metadata
  catches up to the branch/milestone naming.

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

## Removal Candidates

### First Slices

`dart/external/convhull_3d`

- Risk: low.
- Status: in progress on `chore/replace-convhull-3d-release-6.20`.
- Plan: replace the vendored C implementation with the DART-owned native
  `dart/math/detail/ConvexHull.hpp` implementation adapted from DART 7, update
  the geometry include, and keep the legacy C implementation only under
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
- Remaining candidates for feature-only treatment before code removal: Bullet,
  ODE, GUI packages, and OctoMap. Optimizer packages were already removed from
  the default environment.
- Validation: default configure/build, explicit feature configure/build for each
  moved surface, package-component smoke tests, and Gazebo when the change is
  visible to downstream package discovery.

### Medium-Risk Slices

`dart/external/ikfast`

- Risk: medium because the old installed include path may be source-visible.
- Status: in progress on `chore/replace-ikfast-release-6.20`.
- Plan: move the header to the DART-owned `dart/dynamics/ikfast.h` path, update
  in-repo includes, remove the source `dart/external/ikfast` tree, and generate
  a forwarding header at the old `dart/external/ikfast` path for build-tree and
  installed DART 6 source compatibility.
- Validation: IKFast integration tests, generated WAM example builds, and an
  installed-header smoke if the old include path changes.

`dart/external/imgui`

- Risk: medium because the OSG GUI path remains part of DART 6.
- Status: in progress on `chore/replace-imgui-release-6.20`.
- Plan: make the packaged/system ImGui path the default for normal GUI builds,
  preserve the `external-imgui` component plus old installed
  `dart/external/imgui` include path through a system-backed compatibility
  target in default builds, and use an explicit DART-patched FetchContent
  target when `DART_USE_SYSTEM_IMGUI=OFF`.
- Validation: default configure/build with system ImGui, headless system-ImGui
  build of `dart-external-imgui`, explicit fetched-ImGui fallback
  configure/build, OSG GUI target build, install-tree headers, and package
  smoke proving ImGui headers are found through `find_package(DART COMPONENTS
  external-imgui gui-osg)`.

`dart/external/lodepng`

- Risk: medium to high because it is tied to legacy GUI screenshot behavior.
- Plan: do not revive standalone `lodepng` replacement work while GLUT remains.
  DART 6.20 should remove GLUT completely and migrate every GLUT-supported
  feature to OSG first; once OSG owns screenshot output, `lodepng` should be
  removed as a consequence, following the DART 7 sequence in #2044 and #2051.
- Validation: OSG screenshot smoke, all migrated GUI examples/tutorials, package
  metadata proving `glut`, `libxi-dev`, `libxmu-dev`, and `freeglut` are no
  longer advertised, and install/package smokes proving the remaining GUI
  component is OSG-only.

### High-Risk Or Deferred Slices

FCL in DART 6 core

- Risk: high.
- Plan: do not remove FCL from DART 6 core unless maintainers accept a larger
  native-collision backport with package, API, and gz-physics evidence.
- Validation: native-collision parity, installed package/component smokes, full
  collision tests, dartpy coverage, and Gazebo.

Bullet and ODE collision components

- Risk: high for removal, lower for default-environment demotion.
- Plan: first move their packages out of the default Pixi/package path while
  keeping component builds working when dependencies are explicitly installed.
- Validation: component configure/build with dependencies absent and present,
  examples/tutorials that request those components, package smoke, and Gazebo.

`dart/external/odelcpsolver`

- Risk: very high.
- Plan: replace the production ODE-style source tree with the DART-owned native
  Dantzig kernel ported from DART 7, wired through DART 6's existing
  `dart/lcpsolver` and constraint APIs.
- Current PR direction: install only the DART-owned `dart/lcpsolver/dantzig`
  headers and sources. Keep the original ODE implementation only as a
  test-only baseline under `tests/baseline/odelcpsolver` for parity,
  correctness, and performance-comparison evidence.
- Validation: full unit tests for constraints/contact/dynamics, focused LCP
  solver regressions, dartpy smoke where constraint solver types are bound, and
  Gazebo.

OpenSceneGraph and GLUT GUI dependencies

- Risk: high for API removal, medium for default-environment demotion.
- Owner decision: DART 6.20 should completely remove GLUT. All features
  supported by GLUT must be supported by OSG before removal, including example
  interactivity, keyboard/mouse event handling, run-loop behavior, overlays or
  HUDs, screenshots/PNG output, and tutorial coverage.
- Plan: use DART 7 PR #2044 as the migration reference for converting the
  remaining GLUT examples to OSG and deleting `dart/gui/glut`, GLUT headers,
  CMake find modules, package metadata, and dependency declarations. Use #2203
  as the reference for tutorial, manifest, and dependency-doc cleanup after the
  code migration lands. Treat #2051 as the follow-up `lodepng` removal once no
  GLUT screenshot path remains.
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

1. **Branch and package baseline.** Confirm the 6.20 branch/version owner state
   and record the gz-physics version lane for 6.20 before dependency removals.
2. **Default-environment split.** Move optional dependencies out of the default
   Pixi/package path one surface at a time, preserving explicit feature
   environments and component builds.
3. **Low-risk native replacements.** Replace `convhull_3d` with DART-owned
   native math detail code, then move IKFast with a compatibility-forwarder
   decision.
4. **ImGui vendored-source removal.** Prefer system ImGui or an approved fetch
   fallback while preserving the DART 6 OSG GUI component.
5. **GLUT-to-OSG migration and screenshot cleanup.** Migrate every remaining
   GLUT example/tutorial/feature to OSG, remove the GLUT component and package
   metadata, then remove `lodepng` once screenshot output is owned by OSG.
6. **Collision dependency reduction.** Treat Bullet/ODE package moves as
   optional-component packaging work. Treat FCL removal as a native-collision
   backport decision, not a routine cleanup.
7. **LCP solver cleanup.** Replace the production `odelcpsolver` tree with the
   DART-owned native Dantzig kernel, and keep any old ODE code under tests only
   when it is needed as a parity or performance baseline.

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

- Should branch/version metadata move to 6.20 before dependency cleanup begins?
- Which gz-physics lane is the authoritative DART 6.20 downstream gate?
- For IKFast, can the old installed `dart/external/ikfast/ikfast.h` path be
  removed in a future major release after DART 6 keeps compatibility by
  forwarding it to `dart/dynamics/ikfast.h` in the build and install trees?
- For ImGui, should DART 6 keep the DART-patched FetchContent compatibility
  target as the default long-term, or require a patched system package path in a
  future release?
- Resolved on 2026-06-21: DART 6.20 should completely remove GLUT, not split or
  preserve it. All GLUT-supported features must have OSG support before deletion,
  and `lodepng` removal follows the OSG screenshot migration.
