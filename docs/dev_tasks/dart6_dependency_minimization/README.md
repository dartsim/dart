# DART 6.20 Dependency Minimization Plan

## Working Branches

This task is for the DART 6.20 support lane.

- Implementation branch:
  `plan/dart6-dependency-minimization-6.20`, based on
  `origin/release-6.20`.
- DART 7 reference checkout: `/home/js/dev/dartsim/dart/main`, on `main`.
- Do not use the `task_2` worktree on `main` for this task. Use the separate
  main checkout, or remote refs, only as comparison evidence.

Evidence was collected on 2026-06-19 after fetching `origin/main`,
`origin/release-6.20`, and `origin/release-6.19`.

## Current Branch State

- `origin/release-6.20` currently points at `6543bcc37ce` (`Packaging
  6.19.2`) and is tagged `v6.19.2`.
- `package.xml` and `pixi.toml` on `origin/release-6.20` still report version
  `6.19.2`.
- Before any implementation PR, confirm whether the branch is intentionally the
  6.20 lane with pending version metadata, or whether a packaging/version bump
  must land first.

## DART 6 Dependency Inventory

`package.xml` on `release-6.20` lists these package dependencies:

- Required or exported build dependencies: `assimp`, `eigen`, `libfcl-dev`,
  `liburdfdom-dev`, and `tinyxml2`.
- Optional or component-related package dependencies that are still advertised:
  `bullet`, `glut`, `libxi-dev`, and `libxmu-dev`.

`pixi.toml` on `release-6.20` keeps a broad default environment. The important
project dependencies include:

- Core and package-surface dependencies: `assimp`, `eigen`, `fcl`, `fmt`,
  `urdfdom`, `tinyxml2`, and optional `spdlog`.
- Collision and constraint ecosystem dependencies: `bullet-cpp`, `libode`,
  `octomap`, and `fcl`.
- GUI dependencies: `openscenegraph`, `freeglut`, OpenGL/GLU packages, and
  `imgui`.
- Optimizer dependencies: `ipopt`, `nlopt`, and `pagmo-devel`.
- Test, build, and docs tools: CMake, Ninja, GoogleTest, Google Benchmark,
  Doxygen, Sphinx, pytest, and related Python tooling.

CMake on `release-6.20` makes `fmt`, Eigen, FCL, and Assimp part of the core
configure/build path. Bullet, ODE, GUI, optimizer backends, OctoMap, and ImGui
are component or feature surfaces, but the default Pixi environment makes many
of them available by default.

## DART 6 `dart/external` Inventory

`release-6.20` still builds these vendored source trees:

- `dart/external/convhull_3d`
- `dart/external/ikfast`
- `dart/external/imgui`
- `dart/external/lodepng`
- `dart/external/odelcpsolver`

Usage summary:

- `convhull_3d` is included by the math geometry implementation.
- `ikfast` is included by DART's IKFast wrapper, generated WAM examples, and
  integration tests.
- `imgui` is used by the OSG GUI path when `DART_USE_SYSTEM_IMGUI` is disabled.
- `lodepng` is used by the GLUT screenshot path.
- `odelcpsolver` is included by core constraint, contact, boxed LCP, Dantzig,
  PGS, and default `World` solver paths.

## DART 7 Reference State

The main checkout has no `dart/external/` directory.

DART 7 moved or removed the old vendored surfaces as follows:

- Convex hull code moved into DART-owned math detail files.
- IKFast moved to a DART-owned dynamics header path.
- ImGui is resolved through the GUI dependency/fetch path instead of an
  in-tree `dart/external` copy.
- The legacy GLUT/OSG GUI stack was removed as part of the DART 7 clean break.
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
- Plan: rehome the implementation under `dart/math/detail`, update the geometry
  include, and remove the external target/install rule.
- Validation: default build plus focused math/geometry tests that cover convex
  hull behavior.

Default Pixi/package metadata for optional components

- Risk: low to medium, depending on the surface.
- Plan: move optional packages out of the default path one at a time only after
  proving the default configure still succeeds and explicit feature/component
  environments keep their tests.
- Candidates for feature-only treatment before code removal: Bullet, ODE, GUI
  packages, optimizer packages, and OctoMap.
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
- Plan: prefer system ImGui for packaged builds and replace the vendored tree
  with either a FetchContent fallback or a documented GUI feature dependency.
  Do not remove OSG GUI compatibility in this slice.
- Validation: GUI configure/build on Linux and macOS lanes, OSG regression or
  screenshot smoke, and package smoke proving ImGui headers are found.

`dart/external/lodepng`

- Risk: medium to high because it is tied to legacy GUI screenshot behavior.
- Plan: preserve GUI compatibility first. Either replace screenshot encoding
  with an accepted system dependency such as libpng, or defer removal until
  maintainers approve a GUI feature split.
- Validation: GUI screenshot smoke plus package metadata tests for any
  replacement dependency or feature split.

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
- Plan: the narrow cleanup is to rehome the DART 6 solver code under a
  DART-owned internal path, not delete it. True replacement requires a separate
  solver behavior project with full contact and downstream evidence.
- Validation: full unit tests for constraints/contact/dynamics, focused LCP
  solver regressions, dartpy smoke where constraint solver types are bound, and
  Gazebo.

OpenSceneGraph and GLUT GUI dependencies

- Risk: high for API removal, medium for default-environment demotion.
- Plan: do not remove these APIs in a dependency cleanup PR. A safe 6.20 step
  is making GUI dependencies opt-in in default environments while keeping
  feature builds and installed package components intact.
- Validation: GUI feature configure/build, tutorials/examples that request GUI,
  and package-component smoke.

## Sequenced Workstreams

1. **Branch and package baseline.** Confirm the 6.20 branch/version owner state
   and record the gz-physics version lane for 6.20 before dependency removals.
2. **Default-environment split.** Move optional dependencies out of the default
   Pixi/package path one surface at a time, preserving explicit feature
   environments and component builds.
3. **Low-risk external rehomes.** Remove `convhull_3d`, then move IKFast with a
   compatibility-forwarder decision.
4. **ImGui vendored-source removal.** Prefer system ImGui or an approved fetch
   fallback while preserving the DART 6 OSG GUI component.
5. **GUI screenshot dependency decision.** Replace `lodepng` only after the
   maintainers choose between a system PNG dependency and a GUI feature split.
6. **Collision dependency reduction.** Treat Bullet/ODE package moves as
   optional-component packaging work. Treat FCL removal as a native-collision
   backport decision, not a routine cleanup.
7. **LCP solver cleanup.** Rehome `odelcpsolver` internals if the goal is to
   eliminate `dart/external/`. Replace it only under a separate solver-behavior
   plan with full contact and Gazebo evidence.

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
  N=${DART_SAFE_JOBS:-$(python scripts/parallel_jobs.py)}
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
- For ImGui and GUI screenshots, should DART 6 prefer system dependencies,
  FetchContent fallbacks, or feature-only GUI environments?
