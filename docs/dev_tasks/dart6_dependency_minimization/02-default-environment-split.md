# Lane: DART 6.20 dependency reduction (task_3)

> Sub-document of the `dart6_dependency_minimization` task. The task `README.md`
> is the single source of truth for the overall effort (owned by the
> native-replacement lane). **This file owns the task_3 lane only.** Link to the
> README; do not duplicate it.

## Scope (evolved)

This lane started as a "default-environment split" — demoting heavy optional
deps out of the default Pixi env into feature envs. That proved **marginal**:
DART 6.20 still hard-requires **FCL** as a core collision dependency, so demoting
*alternative* backends out of the default env does not remove a dependency, it
only hides alternatives. The lane therefore pivoted to:

1. **Remove genuinely-unnecessary / deprecated dependencies outright** (not just
   shuffle them into feature envs) — optimizer backends (done). A follow-up sweep
   found no other *standalone*-removable dep (boost is OSG-coupled; see "Next").
2. **Defer the one large, high-value reduction** — a native collision detector
   that would make FCL/Bullet/ODE optional — to a separate, evidence-driven
   initiative (see "Deferred").

Retained features keep source-compat; removals of deprecated/optional surfaces
are documented breaking changes for 6.20.

## Coordination with task_2 (native-replacement lane)

- task_2 owns `convhull` (merged), `imgui`, `ikfast`, `lodepng` (native PNG),
  `odelcpsolver` (native Dantzig, #3088 merged) — `dart/external/*` + source.
- Branch setup (per `AGENTS.md`): create topic branches with
  `git switch --no-track -c <type>/<topic> origin/release-6.20` — **never let a
  topic branch track `origin/release-*`** (prevents accidental release-branch
  pushes).
- Shared hot files are `pixi.toml`/`pixi.lock`; **merge** `origin/release-6.20`
  before pushing (never rebase a published PR branch). Do not edit the task
  `README.md` (task_2's SSOT).

## Locked decisions

1. **Optimizer backends → REMOVED** (not demoted): merged in #3105.
2. **Collision pixi-demotion → ABANDONED** (#3106 closed): marginal while FCL is
   core. The meaningful reduction is the native collision detector, **deferred**.
3. **Compatibility bar:** retained features keep source-compat (old `#include`
   paths, component names, exported CMake targets). Removing deprecated optional
   surfaces (e.g. optimizer components/macros) is a documented breaking change.
4. **CI for feature envs uses runtime gates** — `pixi run -e <env> test` (ctest)
   **and** `pixi run -e <env> test-py` (pytest). **`test-all` does NOT run
   tests** — it only builds CMake's `ALL` target (no ctest/pytest), so never
   treat it as runtime coverage. _(maintainer clarification)_

## Status by dependency (release-6.20)

| Dependency | State | Disposition |
| --- | --- | --- |
| Eigen3, assimp, fmt, **FCL**, ccd, tinyxml2, urdfdom, console_bridge | required-core | **Keep.** FCL can only drop via the native-collision backport (deferred). `console_bridge` is linked by `dart/utils/urdf`. |
| spdlog | optional logging | Keep (used). |
| **ipopt, nlopt, pagmo, snopt** | deprecated optimizer backends | ✅ **REMOVED** — extracted to [dart-optimization](https://github.com/dartsim/dart-optimization); merged #3105. |
| **boost (`libboost-devel`)** | declared for pagmo (now gone), but still pulled by `openscenegraph → collada-dom` | **Not independently removable** — OSG keeps boost in the default env, so dropping the explicit entry is cosmetic. DART has no boost usage; it drops out **with the GUI/OSG demotion** (slice 3). |
| bullet-cpp, libode | optional collision backends; gz-physics uses them | **Demotion abandoned** (#3106 closed). Folds into the deferred native-collision initiative. |
| openscenegraph, freeglut, imgui (GUI) | `DART_BUILD_GUI_OSG=ON` pulls them | **Deferred** (blocker below). |
| octomap | optional, but in exported `DART_PKG_EXTERNAL_DEPS` | **Keep** — demotion changes installed headers (`config.hpp`/`VoxelGridShape.hpp`) + the `dart.pc` contract. |

## Done

### Optimizer backends — REMOVED (merged #3105)

Deleted `dart/optimizer/{ipopt,nlopt,pagmo,snopt}/`, the pagmo multi-objective
API, the `dart.optimizer.NloptSolver` binding, `test_MultiObjectiveOptimization`,
the `HAVE_IPOPT/NLOPT/PAGMO/SNOPT` `config.hpp` macros, the
`optimizer-{ipopt,nlopt,pagmo}` components, the `ipopt`/`nlopt`/`pagmo-devel`
Pixi deps, and the now-stale non-Pixi install instructions (`Dockerfile`,
`Brewfile`, `build.rst`). **Kept** the optimizer core
(`Function`/`Problem`/`Solver`/`GradientDescentSolver`) — DART's
`InverseKinematics`/`HierarchicalIK`/`BalanceConstraint` depend on it. gz uses
neither core nor backends (verified). Verified by `dart`+`dartpy`+IK build +
ctest + lint.

## Next — HELD for the native-collision port (maintainer decision 2026-06-21)

After the optimizer removal, no other dependency is *independently* removable
(boost is OSG-coupled — `openscenegraph → collada-dom → libboost`; dropping the
explicit `libboost-devel` is cosmetic — DART has no boost usage, verified by a
clean `dart`+`dartpy`+ctest(100/100)+pytest(52) build without the explicit dep).
The remaining real reductions, with measured default-env footprint (236 pkgs):

| Lever | Removes from default env | Status |
| --- | --- | --- |
| Collision extras (bullet/ode) — was #3106 | `bullet-cpp` 40.6 MiB + `libode` 0.5 MiB ≈ **41 MiB** | held (closed) |
| GUI/OSG chain — slice 3 | OSG 9.1 + boost 3.1 + collada-dom 1.9 + imgui 0.9 + GL/glu/glut ~0.6 ≈ **15.6 MiB** | deferred |
| FCL (core) | `fcl` 1.5 MiB | only via native collision |

**Decision (c):** hold ALL interim Pixi demotions — including the ~41 MiB
collision one — and pursue only the **native-collision detector port** (DART 7
PLAN-035) as the path that eliminates the external collision deps
(FCL/Bullet/ODE) *and* is evidence-driven faster. That port is an XL, separately
scoped initiative (see "Deferred"); not started yet. `boost` + the vestigial
`BOOST_ALL_NO_EMBEDDED_GDB_SCRIPTS` define drop out with the GUI/native work.

## Deferred

### Native collision detector (the real FCL/Bullet/ODE reduction)

- Port DART 7's `dart/collision/native/` (broadphase; narrowphase GJK/MPR/SAT;
  SDF; CCD; raycast; persistent contact manifolds — DART 7 PLAN-035, PRs
  #2652/#2688/#2700) to DART 6.20 as the default detector, making
  FCL/Bullet/ODE optional.
- **Requirements (maintainer):** backward-compat with gz-physics/gz-sim (gz's
  `find_package` requires `DART COMPONENTS collision-bullet collision-ode`), full
  feature/contact parity, and **evidence-driven** performance ≥ Bullet/ODE/FCL.
- **Status:** deferred — XL multi-PR initiative needing its own design doc,
  phased plan, and benchmark harness. Not pursued for 6.20 now.

### GUI stack (OSG/GLUT/imgui)

- Deferred. Blocker: default `config` sets `-DDART_BUILD_DARTPY=ON`, and
  `python/dartpy/CMakeLists.txt` *hard-requires* `dart-gui-osg` via
  `FATAL_ERROR` (not `if(TARGET)`-guarded). Demotion needs that guard + a Windows
  wheel-publish repoint + keeping GUI in the publishing env; coordinate with
  task_2's imgui branch.

## Validation gates (select by touched surface)

- Always: `pixi run check-lint` before commit.
- After any dep change: default `pixi run config` + `build`.
- Runtime coverage: `pixi run test` (ctest) + `pixi run test-py` (pytest).
  **`test-all` only builds the `ALL` target — it does NOT run ctest/pytest**, so
  never treat it as runtime coverage. _(maintainer clarification)_
- For a removed dep: grep the tree for residual references; confirm a clean
  default build.
- Exported/installed surfaces: installed-package/component smoke.
- gz-visible surfaces (collision/constraint/solver/package): `pixi run -e gazebo
  test-gz` (`DART_PARALLEL_JOBS=8` per local OOM gotcha).

## Open decisions (for maintainer)

1. **OctoMap (RESOLVED — keep):** demotion would change installed headers
   (`config.hpp`, `VoxelGridShape.hpp`) and the `dart.pc` `Requires` contract.
   A true-optional `VoxelGridShape` componentization is a separate future effort
   — file a tracking issue? (defaulting to no for now).
2. **GUI demotion — deferred:** OK to later guard dartpy's `dart-gui-osg`
   requirement with `if(TARGET)` (so a default GUI-less build/wheel is possible
   while feature/publish envs keep GUI), or keep GUI in the default env?
3. **Native collision backport — deferred:** confirm when to start the dedicated
   design/scoping effort (gz-compat matrix, feature-parity matrix, benchmark
   plan).

## Progress log

- 2026-06-19: Lane created off `origin/release-6.20`. Baseline inventory + a
  6-agent adversarial scoping workflow. Slice 1 (optimizers) implemented.
- 2026-06-20: Per maintainer decision, **upgraded the optimizer slice from
  demote to full removal**; opened/merged it as **#3105** (squash
  `c4298ce4879`), including the Codex-review fix to remove `ipopt`/`nlopt`/`pagmo`
  from `Dockerfile`/`Brewfile`/`build.rst`. `.gitignore` hygiene fix **#3102**
  merged. Verified via `dart`+`dartpy`+IK build + ctest + lint.
- 2026-06-20: Maintainer clarified CI runtime gates (`test`/`test-py`, not
  `test-all`) on this doc — incorporated into the validation gates above.
- 2026-06-20: Per maintainer decision, **closed #3106** (collision pixi-demotion
  is marginal while FCL is core) and **deferred the native-collision backport**.
- 2026-06-21: Swept for other removable deps. **boost is not independently
  removable** — `openscenegraph → collada-dom → libboost` keeps it in the default
  env (`pixi tree -i libboost`); DART has no boost usage and builds/tests clean
  without the explicit dep, but the removal only lands with the GUI/OSG demotion.
  Standalone easy removals are exhausted; the remaining wins (GUI/OSG, FCL) are
  the deferred initiatives.
- 2026-06-21: Measured per-lever default-env footprint (collision extras ≈41 MiB,
  GUI/OSG ≈15.6 MiB, FCL 1.5 MiB). **Maintainer decision (c): hold ALL interim
  Pixi demotions and pursue only the native-collision port.** task_3's active
  removals are therefore complete (optimizer #3105 merged); #3107 is the durable
  record. Retired the obsolete local integration branch
  `chore/default-env-split-release-6.20` (fully superseded by #3102/#3105/#3107).

## Resume prompt

> Continue the DART 6.20 dependency-reduction lane (task_3). Read this file and
> the task `README.md`. Optimizer backends are removed (#3105 merged); the
> collision pixi-demotion was abandoned (#3106 closed). Per **maintainer decision
> (c)**, ALL interim Pixi demotions are **held** — including the GUI/OSG (slice 3)
> and collision-extras footprint levers — and the lane pursues only the
> **native-collision detector port** (deferred, XL, awaiting the maintainer's
> explicit go-ahead; scoped in `03-native-collision-port-scoping.md`). Standalone
> dep removals are exhausted (**boost is not independently removable** —
> `openscenegraph → collada-dom → libboost`; it only drops with the held GUI/OSG
> work). Do **not** start the GUI/OSG demotion or any other interim demotion
> without a new maintainer decision — task_3's active removals are complete
> (#3105). Feature-env CI (if any) must use `test`/`test-py`, never `test-all`.
