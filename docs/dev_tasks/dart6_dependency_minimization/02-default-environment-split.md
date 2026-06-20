# Lane: Default-Environment Footprint Shrink (DART 6.20)

> Sub-document of the `dart6_dependency_minimization` task. The task
> `README.md` is the single source of truth for the overall effort and is owned
> by the native-replacement lane. **This file owns workstream #2
> ("Default-environment split") only.** Do not duplicate the README here; link
> to it.

## Ownership & coordination

- **This lane (task_3):** demote heavy *optional* dependencies out of the
  **default Pixi environment** so a default `pixi run build` pulls a minimal
  set. Pure packaging/build-config work; no runtime-behavior or public-API
  change.
- **Native-replacement lane (task_2):** `convhull` (merged), `imgui`
  (merged + ongoing), `ikfast`, `lodepng` (native PNG writer), `odelcpsolver`
  (native Dantzig, PR #3088). These touch `dart/external/*` and source code.
- **Why non-overlapping:** workstream #2 had zero commits on any branch when
  this lane started. The only shared hot file is `pixi.toml`; see
  "Coordination" below for the sequencing rule that avoids conflicts.

## Locked decisions (interview 2026-06-19)

1. **Engagement:** parallel, non-overlapping. task_2 keeps its 5 branches;
   task_3 owns the default-environment split. Full takeover later.
2. **Scope ceiling:** through default-footprint shrink. **Native-collision
   backport is OUT of scope for 6.20** (so FCL stays core).
3. **Implementation approach (native lane):** native, matching DART 7.
4. **Compatibility bar:** source-compat via forwarders. Old `#include` paths,
   component names, and exported CMake targets keep working; internal ABI may
   shift within the 6.20 minor. **This lane changes only which packages the
   *default Pixi env* installs — not installed headers, component names, or
   exported targets.**

## Dependency classification (release-6.20 baseline)

Evidence: `pixi.toml` (base `[dependencies]` applies to ALL environments),
`cmake/DARTFindDependencies.cmake`, `dart/CMakeLists.txt`,
`dart/{collision,optimizer,gui}/**/CMakeLists.txt`.

| Dependency | Default state | Verdict for this lane |
| --- | --- | --- |
| Eigen3, assimp, fmt, **FCL**, ccd, tinyxml2, urdfdom, console_bridge, spdlog | required-core (FCL exported in `DART_PKG_EXTERNAL_DEPS`, linked into `dart` PUBLIC) | **Fixed** — FCL cannot move without native-collision backport (out of scope) |
| ipopt, nlopt, pagmo-devel | base `[dependencies]`; deprecated, optional CMake components | **Slice 1 — REMOVED** (extracted to `dart-optimization`; #3105) |
| bullet-cpp, libode | base `[dependencies]`; optional CMake components; **gz-physics depends** | **Slice 2** — must remain in the `gazebo` feature env |
| openscenegraph, freeglut (+ libgl/libglu), imgui | base `[dependencies]`; `DART_BUILD_GUI_OSG=ON` pulls them | **Slice 3** — overlaps task_2 imgui; sequence after its merge |
| octomap | base `[dependencies]`; optional CMake — **but in exported `DART_PKG_EXTERNAL_DEPS`** | **Slice 4 — decision-gated** (changing the exported string is downstream-visible) |

## Mechanism (Pixi env membership only)

`pixi.toml`'s top-level `[dependencies]` is the implicit default feature applied
to **every** environment. To demote a dep:

1. Remove it from base `[dependencies]`.
2. Add it to a named `[feature.<name>.dependencies]`.
3. Keep the `default` environment from including that feature.
4. Add/extend an environment that DOES include it, so CI and component builds
   still exercise the dependent code.

This leaves CMake options, installed headers, component names, and exported
targets untouched (compat bar satisfied). The dependent CMake component simply
auto-skips in a default build (each uses `dart_check_optional_package`) and
builds normally in the feature environment.

## Sequenced slices (each = one PR to `release-6.20`, DART 6.x patch milestone)

### Slice 1 — Optimizers (ipopt, nlopt, pagmo) — MERGED (#3105, full removal)
- **Scope upgraded from demote to REMOVE** (maintainer decision 2026-06-20): the
  IPOPT/NLopt/pagmo/SNOPT backends are deprecated and were extracted to
  [dart-optimization](https://github.com/dartsim/dart-optimization), so they are
  deleted from 6.20 rather than moved to a feature env. Removed the four backend
  subdirs, the pagmo multi-objective API, the `dart.optimizer.NloptSolver`
  binding, `test_MultiObjectiveOptimization`, the `HAVE_IPOPT/NLOPT/PAGMO/SNOPT`
  `config.hpp` macros, the `optimizer-{ipopt,nlopt,pagmo}` components, and the
  `ipopt`/`nlopt`/`pagmo-devel` Pixi deps (no feature env needed).
- **Kept** the optimizer core (`Function`/`Problem`/`Solver`/
  `GradientDescentSolver`) — DART's `InverseKinematics`/`HierarchicalIK`/
  `BalanceConstraint` depend on it. gz gate verified clear (gz uses neither).
- Verified: `dart` + `dartpy` + `test_Optimizer` + `INTEGRATION_InverseKinematics`
  build; `ctest` for Optimizer/IK 100% pass; lint green; `pixi.lock` regenerated.
  Shipped as [#3105](https://github.com/dartsim/dart/pull/3105).

### Slice 2 — Collision extras (bullet-cpp, libode) — OPEN → PR #3106 (rebased onto release-6.20)
- Moved out of base `[dependencies]` into `[feature.collision-extra.dependencies]`;
  added `collision-extra` environment + Linux `collision-extra` CI job. Left
  `[feature.gazebo.dependencies]` bullet/ode UNCHANGED.
- Adversarial verify (workflow): `safeForDefaultBuild=true`, `gzPhysicsRisk=none`
  (gz uses the FCL detector; gazebo env retains both), not in
  `DART_PKG_EXTERNAL_DEPS` → no downstream-contract change. Default
  `all`/`test-all` stays safe because the examples/tutorials that hard-link
  `dart-collision-{bullet,ode}` are `EXCLUDE_FROM_ALL` and
  `dart_build_target_in_source` early-returns on missing targets.
- Accepted caveats: building those bundled examples/tutorials **out-of-source**
  against a minimal default install needs `-e collision-extra` (pre-existing
  optional-dep behavior); default `coverage` job + non-Linux default builds lose
  bullet/ode line-coverage (mirrors slice 1; gz lane + the new Linux job retain
  functional coverage).

### Slice 3 — GUI stack (OSG, GLUT, imgui) — DEFERRED (high risk; maintainer decision)
- **Blocker (workflow-verified):** a default configure HARD-FAILS, not graceful.
  Default `config`/`config-py` set `-DDART_BUILD_DARTPY=ON`, and
  `python/dartpy/CMakeLists.txt:45-56` lists `dart-gui`/`dart-gui-osg` with
  `message(FATAL_ERROR ...)` if a target is missing (NOT `if(TARGET)`-guarded
  like optimizers/collision). Removing GUI deps from default → dartpy fatal.
- Also: Windows wheel publish (`publish_dartpy.yml` uses the `default` env +
  `setup.py` forces DARTPY=ON, GUI_OSG default ON) would fatal; dropping
  `gui`/`gui-osg` from a default *install* breaks downstream
  `find_package(DART COMPONENTS gui-osg)`.
- **Prereqs:** guard dartpy's gui-osg requirement with `if(TARGET)` (or move
  dartpy/GUI into the `gui` env), repoint the Windows wheel build to the `gui`
  env, and keep GUI in whatever env installs/publishes. Coordinate with task_2's
  `chore/replace-imgui` (+~26 commits unmerged; #3081 already merged here).
  Needs maintainer sign-off (open decision #2).

### Slice 4 — OctoMap — RESOLVED: keep in default (do NOT demote)
- Demotion would violate the locked compat bar: `HAVE_OCTOMAP` is baked into the
  **installed** `dart/config.hpp` and wraps the entire `VoxelGridShape.hpp` class
  (installed header surface would change), and octomap is in
  `DART_PKG_EXTERNAL_DEPS` → unconditional `dart.pc` `Requires:` (downstream
  pkg-config contract). True optionality would require componentizing
  `VoxelGridShape` — a separate, out-of-scope effort worth a tracking issue.

## Coordination with task_2 (conflict control)

- **`pixi.toml` is the only shared hot file.** Slices 1–2 edit lines task_2
  does not (optimizers, bullet/ode) — conflict-free. **Slice 3 (GUI/imgui)**
  touches the imgui line + GUI CMake; task_2's imgui rewrite #3081 is already
  merged here (commit `936f2dbabf5`), but `chore/replace-imgui-release-6.20`
  still has ~26 unmerged commits — sequence slice 3 after that lands.
- Rebase rule: PR branches track `origin/release-6.20`; merge latest base
  before any push (never rebase a published branch). task_2's #3088 (native
  Dantzig LCP) is already merged; merge the imgui/lodepng branches in as they
  land. The `.gitignore` hygiene fix shipped as #3102 (merged).
- Do not edit the task `README.md` (task_2's SSOT). Add findings here.

## Validation gates (select by touched surface)

- Always: `pixi run check-lint` before commit.
- After any dep change: default `pixi run config` + `build`.
- For each demoted dep: `pixi run -e <feature> build` + that surface's tests.
- Exported/installed surfaces: installed-package/component smoke.
- Collision/constraint/solver/package surfaces visible to gz: `pixi run -e
  gazebo test-gz` (use `DART_PARALLEL_JOBS=8` per local OOM gotcha).
- dartpy bindings touched: `pixi run test-py`.

## Open decisions (for maintainer)

1. **OctoMap (RESOLVED — keep):** demotion would change installed headers
   (`config.hpp`, `VoxelGridShape.hpp`) and the `dart.pc` Requires contract,
   violating the locked compat bar. Kept in default. A true-optional
   `VoxelGridShape` componentization is a separate future effort — confirm if
   you want a tracking issue filed.
2. **GUI demotion (slice 3) — needs your call:** the blocker is that dartpy
   *hard-requires* `dart-gui-osg`. Preferred fix: guard it with `if(TARGET)`
   (mirrors how optimizers/collision are already handled) so a default GUI-less
   build/wheel is possible while feature/publish envs keep GUI. OK to make that
   dartpy change, or prefer keeping GUI in the default env (skip slice 3)?
3. **CI matrix (confirm for later slices):** slice 1 removed the optimizer
   backends outright, so there is **no `optimization` env** — its coverage now
   lives in `dart-optimization`. Slice 2 adds a dedicated `collision-extra` Pixi
   env + one Linux `collision-extra` CI job (`pixi run -e collision-extra
   test-all`); slice 3 (GUI) would follow the same per-concern-env pattern.
   Alternative the maintainer may prefer one consolidated `full` env/job. Note the
   Linux `coverage` (codecov) + non-Linux default builds lose bullet/ode
   line-coverage; the dedicated job + gz lane retain functional coverage — add
   per-platform jobs only if asked.

## Progress log

- 2026-06-19: Lane created off `origin/release-6.20`
  (`chore/default-env-split-release-6.20`). Baseline inventory + pixi structure
  captured. Slice 1 (optimizers) started.
- 2026-06-19: Slice 1 edits applied — moved `ipopt`/`nlopt`/`pagmo-devel` from
  base `[dependencies]` to `[feature.optimization.dependencies]`, added the
  `optimization` environment, added the Linux `optimization` CI job
  (`ci_ubuntu.yml`), and added a CHANGELOG Build entry.
- 2026-06-19: Slice 1 verified (default skip `HAVE_*=FALSE`; `optimization` env
  resolves `HAVE_*=TRUE`); lint green; `pixi.lock` regenerated. Ran a 6-agent
  scoping workflow (adversarial verify) for slices 2–4: slice 2 (bullet/ode)
  implemented; slice 3 (GUI) deferred (dartpy `FATAL_ERROR` blocker → open
  decision #2); slice 4 (octomap) resolved as keep-in-default.
- 2026-06-20: Opened PRs against `release-6.20` (rebased onto current base, which
  now includes task_2's #3088): `.gitignore` fix #3102 (merged), optimizer #3105,
  collision demotion #3106 (stacked on #3105). Slice 3 (GUI) remains deferred
  pending maintainer decision #2; slice 4 (octomap) kept.
- 2026-06-20: Per maintainer decision, **upgraded slice 1 from demote to full
  removal** of the deprecated optimizer backends (moved to `dart-optimization`);
  #3105 rewritten accordingly and #3106 rebased onto it. Verified via full
  `dart`/`dartpy`/IK build + tests + lint.
- 2026-06-20: Addressed the Codex review on #3105 (silent fix, no bot reply):
  removed `ipopt`/`nlopt`/`pagmo` from the root `Dockerfile`, `Brewfile`, and the
  `build.rst` apt/brew/vcpkg/Arch lists + dependency table. **#3105 merged**
  (squash `c4298ce4879`) into `release-6.20`. Rebased #3106 onto the merged
  `release-6.20` (drops the merged optimizer commit; collision-only), lock
  regenerated unchanged, config + lint re-verified — awaiting CI/merge.

## Resume prompt

> Continue the DART 6.20 "default-environment footprint shrink" lane on branch
> `chore/default-env-split-release-6.20` (based on `origin/release-6.20`). Read
> this file and the task `README.md`. Next action is the in-progress slice
> (see Progress log). Demote one optional dependency group at a time from
> `pixi.toml` base `[dependencies]` into a named feature + environment, prove the
> default build skips the component gracefully and the feature env still tests
> it, and run the gates listed above (gz gate for collision/GUI-adjacent
> changes). Coordinate `pixi.toml` edits with task_2's imgui branch per the
> Coordination section.
