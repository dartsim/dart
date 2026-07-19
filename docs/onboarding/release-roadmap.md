# DART Release Roadmap

DART is moving toward an easier-start research experience with a clean DART 7
API. Python-first APIs are a major part of that direction, layered on a
refreshed C++23 core. Compatibility for the established DART 6 API lives on the
active DART 6 LTS maintenance branch instead of blocking the main-branch DART 7
design. Use the highest maintained `release-6.*` branch advertised by the
remote; this checkout currently sees `release-6.20`.

## Compatibility & Deprecation Policy

- New major releases can introduce API-breaking changes.
- DART 7 is the clean-break release for the DART 6 to DART 7 transition. Do not
  add new DART 6 compatibility shims on main unless a maintainer explicitly
  scopes the work as migration support.
- DART 6 compatibility work, including Gazebo/gz-physics compatibility fixes,
  belongs on the active DART 6 LTS support lane by default.
- Required gz-physics compatibility evidence is validated with
  `pixi run -e gazebo test-gz` on the active DART 6 LTS branch or affected
  release branch. The main-branch Gazebo workflow is a migration canary, not a
  DART 7 API design constraint.

## DART 6 LTS: Compatibility Line

- The active DART 6 LTS branch is the newest maintained `release-6.*` branch;
  at the time this roadmap was updated, that branch is `release-6.20`.
- The normal release milestone for this support lane is `DART 6.20.0`. The
  DART 6.19.4 release and milestone were a narrowly scoped emergency exception for
  [#3387](https://github.com/dartsim/dart/issues/3387), not a reopened 6.19
  maintenance lane; subsequent DART 6 fixes continue to target
  `release-6.20` and `DART 6.20.0`.
- The pinned Gazebo branches on `release-6.20` are
  `gz-physics8_8.0.0` and `gz-sim9_9.0.0`, matching that branch's `pixi.toml`
  integration tasks. Main separately pins `gz-physics9_9.0.0` for its DART 7
  migration canary.
- Backport critical bug fixes, build fixes, security fixes, and
  Gazebo/gz-physics compatibility fixes that released downstream users need.
- Do not backport normal DART 7 features by default.
- Publish the Gazebo support window, branch/version matrix, and sunset date or
  sunset trigger before DART 7 removes legacy API surfaces from main.

### DART 6 LTS Support Packet

This packet is the published support window for the DART 6 LTS compatibility
line. It exists so downstream users — primarily Gazebo via `gz-physics` — know
which branch to track, what fixes to expect, and when the line will close. The
companion CI lane split is active: required Gazebo validation runs on the active
DART 6 LTS branch, while main's gz-physics workflow remains a manual DART 7
migration canary.

**Gazebo support window**

- Pinned `release-6.20` downstream branches: `gz-physics8_8.0.0` and
  `gz-sim9_9.0.0` (the branches cloned by that branch's `pixi.toml` Gazebo
  integration tasks).
- Main's DART 7 migration canary separately pins `gz-physics9_9.0.0` through
  its `pixi.toml` `feature.gazebo.tasks.download-gz` task.
- Validation command: `pixi run -e gazebo test-gz`, run on the active DART 6
  LTS branch or affected release branch when compatibility surfaces change.
  Main-branch runs are migration canaries, not a DART 7 API design constraint.
- Main's DART version compatibility window for the pinned
  `gz-physics9_9.0.0` canary is DART 6.10 through 7.0
  (`scripts/patch_gz_physics.py` `MIN_COMPATIBLE_VERSION = (6, 10)`,
  `MAX_COMPATIBLE_VERSION = (7, 0)`), so DART 7's generated
  `DARTConfigVersion.cmake` still satisfies the downstream `find_package`
  request during migration.

**Branch / version matrix**

| Branch                                         | DART line        | Public API target      | Gazebo pins                            | Backport policy                                              |
| ---------------------------------------------- | ---------------- | ---------------------- | -------------------------------------- | ------------------------------------------------------------ |
| Active `release-6.*`, currently `release-6.20` | DART 6 (6.x LTS) | Established DART 6 API | `gz-physics8_8.0.0`; `gz-sim9_9.0.0`   | Compatibility-critical fixes only (see scope below)          |
| `main`                                         | DART 7 (7.0.0)   | DART 7 clean-break API | `gz-physics9_9.0.0` (migration canary) | DART 7 development; no DART 6 compatibility shims by default |

**Backport scope (DART 6 LTS)**

Backport only:

- critical bug fixes,
- build fixes,
- security fixes,
- Gazebo/gz-physics compatibility fixes that released downstream users need.

Do not backport normal DART 7 features. Each backport should record the
gz-physics compatibility evidence (`pixi run -e gazebo test-gz`) when it touches
a compatibility surface, or note why gz-physics is unaffected.

**Sunset condition (trigger-based)**

The DART 6 LTS support line sunsets on a trigger, not a fixed calendar date:

- **Trigger:** DART 7.0.0 is published to package managers **and** a maintainer
  decision sets a support tail. After DART 7.0.0 is published, DART 6 LTS enters
  a maintenance-only tail of **N months** (security/critical-only), after which
  the line is closed.
- **N is a maintainer decision** and is intentionally left unset here. A typical
  starting point for discussion is 6–12 months after DART 7.0.0 publication, but
  the concrete value, and whether the tail is security-only or also build/compat,
  **requires maintainer sign-off** before it is treated as policy.
- Until DART 7.0.0 is published and N is decided, the active DART 6 LTS branch
  remains the compatibility line and the sunset clock has not started.

> Maintainer sign-off needed: the value of **N** (support-tail length after DART
> 7.0.0 publication) and the tail's fix scope.

## DART 7: Clean-Break Release

- `main` targets DART 7.0.0 as a clean break from the DART 6 public API, with a
  Python-first API layered on the refreshed C++ architecture.
- The classic Skeleton-backed DART 6 API is not a DART 7 compatibility promise.
  Support for users that still need that API remains on the active DART 6 LTS
  branch.
- The ECS-backed world is the DART 7 public API target, but complete promotion
  is gated by core robotics parity evidence plus package/API-boundary evidence.
  Python has already moved the common path to `dartpy.simulation.World` and
  `dartpy.World`; C++ still uses `dart::simulation` and the
  `dart-simulation` component until PLAN-041 completes the
  namespace, target, package, and classic-world removal transaction.
- Algorithmic behavior must cover basic robotics simulation before release.
  IPC, VBD, differentiable simulation, and GPU work remain opt-in roadmap
  features unless a promoted public API depends on them.

### DART 7 Implementation Order

1. **Policy alignment**: update the release roadmap, plan dashboard,
   north-star, API-boundary, CI, release-management, README, and changelog docs
   so DART 7 is the clean break and DART 6 LTS is the compatibility line.
2. **Gazebo lane split**: keep required gz-physics validation on
   `release-6.*`; keep main's gz-physics workflow as a manual migration
   canary. Coordinate branch-protection changes with maintainers before relying
   on that demotion.
3. **DART 6 support packet**: audit fixes on main against `release-6.*` and
   backport only compatibility-critical patches needed for the old API/Gazebo
   line.
4. **Official simulation API promotion**: make PLAN-041 the release-critical
   path. The Python facade is already staged; finish the supported ECS-backed
   C++ subset, resolve the classic `dart::simulation::World` collision, expose
   final headers/targets without the experimental component, and keep parity
   gates attached to the promotion claim rather than deferring the official API
   to DART 8.
5. **Consumer migration and removal**: port in-repo examples, tutorials, tests,
   benchmarks, and Python stubs to the promoted API; then remove the classic
   `World`, DART 6 C++ API shims, legacy dartpy compatibility modules,
   experimental import paths, and gz-only compatibility surfaces from main.
6. **Physical restructuring**: keep `dart/` and `python/` files in the final
   DART 7 locations, with any future mechanical moves isolated from semantic API
   changes.

### DART 7 Checkable Gates

| Gate                         | Required evidence                                                                                                                                               | Local or CI command                                                                                                                                             |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| DART 7 model loading         | URDF, SDF, MJCF, and promoted optional USD loaders import into the promoted world API with topology, DOF, transform, mass/inertia, collision, and Python tests. | Focused simulation loader tests plus `pixi run test-simulation` and `pixi run test-py`; USD gates run only when `DART_BUILD_IO_USD=ON`.                         |
| Rigid dynamics parity        | Shared open-chain scenes match the classic DART 6 path within documented tolerances for gravity, integration, drift, and controls.                              | Release-6.\* branch evidence plus main-branch DART 7 regression tests.                                                                                          |
| Contact/constraint parity    | Contact response, friction, joint limits, motors, mimic/coupler behavior, and loop closures have tests and migration examples.                                  | Focused simulation contact/constraint tests and parity suite rows.                                                                                              |
| Serialization/replay parity  | World topology, state, model assets, and record/replay round-trip with bounded error.                                                                           | Serialization/replay tests for promoted world APIs.                                                                                                             |
| Stable public API promotion  | Promoted APIs hide ECS storage, component types, solver registries, backend details, and implementation escape hatches.                                         | `pixi run check-api-boundaries`, generated stubs, docs, and migration snippets.                                                                                 |
| First simulation works       | README Python, C++ package-project, and Pixi source first-success commands are verified or blocked.                                                             | Run the README quick starts, `pixi run test-published-package-quickstarts`, and `pixi run check-dart7-artifacts`.                                               |
| Core build and tests         | Lint, C++ build, Python build, and focused or full test suites pass for the changed release scope.                                                              | `pixi run lint`, `pixi run build`, `pixi run test-unit`, `pixi run test-py`, `pixi run test-all`, and on Linux CUDA hosts `pixi run -e cuda test-all`.          |
| LCP/contact baseline         | Solver contract and benchmark smoke evidence are recorded before algorithm or compute-scaling promises.                                                         | `ctest --test-dir build/default/cpp/Release -R UNIT_math_lcp_math_lcp_all_solvers_smoke`; `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`. |
| GUI transition               | Any GUI promotion stays aligned with the Filament migration gates and keeps backend details hidden.                                                             | Filament gates in the GUI plan/design docs.                                                                                                                     |
| DART 6 LTS support policy    | Gazebo branch/version matrix, backport scope, and sunset date or sunset trigger are published.                                                                  | `pixi run -e gazebo test-gz` on the active DART 6 LTS branch or affected release branch when compatibility surfaces change.                                     |
| Release metadata and package | Version metadata, changelog, CMake package exports, and package-manager instructions match the release.                                                         | `pixi run lint`; package-project configure/build commands from PLAN-010; release packaging checklist below.                                                     |
| AI workflow/documentation    | AI-facing workflow adapters and plan docs stay synchronized when release workflows change.                                                                      | `pixi run sync-ai-commands`, `pixi run check-ai-commands`, and docs gates from `docs/ai/verification.md`.                                                       |

### Release Packaging Checklist

For a DART 7 packaging or release-hardening pass, record evidence for the
changed scope before opening the PR:

- Version metadata: `package.xml`, `pixi.toml`, generated package metadata, and
  example `find_package(DART ...)` requirements agree on the release version.
- Changelog: `CHANGELOG.md` follows [changelog.md](changelog.md), has the
  release section, date, milestone link, release summary, migration notes, and
  entries for user-visible fixes or changes.
- Package exports: any changed CMake components configure, install, and work
  from a temporary prefix with `pixi run test-cpp-quickstart -- --prefix
<installed-dart-prefix>`.
- First-success paths: README Python, C++ package-project, and Pixi source
  commands either pass or name the current package-artifact blocker.
- Compatibility: changes touching the DART 6 LTS compatibility lane run
  `pixi run -e gazebo test-gz` on the affected release branch or record why
  gz-physics is not affected. Main-branch runs are migration canaries.
- AI/docs: release workflow or plan changes run the AI docs/adapters gates from
  `docs/ai/verification.md`.

## DART 8: Post-DART-7 Major

- DART 8 is reserved for the next major release after the DART 7 clean break.
- Do not defer DART 6 API removal or classic-world deletion to DART 8 by
  default; those are DART 7 clean-break tasks once the parity gates pass.
- DART 8 may remove APIs newly deprecated during the DART 7 line, but it is not
  the active cleanup target for the DART 6 to DART 7 transition.
- Users who still need the DART 6 surface should remain on the active DART 6
  LTS release line until they migrate or the published support window ends.

### DART 7 Compatibility-Debt Inventory

Track these before DART 7 ships so clean-break removals are deliberate:

| Debt area                         | DART 7 handling                                                                                                                                                        | Clean-break gate                                                                                                                  |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| Legacy DART 6 API surface         | Remove from main unless explicitly needed for a bounded migration adapter; keep released support on the active DART 6 LTS branch.                                      | Removal list has migration notes, changelog entries, package status, and DART 6 LTS support status.                               |
| Legacy dartpy 6 API surface       | Remove from the DART 7 public contract rather than carrying a compatibility layer.                                                                                     | Replacement import/API path is documented and covered by tests/stubs.                                                             |
| Classic Skeleton-backed `World`   | Treat as the deletion target once the promoted world has parity evidence.                                                                                              | ECS-backed world has stable wrappers, migration notes, focused tests, and parity evidence.                                        |
| DART 7 world bindings             | Python is promoted to `dartpy.simulation` / `dartpy.World`; C++ remains under `dart::simulation` and `dart-simulation` until PLAN-041 completes the final transaction. | C++ promoted as the official DART 7 API with documented migration path, package smokes, and no stale Python `simulation` surface. |
| Deprecated CMake/package surfaces | Remove package compatibility shims that would make DART 7 satisfy DART 6/gz-physics package ranges.                                                                    | Package exports and examples no longer require deprecated component or header names.                                              |
| Solver implementation details     | Keep Dantzig helper APIs, contact caches, and backend storage out of research-facing contracts.                                                                        | Any remaining public-looking internals are classified as supported, compatibility, or removed.                                    |
| GUI backend exposure              | Keep Filament and other backend details behind migration gates.                                                                                                        | Public GUI API is backend-hidden and old backend-specific entry points have a migration/removal path.                             |

Each release packaging pass should update this inventory when a deprecated API
is added, promoted, moved to the DART 6 LTS support lane, or removed from the
DART 7 target.

Compatibility cleanup reviews should record:

- migration notes or the replacement API for each deprecated surface;
- `CHANGELOG.md` entries for user-visible additions, removals, or migration
  guidance;
- package/export status for affected CMake components, headers, examples, or
  package-manager instructions;
- DART 6 LTS/gz-physics compatibility evidence when the surface affects released
  downstream users on that support lane;
- API-boundary classification for any public-looking surface that remains past
  DART 7.

## Contributing During the Transition

- Prioritize work that advances or hardens the easy-start research API, including Python-first workflows where they are the best user path.
- When updating DART 6 LTS compatibility code, validate gz-physics compatibility
  as described above.
- Share feedback through issues or pull requests if the roadmap overlooks critical scenarios.
