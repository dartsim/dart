# DART Release Roadmap

DART is moving toward an easier-start research experience with a clean DART 7
API. Python-first APIs are a major part of that direction, layered on a
refreshed C++20 core. Compatibility for the established DART 6 API lives on the
`release-6.16` maintenance line instead of blocking the main-branch DART 7
design.

## Compatibility & Deprecation Policy

- New major releases can introduce API-breaking changes.
- DART 7 is the clean-break release for the DART 6 to DART 7 transition. Do not
  add new DART 6 compatibility shims on main unless a maintainer explicitly
  scopes the work as migration support.
- DART 6 compatibility work, including Gazebo/gz-physics compatibility fixes,
  belongs on the DART 6.16 support lane by default.
- Required gz-physics compatibility evidence is validated with
  `pixi run -e gazebo test-gz` on `release-6.16` or release branches. The
  main-branch Gazebo workflow is a migration canary, not a DART 7 API design
  constraint.

## DART 6.16: Compatibility Line

- `release-6.16` is the maintained branch for the established DART 6 API.
- The pinned gz-physics branch for the current support lane is
  `gz-physics9_9.0.0`, matching the `pixi.toml` Gazebo integration task.
- Backport critical bug fixes, build fixes, security fixes, and
  Gazebo/gz-physics compatibility fixes that released downstream users need.
- Do not backport normal DART 7 features by default.
- Publish the Gazebo support window, branch/version matrix, and sunset date or
  sunset trigger before DART 7 removes legacy API surfaces from main.

## DART 7: Clean-Break Release

- `main` targets DART 7.0.0 as a clean break from the DART 6 public API, with a
  Python-first API layered on the refreshed C++ architecture.
- The classic Skeleton-backed DART 6 API is not a DART 7 compatibility promise.
  Support for users that still need that API remains on `release-6.16`.
- The ECS-backed world is the DART 7 public API target, but promotion is gated
  by core robotics parity evidence. Until promotion, its development namespace
  remains `dart::simulation::experimental` and
  `dartpy.simulation_experimental`.
- Algorithmic behavior must cover basic robotics simulation before release.
  IPC, VBD, differentiable simulation, and GPU work remain opt-in roadmap
  features unless a promoted public API depends on them.

### DART 7 Checkable Gates

| Gate                         | Required evidence                                                                                                                    | Local or CI command                                                                                                                                             |
| ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Experimental model loading   | URDF, SDF, MJCF, and SKEL load into the promoted world API with topology, DOF, transform, mass/inertia, collision, and Python tests. | Focused experimental loader tests plus `pixi run test-simulation-experimental` and `pixi run test-py`.                                                          |
| Rigid dynamics parity        | Shared open-chain scenes match the classic DART 6 path within documented tolerances for gravity, integration, drift, and controls.   | Dedicated world-parity suite and focused solver tests.                                                                                                          |
| Contact/constraint parity    | Contact response, friction, joint limits, motors, mimic/coupler behavior, and loop closures have tests and migration examples.       | Focused experimental contact/constraint tests and parity suite rows.                                                                                            |
| Serialization/replay parity  | World topology, state, model assets, and record/replay round-trip with bounded error.                                                | Serialization/replay tests for promoted world APIs.                                                                                                             |
| Stable public API promotion  | Promoted APIs hide ECS storage, component types, solver registries, backend details, and implementation escape hatches.              | `pixi run check-api-boundaries`, generated stubs, docs, and migration snippets.                                                                                 |
| First simulation works       | README Python, C++ package-project, and Pixi source first-success commands are verified or blocked.                                  | Run the README quick starts, `pixi run test-published-package-quickstarts`, and `pixi run check-dart7-artifacts`.                                               |
| Core build and tests         | Lint, C++ build, Python build, and focused or full test suites pass for the changed release scope.                                   | `pixi run lint`, `pixi run build`, `pixi run test-unit`, `pixi run test-py`, or `pixi run test-all`.                                                            |
| LCP/contact baseline         | Solver contract and benchmark smoke evidence are recorded before algorithm or compute-scaling promises.                              | `ctest --test-dir build/default/cpp/Release -R UNIT_math_lcp_math_lcp_all_solvers_smoke`; `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`. |
| GUI transition               | Any GUI promotion stays aligned with the Filament migration gates and keeps backend details hidden.                                  | Filament gates in the GUI plan/design docs.                                                                                                                     |
| DART 6.16 support policy     | Gazebo branch/version matrix, backport scope, and sunset date or sunset trigger are published.                                       | `pixi run -e gazebo test-gz` on `release-6.16` or release branches when compatibility surfaces change.                                                          |
| Release metadata and package | Version metadata, changelog, CMake package exports, and package-manager instructions match the release.                              | `pixi run lint`; package-project configure/build commands from PLAN-010; release packaging checklist below.                                                     |
| AI workflow/documentation    | AI-facing workflow adapters and plan docs stay synchronized when release workflows change.                                           | `pixi run sync-ai-commands`, `pixi run check-ai-commands`, and docs gates from `docs/ai/verification.md`.                                                       |

### Release Packaging Checklist

For a DART 7 packaging or release-hardening pass, record evidence for the
changed scope before opening the PR:

- Version metadata: `package.xml`, `pixi.toml`, generated package metadata, and
  example `find_package(DART ...)` requirements agree on the release version.
- Changelog: `CHANGELOG.md` has the release section, date, milestone link, and
  entries for user-visible fixes or changes.
- Package exports: any changed CMake components configure, install, and work
  from a temporary prefix with `pixi run test-cpp-quickstart -- --prefix
<installed-dart-prefix>`.
- First-success paths: README Python, C++ package-project, and Pixi source
  commands either pass or name the current package-artifact blocker.
- Compatibility: changes touching the DART 6.16 compatibility lane run
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
- Users who still need the DART 6 surface should remain on DART 6.16.x until
  they migrate or the published support window ends.

### DART 7 Compatibility-Debt Inventory

Track these before DART 7 ships so clean-break removals are deliberate:

| Debt area                         | DART 7 handling                                                                                                | Clean-break gate                                                                                      |
| --------------------------------- | -------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Legacy DART 6 API surface         | Remove from main unless explicitly needed for a bounded migration adapter; keep released support on DART 6.16. | Removal list has migration notes, changelog entries, package status, and DART 6.16 support status.    |
| Legacy dartpy 6 API surface       | Remove from the DART 7 public contract rather than carrying a compatibility layer.                             | Replacement import/API path is documented and covered by tests/stubs.                                 |
| Classic Skeleton-backed `World`   | Treat as the deletion target once the promoted world has parity evidence.                                      | ECS-backed world has stable wrappers, migration notes, focused tests, and parity evidence.            |
| Experimental world bindings       | Keep under `dart::simulation::experimental` and `dartpy.simulation_experimental` until promotion gates pass.   | Promoted as the official DART 7/dartpy 7 API with documented C++ and Python migration paths.          |
| Deprecated CMake/package surfaces | Remove package compatibility shims that would make DART 7 satisfy DART 6/gz-physics package ranges.            | Package exports and examples no longer require deprecated component or header names.                  |
| Solver implementation details     | Keep Dantzig helper APIs, contact caches, and backend storage out of research-facing contracts.                | Any remaining public-looking internals are classified as supported, compatibility, or removed.        |
| GUI backend exposure              | Keep Filament and other backend details behind migration gates.                                                | Public GUI API is backend-hidden and old backend-specific entry points have a migration/removal path. |

Each release packaging pass should update this inventory when a deprecated API
is added, promoted, moved to the DART 6.16 support lane, or removed from the
DART 7 target.

Compatibility cleanup reviews should record:

- migration notes or the replacement API for each deprecated surface;
- `CHANGELOG.md` entries for user-visible additions, removals, or migration
  guidance;
- package/export status for affected CMake components, headers, examples, or
  package-manager instructions;
- DART 6.16/gz-physics compatibility evidence when the surface affects released
  downstream users on that support lane;
- API-boundary classification for any public-looking surface that remains past
  DART 7.

## Contributing During the Transition

- Prioritize work that advances or hardens the easy-start research API, including Python-first workflows where they are the best user path.
- When updating DART 6.16 compatibility code, validate gz-physics compatibility
  as described above.
- Share feedback through issues or pull requests if the roadmap overlooks critical scenarios.
