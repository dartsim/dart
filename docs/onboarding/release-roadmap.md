# DART Release Roadmap

DART is moving toward an easier-start research experience while keeping
production users productive on a best-effort basis during the transition.
Python-first APIs are a major part of that direction, layered on a refreshed
C++20 core. This roadmap reflects the current codebase and will become the
source of truth once it is locked.

## Compatibility & Deprecation Policy

- New major releases can introduce API-breaking changes.
- Deprecated APIs are removed in the next major release on a best-effort basis, including for gz-physics integrations.
- Backward compatibility for gz-physics is validated by building and testing against gz-physics with `pixi run -e gazebo test-gz`.

## DART 6: Stable Line

- `release-6.16` remains the maintenance branch for the established DART 6 API. Expect targeted build fixes and critical bug patches only.

## DART 7: Transition Release

- `main` targets DART 7.0.0 with a Python-first API layered on a refreshed C++ architecture.
- The legacy DART 6 API remains available throughout DART 7 to support migration and is deprecated rather than removed during DART 7.
- The classic Skeleton-backed world remains the stable DART 7 simulation path.
  The ECS-backed world is an opt-in experimental surface under
  `dart::simulation::experimental` and `dartpy.simulation_experimental`.
- Algorithmic behavior aligns with DART 6 while the refreshed architecture opens room for multi-core gains. GPU acceleration remains an evidence-gated roadmap item, not a DART 7 public API commitment, until workload benchmarks and packaging constraints justify a backend choice.

### DART 7 Checkable Gates

| Gate                         | Required evidence                                                                                       | Local or CI command                                                                                                                                             |
| ---------------------------- | ------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| First simulation works       | README Python, C++ package-project, and Pixi source first-success commands are verified or blocked.     | Run the README quick starts, `pixi run test-published-package-quickstarts`, and `pixi run check-dart7-artifacts`.                                               |
| Core build and tests         | Lint, C++ build, Python build, and focused or full test suites pass for the changed release scope.      | `pixi run lint`, `pixi run build`, `pixi run test-unit`, `pixi run test-py`, or `pixi run test-all`.                                                            |
| Public API boundaries        | New or promoted APIs are classified as supported, experimental, compatibility, or internal.             | `pixi run check-api-boundaries` plus review of `docs/onboarding/api-boundaries.md`.                                                                             |
| LCP/contact baseline         | Solver contract and benchmark smoke evidence are recorded before algorithm or compute-scaling promises. | `ctest --test-dir build/default/cpp/Release -R UNIT_math_lcp_math_lcp_all_solvers_smoke`; `pixi run bm lcp_compare -- --benchmark_filter=BM_LCP_COMPARE_SMOKE`. |
| GUI transition               | Any GUI promotion stays aligned with the Filament migration gates and keeps backend details hidden.     | Filament dev-task gates in `docs/dev_tasks/filament_gui/08-north-star-migration.md`.                                                                            |
| gz-physics compatibility     | Legacy compatibility fixes that affect released users are checked against gz-physics.                   | `pixi run -e gazebo test-gz` when the change touches compatibility surfaces used by gz-physics.                                                                 |
| Release metadata and package | Version metadata, changelog, CMake package exports, and package-manager instructions match the release. | `pixi run lint`; package-project configure/build commands from PLAN-010; release packaging checklist below.                                                     |
| AI workflow/documentation    | AI-facing workflow adapters and plan docs stay synchronized when release workflows change.              | `pixi run sync-ai-commands`, `pixi run check-ai-commands`, and docs gates from `docs/ai/verification.md`.                                                       |

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
- Compatibility: changes touching legacy release surfaces run
  `pixi run -e gazebo test-gz` or record why gz-physics is not affected.
- AI/docs: release workflow or plan changes run the AI docs/adapters gates from
  `docs/ai/verification.md`.

## DART 8: Clean Break

- DART 8 removes the legacy DART 6 API and deprecated DART 7 APIs.
- DART 8 is the promotion point for the ECS-backed world once it has stable
  public wrappers, migration notes, and parity gates; the classic
  Skeleton-backed `World` is removed rather than mixed with the new world.
- DART 8 becomes the first stable release of the new API; from here we preserve backward compatibility on a best-effort basis.
- Users who still need the legacy surface should remain on DART 7 or DART 6, where that code continues in maintenance.

### DART 8 Compatibility-Debt Inventory

Track these throughout DART 7 so DART 8 removals are deliberate:

| Debt area                         | DART 7 handling                                                                                            | DART 8 gate                                                                                           |
| --------------------------------- | ---------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Legacy DART 6 API surface         | Keep available and deprecated; avoid expanding it unless needed for migration or gz-physics compatibility. | Removal list has migration notes, changelog entries, and gz-physics status.                           |
| Classic Skeleton-backed `World`   | Keep as the stable DART 7 simulation path.                                                                 | ECS-backed world has stable wrappers, migration notes, focused tests, and parity evidence.            |
| Experimental world bindings       | Keep under `dart::simulation::experimental` and `dartpy.simulation_experimental`.                          | Promotion decision has parity gates and documented import/API migration.                              |
| Deprecated CMake/package surfaces | Keep release packaging and compatibility shims only where they support DART 7 users.                       | Package exports and examples no longer require deprecated component or header names.                  |
| Solver implementation details     | Keep Dantzig helper APIs, contact caches, and backend storage out of research-facing contracts.            | Any remaining public-looking internals are classified as supported, compatibility, or removed.        |
| GUI backend exposure              | Keep Filament and other backend details behind migration gates.                                            | Public GUI API is backend-hidden and old backend-specific entry points have a migration/removal path. |

Each release packaging pass should update this inventory when a deprecated API
is added, promoted, kept for compatibility, or removed from the DART 8 target.

Compatibility cleanup reviews should record:

- migration notes or the replacement API for each deprecated surface;
- `CHANGELOG.md` entries for user-visible additions, removals, or migration
  guidance;
- package/export status for affected CMake components, headers, examples, or
  package-manager instructions;
- gz-physics compatibility evidence when the surface affects released
  downstream users;
- API-boundary classification for any public-looking surface that remains past
  DART 7.

## Contributing During the Transition

- Prioritize work that advances or hardens the easy-start research API, including Python-first workflows where they are the best user path.
- When updating legacy code, validate gz-physics compatibility as described above.
- Share feedback through issues or pull requests if the roadmap overlooks critical scenarios.
