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
- Algorithmic behavior aligns with DART 6 while the refreshed architecture opens room for multi-core gains. GPU acceleration remains an evidence-gated roadmap item, not a DART 7 public API commitment, until workload benchmarks and packaging constraints justify a backend choice.

## DART 8: Clean Break

- DART 8 removes the legacy DART 6 API and deprecated DART 7 APIs.
- DART 8 becomes the first stable release of the new API; from here we preserve backward compatibility on a best-effort basis.
- Users who still need the legacy surface should remain on DART 7 or DART 6, where that code continues in maintenance.

## Contributing During the Transition

- Prioritize work that advances or hardens the easy-start research API, including Python-first workflows where they are the best user path.
- When updating legacy code, validate gz-physics compatibility as described above.
- Share feedback through issues or pull requests if the roadmap overlooks critical scenarios.
