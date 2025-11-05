# DART Release Roadmap

DART is moving toward a Python-first experience while keeping production users productive during the transition. This roadmap highlights what stays stable, what changes, and how you can help.

## DART 6: Stable Line

- `release-6.16` remains the maintenance branch for the established DART 6 API. Expect targeted build fixes and critical bug patches only.
- `release-6.15` is archived for historical reference with no further updates.
- Backward compatibility is validated in practice by building against gz-physics with `pixi run -e gazebo test-gz`.

## DART 7: Transition Release

- `main` targets DART 7.0.0 with a Python-first API layered on a refreshed C++ architecture.
- The Python-first API remains in a pre-release state throughout every 7.y.z build, so breaking changes can land between minor or patch releases.
- The C++ surface ships alongside the Python API and stays supported on a best-effort basis so established workflows keep working while users migrate.
- Compatibility for the legacy surface focuses on high-signal validation scenarios rather than strict guarantees. Regressions get fixed as they surface.
- The legacy DART 6 API remains available but receives maintenance fixes only. Deprecations begin removal in the 7.1 window.
- Algorithmic behavior aligns with DART 6 while the refreshed architecture opens room for multi-core gains. GPU acceleration is not currently planned.

## DART 8: Clean Break

- DART 8 removes the legacy DART 6 API and code paths entirely, leaving only the new Python-centric interface.
- DART 8 becomes the first stable release of the new API; from here we preserve backward compatibility on a best-effort basis.
- Users who still need the legacy surface should remain on DART 7 or DART 6, where that code continues in maintenance.

## Contributing During the Transition

- Prioritize work that advances or hardens the Python-first API.
- When updating legacy code, run `pixi run -e gazebo test-gz` to confirm practical compatibility.
- Share feedback through issues or pull requests if the roadmap overlooks critical scenarios.
