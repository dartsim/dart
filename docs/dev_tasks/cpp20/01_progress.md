# C++20 Modernization Progress

## Status

- Active branch: `cpp20/phase8`
- Active PR: pending (phase 8 not opened yet)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (updated with empty-span guards)
- CI monitoring and merge handled by the user.

## Completed

- Phase 3: PR #2365 (merged)
- Phase 4: PR #2367 (merged)
- Phase 5: PR #2371 (merged)
- Phase 6: PR #2373 (merged)
- Earlier phases were merged prior to this tracker (see git history).

## Next

- Execute Phase 8 from `00_plan.md` (string_view parsing inputs).
- Focus on XML/SDF/URDF/Skel helpers and avoid behavioral changes.
- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening the phase PR.
