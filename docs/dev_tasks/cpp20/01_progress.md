# C++20 Modernization Progress

## Status

- Active branch: `cpp20/phase7` (stacked on `cpp20/phase6`)
- Phase 6 PR: https://github.com/dartsim/dart/pull/2373 (user monitors CI/merge)
- Phase 7: in progress (span-first read-only inputs)

## Completed

- Phase 3: PR #2365 (merged)
- Phase 4: PR #2367 (merged)
- Phase 5: PR #2371 (merged)
- Earlier phases were merged prior to this tracker (see git history).

## Next

- Finish Phase 7 and open a stacked PR from `cpp20/phase7`.
- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening the phase PR.

## Notes

- Phase 7 updates so far: span inputs for Group/CollisionGroup/Recording,
  CloneableVector, ReferentialSkeleton helpers, Skeleton setAllMemberObjectData,
  and IkFast dof map validation.
