# C++20 Modernization Progress

## Status

- Active branch: `cpp20/phase7` (based on `main`)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375
- Phase 7: in progress (span-first read-only inputs)

## Completed

- Phase 3: PR #2365 (merged)
- Phase 4: PR #2367 (merged)
- Phase 5: PR #2371 (merged)
- Phase 6: PR #2373 (merged)
- Earlier phases were merged prior to this tracker (see git history).

## Next

- Keep the PR up to date with `main` (resolve conflicts and push).
- Run `pixi run lint` before each commit.
- Run `pixi run test-all` before opening the phase PR.

## Notes

- Phase 7 updates so far: span inputs for Group/CollisionGroup/Recording,
  CloneableVector, ReferentialSkeleton helpers, Skeleton setAllMemberObjectData,
  and IkFast dof map validation.
