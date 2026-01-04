# C++20 Modernization Progress

## Status

- Active branch: `cpp20/phase19`
- Active PR (Phase 19): https://github.com/dartsim/dart/pull/2399
- Phase 18 PR: https://github.com/dartsim/dart/pull/2398
- Phase 17 PR: https://github.com/dartsim/dart/pull/2397
- Phase 16 PR: https://github.com/dartsim/dart/pull/2394
- Phase 15 PR: https://github.com/dartsim/dart/pull/2393
- Phase 14 PR: https://github.com/dartsim/dart/pull/2390
- Phase 13 PR: https://github.com/dartsim/dart/pull/2387
- Phase 12 PR: https://github.com/dartsim/dart/pull/2385
- Phase 11 PR: https://github.com/dartsim/dart/pull/2384
- Phase 10 PR: https://github.com/dartsim/dart/pull/2382 (merged)
- Phase 9 PR: https://github.com/dartsim/dart/pull/2380 (merged)
- Phase 8 PR: https://github.com/dartsim/dart/pull/2376 (merged)
- Phase 7 PR: https://github.com/dartsim/dart/pull/2375 (merged; empty-span guard fix)
- CI monitoring and merge handled by the user.
- Phase 11 compatibility: keep `CollisionDetector`/`BoxedLcpSolver` `getType()`
  returning `const std::string&`; added `getTypeView()` for
  `std::string_view`.
- Latest local validation: `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run test-all`;
  `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run -e gazebo test-gz`
  (passed).

## Completed

- Phase 3: PR #2365 (merged)
- Phase 4: PR #2367 (merged)
- Phase 5: PR #2371 (merged)
- Phase 6: PR #2373 (merged)
- Phase 7: PR #2375 (merged)
- Phase 8: PR #2376 (merged)
- Phase 9: PR #2380 (merged)
- Phase 10: PR #2382 (merged)
- Earlier phases were merged prior to this tracker (see git history).

## Next

- Monitor PR #2384, #2385, #2387, #2390, #2393, #2394, #2397, #2398, and #2399
  CI/review feedback (CI/merge handled by the user).
