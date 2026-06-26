# RESUME — DART 7 Core-Dynamics Perf Forward-Port

## Branch

`perf/core-dynamics-forwardport` (from `origin/main`).

## How to continue

1. Read [README.md](README.md) for scope + the verified freeze-policy rules.
2. Derive exact patches per sub-opt from the source release-6.20 commits
   (`git show <commit>`), re-expressed against main's snake_case tree:
   - `#3028` `32081a2b81b`, `#3037` `16632b39902`, `#3033` `346e315958e`,
     `#3040` `ec8b3a6a5d3` (sub-opt 4b only), `#3023` `fab90e3da41` (5a + 5c only).
3. Apply sequentially (contact_constraint.{cpp,hpp} is touched by #3033/#3040/#3023
   — apply in that order to avoid conflicts). Keep new header members **private**.
4. Build incrementally: `cmake -S . -B build/default/cpp/Release` then
   `cmake --build build/default/cpp/Release --target dart UNIT_dynamics_<X> ...`.
   Note: dynamics/constraint unit tests are registered explicitly via
   `dart_add_test(...)` in `tests/unit/CMakeLists.txt` (NOT globbed).
5. Run affected tests; then `pixi run check-lint` (freeze gate must pass with no
   tags); update `CHANGELOG.md`; open PR (milestone DART 7.0).

## Done so far

- Audit complete; freeze feasibility verified (all sub-opts stay private/.cpp/math
  → clean). Branch created.

## Pitfalls

- `setActuatorTypes`-style `std::span` params reject braced-init-lists in tests
  (use explicit `std::vector<...>{...}`) — seen in the sibling actuator PR #3204.
- `mSpatialNormalA/B` are already **private** in `contact_constraint.hpp`
  (post-`private:` at ~line 184) — type change there is freeze-exempt.
