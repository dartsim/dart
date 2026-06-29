# RESUME — DART 7 Core-Dynamics Perf Forward-Port

## Branch

`perf/core-dynamics-forwardport` (from `origin/main`).

## How to continue (remaining work only — see README "Remaining")

Do these on fresh `main` AFTER #3205 merges (both touch files #3205 changed):

1. **`#3023` 5a** — ContactConstraint skeleton/reactive/tangent caching from
   `fab90e3da41`, re-expressed against `contact_constraint.{hpp,cpp}`; keep new
   members **private**; reconcile with #3033's already-landed conditions.
2. **`#3023` 5c** — contact-pair counting in `constraint_solver.cpp`. Do NOT use a
   `static thread_local` map (reentrancy hazard — see README). Use a per-call /
   per-solver reentrancy-safe, allocation-free flat counter (the release used a
   vector-backed open-addressed structure), or drop the sub-opt.
3. Build (`cmake -S . -B build/default/cpp/Release` then
   `--target dart UNIT_constraint_* UNIT_dynamics_*`); dynamics/constraint unit
   tests are registered explicitly via `dart_add_test(...)` in
   `tests/unit/CMakeLists.txt` (NOT globbed). Run them; `pixi run check-lint`
   (freeze gate clean, no tags); update `CHANGELOG.md`; open PR (milestone DART 7.0).

## Done so far

- Audit + freeze feasibility verified. Shipped: #3204 (actuator fix), #3205
  (bit-exact batch — 6 sub-opts), #3206 (FreeJoint fast-path).
- `#3023` 5c's first cut (static thread_local unordered_map) was reverted from
  #3205 per Codex (bucket-only reuse + reentrancy hazard); redo per step 2 above.

## Pitfalls

- `setActuatorTypes`-style `std::span` params reject braced-init-lists in tests
  (use explicit `std::vector<...>{...}`) — seen in the sibling actuator PR #3204.
- `mSpatialNormalA/B` are already **private** in `contact_constraint.hpp`
  (post-`private:` at ~line 184) — type change there is freeze-exempt.
