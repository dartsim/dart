# Contact Manifold Cache Integration Plan (02)

## Status

- Steps 1-6 implemented with `ContactManifoldCache`

## Step-by-step Code Changes

1. Add cache types
   - New files: `dart/constraint/ContactManifoldCache.hpp` and
     `dart/constraint/ContactManifoldCache.cpp`
   - Define `ContactManifoldCacheOptions`, `ContactManifold`, and update APIs

2. Wire cache into `ConstraintSolver`
   - Add members in `dart/constraint/ConstraintSolver.hpp`:
     - `ContactManifoldCache mContactManifoldCache`
     - `std::vector<collision::Contact> mPersistentContacts`
     - `ContactManifoldCacheOptions mContactManifoldOptions`
   - Add setters/getters in `dart/constraint/ConstraintSolver.hpp/.cpp`:
     - `setContactManifoldCacheOptions(...)`
     - `getContactManifoldCacheOptions() const`
     - `setContactManifoldCacheEnabled(bool)` convenience helper

3. Update constraint creation path
   - In `dart/constraint/ConstraintSolver.cpp`:
     - After `mCollisionGroup->collide(...)`, call
       `mContactManifoldCache.update(mCollisionResult, mContactManifoldOptions, ...)`
     - Select contacts for constraint creation:
       - feature OFF: use `mCollisionResult.getContacts()`
       - feature ON: use `mPersistentContacts`

4. Keep `CollisionResult` semantics unchanged
   - Do not modify `mCollisionResult` or the `World::getLastCollisionResult()`
     behavior in `dart/simulation/World.cpp`

5. Cache lifecycle management
   - Reset cache on `ConstraintSolver::setCollisionDetector()`
   - Reset cache on `ConstraintSolver::removeSkeleton()` and
     `ConstraintSolver::removeAllSkeletons()` (initial safe choice)
   - Consider fine-grained removal later by pair key

6. Backend contact history coordination (optional but recommended)
   - Add `CollisionOption::useBackendContactHistory` (default true) in
     `dart/collision/CollisionOption.hpp/.cpp`
   - In `dart/collision/ode/OdeCollisionDetector.cpp`, gate history reuse on
     this flag to avoid double persistence
   - When ContactManifoldCache is enabled, set
     `mCollisionOption.useBackendContactHistory = false`
   - Status: implemented (auto-disabled per-step when cache is enabled)

7. Reporting contacts used for constraints
   - Add `ConstraintSolver::getContactsUsedForConstraints(...)`
   - Add `World::getContactsUsedForConstraints(...)` convenience API
   - Update `World::bake()` to record constraint contacts for forces/points
   - Status: implemented

8. Sync constraint forces into raw contacts (cache enabled)
   - After solving, copy forces from persistent contacts back into
     `mCollisionResult` for matching rigid contacts
   - Status: implemented

9. Feature flag integration
   - Runtime-only flag in `ConstraintSolver` options (default OFF)
   - Expose through `World` or demo code as needed

## Risks and Mitigations

- Stale collision object pointers in cached contacts
  - Mitigation: reset cache on skeleton add/remove and detector changes
- Determinism issues from unordered iteration
  - Mitigation: stable sort of pair keys and deterministic tie-breaks
- Double persistence in backends that already cache
  - Mitigation: optional backend history disable flag as described above
- Soft-body contacts
  - Mitigation: keep soft contacts on legacy path initially

## Testing, Benchmarking, Demo Plan

- Tests: `docs/dev_tasks/contact_manifold/03_testing.md`
- Benchmarks: `docs/dev_tasks/contact_manifold/04_benchmarking.md`
- GUI demo: `docs/dev_tasks/contact_manifold/05_demo.md`
