# Contact Manifold Cache Testing Plan (03)

## Unit Tests

Add unit tests that operate on synthetic contacts with deterministic inputs.
Suggested location: `tests/unit/constraint/`.

- Patch update logic
  - Insert up to 4 points, ensure count and ordering are correct
  - Replace points when full, ensure deepest contact is retained
  - Uniqueness thresholds prevent near-duplicate churn
- TTL behavior
  - Pairs expire after N frames without contacts
  - Pairs are reused if contacts return before TTL expires
- Determinism
  - Identical inputs produce identical output ordering

Implemented:

- `tests/unit/constraint/test_ContactManifoldCache.cpp`
  - Persistence across frames and churn reduction
  - Cap at 4 contacts and deepest preservation
  - TTL pruning behavior and stale-output retention

## Integration Tests

Suggested location: `tests/integration/constraint/` or
`tests/integration/simulation/`.

- Resting contact stability
  - Box on plane over many steps
  - Expect reduced contact churn when enabled
- Stacking stability
  - Two or more boxes stacked
  - Expect fewer changes in contact count and normals when enabled
- Sliding contact
  - Box sliding on plane or slope
  - Ensure persistent cache does not over-stabilize (contacts update)

Implemented:

- `tests/integration/simulation/test_ContactManifoldCache.cpp`
  - Verifies collision results remain stable when the cache is toggled on
  - Confirms `getContactsUsedForConstraints()` count matches expectations
  - Confirms `World::getContactsUsedForConstraints()` mirrors solver output
  - Confirms cached runs sync non-zero forces into `CollisionResult`

## Legacy Behavior Tests (Feature OFF)

- Verify that enabling the feature flag is the only behavior change
- With flag OFF, contact counts and constraint counts match the legacy path
  within tolerances

## Churn Metrics

Add a helper to compute per-pair contact churn:

- Number of points that changed beyond thresholds frame-to-frame
- Count of contact additions/removals per pair over a window

Use this to assert improvement when enabled and parity when disabled.

## How to Run

Suggested:

```bash
pixi run test
```

Targeted:

```bash
ctest --test-dir build/default/cpp/Release --output-on-failure -R ContactManifold
```

## Executed

```bash
pixi run -- cmake --build build/default/cpp/Release --target UNIT_constraint_ContactManifoldCache
pixi run -- cmake --build build/default/cpp/Release --target INTEGRATION_simulation_ContactManifoldCache
pixi run -- ctest --test-dir build/default/cpp/Release --output-on-failure -R ContactManifoldCache
```
