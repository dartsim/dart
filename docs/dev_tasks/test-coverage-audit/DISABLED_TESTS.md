# Disabled Tests Analysis

This document analyzes the disabled tests in the DART test suite.

## Summary

| Test                                          | Location              | Status      | Reason                          |
| --------------------------------------------- | --------------------- | ----------- | ------------------------------- |
| `ClonedWorldsStayDeterministic`               | test_Issue410.cpp:133 | **ENABLED** | Valid regression test           |
| `ContactsReportNonZeroForceWithLargeTimeStep` | test_Issue410.cpp:189 | **ENABLED** | Valid physics test              |
| `CoMJacobianSignConsistency`                  | test_Joints.cpp:3015  | **ENABLED** | Valid kinematic test            |
| `HierarchicalTransforms`                      | test_frame.cpp:121    | DISABLED    | Requires Phase 2 implementation |

**Result**: 3 of 4 previously disabled tests have been enabled and verified passing.

---

## Remaining Disabled Test

### DISABLED_HierarchicalTransforms

**File**: `tests/unit/simulation/experimental/frame/test_frame.cpp:121`

**Why it remains disabled**:
The test validates automatic hierarchical cache invalidation where modifying a parent frame's transform should automatically invalidate child frame caches. This feature is not yet implemented in the experimental simulation module (Phase 1). The test documents the expected Phase 2 behavior.

**Verification**: Running the test without DISABLED\_ prefix fails:

```
Value of: T_child_world_new.isApprox(T_expected_new)
  Actual: false
Expected: true
```

**Action Required**: Implement automatic hierarchical cache invalidation in Phase 2, then enable this test.

---

## Enabled Tests Verification

All 3 newly enabled tests pass:

```bash
$ ctest -R "INTEGRATION_simulation_Issue410"
1/1 Test #53: INTEGRATION_simulation_Issue410 ...   Passed    3.17 sec

$ ctest -R "INTEGRATION_dynamics_Joints"
1/1 Test #27: INTEGRATION_dynamics_Joints ......   Passed    6.26 sec
```

Full integration test suite: **64/64 tests passed**

---

## Coverage Context (from existing coverage.info)

Coverage results from the current `coverage.info`:

| Scope                               | Line Coverage         | Function Coverage    |
| ----------------------------------- | --------------------- | -------------------- |
| All DART source (`dart/`)           | 62.8% (28,509/45,371) | 58.1% (5,976/10,285) |
| Core DART (excl. gui, experimental) | 69.4% (27,739/39,995) | 61.3% (5,883/9,599)  |
| codecov.yml target                  | 85%                   | -                    |

Notes:

- `dart/simulation/experimental/**` is already excluded in `codecov.yml`, so the remaining disabled test does not affect coverage gating.
- Core coverage is ~16% below target; disabled tests in experimental are not the driver.
