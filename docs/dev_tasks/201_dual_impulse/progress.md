# Issue 201 - Progress

## Status

- Branch: issue/201_split_impulse
- Baseline: origin/main (38c08ab41a7)

## Work Log

- Created initial regression test: tests/integration/constraint/test_SplitImpulse.cpp
- Added test target in tests/integration/CMakeLists.txt
- Added plan/progress docs in docs/dev_tasks/201_dual_impulse/
- Added split impulse phases to constraint info and contact constraint biasing.
- Added position impulse storage/forward-dynamics path in BodyNode/Skeleton.
- Added position LCP pass in ConstraintSolver and position integration in ClassicRigidSolver.
- Updated box_stacking GUI to toggle split impulse.
- Confirmed issue reproduces on origin/main using a temporary worktree:
  Issue201 test fails with body->getLinearVelocity().z() ≈ 1e-3 (expected 0).
- Checked for split impulse references in Bullet sources: no src/BulletDynamics
  tree in repo; no hits under dart/collision/bullet.
- Restored legacy contact error reduction when split impulse is disabled and
  made split impulse opt-in by default; tests now enable it explicitly.

## Validation

- DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run test (PASS).
- pixi run lint (PASS).
- DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run test-all (PASS).
- DART_PARALLEL_JOBS=16 CTEST_PARALLEL_LEVEL=16 pixi run -e gazebo test-gz (PASS).

## Next

- Sync with origin/main, commit, push, open PR, and monitor CI.
