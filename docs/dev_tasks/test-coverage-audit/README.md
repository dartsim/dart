# Test Coverage Audit — Dev Task

## Current Status

- [x] Phase 1: Run coverage report, identify gaps
- [x] Phase 2: Add tests for collision module
- [x] Phase 3: Add tests for constraint module
- [x] Phase 4: Add tests for dynamics module
- [x] Phase 5: Add tests for common module
- [ ] Phase 6: Reach 80% coverage target

## Goal

Improve test coverage from ~57% to 80%+ across the DART codebase.

## Non-Goals

- GUI module coverage (requires OpenGL context, 14% baseline)
- 100% coverage (diminishing returns)

## Key Decisions

- **Simulation-based tests for constraints**: Most constraint methods (`update()`, `isActive()`, `excite()`, `applyImpulse()`) are protected and can only be tested through `World::step()` via ConstraintSolver.
- **ExposedConstraint pattern**: Use subclasses with `using` declarations to expose protected methods for unit testing.
- **sccache added**: Added to pixi for faster coverage builds.

## Coverage Analysis

| Module     | Coverage | Lines | Priority |
| ---------- | -------- | ----- | -------- |
| GUI        | 14.3%    | 5376  | Low      |
| Constraint | 57%      | 3798  | High     |
| Common     | 60.8%    | 1996  | High     |
| Math       | 67.5%    | 5953  | Medium   |

### Low-Coverage Constraint Files

- `SoftContactConstraint.cpp` - 0% (452 lines) - needs soft body setup
- `BallJointConstraint.cpp` - 19.4% → improved with simulation tests
- `WeldJointConstraint.cpp` - 19.0% → improved with simulation tests
- `ServoMotorConstraint.cpp` - 25.0% → improved with unit tests
- `JointLimitConstraint.cpp` - 37.7% → improved with unit tests

## Tests Added

### This Task (Total: ~238 tests)

- **Collision module**: Distance, DistanceFilter, CollisionGroup, etc.
- **Constraint module**: BallJoint, WeldJoint, ServoMotor, JointLimit simulation tests
- **Dynamics module**: BodyNodePtr, WeakBodyNodePtr, ShapeNodePtr tests
- **Common module**: Exception, Result, Stopwatch tests

## Immediate Next Steps

1. Run `pixi run coverage-report` to measure current coverage
2. Identify remaining low-coverage files
3. Add tests targeting specific uncovered code paths
4. Repeat until 80% threshold reached
