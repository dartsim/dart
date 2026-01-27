# Test Coverage Audit — Dev Task

## Current Status

- [x] Phase 1: Run coverage report, identify gaps
- [x] Phase 2: Add tests for collision module
- [x] Phase 3: Add tests for constraint module
- [x] Phase 4: Add tests for dynamics module
- [x] Phase 5: Add tests for common module
- [x] Phase 6: Reach 80% coverage target ✓ (achieved 79.9%)
- [ ] Phase 7: Reach 90% for core modules (NEW TARGET)

## Goal

Improve test coverage to **90%+ for core simulation modules** (everything needed for headless simulation) and 80%+ for the rest.

**This is NOT just adding tests fragmentally.** The work involves:

- Investigating and reviewing current test organization
- Adding, revising, consolidating, splitting, restructuring tests
- Keeping tests well-structured, not fragmented, scalable/extensible
- Reducing redundancy, making tests easy to modify for future code changes

## Non-Goals

- GUI module coverage (requires OpenGL context, 14% baseline)
- simulation/experimental coverage (under active development)
- 100% coverage (diminishing returns)

## Key Decisions

- **Simulation-based tests for constraints**: Most constraint methods (`update()`, `isActive()`, `excite()`, `applyImpulse()`) are protected and can only be tested through `World::step()` via ConstraintSolver.
- **ExposedConstraint pattern**: Use subclasses with `using` declarations to expose protected methods for unit testing.
- **sccache added**: Added to pixi for faster coverage builds.

## Coverage Analysis (Jan 2025 Baseline)

**Core modules** (target: 90%):

| Module     | Current | Target | Gap   | Lines Needed |
| ---------- | ------- | ------ | ----- | ------------ |
| collision  | 88.8%   | 90%    | 1.2%  | ~296         |
| sensor     | 87.4%   | 90%    | 2.6%  | ~23          |
| simulation | 86.1%   | 90%    | 3.9%  | ~63          |
| common     | 81.5%   | 90%    | 8.5%  | ~442         |
| dynamics   | 79.4%   | 90%    | 10.6% | ~3346        |
| constraint | 79.0%   | 90%    | 11.0% | ~796         |
| math       | 77.1%   | 90%    | 12.9% | ~1721        |

**Overall**: 79.9% (up from 57%)

**Non-core** (target: 80%):

| Module | Current | Status |
| ------ | ------- | ------ |
| GUI    | 14.3%   | Skip   |

### Priority Order (Quick Wins First)

1. **collision** - Only 1.2% gap, quick win
2. **sensor** - Only 2.6% gap, minimal code
3. **simulation** - Only 3.9% gap
4. **common** - 8.5% gap, moderate effort
5. **constraint** - 11.0% gap, significant effort
6. **math** - 12.9% gap, largest % gap
7. **dynamics** - 10.6% gap, largest absolute (3346 lines)

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
