# Coverage Gap Analysis

## Module Coverage Summary

| Module     | Source Files | Unit Tests | Integration | Test Ratio | Priority |
| ---------- | ------------ | ---------- | ----------- | ---------- | -------- |
| dynamics   | 179          | 34         | 19          | 0.30       | High     |
| common     | 105          | 16         | 0           | 0.15       | High     |
| collision  | 92           | 14         | 14          | 0.30       | High     |
| math       | 86           | 19         | 0           | 0.22       | Medium   |
| utils      | 81           | 3          | 9           | 0.15       | Medium   |
| gui        | 77           | 6          | 0           | 0.08       | Excluded |
| simulation | 72           | 13         | 9           | 0.31       | High     |
| constraint | 44           | 9          | 6           | 0.34       | High     |
| optimizer  | 8            | 0          | 2           | 0.25       | Low      |
| lcpsolver  | 6            | 0\*        | 0\*         | N/A        | Low      |
| sensor     | 5            | 2          | 0           | 0.40       | Low      |
| io         | 3            | 3          | 9           | 4.00       | Good     |

\*lcpsolver has dedicated tests in tests/common/lcpsolver/

## High Priority Gaps

### 1. dart/common/ — Core Utilities (37 untested headers)

**Untested classes** (no dedicated test file):

- `Aspect`, `AspectWithVersion`, `EmbeddedAspect`, `ProxyAspect`, `RequiresAspect`, `SpecializedForAspect`
- `Castable`, `ClassWithVirtualBase`, `Cloneable`, `CompositeJoiner`
- `Diagnostics`, `LocalResourceRetriever`, `ResourceRetriever`
- `LockableReference`, `MemoryAllocator`, `MemoryAllocatorDebugger`
- ~~`Observer`, `Subject`, `Signal`~~ ✅ DONE
- ~~`SharedLibrary`, `Singleton`, `VersionCounter`~~ ✅ Singleton/VersionCounter DONE
- `Metaprogramming`, `StlHelpers`, `Virtual`, ~~`sub_ptr`~~ ✅ DONE

**Recently tested**: ~~`NameManager`~~ ✅, ~~`Filesystem`~~ (just a namespace alias)

**Quick wins** (simple utilities):

1. ~~`NameManager` - Name generation/uniqueness~~ ✅ DONE (20 tests)
2. `Singleton` - Template singleton pattern
3. `VersionCounter` - Simple counter class
4. `Filesystem` - Just a namespace alias, skip

**Complex but important**:

1. `Aspect` system - Core architecture, needs comprehensive tests
2. ~~`Signal/Observer/Subject` - Event system~~ ✅ DONE (Signal: 18 tests, SubjectObserver: 18 tests)

### 2. dart/dynamics/ — Largest Module

**Well-tested** (have dedicated tests):

- Skeleton, BodyNode, Joint types (FreeJoint, RevoluteJoint, etc.)
- DegreeOfFreedom, Inertia, Frame

**Under-tested** (mentioned but not thoroughly):

- Joint command validation
- Error handling in setters
- Edge cases in kinematics calculations

**Likely untested**:

- Many detail/ implementation classes
- Error recovery paths
- Hierarchical update propagation

### 3. dart/constraint/ — Solver Components

**Tested**:

- Basic constraint types (BallJointConstraint, WeldJointConstraint)
- ConstraintSolver basics

**Gaps**:

- BoxedLcpConstraintSolver edge cases
- Constraint failure modes
- Multiple simultaneous constraints
- Warm-starting behavior

### 4. dart/collision/ — Multiple Backends

**Backend coverage**:
| Backend | Test Files | Status |
|---------|-----------|--------|
| FCL | 5+ | Good |
| Bullet | 3+ | Medium |
| ODE | 2+ | Medium |
| DART native | 2+ | Medium |

**Gaps**:

- Cross-backend consistency tests
- Edge cases: degenerate shapes, near-contact
- Performance regression tests
- CollisionFilter edge cases

## Medium Priority Gaps

### 5. dart/utils/ — Parser Support

**Well-tested**:

- URDF parser (comprehensive integration tests)
- SDF parser (unit + integration)
- MJCF parser (integration tests)
- Resource retrievers

**Gaps**:

- Parser error handling (malformed input)
- Partial/incomplete model handling
- Schema validation errors

### 6. dart/math/ — Numerical Utilities

**Well-tested**:

- Basic geometry (SO3, SE3)
- Helpers and constants

**Gaps**:

- Numerical edge cases (near-singular matrices)
- Random generation functions
- LCP solver numerical stability

## Low Priority (Deferred)

### dart/gui/ — Excluded from Coverage

Requires display; test via headless rendering if feasible.

### dart/optimizer/ — Small Module

Only 8 headers, 2 integration tests exist. Low ROI.

### dart/sensor/ — Small Module

5 headers, 2 unit tests. Adequate for module size.

## Quick Win Test Candidates

Tests that would add coverage with minimal effort:

| Class                       | Module     | Effort | Impact | Status          |
| --------------------------- | ---------- | ------ | ------ | --------------- |
| `NameManager`               | common     | Low    | Low    | ✅ DONE         |
| `Singleton`                 | common     | Low    | Low    | ✅ DONE         |
| `Signal/Observer`           | common     | Medium | Medium | ✅ DONE         |
| `VersionCounter`            | common     | Low    | Low    | ✅ DONE         |
| `sub_ptr`                   | common     | Low    | Low    | ✅ DONE         |
| `CollisionFilter`           | collision  | Medium | High   | Expand existing |
| `ConstraintBase` edge cases | constraint | Medium | High   | Error paths     |
| Parser error handling       | utils      | Medium | Medium | Malformed input |

## Recommended Test Implementation Order

### Phase 1: Infrastructure & Quick Wins (1-2 days)

1. ~~Fix coverage reporting~~ ✅ DONE
2. ~~Add `test_NameManager.cpp`~~ ✅ DONE (20 tests)
3. ~~Add `test_Singleton.cpp`~~ ✅ DONE (8 tests)
4. ~~Add `test_Signal.cpp`~~ ✅ DONE (18 tests)
5. ~~Add `test_VersionCounter.cpp`~~ ✅ DONE (6 tests)
6. ~~Expand `test_SubjectObserver.cpp`~~ ✅ DONE (15 new tests, total 18)
7. Expand CollisionFilter tests

### Phase 2: Core Systems (1 week)

1. Aspect system tests (`test_Aspect.cpp`)
2. Constraint solver edge cases
3. Dynamics error handling paths

### Phase 3: Comprehensive Coverage (2+ weeks)

1. Cross-backend collision consistency
2. Parser error handling
3. Numerical edge cases in math/

## Test ROI Guidelines

**High ROI** (prioritize):

- Error handling paths (bug-prone)
- Public API edge cases
- Cross-module interactions

**Medium ROI**:

- Internal utilities
- Configuration variations

**Low ROI** (avoid):

- Trivial getters/setters
- Private implementation details
- Platform-specific code paths
