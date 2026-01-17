# Error Handling Migration Tracker

**Last Updated**: 2026-01-17

This document tracks the progress of migrating DART's error handling to the modern system.

## Overview

| Phase                  | Status      | Progress |
| ---------------------- | ----------- | -------- |
| Phase 1: Foundation    | Complete    | 100%     |
| Phase 2: API Boundary  | Not Started | 0%       |
| Phase 3: Core Modules  | Not Started | 0%       |
| Phase 4: I/O & Parsers | Not Started | 0%       |
| Phase 5: Documentation | Not Started | 0%       |

## Phase 1: Foundation

### Deliverables

| Item                                | Status | Notes                                    |
| ----------------------------------- | ------ | ---------------------------------------- |
| `dart/common/Exception.hpp`         | Done   | Exception hierarchy                      |
| `dart/common/Exception.cpp`         | Done   | Error handler impl                       |
| `dart/common/Result.hpp`            | Done   | Result<T,E> type                         |
| `docs/onboarding/error-handling.md` | Done   | User guidelines                          |
| Unit tests                          | TODO   | Need test_Exception.cpp, test_Result.cpp |
| CMakeLists.txt update               | TODO   | Add new files to build                   |

## Phase 2: API Boundary Hardening

### Console Output Removal

| File                                        | std::cout/cerr Count | Status                 |
| ------------------------------------------- | -------------------- | ---------------------- |
| `dart/constraint/ConstraintSolver.cpp`      | 35                   | Pending                |
| `dart/constraint/SoftContactConstraint.cpp` | 21                   | Pending (commented)    |
| `dart/dynamics/EulerJoint.cpp`              | 6                    | Pending                |
| `dart/utils/urdf/urdf_world_parser.cpp`     | 5                    | Pending                |
| `dart/constraint/JointLimitConstraint.cpp`  | 5                    | Pending                |
| `dart/common/Stopwatch.cpp`                 | 4                    | Keep (print functions) |
| `dart/dynamics/SoftBodyNode.cpp`            | 2                    | Pending                |
| `dart/common/VersionCounter.cpp`            | 2                    | Pending                |

### [[nodiscard]] Additions

| Module                  | Function Count | Status  |
| ----------------------- | -------------- | ------- |
| `dynamics/Skeleton.hpp` | ~15            | Pending |
| `dynamics/BodyNode.hpp` | ~10            | Pending |
| `simulation/World.hpp`  | ~5             | Pending |
| Parser entry points     | ~10            | Pending |

### Exception Additions

| API                                    | Change                     | Status  |
| -------------------------------------- | -------------------------- | ------- |
| `World::addSkeleton(nullptr)`          | Throw NullPointerException | Pending |
| `Skeleton::getBodyNode(invalid_index)` | Throw OutOfRangeException  | Pending |
| `Skeleton::getJoint(invalid_index)`    | Throw OutOfRangeException  | Pending |
| `Skeleton::getDof(invalid_index)`      | Throw OutOfRangeException  | Pending |

## Phase 3: Core Module Migration

### dart/dynamics/

| File                    | DART_ERROR/WARN Count | Migrated |
| ----------------------- | --------------------- | -------- |
| `Skeleton.cpp`          | 67                    | No       |
| `BodyNode.cpp`          | 14                    | No       |
| `Joint.cpp`             | 18                    | No       |
| `MetaSkeleton.cpp`      | 15                    | No       |
| `InverseKinematics.cpp` | 11                    | No       |
| `Group.cpp`             | 27                    | No       |
| Other files             | ~50                   | No       |

### dart/collision/

| File                                 | DART_ERROR/WARN Count | Migrated |
| ------------------------------------ | --------------------- | -------- |
| `bullet/BulletCollisionDetector.cpp` | 12                    | No       |
| `fcl/FCLCollisionDetector.cpp`       | 11                    | No       |
| `ode/OdeCollisionDetector.cpp`       | 4                     | No       |
| `dart/DARTCollisionDetector.cpp`     | 8                     | No       |
| Other files                          | ~10                   | No       |

### dart/constraint/

| File                        | DART_ERROR/WARN Count | Migrated |
| --------------------------- | --------------------- | -------- |
| `ConstraintSolver.cpp`      | 17                    | No       |
| `ContactConstraint.cpp`     | 6                     | No       |
| `JointLimitConstraint.cpp`  | 6                     | No       |
| `SoftContactConstraint.cpp` | 10                    | No       |
| Other files                 | ~20                   | No       |

## Phase 4: I/O & Parser Migration

### Parser Status

| Parser | Error Handling              | Target              |
| ------ | --------------------------- | ------------------- |
| URDF   | DART_WARN + return nullptr  | Result<SkeletonPtr> |
| SDF    | DART_WARN + return nullptr  | Result<SkeletonPtr> |
| SKEL   | DART_ERROR + return nullptr | Result<SkeletonPtr> |
| MJCF   | Errors vector               | Keep (good pattern) |

### Migration Steps for Each Parser

1. Create `Result<SkeletonPtr>` return type variant
2. Maintain backward-compatible `SkeletonPtr` overload
3. Accumulate errors in Error vector
4. Document new API in header
5. Update examples

## Phase 5: Documentation & Testing

### Documentation

| Document                      | Status  |
| ----------------------------- | ------- |
| API contracts for Skeleton    | Pending |
| API contracts for World       | Pending |
| API contracts for parsers     | Pending |
| Tutorial updates              | Pending |
| Doxygen `@throws` annotations | Pending |

### Testing

| Test Category                        | Status  |
| ------------------------------------ | ------- |
| Exception unit tests                 | TODO    |
| Result unit tests                    | TODO    |
| Integration tests for new exceptions | Pending |
| Exception-free mode tests            | Pending |

## Metrics

### Current State (as of analysis)

| Metric                  | Count |
| ----------------------- | ----- |
| DART_WARN usages        | ~350  |
| DART_ERROR usages       | ~320  |
| DART_FATAL usages       | ~2    |
| return nullptr patterns | ~300  |
| return false patterns   | ~180  |
| Raw std::cout/cerr      | ~91   |

### Target State

| Metric                               | Target                     |
| ------------------------------------ | -------------------------- |
| Undocumented error contracts         | 0                          |
| Raw std::cout/cerr in library        | 0 (except print functions) |
| Functions with missing [[nodiscard]] | < 10                       |
| Untyped exception throws             | 0                          |

## Notes

- Migration should be done module-by-module
- Each module migration should be a separate PR
- Backward compatibility is critical - don't break existing user code
- Use deprecation warnings before removing old APIs
- All changes need corresponding test updates
