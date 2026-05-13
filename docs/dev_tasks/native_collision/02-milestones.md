# Native Collision Milestones

## Phase 1: Native Default Path

Success criteria:

- `ConstraintSolver` default detector selection prefers `dart`.
- `experimental` remains only as a compatibility alias.
- Native contact cache and warm-start paths use the native detector after the
  rename.
- Public docs describe `dart/collision/native/` and `dart` as the default.

Verification:

- Focused unit tests for detector factory/default creation.
- Focused constraint/contact tests that exercise cached contact impulse write
  back.
- `pixi run lint`.

## Phase 2: Feature Parity Gate

Success criteria:

- Collision, distance, and raycast tests cover the shapes and filters DART uses
  from FCL, Bullet, and ODE.
- Unsupported shapes have an explicit implementation or a documented DART-level
  non-requirement.
- Native-vs-reference consistency tests cover contact sign, normal direction,
  depth, distance, nearest point, and raycast semantics.

Verification:

- Native collision unit tests.
- Collision integration tests.
- Backend consistency tests with reference backends enabled.

## Phase 3: gz-physics Gate

Success criteria:

- gz-physics builds against this branch without patches.
- gz-physics tests pass with the native detector as DART's default.
- Legacy class-name and factory-key compatibility is verified.

Verification:

- `pixi run -e gazebo test-gz`.
- Any failing gz-physics case is reduced to a DART test before being fixed.

## Phase 4: Performance Gate

Success criteria:

- Comparative benchmark results are recorded in `03-evidence-gates.md`.
- Native beats the best legacy backend in required narrowphase, broadphase,
  distance, raycast, mixed primitive, and mesh-heavy scenarios.
- No benchmark result is accepted without correctness checks for the same case.

Verification:

- Comparative benchmark suite with FCL, Bullet, and ODE enabled.
- Native-only microbenchmarks for regression tracking.
- Benchmark command lines, commit SHAs, and summary tables recorded.

## Phase 5: Dependency Removal Gate

Success criteria:

- Default DART configure/build no longer requires FCL, Bullet, or ODE for
  collision detection.
- Optional reference backend builds remain available only where needed for
  validation.
- Packaging metadata and CMake component dependencies no longer advertise old
  collision libraries as runtime requirements.

Verification:

- Configure/build with FCL, Bullet, and ODE disabled or absent.
- Install-tree inspection for unwanted runtime links.
- Public headers still satisfy gz-physics compatibility requirements.

## Phase 6: Cleanup And PR Readiness

Success criteria:

- Stale `experimental` wording is gone except for compatibility aliases and
  simulation-experimental code.
- Dev-task learnings are condensed into onboarding docs.
- This folder is removed in the completion PR.

Verification:

- `pixi run lint`.
- `pixi run test-all` when feasible.
- `pixi run -e gazebo test-gz`.
- PR description includes evidence from `03-evidence-gates.md`.
