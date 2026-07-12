---
name: dart-test
description: "DART Test: unit tests, integration tests, CI validation, and debugging"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-test/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART Testing

Load this skill when writing or debugging tests.

## Quick Commands

```bash
pixi run test         # Quick test run
pixi run test-py      # Python tests
pixi run test-all     # Build all CMake targets
pixi run -e gazebo test-gz # Gazebo/gz-physics compatibility gate
```

## Full Documentation

For complete testing guide: `docs/onboarding/testing.md`

For CI/CD troubleshooting: `docs/onboarding/ci-cd.md`

## Test Organization

- Unit tests: `tests/unit/`
- Integration tests: `tests/integration/`
- Regression tests: Near the code they test

## Writing Tests

1. Follow existing patterns in the test directory
2. Use GoogleTest framework
3. Name tests descriptively: `TEST(ClassName, MethodName_Condition_ExpectedResult)`

## CI Validation

Before submitting PR:

```bash
pixi run lint         # Must pass
pixi run test-all     # Must pass when all CMake targets should build
pixi run -e gazebo test-gz # Must pass when Gazebo/gz-physics compatibility could be affected
```

## Debugging Test Failures

```bash
# Run the smallest existing Pixi test task that covers the failure
pixi run test

# Get CI logs
gh run view <RUN_ID> --log-failed
```

## Simulation And Visual End-to-End Checks

When a failure or change depends on scene structure, dynamics,
collision/contact, simulation stepping, OSG output, or a visual example, also
load `dart-verify-sim`. Use its text-first oracle and then an assessed,
claim-tied capture through `DebugOverlay`. Record a concrete no-visual reason
only when capture is unavailable or not applicable.

## Gotchas

- `pixi run build` builds libraries only, NOT every test binary. If you run
  `ctest` directly, build the relevant test target first; `pixi run build-tests`
  builds the release-branch test targets.
- `pixi run test-all` builds all CMake targets on this branch. Pair it with
  `pixi run lint`, `pixi run test`, and `pixi run test-py` when those gates are
  required for the touched surface.
- For package, collision, constraint, or dependency changes that could affect
  downstream users, run `pixi run -e gazebo test-gz` when practical.
