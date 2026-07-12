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
pixi run test-all     # Full validation
pixi run -e cuda test-all # CUDA-enabled full validation on Linux CUDA hosts
pixi run test-unit    # Unit tests
pixi run test-py      # Python tests
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
pixi run test-all     # Must pass
pixi run -e cuda test-all # Must pass on Linux CUDA hosts
```

## Debugging Test Failures

```bash
# Run the smallest existing Pixi test task that covers the failure
pixi run test-unit

# Get CI logs
gh run view <RUN_ID> --log-failed
```

## Simulation And Visual End-to-End Checks

When a failure or change depends on scene structure, dynamics,
collision/contact, simulation stepping, GUI output, or a visual example, also
load `dart-verify-sim`. Use its text-first oracle and then an assessed headless
capture with claim-relevant debug layers. Record a concrete no-visual reason
only when that corroboration is unavailable or not applicable.

## Gotchas

- `pixi run build` builds libraries only, NOT the unit-test binaries. If you run
  `ctest` without building the test target first, you may execute stale binaries
  that silently pass. Build the test target before running its label, e.g. for
  the simulation suite: `pixi run build-simulation-tests` (target
  `dart_simulation_tests`) before `ctest -L simulation`.
- `pixi run test-all` is the default-environment full gate. On Linux hosts with
  a visible NVIDIA CUDA runtime, also run `pixi run -e cuda test-all`; the CUDA
  run preserves the `cuda` Pixi environment and executes the CUDA CTest +
  benchmark smoke path when the runtime is detected.
- The CUDA Pixi config auto-detects visible GPU compute capabilities for
  `DART_CUDA_ARCHITECTURES`; unsupported PTX/toolchain errors usually mean the
  generated CUDA architecture flags need to be checked before blaming test code.
