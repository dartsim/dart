# tests/

Agent guidelines for the test suite.

## Test Organization

| Directory      | Type              | Purpose                    |
| -------------- | ----------------- | -------------------------- |
| `unit/`        | Unit tests        | Isolated component testing |
| `integration/` | Integration tests | Cross-module testing       |
| `benchmark/`   | Benchmarks        | Performance measurement    |

## Running Tests

```bash
pixi run test-all      # Full suite (lint + build + tests)
pixi run test-unit     # Unit tests only (faster)
pixi run test-py       # Python tests only
```

## Test Naming Conventions

- Unit tests: `test_<ClassName>.cpp`
- Integration tests: `test_<Feature>.cpp`
- Python tests: `test_<module>.py`

## Writing Tests

- Use Google Test (`gtest`) for C++ tests
- Use pytest for Python tests
- Place fixtures in `tests/fixtures/` or use `data/` samples
- Prefer small, focused tests over large integration tests

## Key Fixtures

- `data/urdf/` - Sample URDF models
- `data/sdf/` - Sample SDF models
- `data/mjcf/` - Sample MJCF models
- `data/skel/` - Sample SKEL models

## See Also

- @docs/onboarding/testing.md - Detailed testing documentation
- @.claude/skills/dart-test/SKILL.md - Test-related skill
