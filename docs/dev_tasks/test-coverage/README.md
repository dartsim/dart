# Test Suite Audit & Coverage Strategy — Dev Task

## Current Status

- [ ] **Phase 1**: Quick wins to 72% (sensor tests, shape tests, codecov config)
- [ ] **Phase 2**: Systematic to 80% (error paths, Python expansion, refactors)
- [ ] **Phase 3**: Push to 85%+ (parameterized tests, edge cases)
- [ ] **Phase 4**: Stretch goals (90% for core modules)

**Current coverage**: ~67% (Codecov report)
**Target coverage**: 85% overall, 90% for core modules (`math/`, `dynamics/`, `sensor/`)

## Goal

Improve DART test coverage from ~67% to 85%+ with meaningful, maintainable tests. Focus on:

- Eliminating zero-coverage modules (especially new code like `sensor/`)
- Establishing consistent test patterns across all modules
- Preventing regressions via patch coverage requirements

## Non-Goals (for early phases)

- GUI headless testing (blocked on CI infrastructure - see `03-backlog.md`)
- Full Python API parity (focus on most-used APIs first - see `03-backlog.md`)
- Testing `dart/simulation/experimental/` (heavy development - see `03-backlog.md`)

## Key Decisions

- **Tiered coverage targets**: 90% for core (`math/`, `dynamics/`, `sensor/`), 80% for IO/collision
- **Sensor module priority elevated**: New code (not legacy) deserves high coverage from day 1
- **Python scope**: Most-used APIs first, full parity deferred to backlog
- **Pattern**: Use `test_SphereShape.cpp` as template for all shape tests

## Documents

- Coverage analysis and findings: `01-coverage-analysis.md`
- Roadmap and phase criteria: `02-milestones.md`
- Future work backlog: `03-backlog.md`

## Immediate Next Steps

1. Create sensor module tests (`test_Sensor.cpp`, `test_SensorManager.cpp`)
2. Add shape validation tests (5 files using `test_SphereShape.cpp` pattern)
3. Update `codecov.yml` with recommended exclusions and targets
