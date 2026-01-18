# Test Coverage Milestones

## Phase 1 — Quick Wins to 72% (1-2 days)

**Goal**: Eliminate zero-coverage modules, establish patterns

### Deliverables

| Task                                          | Priority | Effort | Target Coverage |
| --------------------------------------------- | -------- | ------ | --------------- |
| `tests/unit/sensor/test_Sensor.cpp`           | P1       | 2h     | 70% of sensor/  |
| `tests/unit/sensor/test_SensorManager.cpp`    | P1       | 2h     | 90% of sensor/  |
| `tests/unit/dynamics/test_CapsuleShape.cpp`   | P1       | 1h     | +0.5%           |
| `tests/unit/dynamics/test_ConeShape.cpp`      | P1       | 1h     | +0.5%           |
| `tests/unit/dynamics/test_CylinderShape.cpp`  | P1       | 1h     | +0.5%           |
| `tests/unit/dynamics/test_EllipsoidShape.cpp` | P1       | 1h     | +0.5%           |
| `tests/unit/dynamics/test_PlaneShape.cpp`     | P1       | 1h     | +0.5%           |
| Update `codecov.yml`                          | P1       | 1h     | Config only     |

### Exit Criteria

- [ ] Sensor module has >70% coverage
- [ ] All basic shapes have dedicated validation tests
- [ ] Codecov config updated with targets and exclusions
- [ ] Overall coverage >= 72%

---

## Phase 2 — Systematic to 80% (1-2 weeks)

**Goal**: Error paths, Python baseline, maintainability refactors

### Deliverables

| Task                                                 | Priority | Effort | Impact                |
| ---------------------------------------------------- | -------- | ------ | --------------------- |
| `tests/unit/dynamics/test_EndEffector.cpp`           | P2       | 2h     | Core component        |
| `tests/unit/dynamics/test_Marker.cpp`                | P2       | 1h     | Core component        |
| `tests/unit/dynamics/test_PointMass.cpp`             | P2       | 2h     | Core component        |
| `tests/unit/io/test_UrdfParser_Errors.cpp`           | P2       | 3h     | Error paths           |
| `tests/unit/io/test_SdfParser_Errors.cpp`            | P2       | 3h     | Error paths           |
| Expand `python/tests/unit/dynamics/test_skeleton.py` | P2       | 3h     | Python baseline       |
| Create `python/tests/fixtures.py`                    | P2       | 4h     | Python infrastructure |
| Add Python shape tests                               | P2       | 2h     | Python coverage       |
| Split monolithic restructuring test                  | P3       | 4h     | Maintainability       |
| Investigate/re-enable Jacobian tests                 | P3       | 4h     | Restore validation    |

### Exit Criteria

- [ ] Overall coverage >= 80%
- [ ] `math/` coverage >= 88%
- [ ] `dynamics/` coverage >= 82%
- [ ] Python test infrastructure established
- [ ] No monolithic tests >100 lines

---

## Phase 3 — Push to 85%+ (2-4 weeks)

**Goal**: Edge cases, parameterization, regression prevention

### Deliverables

| Task                                             | Priority | Effort | Impact                 |
| ------------------------------------------------ | -------- | ------ | ---------------------- |
| Enable patch coverage requirement                | Config   | 1h     | Prevent regression     |
| Parameterized collision tests (all shape combos) | P3       | 8h     | +3%                    |
| Constraint solver edge cases                     | P3       | 6h     | +2%                    |
| Systematic error path audit                      | P3       | 8h     | +2%                    |
| Platform-specific code tests                     | P4       | 4h     | +1%                    |
| Document patterns in `testing.md`                | Docs     | 2h     | Contributor enablement |

### Exit Criteria

- [ ] Overall coverage >= 85%
- [ ] Core modules (`math/`, `dynamics/`, `sensor/`) >= 90%
- [ ] Patch coverage requirement enabled
- [ ] Test patterns documented

---

## Phase 4 — Stretch Goals (Ongoing)

**Goal**: Maximize coverage where feasible

| Task                        | Trigger                 | Notes                |
| --------------------------- | ----------------------- | -------------------- |
| GUI headless testing        | CI infrastructure ready | See `03-backlog.md`  |
| Python full API parity      | After Phase 3           | See `03-backlog.md`  |
| Experimental module testing | APIs stabilize          | See `03-backlog.md`  |
| 90% overall coverage        | After all above         | Reassess feasibility |

---

## Success Metrics

| Metric      | Current | Phase 1 | Phase 2 | Phase 3 | Stretch |
| ----------- | ------- | ------- | ------- | ------- | ------- |
| Overall     | ~67%    | 72%     | 80%     | 85%     | 90%     |
| `math/`     | High    | 85%     | 88%     | 90%+    | 95%     |
| `dynamics/` | Medium  | 75%     | 82%     | 90%     | 92%     |
| `sensor/`   | 0%      | 70%     | 85%     | 90%     | 95%     |
| Python      | Low     | 50%     | 60%     | 70%     | 80%     |

## Quality Constraints

| Metric                   | Constraint                          |
| ------------------------ | ----------------------------------- |
| Flakiness                | 0 flaky tests in CI                 |
| Unit test runtime        | < 60s total                         |
| Integration test runtime | < 5 min total                       |
| Redundancy               | Use parameterization, no duplicates |
| Assertions               | Meaningful (no empty/trivial tests) |
