# Issue 244: Sensor scaffolding progress

## Repro (origin/main)

- `git ls-tree -r --name-only origin/main | rg '^dart/sensor/'` returns nothing (no sensor module).
- `git grep -n "dart::sensor" origin/main -- dart` returns nothing; only MJCF user-sensor fields appear.

## Phase status

- [x] Phase 0: Intake + repro
- [x] Phase 1: Core sensor API
- [x] Phase 2: Tests + GUI example
- [ ] Phase 3: Validation + delivery
