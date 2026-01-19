# Resume: Simulation Experimental API Epic

## Last Session Summary

Phase 1 nearly complete - all major implementations done:

1. **Joint state accessors** - getDOF, get/set for position, velocity, acceleration, torque
2. **Joint limits accessors** - get/set for position lower/upper, velocity, effort limits
3. **RigidBody full implementation** - mass, inertia, pose, velocity, force/torque accumulation
4. **Comprehensive tests** - 41 tests in test_joint.cpp, 14 in test_link.cpp, 14 in test_rigid_body.cpp

## Current Branch

`feature/sim_exp` - 7 commits ahead of main:

```
c1c0df2d2a9 feat(simulation-experimental): Add Joint limits accessors
ec286d3c45f feat(simulation-experimental): Implement RigidBody physics properties
3516429230a docs(simulation-experimental): Update RESUME.md with session progress
f71761582c9 feat(simulation-experimental): Add Joint state accessors
bf298c8e657 test(simulation-experimental): Add comprehensive tests for all joint types
18c2b37807a feat(simulation-experimental): Add epic plan and fill empty test files
```

Working tree is clean.

## Immediate Next Steps

**Phase 1 remaining tasks** (cleanup, not implementation):

1. **Naming audit** - Check all public APIs for snake_case consistency per code-style.md
2. **Exception hierarchy review** - Verify consistent use of InvalidArgumentException, InvalidOperationException, etc.
3. **Lifetime docs** - Add Doxygen comments explaining handle lifetime rules
4. **Design mode guards audit** - Ensure all mutating methods call ensureDesignMode()

**Or proceed to Phase 2**: Python bindings (if Phase 1 is sufficient)

## What's Complete

### Phase 0 (Ground-Truth)

- [x] Epic document created
- [x] test_joint.cpp: 41 tests
- [x] test_link.cpp: 14 tests
- [x] test_rigid_body.cpp: 14 tests
- [x] All joint types tested
- [ ] Hello world example (deferred - low priority)

### Phase 1 (API Cleanup)

- [x] All joint types data model complete
- [x] Joint state accessors (position/velocity/acceleration/torque)
- [x] Joint limits accessors (position lower/upper, velocity, effort)
- [x] RigidBody full implementation (mass, inertia, pose, velocity, forces)
- [ ] Naming audit (snake_case consistency)
- [ ] Exception hierarchy review
- [ ] Lifetime docs in Doxygen
- [ ] Design mode guards audit

## Test Count Summary

| Test File             | # Tests | Coverage                                |
| --------------------- | ------- | --------------------------------------- |
| `test_joint.cpp`      | 41      | All types, DOF, state accessors, limits |
| `test_link.cpp`       | 14      | name, parent joint, frame, copy, chains |
| `test_rigid_body.cpp` | 14      | mass, inertia, pose, velocity, forces   |

Total: 69 tests for handles + existing world/frame/serialization tests

## Key Files Reference

| File                                                    | Purpose                                    |
| ------------------------------------------------------- | ------------------------------------------ |
| `dart/simulation/experimental/world.hpp`                | Main entry point                           |
| `dart/simulation/experimental/multi_body/joint.hpp/cpp` | Joint handle (state + limits)              |
| `dart/simulation/experimental/body/rigid_body.hpp/cpp`  | RigidBody handle (physics)                 |
| `dart/simulation/experimental/comps/joint.hpp`          | JointType enum, Joint component            |
| `dart/simulation/experimental/comps/dynamics.hpp`       | Transform, Velocity, MassProperties, Force |

## How to Resume

```bash
git checkout feature/sim_exp
git status && git log -3 --oneline  # Verify clean, check commits
pixi run build-simulation-experimental-tests
ctest -L simulation-experimental --test-dir build/default/cpp/Release  # Should pass 13/13
```

Then pick from remaining Phase 1 tasks above, or ask about proceeding to Phase 2.
