# Resume: Simulation Experimental API Epic

## Last Session Summary

**Phase 0 and Phase 1 COMPLETE!**

1. **Joint limits accessors** committed
2. **Hello world example** created (`examples/simulation_experimental_hello_world/`)
3. **Naming audit** complete - all APIs follow code-style.md conventions
4. **Exception hierarchy** reviewed - `InvalidOperationException` for design mode violations
5. **Lifetime docs** added to all handle classes (Frame, FreeFrame, FixedFrame, RigidBody)
6. **Design mode guards** verified - World and MultiBody::addLink check simulation mode

## Current Branch

`feature/sim_exp` - 10 commits ahead of main:

```
0c57ff9d0b4 feat(simulation-experimental): Complete Phase 0 and Phase 1
43b548bff91 docs(simulation-experimental): Update progress tracker and RESUME.md
c1c0df2d2a9 feat(simulation-experimental): Add Joint limits accessors
ec286d3c45f feat(simulation-experimental): Implement RigidBody physics properties
3516429230a docs(simulation-experimental): Update RESUME.md with session progress
f71761582c9 feat(simulation-experimental): Add Joint state accessors
bf298c8e657 test(simulation-experimental): Add comprehensive tests for all joint types
18c2b37807a feat(simulation-experimental): Add epic plan and fill empty test files
```

Working tree is clean. All 13 simulation-experimental tests pass.

## What's Complete

### Phase 0 (Ground-Truth) ✅ COMPLETE
- [x] Epic document created
- [x] test_joint.cpp: 41 tests
- [x] test_link.cpp: 14 tests
- [x] test_rigid_body.cpp: 14 tests
- [x] All joint types tested
- [x] Hello world example created

### Phase 1 (API Cleanup) ✅ COMPLETE
- [x] All joint types data model complete
- [x] Joint state accessors (position/velocity/acceleration/torque)
- [x] Joint limits accessors (position lower/upper, velocity, effort)
- [x] RigidBody full implementation (mass, inertia, pose, velocity, forces)
- [x] Naming audit (camelCase methods, PascalCase classes, snake_case files)
- [x] Exception hierarchy review (InvalidOperationException for mode violations)
- [x] Lifetime docs in Doxygen (all handle classes)
- [x] Design mode guards (World and MultiBody check simulation mode)

## Next Phase: Python Bindings (Phase 2)

Ready to start Phase 2 if desired:
- Create `python/dartpy/simulation/experimental/` directory
- Create `module.cpp` for submodule registration
- Bind World, MultiBody, Link, Joint, RigidBody classes
- Add Python docstrings and examples
- Enable `DART_EXPERIMENTAL_BUILD_PYTHON` in CI

## Test Summary

| Test File             | # Tests | Coverage                                       |
| --------------------- | ------- | ---------------------------------------------- |
| `test_joint.cpp`      | 41      | All types, DOF, state accessors, limits        |
| `test_link.cpp`       | 14      | name, parent joint, frame, copy, chains        |
| `test_rigid_body.cpp` | 14      | mass, inertia, pose, velocity, forces          |

Total: 69 tests for handles + existing world/frame/serialization tests = 13 test binaries all passing

## How to Resume

```bash
git checkout feature/sim_exp
git status && git log -3 --oneline
pixi run build-simulation-experimental-tests
ctest -L simulation-experimental --test-dir build/default/cpp/Release
./build/default/cpp/Release/bin/simulation_experimental_hello_world  # Run example
```

Ready for: Push to create PR, or proceed to Phase 2 (Python bindings)
