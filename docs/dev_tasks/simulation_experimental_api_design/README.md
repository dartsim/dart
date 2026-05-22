# Simulation Experimental API Design — Dev Task

## Current Status

- [x] Established DART 7 experimental / DART 8 official API direction in the
      C++ and Python design docs.
- [x] Added first-class loop-closure topology, runtime policy, and residual
      diagnostics in C++ and dartpy.
- [x] Added `World::sync(WorldSyncStage::Kinematics)` and
      `world.sync(sx.WorldSyncStage.KINEMATICS)` for explicit kinematics-only work
      placement.
- [x] Made standard tree-joint positions drive experimental open-chain forward
      kinematics and loop-closure residuals.
- [x] Tightened `dartpy.simulation_experimental` so data-like state uses
      Python properties instead of parallel getter/setter aliases.
- [x] Bound `StateSpace` into `dartpy.simulation_experimental` as a
      storage-independent metadata value object.
- [x] Improved dartpy loop-closure ergonomics with optional auto-naming and
      direct runtime participation properties.
- [ ] Continue closing API gaps from
      `docs/design/simulation_experimental_cpp_api.md` and
      `docs/design/simulation_experimental_python_api.md`.

## Goal

Move the current experimental C++ and Python simulation APIs toward the design
docs: a small public facade, Pythonic dartpy surface, reusable world for
physics and kinematics-only workflows, first-class closed-chain structure, and
stable public concepts whose internals can change before DART 8 promotion.

## Non-Goals For The Current Slice

- Do not modify `/home/js/multiphysics-api-design.md`.
- Do not expose ECS storage, `comps`, EnTT handles, backend task systems, GPU
  resources, solver registries, or implementation details through dartpy.
- Do not add public async stepping until there is a real job/future contract.
- Do not implement loop-closure projection or dynamic solving before compatible
  stages exist.

## Key Decisions

- DART 7/dartpy 7 use the experimental namespace/module as the staging surface;
  DART 8/dartpy 8 promote the mature shape and remove the legacy DART 6 APIs.
- Python experimental APIs must use Python spelling and style only. Class names
  are `PascalCase`, but methods/properties/keywords/enum values use Pythonic
  names and no C++ camelCase aliases.
- Data-like Python state is exposed as properties. Methods remain for
  operations, topology mutation, stepping, synchronization, lookup, and force
  application.
- Solver, executor, backend, and policy names should use algorithm,
  approach, paper, or DART-owned names rather than external engine names.
- The same `World` is the public object model for full physics and
  kinematics-only workflows; pipeline intent, not a separate scene API,
  distinguishes runtime work.
- The freshness principle is implicit fresh reads plus explicit work placement;
  public APIs must not expose dirty flags or cache bits.

## Immediate Next Steps

1. Keep the branch merged with `origin/main`, then push the committed
   `StateSpace` binding.
2. Audit the remaining gap between the design docs and the public C++/dartpy
   staging surface.
3. Prefer the next narrow API slice that improves the public facade without
   exposing ECS storage, component mappers, backend handles, or unsupported
   async/solver contracts.
4. Before each commit, run `pixi run lint`; run focused build/tests for the
   touched API and `pixi run test-py` for dartpy changes.
